from collections import deque
from dataclasses import dataclass
from math import isqrt
from typing import Any, Iterable, Mapping


LANES = ("ocxo1", "ocxo2")
DAC_CODE_SCALE = 1_000_000_000
PPB_SCALE = 1_000_000
FS_PER_NS = 1_000_000
NS_PER_SECOND = 1_000_000_000

# Bootstrap with the existing servo estimate of +100 ppb per code. Each lane's
# measured slope belongs to the baseline that used it.
PROVISIONAL_SLOPE_MICRO_PPB_PER_CODE = 100 * PPB_SCALE
PHASE_DENOMINATOR = DAC_CODE_SCALE * NS_PER_SECOND

_WINDOW_SECONDS = {"10_min": 600, "60_min": 3_600, "8_hour": 28_800, "24_hour": 86_400}
_MINUTE_NS = 60 * NS_PER_SECOND


def round_ratio(numerator: int, denominator: int) -> int:
    magnitude = (2 * abs(numerator) + denominator) // (2 * denominator)
    return -magnitude if numerator < 0 else magnitude


def code_nano_to_voltage_nv(code_nano: int) -> int:
    # CLOCKS uses the AD5693R 5 V nominal full scale and 16-bit code scale.
    return round_ratio(5 * code_nano, 65_536)


def _round_sqrt_ratio(numerator: int, denominator: int) -> int:
    root = isqrt(numerator // denominator)
    return root + int(4 * numerator >= denominator * (2 * root + 1) ** 2)


@dataclass(frozen=True)
class _Endpoint:
    elapsed_ns: int = 0
    measured_ns: int = 0
    phase: tuple[int, int] = (0, 0)
    intervals: int = 0

    def encode(self) -> list[int]:
        return [self.elapsed_ns, self.measured_ns, *self.phase, self.intervals]

    @classmethod
    def decode(cls, record: list[int]) -> "_Endpoint":
        elapsed_ns, measured_ns, ocxo1, ocxo2, intervals = record
        return cls(elapsed_ns, measured_ns, (ocxo1, ocxo2), intervals)


class VoltageClock:
    def __init__(self, baseline: Mapping[str, Any]) -> None:
        self.baseline_id = baseline["id"]
        self._baseline_codes = tuple(baseline[lane]["code_nano"] for lane in LANES)
        self._slopes = tuple(baseline[lane]["slope_micro_ppb_per_code"] for lane in LANES)
        self.last_gnss_ns: int | None = None
        self._endpoint = _Endpoint()
        self._campaign_id: str | None = None
        self._campaign_origin: _Endpoint | None = None
        self._gaps = 0
        self._last_sample = "zero"
        self.reset_statistics()

    def reset_statistics(self) -> None:
        self._statistics_origin = self._endpoint
        self._seconds: deque[_Endpoint] = deque(maxlen=601)
        self._minutes: deque[_Endpoint] = deque(maxlen=1_441)
        self._moments = [[0, 0] for _ in LANES]
        self._remember(self._endpoint)

    def _remember(self, endpoint: _Endpoint) -> None:
        if not self._seconds or endpoint.elapsed_ns > self._seconds[-1].elapsed_ns:
            self._seconds.append(endpoint)
        origin_ns = self._statistics_origin.elapsed_ns
        if (
            not self._minutes
            or (endpoint.elapsed_ns - origin_ns) // _MINUTE_NS
            > (self._minutes[-1].elapsed_ns - origin_ns) // _MINUTE_NS
        ):
            self._minutes.append(endpoint)

    def advance(
        self,
        gnss_ns: int,
        applied_code_nano: Mapping[str, int] | None,
        campaign: str | None,
    ) -> dict[str, Any]:
        previous = self._endpoint
        if self.last_gnss_ns is not None:
            delta_ns = gnss_ns - self.last_gnss_ns
            if delta_ns <= 0:
                raise ValueError("CLOCKS voltage timeline did not advance")
            elapsed_ns = previous.elapsed_ns + delta_ns
            if applied_code_nano is None:
                # GNSS elapsed time is observed across the gap. No DAC correction
                # or frequency population is credited to its missing interval.
                self._endpoint = _Endpoint(
                    elapsed_ns, previous.measured_ns, previous.phase, previous.intervals,
                )
                self._gaps += 1
                self._last_sample = "gap"
            else:
                rate_numerators = tuple(
                    -self._slopes[index] * (
                        applied_code_nano[lane] - self._baseline_codes[index]
                    )
                    for index, lane in enumerate(LANES)
                )
                self._endpoint = _Endpoint(
                    elapsed_ns,
                    previous.measured_ns + delta_ns,
                    (
                        previous.phase[0] + rate_numerators[0] * delta_ns,
                        previous.phase[1] + rate_numerators[1] * delta_ns,
                    ),
                    previous.intervals + 1,
                )
                for moment, rate_numerator in zip(self._moments, rate_numerators):
                    moment[0] += rate_numerator
                    moment[1] += rate_numerator ** 2
                self._last_sample = "admitted"
            self._remember(self._endpoint)

        self.last_gnss_ns = gnss_ns
        self.set_campaign(campaign)
        return self.snapshot()

    def set_campaign(self, campaign: str | None) -> None:
        if campaign != self._campaign_id:
            self._campaign_id = campaign
            self._campaign_origin = self._endpoint if campaign is not None else None

    def _clockfaces(self, origin: _Endpoint) -> dict[str, int | None]:
        now = self._endpoint
        gnss_fs = (now.elapsed_ns - origin.elapsed_ns) * FS_PER_NS
        clockfaces: dict[str, int | None] = {"gnss": gnss_fs, "vclock": gnss_fs}
        for index, lane in enumerate(LANES):
            if self._last_sample == "gap" and now != origin:
                clockfaces[lane] = None
            else:
                # Keeping the phase numerator carries every fractional femtosecond.
                phase_delta = now.phase[index] - origin.phase[index]
                clockfaces[lane] = gnss_fs + round_ratio(phase_delta, PHASE_DENOMINATOR)
        return clockfaces

    def _window_anchor(self, seconds: int) -> _Endpoint | None:
        now = self._endpoint
        target = now.elapsed_ns - seconds * NS_PER_SECOND
        if target <= self._statistics_origin.elapsed_ns:
            return self._statistics_origin
        history = self._seconds if seconds == 600 else self._minutes
        return next((row for row in history if target <= row.elapsed_ns < now.elapsed_ns), None)

    def _ppb_e6(self, index: int, anchor: _Endpoint | None) -> int | None:
        if anchor is None:
            return None
        measured_ns = self._endpoint.measured_ns - anchor.measured_ns
        if measured_ns <= 0:
            return None
        phase_delta = self._endpoint.phase[index] - anchor.phase[index]
        return round_ratio(phase_delta, measured_ns * DAC_CODE_SCALE)

    def _welford(self, index: int, count: int) -> dict[str, int | None]:
        if count == 0:
            return {"n": 0, "mean_e6": None, "stddev_e6": None, "stderr_e6": None}
        total, sum_squares = self._moments[index]
        # Exact integer moments give the Welford sample variance and standard
        # error without a precision loss from floating-point cancellation.
        variance_numerator = count * sum_squares - total ** 2
        variance_denominator = count * (count - 1) * DAC_CODE_SCALE ** 2
        return {
            "n": count,
            "mean_e6": round_ratio(total, count * DAC_CODE_SCALE),
            "stddev_e6": _round_sqrt_ratio(variance_numerator, variance_denominator) if count > 1 else 0,
            "stderr_e6": _round_sqrt_ratio(variance_numerator, variance_denominator * count) if count > 1 else 0,
        }

    def snapshot(self) -> dict[str, Any]:
        anchors = {name: self._window_anchor(seconds) for name, seconds in _WINDOW_SECONDS.items()}
        anchors["total"] = self._statistics_origin
        count = self._endpoint.intervals - self._statistics_origin.intervals
        stats: dict[str, Any] = {
            lane: {
                "ppb_buckets_e6": {name: 0 for name in anchors},
                "welford": {"n": count, "mean_e6": 0, "stddev_e6": 0, "stderr_e6": 0},
            }
            for lane in ("gnss", "vclock")
        }
        for index, lane in enumerate(LANES):
            stats[lane] = {
                "ppb_buckets_e6": {name: self._ppb_e6(index, anchor) for name, anchor in anchors.items()},
                "welford": self._welford(index, count),
            }

        campaign = None
        if self._campaign_origin is not None:
            campaign = {
                "segment_id": self._campaign_id,
                "clockfaces_fs": self._clockfaces(self._campaign_origin),
                "ppb_e6": {"gnss": 0, "vclock": 0},
            }
            for index, lane in enumerate(LANES):
                campaign["ppb_e6"][lane] = self._ppb_e6(index, self._campaign_origin)
        return {
            "clockfaces_fs": self._clockfaces(_Endpoint()),
            "stats": stats,
            "campaign": campaign,
            "gaps": self._gaps,
            "last_sample": self._last_sample,
        }

    def checkpoint(self) -> dict[str, Any]:
        return {
            "baseline_id": self.baseline_id,
            "last_gnss_ns": self.last_gnss_ns,
            "endpoint": self._endpoint.encode(),
            "statistics_origin": self._statistics_origin.encode(),
            "campaign_id": self._campaign_id,
            "campaign_origin": None if self._campaign_origin is None else self._campaign_origin.encode(),
            "moments": [moment.copy() for moment in self._moments],
            "gaps": self._gaps,
            "last_sample": self._last_sample,
        }

    @classmethod
    def restore(
        cls,
        baseline: Mapping[str, Any],
        checkpoint: Mapping[str, Any],
        history: Iterable[Mapping[str, Any]] = (),
    ) -> "VoltageClock":
        clock = cls(baseline)
        if checkpoint["baseline_id"] != clock.baseline_id:
            raise ValueError("CLOCKS voltage checkpoint belongs to another baseline")

        clock.last_gnss_ns = checkpoint["last_gnss_ns"]
        clock._endpoint = _Endpoint.decode(checkpoint["endpoint"])
        clock._statistics_origin = _Endpoint.decode(checkpoint["statistics_origin"])
        clock._campaign_id = checkpoint["campaign_id"]
        campaign_origin = checkpoint["campaign_origin"]
        clock._campaign_origin = None if campaign_origin is None else _Endpoint.decode(campaign_origin)
        clock._moments = [list(moment) for moment in checkpoint["moments"]]
        clock._gaps = checkpoint["gaps"]
        clock._last_sample = checkpoint["last_sample"]

        clock._seconds.clear()
        clock._minutes.clear()
        clock._remember(clock._statistics_origin)
        # Each persisted row carries only one endpoint. Rebuild the bounded
        # windows from ascending rows within the exact reporting population.
        for row in history:
            if (
                row["baseline_id"] != clock.baseline_id
                or row["statistics_origin"] != checkpoint["statistics_origin"]
            ):
                continue
            endpoint = _Endpoint.decode(row["endpoint"])
            if clock._statistics_origin.elapsed_ns < endpoint.elapsed_ns <= clock._endpoint.elapsed_ns:
                clock._remember(endpoint)
        clock._remember(clock._endpoint)
        return clock
