"""ZPNet campaign_analyzer — holistic CLOCKS lineage and recovery audit.

Despite the historical filename, this is no longer campaign-scoped at its core.

The analyzer streams canonical CLOCKS_V4 rows in durable PostgreSQL insertion
order and treats the always-on CLOCKS instrument lineage as the primary object
of study.  Optional TEMPEST campaign state is audited as a dependent view.

The purpose is deliberately strict: restart/recovery is correct only when every
recoverable sufficient state either continues exactly or changes for an explicit
and lawful reason.  The analyzer therefore cross-checks independent witnesses:

* durable row order versus boot-local physical sequence;
* CLOCKS clockfaces and completed PPS identity;
* statistical reset/update chronology;
* every firmware Welford sufficient state;
* OCXO TAU sufficient state and TOTAL PPB;
* Better-Buckets cumulative endpoint reconstruction;
* Pi-owned DAC Welfords against both DAC samples and OCXO ancestry;
* Pi-owned GNSS_RAW Welford continuation;
* campaign public-time/clockface/statistical closure;
* recovery quarantine/supersession testimony.

It never repairs, fills, smooths, or substitutes missing state.  Unknown or
unprovable continuity is reported as such.

Memory use is bounded by the current/previous compact row, Better-Buckets ring
geometry, online counters, and bounded examples.
"""

from __future__ import annotations

import json
import math
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Deque, Dict, Iterator, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_PPS = 1_008_000_000
DEFAULT_BATCH_SIZE = 32
DEFAULT_PAUSE_MS = 0
MAX_EXAMPLES = 16
CAMPAIGN_TYPE = "TEMPEST"
CLOCKS_SCHEMA = "CLOCKS_V4"

PPB_TOLERANCE = 2.0e-6
DISPLAY_PPB_TOLERANCE = 6.0e-4
TAU_TOLERANCE = 2.0e-12
WELFORD_FLOAT_ABS_TOL = 2.0e-8
WELFORD_FLOAT_REL_TOL = 2.0e-12
DAC_SAMPLE_TOL = 1.0e-9

PPB_10_MIN_SECONDS = 10 * 60
PPB_60_MIN_SECONDS = 60 * 60
PPB_8_HOUR_SECONDS = 8 * 60 * 60
PPB_24_HOUR_SECONDS = 24 * 60 * 60
PPB_SECOND_CAPACITY = PPB_10_MIN_SECONDS + 1
PPB_MINUTE_CAPACITY = 24 * 60 + 2

STAT_LANES = ("gnss", "dwt", "vclock", "ocxo1", "ocxo2")
WELFORD_LANES = (
    "gnss",
    "dwt",
    "vclock",
    "ocxo1",
    "ocxo2",
    "pps_witness",
    "ocxo1_dac",
    "ocxo2_dac",
)
PPB_LANES = ("dwt", "vclock", "ocxo1", "ocxo2")


def d(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def req_dict(value: Any, where: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{where} must be an object")
    return value


def req_int(value: Any, where: str, *, minimum: Optional[int] = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{where} must be an integer; got {value!r}")
    out = int(value)
    if minimum is not None and out < minimum:
        raise ValueError(f"{where} must be >= {minimum}; got {out}")
    return out


def opt_int(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def req_float(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{where} must be numeric; got {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{where} must be finite; got {value!r}")
    return out


def opt_float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return out if math.isfinite(out) else None


def opt_bool(value: Any) -> Optional[bool]:
    return value if isinstance(value, bool) else None


def path(obj: Dict[str, Any], dotted: str) -> Any:
    cur: Any = obj
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def close(a: float, b: float, *, abs_tol: float, rel_tol: float = 0.0) -> bool:
    return math.isclose(a, b, abs_tol=abs_tol, rel_tol=rel_tol)


def utc_seconds(text: str) -> Optional[int]:
    raw = str(text or "").strip()
    if not raw:
        return None
    try:
        dt = datetime.fromisoformat(raw.replace("Z", "+00:00"))
    except ValueError:
        return None
    if dt.tzinfo is None:
        return None
    return int(dt.astimezone(timezone.utc).timestamp())


@dataclass(frozen=True)
class Welford:
    n: int
    mean: float
    m2: float
    min_value: float
    max_value: float
    stddev: Optional[float]
    stderr: Optional[float]

    @classmethod
    def parse(cls, obj: Any, where: str) -> "Welford":
        src = req_dict(obj, where)
        n = req_int(src.get("n"), f"{where}.n", minimum=0)
        mean = req_float(src.get("mean"), f"{where}.mean")
        m2 = req_float(src.get("m2"), f"{where}.m2")
        min_value = req_float(src.get("min"), f"{where}.min")
        max_value = req_float(src.get("max"), f"{where}.max")
        stddev = opt_float(src.get("stddev"))
        stderr = opt_float(src.get("stderr"))
        if n == 0:
            if any(v != 0.0 for v in (mean, m2, min_value, max_value)):
                raise ValueError(f"{where}: empty Welford publishes non-zero sufficient state")
        else:
            if m2 < -WELFORD_FLOAT_ABS_TOL:
                raise ValueError(f"{where}: negative m2={m2}")
            if min_value > max_value:
                raise ValueError(f"{where}: min={min_value} > max={max_value}")
            if mean < min_value - WELFORD_FLOAT_ABS_TOL or mean > max_value + WELFORD_FLOAT_ABS_TOL:
                raise ValueError(
                    f"{where}: mean={mean} outside [{min_value}, {max_value}]"
                )
        return cls(n, mean, m2, min_value, max_value, stddev, stderr)


@dataclass(frozen=True)
class TauState:
    valid: bool
    sample_count: int
    interval_count: int
    cumulative_reference_ns: int
    cumulative_clock_ns: int
    cumulative_clock_ns_exact: float
    tau: Optional[float]
    ppb: Optional[float]
    first_refined_ns: Optional[int]
    last_refined_ns: Optional[int]
    last_pps_sequence: Optional[int]
    last_interval_pps_sequence: Optional[int]
    gap_reset_count: Optional[int]
    reset_count: Optional[int]
    reject_count: Optional[int]

    @classmethod
    def parse(cls, obj: Any, where: str) -> "TauState":
        src = req_dict(obj, where)
        return cls(
            valid=bool(src.get("valid") is True),
            sample_count=req_int(src.get("sample_count"), f"{where}.sample_count", minimum=0),
            interval_count=req_int(src.get("interval_count"), f"{where}.interval_count", minimum=0),
            cumulative_reference_ns=req_int(
                src.get("cumulative_reference_ns"),
                f"{where}.cumulative_reference_ns",
                minimum=0,
            ),
            cumulative_clock_ns=req_int(
                src.get("cumulative_clock_ns"),
                f"{where}.cumulative_clock_ns",
                minimum=0,
            ),
            cumulative_clock_ns_exact=req_float(
                src.get("cumulative_clock_ns_exact"),
                f"{where}.cumulative_clock_ns_exact",
            ),
            tau=opt_float(src.get("tau")),
            ppb=opt_float(src.get("ppb")),
            first_refined_ns=opt_int(src.get("first_refined_ns")),
            last_refined_ns=opt_int(src.get("last_refined_ns")),
            last_pps_sequence=opt_int(src.get("last_pps_sequence")),
            last_interval_pps_sequence=opt_int(src.get("last_interval_pps_sequence")),
            gap_reset_count=opt_int(src.get("gap_reset_count")),
            reset_count=opt_int(src.get("reset_count")),
            reject_count=opt_int(src.get("reject_count")),
        )


@dataclass(frozen=True)
class BetterEndpoint:
    rolling_sequence: int
    reference_ns: int
    dwt_error_cycles: float
    ocxo1_error_ns: int
    ocxo2_error_ns: int
    interval_count: int


@dataclass
class BetterBuckets:
    epoch: Optional[int] = None
    complete_from_origin: bool = False
    current: BetterEndpoint = field(
        default_factory=lambda: BetterEndpoint(0, 0, 0.0, 0, 0, 0)
    )
    origin: Optional[BetterEndpoint] = None
    seconds: Deque[BetterEndpoint] = field(
        default_factory=lambda: deque(maxlen=PPB_SECOND_CAPACITY)
    )
    minutes: Deque[BetterEndpoint] = field(
        default_factory=lambda: deque(maxlen=PPB_MINUTE_CAPACITY)
    )
    last_minute_key: Optional[int] = None
    previous_raw: Optional[Tuple[int, int, int, int, int]] = None

    def reset(self, epoch: int, first_rolling_sequence: int) -> None:
        self.epoch = epoch
        self.complete_from_origin = first_rolling_sequence in (0, 1)
        self.current = BetterEndpoint(0, 0, 0.0, 0, 0, 0)
        self.origin = None
        self.seconds.clear()
        self.minutes.clear()
        self.last_minute_key = None
        self.previous_raw = None

    @staticmethod
    def minute_key(rolling_sequence: int) -> int:
        return ((rolling_sequence - 1) // 60) + 1

    def consume(self, row: "Row") -> None:
        if self.epoch != row.reset_count:
            self.reset(row.reset_count, row.rolling_sequence)

        if not row.rolling_endpoint_admitted or row.rolling_sequence <= 0:
            self.previous_raw = None
            return

        raw = (
            row.rolling_sequence,
            row.clock_gnss_ns,
            row.clock_dwt_cycles,
            row.clock_ocxo1_ns,
            row.clock_ocxo2_ns,
        )

        cur = self.current
        next_ep = BetterEndpoint(
            rolling_sequence=row.rolling_sequence,
            reference_ns=cur.reference_ns,
            dwt_error_cycles=cur.dwt_error_cycles,
            ocxo1_error_ns=cur.ocxo1_error_ns,
            ocxo2_error_ns=cur.ocxo2_error_ns,
            interval_count=cur.interval_count,
        )

        advanced_by_reconstruction: Optional[bool] = None
        if self.previous_raw is not None:
            pseq, pgnss, pdwt, po1, po2 = self.previous_raw
            advanced_by_reconstruction = False
            adjacent = (
                row.rolling_sequence == pseq + 1
                and row.clock_gnss_ns > pgnss
                and row.clock_dwt_cycles > pdwt
                and row.clock_ocxo1_ns > po1
                and row.clock_ocxo2_ns > po2
            )
            if adjacent:
                reference_delta = row.clock_gnss_ns - pgnss
                dwt_delta = row.clock_dwt_cycles - pdwt
                ocxo1_delta = row.clock_ocxo1_ns - po1
                ocxo2_delta = row.clock_ocxo2_ns - po2
                expected_dwt_delta = (
                    float(reference_delta) * float(DWT_EXPECTED_PER_PPS)
                    / float(NS_PER_SECOND)
                )
                next_ep = BetterEndpoint(
                    rolling_sequence=row.rolling_sequence,
                    reference_ns=cur.reference_ns + reference_delta,
                    dwt_error_cycles=cur.dwt_error_cycles + float(dwt_delta) - expected_dwt_delta,
                    ocxo1_error_ns=cur.ocxo1_error_ns + (ocxo1_delta - reference_delta),
                    ocxo2_error_ns=cur.ocxo2_error_ns + (ocxo2_delta - reference_delta),
                    interval_count=cur.interval_count + 1,
                )
                advanced_by_reconstruction = True

        # Firmware testimony is authoritative.  A disagreement here means our
        # durable rows cannot reproduce the producer's Better-Buckets ancestry.
        if (
            advanced_by_reconstruction is not None
            and row.rolling_interval_advanced != advanced_by_reconstruction
        ):
            row.audit.problem(
                "better_interval_advance",
                row,
                f"published={row.rolling_interval_advanced} "
                f"reconstructed={advanced_by_reconstruction}",
            )

        self.current = next_ep
        if self.origin is None:
            self.origin = next_ep

        self.seconds.append(next_ep)
        minute_key = self.minute_key(row.rolling_sequence)
        if minute_key != self.last_minute_key:
            self.minutes.append(next_ep)
            self.last_minute_key = minute_key

        self.previous_raw = raw

    @staticmethod
    def find_anchor(
        ring: Deque[BetterEndpoint],
        target: int,
        current_sequence: int,
    ) -> Optional[BetterEndpoint]:
        for endpoint in ring:
            if target <= endpoint.rolling_sequence < current_sequence:
                return endpoint
        return None

    def anchor(
        self,
        window_seconds: int,
        *,
        exact_second_history: bool,
    ) -> Optional[BetterEndpoint]:
        if (
            self.origin is None
            or self.current.rolling_sequence <= 0
            or self.origin.rolling_sequence >= self.current.rolling_sequence
        ):
            return None
        if self.current.rolling_sequence <= window_seconds:
            return self.origin
        target = self.current.rolling_sequence - window_seconds
        ring = self.seconds if exact_second_history else self.minutes
        return self.find_anchor(ring, target, self.current.rolling_sequence)

    def expected_ppb(
        self,
        lane: str,
        window_seconds: int,
        *,
        exact_second_history: bool,
    ) -> Optional[float]:
        if not self.complete_from_origin:
            return None
        anchor = self.anchor(window_seconds, exact_second_history=exact_second_history)
        if anchor is None:
            return None
        interval_count = self.current.interval_count - anchor.interval_count
        reference_ns = self.current.reference_ns - anchor.reference_ns
        if interval_count <= 0 or reference_ns <= 0:
            return None
        if lane == "dwt":
            expected_cycles = (
                float(reference_ns) * float(DWT_EXPECTED_PER_PPS)
                / float(NS_PER_SECOND)
            )
            error = self.current.dwt_error_cycles - anchor.dwt_error_cycles
            return error * 1.0e9 / expected_cycles
        if lane == "vclock":
            return 0.0
        if lane == "ocxo1":
            error = self.current.ocxo1_error_ns - anchor.ocxo1_error_ns
            return float(error) * 1.0e9 / float(reference_ns)
        if lane == "ocxo2":
            error = self.current.ocxo2_error_ns - anchor.ocxo2_error_ns
            return float(error) * 1.0e9 / float(reference_ns)
        raise ValueError(f"unknown Better-Buckets lane {lane!r}")


@dataclass
class Row:
    audit: "Audit"
    db_id: int
    ts: str
    payload: Dict[str, Any]

    sequence: int
    completed_pps_sequence: int
    gnss_utc: str
    gnss_utc_s: Optional[int]
    instrument_age_seconds: int

    clock_gnss_ns: int
    clock_dwt_cycles: int
    clock_ocxo1_ns: int
    clock_ocxo2_ns: int
    clockface_pps_sequence: int

    reset_count: int
    update_count: int
    stats_last_pps_sequence: int
    rolling_sequence: int
    rolling_endpoint_admitted: bool
    rolling_interval_advanced: bool

    welfords: Dict[str, Welford]
    tau: Dict[str, TauState]
    ppb: Dict[str, float]
    frequency_tau: Dict[str, Optional[float]]
    buckets: Dict[str, Dict[str, float]]

    dac_hw: Dict[str, int]
    dac_readback: Dict[str, Optional[int]]
    dac_target: Dict[str, float]
    dither_enabled: bool
    servo_mode: str

    gnss_raw_drift_ppb: Optional[float]
    gnss_raw_welford: Optional[Welford]
    gnss_raw_clockface_n: Optional[int]
    gnss_raw_ns: Optional[int]
    gnss_raw_ref_ns: Optional[int]

    campaign_name: Optional[str]
    campaign_public_count: Optional[int]
    campaign_clockfaces: Dict[str, int]
    campaign_ppb: Dict[str, float]
    campaign_science_eligible: Optional[bool]

    recovery_classification: Optional[str]
    superseded: bool

    @classmethod
    def parse(
        cls,
        audit: "Audit",
        db_id: int,
        ts: str,
        payload: Dict[str, Any],
    ) -> "Row":
        root = payload
        if root.get("schema") != CLOCKS_SCHEMA:
            inner = d(root.get("payload"))
            if inner.get("schema") == CLOCKS_SCHEMA:
                root = inner
        if root.get("schema") != CLOCKS_SCHEMA:
            raise ValueError(f"db_id={db_id}: payload is not {CLOCKS_SCHEMA}")

        clocks = req_dict(root.get("clocks"), f"db_id={db_id}.clocks")
        if clocks.get("schema") != "CLOCKS_INSTRUMENT_STATE_V1":
            raise ValueError(
                f"db_id={db_id}: clocks schema={clocks.get('schema')!r}"
            )
        stats = req_dict(clocks.get("stats"), f"db_id={db_id}.clocks.stats")
        clockfaces = req_dict(
            clocks.get("clockfaces"),
            f"db_id={db_id}.clocks.clockfaces",
        )
        control = req_dict(clocks.get("control"), f"db_id={db_id}.clocks.control")

        welfords: Dict[str, Welford] = {}
        for lane in STAT_LANES:
            welfords[lane] = Welford.parse(
                path(stats, f"{lane}.welford"),
                f"db_id={db_id}.stats.{lane}.welford",
            )
        aux = req_dict(
            stats.get("auxiliary_welford"),
            f"db_id={db_id}.stats.auxiliary_welford",
        )
        welfords["pps_witness"] = Welford.parse(
            aux.get("pps_witness"),
            f"db_id={db_id}.stats.auxiliary_welford.pps_witness",
        )
        welfords["ocxo1_dac"] = Welford.parse(
            aux.get("ocxo1_dac"),
            f"db_id={db_id}.stats.auxiliary_welford.ocxo1_dac",
        )
        welfords["ocxo2_dac"] = Welford.parse(
            aux.get("ocxo2_dac"),
            f"db_id={db_id}.stats.auxiliary_welford.ocxo2_dac",
        )

        tau = {
            "ocxo1": TauState.parse(
                stats.get("ocxo1_tau_state"),
                f"db_id={db_id}.stats.ocxo1_tau_state",
            ),
            "ocxo2": TauState.parse(
                stats.get("ocxo2_tau_state"),
                f"db_id={db_id}.stats.ocxo2_tau_state",
            ),
        }

        ppb: Dict[str, float] = {}
        frequency_tau: Dict[str, Optional[float]] = {}
        buckets: Dict[str, Dict[str, float]] = {}
        for lane in PPB_LANES:
            lane_obj = req_dict(stats.get(lane), f"db_id={db_id}.stats.{lane}")
            ppb[lane] = req_float(lane_obj.get("ppb"), f"db_id={db_id}.stats.{lane}.ppb")
            frequency_tau[lane] = opt_float(lane_obj.get("tau"))
            bucket_obj = req_dict(
                lane_obj.get("ppb_buckets"),
                f"db_id={db_id}.stats.{lane}.ppb_buckets",
            )
            buckets[lane] = {}
            for key in ("10_min", "60_min", "8_hour", "24_hour", "total"):
                value = opt_float(bucket_obj.get(key))
                if value is not None:
                    buckets[lane][key] = value

        dac_hw: Dict[str, int] = {}
        dac_readback: Dict[str, Optional[int]] = {}
        dac_target: Dict[str, float] = {}
        for lane in ("ocxo1", "ocxo2"):
            lane_obj = req_dict(control.get(lane), f"db_id={db_id}.control.{lane}")
            dac_hw[lane] = req_int(
                lane_obj.get("hw_code"),
                f"db_id={db_id}.control.{lane}.hw_code",
                minimum=0,
            )
            dac_readback[lane] = opt_int(lane_obj.get("readback_code"))
            dac_target[lane] = req_float(
                lane_obj.get("target_code"),
                f"db_id={db_id}.control.{lane}.target_code",
            )

        gnss_raw = d(clocks.get("gnss_raw"))
        gnss_raw_welford: Optional[Welford] = None
        if gnss_raw:
            gnss_raw_welford = Welford.parse(
                gnss_raw.get("welford"),
                f"db_id={db_id}.clocks.gnss_raw.welford",
            )
        gnss_raw_instr = d(gnss_raw.get("instrument"))

        campaign_obj = d(root.get("campaign"))
        campaign_name: Optional[str] = None
        campaign_public_count: Optional[int] = None
        campaign_clockfaces: Dict[str, int] = {}
        campaign_ppb: Dict[str, float] = {}
        campaign_science_eligible: Optional[bool] = None
        if campaign_obj.get("schema") == "TEMPEST_FRAGMENT_V1":
            campaign_name = str(campaign_obj.get("name") or "") or None
            campaign_public_count = opt_int(campaign_obj.get("public_count"))
            cfaces = d(campaign_obj.get("clockfaces"))
            for key in ("gnss_ns", "dwt_cycles", "ocxo1_ns", "ocxo2_ns"):
                value = opt_int(cfaces.get(key))
                if value is not None:
                    campaign_clockfaces[key] = value
            cppb = d(d(campaign_obj.get("stats")).get("ppb"))
            for lane in PPB_LANES:
                value = opt_float(cppb.get(lane))
                if value is not None:
                    campaign_ppb[lane] = value
            campaign_science_eligible = opt_bool(
                d(campaign_obj.get("disposition")).get("science_eligible")
            )

        recovery = d(root.get("recovery_custody"))
        classification = str(recovery.get("classification") or "").strip() or None

        return cls(
            audit=audit,
            db_id=db_id,
            ts=ts,
            payload=root,
            sequence=req_int(root.get("sequence"), f"db_id={db_id}.sequence", minimum=1),
            completed_pps_sequence=req_int(
                clocks.get("completed_pps_sequence"),
                f"db_id={db_id}.clocks.completed_pps_sequence",
                minimum=1,
            ),
            gnss_utc=str(clocks.get("gnss_time_utc") or ""),
            gnss_utc_s=utc_seconds(str(clocks.get("gnss_time_utc") or "")),
            instrument_age_seconds=req_int(
                clocks.get("instrument_age_seconds"),
                f"db_id={db_id}.clocks.instrument_age_seconds",
                minimum=0,
            ),
            clock_gnss_ns=req_int(
                clockfaces.get("gnss_ns"),
                f"db_id={db_id}.clocks.clockfaces.gnss_ns",
                minimum=0,
            ),
            clock_dwt_cycles=req_int(
                clockfaces.get("dwt_cycles"),
                f"db_id={db_id}.clocks.clockfaces.dwt_cycles",
                minimum=0,
            ),
            clock_ocxo1_ns=req_int(
                clockfaces.get("ocxo1_ns"),
                f"db_id={db_id}.clocks.clockfaces.ocxo1_ns",
                minimum=0,
            ),
            clock_ocxo2_ns=req_int(
                clockfaces.get("ocxo2_ns"),
                f"db_id={db_id}.clocks.clockfaces.ocxo2_ns",
                minimum=0,
            ),
            clockface_pps_sequence=req_int(
                clockfaces.get("pps_sequence"),
                f"db_id={db_id}.clocks.clockfaces.pps_sequence",
                minimum=1,
            ),
            reset_count=req_int(
                stats.get("reset_count"),
                f"db_id={db_id}.stats.reset_count",
                minimum=0,
            ),
            update_count=req_int(
                stats.get("update_count"),
                f"db_id={db_id}.stats.update_count",
                minimum=1,
            ),
            stats_last_pps_sequence=req_int(
                stats.get("last_pps_sequence"),
                f"db_id={db_id}.stats.last_pps_sequence",
                minimum=1,
            ),
            rolling_sequence=req_int(
                stats.get("rolling_ppb_current_sequence"),
                f"db_id={db_id}.stats.rolling_ppb_current_sequence",
                minimum=0,
            ),
            rolling_endpoint_admitted=bool(
                stats.get("rolling_ppb_endpoint_admitted") is True
            ),
            rolling_interval_advanced=bool(
                stats.get("rolling_ppb_interval_advanced") is True
            ),
            welfords=welfords,
            tau=tau,
            ppb=ppb,
            frequency_tau=frequency_tau,
            buckets=buckets,
            dac_hw=dac_hw,
            dac_readback=dac_readback,
            dac_target=dac_target,
            dither_enabled=bool(control.get("dither_operator_enabled") is True),
            servo_mode=str(control.get("servo_mode") or ""),
            gnss_raw_drift_ppb=opt_float(gnss_raw.get("drift_ppb")),
            gnss_raw_welford=gnss_raw_welford,
            gnss_raw_clockface_n=opt_int(gnss_raw_instr.get("clockface_n")),
            gnss_raw_ns=opt_int(gnss_raw_instr.get("ns")),
            gnss_raw_ref_ns=opt_int(gnss_raw_instr.get("ref_ns")),
            campaign_name=campaign_name,
            campaign_public_count=campaign_public_count,
            campaign_clockfaces=campaign_clockfaces,
            campaign_ppb=campaign_ppb,
            campaign_science_eligible=campaign_science_eligible,
            recovery_classification=classification,
            superseded=bool(root.get("holistic_restore_superseded") is True),
        )


@dataclass
class Audit:
    scope: str
    rows: int = 0
    first_db_id: Optional[int] = None
    last_db_id: Optional[int] = None
    first_ts: str = ""
    last_ts: str = ""
    first_gnss: str = ""
    last_gnss: str = ""

    problems: Counter[str] = field(default_factory=Counter)
    events: Counter[str] = field(default_factory=Counter)
    examples: Dict[str, list[str]] = field(default_factory=dict)

    epochs: Counter[int] = field(default_factory=Counter)
    campaigns: Counter[str] = field(default_factory=Counter)
    recovery_classes: Counter[str] = field(default_factory=Counter)

    better: BetterBuckets = field(default_factory=BetterBuckets)
    exact_welford_steps: int = 0
    exact_dac_steps: int = 0
    exact_gnss_raw_steps: int = 0
    bucket_checks: int = 0
    total_ppb_checks: int = 0
    tau_checks: int = 0

    physical_reboots: int = 0
    durable_gaps: int = 0
    gnss_gap_seconds: int = 0
    stats_resets: int = 0
    campaign_splices: int = 0

    def note(self, bucket: str, message: str) -> None:
        examples = self.examples.setdefault(bucket, [])
        if len(examples) < MAX_EXAMPLES:
            examples.append(message)

    def problem(self, kind: str, row: Row, detail: str) -> None:
        self.problems[kind] += 1
        self.note(kind, f"db_id={row.db_id} gnss={row.gnss_utc or '?'} {detail}")

    def event(self, kind: str, row: Row, detail: str) -> None:
        self.events[kind] += 1
        self.note(f"event:{kind}", f"db_id={row.db_id} gnss={row.gnss_utc or '?'} {detail}")


def welford_state_same(a: Welford, b: Welford) -> bool:
    return (
        a.n == b.n
        and close(a.mean, b.mean, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL)
        and close(a.m2, b.m2, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL)
        and close(
            a.min_value,
            b.min_value,
            abs_tol=WELFORD_FLOAT_ABS_TOL,
            rel_tol=WELFORD_FLOAT_REL_TOL,
        )
        and close(
            a.max_value,
            b.max_value,
            abs_tol=WELFORD_FLOAT_ABS_TOL,
            rel_tol=WELFORD_FLOAT_REL_TOL,
        )
    )


def check_welford_step(
    audit: Audit,
    row: Row,
    name: str,
    previous: Welford,
    current: Welford,
    *,
    known_sample: Optional[float] = None,
) -> None:
    if current.n < previous.n:
        audit.problem(
            "welford_regression",
            row,
            f"{name}.n {previous.n}->{current.n} within reset_count={row.reset_count}",
        )
        return

    if current.n == previous.n:
        if not welford_state_same(previous, current):
            audit.problem(
                "welford_mutated_without_population",
                row,
                f"{name} n={current.n} but sufficient state changed",
            )
        return

    if current.n != previous.n + 1:
        audit.event(
            "welford_unobserved_advancement",
            row,
            f"{name}.n {previous.n}->{current.n}; cannot single-step prove hidden samples",
        )
        return

    if known_sample is None:
        # Derive the unique sample implied by the two means.
        sample = current.n * current.mean - previous.n * previous.mean
    else:
        sample = float(known_sample)

    expected_mean = previous.mean + (sample - previous.mean) / current.n
    expected_m2 = previous.m2 + (sample - previous.mean) * (sample - expected_mean)
    expected_min = sample if previous.n == 0 else min(previous.min_value, sample)
    expected_max = sample if previous.n == 0 else max(previous.max_value, sample)

    checks = (
        close(
            current.mean,
            expected_mean,
            abs_tol=WELFORD_FLOAT_ABS_TOL,
            rel_tol=WELFORD_FLOAT_REL_TOL,
        ),
        close(
            current.m2,
            expected_m2,
            abs_tol=max(WELFORD_FLOAT_ABS_TOL, abs(expected_m2) * 2.0e-12),
            rel_tol=2.0e-12,
        ),
        close(
            current.min_value,
            expected_min,
            abs_tol=WELFORD_FLOAT_ABS_TOL,
            rel_tol=WELFORD_FLOAT_REL_TOL,
        ),
        close(
            current.max_value,
            expected_max,
            abs_tol=WELFORD_FLOAT_ABS_TOL,
            rel_tol=WELFORD_FLOAT_REL_TOL,
        ),
    )
    if not all(checks):
        audit.problem(
            "welford_recurrence",
            row,
            f"{name} one-sample recurrence failed sample={sample:.12g} "
            f"n={previous.n}->{current.n}",
        )
    else:
        audit.exact_welford_steps += 1


def check_welford_geometry(audit: Audit, row: Row, name: str, w: Welford) -> None:
    if w.n < 2:
        expected_sd = 0.0
    else:
        expected_sd = math.sqrt(max(0.0, w.m2) / float(w.n - 1))
    expected_se = expected_sd / math.sqrt(float(w.n)) if w.n else 0.0

    if w.stddev is not None and not close(
        w.stddev,
        expected_sd,
        abs_tol=2.0e-6,
        rel_tol=2.0e-9,
    ):
        audit.problem(
            "welford_stddev",
            row,
            f"{name} published={w.stddev:.12g} expected={expected_sd:.12g}",
        )
    if w.stderr is not None and not close(
        w.stderr,
        expected_se,
        abs_tol=2.0e-6,
        rel_tol=2.0e-9,
    ):
        audit.problem(
            "welford_stderr",
            row,
            f"{name} published={w.stderr:.12g} expected={expected_se:.12g}",
        )


def check_tau(audit: Audit, row: Row, lane: str) -> None:
    tau = row.tau[lane]
    w = row.welfords[lane]

    if tau.sample_count != tau.interval_count:
        audit.problem(
            "tau_population",
            row,
            f"{lane} sample_count={tau.sample_count} interval_count={tau.interval_count}",
        )
    if tau.sample_count != w.n:
        audit.problem(
            "tau_welford_population",
            row,
            f"{lane} tau.n={tau.sample_count} welford.n={w.n}",
        )
    if tau.cumulative_reference_ns != tau.sample_count * NS_PER_SECOND:
        audit.problem(
            "tau_reference_total",
            row,
            f"{lane} reference={tau.cumulative_reference_ns} "
            f"expected={tau.sample_count * NS_PER_SECOND}",
        )

    if tau.valid:
        if tau.cumulative_clock_ns_exact <= 0.0 or tau.cumulative_reference_ns <= 0:
            audit.problem("tau_valid_without_population", row, lane)
            return
        expected_tau = (
            float(tau.cumulative_reference_ns) / tau.cumulative_clock_ns_exact
        )
        expected_ppb = (expected_tau - 1.0) * 1.0e9
        published_tau = row.frequency_tau.get(lane)
        if published_tau is not None and not close(
            published_tau,
            expected_tau,
            abs_tol=TAU_TOLERANCE,
            rel_tol=TAU_TOLERANCE,
        ):
            audit.problem(
                "tau_ratio",
                row,
                f"{lane} published={published_tau} expected={expected_tau:.15f}",
            )
        if not close(
            row.ppb[lane],
            expected_ppb,
            abs_tol=DISPLAY_PPB_TOLERANCE,
            rel_tol=2.0e-12,
        ):
            audit.problem(
                "total_ppb_tau",
                row,
                f"{lane} stats.ppb={row.ppb[lane]:.9f} tau={expected_ppb:.9f}",
            )
        total = row.buckets[lane].get("total")
        if total is not None:
            audit.total_ppb_checks += 1
            if not close(total, expected_ppb, abs_tol=PPB_TOLERANCE, rel_tol=2.0e-12):
                audit.problem(
                    "bucket_total_tau",
                    row,
                    f"{lane} total={total:.9f} tau={expected_ppb:.9f}",
                )
        audit.tau_checks += 1


def check_dwt_vclock_total(audit: Audit, row: Row, lane: str) -> None:
    w = row.welfords[lane]
    if not close(
        row.ppb[lane],
        w.mean,
        abs_tol=DISPLAY_PPB_TOLERANCE,
        rel_tol=2.0e-12,
    ):
        audit.problem(
            "total_ppb_welford",
            row,
            f"{lane} stats.ppb={row.ppb[lane]:.9f} mean={w.mean:.9f}",
        )
    total = row.buckets[lane].get("total")
    if total is not None:
        audit.total_ppb_checks += 1
        if not close(total, w.mean, abs_tol=PPB_TOLERANCE, rel_tol=2.0e-12):
            audit.problem(
                "bucket_total_welford",
                row,
                f"{lane} total={total:.9f} mean={w.mean:.9f}",
            )


def check_buckets(audit: Audit, row: Row) -> None:
    audit.better.consume(row)
    windows = (
        ("10_min", PPB_10_MIN_SECONDS, True),
        ("60_min", PPB_60_MIN_SECONDS, False),
        ("8_hour", PPB_8_HOUR_SECONDS, False),
        ("24_hour", PPB_24_HOUR_SECONDS, False),
    )
    for lane in PPB_LANES:
        for key, seconds, exact in windows:
            published = row.buckets[lane].get(key)
            expected = audit.better.expected_ppb(
                lane,
                seconds,
                exact_second_history=exact,
            )
            if published is None or expected is None:
                continue
            audit.bucket_checks += 1
            if not close(
                published,
                expected,
                abs_tol=PPB_TOLERANCE,
                rel_tol=2.0e-12,
            ):
                audit.problem(
                    "better_bucket_value",
                    row,
                    f"{lane}.{key} published={published:.9f} "
                    f"reconstructed={expected:.9f}",
                )


def check_row_internal(audit: Audit, row: Row) -> None:
    for name, value in (
        ("completed_pps_sequence", row.completed_pps_sequence),
        ("clockfaces.pps_sequence", row.clockface_pps_sequence),
        ("stats.last_pps_sequence", row.stats_last_pps_sequence),
    ):
        if value != row.sequence:
            audit.problem(
                "physical_identity",
                row,
                f"sequence={row.sequence} {name}={value}",
            )

    if row.update_count <= 0:
        audit.problem("stats_update_count", row, f"update_count={row.update_count}")

    expected_clock_gnss = row.instrument_age_seconds * NS_PER_SECOND
    if row.clock_gnss_ns != expected_clock_gnss:
        audit.problem(
            "instrument_gnss_age_identity",
            row,
            f"gnss_ns={row.clock_gnss_ns} instrument_age={row.instrument_age_seconds} "
            f"expected={expected_clock_gnss}",
        )

    # CLOCKS Alpha updates exactly one row per stats update.  The rolling
    # sequence can legitimately lag on an excluded endpoint, but never lead.
    if row.rolling_sequence > row.update_count:
        audit.problem(
            "rolling_sequence_leads_stats",
            row,
            f"rolling={row.rolling_sequence} update={row.update_count}",
        )

    for name, w in row.welfords.items():
        check_welford_geometry(audit, row, name, w)

    # Restore-authority ancestry invariant that exposed the historical poison.
    for lane in ("ocxo1", "ocxo2"):
        dac = row.welfords[f"{lane}_dac"]
        ocxo = row.welfords[lane]
        if dac.n > ocxo.n:
            audit.problem(
                "dac_ocxo_ancestry",
                row,
                f"{lane} dac.n={dac.n} > ocxo.n={ocxo.n}",
            )

    for lane in ("ocxo1", "ocxo2"):
        readback = row.dac_readback[lane]
        if readback is not None and readback != row.dac_hw[lane]:
            audit.problem(
                "dac_readback",
                row,
                f"{lane} hw={row.dac_hw[lane]} readback={readback}",
            )
        if not row.dither_enabled:
            rounded = int(math.floor(row.dac_target[lane] + 0.5))
            if row.dac_hw[lane] != rounded:
                audit.problem(
                    "dac_static_target",
                    row,
                    f"{lane} target={row.dac_target[lane]} "
                    f"hw={row.dac_hw[lane]} rounded={rounded}",
                )
        else:
            lo = int(math.floor(row.dac_target[lane]))
            hi = int(math.ceil(row.dac_target[lane]))
            if row.dac_hw[lane] not in {lo, hi}:
                audit.problem(
                    "dac_dither_target",
                    row,
                    f"{lane} target={row.dac_target[lane]} hw={row.dac_hw[lane]} "
                    f"expected one of {lo}/{hi}",
                )

    check_dwt_vclock_total(audit, row, "dwt")
    check_dwt_vclock_total(audit, row, "vclock")
    check_tau(audit, row, "ocxo1")
    check_tau(audit, row, "ocxo2")

    if row.gnss_raw_clockface_n is not None and row.gnss_raw_ref_ns is not None:
        expected_ref = row.gnss_raw_clockface_n * NS_PER_SECOND
        if row.gnss_raw_ref_ns != expected_ref:
            audit.problem(
                "gnss_raw_reference",
                row,
                f"clockface_n={row.gnss_raw_clockface_n} "
                f"ref_ns={row.gnss_raw_ref_ns} expected={expected_ref}",
            )

    if row.campaign_name is not None:
        if row.campaign_public_count is None:
            audit.problem("campaign_identity", row, "campaign has no public_count")
        else:
            cgnss = row.campaign_clockfaces.get("gnss_ns")
            expected = row.campaign_public_count * NS_PER_SECOND
            if cgnss is None or cgnss != expected:
                audit.problem(
                    "campaign_gnss_clockface",
                    row,
                    f"public_count={row.campaign_public_count} gnss_ns={cgnss} "
                    f"expected={expected}",
                )

        cfaces = row.campaign_clockfaces
        cgnss = cfaces.get("gnss_ns")
        if cgnss and cgnss > 0:
            expected_values: Dict[str, float] = {}
            dwt = cfaces.get("dwt_cycles")
            o1 = cfaces.get("ocxo1_ns")
            o2 = cfaces.get("ocxo2_ns")
            if dwt is not None:
                expected_values["dwt"] = (
                    float(dwt) / (float(cgnss) * 1.008) - 1.0
                ) * 1.0e9
            expected_values["vclock"] = 0.0
            if o1 is not None:
                expected_values["ocxo1"] = (
                    float(o1) / float(cgnss) - 1.0
                ) * 1.0e9
            if o2 is not None:
                expected_values["ocxo2"] = (
                    float(o2) / float(cgnss) - 1.0
                ) * 1.0e9
            for lane, expected_ppb in expected_values.items():
                published = row.campaign_ppb.get(lane)
                if published is not None and not close(
                    published,
                    expected_ppb,
                    abs_tol=PPB_TOLERANCE,
                    rel_tol=2.0e-12,
                ):
                    audit.problem(
                        "campaign_ppb",
                        row,
                        f"{lane} published={published:.9f} "
                        f"clockface={expected_ppb:.9f}",
                    )


def check_adjacent(audit: Audit, prev: Row, cur: Row) -> None:
    if cur.db_id <= prev.db_id:
        audit.problem(
            "db_chronology",
            cur,
            f"id {prev.db_id}->{cur.db_id}",
        )

    if prev.gnss_utc_s is not None and cur.gnss_utc_s is not None:
        elapsed = cur.gnss_utc_s - prev.gnss_utc_s
        if elapsed <= 0:
            audit.problem(
                "gnss_utc_chronology",
                cur,
                f"{prev.gnss_utc}->{cur.gnss_utc}",
            )
        elif elapsed > 1:
            audit.durable_gaps += 1
            audit.gnss_gap_seconds += elapsed - 1
            audit.event(
                "durable_gap",
                cur,
                f"elapsed={elapsed}s db_id={prev.db_id}->{cur.db_id}",
            )

        # Always-on logical clockfaces must carry elapsed GNSS time across both
        # Pi and Teensy recovery.  This is one of the strongest holistic checks.
        if elapsed > 0:
            expected_gnss_delta = elapsed * NS_PER_SECOND
            actual_gnss_delta = cur.clock_gnss_ns - prev.clock_gnss_ns
            if actual_gnss_delta != expected_gnss_delta:
                audit.problem(
                    "clockface_gnss_continuity",
                    cur,
                    f"elapsed={elapsed}s delta={actual_gnss_delta} "
                    f"expected={expected_gnss_delta}",
                )
            age_delta = cur.instrument_age_seconds - prev.instrument_age_seconds
            if age_delta != elapsed:
                audit.problem(
                    "instrument_age_continuity",
                    cur,
                    f"elapsed={elapsed}s age_delta={age_delta}",
                )

    seq_delta = cur.sequence - prev.sequence
    if seq_delta < 0:
        audit.physical_reboots += 1
        audit.event(
            "physical_sequence_rebase",
            cur,
            f"sequence {prev.sequence}->{cur.sequence}",
        )
    elif seq_delta == 0:
        audit.problem(
            "physical_sequence_repeat",
            cur,
            f"sequence={cur.sequence}",
        )
    elif seq_delta > 1:
        audit.event(
            "physical_sequence_gap",
            cur,
            f"sequence {prev.sequence}->{cur.sequence}",
        )

    if cur.reset_count < prev.reset_count:
        audit.problem(
            "stats_reset_regression",
            cur,
            f"reset_count {prev.reset_count}->{cur.reset_count}",
        )
    elif cur.reset_count > prev.reset_count:
        audit.stats_resets += 1
        audit.event(
            "stats_epoch",
            cur,
            f"reset_count {prev.reset_count}->{cur.reset_count} "
            f"update_count={cur.update_count}",
        )
        if cur.update_count != 1:
            audit.problem(
                "stats_epoch_birth",
                cur,
                f"new reset_count={cur.reset_count} begins update_count={cur.update_count}, expected 1",
            )
        if cur.rolling_sequence not in (0, 1):
            audit.problem(
                "rolling_epoch_birth",
                cur,
                f"new reset_count={cur.reset_count} rolling_sequence={cur.rolling_sequence}",
            )
    else:
        update_delta = cur.update_count - prev.update_count
        if update_delta <= 0:
            audit.problem(
                "stats_update_chronology",
                cur,
                f"reset_count={cur.reset_count} update_count "
                f"{prev.update_count}->{cur.update_count}",
            )
        elif seq_delta > 0 and update_delta != seq_delta:
            audit.problem(
                "stats_physical_chronology",
                cur,
                f"same Teensy lifetime sequence_delta={seq_delta} "
                f"update_delta={update_delta}",
            )
        elif seq_delta < 0 and update_delta > 1:
            audit.event(
                "stats_post_reboot_advancement",
                cur,
                f"physical sequence rebased while update_count advanced "
                f"{prev.update_count}->{cur.update_count}",
            )
        for name in WELFORD_LANES:
            known_sample: Optional[float] = None
            if name == "ocxo1_dac":
                known_sample = float(cur.dac_target["ocxo1"])
            elif name == "ocxo2_dac":
                known_sample = float(cur.dac_target["ocxo2"])
            check_welford_step(
                audit,
                cur,
                name,
                prev.welfords[name],
                cur.welfords[name],
                known_sample=known_sample,
            )
            if name.endswith("_dac") and cur.welfords[name].n == prev.welfords[name].n + 1:
                audit.exact_dac_steps += 1

    # Pi GNSS_RAW has its own lifetime.  A Teensy sequence rebase must not reset
    # it.  If the Welford advances exactly once, the current GNSS drift value is
    # the sample and closes the recurrence independently.
    if prev.gnss_raw_welford is not None and cur.gnss_raw_welford is not None:
        if (
            cur.gnss_raw_welford.n == prev.gnss_raw_welford.n + 1
            and cur.gnss_raw_drift_ppb is not None
        ):
            before = audit.exact_welford_steps
            check_welford_step(
                audit,
                cur,
                "gnss_raw",
                prev.gnss_raw_welford,
                cur.gnss_raw_welford,
                known_sample=cur.gnss_raw_drift_ppb,
            )
            if audit.exact_welford_steps > before:
                audit.exact_gnss_raw_steps += 1
        elif cur.gnss_raw_welford.n < prev.gnss_raw_welford.n:
            audit.problem(
                "gnss_raw_welford_regression",
                cur,
                f"n {prev.gnss_raw_welford.n}->{cur.gnss_raw_welford.n}",
            )

    if (
        prev.gnss_raw_clockface_n is not None
        and cur.gnss_raw_clockface_n is not None
        and cur.gnss_raw_clockface_n != prev.gnss_raw_clockface_n + 1
    ):
        audit.problem(
            "gnss_raw_clockface_chronology",
            cur,
            f"clockface_n {prev.gnss_raw_clockface_n}->{cur.gnss_raw_clockface_n}",
        )

    # Same campaign: public time is elapsed campaign time, so after recovery a
    # truthful forward splice must equal elapsed GNSS time, not necessarily +1.
    if (
        prev.campaign_name is not None
        and cur.campaign_name == prev.campaign_name
        and prev.campaign_public_count is not None
        and cur.campaign_public_count is not None
    ):
        public_delta = cur.campaign_public_count - prev.campaign_public_count
        elapsed = (
            cur.gnss_utc_s - prev.gnss_utc_s
            if prev.gnss_utc_s is not None and cur.gnss_utc_s is not None
            else None
        )
        if public_delta <= 0:
            audit.problem(
                "campaign_public_chronology",
                cur,
                f"{prev.campaign_public_count}->{cur.campaign_public_count}",
            )
        elif elapsed is not None and public_delta != elapsed:
            audit.problem(
                "campaign_public_elapsed",
                cur,
                f"public_delta={public_delta} elapsed_gnss={elapsed}",
            )
        elif public_delta > 1:
            audit.campaign_splices += 1
            audit.event(
                "campaign_forward_splice",
                cur,
                f"{prev.campaign_public_count}->{cur.campaign_public_count} "
                f"elapsed={elapsed}s",
            )

    if cur.superseded and not prev.superseded:
        audit.event("superseded_region", cur, "row marked holistic_restore_superseded")


def process_row(audit: Audit, prev: Optional[Row], row: Row) -> None:
    audit.rows += 1
    audit.first_db_id = row.db_id if audit.first_db_id is None else audit.first_db_id
    audit.last_db_id = row.db_id
    audit.first_ts = row.ts if not audit.first_ts else audit.first_ts
    audit.last_ts = row.ts
    audit.first_gnss = row.gnss_utc if not audit.first_gnss else audit.first_gnss
    audit.last_gnss = row.gnss_utc
    audit.epochs[row.reset_count] += 1
    if row.campaign_name:
        audit.campaigns[row.campaign_name] += 1
    if row.recovery_classification:
        audit.recovery_classes[row.recovery_classification] += 1

    check_row_internal(audit, row)
    if prev is not None:
        check_adjacent(audit, prev, row)
    check_buckets(audit, row)


def iter_rows(
    *,
    campaign: Optional[str],
    from_id: int,
    to_id: int,
    batch_size: int,
    pause_ms: int,
    limit: int,
) -> Iterator[Tuple[int, str, Dict[str, Any]]]:
    where = [
        "campaign_type = %s",
        "payload #>> '{schema}' = %s",
    ]
    params: list[Any] = [CAMPAIGN_TYPE, CLOCKS_SCHEMA]

    if campaign is not None:
        where.append("campaign = %s")
        params.append(campaign)
    if from_id > 0:
        where.append("id >= %s")
        params.append(from_id)
    if to_id > 0:
        where.append("id <= %s")
        params.append(to_id)

    sql = f"""
        SELECT id, ts, payload
        FROM campaign_detail
        WHERE {' AND '.join(where)}
        ORDER BY id ASC
    """
    if limit > 0:
        sql += " LIMIT %s"
        params.append(limit)

    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="campaign_analyzer_clocks_lineage")
        cur.itersize = max(1, batch_size)
        cur.execute(sql, tuple(params))

        in_batch = 0
        for dbrow in cur:
            payload = dbrow["payload"]
            if isinstance(payload, str):
                payload = json.loads(payload)
            if not isinstance(payload, dict):
                raise RuntimeError(f"campaign_detail id={dbrow['id']} payload is not an object")
            yield int(dbrow["id"]), str(dbrow["ts"]), payload
            in_batch += 1
            if in_batch >= max(1, batch_size):
                in_batch = 0
                if pause_ms:
                    time.sleep(pause_ms / 1000.0)


def scope_report() -> None:
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor()

        cur.execute(
            """
            SELECT
                count(*) AS rows,
                min(id) AS min_id,
                max(id) AS max_id,
                min(ts) AS first_ts,
                max(ts) AS last_ts,
                count(*) FILTER (WHERE campaign IS NULL) AS ambient_rows,
                count(*) FILTER (WHERE campaign IS NOT NULL) AS campaign_rows,
                count(*) FILTER (
                    WHERE COALESCE((payload #>> '{holistic_restore_superseded}')::boolean, false)
                ) AS superseded_rows
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{schema}' = %s
            """,
            (CAMPAIGN_TYPE, CLOCKS_SCHEMA),
        )
        totals = cur.fetchone() or {}

        cur.execute(
            """
            SELECT
                (payload #>> '{clocks,stats,reset_count}')::bigint AS reset_count,
                min(id) AS min_id,
                max(id) AS max_id,
                count(*) AS rows,
                min(ts) AS first_ts,
                max(ts) AS last_ts
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{schema}' = %s
              AND payload #>> '{clocks,stats,reset_count}' IS NOT NULL
            GROUP BY 1
            ORDER BY 1
            """,
            (CAMPAIGN_TYPE, CLOCKS_SCHEMA),
        )
        epochs = cur.fetchall()

        cur.execute(
            """
            SELECT campaign, count(*) AS rows, min(id) AS min_id, max(id) AS max_id,
                   max(ts) AS last_ts
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{schema}' = %s
              AND campaign IS NOT NULL
            GROUP BY campaign
            ORDER BY max(ts) DESC
            LIMIT 20
            """,
            (CAMPAIGN_TYPE, CLOCKS_SCHEMA),
        )
        campaigns = cur.fetchall()

    print("=" * 88)
    print("CLOCKS_V4 AUDIT SCOPE")
    print("=" * 88)
    print(
        f"Rows: {int(totals.get('rows') or 0):,}   "
        f"DB id: {totals.get('min_id')}..{totals.get('max_id')}   "
        f"ambient={int(totals.get('ambient_rows') or 0):,}   "
        f"campaign={int(totals.get('campaign_rows') or 0):,}   "
        f"superseded={int(totals.get('superseded_rows') or 0):,}"
    )
    print(f"Time: {totals.get('first_ts')} -> {totals.get('last_ts')}")

    print("\nSTATISTICAL EPOCHS")
    if not epochs:
        print("  none")
    for row in epochs:
        print(
            f"  reset={int(row['reset_count']):>4d} "
            f"rows={int(row['rows']):>9,d} "
            f"id={row['min_id']}..{row['max_id']} "
            f"{row['first_ts']} -> {row['last_ts']}"
        )

    print("\nRECENT CAMPAIGNS")
    if not campaigns:
        print("  none")
    for row in campaigns:
        print(
            f"  {str(row['campaign']):<24s} rows={int(row['rows']):>9,d} "
            f"id={row['min_id']}..{row['max_id']} last={row['last_ts']}"
        )

    print("\nUsage:")
    print("  campaign_analyzer.py --all")
    print("  campaign_analyzer.py CAMPAIGN")
    print("  campaign_analyzer.py --all --from-id N --to-id N")
    print("  Optional: --batch-size N --pause-ms N --limit N")


def print_report(audit: Audit) -> None:
    print("=" * 88)
    print(f"HOLISTIC CLOCKS AUDIT: {audit.scope}")
    print("=" * 88)
    if audit.rows == 0:
        print("No matching CLOCKS_V4 rows.")
        return

    print(f"Rows:              {audit.rows:,}")
    print(f"DB id:             {audit.first_db_id} -> {audit.last_db_id}")
    print(f"Database time:     {audit.first_ts} -> {audit.last_ts}")
    print(f"GNSS time:         {audit.first_gnss} -> {audit.last_gnss}")
    print(f"Physical rebases:  {audit.physical_reboots:,}")
    print(f"Durable gaps:      {audit.durable_gaps:,} ({audit.gnss_gap_seconds:,} missing second(s))")
    print(f"Stats epoch births:{audit.stats_resets:>10,d}")
    print(f"Campaign splices:  {audit.campaign_splices:,}")

    print("\nCOVERAGE")
    print(f"  Exact Welford one-sample proofs: {audit.exact_welford_steps:,}")
    print(f"  Exact DAC one-sample proofs:     {audit.exact_dac_steps:,}")
    print(f"  Exact GNSS_RAW proofs:           {audit.exact_gnss_raw_steps:,}")
    print(f"  OCXO TAU closure checks:         {audit.tau_checks:,}")
    print(f"  TOTAL PPB closure checks:        {audit.total_ppb_checks:,}")
    print(f"  Better-Buckets value checks:     {audit.bucket_checks:,}")
    print(
        "  Better-Buckets origin coverage: "
        + ("COMPLETE" if audit.better.complete_from_origin else "PARTIAL/UNKNOWN")
    )

    print("\nSTATISTICAL EPOCHS")
    for epoch, count in sorted(audit.epochs.items()):
        print(f"  reset_count={epoch:<6d} rows={count:,}")

    print("\nCAMPAIGNS OBSERVED")
    if audit.campaigns:
        for name, count in audit.campaigns.most_common():
            print(f"  {name:<24s} {count:,}")
    else:
        print("  none (ambient CLOCKS only)")

    print("\nRECOVERY / CUSTODY EVENTS")
    if audit.events:
        for name, count in sorted(audit.events.items()):
            print(f"  {name:<34s} {count:,}")
    else:
        print("  none")
    if audit.recovery_classes:
        print("  recovery classifications:")
        for name, count in audit.recovery_classes.most_common():
            print(f"    {name:<30s} {count:,}")

    print("\nSTRICT FINDINGS")
    total = sum(audit.problems.values())
    if not audit.problems:
        print("  NONE")
    else:
        for name, count in sorted(audit.problems.items()):
            print(f"  {name:<38s} {count:,}")
    print(f"  {'TOTAL':<38s} {total:,}")

    if audit.examples:
        print("\nBOUNDED EVIDENCE")
        for kind, examples in audit.examples.items():
            print(f"  [{kind}]")
            for example in examples:
                print(f"    {example}")

    print("\nVERDICT")
    if total:
        print(
            f"  INVESTIGATE — {total:,} strict continuity/closure finding(s). "
            "Recovery is not accepted as exact."
        )
    elif not audit.better.complete_from_origin:
        print(
            "  STRUCTURALLY CLEAN, BUT BETTER-BUCKETS ORIGIN NOT IN SCOPE — "
            "rerun from the beginning of the statistical epoch before declaring "
            "holistic restore exact."
        )
    else:
        print(
            "  CLEAN — all continuity and sufficient-state courts that are "
            "provable from the selected durable CLOCKS_V4 scope closed exactly."
        )


@dataclass(frozen=True)
class Args:
    campaign: Optional[str]
    all_rows: bool
    from_id: int
    to_id: int
    batch_size: int
    pause_ms: int
    limit: int


def parse(argv: Sequence[str]) -> Args:
    campaign: Optional[str] = None
    all_rows = False
    from_id = 0
    to_id = 0
    batch_size = DEFAULT_BATCH_SIZE
    pause_ms = DEFAULT_PAUSE_MS
    limit = 0

    idx = 1
    while idx < len(argv):
        arg = argv[idx]
        if arg == "--all":
            all_rows = True
            idx += 1
            continue
        if arg in {"--from-id", "--to-id", "--batch-size", "--pause-ms", "--limit"}:
            if idx + 1 >= len(argv):
                raise SystemExit(f"{arg} requires a value")
            value = int(argv[idx + 1])
            idx += 2
            if arg == "--from-id":
                from_id = value
            elif arg == "--to-id":
                to_id = value
            elif arg == "--batch-size":
                batch_size = value
            elif arg == "--pause-ms":
                pause_ms = value
            else:
                limit = value
            continue
        if arg.startswith("--") and "=" in arg:
            key, raw = arg.split("=", 1)
            value = int(raw)
            if key == "--from-id":
                from_id = value
            elif key == "--to-id":
                to_id = value
            elif key == "--batch-size":
                batch_size = value
            elif key == "--pause-ms":
                pause_ms = value
            elif key == "--limit":
                limit = value
            else:
                raise SystemExit(f"unknown option {key}")
            idx += 1
            continue
        if arg.startswith("--"):
            raise SystemExit(f"unknown option {arg}")
        if campaign is not None:
            raise SystemExit("only one campaign may be specified")
        campaign = arg
        idx += 1

    if campaign is not None and all_rows:
        raise SystemExit("choose either a campaign or --all")
    if batch_size <= 0 or pause_ms < 0 or limit < 0 or from_id < 0 or to_id < 0:
        raise SystemExit("invalid negative/zero option")
    if from_id and to_id and from_id > to_id:
        raise SystemExit("--from-id may not exceed --to-id")

    return Args(campaign, all_rows, from_id, to_id, batch_size, pause_ms, limit)


def main(argv: Sequence[str]) -> None:
    args = parse(argv)
    if args.campaign is None and not args.all_rows:
        scope_report()
        return

    scope = args.campaign if args.campaign is not None else "ALL CLOCKS_V4"
    audit = Audit(scope=scope)
    previous: Optional[Row] = None

    for db_id, ts, payload in iter_rows(
        campaign=args.campaign,
        from_id=args.from_id,
        to_id=args.to_id,
        batch_size=args.batch_size,
        pause_ms=args.pause_ms,
        limit=args.limit,
    ):
        try:
            row = Row.parse(audit, db_id, ts, payload)
        except Exception as exc:
            # The analyzer itself must fail loud on malformed canonical testimony.
            raise RuntimeError(f"cannot decode canonical CLOCKS_V4 db_id={db_id}: {exc}") from exc
        process_row(audit, previous, row)
        previous = row

    print_report(audit)


if __name__ == "__main__":
    main(sys.argv)
