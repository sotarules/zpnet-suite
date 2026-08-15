"""ZPNet campaign_analyzer — proof-ledger audit for canonical CLOCKS_V4.

This analyzer treats durable CLOCKS_V4 as scientific testimony, not as material
from which missing instrument history may be reconstructed.  The ownership map
is explicit:

  * Teensy Alpha owns physical CLOCKS observations and firmware statistics.
  * Pi CLOCKS owns DAC/GNSS_RAW custody, recovery orchestration, and persistence.
  * PostgreSQL preserves testimony; it is never a substitute author of Alpha state.
  * TEMPEST is a campaign-relative view layered on the always-on CLOCKS instrument.

The post-recovery CLOCKS format is intentionally self-proving.  Every durable row
carries producer-authored Better-Buckets checkpoint/proof testimony, while Pi may
also carry a literal bounded Better-Buckets resurrection image.  The analyzer
therefore validates those witnesses directly.  Relational columns and partial
indexes are navigation aids only; payload testimony remains authoritative.
Database replay is used only as an independent adjacent-row cross-check when the
required raw observations are actually present; gaps remain gaps.

The final verdict is a proof ledger rather than a single continuity heuristic.
Physical lifetime, Alpha statistics, Better-Buckets science, resurrection custody,
Pi-owned state, TEMPEST chronology, and each recovery episode are adjudicated
independently and then rolled into one holistic verdict.
"""

from __future__ import annotations

import json
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


# -----------------------------------------------------------------------------
# Contract / tolerances
# -----------------------------------------------------------------------------

CLOCKS_SCHEMA = "CLOCKS_V4"
CLOCKS_INSTRUMENT_SCHEMA = "CLOCKS_INSTRUMENT_STATE_V1"
TEMPEST_SCHEMA = "TEMPEST_FRAGMENT_V1"
CAMPAIGN_TYPE = "TEMPEST"

FW_PPB_SCHEMA = "CLOCKS_PPB_CHECKPOINT_DELTA_V1"
PI_PPB_SCHEMA = "PI_CLOCKS_PPB_RESTORE_CHECKPOINT_V1"
RECOVERY_CUSTODY_SCHEMA = "PI_CLOCKS_RECOVERY_CUSTODY_V1"

NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_PPS = 1_008_000_000

PPB_10_MIN_SECONDS = 10 * 60
PPB_60_MIN_SECONDS = 60 * 60
PPB_8_HOUR_SECONDS = 8 * 60 * 60
PPB_24_HOUR_SECONDS = 24 * 60 * 60
PPB_SECOND_CAPACITY = PPB_10_MIN_SECONDS + 1
PPB_MINUTE_CAPACITY = PPB_24_HOUR_SECONDS // 60 + 2

PPB_WINDOWS: Tuple[Tuple[str, int], ...] = (
    ("10_min", PPB_10_MIN_SECONDS),
    ("60_min", PPB_60_MIN_SECONDS),
    ("8_hour", PPB_8_HOUR_SECONDS),
    ("24_hour", PPB_24_HOUR_SECONDS),
)
PPB_LANES = ("dwt", "vclock", "ocxo1", "ocxo2")
FIRMWARE_WELFORD_LANES = ("gnss", "dwt", "vclock", "ocxo1", "ocxo2", "pps_witness")
PI_WELFORD_LANES = ("ocxo1_dac", "ocxo2_dac")
ALL_WELFORD_LANES = FIRMWARE_WELFORD_LANES + PI_WELFORD_LANES

DEFAULT_BATCH_SIZE = 32
DEFAULT_PAUSE_MS = 0
MAX_EXAMPLES = 18

PPB_PROOF_TOLERANCE = 2.0e-6
DISPLAY_PPB_TOLERANCE = 6.0e-4
TAU_TOLERANCE = 2.0e-12
WELFORD_FLOAT_ABS_TOL = 2.0e-8
WELFORD_FLOAT_REL_TOL = 2.0e-12
WELFORD_SERIALIZED_MEAN_QUANTUM = 1.0e-6
DAC_SAMPLE_TOL = 1.0e-9


# -----------------------------------------------------------------------------
# Generic helpers
# -----------------------------------------------------------------------------


def d(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def req_dict(value: Any, where: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{where} must be an object")
    return value


def req_list(value: Any, where: str) -> List[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{where} must be an array")
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


def minute_key(rolling_sequence: int) -> int:
    if rolling_sequence <= 0:
        return 0
    return ((rolling_sequence - 1) // 60) + 1


def expected_ring_counts(current_sequence: int) -> Tuple[int, int]:
    if current_sequence <= 0:
        return 0, 0
    second_count = min(current_sequence, PPB_SECOND_CAPACITY)
    minute_count = min(minute_key(current_sequence), PPB_MINUTE_CAPACITY)
    return second_count, minute_count


# -----------------------------------------------------------------------------
# Scientific sufficient-state models
# -----------------------------------------------------------------------------


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
        out = cls(
            n=req_int(src.get("n"), f"{where}.n", minimum=0),
            mean=req_float(src.get("mean"), f"{where}.mean"),
            m2=req_float(src.get("m2"), f"{where}.m2"),
            min_value=req_float(src.get("min"), f"{where}.min"),
            max_value=req_float(src.get("max"), f"{where}.max"),
            stddev=opt_float(src.get("stddev")),
            stderr=opt_float(src.get("stderr")),
        )
        if out.n == 0:
            if any(v != 0.0 for v in (out.mean, out.m2, out.min_value, out.max_value)):
                raise ValueError(f"{where}: empty Welford publishes non-zero sufficient state")
        else:
            if out.m2 < -WELFORD_FLOAT_ABS_TOL:
                raise ValueError(f"{where}: negative m2={out.m2}")
            if out.min_value > out.max_value:
                raise ValueError(f"{where}: min={out.min_value} > max={out.max_value}")
            if (
                out.mean < out.min_value - WELFORD_FLOAT_ABS_TOL
                or out.mean > out.max_value + WELFORD_FLOAT_ABS_TOL
            ):
                raise ValueError(
                    f"{where}: mean={out.mean} outside [{out.min_value}, {out.max_value}]"
                )
        return out

    def sufficient_equal(self, other: "Welford") -> bool:
        return bool(
            self.n == other.n
            and close(self.mean, other.mean, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL)
            and close(
                self.m2,
                other.m2,
                abs_tol=max(WELFORD_FLOAT_ABS_TOL, abs(self.m2) * 2.0e-12),
                rel_tol=2.0e-12,
            )
            and close(
                self.min_value,
                other.min_value,
                abs_tol=WELFORD_FLOAT_ABS_TOL,
                rel_tol=WELFORD_FLOAT_REL_TOL,
            )
            and close(
                self.max_value,
                other.max_value,
                abs_tol=WELFORD_FLOAT_ABS_TOL,
                rel_tol=WELFORD_FLOAT_REL_TOL,
            )
        )


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
                src.get("cumulative_reference_ns"), f"{where}.cumulative_reference_ns", minimum=0
            ),
            cumulative_clock_ns=req_int(
                src.get("cumulative_clock_ns"), f"{where}.cumulative_clock_ns", minimum=0
            ),
            cumulative_clock_ns_exact=req_float(
                src.get("cumulative_clock_ns_exact"), f"{where}.cumulative_clock_ns_exact"
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

    @classmethod
    def parse(cls, obj: Any, where: str) -> "BetterEndpoint":
        src = req_dict(obj, where)
        return cls(
            rolling_sequence=req_int(
                src.get("rolling_sequence"), f"{where}.rolling_sequence", minimum=0
            ),
            reference_ns=req_int(src.get("reference_ns"), f"{where}.reference_ns", minimum=0),
            dwt_error_cycles=req_float(
                src.get("dwt_error_cycles"), f"{where}.dwt_error_cycles"
            ),
            ocxo1_error_ns=req_int(src.get("ocxo1_error_ns"), f"{where}.ocxo1_error_ns"),
            ocxo2_error_ns=req_int(src.get("ocxo2_error_ns"), f"{where}.ocxo2_error_ns"),
            interval_count=req_int(
                src.get("interval_count"), f"{where}.interval_count", minimum=0
            ),
        )

    @classmethod
    def zero(cls) -> "BetterEndpoint":
        return cls(0, 0, 0.0, 0, 0, 0)

    def equal(self, other: "BetterEndpoint") -> bool:
        return bool(
            self.rolling_sequence == other.rolling_sequence
            and self.reference_ns == other.reference_ns
            and close(self.dwt_error_cycles, other.dwt_error_cycles, abs_tol=1.0e-9, rel_tol=1.0e-15)
            and self.ocxo1_error_ns == other.ocxo1_error_ns
            and self.ocxo2_error_ns == other.ocxo2_error_ns
            and self.interval_count == other.interval_count
        )

    def monotonic_after(self, other: "BetterEndpoint") -> bool:
        return bool(
            self.rolling_sequence >= other.rolling_sequence
            and self.reference_ns >= other.reference_ns
            and self.interval_count >= other.interval_count
        )


def ppb_from_endpoints(current: BetterEndpoint, anchor: BetterEndpoint, lane: str) -> float:
    reference_ns = current.reference_ns - anchor.reference_ns
    if reference_ns <= 0:
        raise ValueError("Better-Buckets proof has non-positive reference interval")
    if lane == "dwt":
        expected_cycles = float(reference_ns) * float(DWT_EXPECTED_PER_PPS) / float(NS_PER_SECOND)
        error = current.dwt_error_cycles - anchor.dwt_error_cycles
        return error * 1.0e9 / expected_cycles
    if lane == "vclock":
        return 0.0
    if lane == "ocxo1":
        return float(current.ocxo1_error_ns - anchor.ocxo1_error_ns) * 1.0e9 / float(reference_ns)
    if lane == "ocxo2":
        return float(current.ocxo2_error_ns - anchor.ocxo2_error_ns) * 1.0e9 / float(reference_ns)
    raise ValueError(f"unknown Better-Buckets lane {lane!r}")


@dataclass(frozen=True)
class FirmwareWindowProof:
    valid: bool
    sample_count: int
    anchor: Optional[BetterEndpoint]


@dataclass(frozen=True)
class FirmwareCheckpoint:
    valid: bool
    reset_count: int
    update_count: int
    rolling_sequence: int
    second_count: int
    minute_count: int
    last_minute_key: int
    origin_valid: bool
    origin: BetterEndpoint
    current: BetterEndpoint
    second_append: Optional[BetterEndpoint]
    minute_append: Optional[BetterEndpoint]
    proof: Dict[str, FirmwareWindowProof]

    @classmethod
    def parse(cls, stats: Dict[str, Any], where: str) -> "FirmwareCheckpoint":
        raw = req_dict(stats.get("rolling_ppb_checkpoint"), f"{where}.rolling_ppb_checkpoint")
        if raw.get("schema") != FW_PPB_SCHEMA:
            raise ValueError(
                f"{where}.rolling_ppb_checkpoint schema={raw.get('schema')!r}, expected {FW_PPB_SCHEMA}"
            )
        if raw.get("valid") is not True:
            raise ValueError(f"{where}.rolling_ppb_checkpoint is not valid")

        reset_count = req_int(stats.get("reset_count"), f"{where}.reset_count", minimum=0)
        update_count = req_int(stats.get("update_count"), f"{where}.update_count", minimum=0)
        stats_current = req_int(
            stats.get("rolling_ppb_current_sequence"),
            f"{where}.rolling_ppb_current_sequence",
            minimum=0,
        )
        rolling_sequence = req_int(
            raw.get("rolling_sequence"), f"{where}.rolling_ppb_checkpoint.rolling_sequence", minimum=0
        )
        if rolling_sequence != stats_current:
            raise ValueError(
                f"{where}: firmware checkpoint rolling_sequence={rolling_sequence} "
                f"!= stats current={stats_current}"
            )
        if rolling_sequence > update_count:
            raise ValueError(f"{where}: firmware checkpoint leads stats update_count")

        second_count = req_int(
            raw.get("second_count"), f"{where}.rolling_ppb_checkpoint.second_count", minimum=0
        )
        minute_count = req_int(
            raw.get("minute_count"), f"{where}.rolling_ppb_checkpoint.minute_count", minimum=0
        )
        if second_count > PPB_SECOND_CAPACITY:
            raise ValueError(f"{where}: firmware second_count exceeds capacity")
        if minute_count > PPB_MINUTE_CAPACITY:
            raise ValueError(f"{where}: firmware minute_count exceeds capacity")
        last_minute_key = req_int(
            raw.get("last_minute_key"), f"{where}.rolling_ppb_checkpoint.last_minute_key", minimum=0
        )

        current = BetterEndpoint.parse(raw.get("current"), f"{where}.rolling_ppb_checkpoint.current")
        if current.rolling_sequence != rolling_sequence:
            raise ValueError(f"{where}: firmware current endpoint identity mismatch")
        origin_valid = bool(raw.get("origin_valid") is True)
        origin = (
            BetterEndpoint.parse(raw.get("origin"), f"{where}.rolling_ppb_checkpoint.origin")
            if origin_valid
            else BetterEndpoint.zero()
        )
        if origin_valid and not current.monotonic_after(origin):
            raise ValueError(f"{where}: firmware origin follows current")

        def parse_append(name: str) -> Optional[BetterEndpoint]:
            node = d(raw.get(name))
            if node.get("valid") is not True:
                return None
            return BetterEndpoint.parse(node.get("endpoint"), f"{where}.rolling_ppb_checkpoint.{name}.endpoint")

        second_append = parse_append("second_append")
        minute_append = parse_append("minute_append")
        endpoint_admitted = bool(stats.get("rolling_ppb_endpoint_admitted") is True)
        interval_advanced = bool(stats.get("rolling_ppb_interval_advanced") is True)
        if (second_append is not None) != endpoint_admitted:
            raise ValueError(f"{where}: second_append disagrees with endpoint admission")
        if interval_advanced and not endpoint_admitted:
            raise ValueError(f"{where}: interval advanced on excluded endpoint")
        if second_append is not None and not second_append.equal(current):
            raise ValueError(f"{where}: second_append is not current endpoint")
        if minute_append is not None and second_append is None:
            raise ValueError(f"{where}: minute_append without second_append")
        if minute_append is not None and not minute_append.equal(current):
            raise ValueError(f"{where}: minute_append is not current endpoint")

        if rolling_sequence > 0:
            if not origin_valid or second_count <= 0:
                raise ValueError(f"{where}: nonempty firmware checkpoint lacks origin/ring")
            if minute_count > 0 and last_minute_key != minute_key(rolling_sequence):
                raise ValueError(f"{where}: firmware minute key does not match rolling sequence")
        elif second_count or minute_count or origin_valid:
            raise ValueError(f"{where}: empty firmware checkpoint has nonempty state")

        proof: Dict[str, FirmwareWindowProof] = {}
        for key, _seconds in PPB_WINDOWS:
            node = req_dict(raw.get(key), f"{where}.rolling_ppb_checkpoint.{key}")
            valid = bool(node.get("valid") is True)
            sample_count = req_int(
                node.get("sample_count"), f"{where}.rolling_ppb_checkpoint.{key}.sample_count", minimum=0
            )
            anchor: Optional[BetterEndpoint] = None
            if valid:
                if sample_count <= 0:
                    raise ValueError(f"{where}: {key} valid proof has zero sample_count")
                anchor = BetterEndpoint.parse(
                    node.get("anchor"), f"{where}.rolling_ppb_checkpoint.{key}.anchor"
                )
                expected_samples = current.interval_count - anchor.interval_count
                if sample_count != expected_samples:
                    raise ValueError(
                        f"{where}: {key} sample_count={sample_count} endpoint_delta={expected_samples}"
                    )
            proof[key] = FirmwareWindowProof(valid, sample_count, anchor)

        return cls(
            True,
            reset_count,
            update_count,
            rolling_sequence,
            second_count,
            minute_count,
            last_minute_key,
            origin_valid,
            origin,
            current,
            second_append,
            minute_append,
            proof,
        )


@dataclass(frozen=True)
class PiCheckpoint:
    valid: bool
    recoverable: bool
    status: str
    status_reason: str
    reset_count: int
    update_count: int
    rolling_sequence: int
    current_sequence: int
    last_minute_key: int
    origin_valid: bool
    origin: BetterEndpoint
    current: BetterEndpoint
    expected_second_count: int
    expected_minute_count: int
    second_history: Tuple[BetterEndpoint, ...]
    minute_history: Tuple[BetterEndpoint, ...]
    contiguous_from_update_count: Optional[int]
    gap_count: int
    last_gap: Dict[str, Any]
    seeded_from_durable: bool
    seed_source_db_detail_id: Optional[int]
    proof_checks: int

    @classmethod
    def parse_optional(cls, obj: Any, where: str) -> Optional["PiCheckpoint"]:
        if obj is None:
            return None
        raw = req_dict(obj, where)
        if raw.get("schema") != PI_PPB_SCHEMA:
            raise ValueError(f"{where} schema={raw.get('schema')!r}, expected {PI_PPB_SCHEMA}")
        if raw.get("valid") is not True:
            raise ValueError(f"{where} is not valid")

        reset_count = req_int(raw.get("reset_count"), f"{where}.reset_count", minimum=0)
        update_count = req_int(raw.get("update_count"), f"{where}.update_count", minimum=0)
        rolling_sequence = req_int(raw.get("rolling_sequence"), f"{where}.rolling_sequence", minimum=0)
        current_sequence = req_int(raw.get("current_sequence"), f"{where}.current_sequence", minimum=0)
        if rolling_sequence != update_count:
            raise ValueError(f"{where}: restore rolling_sequence must equal update_count")
        if current_sequence > rolling_sequence:
            raise ValueError(f"{where}: current_sequence leads restore identity")

        expected_second = req_int(
            raw.get("expected_second_count"), f"{where}.expected_second_count", minimum=0
        )
        expected_minute = req_int(
            raw.get("expected_minute_count"), f"{where}.expected_minute_count", minimum=0
        )
        if expected_second > PPB_SECOND_CAPACITY or expected_minute > PPB_MINUTE_CAPACITY:
            raise ValueError(f"{where}: expected ring count exceeds capacity")

        last_minute_key = req_int(raw.get("last_minute_key"), f"{where}.last_minute_key", minimum=0)
        current = BetterEndpoint.parse(raw.get("current"), f"{where}.current")
        if current.rolling_sequence != current_sequence:
            raise ValueError(f"{where}: current endpoint identity mismatch")
        origin_valid = bool(raw.get("origin_valid") is True)
        origin = BetterEndpoint.parse(raw.get("origin"), f"{where}.origin") if origin_valid else BetterEndpoint.zero()
        if origin_valid and not current.monotonic_after(origin):
            raise ValueError(f"{where}: origin follows current")

        def parse_history(name: str, capacity: int) -> Tuple[BetterEndpoint, ...]:
            arr = req_list(raw.get(name), f"{where}.{name}")
            if len(arr) > capacity:
                raise ValueError(f"{where}.{name} exceeds capacity")
            out: List[BetterEndpoint] = []
            prev: Optional[BetterEndpoint] = None
            for idx, item in enumerate(arr):
                ep = BetterEndpoint.parse(item, f"{where}.{name}[{idx}]")
                if prev is not None:
                    if ep.rolling_sequence <= prev.rolling_sequence:
                        raise ValueError(f"{where}.{name} is not chronological")
                    if not ep.monotonic_after(prev):
                        raise ValueError(f"{where}.{name} cumulative state regresses")
                if ep.rolling_sequence > current_sequence:
                    raise ValueError(f"{where}.{name} extends past current")
                out.append(ep)
                prev = ep
            return tuple(out)

        seconds = parse_history("second_history", PPB_SECOND_CAPACITY)
        minutes = parse_history("minute_history", PPB_MINUTE_CAPACITY)
        if len(seconds) > expected_second or len(minutes) > expected_minute:
            raise ValueError(f"{where}: literal history exceeds producer ring counts")
        if current_sequence > 0 and seconds and not seconds[-1].equal(current):
            raise ValueError(f"{where}: second history tail is not current")

        computed_recoverable = bool(
            len(seconds) == expected_second
            and len(minutes) == expected_minute
            and (
                (
                    current_sequence == 0
                    and expected_second == 0
                    and expected_minute == 0
                    and not origin_valid
                )
                or (
                    current_sequence > 0
                    and origin_valid
                    and bool(seconds)
                    and seconds[-1].equal(current)
                    and last_minute_key == minute_key(current_sequence)
                    and (not minutes or minute_key(minutes[-1].rolling_sequence) == last_minute_key)
                )
            )
        )
        published_recoverable = bool(raw.get("recoverable") is True)
        if published_recoverable != computed_recoverable:
            raise ValueError(f"{where}: recoverable flag disagrees with literal history")

        return cls(
            valid=True,
            recoverable=computed_recoverable,
            status=str(raw.get("status") or ""),
            status_reason=str(raw.get("status_reason") or ""),
            reset_count=reset_count,
            update_count=update_count,
            rolling_sequence=rolling_sequence,
            current_sequence=current_sequence,
            last_minute_key=last_minute_key,
            origin_valid=origin_valid,
            origin=origin,
            current=current,
            expected_second_count=expected_second,
            expected_minute_count=expected_minute,
            second_history=seconds,
            minute_history=minutes,
            contiguous_from_update_count=opt_int(raw.get("contiguous_from_update_count")),
            gap_count=req_int(raw.get("gap_count") or 0, f"{where}.gap_count", minimum=0),
            last_gap=d(raw.get("last_gap")),
            seeded_from_durable=bool(raw.get("seeded_from_durable") is True),
            seed_source_db_detail_id=opt_int(raw.get("seed_source_db_detail_id")),
            proof_checks=req_int(raw.get("proof_checks") or 0, f"{where}.proof_checks", minimum=0),
        )


@dataclass(frozen=True)
class RecoveryCustody:
    generation: str
    entered_at_utc: str
    reason: str
    classification: str
    restore_authority: bool
    physical_sequence_regression: bool
    classified_at_utc: str
    regression_witness: Dict[str, Any]

    @classmethod
    def parse_optional(cls, obj: Any, where: str) -> Optional["RecoveryCustody"]:
        if obj is None:
            return None
        raw = req_dict(obj, where)
        if raw.get("schema") != RECOVERY_CUSTODY_SCHEMA:
            raise ValueError(f"{where} schema={raw.get('schema')!r}, expected {RECOVERY_CUSTODY_SCHEMA}")
        generation = str(raw.get("generation") or "").strip()
        if not generation:
            raise ValueError(f"{where}.generation missing")
        return cls(
            generation=generation,
            entered_at_utc=str(raw.get("entered_at_utc") or ""),
            reason=str(raw.get("reason") or ""),
            classification=str(raw.get("classification") or "").strip(),
            restore_authority=bool(raw.get("restore_authority") is True),
            physical_sequence_regression=bool(raw.get("physical_sequence_regression") is True),
            classified_at_utc=str(raw.get("classified_at_utc") or ""),
            regression_witness=d(raw.get("regression_witness")),
        )


# -----------------------------------------------------------------------------
# Canonical row model
# -----------------------------------------------------------------------------


@dataclass
class Row:
    db_id: int
    ts: str
    payload: Dict[str, Any]

    sequence: int
    completed_pps_sequence: int
    clockface_pps_sequence: int
    stats_last_pps_sequence: int
    completed_row_coherent: bool
    snapshot_ok: Optional[bool]
    instrument_age_seconds: int
    gnss_utc: str
    gnss_utc_s: Optional[int]

    clock_gnss_ns: int
    clock_dwt_cycles: int
    clock_ocxo1_ns: int
    clock_ocxo2_ns: int

    reset_count: int
    update_count: int
    rolling_sequence: int
    endpoint_admitted: bool
    interval_advanced: bool

    welfords: Dict[str, Welford]
    tau: Dict[str, TauState]
    ppb: Dict[str, float]
    frequency_tau: Dict[str, Optional[float]]
    buckets: Dict[str, Dict[str, float]]

    firmware_checkpoint: FirmwareCheckpoint
    pi_checkpoint: Optional[PiCheckpoint]

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
    campaign_control_eligible: Optional[bool]
    campaign_status: str

    recovery: Optional[RecoveryCustody]
    superseded: bool

    @classmethod
    def parse(cls, db_id: int, ts: str, payload: Dict[str, Any]) -> "Row":
        root = payload
        if root.get("schema") != CLOCKS_SCHEMA:
            inner = d(root.get("payload"))
            if inner.get("schema") == CLOCKS_SCHEMA:
                root = inner
        if root.get("schema") != CLOCKS_SCHEMA:
            raise ValueError(f"db_id={db_id}: payload is not {CLOCKS_SCHEMA}")

        clocks = req_dict(root.get("clocks"), f"db_id={db_id}.clocks")
        if clocks.get("schema") != CLOCKS_INSTRUMENT_SCHEMA:
            raise ValueError(
                f"db_id={db_id}: clocks schema={clocks.get('schema')!r}, expected {CLOCKS_INSTRUMENT_SCHEMA}"
            )
        stats = req_dict(clocks.get("stats"), f"db_id={db_id}.clocks.stats")
        clockfaces = req_dict(clocks.get("clockfaces"), f"db_id={db_id}.clocks.clockfaces")
        control = req_dict(clocks.get("control"), f"db_id={db_id}.clocks.control")

        welfords: Dict[str, Welford] = {}
        for lane in ("gnss", "dwt", "vclock", "ocxo1", "ocxo2"):
            welfords[lane] = Welford.parse(
                path(stats, f"{lane}.welford"), f"db_id={db_id}.stats.{lane}.welford"
            )
        aux = req_dict(stats.get("auxiliary_welford"), f"db_id={db_id}.stats.auxiliary_welford")
        for lane in ("pps_witness", "ocxo1_dac", "ocxo2_dac"):
            welfords[lane] = Welford.parse(
                aux.get(lane), f"db_id={db_id}.stats.auxiliary_welford.{lane}"
            )

        tau = {
            "ocxo1": TauState.parse(stats.get("ocxo1_tau_state"), f"db_id={db_id}.stats.ocxo1_tau_state"),
            "ocxo2": TauState.parse(stats.get("ocxo2_tau_state"), f"db_id={db_id}.stats.ocxo2_tau_state"),
        }

        ppb: Dict[str, float] = {}
        frequency_tau: Dict[str, Optional[float]] = {}
        buckets: Dict[str, Dict[str, float]] = {}
        for lane in PPB_LANES:
            lane_obj = req_dict(stats.get(lane), f"db_id={db_id}.stats.{lane}")
            ppb[lane] = req_float(lane_obj.get("ppb"), f"db_id={db_id}.stats.{lane}.ppb")
            frequency_tau[lane] = opt_float(lane_obj.get("tau"))
            bucket_obj = req_dict(lane_obj.get("ppb_buckets"), f"db_id={db_id}.stats.{lane}.ppb_buckets")
            buckets[lane] = {
                key: value
                for key in ("10_min", "60_min", "8_hour", "24_hour", "total")
                if (value := opt_float(bucket_obj.get(key))) is not None
            }

        dac_hw: Dict[str, int] = {}
        dac_readback: Dict[str, Optional[int]] = {}
        dac_target: Dict[str, float] = {}
        for lane in ("ocxo1", "ocxo2"):
            lane_obj = req_dict(control.get(lane), f"db_id={db_id}.control.{lane}")
            dac_hw[lane] = req_int(lane_obj.get("hw_code"), f"db_id={db_id}.control.{lane}.hw_code", minimum=0)
            dac_readback[lane] = opt_int(lane_obj.get("readback_code"))
            dac_target[lane] = req_float(lane_obj.get("target_code"), f"db_id={db_id}.control.{lane}.target_code")

        gnss_raw = d(clocks.get("gnss_raw"))
        gnss_raw_welford = (
            Welford.parse(gnss_raw.get("welford"), f"db_id={db_id}.clocks.gnss_raw.welford")
            if gnss_raw
            else None
        )
        gnss_raw_instr = d(gnss_raw.get("instrument"))

        campaign_obj = d(root.get("campaign"))
        campaign_name: Optional[str] = None
        campaign_public_count: Optional[int] = None
        campaign_clockfaces: Dict[str, int] = {}
        campaign_ppb: Dict[str, float] = {}
        campaign_science_eligible: Optional[bool] = None
        campaign_control_eligible: Optional[bool] = None
        campaign_status = ""
        if campaign_obj:
            if campaign_obj.get("schema") != TEMPEST_SCHEMA:
                raise ValueError(
                    f"db_id={db_id}: campaign schema={campaign_obj.get('schema')!r}, expected {TEMPEST_SCHEMA}"
                )
            campaign_name = str(campaign_obj.get("name") or "").strip() or None
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
            disposition = d(campaign_obj.get("disposition"))
            campaign_science_eligible = opt_bool(disposition.get("science_eligible"))
            campaign_control_eligible = opt_bool(disposition.get("control_eligible"))
            campaign_status = str(disposition.get("status") or "")

        firmware_checkpoint = FirmwareCheckpoint.parse(stats, f"db_id={db_id}.stats")
        pi_checkpoint = PiCheckpoint.parse_optional(
            clocks.get("ppb_restore_checkpoint"), f"db_id={db_id}.clocks.ppb_restore_checkpoint"
        )
        recovery = RecoveryCustody.parse_optional(root.get("recovery_custody"), f"db_id={db_id}.recovery_custody")

        return cls(
            db_id=db_id,
            ts=ts,
            payload=root,
            sequence=req_int(root.get("sequence"), f"db_id={db_id}.sequence", minimum=1),
            completed_pps_sequence=req_int(
                clocks.get("completed_pps_sequence"), f"db_id={db_id}.clocks.completed_pps_sequence", minimum=1
            ),
            clockface_pps_sequence=req_int(
                clockfaces.get("pps_sequence"), f"db_id={db_id}.clocks.clockfaces.pps_sequence", minimum=1
            ),
            stats_last_pps_sequence=req_int(
                stats.get("last_pps_sequence"), f"db_id={db_id}.stats.last_pps_sequence", minimum=1
            ),
            completed_row_coherent=bool(clocks.get("completed_row_coherent") is True),
            snapshot_ok=opt_bool(clocks.get("snapshot_ok")),
            instrument_age_seconds=req_int(
                clocks.get("instrument_age_seconds"), f"db_id={db_id}.clocks.instrument_age_seconds", minimum=0
            ),
            gnss_utc=str(clocks.get("gnss_time_utc") or ""),
            gnss_utc_s=utc_seconds(str(clocks.get("gnss_time_utc") or "")),
            clock_gnss_ns=req_int(clockfaces.get("gnss_ns"), f"db_id={db_id}.clockfaces.gnss_ns", minimum=0),
            clock_dwt_cycles=req_int(clockfaces.get("dwt_cycles"), f"db_id={db_id}.clockfaces.dwt_cycles", minimum=0),
            clock_ocxo1_ns=req_int(clockfaces.get("ocxo1_ns"), f"db_id={db_id}.clockfaces.ocxo1_ns", minimum=0),
            clock_ocxo2_ns=req_int(clockfaces.get("ocxo2_ns"), f"db_id={db_id}.clockfaces.ocxo2_ns", minimum=0),
            reset_count=req_int(stats.get("reset_count"), f"db_id={db_id}.stats.reset_count", minimum=0),
            update_count=req_int(stats.get("update_count"), f"db_id={db_id}.stats.update_count", minimum=0),
            rolling_sequence=req_int(
                stats.get("rolling_ppb_current_sequence"),
                f"db_id={db_id}.stats.rolling_ppb_current_sequence",
                minimum=0,
            ),
            endpoint_admitted=bool(stats.get("rolling_ppb_endpoint_admitted") is True),
            interval_advanced=bool(stats.get("rolling_ppb_interval_advanced") is True),
            welfords=welfords,
            tau=tau,
            ppb=ppb,
            frequency_tau=frequency_tau,
            buckets=buckets,
            firmware_checkpoint=firmware_checkpoint,
            pi_checkpoint=pi_checkpoint,
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
            campaign_control_eligible=campaign_control_eligible,
            campaign_status=campaign_status,
            recovery=recovery,
            superseded=bool(root.get("holistic_restore_superseded") is True),
        )

    @property
    def restore_authority(self) -> bool:
        if self.superseded:
            return False
        if self.recovery is not None and self.recovery.restore_authority is False:
            return False
        return True


# -----------------------------------------------------------------------------
# Audit bookkeeping / proof ledger
# -----------------------------------------------------------------------------


@dataclass
class ProofLane:
    passed: int = 0
    bounded: int = 0
    unavailable: int = 0
    failed: int = 0

    def mark(self, status: str) -> None:
        if status == "passed":
            self.passed += 1
        elif status == "bounded":
            self.bounded += 1
        elif status == "unavailable":
            self.unavailable += 1
        elif status == "failed":
            self.failed += 1
        else:
            raise ValueError(f"unknown proof status {status!r}")


@dataclass
class RecoveryEpisode:
    generation: str
    reason: str = ""
    classifications: Counter[str] = field(default_factory=Counter)
    first_db_id: Optional[int] = None
    last_db_id: Optional[int] = None
    source_db_id: Optional[int] = None
    boundary_db_id: Optional[int] = None
    source_reset: Optional[int] = None
    source_update: Optional[int] = None
    boundary_reset: Optional[int] = None
    boundary_update: Optional[int] = None
    physical_rebase: bool = False
    ppb_recoverable: Optional[bool] = None
    lineage_status: str = "UNASSESSED"


@dataclass
class Audit:
    scope: str
    rows: int = 0
    authoritative_rows: int = 0
    superseded_rows: int = 0
    first_db_id: Optional[int] = None
    last_db_id: Optional[int] = None
    first_ts: str = ""
    last_ts: str = ""
    first_gnss: str = ""
    last_gnss: str = ""

    problems: Counter[str] = field(default_factory=Counter)
    events: Counter[str] = field(default_factory=Counter)
    examples: Dict[str, List[str]] = field(default_factory=dict)
    proofs: Dict[str, ProofLane] = field(default_factory=dict)

    epochs: Counter[int] = field(default_factory=Counter)
    campaigns: Counter[str] = field(default_factory=Counter)
    boundaries: Counter[str] = field(default_factory=Counter)
    recovery_classes: Counter[str] = field(default_factory=Counter)
    recovery_episodes: Dict[str, RecoveryEpisode] = field(default_factory=dict)

    physical_rebases: int = 0
    durable_gaps: int = 0
    durable_missing_rows: int = 0
    stats_resets: int = 0
    campaign_splices: int = 0

    firmware_ppb_window_checks: int = 0
    firmware_pi_checkpoint_crosschecks: int = 0
    pi_checkpoint_recoverable_rows: int = 0
    pi_checkpoint_warming_rows: int = 0
    exact_welford_steps: int = 0
    bounded_welford_steps: int = 0
    exact_dac_steps: int = 0
    exact_gnss_raw_steps: int = 0
    tau_checks: int = 0
    total_ppb_checks: int = 0
    append_delta_checks: int = 0

    last_authoritative: Optional[Row] = None
    last_campaign_authoritative: Dict[str, Row] = field(default_factory=dict)

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

    def proof(self, lane: str, status: str) -> None:
        self.proofs.setdefault(lane, ProofLane()).mark(status)


# -----------------------------------------------------------------------------
# Row-local courts
# -----------------------------------------------------------------------------


def check_welford_geometry(audit: Audit, row: Row, name: str, w: Welford) -> None:
    expected_sd = math.sqrt(max(0.0, w.m2) / float(w.n - 1)) if w.n >= 2 else 0.0
    expected_se = expected_sd / math.sqrt(float(w.n)) if w.n else 0.0
    if w.stddev is not None and not close(w.stddev, expected_sd, abs_tol=2.0e-6, rel_tol=2.0e-9):
        audit.problem("welford_stddev", row, f"{name} published={w.stddev:.12g} expected={expected_sd:.12g}")
        audit.proof("welford_geometry", "failed")
    else:
        audit.proof("welford_geometry", "passed")
    if w.stderr is not None and not close(w.stderr, expected_se, abs_tol=2.0e-6, rel_tol=2.0e-9):
        audit.problem("welford_stderr", row, f"{name} published={w.stderr:.12g} expected={expected_se:.12g}")
        audit.proof("welford_geometry", "failed")
    else:
        audit.proof("welford_geometry", "passed")


def check_tau(audit: Audit, row: Row, lane: str) -> None:
    tau = row.tau[lane]
    w = row.welfords[lane]
    failed = False
    if tau.sample_count != tau.interval_count:
        audit.problem("tau_population", row, f"{lane} sample_count={tau.sample_count} interval_count={tau.interval_count}")
        failed = True
    if tau.sample_count != w.n:
        audit.problem("tau_welford_population", row, f"{lane} tau.n={tau.sample_count} welford.n={w.n}")
        failed = True
    if tau.cumulative_reference_ns != tau.sample_count * NS_PER_SECOND:
        audit.problem(
            "tau_reference_total",
            row,
            f"{lane} reference={tau.cumulative_reference_ns} expected={tau.sample_count * NS_PER_SECOND}",
        )
        failed = True

    if tau.valid:
        if tau.cumulative_clock_ns_exact <= 0.0 or tau.cumulative_reference_ns <= 0:
            audit.problem("tau_valid_without_population", row, lane)
            audit.proof("tau_total", "failed")
            return
        expected_tau = float(tau.cumulative_reference_ns) / tau.cumulative_clock_ns_exact
        expected_ppb = (expected_tau - 1.0) * 1.0e9
        published_tau = row.frequency_tau.get(lane)
        if published_tau is not None and not close(
            published_tau, expected_tau, abs_tol=TAU_TOLERANCE, rel_tol=TAU_TOLERANCE
        ):
            audit.problem("tau_ratio", row, f"{lane} published={published_tau} expected={expected_tau:.15f}")
            failed = True
        if not close(row.ppb[lane], expected_ppb, abs_tol=DISPLAY_PPB_TOLERANCE, rel_tol=2.0e-12):
            audit.problem("total_ppb_tau", row, f"{lane} stats.ppb={row.ppb[lane]:.9f} tau={expected_ppb:.9f}")
            failed = True
        total = row.buckets[lane].get("total")
        if total is not None:
            audit.total_ppb_checks += 1
            if not close(total, expected_ppb, abs_tol=PPB_PROOF_TOLERANCE, rel_tol=2.0e-12):
                audit.problem("bucket_total_tau", row, f"{lane} total={total:.9f} tau={expected_ppb:.9f}")
                failed = True
        audit.tau_checks += 1
    audit.proof("tau_total", "failed" if failed else "passed")


def check_dwt_vclock_total(audit: Audit, row: Row, lane: str) -> None:
    w = row.welfords[lane]
    failed = False
    if not close(row.ppb[lane], w.mean, abs_tol=DISPLAY_PPB_TOLERANCE, rel_tol=2.0e-12):
        audit.problem("total_ppb_welford", row, f"{lane} stats.ppb={row.ppb[lane]:.9f} mean={w.mean:.9f}")
        failed = True
    total = row.buckets[lane].get("total")
    if total is not None:
        audit.total_ppb_checks += 1
        if not close(total, w.mean, abs_tol=PPB_PROOF_TOLERANCE, rel_tol=2.0e-12):
            audit.problem("bucket_total_welford", row, f"{lane} total={total:.9f} mean={w.mean:.9f}")
            failed = True
    audit.proof("total_ppb", "failed" if failed else "passed")


def check_firmware_checkpoint(audit: Audit, row: Row) -> None:
    fw = row.firmware_checkpoint
    failed = False
    if fw.reset_count != row.reset_count or fw.update_count != row.update_count:
        audit.problem(
            "firmware_checkpoint_stats_identity",
            row,
            f"checkpoint reset/update={fw.reset_count}/{fw.update_count} row={row.reset_count}/{row.update_count}",
        )
        failed = True
    if fw.rolling_sequence != row.rolling_sequence:
        audit.problem(
            "firmware_checkpoint_rolling_identity",
            row,
            f"checkpoint={fw.rolling_sequence} row={row.rolling_sequence}",
        )
        failed = True

    for key, _seconds in PPB_WINDOWS:
        proof = fw.proof[key]
        for lane in PPB_LANES:
            published = row.buckets[lane].get(key)
            if not proof.valid:
                if published is not None:
                    audit.problem("firmware_ppb_value_without_proof", row, f"{lane}.{key}={published}")
                    failed = True
                audit.proof("firmware_ppb_window", "unavailable")
                continue
            if proof.anchor is None or published is None:
                audit.problem("firmware_ppb_proof_shape", row, f"{lane}.{key} proof/value incomplete")
                audit.proof("firmware_ppb_window", "failed")
                failed = True
                continue
            try:
                computed = ppb_from_endpoints(fw.current, proof.anchor, lane)
            except ValueError as exc:
                audit.problem("firmware_ppb_proof_math", row, f"{lane}.{key}: {exc}")
                audit.proof("firmware_ppb_window", "failed")
                failed = True
                continue
            audit.firmware_ppb_window_checks += 1
            if not close(published, computed, abs_tol=PPB_PROOF_TOLERANCE, rel_tol=2.0e-12):
                audit.problem(
                    "firmware_ppb_proof",
                    row,
                    f"{lane}.{key} published={published:.9f} computed={computed:.9f}",
                )
                audit.proof("firmware_ppb_window", "failed")
                failed = True
            else:
                audit.proof("firmware_ppb_window", "passed")

    audit.proof("firmware_checkpoint_structure", "failed" if failed else "passed")


def check_pi_checkpoint(audit: Audit, row: Row) -> None:
    pi = row.pi_checkpoint
    fw = row.firmware_checkpoint
    if pi is None:
        audit.proof("pi_resurrection_checkpoint", "unavailable")
        return

    failed = False
    comparisons = (
        (pi.reset_count == row.reset_count, "reset_count"),
        (pi.update_count == row.update_count, "update_count"),
        (pi.rolling_sequence == row.update_count, "restore rolling_sequence"),
        (pi.current_sequence == fw.rolling_sequence, "current_sequence"),
        (pi.expected_second_count == fw.second_count, "expected_second_count"),
        (pi.expected_minute_count == fw.minute_count, "expected_minute_count"),
        (pi.last_minute_key == fw.last_minute_key, "last_minute_key"),
        (pi.origin_valid == fw.origin_valid, "origin_valid"),
        (pi.current.equal(fw.current), "current endpoint"),
    )
    for ok, label in comparisons:
        audit.firmware_pi_checkpoint_crosschecks += 1
        if not ok:
            audit.problem("pi_firmware_checkpoint_transcription", row, label)
            failed = True

    if pi.origin_valid and not pi.origin.equal(fw.origin):
        audit.problem("pi_firmware_checkpoint_transcription", row, "origin endpoint")
        failed = True

    if fw.second_append is not None:
        if not pi.second_history or not pi.second_history[-1].equal(fw.second_append):
            audit.problem("pi_checkpoint_second_append", row, "firmware append is not Pi second-history tail")
            failed = True
    if fw.minute_append is not None:
        if not pi.minute_history or not pi.minute_history[-1].equal(fw.minute_append):
            audit.problem("pi_checkpoint_minute_append", row, "firmware append is not Pi minute-history tail")
            failed = True

    expected_second, expected_minute = expected_ring_counts(pi.current_sequence)
    # Producer counts are authoritative; this geometry check is intentionally
    # advisory to implementation shape, not a replacement for firmware testimony.
    if pi.expected_second_count > expected_second or pi.expected_minute_count > expected_minute:
        audit.problem(
            "pi_checkpoint_ring_geometry",
            row,
            f"expected counts {pi.expected_second_count}/{pi.expected_minute_count} "
            f"exceed geometry {expected_second}/{expected_minute}",
        )
        failed = True

    if pi.recoverable:
        audit.pi_checkpoint_recoverable_rows += 1
        audit.proof("pi_resurrection_checkpoint", "failed" if failed else "passed")
    else:
        audit.pi_checkpoint_warming_rows += 1
        audit.proof("pi_resurrection_checkpoint", "failed" if failed else "bounded")
        audit.event(
            "pi_checkpoint_warming",
            row,
            f"status={pi.status or '?'} second={len(pi.second_history)}/{pi.expected_second_count} "
            f"minute={len(pi.minute_history)}/{pi.expected_minute_count} gaps={pi.gap_count}",
        )


def check_row_internal(audit: Audit, row: Row) -> None:
    identity_failed = False
    for name, value in (
        ("completed_pps_sequence", row.completed_pps_sequence),
        ("clockfaces.pps_sequence", row.clockface_pps_sequence),
        ("stats.last_pps_sequence", row.stats_last_pps_sequence),
    ):
        if value != row.sequence:
            audit.problem("physical_identity", row, f"sequence={row.sequence} {name}={value}")
            identity_failed = True
    if not row.completed_row_coherent:
        audit.problem("physical_identity", row, "completed_row_coherent=false")
        identity_failed = True
    if row.snapshot_ok is False:
        audit.problem("physical_snapshot", row, "snapshot_ok=false")
        identity_failed = True
    audit.proof("physical_row_identity", "failed" if identity_failed else "passed")

    expected_clock_gnss = row.instrument_age_seconds * NS_PER_SECOND
    if row.clock_gnss_ns != expected_clock_gnss:
        audit.problem(
            "instrument_gnss_age_identity",
            row,
            f"gnss_ns={row.clock_gnss_ns} instrument_age={row.instrument_age_seconds} expected={expected_clock_gnss}",
        )
        audit.proof("instrument_clockface", "failed")
    else:
        audit.proof("instrument_clockface", "passed")

    if row.rolling_sequence > row.update_count:
        audit.problem(
            "rolling_sequence_leads_stats", row, f"rolling={row.rolling_sequence} update={row.update_count}"
        )

    for name, w in row.welfords.items():
        check_welford_geometry(audit, row, name, w)

    for lane in ("ocxo1", "ocxo2"):
        readback = row.dac_readback[lane]
        if readback is not None and readback != row.dac_hw[lane]:
            audit.problem("dac_readback", row, f"{lane} hw={row.dac_hw[lane]} readback={readback}")
        if not row.dither_enabled:
            rounded = int(math.floor(row.dac_target[lane] + 0.5))
            if row.dac_hw[lane] != rounded:
                audit.problem(
                    "dac_static_target",
                    row,
                    f"{lane} target={row.dac_target[lane]} hw={row.dac_hw[lane]} rounded={rounded}",
                )
        else:
            lo = int(math.floor(row.dac_target[lane]))
            hi = int(math.ceil(row.dac_target[lane]))
            if row.dac_hw[lane] not in {lo, hi}:
                audit.problem(
                    "dac_dither_target",
                    row,
                    f"{lane} target={row.dac_target[lane]} hw={row.dac_hw[lane]} expected={lo}/{hi}",
                )

    check_dwt_vclock_total(audit, row, "dwt")
    check_dwt_vclock_total(audit, row, "vclock")
    check_tau(audit, row, "ocxo1")
    check_tau(audit, row, "ocxo2")
    check_firmware_checkpoint(audit, row)
    check_pi_checkpoint(audit, row)

    if row.gnss_raw_clockface_n is not None and row.gnss_raw_ref_ns is not None:
        expected_ref = row.gnss_raw_clockface_n * NS_PER_SECOND
        if row.gnss_raw_ref_ns != expected_ref:
            audit.problem(
                "gnss_raw_reference",
                row,
                f"clockface_n={row.gnss_raw_clockface_n} ref_ns={row.gnss_raw_ref_ns} expected={expected_ref}",
            )

    check_campaign_row(audit, row)


def check_campaign_row(audit: Audit, row: Row) -> None:
    if row.campaign_name is None:
        return
    failed = False
    if row.campaign_public_count is None or row.campaign_public_count <= 0:
        audit.problem("campaign_identity", row, f"public_count={row.campaign_public_count}")
        audit.proof("campaign_row", "failed")
        return

    cgnss = row.campaign_clockfaces.get("gnss_ns")
    expected = row.campaign_public_count * NS_PER_SECOND
    if cgnss != expected:
        audit.problem(
            "campaign_gnss_clockface",
            row,
            f"public_count={row.campaign_public_count} gnss_ns={cgnss} expected={expected}",
        )
        failed = True

    if cgnss and cgnss > 0:
        expected_values: Dict[str, float] = {"vclock": 0.0}
        dwt = row.campaign_clockfaces.get("dwt_cycles")
        o1 = row.campaign_clockfaces.get("ocxo1_ns")
        o2 = row.campaign_clockfaces.get("ocxo2_ns")
        if dwt is not None:
            expected_values["dwt"] = (float(dwt) / (float(cgnss) * 1.008) - 1.0) * 1.0e9
        if o1 is not None:
            expected_values["ocxo1"] = (float(o1) / float(cgnss) - 1.0) * 1.0e9
        if o2 is not None:
            expected_values["ocxo2"] = (float(o2) / float(cgnss) - 1.0) * 1.0e9
        for lane, expected_ppb in expected_values.items():
            published = row.campaign_ppb.get(lane)
            if published is not None and not close(
                published, expected_ppb, abs_tol=PPB_PROOF_TOLERANCE, rel_tol=2.0e-12
            ):
                audit.problem(
                    "campaign_ppb",
                    row,
                    f"{lane} published={published:.9f} clockface={expected_ppb:.9f}",
                )
                failed = True

    audit.proof("campaign_row", "failed" if failed else "passed")


# -----------------------------------------------------------------------------
# Recurrence and boundary courts
# -----------------------------------------------------------------------------


def check_welford_one_step(
    audit: Audit,
    row: Row,
    name: str,
    previous: Welford,
    current: Welford,
    *,
    known_sample: Optional[float] = None,
) -> str:
    if current.n != previous.n + 1:
        return "not_one_step"

    if known_sample is not None:
        sample = float(known_sample)
        expected_mean = previous.mean + (sample - previous.mean) / current.n
        expected_m2 = previous.m2 + (sample - previous.mean) * (sample - expected_mean)
        expected_min = sample if previous.n == 0 else min(previous.min_value, sample)
        expected_max = sample if previous.n == 0 else max(previous.max_value, sample)
        checks = (
            close(current.mean, expected_mean, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL),
            close(
                current.m2,
                expected_m2,
                abs_tol=max(WELFORD_FLOAT_ABS_TOL, abs(expected_m2) * 2.0e-12),
                rel_tol=2.0e-12,
            ),
            close(current.min_value, expected_min, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL),
            close(current.max_value, expected_max, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL),
        )
        if all(checks):
            audit.exact_welford_steps += 1
            return "exact"
        audit.problem(
            "welford_recurrence",
            row,
            f"{name} known-sample recurrence failed sample={sample:.12g} n={previous.n}->{current.n}",
        )
        return "failed"

    # The producer publishes means to finite decimal precision.  Infer only a
    # bounded one-sample compatibility interval; never pretend to know x exactly.
    nominal_sample = current.n * current.mean - previous.n * previous.mean
    expected_mean = previous.mean + (nominal_sample - previous.mean) / current.n
    expected_m2 = previous.m2 + (nominal_sample - previous.mean) * (nominal_sample - expected_mean)
    expected_min = nominal_sample if previous.n == 0 else min(previous.min_value, nominal_sample)
    expected_max = nominal_sample if previous.n == 0 else max(previous.max_value, nominal_sample)
    nominal = (
        close(current.mean, expected_mean, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL)
        and close(
            current.m2,
            expected_m2,
            abs_tol=max(WELFORD_FLOAT_ABS_TOL, abs(expected_m2) * 2.0e-12),
            rel_tol=2.0e-12,
        )
        and close(current.min_value, expected_min, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL)
        and close(current.max_value, expected_max, abs_tol=WELFORD_FLOAT_ABS_TOL, rel_tol=WELFORD_FLOAT_REL_TOL)
    )
    if nominal:
        audit.exact_welford_steps += 1
        return "exact"

    half_q = WELFORD_SERIALIZED_MEAN_QUANTUM / 2.0
    prev_mean_lo = previous.mean - half_q
    prev_mean_hi = previous.mean + half_q
    cur_mean_lo = current.mean - half_q
    cur_mean_hi = current.mean + half_q
    sample_candidates = (
        current.n * cur_mean_lo - previous.n * prev_mean_lo,
        current.n * cur_mean_lo - previous.n * prev_mean_hi,
        current.n * cur_mean_hi - previous.n * prev_mean_lo,
        current.n * cur_mean_hi - previous.n * prev_mean_hi,
    )
    sample_lo = min(sample_candidates)
    sample_hi = max(sample_candidates)
    diff_lo = cur_mean_lo - prev_mean_hi
    diff_hi = cur_mean_hi - prev_mean_lo
    diff_sq_min = 0.0 if diff_lo <= 0.0 <= diff_hi else min(diff_lo * diff_lo, diff_hi * diff_hi)
    diff_sq_max = max(diff_lo * diff_lo, diff_hi * diff_hi)
    scale = float(current.n * previous.n)
    m2_lo = previous.m2 + scale * diff_sq_min
    m2_hi = previous.m2 + scale * diff_sq_max
    m2_tol = max(WELFORD_FLOAT_ABS_TOL, abs(current.m2) * 2.0e-12, abs(m2_hi) * 2.0e-12)
    min_lo = sample_lo if previous.n == 0 else min(previous.min_value, sample_lo)
    min_hi = sample_hi if previous.n == 0 else min(previous.min_value, sample_hi)
    max_lo = sample_lo if previous.n == 0 else max(previous.max_value, sample_lo)
    max_hi = sample_hi if previous.n == 0 else max(previous.max_value, sample_hi)
    compatible = bool(
        m2_lo - m2_tol <= current.m2 <= m2_hi + m2_tol
        and min_lo - WELFORD_FLOAT_ABS_TOL <= current.min_value <= min_hi + WELFORD_FLOAT_ABS_TOL
        and max_lo - WELFORD_FLOAT_ABS_TOL <= current.max_value <= max_hi + WELFORD_FLOAT_ABS_TOL
    )
    if compatible:
        audit.bounded_welford_steps += 1
        audit.event(
            "welford_serialization_limited",
            row,
            f"{name} n={previous.n}->{current.n} sample_interval=[{sample_lo:.12g},{sample_hi:.12g}]",
        )
        return "bounded"

    audit.problem(
        "welford_recurrence",
        row,
        f"{name} one-sample recurrence incompatible with serialized-mean uncertainty n={previous.n}->{current.n}",
    )
    return "failed"


def classify_boundary(prev: Row, cur: Row) -> str:
    if cur.reset_count > prev.reset_count:
        return "STATS_RESET"
    if cur.reset_count < prev.reset_count:
        return "STATS_RESET_REGRESSION"

    seq_delta = cur.sequence - prev.sequence
    if seq_delta < 0:
        if cur.update_count == prev.update_count + 1:
            return "PHYSICAL_REBOOT_ALPHA_N_PLUS_1"
        if cur.update_count > prev.update_count:
            return "PHYSICAL_REBOOT_ALPHA_CONTINUED"
        return "PHYSICAL_REBOOT_UNPROVED"
    if seq_delta == 0:
        return "PHYSICAL_SEQUENCE_REPEAT"
    if seq_delta == 1:
        if cur.update_count == prev.update_count + 1:
            return "NORMAL_CONTINUATION"
        return "ADJACENT_STATS_MISMATCH"
    return "DURABLE_OBSERVATION_GAP"


def check_endpoint_append_delta(audit: Audit, prev: Row, cur: Row) -> None:
    if not (prev.endpoint_admitted and cur.endpoint_admitted):
        return
    if cur.rolling_sequence != prev.rolling_sequence + 1:
        return
    if cur.sequence != prev.sequence + 1:
        return
    p = prev.firmware_checkpoint.current
    c = cur.firmware_checkpoint.current
    ref_delta = cur.clock_gnss_ns - prev.clock_gnss_ns
    dwt_delta = cur.clock_dwt_cycles - prev.clock_dwt_cycles
    o1_delta = cur.clock_ocxo1_ns - prev.clock_ocxo1_ns
    o2_delta = cur.clock_ocxo2_ns - prev.clock_ocxo2_ns
    expected_dwt_error_delta = float(dwt_delta) - float(ref_delta) * 1.008
    expected_interval = p.interval_count + (1 if cur.interval_advanced else 0)
    checks = (
        c.reference_ns - p.reference_ns == ref_delta,
        close(
            c.dwt_error_cycles - p.dwt_error_cycles,
            expected_dwt_error_delta,
            abs_tol=2.0e-6,
            rel_tol=2.0e-12,
        ),
        c.ocxo1_error_ns - p.ocxo1_error_ns == o1_delta - ref_delta,
        c.ocxo2_error_ns - p.ocxo2_error_ns == o2_delta - ref_delta,
        c.interval_count == expected_interval,
    )
    audit.append_delta_checks += 1
    if not all(checks):
        audit.problem(
            "firmware_checkpoint_append_delta",
            cur,
            f"rolling {prev.rolling_sequence}->{cur.rolling_sequence} raw clockface delta does not close current endpoint",
        )
        audit.proof("firmware_append_from_raw", "failed")
    else:
        audit.proof("firmware_append_from_raw", "passed")


def check_adjacent_authoritative(audit: Audit, prev: Row, cur: Row) -> str:
    boundary = classify_boundary(prev, cur)
    audit.boundaries[boundary] += 1

    if cur.db_id <= prev.db_id:
        audit.problem("db_chronology", cur, f"id {prev.db_id}->{cur.db_id}")

    seq_delta = cur.sequence - prev.sequence
    if prev.gnss_utc_s is not None and cur.gnss_utc_s is not None:
        utc_delta = cur.gnss_utc_s - prev.gnss_utc_s
        if seq_delta > 0 and utc_delta != seq_delta:
            audit.event(
                "gnss_utc_context_skew",
                cur,
                f"utc_delta={utc_delta}s physical_sequence_delta={seq_delta} {prev.gnss_utc}->{cur.gnss_utc}",
            )

    if boundary == "STATS_RESET_REGRESSION":
        audit.problem("stats_reset_regression", cur, f"reset_count {prev.reset_count}->{cur.reset_count}")
        audit.proof("alpha_statistics_lineage", "failed")
        return boundary

    if boundary == "STATS_RESET":
        audit.stats_resets += 1
        audit.event(
            "stats_epoch",
            cur,
            f"reset_count {prev.reset_count}->{cur.reset_count} update_count={cur.update_count}",
        )
        failed = False
        if cur.update_count != 1:
            audit.problem("stats_epoch_birth", cur, f"new epoch update_count={cur.update_count}, expected 1")
            failed = True
        if cur.rolling_sequence not in (0, 1):
            audit.problem("rolling_epoch_birth", cur, f"rolling_sequence={cur.rolling_sequence}")
            failed = True
        if cur.pi_checkpoint is not None and cur.pi_checkpoint.recoverable is False:
            # A reset observed from birth should be immediately self-contained.
            audit.problem("stats_epoch_checkpoint", cur, "new epoch Pi checkpoint is not recoverable")
            failed = True
        audit.proof("alpha_statistics_lineage", "failed" if failed else "passed")
        return boundary

    if boundary == "PHYSICAL_SEQUENCE_REPEAT":
        audit.problem("physical_sequence_repeat", cur, f"sequence={cur.sequence}")
        audit.proof("physical_lifetime", "failed")
        return boundary

    if seq_delta > 0:
        expected_gnss_delta = seq_delta * NS_PER_SECOND
        actual_gnss_delta = cur.clock_gnss_ns - prev.clock_gnss_ns
        age_delta = cur.instrument_age_seconds - prev.instrument_age_seconds
        physical_failed = False
        if actual_gnss_delta != expected_gnss_delta:
            audit.problem(
                "clockface_gnss_continuity",
                cur,
                f"sequence_delta={seq_delta} delta={actual_gnss_delta} expected={expected_gnss_delta}",
            )
            physical_failed = True
        if age_delta != seq_delta:
            audit.problem(
                "instrument_age_continuity",
                cur,
                f"sequence_delta={seq_delta} age_delta={age_delta}",
            )
            physical_failed = True
        audit.proof("physical_lifetime", "failed" if physical_failed else "passed")

    if boundary == "DURABLE_OBSERVATION_GAP":
        audit.durable_gaps += 1
        audit.durable_missing_rows += seq_delta - 1
        audit.event(
            "durable_observation_gap",
            cur,
            f"sequence {prev.sequence}->{cur.sequence} missing_durable={seq_delta - 1} "
            f"stats {prev.update_count}->{cur.update_count}",
        )
        if cur.update_count <= prev.update_count:
            audit.problem(
                "stats_gap_nonmonotonic",
                cur,
                f"update_count {prev.update_count}->{cur.update_count}",
            )
            audit.proof("alpha_statistics_lineage", "failed")
        else:
            audit.proof("alpha_statistics_lineage", "bounded")
            audit.event(
                "stats_unobserved_advancement",
                cur,
                f"update_count {prev.update_count}->{cur.update_count}; hidden rows are not replayed",
            )
        check_pi_owned_gap(audit, prev, cur)
        return boundary

    if boundary in {"PHYSICAL_REBOOT_ALPHA_N_PLUS_1", "PHYSICAL_REBOOT_ALPHA_CONTINUED", "PHYSICAL_REBOOT_UNPROVED"}:
        audit.physical_rebases += 1
        audit.event(
            "physical_sequence_rebase",
            cur,
            f"sequence {prev.sequence}->{cur.sequence} stats {prev.reset_count}/{prev.update_count} "
            f"->{cur.reset_count}/{cur.update_count}",
        )
        check_resurrection_boundary(audit, prev, cur, boundary)
        return boundary

    if boundary == "ADJACENT_STATS_MISMATCH":
        audit.problem(
            "adjacent_stats_chronology",
            cur,
            f"physical sequence {prev.sequence}->{cur.sequence} but update_count {prev.update_count}->{cur.update_count}",
        )
        audit.proof("alpha_statistics_lineage", "failed")
    else:
        # Exact ordinary N+1 row.
        audit.proof("alpha_statistics_lineage", "passed")
        check_exact_one_row_statistics(audit, prev, cur)
        check_endpoint_append_delta(audit, prev, cur)
        check_pi_owned_one_row(audit, prev, cur)

    return boundary


def check_exact_one_row_statistics(audit: Audit, prev: Row, cur: Row) -> None:
    for name in FIRMWARE_WELFORD_LANES:
        p = prev.welfords[name]
        c = cur.welfords[name]
        delta = c.n - p.n
        if delta == 0:
            if not c.sufficient_equal(p):
                audit.problem("welford_frozen_mutation", cur, f"{name} n unchanged at {c.n} but state changed")
                audit.proof("firmware_welford_lineage", "failed")
            else:
                audit.proof("firmware_welford_lineage", "passed")
        elif delta == 1:
            status = check_welford_one_step(audit, cur, name, p, c)
            audit.proof(
                "firmware_welford_lineage",
                "passed" if status == "exact" else "bounded" if status == "bounded" else "failed",
            )
        else:
            audit.problem("welford_population_jump", cur, f"{name} n {p.n}->{c.n} on adjacent physical row")
            audit.proof("firmware_welford_lineage", "failed")


def check_pi_owned_one_row(audit: Audit, prev: Row, cur: Row) -> None:
    for lane in ("ocxo1", "ocxo2"):
        name = f"{lane}_dac"
        p = prev.welfords[name]
        c = cur.welfords[name]
        delta = c.n - p.n
        if delta == 0:
            if not c.sufficient_equal(p):
                audit.problem("dac_welford_frozen_mutation", cur, f"{name} n={c.n} state changed")
                audit.proof("pi_dac_lineage", "failed")
            else:
                audit.proof("pi_dac_lineage", "passed")
        elif delta == 1:
            status = check_welford_one_step(
                audit, cur, name, p, c, known_sample=float(cur.dac_target[lane])
            )
            if status == "exact":
                audit.exact_dac_steps += 1
                audit.proof("pi_dac_lineage", "passed")
            else:
                audit.proof("pi_dac_lineage", "failed")
        else:
            audit.problem("dac_welford_population_jump", cur, f"{name} n {p.n}->{c.n}")
            audit.proof("pi_dac_lineage", "failed")

    if prev.gnss_raw_welford is None or cur.gnss_raw_welford is None:
        audit.proof("pi_gnss_raw_lineage", "unavailable")
    else:
        p = prev.gnss_raw_welford
        c = cur.gnss_raw_welford
        delta = c.n - p.n
        if delta == 0:
            if not c.sufficient_equal(p):
                audit.problem("gnss_raw_frozen_mutation", cur, f"n={c.n} state changed")
                audit.proof("pi_gnss_raw_lineage", "failed")
            else:
                audit.proof("pi_gnss_raw_lineage", "passed")
        elif delta == 1 and cur.gnss_raw_drift_ppb is not None:
            status = check_welford_one_step(
                audit,
                cur,
                "gnss_raw",
                p,
                c,
                known_sample=cur.gnss_raw_drift_ppb,
            )
            if status == "exact":
                audit.exact_gnss_raw_steps += 1
                audit.proof("pi_gnss_raw_lineage", "passed")
            else:
                audit.proof("pi_gnss_raw_lineage", "failed")
        else:
            audit.problem("gnss_raw_population_jump", cur, f"n {p.n}->{c.n} on adjacent physical row")
            audit.proof("pi_gnss_raw_lineage", "failed")

        if prev.gnss_raw_clockface_n is not None and cur.gnss_raw_clockface_n is not None:
            if cur.gnss_raw_clockface_n != prev.gnss_raw_clockface_n + 1:
                audit.problem(
                    "gnss_raw_clockface_chronology",
                    cur,
                    f"clockface_n {prev.gnss_raw_clockface_n}->{cur.gnss_raw_clockface_n}",
                )
                audit.proof("pi_gnss_raw_lineage", "failed")


def check_pi_owned_gap(audit: Audit, prev: Row, cur: Row) -> None:
    # Database gaps hide Pi observations.  The correct theorem is monotonicity,
    # not a fabricated sequence of one-sample recurrences.
    for lane in ("ocxo1_dac", "ocxo2_dac"):
        if cur.welfords[lane].n < prev.welfords[lane].n:
            audit.problem("pi_dac_regression", cur, f"{lane} n {prev.welfords[lane].n}->{cur.welfords[lane].n}")
            audit.proof("pi_dac_lineage", "failed")
        else:
            audit.proof("pi_dac_lineage", "bounded")
    if prev.gnss_raw_welford is not None and cur.gnss_raw_welford is not None:
        if cur.gnss_raw_welford.n < prev.gnss_raw_welford.n:
            audit.problem(
                "gnss_raw_welford_regression",
                cur,
                f"n {prev.gnss_raw_welford.n}->{cur.gnss_raw_welford.n}",
            )
            audit.proof("pi_gnss_raw_lineage", "failed")
        else:
            audit.proof("pi_gnss_raw_lineage", "bounded")


def check_resurrection_boundary(audit: Audit, source: Row, boundary: Row, kind: str) -> None:
    failed = False
    exact_n_plus_1 = boundary.update_count == source.update_count + 1
    same_epoch = boundary.reset_count == source.reset_count
    if not same_epoch:
        audit.problem(
            "resurrection_epoch",
            boundary,
            f"reset_count {source.reset_count}->{boundary.reset_count}",
        )
        failed = True
    if boundary.update_count <= source.update_count:
        audit.problem(
            "resurrection_update_count",
            boundary,
            f"update_count {source.update_count}->{boundary.update_count}",
        )
        failed = True

    for label, p, c in (
        ("gnss_ns", source.clock_gnss_ns, boundary.clock_gnss_ns),
        ("dwt_cycles", source.clock_dwt_cycles, boundary.clock_dwt_cycles),
        ("ocxo1_ns", source.clock_ocxo1_ns, boundary.clock_ocxo1_ns),
        ("ocxo2_ns", source.clock_ocxo2_ns, boundary.clock_ocxo2_ns),
    ):
        if c <= p:
            audit.problem("resurrection_clockface", boundary, f"{label} {p}->{c} did not advance")
            failed = True

    if boundary.pi_checkpoint is None:
        audit.problem("resurrection_checkpoint_missing", boundary, "Pi resurrection checkpoint absent")
        failed = True
    elif not boundary.pi_checkpoint.recoverable:
        audit.problem(
            "resurrection_checkpoint_unrecoverable",
            boundary,
            f"status={boundary.pi_checkpoint.status} second={len(boundary.pi_checkpoint.second_history)}/"
            f"{boundary.pi_checkpoint.expected_second_count} minute={len(boundary.pi_checkpoint.minute_history)}/"
            f"{boundary.pi_checkpoint.expected_minute_count}",
        )
        failed = True

    # Statistical sufficient state must never regress.  Exact N+1 resurrection
    # allows each science population either to freeze (excluded row) or advance once.
    for name in FIRMWARE_WELFORD_LANES:
        p = source.welfords[name]
        c = boundary.welfords[name]
        if c.n < p.n:
            audit.problem("resurrection_welford_regression", boundary, f"{name} n {p.n}->{c.n}")
            failed = True
        elif exact_n_plus_1 and c.n - p.n not in (0, 1):
            audit.problem("resurrection_welford_population", boundary, f"{name} n {p.n}->{c.n}")
            failed = True

    if failed:
        audit.proof("alpha_resurrection", "failed")
        return

    if exact_n_plus_1:
        audit.event(
            "alpha_resurrection_n_plus_1",
            boundary,
            f"physical sequence {source.sequence}->{boundary.sequence}; stats update {source.update_count}->{boundary.update_count}",
        )
        audit.proof("alpha_resurrection", "passed")
    else:
        # This can be a surviving/restored Alpha with unobserved post-source updates.
        audit.event(
            "alpha_resurrection_with_unobserved_advancement",
            boundary,
            f"physical sequence {source.sequence}->{boundary.sequence}; stats update {source.update_count}->{boundary.update_count}",
        )
        audit.proof("alpha_resurrection", "bounded")

    # Pi-owned state is independently recoverable.  A cold Alpha resurrection does
    # not license Pi state regression.
    for name in PI_WELFORD_LANES:
        if boundary.welfords[name].n < source.welfords[name].n:
            audit.problem("pi_state_regression_at_resurrection", boundary, f"{name} n {source.welfords[name].n}->{boundary.welfords[name].n}")
            audit.proof("pi_dac_lineage", "failed")
    if source.gnss_raw_welford is not None and boundary.gnss_raw_welford is not None:
        if boundary.gnss_raw_welford.n < source.gnss_raw_welford.n:
            audit.problem(
                "pi_state_regression_at_resurrection",
                boundary,
                f"gnss_raw n {source.gnss_raw_welford.n}->{boundary.gnss_raw_welford.n}",
            )
            audit.proof("pi_gnss_raw_lineage", "failed")


# -----------------------------------------------------------------------------
# Recovery custody and TEMPEST chronology
# -----------------------------------------------------------------------------


def observe_recovery_custody(audit: Audit, row: Row) -> None:
    recovery = row.recovery
    if recovery is None:
        return
    audit.recovery_classes[recovery.classification or "UNCLASSIFIED"] += 1
    ep = audit.recovery_episodes.setdefault(
        recovery.generation,
        RecoveryEpisode(generation=recovery.generation, reason=recovery.reason),
    )
    ep.first_db_id = row.db_id if ep.first_db_id is None else min(ep.first_db_id, row.db_id)
    ep.last_db_id = row.db_id if ep.last_db_id is None else max(ep.last_db_id, row.db_id)
    ep.classifications[recovery.classification or "UNCLASSIFIED"] += 1
    ep.physical_rebase = ep.physical_rebase or recovery.physical_sequence_regression

    if recovery.restore_authority and ep.boundary_db_id is None:
        source = audit.last_authoritative
        ep.boundary_db_id = row.db_id
        ep.boundary_reset = row.reset_count
        ep.boundary_update = row.update_count
        ep.ppb_recoverable = bool(row.pi_checkpoint and row.pi_checkpoint.recoverable)
        if source is not None:
            ep.source_db_id = source.db_id
            ep.source_reset = source.reset_count
            ep.source_update = source.update_count
            lineage_ok = bool(
                row.reset_count == source.reset_count
                and row.update_count > source.update_count
                and row.clock_gnss_ns > source.clock_gnss_ns
                and row.clock_dwt_cycles > source.clock_dwt_cycles
                and row.clock_ocxo1_ns > source.clock_ocxo1_ns
                and row.clock_ocxo2_ns > source.clock_ocxo2_ns
                and row.pi_checkpoint is not None
                and row.pi_checkpoint.recoverable
            )
            if recovery.classification == "RESTORED_RECOVERY_LINEAGE":
                ep.lineage_status = "PROVED" if lineage_ok else "FAILED"
                audit.proof("recovery_custody", "passed" if lineage_ok else "failed")
                if not lineage_ok:
                    audit.problem(
                        "recovery_custody_lineage",
                        row,
                        f"generation={recovery.generation} source_db={source.db_id} boundary_db={row.db_id}",
                    )
            elif recovery.classification == "LIVE_REATTACH_CONTINUITY":
                # Live Alpha can advance while Pi is absent, so >source is enough.
                ep.lineage_status = "PROVED" if lineage_ok else "FAILED"
                audit.proof("recovery_custody", "passed" if lineage_ok else "failed")
            else:
                ep.lineage_status = "BOUNDARY_PRESENT"
                audit.proof("recovery_custody", "bounded")


def check_campaign_chronology(audit: Audit, row: Row, boundary: Optional[str]) -> None:
    if row.campaign_name is None or row.campaign_public_count is None or not row.restore_authority:
        return
    prev = audit.last_campaign_authoritative.get(row.campaign_name)
    if prev is not None and prev.campaign_public_count is not None:
        delta = row.campaign_public_count - prev.campaign_public_count
        if delta <= 0:
            audit.problem(
                "campaign_public_chronology",
                row,
                f"{row.campaign_name} {prev.campaign_public_count}->{row.campaign_public_count}",
            )
            audit.proof("campaign_timeline", "failed")
        elif delta == 1:
            audit.proof("campaign_timeline", "passed")
        else:
            audit.campaign_splices += 1
            lawful = bool(
                boundary in {
                    "DURABLE_OBSERVATION_GAP",
                    "PHYSICAL_REBOOT_ALPHA_N_PLUS_1",
                    "PHYSICAL_REBOOT_ALPHA_CONTINUED",
                }
                or (row.recovery is not None and row.recovery.restore_authority)
            )
            audit.event(
                "campaign_forward_splice",
                row,
                f"{row.campaign_name} {prev.campaign_public_count}->{row.campaign_public_count} delta={delta} "
                f"lawful_context={lawful}",
            )
            if lawful:
                audit.proof("campaign_timeline", "passed")
            else:
                audit.problem(
                    "campaign_unexplained_splice",
                    row,
                    f"public_count {prev.campaign_public_count}->{row.campaign_public_count}",
                )
                audit.proof("campaign_timeline", "failed")
    audit.last_campaign_authoritative[row.campaign_name] = row


# -----------------------------------------------------------------------------
# Main streaming fold
# -----------------------------------------------------------------------------


def process_row(audit: Audit, row: Row) -> None:
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

    check_row_internal(audit, row)
    observe_recovery_custody(audit, row)

    if row.superseded:
        audit.superseded_rows += 1
        audit.event(
            "superseded_recovery_evidence",
            row,
            f"classification={row.recovery.classification if row.recovery else '?'}",
        )
        audit.proof("restore_authority", "passed")
        # Superseded rows are evidence about the failed/newborn side of recovery,
        # never the predecessor for scientific lineage.
        return

    if row.recovery is not None and not row.recovery.restore_authority:
        audit.superseded_rows += 1
        audit.event(
            "quarantined_recovery_evidence",
            row,
            f"generation={row.recovery.generation} classification={row.recovery.classification}",
        )
        audit.proof("restore_authority", "passed")
        return

    audit.authoritative_rows += 1
    audit.proof("restore_authority", "passed")

    prev = audit.last_authoritative
    boundary: Optional[str] = None
    if prev is not None:
        boundary = check_adjacent_authoritative(audit, prev, row)
    check_campaign_chronology(audit, row, boundary)
    audit.last_authoritative = row


# -----------------------------------------------------------------------------
# Database scope / CLI
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class Args:
    campaign: Optional[str]
    all_rows: bool
    from_id: int
    to_id: int
    batch_size: int
    pause_ms: int
    limit: int


def campaign_bounds(campaign: str) -> Tuple[int, int]:
    """Return the first/last durable CLOCKS_V4 IDs for one TEMPEST campaign.

    Do not aggregate the whole campaign merely to discover its endpoints.
    The matching partial index is ordered by ``id``, so two one-row probes
    establish the exact bounds without a JSON-filtered campaign scan.
    """
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor()

        def endpoint(order: str) -> Optional[int]:
            cur.execute(
                f"""
                SELECT id
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND campaign = %s
                  AND payload #>> '{{schema}}' = %s
                ORDER BY id {order}
                LIMIT 1
                """,
                (CAMPAIGN_TYPE, campaign, CLOCKS_SCHEMA),
            )
            row = cur.fetchone()
            return opt_int(row.get("id")) if row else None

        lo = endpoint("ASC")
        hi = endpoint("DESC")

    if lo is None or hi is None:
        raise SystemExit(f"no CLOCKS_V4 rows found for campaign {campaign!r}")
    return lo, hi


def iter_rows(args: Args) -> Iterator[Tuple[int, str, Dict[str, Any]]]:
    """Stream the proof ledger using a minimal canonical payload projection.

    The analyzer still retrieves every producer/Pi witness it adjudicates.
    Fields outside that proof surface are left in PostgreSQL so the enlarged
    always-on recovery ledger does not force unrelated row transfer/decoding.
    """
    from_id = args.from_id
    to_id = args.to_id
    if args.campaign is not None:
        campaign_lo, campaign_hi = campaign_bounds(args.campaign)
        from_id = max(from_id, campaign_lo) if from_id else campaign_lo
        to_id = min(to_id, campaign_hi) if to_id else campaign_hi

    where = ["campaign_type = %s", "payload #>> '{schema}' = %s"]
    params: List[Any] = [CAMPAIGN_TYPE, CLOCKS_SCHEMA]
    if from_id > 0:
        where.append("id >= %s")
        params.append(from_id)
    if to_id > 0:
        where.append("id <= %s")
        params.append(to_id)

    sql = f"""
        SELECT
            id,
            ts,
            payload -> 'sequence' AS sequence_payload,
            payload -> 'campaign' AS campaign_payload,
            payload -> 'recovery_custody' AS recovery_custody_payload,
            payload -> 'holistic_restore_superseded' AS superseded_payload,
            payload #> '{{clocks,schema}}' AS clocks_schema_payload,
            payload #> '{{clocks,completed_pps_sequence}}' AS completed_pps_sequence_payload,
            payload #> '{{clocks,completed_row_coherent}}' AS completed_row_coherent_payload,
            payload #> '{{clocks,snapshot_ok}}' AS snapshot_ok_payload,
            payload #> '{{clocks,instrument_age_seconds}}' AS instrument_age_seconds_payload,
            payload #> '{{clocks,gnss_time_utc}}' AS gnss_time_utc_payload,
            payload #> '{{clocks,clockfaces}}' AS clockfaces_payload,
            payload #> '{{clocks,stats}}' AS stats_payload,
            payload #> '{{clocks,control}}' AS control_payload,
            payload #> '{{clocks,gnss_raw}}' AS gnss_raw_payload,
            payload #> '{{clocks,ppb_restore_checkpoint}}' AS ppb_restore_checkpoint_payload
        FROM campaign_detail
        WHERE {' AND '.join(where)}
        ORDER BY id ASC
    """
    if args.limit > 0:
        sql += " LIMIT %s"
        params.append(args.limit)

    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="campaign_analyzer_proof_ledger")
        cur.itersize = max(1, args.batch_size)
        cur.execute(sql, tuple(params))
        in_batch = 0

        def json_value(value: Any) -> Any:
            # PostgreSQL JSONB scalar strings are already returned by the DB
            # adapter as ordinary Python strings (for example
            # CLOCKS_INSTRUMENT_STATE_V1 or an ISO UTC timestamp).  Only decode
            # a string when it is visibly a serialized JSON container left by a
            # different adapter configuration.
            if isinstance(value, str):
                stripped = value.lstrip()
                if stripped.startswith("{") or stripped.startswith("["):
                    return json.loads(value)
            return value

        for dbrow in cur:
            # Reconstitute only the canonical testimony actually consumed by
            # Row.parse().  In particular, stats and ppb_restore_checkpoint
            # remain complete because they are direct subjects of the proof
            # ledger; unrelated SYSTEM/environment decoration stays in SQL.
            clocks: Dict[str, Any] = {
                "schema": json_value(dbrow["clocks_schema_payload"]),
                "completed_pps_sequence": json_value(dbrow["completed_pps_sequence_payload"]),
                "completed_row_coherent": json_value(dbrow["completed_row_coherent_payload"]),
                "snapshot_ok": json_value(dbrow["snapshot_ok_payload"]),
                "instrument_age_seconds": json_value(dbrow["instrument_age_seconds_payload"]),
                "gnss_time_utc": json_value(dbrow["gnss_time_utc_payload"]),
                "clockfaces": json_value(dbrow["clockfaces_payload"]),
                "stats": json_value(dbrow["stats_payload"]),
                "control": json_value(dbrow["control_payload"]),
                "gnss_raw": json_value(dbrow["gnss_raw_payload"]),
                "ppb_restore_checkpoint": json_value(dbrow["ppb_restore_checkpoint_payload"]),
            }
            payload: Dict[str, Any] = {
                "schema": CLOCKS_SCHEMA,
                "sequence": json_value(dbrow["sequence_payload"]),
                "clocks": clocks,
            }

            campaign = json_value(dbrow["campaign_payload"])
            if campaign is not None:
                payload["campaign"] = campaign

            recovery_custody = json_value(dbrow["recovery_custody_payload"])
            if recovery_custody is not None:
                payload["recovery_custody"] = recovery_custody

            superseded = json_value(dbrow["superseded_payload"])
            if superseded is not None:
                payload["holistic_restore_superseded"] = superseded

            yield int(dbrow["id"]), str(dbrow["ts"]), payload

            in_batch += 1
            if in_batch >= max(1, args.batch_size):
                in_batch = 0
                if args.pause_ms:
                    time.sleep(args.pause_ms / 1000.0)


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
                ) AS superseded_rows,
                count(*) FILTER (
                    WHERE COALESCE((payload #>> '{clocks,ppb_restore_checkpoint,recoverable}')::boolean, false)
                ) AS recoverable_checkpoint_rows
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{schema}' = %s
            """,
            (CAMPAIGN_TYPE, CLOCKS_SCHEMA),
        )
        totals = cur.fetchone() or {}
        cur.execute(
            """
            SELECT campaign, count(*) AS rows, min(id) AS min_id, max(id) AS max_id, max(ts) AS last_ts
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

    print("=" * 96)
    print("CLOCKS_V4 PROOF-LEDGER AUDIT SCOPE")
    print("=" * 96)
    print(
        f"Rows: {int(totals.get('rows') or 0):,}   "
        f"DB id: {totals.get('min_id')}..{totals.get('max_id')}   "
        f"ambient={int(totals.get('ambient_rows') or 0):,}   "
        f"campaign={int(totals.get('campaign_rows') or 0):,}   "
        f"superseded={int(totals.get('superseded_rows') or 0):,}   "
        f"ppb_recoverable={int(totals.get('recoverable_checkpoint_rows') or 0):,}"
    )
    print(f"Time: {totals.get('first_ts')} -> {totals.get('last_ts')}")
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
    print("\nNamed campaign scope includes every CLOCKS_V4 row between that campaign's first and last durable IDs,")
    print("so ambient/recovery testimony inside the campaign interval is not accidentally hidden.")


def parse_args(argv: Sequence[str]) -> Args:
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
            raise SystemExit("only one campaign name may be supplied")
        campaign = arg
        idx += 1

    if all_rows and campaign is not None:
        raise SystemExit("choose either --all or a campaign name")
    if not all_rows and campaign is None and any(v > 0 for v in (from_id, to_id, limit)):
        all_rows = True
    if batch_size <= 0 or pause_ms < 0 or limit < 0 or from_id < 0 or to_id < 0:
        raise SystemExit("numeric options must be non-negative; --batch-size must be > 0")
    if from_id and to_id and from_id > to_id:
        raise SystemExit("--from-id must be <= --to-id")
    return Args(campaign, all_rows, from_id, to_id, batch_size, pause_ms, limit)


# -----------------------------------------------------------------------------
# Report / verdict
# -----------------------------------------------------------------------------


def proof_status(lane: ProofLane) -> str:
    if lane.failed:
        return "FAIL"
    if lane.passed and not lane.bounded and not lane.unavailable:
        return "EXACT"
    if lane.passed or lane.bounded:
        return "PROVED/BOUNDED"
    return "NOT OBSERVED"


def print_report(audit: Audit) -> None:
    print("=" * 96)
    print(f"HOLISTIC CLOCKS PROOF LEDGER: {audit.scope}")
    print("=" * 96)
    if audit.rows == 0:
        print("No matching CLOCKS_V4 rows.")
        return

    print(f"Rows:                 {audit.rows:,}")
    print(f"Authoritative rows:   {audit.authoritative_rows:,}")
    print(f"Quarantined/superseded:{audit.superseded_rows:>10,d}")
    print(f"DB id:                {audit.first_db_id} -> {audit.last_db_id}")
    print(f"Database time:        {audit.first_ts} -> {audit.last_ts}")
    print(f"GNSS context time:    {audit.first_gnss} -> {audit.last_gnss}")
    print(f"Physical rebases:     {audit.physical_rebases:,}")
    print(f"Durable gaps:         {audit.durable_gaps:,} ({audit.durable_missing_rows:,} absent row(s))")
    print(f"Stats epoch births:   {audit.stats_resets:,}")
    print(f"Campaign splices:     {audit.campaign_splices:,}")

    print("\nDIRECT PROOF COVERAGE")
    print(f"  Firmware Better-Buckets window checks: {audit.firmware_ppb_window_checks:,}")
    print(f"  Firmware↔Pi checkpoint cross-checks:    {audit.firmware_pi_checkpoint_crosschecks:,}")
    print(f"  Recoverable Pi checkpoint rows:         {audit.pi_checkpoint_recoverable_rows:,}")
    print(f"  Warming/incomplete Pi checkpoint rows:  {audit.pi_checkpoint_warming_rows:,}")
    print(f"  Raw→checkpoint adjacent append checks:  {audit.append_delta_checks:,}")
    print(f"  Exact Welford one-step proofs:          {audit.exact_welford_steps:,}")
    print(f"  Serialization-bounded Welford proofs:   {audit.bounded_welford_steps:,}")
    print(f"  Exact DAC recurrence proofs:            {audit.exact_dac_steps:,}")
    print(f"  Exact GNSS_RAW recurrence proofs:       {audit.exact_gnss_raw_steps:,}")
    print(f"  OCXO TAU closure checks:                {audit.tau_checks:,}")
    print(f"  TOTAL PPB closure checks:               {audit.total_ppb_checks:,}")

    print("\nSTATISTICAL EPOCHS")
    for epoch, count in sorted(audit.epochs.items()):
        print(f"  reset_count={epoch:<6d} rows={count:,}")

    print("\nBOUNDARY SPECIES")
    if audit.boundaries:
        for name, count in audit.boundaries.most_common():
            print(f"  {name:<42s} {count:,}")
    else:
        print("  none")

    print("\nCAMPAIGNS OBSERVED")
    if audit.campaigns:
        for name, count in audit.campaigns.most_common():
            print(f"  {name:<28s} {count:,}")
    else:
        print("  none (ambient CLOCKS only)")

    print("\nRECOVERY EPISODES")
    if not audit.recovery_episodes:
        print("  none with explicit recovery_custody testimony")
    else:
        for generation, ep in audit.recovery_episodes.items():
            classes = ", ".join(f"{k}:{v}" for k, v in ep.classifications.items())
            print(
                f"  {generation}: {ep.lineage_status} reason={ep.reason or '?'} "
                f"rows={ep.first_db_id}..{ep.last_db_id} source={ep.source_db_id} "
                f"boundary={ep.boundary_db_id} stats={ep.source_reset}/{ep.source_update}"
                f"->{ep.boundary_reset}/{ep.boundary_update} ppb_recoverable={ep.ppb_recoverable}"
            )
            print(f"      classifications: {classes}")

    print("\nPROOF LEDGER")
    ordered = (
        "physical_row_identity",
        "physical_lifetime",
        "instrument_clockface",
        "alpha_statistics_lineage",
        "firmware_welford_lineage",
        "tau_total",
        "total_ppb",
        "firmware_checkpoint_structure",
        "firmware_ppb_window",
        "firmware_append_from_raw",
        "pi_resurrection_checkpoint",
        "pi_dac_lineage",
        "pi_gnss_raw_lineage",
        "alpha_resurrection",
        "recovery_custody",
        "campaign_row",
        "campaign_timeline",
        "restore_authority",
        "welford_geometry",
    )
    for name in ordered:
        lane = audit.proofs.get(name, ProofLane())
        print(
            f"  {name:<34s} {proof_status(lane):<15s} "
            f"exact={lane.passed:,} bounded={lane.bounded:,} unavailable={lane.unavailable:,} fail={lane.failed:,}"
        )

    print("\nRECOVERY / CUSTODY EVENTS")
    if audit.events:
        for name, count in sorted(audit.events.items()):
            print(f"  {name:<42s} {count:,}")
    else:
        print("  none")

    print("\nSTRICT FINDINGS")
    total = sum(audit.problems.values())
    if not audit.problems:
        print("  NONE")
    else:
        for name, count in sorted(audit.problems.items()):
            print(f"  {name:<42s} {count:,}")
    print(f"  {'TOTAL':<42s} {total:,}")

    if audit.examples:
        print("\nBOUNDED EVIDENCE")
        for kind, examples in audit.examples.items():
            print(f"  [{kind}]")
            for example in examples:
                print(f"    {example}")

    print("\nVERDICT")
    if total:
        print(
            f"  INVESTIGATE — {total:,} strict proof failure(s). At least one custody or "
            "scientific lineage theorem did not close."
        )
        return

    bounded = sum(lane.bounded for lane in audit.proofs.values())
    unavailable = sum(lane.unavailable for lane in audit.proofs.values())
    if bounded or unavailable:
        print(
            "  STRUCTURALLY CLEAN WITH EXPLICIT LIMITS — every attempted strict court closed, "
            "but one or more intervals contain acknowledged missing observations, warming "
            "checkpoint custody, serialization-limited recurrence, or an unobserved proof species. "
            "No missing scientific history was synthesized."
        )
        return

    print(
        "  HOLISTIC RESTORE EXACT — every observed physical, statistical, Better-Buckets, "
        "Pi-custody, campaign, and recovery theorem closed from producer-authored or literal "
        "durable testimony. No inferred or synthetic scientific history was required."
    )


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    if not args.all_rows and args.campaign is None:
        scope_report()
        return 0

    scope = args.campaign if args.campaign is not None else "ALL CLOCKS_V4"
    audit = Audit(scope=scope)
    for db_id, ts, payload in iter_rows(args):
        try:
            row = Row.parse(db_id, ts, payload)
        except Exception as exc:
            # A malformed canonical row is itself a strict scientific finding.  We
            # cannot manufacture a Row to continue through deeper courts, but the
            # stream can continue so the operator sees whether corruption is local.
            audit.rows += 1
            audit.problems["row_parse"] += 1
            audit.note("row_parse", f"db_id={db_id} ts={ts} {type(exc).__name__}: {exc}")
            continue
        process_row(audit, row)

    print_report(audit)
    return 1 if audit.problems else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
