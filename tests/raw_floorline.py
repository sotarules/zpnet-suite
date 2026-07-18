"""
ZPNet Raw FloorLine — per-row FloorLine estimator autopsy and anchor simulation.

Reads the rich ``forensics.floorline`` snapshots captured by the temporary
FloorLine instrumentation build.  The report is deliberately historical and
read-only: it does not modify TIMEBASE or recompute authoritative clock science.

The main table emits one line per TIMEBASE row per selected clock.  It shows the
single-minimum anchor used by firmware, the shape of the lowest-error cluster,
the resulting endpoint/interval errors, and a simulated alternative anchor.

Typical ZPNet usage:

    .zt raw_floorline ShakeOut1
    .zt raw_floorline ShakeOut1 --clock VCLOCK
    .zt raw_floorline ShakeOut1 --interesting-only
    .zt raw_floorline ShakeOut1 --anchor median8 --details
    .zt raw_floorline ShakeOut1 --anchor second --limit 100

Direct usage:

    python -m zpnet.tests.raw_floorline ShakeOut1
    python raw_floorline.py ShakeOut1 --clock OCXO1

Alternative anchors:

    second    second-lowest pre-shift fit error (e1)
    median4   median of e0..e3, i.e. average(e1, e2)
    median8   median of e0..e7, i.e. average(e3, e4) [default]
    mean8     arithmetic mean of e0..e7
    trimmed8  arithmetic mean of e1..e6

Column doctrine:

    e0/e1       lowest and second-lowest pre-shift fit errors, cycles
    gap         e1 - e0; large positive values identify an isolated minimum
    shift       firmware's actual downward intercept shift, cycles
    alt         selected alternative shift, cycles
    lift        alt - shift; how far the alternative raises the fitted line
    fl_b        captured FloorLine endpoint - observed endpoint, cycles
    alt_b       simulated alternative endpoint - observed endpoint, cycles
    fl_i        captured FloorLine interval - observed interval, cycles
    alt_i       simulated alternative interval - observed interval, cycles
    slp_i       slope-derived interval - observed interval, cycles
    rcn         reconstructed current pre-clamp endpoint minus captured endpoint;
                should be zero and validates the simulator against firmware math

The simulator uses process_interrupt's exact signed Q16 rounding and one-sided
endpoint clamp.  It reconstructs the endpoint from:

    base_dwt + round_q16(full_slope_q16 * edge_x
                         + pre_shift_intercept_q16
                         + selected_anchor_q16)

The report expects the temporary ``FLOORLINE_FORENSICS_V1`` schema used by the
ShakeOut1 campaign.  Rows without that schema remain visible as missing coverage
in the summary rather than being silently interpreted through older aliases.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


CLOCKS: Tuple[str, ...] = ("VCLOCK", "OCXO1", "OCXO2")
LANE_KEYS: Dict[str, str] = {
    "VCLOCK": "vclock",
    "OCXO1": "ocxo1",
    "OCXO2": "ocxo2",
}
ANCHOR_CHOICES: Tuple[str, ...] = (
    "second",
    "median4",
    "median8",
    "mean8",
    "trimmed8",
)
DEFAULT_ANCHOR = "median8"
DEFAULT_GAP_GATE_CYCLES = 2.0
DEFAULT_SHIFT_GATE_CYCLES = 4.0
DEFAULT_INTERVAL_GATE_CYCLES = 4
Q16_SCALE = 65536.0
U32_MASK = 0xFFFFFFFF


@dataclass
class RunningStats:
    n: int = 0
    mean: float = 0.0
    m2: float = 0.0
    sum_sq: float = 0.0
    min_val: float = math.inf
    max_val: float = -math.inf
    max_abs: float = 0.0

    def update(self, value: Optional[float]) -> None:
        if value is None or not math.isfinite(value):
            return
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (value - self.mean)
        self.sum_sq += value * value
        self.min_val = min(self.min_val, value)
        self.max_val = max(self.max_val, value)
        self.max_abs = max(self.max_abs, abs(value))

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0

    @property
    def rms(self) -> float:
        return math.sqrt(self.sum_sq / self.n) if self.n else 0.0


@dataclass
class PairStats:
    n: int = 0
    mean_x: float = 0.0
    mean_y: float = 0.0
    sxx: float = 0.0
    syy: float = 0.0
    sxy: float = 0.0

    def update(self, x: Optional[float], y: Optional[float]) -> None:
        if x is None or y is None or not math.isfinite(x) or not math.isfinite(y):
            return
        self.n += 1
        dx = x - self.mean_x
        dy = y - self.mean_y
        self.mean_x += dx / self.n
        self.mean_y += dy / self.n
        self.sxx += dx * (x - self.mean_x)
        self.syy += dy * (y - self.mean_y)
        self.sxy += dx * (y - self.mean_y)

    @property
    def correlation(self) -> Optional[float]:
        if self.n < 2 or self.sxx <= 0.0 or self.syy <= 0.0:
            return None
        return self.sxy / math.sqrt(self.sxx * self.syy)


# -----------------------------------------------------------------------------
# Database and schema helpers
# -----------------------------------------------------------------------------


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                NULLIF(payload->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_count', '')::bigint,
                NULLIF(payload->'payload'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_count', '')::bigint
            ) ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            result.append(payload)
    return result


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    obj = root.get("forensics")
    if isinstance(obj, dict):
        return obj
    frag = _frag(rec)
    obj = frag.get("forensics")
    return obj if isinstance(obj, dict) else {}


def _floorline(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = _frag(rec)
    forensic = _forensics(rec)
    for candidate in (
        forensic.get("floorline"),
        root.get("floorline"),
        frag.get("floorline"),
    ):
        if isinstance(candidate, dict):
            return candidate
    return {}


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _as_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _as_bool(value: Any) -> Optional[bool]:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("true", "t", "yes", "y", "1"):
            return True
        if text in ("false", "f", "no", "n", "0"):
            return False
    return None


def _as_str(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value)
    return text if text else None


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        result = _as_int(value)
        if result is not None:
            return result
    return None


def _first_bool(*values: Any) -> Optional[bool]:
    for value in values:
        result = _as_bool(value)
        if result is not None:
            return result
    return None


def _first_str(*values: Any) -> Optional[str]:
    for value in values:
        result = _as_str(value)
        if result is not None:
            return result
    return None


def pps_count_from_schema(rec: Dict[str, Any]) -> Optional[int]:
    root = _root(rec)
    frag = _frag(rec)
    forensic = _forensics(rec)
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_vclock_count"),
        forensic.get("teensy_pps_vclock_count"),
        root.get("teensy_pps_count"),
        frag.get("teensy_pps_count"),
        forensic.get("teensy_pps_count"),
    )


# -----------------------------------------------------------------------------
# Firmware-equivalent arithmetic and alternative anchors
# -----------------------------------------------------------------------------


def q16_cycles(value: Optional[int]) -> Optional[float]:
    return None if value is None else float(value) / Q16_SCALE


def round_q16_signed(value: int) -> int:
    """Mirror lower_env_round_q16_u32() as a signed Python integer."""
    if value >= 0:
        return (value + 32768) >> 16
    return -(((-value) + 32768) >> 16)


def delta_u32(now: int, previous: int) -> int:
    return (now - previous) & U32_MASK


def signed_delta_u32(from_dwt: int, to_dwt: int) -> int:
    delta = (to_dwt - from_dwt) & U32_MASK
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def anchor_q16(errors: Sequence[int], strategy: str) -> Optional[int]:
    if not errors:
        return None
    ordered = sorted(int(value) for value in errors)

    if strategy == "second":
        return ordered[1] if len(ordered) >= 2 else ordered[0]
    if strategy == "median4":
        sample = ordered[:4]
        if len(sample) < 4:
            return _median_q16(sample)
        return _average_int(sample[1], sample[2])
    if strategy == "median8":
        sample = ordered[:8]
        if len(sample) < 8:
            return _median_q16(sample)
        return _average_int(sample[3], sample[4])
    if strategy == "mean8":
        sample = ordered[:8]
        return int(round(sum(sample) / len(sample)))
    if strategy == "trimmed8":
        sample = ordered[:8]
        if len(sample) >= 4:
            sample = sample[1:-1]
        return int(round(sum(sample) / len(sample)))
    raise ValueError(f"unknown anchor strategy {strategy!r}")


def _average_int(a: int, b: int) -> int:
    # Symmetric nearest-integer average.  Q16 values can be negative.
    return int(round((a + b) / 2.0))


def _median_q16(values: Sequence[int]) -> Optional[int]:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) & 1:
        return ordered[mid]
    return _average_int(ordered[mid - 1], ordered[mid])


def simulate_endpoint(
    *,
    base_dwt: Optional[int],
    full_slope_q16: Optional[int],
    edge_x: Optional[int],
    pre_shift_intercept_q16: Optional[int],
    selected_anchor_q16: Optional[int],
    observed_dwt: Optional[int],
) -> Tuple[Optional[int], Optional[int], Optional[bool]]:
    """Return (pre-clamp endpoint, final endpoint, clamped)."""
    required = (
        base_dwt,
        full_slope_q16,
        edge_x,
        pre_shift_intercept_q16,
        selected_anchor_q16,
        observed_dwt,
    )
    if any(value is None for value in required):
        return None, None, None

    assert base_dwt is not None
    assert full_slope_q16 is not None
    assert edge_x is not None
    assert pre_shift_intercept_q16 is not None
    assert selected_anchor_q16 is not None
    assert observed_dwt is not None

    inferred_delta_q16 = (
        full_slope_q16 * edge_x
        + pre_shift_intercept_q16
        + selected_anchor_q16
    )
    before_clamp = (base_dwt + round_q16_signed(inferred_delta_q16)) & U32_MASK
    bias = signed_delta_u32(observed_dwt, before_clamp)
    if bias > 0:
        return before_clamp, observed_dwt, True
    return before_clamp, before_clamp, False


# -----------------------------------------------------------------------------
# Row extraction and simulation
# -----------------------------------------------------------------------------


def lowest_errors_q16(lane_obj: Dict[str, Any]) -> List[int]:
    errors_obj = _nested_get(lane_obj, "fit", "lowest_pre_shift_fit_errors")
    if not isinstance(errors_obj, dict):
        return []
    count = _first_int(errors_obj.get("count"), 8) or 0
    result: List[int] = []
    for index in range(min(max(count, 0), 32)):
        value = _as_int(errors_obj.get(f"e{index}_q16_cycles"))
        if value is not None:
            result.append(value)
    return result


def extract_lane_snapshot(
    rec: Dict[str, Any],
    lane: str,
    alternative: str,
) -> Dict[str, Any]:
    fl_root = _floorline(rec)
    lane_key = LANE_KEYS[lane]
    lane_obj = fl_root.get(lane_key)
    lane_obj = lane_obj if isinstance(lane_obj, dict) else {}

    window = lane_obj.get("window") if isinstance(lane_obj.get("window"), dict) else {}
    endpoint = lane_obj.get("endpoint") if isinstance(lane_obj.get("endpoint"), dict) else {}
    fit = lane_obj.get("fit") if isinstance(lane_obj.get("fit"), dict) else {}
    interval = lane_obj.get("interval") if isinstance(lane_obj.get("interval"), dict) else {}
    state = lane_obj.get("state") if isinstance(lane_obj.get("state"), dict) else {}

    errors = lowest_errors_q16(lane_obj)
    chosen_shift_q16 = _as_int(fit.get("lower_envelope_shift_q16_cycles"))
    alt_shift_q16 = anchor_q16(errors, alternative)

    base_dwt = _as_int(window.get("active_base_dwt"))
    full_slope_q16 = _as_int(fit.get("slope_q16_cycles_per_sample"))
    edge_x = _as_int(fit.get("edge_x"))
    pre_intercept_q16 = _as_int(fit.get("fit_intercept_pre_shift_q16_cycles"))
    observed_dwt = _as_int(endpoint.get("observed_dwt"))

    current_before, current_simulated, current_clamped = simulate_endpoint(
        base_dwt=base_dwt,
        full_slope_q16=full_slope_q16,
        edge_x=edge_x,
        pre_shift_intercept_q16=pre_intercept_q16,
        selected_anchor_q16=chosen_shift_q16,
        observed_dwt=observed_dwt,
    )
    alt_before, alt_endpoint, alt_clamped = simulate_endpoint(
        base_dwt=base_dwt,
        full_slope_q16=full_slope_q16,
        edge_x=edge_x,
        pre_shift_intercept_q16=pre_intercept_q16,
        selected_anchor_q16=alt_shift_q16,
        observed_dwt=observed_dwt,
    )

    captured_before = _as_int(endpoint.get("inferred_dwt_before_clamp"))
    captured_endpoint = _as_int(endpoint.get("inferred_dwt"))
    captured_bias = _first_int(
        endpoint.get("inferred_minus_observed_cycles"),
        signed_delta_u32(observed_dwt, captured_endpoint)
        if observed_dwt is not None and captured_endpoint is not None
        else None,
    )
    alt_bias = (
        signed_delta_u32(observed_dwt, alt_endpoint)
        if observed_dwt is not None and alt_endpoint is not None
        else None
    )

    e0 = errors[0] if len(errors) >= 1 else None
    e1 = errors[1] if len(errors) >= 2 else None
    gap_q16 = (e1 - e0) if e0 is not None and e1 is not None else None
    lift_q16 = (
        alt_shift_q16 - chosen_shift_q16
        if alt_shift_q16 is not None and chosen_shift_q16 is not None
        else None
    )

    return {
        "pps_count": pps_count_from_schema(rec),
        "lane": lane,
        "schema": _as_str(fl_root.get("schema")),
        "root_available": bool(fl_root),
        "available": _first_bool(lane_obj.get("available"), bool(lane_obj)),
        "aligned": _as_bool(lane_obj.get("consumer_sequence_aligned")),
        "expected_sequence": _as_int(lane_obj.get("consumer_expected_sequence")),
        "sequence": _as_int(lane_obj.get("sequence")),
        "valid": _as_bool(lane_obj.get("valid")),
        "candidate_present": _as_bool(lane_obj.get("candidate_present")),
        "fallback_used": _as_bool(lane_obj.get("fallback_used")),
        "publish_source": _as_str(lane_obj.get("publish_source")),
        "publish_reason": _as_str(lane_obj.get("publish_reason")),
        "sample_count": _as_int(window.get("sample_count")),
        "sample_count_before": _as_int(window.get("sample_count_before_closing_sample")),
        "closing_sample_x": _as_int(window.get("closing_sample_x")),
        "bucket_count": _as_int(window.get("selected_bucket_count")),
        "overflow_before": _as_bool(window.get("overflow_reset_before_sample")),
        "overflow_count_before": _as_int(window.get("overflow_reset_count_before")),
        "overflow_count_after": _as_int(window.get("overflow_reset_count_after")),
        "invalid_window_count": _as_int(state.get("invalid_window_count")),
        "update_count": _as_int(state.get("update_count")),
        "observed_dwt": observed_dwt,
        "captured_before_clamp": captured_before,
        "captured_endpoint": captured_endpoint,
        "captured_clamped": _as_bool(endpoint.get("endpoint_clamped_to_observed")),
        "captured_bias": captured_bias,
        "current_reconstructed_before": current_before,
        "current_reconstructed_endpoint": current_simulated,
        "current_reconstructed_clamped": current_clamped,
        "reconstruction_delta": (
            signed_delta_u32(captured_before, current_before)
            if captured_before is not None and current_before is not None
            else None
        ),
        "alt_before_clamp": alt_before,
        "alt_endpoint": alt_endpoint,
        "alt_clamped": alt_clamped,
        "alt_bias": alt_bias,
        "base_dwt": base_dwt,
        "edge_x": edge_x,
        "full_slope_q16": full_slope_q16,
        "slope_derived_interval": _as_int(fit.get("slope_derived_interval_cycles")),
        "pre_intercept_q16": pre_intercept_q16,
        "post_intercept_q16": _as_int(fit.get("fit_intercept_post_shift_q16_cycles")),
        "chosen_shift_q16": chosen_shift_q16,
        "alt_shift_q16": alt_shift_q16,
        "lift_q16": lift_q16,
        "errors_q16": errors,
        "e0_q16": e0,
        "e1_q16": e1,
        "gap_q16": gap_q16,
        "reported_observed_interval": _as_int(interval.get("observed_cycles")),
        "reported_floorline_interval": _as_int(interval.get("inferred_cycles")),
        "reported_floorline_interval_error": _as_int(
            interval.get("inferred_minus_observed_cycles")
        ),
        "candidate_interval_error": _as_int(interval.get("candidate_interval_error_cycles")),
    }


def enrich_intervals(rows: List[Dict[str, Any]]) -> None:
    previous: Dict[str, Optional[Dict[str, Any]]] = {lane: None for lane in CLOCKS}

    for row in rows:
        lane = row["lane"]
        prev = previous[lane]
        sequence = row.get("sequence")
        contiguous = bool(
            prev is not None
            and sequence is not None
            and prev.get("sequence") is not None
            and sequence == prev["sequence"] + 1
        )
        row["sequence_contiguous"] = contiguous

        observed_interval = row.get("reported_observed_interval")
        actual_interval = row.get("reported_floorline_interval")
        if contiguous and row.get("observed_dwt") is not None and prev.get("observed_dwt") is not None:
            row["derived_observed_interval"] = delta_u32(
                row["observed_dwt"], prev["observed_dwt"]
            )
            if observed_interval is None:
                observed_interval = row["derived_observed_interval"]
        else:
            row["derived_observed_interval"] = None

        if contiguous and row.get("captured_endpoint") is not None and prev.get("captured_endpoint") is not None:
            row["derived_floorline_interval"] = delta_u32(
                row["captured_endpoint"], prev["captured_endpoint"]
            )
            if actual_interval is None:
                actual_interval = row["derived_floorline_interval"]
        else:
            row["derived_floorline_interval"] = None

        if contiguous and row.get("alt_endpoint") is not None and prev.get("alt_endpoint") is not None:
            row["alt_interval"] = delta_u32(row["alt_endpoint"], prev["alt_endpoint"])
        else:
            row["alt_interval"] = None

        row["observed_interval"] = observed_interval
        row["actual_interval"] = actual_interval
        row["actual_interval_error"] = _first_int(
            row.get("reported_floorline_interval_error"),
            actual_interval - observed_interval
            if actual_interval is not None and observed_interval is not None
            else None,
        )
        row["alt_interval_error"] = (
            row["alt_interval"] - observed_interval
            if row.get("alt_interval") is not None and observed_interval is not None
            else None
        )
        row["slope_interval_error"] = (
            row["slope_derived_interval"] - observed_interval
            if row.get("slope_derived_interval") is not None and observed_interval is not None
            else None
        )

        if contiguous and row.get("captured_bias") is not None and prev.get("captured_bias") is not None:
            row["bias_difference"] = row["captured_bias"] - prev["captured_bias"]
        else:
            row["bias_difference"] = None

        if contiguous and row.get("alt_bias") is not None and prev.get("alt_bias") is not None:
            row["alt_bias_difference"] = row["alt_bias"] - prev["alt_bias"]
        else:
            row["alt_bias_difference"] = None

        row["actual_interval_identity_error"] = (
            row["actual_interval_error"] - row["bias_difference"]
            if row.get("actual_interval_error") is not None and row.get("bias_difference") is not None
            else None
        )
        row["alt_interval_identity_error"] = (
            row["alt_interval_error"] - row["alt_bias_difference"]
            if row.get("alt_interval_error") is not None and row.get("alt_bias_difference") is not None
            else None
        )

        previous[lane] = row


def collect_rows(
    records: Iterable[Dict[str, Any]],
    clocks: Tuple[str, ...],
    alternative: str,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for rec in records:
        for lane in clocks:
            rows.append(extract_lane_snapshot(rec, lane, alternative))
    enrich_intervals(rows)
    return rows


# -----------------------------------------------------------------------------
# Filtering and presentation
# -----------------------------------------------------------------------------


def normalize_clock(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    text = value.strip().upper()
    aliases = {
        "V": "VCLOCK",
        "VC": "VCLOCK",
        "VCLOCK": "VCLOCK",
        "O1": "OCXO1",
        "OCXO1": "OCXO1",
        "O2": "OCXO2",
        "OCXO2": "OCXO2",
    }
    if text not in aliases:
        raise ValueError(f"unknown clock {value!r}; expected VCLOCK, OCXO1, or OCXO2")
    return aliases[text]


def selected_clocks(clock: Optional[str]) -> Tuple[str, ...]:
    return CLOCKS if clock is None else (clock,)


def is_interesting(
    row: Dict[str, Any],
    gap_gate: float,
    shift_gate: float,
    interval_gate: int,
) -> bool:
    if not row.get("root_available") or not row.get("available"):
        return True
    if row.get("aligned") is False or row.get("valid") is False:
        return True
    if row.get("fallback_used") is True or row.get("overflow_before") is True:
        return True
    if row.get("reconstruction_delta") not in (None, 0):
        return True

    gap = q16_cycles(row.get("gap_q16"))
    shift = q16_cycles(row.get("chosen_shift_q16"))
    if gap is not None and gap >= gap_gate:
        return True
    if shift is not None and abs(shift) >= shift_gate:
        return True
    for key in ("captured_bias", "actual_interval_error"):
        value = row.get(key)
        if value is not None and abs(value) >= interval_gate:
            return True
    return False


def fmt_int(value: Optional[int], width: int, signed: bool = False) -> str:
    if value is None:
        text = "---"
    else:
        text = f"{value:+d}" if signed else f"{value:d}"
    return f"{text:>{width}s}"


def fmt_float(value: Optional[float], width: int, decimals: int = 2, signed: bool = True) -> str:
    if value is None:
        text = "---"
    else:
        text = f"{value:+.{decimals}f}" if signed else f"{value:.{decimals}f}"
    return f"{text:>{width}s}"


def fmt_flag(value: Optional[bool], true_char: str = "Y", false_char: str = "N") -> str:
    if value is None:
        return "-"
    return true_char if value else false_char


def table_header() -> Tuple[str, str]:
    labels = (
        ("pps", 6),
        ("lane", 6),
        ("seq", 6),
        ("A", 1),
        ("V", 1),
        ("smp", 4),
        ("bkt", 3),
        ("O", 1),
        ("e0", 8),
        ("e1", 8),
        ("gap", 7),
        ("shift", 8),
        ("alt", 8),
        ("lift", 7),
        ("fl_b", 6),
        ("alt_b", 6),
        ("fl_i", 6),
        ("alt_i", 6),
        ("slp_i", 6),
        ("rcn", 4),
        ("reason", 12),
    )
    header = "  ".join(f"{name:>{width}s}" for name, width in labels)
    sep = "  ".join("─" * width for _, width in labels)
    return header, sep


def row_line(row: Dict[str, Any]) -> str:
    reason = row.get("publish_reason") or "---"
    if len(reason) > 12:
        reason = reason[:12]
    return "  ".join(
        (
            fmt_int(row.get("pps_count"), 6),
            f"{row['lane']:>6s}",
            fmt_int(row.get("sequence"), 6),
            fmt_flag(row.get("aligned")),
            fmt_flag(row.get("valid")),
            fmt_int(row.get("sample_count"), 4),
            fmt_int(row.get("bucket_count"), 3),
            fmt_flag(row.get("overflow_before"), "!", "."),
            fmt_float(q16_cycles(row.get("e0_q16")), 8),
            fmt_float(q16_cycles(row.get("e1_q16")), 8),
            fmt_float(q16_cycles(row.get("gap_q16")), 7),
            fmt_float(q16_cycles(row.get("chosen_shift_q16")), 8),
            fmt_float(q16_cycles(row.get("alt_shift_q16")), 8),
            fmt_float(q16_cycles(row.get("lift_q16")), 7),
            fmt_int(row.get("captured_bias"), 6, signed=True),
            fmt_int(row.get("alt_bias"), 6, signed=True),
            fmt_int(row.get("actual_interval_error"), 6, signed=True),
            fmt_int(row.get("alt_interval_error"), 6, signed=True),
            fmt_int(row.get("slope_interval_error"), 6, signed=True),
            fmt_int(row.get("reconstruction_delta"), 4, signed=True),
            f"{reason:>12s}",
        )
    )


def detail_lines(row: Dict[str, Any], alternative: str) -> List[str]:
    errors = row.get("errors_q16") or []
    errors_text = " ".join(
        f"e{i}={q16_cycles(value):+.3f}" for i, value in enumerate(errors)
    ) or "---"
    lines = [f"      errors: {errors_text} cycles"]
    lines.append(
        "      window: "
        f"before={row.get('sample_count_before', '---')} "
        f"x={row.get('closing_sample_x', '---')} "
        f"samples={row.get('sample_count', '---')} "
        f"buckets={row.get('bucket_count', '---')} "
        f"overflow_before={row.get('overflow_before', '---')} "
        f"overflow_count={row.get('overflow_count_before', '---')}→"
        f"{row.get('overflow_count_after', '---')} "
        f"invalid_windows={row.get('invalid_window_count', '---')}"
    )
    lines.append(
        "      endpoint: "
        f"obs={row.get('observed_dwt', '---')} "
        f"captured_pre={row.get('captured_before_clamp', '---')} "
        f"captured={row.get('captured_endpoint', '---')} "
        f"reconstructed={row.get('current_reconstructed_before', '---')} "
        f"{alternative}_pre={row.get('alt_before_clamp', '---')} "
        f"{alternative}={row.get('alt_endpoint', '---')}"
    )
    lines.append(
        "      intervals: "
        f"observed={row.get('observed_interval', '---')} "
        f"floorline={row.get('actual_interval', '---')} "
        f"{alternative}={row.get('alt_interval', '---')} "
        f"slope={row.get('slope_derived_interval', '---')} "
        f"identity(fl)={row.get('actual_interval_identity_error', '---')} "
        f"identity(alt)={row.get('alt_interval_identity_error', '---')}"
    )
    return lines


def print_stats(label: str, stats: RunningStats, decimals: int = 3) -> None:
    if stats.n == 0:
        print(f"  {label:<46s} no samples")
        return
    print(
        f"  {label:<46s} "
        f"n={stats.n:>5,d}  "
        f"mean={stats.mean:+10,.{decimals}f}  "
        f"sd={stats.stddev:9,.{decimals}f}  "
        f"rms={stats.rms:9,.{decimals}f}  "
        f"min={stats.min_val:+9,.{decimals}f}  "
        f"max={stats.max_val:+9,.{decimals}f}  "
        f"|max|={stats.max_abs:9,.{decimals}f}"
    )


def summarize(
    rows: List[Dict[str, Any]],
    clocks: Tuple[str, ...],
    alternative: str,
    gap_gate: float,
) -> None:
    print()
    print("Coverage and simulation summary")
    print("═══════════════════════════════")

    for lane in clocks:
        lane_rows = [row for row in rows if row["lane"] == lane]
        available = sum(bool(row.get("available")) for row in lane_rows)
        aligned = sum(row.get("aligned") is True for row in lane_rows)
        valid = sum(row.get("valid") is True for row in lane_rows)
        overflow = sum(row.get("overflow_before") is True for row in lane_rows)
        reconstruction_bad = sum(
            row.get("reconstruction_delta") not in (None, 0) for row in lane_rows
        )
        chosen_equals_e0 = sum(
            row.get("chosen_shift_q16") is not None
            and row.get("chosen_shift_q16") == row.get("e0_q16")
            for row in lane_rows
        )
        isolated = sum(
            (q16_cycles(row.get("gap_q16")) or -math.inf) >= gap_gate
            for row in lane_rows
        )

        shift_stats = RunningStats()
        alt_stats = RunningStats()
        gap_stats = RunningStats()
        lift_stats = RunningStats()
        actual_bias_stats = RunningStats()
        alt_bias_stats = RunningStats()
        actual_interval_stats = RunningStats()
        alt_interval_stats = RunningStats()
        slope_interval_stats = RunningStats()
        interval_identity_stats = RunningStats()
        alt_identity_stats = RunningStats()
        shift_bias_corr = PairStats()
        gap_lift_corr = PairStats()

        endpoint_improved = 0
        endpoint_compared = 0
        interval_improved = 0
        interval_compared = 0

        for row in lane_rows:
            chosen_shift = q16_cycles(row.get("chosen_shift_q16"))
            alt_shift = q16_cycles(row.get("alt_shift_q16"))
            gap = q16_cycles(row.get("gap_q16"))
            lift = q16_cycles(row.get("lift_q16"))
            actual_bias = _as_float(row.get("captured_bias"))
            alt_bias = _as_float(row.get("alt_bias"))
            actual_ierr = _as_float(row.get("actual_interval_error"))
            alt_ierr = _as_float(row.get("alt_interval_error"))
            slope_ierr = _as_float(row.get("slope_interval_error"))

            shift_stats.update(chosen_shift)
            alt_stats.update(alt_shift)
            gap_stats.update(gap)
            lift_stats.update(lift)
            actual_bias_stats.update(actual_bias)
            alt_bias_stats.update(alt_bias)
            actual_interval_stats.update(actual_ierr)
            alt_interval_stats.update(alt_ierr)
            slope_interval_stats.update(slope_ierr)
            interval_identity_stats.update(_as_float(row.get("actual_interval_identity_error")))
            alt_identity_stats.update(_as_float(row.get("alt_interval_identity_error")))
            shift_bias_corr.update(chosen_shift, actual_bias)
            gap_lift_corr.update(gap, lift)

            if actual_bias is not None and alt_bias is not None:
                endpoint_compared += 1
                if abs(alt_bias) < abs(actual_bias):
                    endpoint_improved += 1
            if actual_ierr is not None and alt_ierr is not None:
                interval_compared += 1
                if abs(alt_ierr) < abs(actual_ierr):
                    interval_improved += 1

        print()
        print(f"{lane}")
        print("─" * len(lane))
        print(
            f"  rows={len(lane_rows):,}  available={available:,}  aligned={aligned:,}  "
            f"valid={valid:,}  overflow-at-close={overflow:,}  "
            f"isolated-e0(gap≥{gap_gate:g})={isolated:,}"
        )
        print(
            f"  chosen-shift==e0: {chosen_equals_e0:,}/{len(lane_rows):,}  "
            f"simulator reconstruction mismatches: {reconstruction_bad:,}"
        )
        print_stats("Firmware chosen shift", shift_stats)
        print_stats(f"Alternative shift ({alternative})", alt_stats)
        print_stats("e1 - e0 isolation gap", gap_stats)
        print_stats("Alternative line lift", lift_stats)
        print_stats("Captured endpoint bias", actual_bias_stats)
        print_stats(f"Simulated {alternative} endpoint bias", alt_bias_stats)
        print_stats("Captured FloorLine interval error", actual_interval_stats)
        print_stats(f"Simulated {alternative} interval error", alt_interval_stats)
        print_stats("Slope-derived interval error", slope_interval_stats)
        print_stats("Captured interval identity error", interval_identity_stats)
        print_stats(f"Simulated {alternative} identity error", alt_identity_stats)

        endpoint_pct = (
            100.0 * endpoint_improved / endpoint_compared if endpoint_compared else 0.0
        )
        interval_pct = (
            100.0 * interval_improved / interval_compared if interval_compared else 0.0
        )
        print(
            f"  |endpoint bias| improved: {endpoint_improved:,}/{endpoint_compared:,} "
            f"({endpoint_pct:.1f}%)"
        )
        print(
            f"  |interval error| improved: {interval_improved:,}/{interval_compared:,} "
            f"({interval_pct:.1f}%)"
        )
        corr = shift_bias_corr.correlation
        print(
            "  corr(chosen shift, endpoint bias): "
            + (f"{corr:+.6f}" if corr is not None else "---")
        )
        corr = gap_lift_corr.correlation
        print(
            "  corr(e0 isolation gap, alternative lift): "
            + (f"{corr:+.6f}" if corr is not None else "---")
        )


def analyze(
    campaign: str,
    *,
    limit: int,
    clock: Optional[str],
    alternative: str,
    interesting_only: bool,
    details: bool,
    gap_gate: float,
    shift_gate: float,
    interval_gate: int,
) -> None:
    records = fetch_timebase(campaign)
    if not records:
        print(f"No TIMEBASE rows for campaign {campaign!r}")
        return

    if limit > 0:
        records = records[:limit]

    clocks = selected_clocks(clock)
    rows = collect_rows(records, clocks, alternative)
    for row in rows:
        row["interesting"] = is_interesting(
            row,
            gap_gate=gap_gate,
            shift_gate=shift_gate,
            interval_gate=interval_gate,
        )

    displayed = [row for row in rows if not interesting_only or row["interesting"]]

    print(
        f"Campaign: {campaign}  "
        f"({len(records):,} TIMEBASE rows, {len(rows):,} lane rows, "
        f"anchor={alternative})"
    )
    print()
    print("FloorLine lower-envelope anchor audit")
    print("═════════════════════════════════")
    print("  One output row per TIMEBASE row per selected clock.")
    print("  shift is firmware's single-minimum anchor; alt is the simulated replacement.")
    print("  fl_b/alt_b are endpoint biases; fl_i/alt_i are interval errors.")
    print("  slp_i is the independent fitted-slope interval error.")
    print("  rcn validates the simulator against the captured current endpoint; expect 0.")
    print(
        f"  Interesting gates: e1-e0 gap ≥ {gap_gate:g}, "
        f"|shift| ≥ {shift_gate:g}, |bias/interval error| ≥ {interval_gate:d} cycles."
    )
    if interesting_only:
        print("  Interesting-only mode: ordinary rows are suppressed; summary covers all rows.")
    print()

    header, separator = table_header()
    print(header)
    print(separator)
    for row in displayed:
        print(row_line(row))
        if details and row.get("interesting"):
            for line in detail_lines(row, alternative):
                print(line)

    print()
    print(f"Lane rows shown:    {len(displayed):,}")
    print(f"Lane rows analyzed: {len(rows):,}")
    missing_schema = sum(row.get("schema") != "FLOORLINE_FORENSICS_V1" for row in rows)
    print(f"Missing/other schema lane rows: {missing_schema:,}")

    summarize(rows, clocks, alternative, gap_gate)

    print()
    print("Interpretation guide")
    print("════════════════════")
    print("  • Large gap with large positive lift means one isolated e0 pulled the whole line down.")
    print("  • rcn=0 proves the report reconstructed firmware's current endpoint exactly.")
    print("  • Compare fl_b vs alt_b to judge endpoint quantization/latency relief.")
    print("  • Compare fl_i vs alt_i across the campaign; interval noise is the change in")
    print("    endpoint bias from one row to the next.")
    print("  • Compare alt_i with slp_i.  Agreement suggests the slope fit is healthy and")
    print("    the vertical anchor—not the regression slope—is the dominant defect.")
    print("  • Overflow-at-close and sequence/alignment failures are mechanical custody")
    print("    problems and should be separated from anchor-statistics experiments.")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="raw_floorline",
        description="Analyze FLOORLINE_FORENSICS_V1 rows and simulate robust anchors.",
    )
    parser.add_argument("campaign", help="TIMEBASE campaign name, e.g. ShakeOut1")
    parser.add_argument(
        "extras",
        nargs="*",
        help="optional positional limit and/or clock for .zt compatibility",
    )
    parser.add_argument("--limit", type=int, default=None, help="maximum TIMEBASE rows")
    parser.add_argument("--clock", help="VCLOCK, OCXO1, or OCXO2")
    parser.add_argument(
        "--anchor",
        choices=ANCHOR_CHOICES,
        default=DEFAULT_ANCHOR,
        help=f"alternative vertical anchor (default: {DEFAULT_ANCHOR})",
    )
    parser.add_argument(
        "--interesting-only",
        action="store_true",
        help="show only rows crossing a diagnostic gate",
    )
    parser.add_argument(
        "--details",
        action="store_true",
        help="print all eight errors and endpoint/interval provenance for interesting rows",
    )
    parser.add_argument(
        "--gap-gate",
        type=float,
        default=DEFAULT_GAP_GATE_CYCLES,
        help=f"isolated-minimum e1-e0 gate in cycles (default: {DEFAULT_GAP_GATE_CYCLES:g})",
    )
    parser.add_argument(
        "--shift-gate",
        type=float,
        default=DEFAULT_SHIFT_GATE_CYCLES,
        help=f"absolute chosen-shift gate in cycles (default: {DEFAULT_SHIFT_GATE_CYCLES:g})",
    )
    parser.add_argument(
        "--interval-gate",
        type=int,
        default=DEFAULT_INTERVAL_GATE_CYCLES,
        help=f"absolute endpoint/interval-error gate (default: {DEFAULT_INTERVAL_GATE_CYCLES})",
    )

    args = parser.parse_args(argv)
    positional_limit: Optional[int] = None
    positional_clock: Optional[str] = None
    for item in args.extras:
        try:
            value = int(item)
        except ValueError:
            if positional_clock is not None:
                parser.error(f"unexpected positional argument {item!r}")
            positional_clock = item
        else:
            if positional_limit is not None:
                parser.error(f"unexpected second positional limit {item!r}")
            positional_limit = value

    if args.limit is not None and positional_limit is not None:
        parser.error("specify the row limit either positionally or with --limit, not both")
    if args.clock is not None and positional_clock is not None:
        parser.error("specify the clock either positionally or with --clock, not both")

    args.limit = args.limit if args.limit is not None else (positional_limit or 0)
    raw_clock = args.clock if args.clock is not None else positional_clock
    try:
        args.clock = normalize_clock(raw_clock)
    except ValueError as exc:
        parser.error(str(exc))

    if args.limit < 0:
        parser.error("--limit must be zero or positive")
    if args.gap_gate < 0.0:
        parser.error("--gap-gate must be non-negative")
    if args.shift_gate < 0.0:
        parser.error("--shift-gate must be non-negative")
    if args.interval_gate < 0:
        parser.error("--interval-gate must be non-negative")
    return args

def main() -> None:
    args = parse_args()
    analyze(
        args.campaign,
        limit=args.limit,
        clock=args.clock,
        alternative=args.anchor,
        interesting_only=args.interesting_only,
        details=args.details,
        gap_gate=args.gap_gate,
        shift_gate=args.shift_gate,
        interval_gate=args.interval_gate,
    )


if __name__ == "__main__":
    main()
