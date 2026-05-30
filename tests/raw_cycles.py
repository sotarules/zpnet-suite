"""
ZPNet Raw Cycles — residual-focused prediction + OCXO service audit.

Reads TIMEBASE rows for a campaign and prints a compact one-second DWT-cycle
audit for four rails:

  • PPS    — static prediction audit: previous physical PPS DWT interval
  • VCLOCK — static prediction audit + VCLOCK Alpha-forensics/counter identity
  • OCXO1  — static prediction audit + OCXO compare-service forensics
  • OCXO2  — static prediction audit + OCXO compare-service forensics

The report is intentionally focused on the residual question.  The main table
shows:

  actual cycles, static prediction cycles, residual cycles

for PPS, VCLOCK, OCXO1, and OCXO2.  For OCXO lanes it also shows the newly
published compare-service diagnostics that explain whether the DWT-at-edge was
captured at the actual compare target or after ISR service latency:

  svc    = signed service offset in OCXO ticks
           negative => early/pre-target service
           zero     => exactly at target
           positive => late/on-or-after-target service

  svcΔ   = current svc minus previous svc for the same lane

  late   = interpreted late ticks (0 when service was early)
  early  = interpreted early ticks (0 when service was on/after target)
  cls    = service class: OK=on/after target, ERLY=early/pre-target, FIRQ=false IRQ
  cdelta = Alpha-forensics counter32 delta since previous event

A flat cdelta of 10,000,000 proves the synthetic target ladder is intact.
For VCLOCK, v_cΔ distinguishes a missing/bookend-skipped event (20,000,000)
from a DWT/ledger/reporting anomaly (10,000,000 with an ugly DWT interval).

The report reads TIMEBASE_FRAGMENT_V2's nested clock sections first, while
retaining flat-key fallbacks for older campaigns.  It also reads the
experimental linear-regression diagnostics published through Alpha/Beta.  The
main table prints each lane's traditional interval beside the
regression-inferred interval:

  *_act  = traditional firmware/static-prediction interval
  *_lr   = interval between consecutive regression-inferred DWT endpoints
  *_lrΔ  = regression interval minus traditional interval

For OCXO lanes, raw_cycles still emulates the older service-corrected endpoint
experiment when ocxo*_forensics_service_corrected_dwt_at_event is present.
The o*_c_res columns are retained as the scalar-correction comparison while
*_lr columns show the newer linear-regression diagnostic surface.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    python raw_cycles.py <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


DEFAULT_DWT_EXPECTED_PER_PPS = 1_008_000_000
EXPECTED_OCXO_CDELTA = 10_000_000
EXCURSION_THRESHOLD_CYCLES = 100


class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        self.min_val = min(self.min_val, x)
        self.max_val = max(self.max_val, x)

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


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


def _prediction(frag: Dict[str, Any]) -> Dict[str, Any]:
    pred = frag.get("prediction")
    return pred if isinstance(pred, dict) else {}


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def _first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = _as_int(v)
        if out is not None:
            return out
    return None


def _as_bool(v: Any) -> Optional[bool]:
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        text = v.strip().lower()
        if text in ("true", "t", "yes", "y", "1"):
            return True
        if text in ("false", "f", "no", "n", "0"):
            return False
    return None


def _first_bool(*values: Any) -> Optional[bool]:
    for v in values:
        out = _as_bool(v)
        if out is not None:
            return out
    return None


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _first_path_int(*paths: Tuple[Dict[str, Any], Tuple[str, ...]]) -> Optional[int]:
    for obj, path in paths:
        out = _as_int(_nested_get(obj, *path))
        if out is not None:
            return out
    return None


def _first_path_float(*paths: Tuple[Dict[str, Any], Tuple[str, ...]]) -> Optional[float]:
    for obj, path in paths:
        out = _as_float(_nested_get(obj, *path))
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 2,
               signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _fmt_str(v: Optional[str], width: int = 0) -> str:
    if v is None:
        s = "---"
    else:
        s = v[:width] if width else v
    return f"{s:<{width}s}" if width else s


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _print_welford(name: str, w: Welford, unit: str = "cycles",
                   decimals: int = 3) -> None:
    if w.n == 0:
        print(f"  {name:<46s} no samples")
        return
    print(
        f"  {name:<46s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+12,.{decimals}f} {unit:<6s}  "
        f"sd={w.stddev:10,.{decimals}f}  "
        f"se={w.stderr:9,.{decimals}f}  "
        f"min={w.min_val:+12,.{decimals}f}  "
        f"max={w.max_val:+12,.{decimals}f}"
    )


def physical_pps_dwt_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "pps", "dwt_at_edge"),
        _nested_get(root, "fragment", "pps", "dwt_at_edge"),
        frag.get("pps_dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_normalized_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_raw_at_edge"),
        frag.get("pps_diag_pps_edge_dwt_isr_entry_raw"),
        frag.get("pps_edge_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
    )


def vclock_preferred_dwt_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Tuple[Optional[int], str]:
    lattice_dwt = _first_int(
        _nested_get(frag, "dwt", "at_pps_vclock"),
        _nested_get(root, "fragment", "dwt", "at_pps_vclock"),
        frag.get("dwt_at_pps_vclock"),
        root.get("dwt_at_pps_vclock"),
    )
    if lattice_dwt is not None:
        return lattice_dwt, "lattice"

    phase_estimate_dwt = _first_int(
        frag.get("pps_vclock_phase_estimated_dwt_at_edge"),
        root.get("pps_vclock_phase_estimated_dwt_at_edge"),
    )
    if phase_estimate_dwt is not None:
        return phase_estimate_dwt, "phase-old"

    forensic_dwt = _first_int(
        _nested_get(frag, "vclock", "forensics", "last_event_dwt"),
        _nested_get(root, "vclock", "alpha_event", "last_event_dwt"),
        _nested_get(root, "vclock", "forensics", "last_event_dwt"),
    )
    if forensic_dwt is not None:
        return forensic_dwt, "forensics"

    return None, "---"


def pps_vclock_phase_cycles_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "pps", "vclock_phase_cycles"),
        _nested_get(root, "fragment", "pps", "vclock_phase_cycles"),
        frag.get("pps_vclock_phase_cycles"),
        root.get("pps_vclock_phase_cycles"),
    )


def lane_prediction(frag: Dict[str, Any],
                    pred: Dict[str, Any],
                    lane: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    """Return actual cycles, static prediction cycles, and residual cycles.

    TIMEBASE_FRAGMENT_V2 stores the four-rail prediction surface as
    prediction.<lane>.{actual_cycles,prediction_cycles,residual_cycles}.
    Older rows used flat prediction keys, so keep those as fallbacks.
    """

    lane_pred = pred.get(lane) if isinstance(pred.get(lane), dict) else {}

    actual = _first_int(
        lane_pred.get("actual_cycles"),
        frag.get(f"{lane}_actual_cycles"),
        pred.get(f"{lane}_actual_cycles"),
    )
    static = _first_int(
        lane_pred.get("prediction_cycles"),
        lane_pred.get("static_prediction_cycles"),
        frag.get(f"{lane}_prediction_cycles"),
        pred.get(f"{lane}_prediction_cycles"),
        pred.get(f"{lane}_static_prediction_cycles"),
    )
    residual = _first_int(
        lane_pred.get("residual_cycles"),
        lane_pred.get("static_residual_cycles"),
        frag.get(f"{lane}_residual_cycles"),
        pred.get(f"{lane}_residual_cycles"),
        pred.get(f"{lane}_static_residual_cycles"),
    )

    if residual is None and actual is not None and static is not None:
        residual = actual - static

    return actual, static, residual


def ocxo_forensic_last_event_dwt_from_schema(root: Dict[str, Any],
                                             frag: Dict[str, Any],
                                             lane: str) -> Optional[int]:
    flat_keys = (
        f"{lane}_alpha_event_last_event_dwt",
        f"{lane}_forensics_last_event_dwt",
        f"{lane}_last_event_dwt",
        f"{lane}_event_dwt",
        f"{lane}_dwt_at_edge",
    )

    flat = _first_int(*(frag.get(k) for k in flat_keys),
                      *(root.get(k) for k in flat_keys))
    if flat is not None:
        return flat

    return _first_path_int(
        (frag, (lane, "forensics", "last_event_dwt")),
        (root, ("fragment", lane, "forensics", "last_event_dwt")),
        (frag, (lane, "alpha_event", "last_event_dwt")),
        (root, (lane, "alpha_event", "last_event_dwt")),
        (root, (lane, "forensics", "last_event_dwt")),
        (frag, ("clock_forensics", lane, "alpha_event", "last_event_dwt")),
        (root, ("clock_forensics", lane, "alpha_event", "last_event_dwt")),
        (frag, ("forensics", lane, "alpha_event", "last_event_dwt")),
        (root, ("forensics", lane, "alpha_event", "last_event_dwt")),
        (frag, ("clock_forensics", lane, "last_event_dwt")),
        (root, ("clock_forensics", lane, "last_event_dwt")),
    )


def ocxo_forensic_counter32_delta_from_schema(root: Dict[str, Any],
                                              frag: Dict[str, Any],
                                              lane: str) -> Optional[int]:
    flat_keys = (
        f"{lane}_forensics_counter32_delta_since_previous_event",
        f"{lane}_counter32_delta_since_previous_event",
    )
    flat = _first_int(*(frag.get(k) for k in flat_keys),
                      *(root.get(k) for k in flat_keys))
    if flat is not None:
        return flat

    return _first_path_int(
        (frag, (lane, "forensics", "counter32_delta_since_previous_event")),
        (root, ("fragment", lane, "forensics", "counter32_delta_since_previous_event")),
        (root, (lane, "alpha_event", "counter32_delta_since_previous_event")),
        (root, (lane, "forensics", "counter32_delta_since_previous_event")),
        (frag, ("clock_forensics", lane, "alpha_event",
                "counter32_delta_since_previous_event")),
        (root, ("clock_forensics", lane, "alpha_event",
                "counter32_delta_since_previous_event")),
        (frag, ("forensics", lane, "counter32_delta_since_previous_event")),
        (root, ("forensics", lane, "counter32_delta_since_previous_event")),
    )



def ocxo_service_corrected_dwt_from_schema(root: Dict[str, Any],
                                           frag: Dict[str, Any],
                                           lane: str) -> Optional[int]:
    prefix = f"{lane}_forensics_"
    value = _first_int(
        _nested_get(frag, lane, "service", "corrected_dwt_at_event"),
        _nested_get(root, "fragment", lane, "service", "corrected_dwt_at_event"),
        frag.get(prefix + "service_corrected_dwt_at_event"),
        root.get(prefix + "service_corrected_dwt_at_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event",
                    "ocxo_service", "corrected_dwt_at_event"),
        _nested_get(root, "clock_forensics", lane, "alpha_event",
                    "ocxo_service", "corrected_dwt_at_event"),
        _nested_get(frag, "clock_forensics", lane, "ocxo_service",
                    "corrected_dwt_at_event"),
        _nested_get(root, "clock_forensics", lane, "ocxo_service",
                    "corrected_dwt_at_event"),
    )
    # In the diagnostic-only regression build this field may be intentionally
    # zeroed. Treat zero as unavailable so corrected-residual emulation does
    # not manufacture one-DWT-wrap-sized artifacts.
    return value if value not in (None, 0) else None


def corrected_residual(current_corrected_actual: Optional[int],
                       previous_corrected_actual: Optional[int]) -> Optional[int]:
    if current_corrected_actual is None or previous_corrected_actual is None:
        return None
    return current_corrected_actual - previous_corrected_actual


def regression_diag_from_schema(root: Dict[str, Any],
                                frag: Dict[str, Any],
                                lane: str) -> Dict[str, Any]:
    """Return diagnostic-only linear-regression fields for one lane.

    Preferred source is TIMEBASE_FRAGMENT_V2's nested
    <lane>.regression object. Flat <lane>_forensics_regression_* keys and
    focused CLOCKS REPORT_FORENSICS* payloads remain fallbacks.
    """

    prefix = f"{lane}_forensics_regression_"

    def flat_int(suffix: str) -> Optional[int]:
        return _first_int(frag.get(prefix + suffix), root.get(prefix + suffix))

    def flat_bool(suffix: str) -> Optional[bool]:
        return _first_bool(frag.get(prefix + suffix), root.get(prefix + suffix))

    def nested_value(suffix: str) -> Any:
        paths = (
            (frag, (lane, "regression", suffix)),
            (root, ("fragment", lane, "regression", suffix)),
            (root, (lane, "regression", suffix)),
            (root, (lane, "alpha_event", "regression", suffix)),
            (frag, ("clock_forensics", lane, "alpha_event", "regression", suffix)),
            (root, ("clock_forensics", lane, "alpha_event", "regression", suffix)),
            (frag, ("forensics", lane, "alpha_event", "regression", suffix)),
            (root, ("forensics", lane, "alpha_event", "regression", suffix)),
            (frag, (lane, "alpha_event", "regression", suffix)),
        )
        for obj, path in paths:
            value = _nested_get(obj, *path)
            if value is not None:
                return value
        return None

    def value_int(suffix: str) -> Optional[int]:
        return _first_int(flat_int(suffix), _as_int(nested_value(suffix)))

    inferred = value_int("inferred_dwt_at_event")
    observed = value_int("observed_dwt_at_event")
    valid = _first_bool(flat_bool("valid"), nested_value("valid"))

    return {
        "valid": bool(valid) if valid is not None else (inferred is not None and inferred != 0),
        "sample_count": value_int("sample_count"),
        "sequence": value_int("sequence"),
        "target_counter32": value_int("target_counter32_at_event"),
        "target_hardware16": value_int("target_hardware16_at_event"),
        "observed_hardware16": value_int("observed_hardware16_at_event"),
        "observed_dwt": observed,
        "inferred_dwt": inferred,
        "inferred_minus_observed": value_int("inferred_minus_observed_cycles"),
        "slope_q16": value_int("slope_q16_cycles_per_sample"),
        "slope_delta_q16": value_int("slope_delta_q16_cycles_per_sample"),
        "fit_error_mean_q16": value_int("fit_error_mean_q16_cycles"),
        "fit_error_stddev_q16": value_int("fit_error_stddev_q16_cycles"),
        "fit_error_min": value_int("fit_error_min_cycles"),
        "fit_error_max": value_int("fit_error_max_cycles"),
        "fit_error_gt_plus4": value_int("fit_error_gt_plus4_count"),
        "fit_error_lt_minus4": value_int("fit_error_lt_minus4_count"),
        "fit_error_abs_gt4": value_int("fit_error_abs_gt4_count"),
    }


def regression_endpoint_dwt(reg: Dict[str, Any]) -> Optional[int]:
    dwt = reg.get("inferred_dwt")
    if reg.get("valid") and isinstance(dwt, int) and dwt != 0:
        return dwt
    return None


def q16_to_cycles(v: Optional[int]) -> Optional[float]:
    return None if v is None else float(v) / 65536.0


def regression_residual(current_regression_actual: Optional[int],
                        previous_regression_actual: Optional[int]) -> Optional[int]:
    if current_regression_actual is None or previous_regression_actual is None:
        return None
    return current_regression_actual - previous_regression_actual


def vclock_counter32_at_event_from_schema(root: Dict[str, Any],
                                         frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "vclock", "forensics", "last_event_counter32"),
        _nested_get(root, "fragment", "vclock", "forensics", "last_event_counter32"),
        _nested_get(root, "vclock", "alpha_event", "last_event_counter32"),
        _nested_get(root, "vclock", "forensics", "last_event_counter32"),
        frag.get("vclock_forensics_last_event_counter32"),
        root.get("vclock_forensics_last_event_counter32"),
        _nested_get(frag, "vclock", "counter32_at_pps_vclock"),
        frag.get("counter32_at_pps_vclock"),
        root.get("counter32_at_pps_vclock"),
        _nested_get(frag, "clock_forensics", "vclock", "alpha_event",
                    "last_event_counter32"),
        _nested_get(root, "clock_forensics", "vclock", "alpha_event",
                    "last_event_counter32"),
    )


def ocxo_service_diag_from_schema(root: Dict[str, Any],
                                  frag: Dict[str, Any],
                                  lane: str) -> Dict[str, Optional[int]]:
    """Return compact OCXO compare-service diagnostics for a lane.

    TIMEBASE_FRAGMENT_V2 stores this as <lane>.service. Older flat and
    focused report schemas remain fallbacks.
    """

    prefix = f"{lane}_forensics_"
    out = {
        "service_class": _first_int(
            _nested_get(frag, lane, "service", "class"),
            _nested_get(root, "fragment", lane, "service", "class"),
            frag.get(prefix + "service_class"),
            root.get(prefix + "service_class"),
            _nested_get(frag, "clock_forensics", lane, "service_class"),
            _nested_get(root, "clock_forensics", lane, "service_class"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "class"),
        ),
        "service_offset": _first_int(
            _nested_get(frag, lane, "service", "offset_ticks"),
            _nested_get(root, "fragment", lane, "service", "offset_ticks"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "offset_signed_ticks"),
            frag.get(prefix + "service_offset_ticks"),
            root.get(prefix + "service_offset_ticks"),
            _nested_get(frag, "clock_forensics", lane, "service_offset_ticks"),
            _nested_get(root, "clock_forensics", lane, "service_offset_ticks"),
        ),
        "service_offset_abs": _first_int(
            _nested_get(frag, lane, "service", "offset_abs_ticks"),
            _nested_get(root, "fragment", lane, "service", "offset_abs_ticks"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "offset_abs_ticks"),
            frag.get(prefix + "service_offset_abs_ticks"),
            root.get(prefix + "service_offset_abs_ticks"),
        ),
        "late": _first_int(
            _nested_get(frag, lane, "service", "late_ticks"),
            _nested_get(frag, lane, "service", "interpreted_late_ticks"),
            _nested_get(root, "fragment", lane, "service", "late_ticks"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "interpreted_late_ticks"),
            frag.get(prefix + "interpreted_late_ticks"),
            root.get(prefix + "interpreted_late_ticks"),
        ),
        "early": _first_int(
            _nested_get(frag, lane, "service", "early_ticks"),
            _nested_get(root, "fragment", lane, "service", "early_ticks"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "early_ticks"),
            frag.get(prefix + "early_ticks"),
            root.get(prefix + "early_ticks"),
        ),
        "target_delta": _first_int(
            _nested_get(frag, lane, "service", "target_delta_mod65536_ticks"),
            _nested_get(root, "fragment", lane, "service", "target_delta_mod65536_ticks"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "target_delta_mod65536_ticks"),
            frag.get(prefix + "target_delta_mod65536_ticks"),
            root.get(prefix + "target_delta_mod65536_ticks"),
        ),
        "arm_remaining": _first_int(
            _nested_get(frag, lane, "service", "arm_remaining_ticks"),
            _nested_get(root, "fragment", lane, "service", "arm_remaining_ticks"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "arm_remaining_ticks"),
            frag.get(prefix + "arm_remaining_ticks"),
            root.get(prefix + "arm_remaining_ticks"),
        ),
        "arm_to_isr": _first_int(
            _nested_get(frag, lane, "service", "arm_to_isr_ticks"),
            _nested_get(root, "fragment", lane, "service", "arm_to_isr_ticks"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "arm_to_isr_ticks"),
            frag.get(prefix + "arm_to_isr_ticks"),
            root.get(prefix + "arm_to_isr_ticks"),
        ),
        "arm_to_isr_dwt": _first_int(
            _nested_get(frag, lane, "service", "arm_to_isr_dwt_cycles"),
            _nested_get(root, "fragment", lane, "service", "arm_to_isr_dwt_cycles"),
            _nested_get(root, lane, "alpha_event", "ocxo_service", "arm_to_isr_dwt_cycles"),
            frag.get(prefix + "arm_to_isr_dwt_cycles"),
            root.get(prefix + "arm_to_isr_dwt_cycles"),
        ),
    }

    return out


def service_class_short(service_class: Optional[int]) -> str:
    if service_class == 1:
        return "FIRQ"
    if service_class == 2:
        return "ERLY"
    if service_class == 3:
        return "OK"
    if service_class == 0:
        return "NONE"
    return "---"


def add_optional(w: Welford, v: Optional[int]) -> None:
    if v is not None:
        w.update(float(v))


def add_optional_float(w: Welford, v: Optional[float]) -> None:
    if v is not None:
        w.update(float(v))


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows):,} rows)")
    print()
    print("Residual-focused raw cycle audit")
    print("════════════════════════════════")
    print("  act/pred/res are firmware static prediction surfaces: residual = actual - prior actual.")
    print("  OCXO svc is signed compare-service offset in OCXO ticks: negative=early, positive=late.")
    print("  OCXO svcΔ is the row-to-row change in that signed service offset.")
    print("  v_cΔ is VCLOCK counter32 delta; 10,000,000 is normal, 20,000,000 means a skipped bookend.")
    print("  v_fΔ is VCLOCK forensic-DWT interval minus firmware v_act; nonzero means schema disagreement.")
    print("  Schema reader prefers TIMEBASE_FRAGMENT_V2 nested clock objects, with flat fallback.")
    print("  *_lr is the linear-regression-inferred interval; *_lrΔ = *_lr - *_act.")
    print("  o*_c_res emulates the older service-corrected OCXO DWT endpoint experiment.")
    print("  OCXO cΔ should be flat 10,000,000; deviations indicate target-identity/counter-ladder trouble.")
    print()

    header = (
        f"{'pps':>6s}  "
        f"{'p_act':>13s} {'p_pred':>13s} {'p_res':>8s}  "
        f"{'v_act':>13s} {'v_lr':>13s} {'v_lrΔ':>8s} {'v_pred':>13s} {'v_res':>8s} {'v_lr_res':>9s} "
        f"{'phase':>5s} {'v_cΔ':>11s} {'v_fΔ':>8s}  "
        f"{'o1_act':>13s} {'o1_lr':>13s} {'o1_lrΔ':>8s} {'o1_pred':>13s} {'o1_res':>8s} {'o1_lr_res':>9s} {'o1_c_res':>9s} "
        f"{'o1_svc':>7s} {'o1_sΔ':>7s} {'o1_late':>7s} {'o1_early':>8s} {'o1_cls':>5s} {'o1_cΔ':>11s}  "
        f"{'o2_act':>13s} {'o2_lr':>13s} {'o2_lrΔ':>8s} {'o2_pred':>13s} {'o2_res':>8s} {'o2_lr_res':>9s} {'o2_c_res':>9s} "
        f"{'o2_svc':>7s} {'o2_sΔ':>7s} {'o2_late':>7s} {'o2_early':>8s} {'o2_cls':>5s} {'o2_cΔ':>11s}  "
        f"{'dwt_ppb':>10s}"
    )
    print(header)
    print(
        f"{'─'*6}  "
        f"{'─'*13} {'─'*13} {'─'*8}  "
        f"{'─'*13} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*9} "
        f"{'─'*5} {'─'*11} {'─'*8}  "
        f"{'─'*13} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*9} {'─'*9} "
        f"{'─'*7} {'─'*7} {'─'*7} {'─'*8} {'─'*5} {'─'*11}  "
        f"{'─'*13} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*9} {'─'*9} "
        f"{'─'*7} {'─'*7} {'─'*7} {'─'*8} {'─'*5} {'─'*11}  "
        f"{'─'*10}"
    )

    shown = 0
    gaps = 0

    prev_pps_count: Optional[int] = None
    prev_physical_pps_dwt: Optional[int] = None
    prev_vclock_dwt: Optional[int] = None
    prev_vclock_forensic_dwt: Optional[int] = None
    prev_vclock_regression_dwt: Optional[int] = None
    prev_vclock_regression_actual: Optional[int] = None
    prev_vclock_counter32: Optional[int] = None
    prev_ocxo1_forensic_dwt: Optional[int] = None
    prev_ocxo2_forensic_dwt: Optional[int] = None
    prev_ocxo1_regression_dwt: Optional[int] = None
    prev_ocxo2_regression_dwt: Optional[int] = None
    prev_ocxo1_regression_actual: Optional[int] = None
    prev_ocxo2_regression_actual: Optional[int] = None
    prev_ocxo1_corrected_dwt: Optional[int] = None
    prev_ocxo2_corrected_dwt: Optional[int] = None
    prev_o1_corrected_actual: Optional[int] = None
    prev_o2_corrected_actual: Optional[int] = None
    prev_o1_service: Optional[int] = None
    prev_o2_service: Optional[int] = None

    stats: Dict[str, Welford] = {
        "pps_actual": Welford(),
        "pps_residual": Welford(),
        "phase": Welford(),
        "vclock_actual": Welford(),
        "vclock_residual": Welford(),
        "vclock_regression_actual": Welford(),
        "vclock_regression_residual": Welford(),
        "vclock_regression_minus_traditional": Welford(),
        "vclock_regression_inferred_minus_observed": Welford(),
        "vclock_regression_fit_stddev": Welford(),
        "vclock_regression_sample_count": Welford(),
        "vclock_forensic_delta": Welford(),
        "vclock_counter32_delta": Welford(),
        "ocxo1_actual": Welford(),
        "ocxo1_residual": Welford(),
        "ocxo1_regression_actual": Welford(),
        "ocxo1_regression_residual": Welford(),
        "ocxo1_regression_minus_traditional": Welford(),
        "ocxo1_regression_inferred_minus_observed": Welford(),
        "ocxo1_regression_fit_stddev": Welford(),
        "ocxo1_regression_sample_count": Welford(),
        "ocxo1_forensic_delta": Welford(),
        "ocxo1_corrected_actual": Welford(),
        "ocxo1_corrected_residual": Welford(),
        "ocxo1_residual_collapse": Welford(),
        "ocxo1_counter32_delta": Welford(),
        "ocxo1_service_offset": Welford(),
        "ocxo1_service_delta": Welford(),
        "ocxo1_late": Welford(),
        "ocxo1_early": Welford(),
        "ocxo1_arm_error": Welford(),
        "ocxo2_actual": Welford(),
        "ocxo2_residual": Welford(),
        "ocxo2_regression_actual": Welford(),
        "ocxo2_regression_residual": Welford(),
        "ocxo2_regression_minus_traditional": Welford(),
        "ocxo2_regression_inferred_minus_observed": Welford(),
        "ocxo2_regression_fit_stddev": Welford(),
        "ocxo2_regression_sample_count": Welford(),
        "ocxo2_forensic_delta": Welford(),
        "ocxo2_corrected_actual": Welford(),
        "ocxo2_corrected_residual": Welford(),
        "ocxo2_residual_collapse": Welford(),
        "ocxo2_counter32_delta": Welford(),
        "ocxo2_service_offset": Welford(),
        "ocxo2_service_delta": Welford(),
        "ocxo2_late": Welford(),
        "ocxo2_early": Welford(),
        "ocxo2_arm_error": Welford(),
    }

    coverage = {
        "rows": 0,
        "vclock_forensic_delta": 0,
        "vclock_regression_actual": 0,
        "vclock_regression_residual": 0,
        "vclock_counter32_delta": 0,
        "ocxo1_service": 0,
        "ocxo2_service": 0,
        "ocxo1_counter32_delta": 0,
        "ocxo2_counter32_delta": 0,
        "ocxo1_forensic_delta": 0,
        "ocxo2_forensic_delta": 0,
        "ocxo1_regression_actual": 0,
        "ocxo2_regression_actual": 0,
        "ocxo1_regression_residual": 0,
        "ocxo2_regression_residual": 0,
        "ocxo1_corrected_residual": 0,
        "ocxo2_corrected_residual": 0,
    }

    service_class_counts = {
        "ocxo1": {},
        "ocxo2": {},
    }

    excursions: List[Dict[str, Any]] = []
    vclock_excursions: List[Dict[str, Any]] = []

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        pred = _prediction(frag)
        pps_count = _first_int(
            root.get("pps_count"),
            frag.get("pps_count"),
            _nested_get(frag, "pps", "count"),
            frag.get("teensy_pps_vclock_count"),
            frag.get("teensy_pps_count"),
        )
        if pps_count is None:
            continue

        if prev_pps_count is not None and pps_count != prev_pps_count + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps_count:,} → {pps_count:,} ---")
            prev_physical_pps_dwt = None
            prev_vclock_dwt = None
            prev_vclock_forensic_dwt = None
            prev_vclock_regression_dwt = None
            prev_vclock_regression_actual = None
            prev_vclock_counter32 = None
            prev_ocxo1_forensic_dwt = None
            prev_ocxo2_forensic_dwt = None
            prev_ocxo1_regression_dwt = None
            prev_ocxo2_regression_dwt = None
            prev_ocxo1_regression_actual = None
            prev_ocxo2_regression_actual = None
            prev_ocxo1_corrected_dwt = None
            prev_ocxo2_corrected_dwt = None
            prev_o1_corrected_actual = None
            prev_o2_corrected_actual = None
            prev_o1_service = None
            prev_o2_service = None

        physical_pps_dwt = physical_pps_dwt_from_schema(root, frag)
        vclock_dwt, _vclock_source = vclock_preferred_dwt_from_schema(root, frag)

        pps_phase = pps_vclock_phase_cycles_from_schema(root, frag)
        if pps_phase is None and physical_pps_dwt is not None and vclock_dwt is not None:
            pps_phase = abs(_signed_delta_u32(vclock_dwt, physical_pps_dwt))

        pps_actual, pps_pred, pps_res = lane_prediction(frag, pred, "pps")
        v_actual, v_pred, v_res = lane_prediction(frag, pred, "vclock")
        o1_actual, o1_pred, o1_res = lane_prediction(frag, pred, "ocxo1")
        o2_actual, o2_pred, o2_res = lane_prediction(frag, pred, "ocxo2")

        # Fallbacks for older rows that lack the compact prediction surface.
        if pps_actual is None and physical_pps_dwt is not None and prev_physical_pps_dwt is not None:
            pps_actual = _delta_u32(physical_pps_dwt, prev_physical_pps_dwt)
        if v_actual is None and vclock_dwt is not None and prev_vclock_dwt is not None:
            v_actual = _delta_u32(vclock_dwt, prev_vclock_dwt)

        vclock_forensic_dwt = ocxo_forensic_last_event_dwt_from_schema(root, frag, "vclock")
        vclock_event_counter32 = vclock_counter32_at_event_from_schema(root, frag)

        v_forensic_actual: Optional[int] = None
        if vclock_forensic_dwt is not None and prev_vclock_forensic_dwt is not None:
            v_forensic_actual = _delta_u32(vclock_forensic_dwt, prev_vclock_forensic_dwt)
        v_forensic_delta = (
            v_forensic_actual - v_actual
            if v_forensic_actual is not None and v_actual is not None
            else None
        )

        v_cdelta = ocxo_forensic_counter32_delta_from_schema(root, frag, "vclock")
        if (v_cdelta is None and
            vclock_event_counter32 is not None and
            prev_vclock_counter32 is not None):
            v_cdelta = _delta_u32(vclock_event_counter32, prev_vclock_counter32)

        v_reg = regression_diag_from_schema(root, frag, "vclock")
        v_reg_dwt = regression_endpoint_dwt(v_reg)
        v_reg_actual: Optional[int] = None
        if v_reg_dwt is not None and prev_vclock_regression_dwt is not None:
            v_reg_actual = _delta_u32(v_reg_dwt, prev_vclock_regression_dwt)
        v_reg_res = regression_residual(v_reg_actual, prev_vclock_regression_actual)
        v_reg_delta = (
            v_reg_actual - v_actual
            if v_reg_actual is not None and v_actual is not None
            else None
        )

        o1_forensic_dwt = ocxo_forensic_last_event_dwt_from_schema(root, frag, "ocxo1")
        o2_forensic_dwt = ocxo_forensic_last_event_dwt_from_schema(root, frag, "ocxo2")
        o1_corrected_dwt = ocxo_service_corrected_dwt_from_schema(root, frag, "ocxo1")
        o2_corrected_dwt = ocxo_service_corrected_dwt_from_schema(root, frag, "ocxo2")

        o1_corrected_actual: Optional[int] = None
        if o1_corrected_dwt is not None and prev_ocxo1_corrected_dwt is not None:
            o1_corrected_actual = _delta_u32(o1_corrected_dwt, prev_ocxo1_corrected_dwt)
        o1_corrected_res = corrected_residual(o1_corrected_actual, prev_o1_corrected_actual)
        o1_residual_collapse = (
            abs(o1_res) - abs(o1_corrected_res)
            if o1_res is not None and o1_corrected_res is not None
            else None
        )

        o2_corrected_actual: Optional[int] = None
        if o2_corrected_dwt is not None and prev_ocxo2_corrected_dwt is not None:
            o2_corrected_actual = _delta_u32(o2_corrected_dwt, prev_ocxo2_corrected_dwt)
        o2_corrected_res = corrected_residual(o2_corrected_actual, prev_o2_corrected_actual)
        o2_residual_collapse = (
            abs(o2_res) - abs(o2_corrected_res)
            if o2_res is not None and o2_corrected_res is not None
            else None
        )

        o1_forensic_actual: Optional[int] = None
        if o1_forensic_dwt is not None and prev_ocxo1_forensic_dwt is not None:
            o1_forensic_actual = _delta_u32(o1_forensic_dwt, prev_ocxo1_forensic_dwt)
        o1_forensic_delta = (
            o1_forensic_actual - o1_actual
            if o1_forensic_actual is not None and o1_actual is not None
            else None
        )

        o2_forensic_actual: Optional[int] = None
        if o2_forensic_dwt is not None and prev_ocxo2_forensic_dwt is not None:
            o2_forensic_actual = _delta_u32(o2_forensic_dwt, prev_ocxo2_forensic_dwt)
        o2_forensic_delta = (
            o2_forensic_actual - o2_actual
            if o2_forensic_actual is not None and o2_actual is not None
            else None
        )

        o1_reg = regression_diag_from_schema(root, frag, "ocxo1")
        o2_reg = regression_diag_from_schema(root, frag, "ocxo2")
        o1_reg_dwt = regression_endpoint_dwt(o1_reg)
        o2_reg_dwt = regression_endpoint_dwt(o2_reg)

        o1_reg_actual: Optional[int] = None
        if o1_reg_dwt is not None and prev_ocxo1_regression_dwt is not None:
            o1_reg_actual = _delta_u32(o1_reg_dwt, prev_ocxo1_regression_dwt)
        o1_reg_res = regression_residual(o1_reg_actual, prev_ocxo1_regression_actual)
        o1_reg_delta = (
            o1_reg_actual - o1_actual
            if o1_reg_actual is not None and o1_actual is not None
            else None
        )

        o2_reg_actual: Optional[int] = None
        if o2_reg_dwt is not None and prev_ocxo2_regression_dwt is not None:
            o2_reg_actual = _delta_u32(o2_reg_dwt, prev_ocxo2_regression_dwt)
        o2_reg_res = regression_residual(o2_reg_actual, prev_ocxo2_regression_actual)
        o2_reg_delta = (
            o2_reg_actual - o2_actual
            if o2_reg_actual is not None and o2_actual is not None
            else None
        )

        o1_cdelta = ocxo_forensic_counter32_delta_from_schema(root, frag, "ocxo1")
        o2_cdelta = ocxo_forensic_counter32_delta_from_schema(root, frag, "ocxo2")

        o1_diag = ocxo_service_diag_from_schema(root, frag, "ocxo1")
        o2_diag = ocxo_service_diag_from_schema(root, frag, "ocxo2")

        o1_svc = o1_diag["service_offset"]
        o2_svc = o2_diag["service_offset"]

        o1_svc_delta = (
            o1_svc - prev_o1_service
            if o1_svc is not None and prev_o1_service is not None
            else None
        )
        o2_svc_delta = (
            o2_svc - prev_o2_service
            if o2_svc is not None and prev_o2_service is not None
            else None
        )

        o1_class = service_class_short(o1_diag["service_class"])
        o2_class = service_class_short(o2_diag["service_class"])

        for lane, cls in (("ocxo1", o1_class), ("ocxo2", o2_class)):
            service_class_counts[lane][cls] = service_class_counts[lane].get(cls, 0) + 1

        dwt_ppb = _as_float(_nested_get(frag, "stats", "dwt", "ppb"))
        if dwt_ppb is None:
            dwt_ppb = _as_float(_nested_get(root, "fragment", "stats", "dwt", "ppb"))
        if dwt_ppb is None:
            dwt_ppb = _as_float(frag.get("dwt_ppb"))
        if dwt_ppb is None:
            dwt_ppb = _as_float(root.get("dwt_ppb"))

        coverage["rows"] += 1
        if v_cdelta is not None:
            coverage["vclock_counter32_delta"] += 1
        if v_forensic_delta is not None:
            coverage["vclock_forensic_delta"] += 1
        if v_reg_actual is not None:
            coverage["vclock_regression_actual"] += 1
        if v_reg_res is not None:
            coverage["vclock_regression_residual"] += 1
        if o1_svc is not None:
            coverage["ocxo1_service"] += 1
        if o2_svc is not None:
            coverage["ocxo2_service"] += 1
        if o1_cdelta is not None:
            coverage["ocxo1_counter32_delta"] += 1
        if o2_cdelta is not None:
            coverage["ocxo2_counter32_delta"] += 1
        if o1_forensic_delta is not None:
            coverage["ocxo1_forensic_delta"] += 1
        if o2_forensic_delta is not None:
            coverage["ocxo2_forensic_delta"] += 1
        if o1_reg_actual is not None:
            coverage["ocxo1_regression_actual"] += 1
        if o2_reg_actual is not None:
            coverage["ocxo2_regression_actual"] += 1
        if o1_reg_res is not None:
            coverage["ocxo1_regression_residual"] += 1
        if o2_reg_res is not None:
            coverage["ocxo2_regression_residual"] += 1
        if o1_corrected_res is not None:
            coverage["ocxo1_corrected_residual"] += 1
        if o2_corrected_res is not None:
            coverage["ocxo2_corrected_residual"] += 1

        add_optional(stats["pps_actual"], pps_actual)
        add_optional(stats["pps_residual"], pps_res)
        add_optional(stats["phase"], pps_phase)
        add_optional(stats["vclock_actual"], v_actual)
        add_optional(stats["vclock_residual"], v_res)
        add_optional(stats["vclock_regression_actual"], v_reg_actual)
        add_optional(stats["vclock_regression_residual"], v_reg_res)
        add_optional(stats["vclock_regression_minus_traditional"], v_reg_delta)
        add_optional(stats["vclock_regression_inferred_minus_observed"],
                     v_reg["inferred_minus_observed"])
        add_optional_float(stats["vclock_regression_fit_stddev"],
                           q16_to_cycles(v_reg["fit_error_stddev_q16"]))
        add_optional(stats["vclock_regression_sample_count"], v_reg["sample_count"])
        add_optional(stats["vclock_forensic_delta"], v_forensic_delta)
        add_optional(stats["vclock_counter32_delta"], v_cdelta)
        add_optional(stats["ocxo1_actual"], o1_actual)
        add_optional(stats["ocxo1_residual"], o1_res)
        add_optional(stats["ocxo1_regression_actual"], o1_reg_actual)
        add_optional(stats["ocxo1_regression_residual"], o1_reg_res)
        add_optional(stats["ocxo1_regression_minus_traditional"], o1_reg_delta)
        add_optional(stats["ocxo1_regression_inferred_minus_observed"],
                     o1_reg["inferred_minus_observed"])
        add_optional_float(stats["ocxo1_regression_fit_stddev"],
                           q16_to_cycles(o1_reg["fit_error_stddev_q16"]))
        add_optional(stats["ocxo1_regression_sample_count"], o1_reg["sample_count"])
        add_optional(stats["ocxo1_forensic_delta"], o1_forensic_delta)
        add_optional(stats["ocxo1_corrected_actual"], o1_corrected_actual)
        add_optional(stats["ocxo1_corrected_residual"], o1_corrected_res)
        add_optional(stats["ocxo1_residual_collapse"], o1_residual_collapse)
        add_optional(stats["ocxo1_counter32_delta"], o1_cdelta)
        add_optional(stats["ocxo1_service_offset"], o1_svc)
        add_optional(stats["ocxo1_service_delta"], o1_svc_delta)
        add_optional(stats["ocxo1_late"], o1_diag["late"])
        add_optional(stats["ocxo1_early"], o1_diag["early"])
        if o1_diag["arm_to_isr"] is not None and o1_diag["arm_remaining"] is not None:
            add_optional(stats["ocxo1_arm_error"], o1_diag["arm_to_isr"] - o1_diag["arm_remaining"])
        add_optional(stats["ocxo2_actual"], o2_actual)
        add_optional(stats["ocxo2_residual"], o2_res)
        add_optional(stats["ocxo2_regression_actual"], o2_reg_actual)
        add_optional(stats["ocxo2_regression_residual"], o2_reg_res)
        add_optional(stats["ocxo2_regression_minus_traditional"], o2_reg_delta)
        add_optional(stats["ocxo2_regression_inferred_minus_observed"],
                     o2_reg["inferred_minus_observed"])
        add_optional_float(stats["ocxo2_regression_fit_stddev"],
                           q16_to_cycles(o2_reg["fit_error_stddev_q16"]))
        add_optional(stats["ocxo2_regression_sample_count"], o2_reg["sample_count"])
        add_optional(stats["ocxo2_forensic_delta"], o2_forensic_delta)
        add_optional(stats["ocxo2_corrected_actual"], o2_corrected_actual)
        add_optional(stats["ocxo2_corrected_residual"], o2_corrected_res)
        add_optional(stats["ocxo2_residual_collapse"], o2_residual_collapse)
        add_optional(stats["ocxo2_counter32_delta"], o2_cdelta)
        add_optional(stats["ocxo2_service_offset"], o2_svc)
        add_optional(stats["ocxo2_service_delta"], o2_svc_delta)
        add_optional(stats["ocxo2_late"], o2_diag["late"])
        add_optional(stats["ocxo2_early"], o2_diag["early"])
        if o2_diag["arm_to_isr"] is not None and o2_diag["arm_remaining"] is not None:
            add_optional(stats["ocxo2_arm_error"], o2_diag["arm_to_isr"] - o2_diag["arm_remaining"])

        if v_res is not None and abs(v_res) >= EXCURSION_THRESHOLD_CYCLES:
            vclock_excursions.append({
                "pps": pps_count,
                "v_res": v_res,
                "v_act": v_actual,
                "v_pred": v_pred,
                "v_reg_actual": v_reg_actual,
                "v_reg_delta": v_reg_delta,
                "v_reg_res": v_reg_res,
                "v_reg_io": v_reg["inferred_minus_observed"],
                "v_cdelta": v_cdelta,
                "v_forensic_delta": v_forensic_delta,
                "v_forensic_actual": v_forensic_actual,
                "phase": pps_phase,
                "p_res": pps_res,
            })

        if (o1_res is not None and abs(o1_res) >= EXCURSION_THRESHOLD_CYCLES) or (
            o2_res is not None and abs(o2_res) >= EXCURSION_THRESHOLD_CYCLES
        ):
            excursions.append({
                "pps": pps_count,
                "o1_res": o1_res,
                "o1_cres": o1_corrected_res,
                "o1_cact": o1_corrected_actual,
                "o1_collapse": o1_residual_collapse,
                "o1_act": o1_actual,
                "o1_reg_actual": o1_reg_actual,
                "o1_reg_delta": o1_reg_delta,
                "o1_reg_res": o1_reg_res,
                "o1_reg_io": o1_reg["inferred_minus_observed"],
                "o1_pred": o1_pred,
                "o1_svc": o1_svc,
                "o1_svc_delta": o1_svc_delta,
                "o1_late": o1_diag["late"],
                "o1_early": o1_diag["early"],
                "o1_arm": o1_diag["arm_remaining"],
                "o1_isr": o1_diag["arm_to_isr"],
                "o1_cls": o1_class,
                "o2_res": o2_res,
                "o2_cres": o2_corrected_res,
                "o2_cact": o2_corrected_actual,
                "o2_collapse": o2_residual_collapse,
                "o2_act": o2_actual,
                "o2_reg_actual": o2_reg_actual,
                "o2_reg_delta": o2_reg_delta,
                "o2_reg_res": o2_reg_res,
                "o2_reg_io": o2_reg["inferred_minus_observed"],
                "o2_pred": o2_pred,
                "o2_svc": o2_svc,
                "o2_svc_delta": o2_svc_delta,
                "o2_late": o2_diag["late"],
                "o2_early": o2_diag["early"],
                "o2_arm": o2_diag["arm_remaining"],
                "o2_isr": o2_diag["arm_to_isr"],
                "o2_cls": o2_class,
                "v_res": v_res,
                "p_res": pps_res,
            })

        print(
            f"{pps_count:>6d}  "
            f"{_fmt_int(pps_actual, 13)} {_fmt_int(pps_pred, 13)} {_fmt_int(pps_res, 8, signed=True)}  "
            f"{_fmt_int(v_actual, 13)} {_fmt_int(v_reg_actual, 13)} {_fmt_int(v_reg_delta, 8, signed=True)} "
            f"{_fmt_int(v_pred, 13)} {_fmt_int(v_res, 8, signed=True)} {_fmt_int(v_reg_res, 9, signed=True)} "
            f"{_fmt_int(pps_phase, 5)} {_fmt_int(v_cdelta, 11)} {_fmt_int(v_forensic_delta, 8, signed=True)}  "
            f"{_fmt_int(o1_actual, 13)} {_fmt_int(o1_reg_actual, 13)} {_fmt_int(o1_reg_delta, 8, signed=True)} "
            f"{_fmt_int(o1_pred, 13)} {_fmt_int(o1_res, 8, signed=True)} {_fmt_int(o1_reg_res, 9, signed=True)} {_fmt_int(o1_corrected_res, 9, signed=True)} "
            f"{_fmt_int(o1_svc, 7, signed=True)} {_fmt_int(o1_svc_delta, 7, signed=True)} "
            f"{_fmt_int(o1_diag['late'], 7)} {_fmt_int(o1_diag['early'], 8)} {_fmt_str(o1_class, 5)} "
            f"{_fmt_int(o1_cdelta, 11)}  "
            f"{_fmt_int(o2_actual, 13)} {_fmt_int(o2_reg_actual, 13)} {_fmt_int(o2_reg_delta, 8, signed=True)} "
            f"{_fmt_int(o2_pred, 13)} {_fmt_int(o2_res, 8, signed=True)} {_fmt_int(o2_reg_res, 9, signed=True)} {_fmt_int(o2_corrected_res, 9, signed=True)} "
            f"{_fmt_int(o2_svc, 7, signed=True)} {_fmt_int(o2_svc_delta, 7, signed=True)} "
            f"{_fmt_int(o2_diag['late'], 7)} {_fmt_int(o2_diag['early'], 8)} {_fmt_str(o2_class, 5)} "
            f"{_fmt_int(o2_cdelta, 11)}  "
            f"{_fmt_float(dwt_ppb, 10, 3, signed=True)}"
        )

        prev_pps_count = pps_count
        if physical_pps_dwt is not None:
            prev_physical_pps_dwt = physical_pps_dwt
        if vclock_dwt is not None:
            prev_vclock_dwt = vclock_dwt
        if vclock_forensic_dwt is not None:
            prev_vclock_forensic_dwt = vclock_forensic_dwt
        if v_reg_dwt is not None:
            prev_vclock_regression_dwt = v_reg_dwt
        if v_reg_actual is not None:
            prev_vclock_regression_actual = v_reg_actual
        if vclock_event_counter32 is not None:
            prev_vclock_counter32 = vclock_event_counter32
        if o1_forensic_dwt is not None:
            prev_ocxo1_forensic_dwt = o1_forensic_dwt
        if o2_forensic_dwt is not None:
            prev_ocxo2_forensic_dwt = o2_forensic_dwt
        if o1_reg_dwt is not None:
            prev_ocxo1_regression_dwt = o1_reg_dwt
        if o2_reg_dwt is not None:
            prev_ocxo2_regression_dwt = o2_reg_dwt
        if o1_reg_actual is not None:
            prev_ocxo1_regression_actual = o1_reg_actual
        if o2_reg_actual is not None:
            prev_ocxo2_regression_actual = o2_reg_actual
        if o1_corrected_dwt is not None:
            prev_ocxo1_corrected_dwt = o1_corrected_dwt
        if o2_corrected_dwt is not None:
            prev_ocxo2_corrected_dwt = o2_corrected_dwt
        if o1_corrected_actual is not None:
            prev_o1_corrected_actual = o1_corrected_actual
        if o2_corrected_actual is not None:
            prev_o2_corrected_actual = o2_corrected_actual
        if o1_svc is not None:
            prev_o1_service = o1_svc
        if o2_svc is not None:
            prev_o2_service = o2_svc

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()

    print("Schema coverage")
    print("═══════════════")
    print(f"  rows                         = {coverage['rows']:,}")
    print(f"  VCLOCK forensic delta rows    = {coverage['vclock_forensic_delta']:,}")
    print(f"  VCLOCK regression actual rows = {coverage['vclock_regression_actual']:,}")
    print(f"  VCLOCK regression resid rows  = {coverage['vclock_regression_residual']:,}")
    print(f"  VCLOCK counter32 delta rows   = {coverage['vclock_counter32_delta']:,}")
    print(f"  OCXO1 service diag rows       = {coverage['ocxo1_service']:,}")
    print(f"  OCXO2 service diag rows       = {coverage['ocxo2_service']:,}")
    print(f"  OCXO1 forensic delta rows     = {coverage['ocxo1_forensic_delta']:,}")
    print(f"  OCXO2 forensic delta rows     = {coverage['ocxo2_forensic_delta']:,}")
    print(f"  OCXO1 regression actual rows  = {coverage['ocxo1_regression_actual']:,}")
    print(f"  OCXO2 regression actual rows  = {coverage['ocxo2_regression_actual']:,}")
    print(f"  OCXO1 regression resid rows   = {coverage['ocxo1_regression_residual']:,}")
    print(f"  OCXO2 regression resid rows   = {coverage['ocxo2_regression_residual']:,}")
    print(f"  OCXO1 corrected residual rows = {coverage['ocxo1_corrected_residual']:,}")
    print(f"  OCXO2 corrected residual rows = {coverage['ocxo2_corrected_residual']:,}")
    print(f"  OCXO1 counter32 delta rows    = {coverage['ocxo1_counter32_delta']:,}")
    print(f"  OCXO2 counter32 delta rows    = {coverage['ocxo2_counter32_delta']:,}")
    print(f"  OCXO1 service classes         = {service_class_counts['ocxo1']}")
    print(f"  OCXO2 service classes         = {service_class_counts['ocxo2']}")
    print()

    print("Summary")
    print("═══════")
    _print_welford("PPS actual cycles", stats["pps_actual"])
    _print_welford("PPS static residual", stats["pps_residual"])
    _print_welford("PPS→VCLOCK phase offset", stats["phase"])
    _print_welford("VCLOCK actual cycles", stats["vclock_actual"])
    _print_welford("VCLOCK static residual", stats["vclock_residual"])
    _print_welford("VCLOCK LR actual cycles", stats["vclock_regression_actual"])
    _print_welford("VCLOCK LR residual", stats["vclock_regression_residual"])
    _print_welford("VCLOCK LR interval minus traditional", stats["vclock_regression_minus_traditional"])
    _print_welford("VCLOCK LR inferred minus observed", stats["vclock_regression_inferred_minus_observed"])
    _print_welford("VCLOCK LR fit stddev", stats["vclock_regression_fit_stddev"])
    _print_welford("VCLOCK LR sample count", stats["vclock_regression_sample_count"], unit="samples")
    _print_welford("VCLOCK forensic delta", stats["vclock_forensic_delta"])
    _print_welford("VCLOCK counter32 delta", stats["vclock_counter32_delta"], unit="ticks")
    _print_welford("OCXO1 actual cycles", stats["ocxo1_actual"])
    _print_welford("OCXO1 static residual", stats["ocxo1_residual"])
    _print_welford("OCXO1 LR actual cycles", stats["ocxo1_regression_actual"])
    _print_welford("OCXO1 LR residual", stats["ocxo1_regression_residual"])
    _print_welford("OCXO1 LR interval minus traditional", stats["ocxo1_regression_minus_traditional"])
    _print_welford("OCXO1 LR inferred minus observed", stats["ocxo1_regression_inferred_minus_observed"])
    _print_welford("OCXO1 LR fit stddev", stats["ocxo1_regression_fit_stddev"])
    _print_welford("OCXO1 LR sample count", stats["ocxo1_regression_sample_count"], unit="samples")
    _print_welford("OCXO1 forensic delta", stats["ocxo1_forensic_delta"])
    _print_welford("OCXO1 corrected actual cycles", stats["ocxo1_corrected_actual"])
    _print_welford("OCXO1 corrected residual", stats["ocxo1_corrected_residual"])
    _print_welford("OCXO1 residual |collapse|", stats["ocxo1_residual_collapse"])
    _print_welford("OCXO1 counter32 delta", stats["ocxo1_counter32_delta"], unit="ticks")
    _print_welford("OCXO1 service offset", stats["ocxo1_service_offset"], unit="ticks")
    _print_welford("OCXO1 service offset delta", stats["ocxo1_service_delta"], unit="ticks")
    _print_welford("OCXO1 interpreted late", stats["ocxo1_late"], unit="ticks")
    _print_welford("OCXO1 interpreted early", stats["ocxo1_early"], unit="ticks")
    _print_welford("OCXO1 arm_to_isr - arm_remaining", stats["ocxo1_arm_error"], unit="ticks")
    _print_welford("OCXO2 actual cycles", stats["ocxo2_actual"])
    _print_welford("OCXO2 static residual", stats["ocxo2_residual"])
    _print_welford("OCXO2 LR actual cycles", stats["ocxo2_regression_actual"])
    _print_welford("OCXO2 LR residual", stats["ocxo2_regression_residual"])
    _print_welford("OCXO2 LR interval minus traditional", stats["ocxo2_regression_minus_traditional"])
    _print_welford("OCXO2 LR inferred minus observed", stats["ocxo2_regression_inferred_minus_observed"])
    _print_welford("OCXO2 LR fit stddev", stats["ocxo2_regression_fit_stddev"])
    _print_welford("OCXO2 LR sample count", stats["ocxo2_regression_sample_count"], unit="samples")
    _print_welford("OCXO2 forensic delta", stats["ocxo2_forensic_delta"])
    _print_welford("OCXO2 corrected actual cycles", stats["ocxo2_corrected_actual"])
    _print_welford("OCXO2 corrected residual", stats["ocxo2_corrected_residual"])
    _print_welford("OCXO2 residual |collapse|", stats["ocxo2_residual_collapse"])
    _print_welford("OCXO2 counter32 delta", stats["ocxo2_counter32_delta"], unit="ticks")
    _print_welford("OCXO2 service offset", stats["ocxo2_service_offset"], unit="ticks")
    _print_welford("OCXO2 service offset delta", stats["ocxo2_service_delta"], unit="ticks")
    _print_welford("OCXO2 interpreted late", stats["ocxo2_late"], unit="ticks")
    _print_welford("OCXO2 interpreted early", stats["ocxo2_early"], unit="ticks")
    _print_welford("OCXO2 arm_to_isr - arm_remaining", stats["ocxo2_arm_error"], unit="ticks")
    print()

    print(f"VCLOCK residual excursions (|residual| ≥ {EXCURSION_THRESHOLD_CYCLES} cycles)")
    print("════════════════════════════════════════════════════════════")
    if not vclock_excursions:
        print("  none")
    else:
        v_header = (
            f"{'pps':>6s} {'v_res':>8s} {'v_lr_res':>9s} {'v_act':>13s} {'v_lr':>13s} {'v_lrΔ':>8s} {'v_pred':>13s} "
            f"{'v_i-o':>8s} {'v_cΔ':>11s} {'v_fΔ':>8s} {'v_fact':>13s} {'phase':>5s} {'p_res':>8s}"
        )
        print(v_header)
        print(
            f"{'─'*6} {'─'*8} {'─'*9} {'─'*13} {'─'*13} {'─'*8} {'─'*13} "
            f"{'─'*8} {'─'*11} {'─'*8} {'─'*13} {'─'*5} {'─'*8}"
        )
        for ex in vclock_excursions[:60]:
            print(
                f"{ex['pps']:>6d} "
                f"{_fmt_int(ex['v_res'], 8, signed=True)} "
                f"{_fmt_int(ex['v_reg_res'], 9, signed=True)} "
                f"{_fmt_int(ex['v_act'], 13)} {_fmt_int(ex['v_reg_actual'], 13)} {_fmt_int(ex['v_reg_delta'], 8, signed=True)} {_fmt_int(ex['v_pred'], 13)} "
                f"{_fmt_int(ex['v_reg_io'], 8, signed=True)} "
                f"{_fmt_int(ex['v_cdelta'], 11)} "
                f"{_fmt_int(ex['v_forensic_delta'], 8, signed=True)} "
                f"{_fmt_int(ex['v_forensic_actual'], 13)} "
                f"{_fmt_int(ex['phase'], 5)} "
                f"{_fmt_int(ex['p_res'], 8, signed=True)}"
            )
        if len(vclock_excursions) > 60:
            print(f"  ... {len(vclock_excursions) - 60:,} more VCLOCK excursion rows omitted")
    print()

    print(f"OCXO residual excursions (|residual| ≥ {EXCURSION_THRESHOLD_CYCLES} cycles)")
    print("════════════════════════════════════════════════════════════")
    if not excursions:
        print("  none")
    else:
        ex_header = (
            f"{'pps':>6s} {'lane':>5s} {'res':>8s} {'lr_res':>8s} {'lrΔ':>8s} {'c_res':>8s} {'collapse':>8s} "
            f"{'act':>13s} {'lr_act':>13s} {'c_act':>13s} {'pred':>13s} "
            f"{'svc':>7s} {'svcΔ':>7s} {'late':>7s} {'early':>7s} "
            f"{'arm':>7s} {'isr':>7s} {'cls':>5s} {'p_res':>8s} {'v_res':>8s}"
        )
        print(ex_header)
        print(
            f"{'─'*6} {'─'*5} {'─'*8} {'─'*8} {'─'*8} {'─'*8} {'─'*8} "
            f"{'─'*13} {'─'*13} {'─'*13} {'─'*13} "
            f"{'─'*7} {'─'*7} {'─'*7} {'─'*7} "
            f"{'─'*7} {'─'*7} {'─'*5} {'─'*8} {'─'*8}"
        )
        for ex in excursions[:60]:
            for lane in ("o1", "o2"):
                res = ex[f"{lane}_res"]
                if res is None or abs(res) < EXCURSION_THRESHOLD_CYCLES:
                    continue
                print(
                    f"{ex['pps']:>6d} {lane:>5s} "
                    f"{_fmt_int(res, 8, signed=True)} "
                    f"{_fmt_int(ex[f'{lane}_reg_res'], 8, signed=True)} "
                    f"{_fmt_int(ex[f'{lane}_reg_delta'], 8, signed=True)} "
                    f"{_fmt_int(ex[f'{lane}_cres'], 8, signed=True)} "
                    f"{_fmt_int(ex[f'{lane}_collapse'], 8, signed=True)} "
                    f"{_fmt_int(ex[f'{lane}_act'], 13)} {_fmt_int(ex[f'{lane}_reg_actual'], 13)} {_fmt_int(ex[f'{lane}_cact'], 13)} {_fmt_int(ex[f'{lane}_pred'], 13)} "
                    f"{_fmt_int(ex[f'{lane}_svc'], 7, signed=True)} "
                    f"{_fmt_int(ex[f'{lane}_svc_delta'], 7, signed=True)} "
                    f"{_fmt_int(ex[f'{lane}_late'], 7)} "
                    f"{_fmt_int(ex[f'{lane}_early'], 7)} "
                    f"{_fmt_int(ex[f'{lane}_arm'], 7)} "
                    f"{_fmt_int(ex[f'{lane}_isr'], 7)} "
                    f"{_fmt_str(ex[f'{lane}_cls'], 5)} "
                    f"{_fmt_int(ex['p_res'], 8, signed=True)} "
                    f"{_fmt_int(ex['v_res'], 8, signed=True)}"
                )
        if len(excursions) > 60:
            print(f"  ... {len(excursions) - 60:,} more excursion rows omitted")

    print()
    print("Notes")
    print("═════")
    print("  • v_cΔ is VCLOCK counter32 delta.  Normal is 10,000,000; 20,000,000 means")
    print("    the consumed VCLOCK event stream skipped one public bookend.")
    print("  • v_fΔ is VCLOCK forensic-DWT interval minus v_act.  Nonzero means the")
    print("    Alpha-forensics DWT surface disagrees with the flat prediction surface.")
    print("  • *_lr is computed from *_forensics_regression_inferred_dwt_at_event.")
    print("    It is diagnostic-only here and does not imply Alpha used that endpoint.")
    print("  • *_lrΔ = regression interval minus traditional interval.")
    print("  • *_lr_res = regression interval minus previous regression interval.")
    print("  • o*_c_res is computed from ocxo*_forensics_service_corrected_dwt_at_event.")
    print("    It emulates the older scalar service-corrected DWT endpoint experiment.")
    print("  • collapse = abs(raw residual) - abs(corrected residual); positive is good.")
    print("  • svc is signed OCXO compare-service offset in 10 MHz ticks.")
    print("    Positive values mean ISR service was after target; negative values mean early/pre-target.")
    print("  • svcΔ is row-to-row change in svc.  Residual excursions often track changes in service timing.")
    print("  • cls: OK=ON_OR_AFTER_TARGET, ERLY=EARLY_PRETARGET, FIRQ=FALSE_IRQ.")
    print("  • cΔ should be exactly 10,000,000; this proves the OCXO synthetic target ladder is intact.")
    print("  • forensic delta is summarized but not printed in the main table; nonzero means Alpha DWT")
    print("    reconstruction disagrees with the firmware prediction actual.")
    print()


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit]")
        raise SystemExit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
