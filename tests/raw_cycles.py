"""
ZPNet Raw Cycles — raw / FloorLine / published cycle audit.

Reads TIMEBASE rows for a campaign and prints a compact one-second DWT-cycle
audit.  PPS is always shown as the base rail.  By default the report shows
VCLOCK, OCXO1, and OCXO2.  A clock filter narrows the view to PPS + one lane.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> --clock OCXO2 [limit]
    python raw_cycles.py <campaign_name> --limit 300 --clock VCLOCK

Clock filter:
    VCLOCK, OCXO1, OCXO2

Column doctrine:
    p_act     physical PPS actual interval, in DWT cycles.
    p_Δ       physical PPS first-difference residual.

    *_raw     raw/evidence interval from dwt_interval_gate.observed_cycles
              when present, otherwise from original/observed DWT endpoints.
    *_rawΔ    raw interval first-difference residual:
              raw_interval[n] - raw_interval[n-1].
    *_fl      FloorLine inferred interval from consecutive FloorLine DWT
              endpoints when available.  The fitted/slope interval remains
              a fallback/audit surface only.
    *_flΔ     FloorLine endpoint interval first-difference residual:
              fl_interval[n] - fl_interval[n-1].
    *_pub     the subscriber-facing published interval.
    *_pubΔ    published interval first-difference residual:
              pub_interval[n] - pub_interval[n-1].
    *_cΔ      counter32 delta since the previous subscriber event.

Core rail columns are always shown for each selected lane so the report shape
stays stable across current FloorLine firmware builds.  Truly auxiliary columns
(such as counter32 lineage) are still omitted when absent.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


CLOCKS = ("VCLOCK", "OCXO1", "OCXO2")
LANE_KEYS = {"VCLOCK": "vclock", "OCXO1": "ocxo1", "OCXO2": "ocxo2"}
PREFIXES = {"VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}
LANE_MICRO_PREFIXES = {"vclock": "v", "ocxo1": "o1", "ocxo2": "o2"}


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


def _forensics_root(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    f = root.get("forensics")
    return f if isinstance(f, dict) else {}


def _prediction(frag: Dict[str, Any]) -> Dict[str, Any]:
    pred = frag.get("prediction")
    return pred if isinstance(pred, dict) else {}


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


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


def _as_bool(v: Any) -> Optional[bool]:
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        t = v.strip().lower()
        if t in ("true", "t", "yes", "y", "1"):
            return True
        if t in ("false", "f", "no", "n", "0"):
            return False
    return None


def _first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = _as_int(v)
        if out is not None:
            return out
    return None


def _first_float(*values: Any) -> Optional[float]:
    for v in values:
        out = _as_float(v)
        if out is not None:
            return out
    return None


def _first_bool(*values: Any) -> Optional[bool]:
    for v in values:
        out = _as_bool(v)
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
    s = "---" if v is None else str(v)
    if width and len(s) > width:
        s = s[:width]
    return f"{s:>{width}s}" if width else s


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _interval_delta(current: Optional[int], previous: Optional[int]) -> Optional[int]:
    if current is None or previous is None:
        return None
    return current - previous


def _interval_from_endpoints(now: Optional[int],
                             prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    return _delta_u32(now, prev)


def _print_welford(name: str, w: Welford, unit: str = "cycles",
                   decimals: int = 3) -> None:
    if w.n == 0:
        print(f"  {name:<42s} no samples")
        return
    print(
        f"  {name:<42s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+12,.{decimals}f} {unit:<6s}  "
        f"sd={w.stddev:10,.{decimals}f}  "
        f"se={w.stderr:9,.{decimals}f}  "
        f"min={w.min_val:+12,.{decimals}f}  "
        f"max={w.max_val:+12,.{decimals}f}"
    )


def add_optional(w: Welford, v: Optional[int]) -> None:
    if v is not None:
        w.update(float(v))


def normalize_clock_filter(clock: Optional[str]) -> Optional[str]:
    if clock is None:
        return None
    c = clock.strip().upper()
    if c in ("V", "VC", "VCLOCK"):
        return "VCLOCK"
    if c in ("O1", "OCXO1"):
        return "OCXO1"
    if c in ("O2", "OCXO2"):
        return "OCXO2"
    raise ValueError(f"unknown clock '{clock}', expected VCLOCK, OCXO1, or OCXO2")


def selected_clocks(clock_filter: Optional[str]) -> Tuple[str, ...]:
    return CLOCKS if clock_filter is None else (clock_filter,)


def physical_pps_dwt_from_schema(root: Dict[str, Any],
                                 frag: Dict[str, Any],
                                 forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "pps", "dwt_at_edge"),
        frag.get("pps_dwt_at_edge"),
        forensic.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_normalized_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_raw_at_edge"),
        frag.get("pps_diag_pps_edge_dwt_isr_entry_raw"),
        frag.get("pps_edge_dwt_at_edge"),
    )


def lane_prediction(frag: Dict[str, Any],
                    pred: Dict[str, Any],
                    lane: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
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


def lane_alpha_event(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str) -> Dict[str, Any]:
    """Best-effort TIMEBASE Alpha-event object for one lane.

    The firmware has gone through several schemas.  This function deliberately
    searches both current nested clock_forensics objects and older flattened
    lane-forensics locations.
    """
    candidates = (
        _nested_get(root, "clock_forensics", lane, "alpha_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event"),
        _nested_get(forensic, lane, "alpha_event"),
        _nested_get(frag, lane, "alpha_event"),
        _nested_get(root, lane, "alpha_event"),
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def _flat_prefixes(lane: str) -> Tuple[str, ...]:
    return (
        f"{lane}_forensics_",
        f"{lane}_alpha_event_",
        f"{lane}_",
    )


def _micro_values(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  lane: str,
                  *suffixes: str) -> List[Any]:
    """Values from the compact TIMEBASE_FORENSICS micro raw-cycle schema.

    Newer firmware emits flat fields directly under payload.forensics, e.g.
    v_raw/v_ema/v_fl/v_pub, o1_fl_cyc, o2_fl_err.  The older parser only knew
    how to look inside nested Alpha forensics, so these fields were present in
    TIMEBASE but invisible to raw_cycles.
    """
    prefix = LANE_MICRO_PREFIXES.get(lane)
    if not prefix:
        return []

    values: List[Any] = []
    sources = (forensic, frag, root)
    for suffix in suffixes:
        key = f"{prefix}_{suffix}"
        for source in sources:
            if isinstance(source, dict):
                values.append(source.get(key))
    return values


def _micro_first_int(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str,
                     *suffixes: str) -> Optional[int]:
    return _first_int(*_micro_values(root, frag, forensic, lane, *suffixes))


def _micro_first_bool(root: Dict[str, Any],
                      frag: Dict[str, Any],
                      forensic: Dict[str, Any],
                      lane: str,
                      *suffixes: str) -> Optional[bool]:
    return _first_bool(*_micro_values(root, frag, forensic, lane, *suffixes))


def dwt_authority_diag(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Dict[str, Any]:
    f = lane_alpha_event(root, frag, forensic, lane)
    a = f.get("dwt_authority") if isinstance(f.get("dwt_authority"), dict) else {}

    def fi(name: str, *legacy: str) -> Optional[int]:
        vals: List[Any] = [a.get(name), f.get(f"dwt_{name}")]
        vals.extend(f.get(k) for k in legacy)
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + "dwt_" + name))
            vals.append(root.get(prefix + "dwt_" + name))
        return _first_int(*vals)

    def fb(name: str, *legacy: str) -> Optional[bool]:
        vals: List[Any] = [a.get(name), f.get(f"dwt_{name}")]
        vals.extend(f.get(k) for k in legacy)
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + "dwt_" + name))
            vals.append(root.get(prefix + "dwt_" + name))
        return _first_bool(*vals)

    return {
        "original_dwt": _first_int(
            _micro_first_int(root, frag, forensic, lane, "raw", "orig"),
            a.get("original_at_event"),
            f.get("dwt_original_at_event"),
            *[frag.get(prefix + "dwt_original_at_event") for prefix in _flat_prefixes(lane)],
            *[root.get(prefix + "dwt_original_at_event") for prefix in _flat_prefixes(lane)],
        ),
        "used_dwt": _first_int(
            _micro_first_int(root, frag, forensic, lane, "pub", "used"),
            a.get("used_at_event"),
            f.get("dwt_used_at_event"),
            f.get("last_event_dwt"),
            *[frag.get(prefix + "dwt_used_at_event") for prefix in _flat_prefixes(lane)],
            *[root.get(prefix + "dwt_used_at_event") for prefix in _flat_prefixes(lane)],
        ),
        "predicted_dwt": _first_int(
            _micro_first_int(root, frag, forensic, lane, "ema"),
            a.get("predicted_at_event"),
            f.get("dwt_predicted_at_event"),
            *[frag.get(prefix + "dwt_predicted_at_event") for prefix in _flat_prefixes(lane)],
            *[root.get(prefix + "dwt_predicted_at_event") for prefix in _flat_prefixes(lane)],
        ),
        "event_from_isr_entry_raw": _first_int(
            a.get("event_from_isr_entry_raw"),
            f.get("dwt_event_from_isr_entry_raw"),
        ),
        "synthetic": _first_bool(a.get("synthetic"), f.get("dwt_synthetic")),
        "synthetic_error": _first_int(
            a.get("synthetic_error_cycles"),
            f.get("dwt_synthetic_error_cycles"),
        ),
        "published_minus_event_cycles": _first_int(
            a.get("published_minus_event_cycles"),
            f.get("dwt_published_minus_event_cycles"),
        ),
        "used_minus_event_cycles": _first_int(
            a.get("used_minus_event_cycles"),
            f.get("dwt_used_minus_event_cycles"),
        ),
    }


def dwt_gate_diag(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  lane: str) -> Dict[str, Any]:
    f = lane_alpha_event(root, frag, forensic, lane)
    gate = f.get("dwt_interval_gate") if isinstance(f.get("dwt_interval_gate"), dict) else {}

    values = {}
    for name, direct in (
        ("valid", "dwt_interval_gate_valid"),
        ("accepted", "dwt_interval_sample_accepted"),
        ("rejected", "dwt_interval_sample_rejected"),
        ("ema_updated", "dwt_interval_ema_updated"),
        ("resync_applied", "dwt_interval_resync_applied"),
    ):
        values[name] = _first_bool(gate.get(name), f.get(direct))

    for name, direct in (
        ("observed_cycles", "dwt_interval_observed_cycles"),
        ("prediction_cycles", "dwt_interval_prediction_cycles"),
        ("effective_cycles", "dwt_interval_effective_cycles"),
        ("residual_cycles", "dwt_interval_residual_cycles"),
        ("threshold_cycles", "dwt_interval_gate_threshold_cycles"),
        ("accept_count", "dwt_interval_accept_count"),
        ("reject_count", "dwt_interval_reject_count"),
        ("resync_count", "dwt_interval_resync_count"),
        ("reject_streak", "dwt_interval_reject_streak"),
    ):
        values[name] = _first_int(gate.get(name), f.get(direct))

    values["observed_cycles"] = _first_int(
        _micro_first_int(root, frag, forensic, lane, "obs", "raw_cyc"),
        values.get("observed_cycles"),
    )
    values["effective_cycles"] = _first_int(
        _micro_first_int(root, frag, forensic, lane, "eff", "ema_cyc"),
        values.get("effective_cycles"),
    )
    values["residual_cycles"] = _first_int(
        _micro_first_int(root, frag, forensic, lane, "res"),
        values.get("residual_cycles"),
    )

    return values


def regression_diag(root: Dict[str, Any],
                    frag: Dict[str, Any],
                    forensic: Dict[str, Any],
                    lane: str) -> Dict[str, Any]:
    """Extract FloorLine / lower-envelope diagnostics.

    The firmware still transports the lower-envelope result through historical
    regression_* fields.  This function names it FloorLine at the report layer.
    """
    f = lane_alpha_event(root, frag, forensic, lane)
    r = f.get("regression") if isinstance(f.get("regression"), dict) else {}

    def ri(name: str, flat_name: str = "") -> Optional[int]:
        key = flat_name or f"regression_{name}"
        vals: List[Any] = [r.get(name), f.get(key)]
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + key))
            vals.append(root.get(prefix + key))
        return _first_int(*vals)

    def rb(name: str, flat_name: str = "") -> Optional[bool]:
        key = flat_name or f"regression_{name}"
        vals: List[Any] = [r.get(name), f.get(key)]
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + key))
            vals.append(root.get(prefix + key))
        return _first_bool(*vals)

    micro_fl_dwt = _micro_first_int(root, frag, forensic, lane, "fl")
    micro_observed_dwt = _micro_first_int(root, frag, forensic, lane, "raw", "orig")
    micro_fl_interval = _micro_first_int(root, frag, forensic, lane, "fl_cyc")
    micro_observed_interval = _micro_first_int(root, frag, forensic, lane, "obs")
    micro_fl_err = _micro_first_int(root, frag, forensic, lane, "fl_err")
    micro_present = (
        micro_fl_dwt is not None or
        micro_fl_interval is not None or
        micro_fl_err is not None
    )

    return {
        "valid": _first_bool(rb("valid"), True if micro_present else None),
        "sequence": ri("sequence"),
        "sample_count": _first_int(ri("sample_count"), 1 if micro_present else None),
        "observed_dwt": _first_int(micro_observed_dwt, ri("observed_dwt_at_event")),
        "inferred_dwt": _first_int(micro_fl_dwt, ri("inferred_dwt_at_event")),
        "inferred_interval_cycles": _first_int(micro_fl_interval),
        "observed_interval_cycles": _first_int(micro_observed_interval),
        "inferred_minus_observed": _first_int(micro_fl_err, ri("inferred_minus_observed_cycles")),
        "target_counter32": ri("target_counter32_at_event"),
        "target_hardware16": ri("target_hardware16_at_event"),
        "observed_hardware16": ri("observed_hardware16_at_event"),
        "fit_error_abs_gt4_count": ri("fit_error_abs_gt4_count"),
        "fit_error_max_cycles": ri("fit_error_max_cycles"),
        "fit_error_min_cycles": ri("fit_error_min_cycles"),
    }


def lane_counter_delta(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Optional[int]:
    f = lane_alpha_event(root, frag, forensic, lane)
    return _first_int(
        f.get("counter32_delta_since_previous_event"),
        _nested_get(f, "dwt_interval_adjacency", "counter_delta_ticks"),
        _nested_get(f, "ocxo_service", "last_counter_delta_ticks"),
        frag.get(f"{lane}_forensics_counter32_delta_since_previous_event"),
        root.get(f"{lane}_forensics_counter32_delta_since_previous_event"),
        frag.get(f"{lane}_counter32_delta_since_previous_event"),
        root.get(f"{lane}_counter32_delta_since_previous_event"),
    )


def pps_count_from_schema(root: Dict[str, Any],
                          frag: Dict[str, Any],
                          forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def dwt_ppb_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[float]:
    return _first_float(
        _nested_get(frag, "stats", "dwt", "ppb"),
        _nested_get(root, "fragment", "stats", "dwt", "ppb"),
        frag.get("dwt_ppb"),
        root.get("dwt_ppb"),
    )


def collect_rows(rows: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int]:
    out: List[Dict[str, Any]] = []
    gaps = 0

    prev_pps_count: Optional[int] = None
    prev_pps_dwt: Optional[int] = None
    prev_pps_interval: Optional[int] = None
    prev_lane_dwt: Dict[str, Dict[str, Optional[int]]] = {
        lane: {"raw": None, "fl": None, "pub": None}
        for lane in CLOCKS
    }
    prev_lane_interval: Dict[str, Dict[str, Optional[int]]] = {
        lane: {"raw": None, "fl": None, "pub": None}
        for lane in CLOCKS
    }

    def reset_deltas_after_gap() -> None:
        nonlocal prev_pps_dwt, prev_pps_interval, prev_lane_dwt, prev_lane_interval
        prev_pps_dwt = None
        prev_pps_interval = None
        prev_lane_dwt = {
            lane: {"raw": None, "fl": None, "pub": None}
            for lane in CLOCKS
        }
        prev_lane_interval = {
            lane: {"raw": None, "fl": None, "pub": None}
            for lane in CLOCKS
        }

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics_root(rec)
        pred = _prediction(frag)

        pps_count = pps_count_from_schema(root, frag, forensic)
        if pps_count is None:
            continue

        if prev_pps_count is not None and pps_count != prev_pps_count + 1:
            gaps += 1
            reset_deltas_after_gap()

        pps_dwt = physical_pps_dwt_from_schema(root, frag, forensic)
        pps_actual, pps_pred, _pps_residual = lane_prediction(frag, pred, "pps")
        if pps_actual is None:
            pps_actual = _interval_from_endpoints(pps_dwt, prev_pps_dwt)
        pps_delta = _interval_delta(pps_actual, prev_pps_interval)

        row: Dict[str, Any] = {
            "pps_count": pps_count,
            "pps_actual": pps_actual,
            "pps_delta": pps_delta,
            "dwt_ppb": dwt_ppb_from_schema(root, frag),
            "lanes": {},
        }

        for lane in CLOCKS:
            lane_key = LANE_KEYS[lane]
            pub_actual, _pub_pred, _pub_res = lane_prediction(frag, pred, lane_key)
            f = lane_alpha_event(root, frag, forensic, lane_key)
            auth = dwt_authority_diag(root, frag, forensic, lane_key)
            gate = dwt_gate_diag(root, frag, forensic, lane_key)
            fl = regression_diag(root, frag, forensic, lane_key)

            original_dwt = auth["original_dwt"]
            used_dwt = auth["used_dwt"]
            predicted_dwt = auth["predicted_dwt"]
            fl_observed_dwt = fl["observed_dwt"]
            fl_inferred_dwt = fl["inferred_dwt"]

            raw_interval = gate.get("observed_cycles")
            if raw_interval is None:
                raw_interval = _interval_from_endpoints(original_dwt, prev_lane_dwt[lane]["raw"])
            if raw_interval is None:
                raw_interval = _interval_from_endpoints(fl_observed_dwt, prev_lane_dwt[lane]["raw"])
            if raw_interval is None:
                raw_interval = pub_actual

            # Legacy firmware used dwt_interval_gate.effective_cycles as an
            # EMA-authored interval.  Current EMA-retired firmware still emits
            # *_eff as a compatibility/effective interval, but it is no longer
            # a report rail.  Keep it only as a hidden fallback for old rows
            # whose published endpoint equals a predicted/effective endpoint.
            effective_interval = gate.get("effective_cycles")
            if effective_interval is None:
                effective_interval = _interval_from_endpoints(predicted_dwt, prev_lane_dwt[lane]["pub"])

            # FloorLine has two related cycle-count surfaces:
            #
            #   1. endpoint interval: fl_dwt[n] - fl_dwt[n-1]
            #   2. fitted/slope interval: the lower-envelope fit's current
            #      one-second interval estimate, emitted as *_fl_cyc
            #
            # The visible *_fl column must use the endpoint interval first,
            # because *_pub is also endpoint-derived.  Otherwise raw_cycles
            # compares a fitted interval against a published endpoint interval
            # and falsely suggests that FloorLine is not the publication source.
            fl_reported_interval = fl.get("inferred_interval_cycles")
            fl_endpoint_interval = _interval_from_endpoints(
                fl_inferred_dwt, prev_lane_dwt[lane]["fl"]
            )
            fl_interval = fl_endpoint_interval
            if fl_interval is None:
                fl_interval = fl_reported_interval

            pub_interval = _interval_from_endpoints(used_dwt, prev_lane_dwt[lane]["pub"])
            if pub_interval is None and used_dwt is not None:
                # The micro schema gives endpoint candidates plus per-algorithm
                # intervals.  When the published endpoint equals one candidate,
                # use that candidate's interval even on the first printed row.
                if (
                    fl_inferred_dwt is not None and
                    used_dwt == fl_inferred_dwt and
                    fl_interval is not None
                ):
                    pub_interval = fl_interval
                elif (
                    predicted_dwt is not None and
                    used_dwt == predicted_dwt and
                    effective_interval is not None
                ):
                    pub_interval = effective_interval
                elif (
                    original_dwt is not None and
                    used_dwt == original_dwt and
                    raw_interval is not None
                ):
                    pub_interval = raw_interval
            if pub_interval is None:
                pub_interval = _first_int(
                    f.get("dwt_cycles_between_edges"),
                    pub_actual,
                )

            data = {
                "raw": raw_interval,
                "raw_delta": _interval_delta(raw_interval, prev_lane_interval[lane]["raw"]),
                "fl": fl_interval,
                "fl_delta": _interval_delta(fl_interval, prev_lane_interval[lane]["fl"]),
                "pub": pub_interval,
                "pub_delta": _interval_delta(pub_interval, prev_lane_interval[lane]["pub"]),
                "counter_delta": lane_counter_delta(root, frag, forensic, lane_key),
                "fl_edge_minus_observed": fl.get("inferred_minus_observed"),
                "pub_minus_fl": _interval_delta(pub_interval, fl_interval),
                "pub_edge_minus_fl_edge": (
                    _signed_delta_u32(used_dwt, fl_inferred_dwt)
                    if used_dwt is not None and fl_inferred_dwt is not None
                    else None
                ),
                "fl_fit_minus_endpoint_interval": _interval_delta(
                    fl_reported_interval, fl_endpoint_interval
                ),
                "fl_sample_count": fl.get("sample_count"),
                "fl_valid": fl.get("valid"),
                "pub_synthetic": auth.get("synthetic"),
                "pub_used_dwt": used_dwt,
                "raw_dwt": _first_int(original_dwt, fl_observed_dwt),
                "fl_dwt": fl_inferred_dwt,
            }
            row["lanes"][lane] = data

            for algo in ("raw", "fl", "pub"):
                if data[algo] is not None:
                    prev_lane_interval[lane][algo] = data[algo]

            if original_dwt is not None:
                prev_lane_dwt[lane]["raw"] = original_dwt
            elif fl_observed_dwt is not None:
                prev_lane_dwt[lane]["raw"] = fl_observed_dwt


            if fl_inferred_dwt is not None:
                prev_lane_dwt[lane]["fl"] = fl_inferred_dwt
            if used_dwt is not None:
                prev_lane_dwt[lane]["pub"] = used_dwt

        out.append(row)

        prev_pps_count = pps_count
        if pps_dwt is not None:
            prev_pps_dwt = pps_dwt
        if pps_actual is not None:
            prev_pps_interval = pps_actual

    return out, gaps


def available_columns(collected: List[Dict[str, Any]],
                      clocks: Tuple[str, ...]) -> Dict[str, List[str]]:
    # The four algorithm rails are the purpose of this report.  Keep them
    # visible even when the current campaign/build does not expose one of the
    # rails yet; otherwise a missing FloorLine extraction looks like a report
    # formatting decision instead of useful evidence.
    always = [
        "raw", "raw_delta",
        "fl", "fl_delta",
        "pub", "pub_delta",
    ]
    optional = ["counter_delta"]

    out: Dict[str, List[str]] = {}
    for lane in clocks:
        cols = list(always)
        for col in optional:
            if any(row["lanes"][lane].get(col) is not None for row in collected):
                cols.append(col)
        out[lane] = cols
    return out


def _col_label(lane: str, col: str) -> str:
    p = PREFIXES[lane]
    return {
        "raw": f"{p}_raw",
        "raw_delta": f"{p}_rawΔ",
        "fl": f"{p}_fl",
        "fl_delta": f"{p}_flΔ",
        "pub": f"{p}_pub",
        "pub_delta": f"{p}_pubΔ",
        "counter_delta": f"{p}_cΔ",
    }[col]


def _col_width(col: str) -> int:
    return {
        "raw": 13,
        "raw_delta": 8,
        "fl": 13,
        "fl_delta": 8,
        "pub": 13,
        "pub_delta": 8,
        "counter_delta": 11,
    }[col]


def _col_signed(col: str) -> bool:
    return col in ("raw_delta", "fl_delta", "pub_delta")


def _headers_for(clocks: Tuple[str, ...],
                 cols: Dict[str, List[str]],
                 include_dwt_ppb: bool) -> Tuple[str, str]:
    parts = [
        f"{'pps':>6s}",
        f"{'p_act':>13s}",
        f"{'p_Δ':>8s}",
    ]
    for lane in clocks:
        for col in cols[lane]:
            label = _col_label(lane, col)
            parts.append(f"{label:>{_col_width(col)}s}")
    if include_dwt_ppb:
        parts.append(f"{'dwt_ppb':>10s}")
    return "  ".join(parts), "  ".join("─" * len(p) for p in parts)


def _row_line(row: Dict[str, Any],
              clocks: Tuple[str, ...],
              cols: Dict[str, List[str]],
              include_dwt_ppb: bool) -> str:
    parts = [
        f"{row['pps_count']:>6d}",
        _fmt_int(row["pps_actual"], 13),
        _fmt_int(row["pps_delta"], 8, signed=True),
    ]
    for lane in clocks:
        data = row["lanes"][lane]
        for col in cols[lane]:
            parts.append(
                _fmt_int(data[col], _col_width(col), signed=_col_signed(col))
            )
    if include_dwt_ppb:
        parts.append(_fmt_float(row["dwt_ppb"], 10, 3, signed=True))
    return "  ".join(parts)


def _has_any(collected: List[Dict[str, Any]], lane: str, key: str) -> bool:
    return any(row["lanes"][lane].get(key) is not None for row in collected)


def analyze(campaign: str,
            limit: int = 0,
            clock_filter: Optional[str] = None,
            slip_view: bool = False) -> None:
    if slip_view:
        raise SystemExit("--slip was retired from this compact FloorLine report; use an older raw_cycles if SlipLedger courtroom output is needed")

    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    clock_filter = normalize_clock_filter(clock_filter) if clock_filter else None
    clocks = selected_clocks(clock_filter)

    collected, gaps = collect_rows(rows)
    if limit:
        collected = collected[:limit]

    cols = available_columns(collected, clocks)
    include_dwt_ppb = any(row.get("dwt_ppb") is not None for row in collected)

    clock_label = "ALL" if clock_filter is None else clock_filter
    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={clock_label})")
    print()
    print("Raw / FloorLine / published cycle audit")
    print("══════════════════════════════════════")
    print("  PPS is always the base rail.")
    print("  *_raw = evidence interval, *_fl = FloorLine interval,")
    print("  *_pub = subscriber-facing published interval.")
    print("  *Δ columns are per-rail cycle-count residuals: interval[n] - interval[n-1].")
    print("  Core rail columns are always shown; missing FL data is shown as --- on purpose.")
    print()

    header, sep = _headers_for(clocks, cols, include_dwt_ppb)
    print(header)
    print(sep)
    for row in collected:
        print(_row_line(row, clocks, cols, include_dwt_ppb))

    print()
    print(f"Rows shown: {len(collected):,}")
    print(f"Gaps:       {gaps:,}")
    print()

    stats: Dict[str, Welford] = {
        "pps_actual": Welford(),
        "pps_delta": Welford(),
    }
    for lane in clocks:
        for key in ("raw", "raw_delta",
                    "fl", "fl_delta", "pub", "pub_delta",
                    "counter_delta", "fl_edge_minus_observed",
                    "pub_minus_fl", "pub_edge_minus_fl_edge",
                    "fl_fit_minus_endpoint_interval"):
            stats[f"{lane}.{key}"] = Welford()

    coverage: Dict[str, Dict[str, int]] = {
        lane: {key: 0 for key in ("raw", "fl", "pub", "counter_delta")}
        for lane in clocks
    }

    for row in collected:
        add_optional(stats["pps_actual"], row["pps_actual"])
        add_optional(stats["pps_delta"], row["pps_delta"])
        for lane in clocks:
            data = row["lanes"][lane]
            for key in ("raw", "raw_delta",
                        "fl", "fl_delta", "pub", "pub_delta",
                        "counter_delta", "fl_edge_minus_observed",
                        "pub_minus_fl", "pub_edge_minus_fl_edge",
                        "fl_fit_minus_endpoint_interval"):
                add_optional(stats[f"{lane}.{key}"], data.get(key))
            for key in coverage[lane]:
                if data.get(key) is not None:
                    coverage[lane][key] += 1

    print("Schema coverage")
    print("═══════════════")
    print(f"  rows = {len(collected):,}")
    for lane in clocks:
        c = coverage[lane]
        print(
            f"  {lane:<6s} raw={c['raw']:,}  "
            f"fl={c['fl']:,}  pub={c['pub']:,}  cΔ={c['counter_delta']:,}"
        )
    print()

    print("Summary")
    print("═══════")
    _print_welford("PPS actual cycles", stats["pps_actual"])
    _print_welford("PPS first-difference residual", stats["pps_delta"])

    for lane in clocks:
        print()
        _print_welford(f"{lane} raw interval", stats[f"{lane}.raw"])
        _print_welford(f"{lane} raw cycle-count residual", stats[f"{lane}.raw_delta"])
        _print_welford(f"{lane} FloorLine interval", stats[f"{lane}.fl"])
        _print_welford(f"{lane} FloorLine cycle-count residual", stats[f"{lane}.fl_delta"])
        _print_welford(f"{lane} published interval", stats[f"{lane}.pub"])
        _print_welford(f"{lane} published cycle-count residual", stats[f"{lane}.pub_delta"])
        _print_welford(f"{lane} published interval - FloorLine interval", stats[f"{lane}.pub_minus_fl"])
        _print_welford(f"{lane} published edge - FloorLine edge", stats[f"{lane}.pub_edge_minus_fl_edge"])
        _print_welford(f"{lane} reported FL fit interval - endpoint FL interval", stats[f"{lane}.fl_fit_minus_endpoint_interval"])
        _print_welford(f"{lane} FL edge - observed edge", stats[f"{lane}.fl_edge_minus_observed"])
        _print_welford(f"{lane} counter32 delta", stats[f"{lane}.counter_delta"], unit="ticks")

    print()
    print("Notes")
    print("═════")
    print("  • The old EMA/effective columns were removed from the normal view;")
    print("    this report is now focused on the publication decision: raw vs FloorLine vs pub.")
    print("  • *_pub is the ground-truth subscriber-facing interval.")
    print("  • *_fl is now endpoint-derived first: fl_dwt[n] - fl_dwt[n-1].")
    print("    The fitted/slope FloorLine interval is used only as a fallback and")
    print("    audited as 'reported FL fit interval - endpoint FL interval'.")
    print("  • In FloorLine-authored firmware, 'published edge - FloorLine edge'")
    print("    and 'published interval - FloorLine interval' should be zero.")
    print("  • *Δ columns are cycle-count residuals computed separately per rail:")
    print("    interval[n] - interval[n-1].")
    print("  • FloorLine columns are intentionally always visible.  If *_fl stays ---,")
    print("    the TIMEBASE row stream did not expose a FloorLine endpoint that this parser found.")
    print("  • Use --clock VCLOCK, --clock OCXO1, or --clock OCXO2 to keep the row narrow.")
    print()


def parse_args(argv: List[str]) -> Tuple[str, int, Optional[str], bool]:
    if len(argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit] [clock]")
        print("       raw_cycles <campaign_name> --clock OCXO2 [limit]")
        print("       raw_cycles <campaign_name> --limit 300 --clock VCLOCK")
        raise SystemExit(1)

    campaign = argv[1]
    limit = 0
    clock: Optional[str] = None
    slip_view = False

    i = 2
    while i < len(argv):
        arg = argv[i]
        if arg == "--slip":
            slip_view = True
            i += 1
            continue
        if arg == "--clock":
            if i + 1 >= len(argv):
                raise SystemExit("--clock requires VCLOCK, OCXO1, or OCXO2")
            clock = argv[i + 1]
            i += 2
            continue
        if arg.startswith("--clock="):
            clock = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--limit":
            if i + 1 >= len(argv):
                raise SystemExit("--limit requires an integer")
            try:
                limit = int(argv[i + 1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 2
            continue
        if arg.startswith("--limit="):
            try:
                limit = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 1
            continue
        if arg.upper() in CLOCKS or arg.upper() in ("V", "VC", "O1", "O2"):
            clock = arg
            i += 1
            continue
        try:
            limit = int(arg)
        except ValueError as exc:
            raise SystemExit(f"unknown argument '{arg}'") from exc
        i += 1

    return campaign, limit, normalize_clock_filter(clock) if clock else None, slip_view


def main() -> None:
    campaign, limit, clock, slip_view = parse_args(sys.argv)
    analyze(campaign, limit, clock, slip_view)


if __name__ == "__main__":
    main()
