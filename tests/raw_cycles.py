"""
ZPNet Raw Cycles — residual-separated prediction + adjacency audit.

Reads TIMEBASE rows for a campaign and prints a compact one-second DWT-cycle
audit. The report stays deliberately one-line-per-second.

Default view:
    PPS + VCLOCK + OCXO1 + OCXO2

Focused view:
    PPS + one selected clock

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> --clock OCXO2 [limit]

Clock filter:
    VCLOCK, OCXO1, OCXO2

Column doctrine:
    p_pred is the PPS static cycle-count prediction interval.
    p_act  is the physical PPS actual interval.
    p_Δ    is physical PPS actual[n] - physical PPS actual[n-1].

    *_raw   is the raw/evidence interval from TIMEBASE_FORENSICS
            dwt_interval_gate.observed_cycles when available.
    *_rawΔ  is the old-style raw first-difference residual:
            raw[n] - raw[n-1].
    *_gPred is the EMA/gate prediction interval from process_interrupt.
    *_gateR is the EMA/gate residual: raw - gPred.
    *_pub   is the subscriber-facing published/smoothed interval from the
            firmware static prediction surface, falling back to gate effective.
    *_pubΔ  is the old-style published first-difference residual:
            pub[n] - pub[n-1].

    *_adj is OCXO counter-adjacency result: OK / REJ / BAD / ---.
    *_aΔ  is adjacency counter_delta_ticks - expected_counter_delta_ticks.
    *_aN  is cumulative adjacency reject_count.
    *_svc is compact OCXO compare-service offset in 10 MHz ticks.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


EXPECTED_OCXO_CDELTA = 10_000_000
EXCURSION_THRESHOLD_CYCLES = 100
CLOCKS = ("VCLOCK", "OCXO1", "OCXO2")


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
        text = v.strip().lower()
        if text in ("true", "t", "yes", "y", "1"):
            return True
        if text in ("false", "f", "no", "n", "0"):
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


def _first_path_int(*paths: Tuple[Dict[str, Any], Tuple[str, ...]]) -> Optional[int]:
    for obj, path in paths:
        out = _as_int(_nested_get(obj, *path))
        if out is not None:
            return out
    return None


def _first_path_bool(*paths: Tuple[Dict[str, Any], Tuple[str, ...]]) -> Optional[bool]:
    for obj, path in paths:
        out = _as_bool(_nested_get(obj, *path))
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


def vclock_preferred_dwt_from_schema(root: Dict[str, Any],
                                     frag: Dict[str, Any],
                                     forensic: Dict[str, Any]) -> Tuple[Optional[int], str]:
    lattice_dwt = _first_int(
        _nested_get(frag, "dwt", "at_pps_vclock"),
        frag.get("dwt_at_pps_vclock"),
        forensic.get("dwt_at_pps_vclock"),
        root.get("dwt_at_pps_vclock"),
    )
    if lattice_dwt is not None:
        return lattice_dwt, "lattice"

    forensic_dwt = _first_int(
        _nested_get(forensic, "vclock", "forensics", "last_event_dwt"),
        _nested_get(frag, "vclock", "forensics", "last_event_dwt"),
        _nested_get(root, "vclock", "forensics", "last_event_dwt"),
        _nested_get(root, "vclock", "alpha_event", "last_event_dwt"),
    )
    if forensic_dwt is not None:
        return forensic_dwt, "forensics"

    return None, "---"


def pps_vclock_phase_cycles_from_schema(root: Dict[str, Any],
                                        frag: Dict[str, Any],
                                        forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "pps", "vclock_phase_cycles"),
        frag.get("pps_vclock_phase_cycles"),
        forensic.get("pps_vclock_phase_cycles"),
        root.get("pps_vclock_phase_cycles"),
    )


def lane_forensics_obj(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Dict[str, Any]:
    candidates = (
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
        _nested_get(root, "fragment", lane, "forensics"),
        _nested_get(root, "clock_forensics", lane, "alpha_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_service_obj(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str) -> Dict[str, Any]:
    candidates = (
        _nested_get(forensic, lane, "service"),
        _nested_get(frag, lane, "service"),
        _nested_get(root, lane, "service"),
        _nested_get(root, "fragment", lane, "service"),
        _nested_get(root, "clock_forensics", lane, "alpha_event", "ocxo_service"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event", "ocxo_service"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_last_event_dwt(root: Dict[str, Any],
                        frag: Dict[str, Any],
                        forensic: Dict[str, Any],
                        lane: str) -> Optional[int]:
    prefix = f"{lane}_forensics_"
    f = lane_forensics_obj(root, frag, forensic, lane)
    return _first_int(
        f.get("last_event_dwt"),
        f.get("alpha_event_last_event_dwt"),
        frag.get(prefix + "last_event_dwt"),
        root.get(prefix + "last_event_dwt"),
        frag.get(f"{lane}_alpha_event_last_event_dwt"),
        root.get(f"{lane}_alpha_event_last_event_dwt"),
    )


def lane_counter_delta(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Optional[int]:
    f = lane_forensics_obj(root, frag, forensic, lane)
    prefix = f"{lane}_forensics_"
    return _first_int(
        f.get("counter32_delta_since_previous_event"),
        frag.get(prefix + "counter32_delta_since_previous_event"),
        root.get(prefix + "counter32_delta_since_previous_event"),
        frag.get(f"{lane}_counter32_delta_since_previous_event"),
        root.get(f"{lane}_counter32_delta_since_previous_event"),
    )


def vclock_counter32_at_event(root: Dict[str, Any],
                              frag: Dict[str, Any],
                              forensic: Dict[str, Any]) -> Optional[int]:
    f = lane_forensics_obj(root, frag, forensic, "vclock")
    return _first_int(
        f.get("last_event_counter32"),
        _nested_get(frag, "vclock", "counter32_at_pps_vclock"),
        frag.get("counter32_at_pps_vclock"),
        forensic.get("counter32_at_pps_vclock"),
        root.get("counter32_at_pps_vclock"),
    )


def dwt_gate_diag(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  lane: str) -> Dict[str, Any]:
    f = lane_forensics_obj(root, frag, forensic, lane)
    gate = f.get("dwt_interval_gate") if isinstance(f.get("dwt_interval_gate"), dict) else {}
    prefix = f"{lane}_forensics_"

    return {
        "valid": _first_bool(gate.get("valid"), f.get("dwt_interval_gate_valid"),
                             frag.get(prefix + "dwt_interval_gate_valid"),
                             root.get(prefix + "dwt_interval_gate_valid")),
        "accepted": _first_bool(gate.get("accepted"), f.get("dwt_interval_sample_accepted"),
                                frag.get(prefix + "dwt_interval_sample_accepted"),
                                root.get(prefix + "dwt_interval_sample_accepted")),
        "rejected": _first_bool(gate.get("rejected"), f.get("dwt_interval_sample_rejected"),
                                frag.get(prefix + "dwt_interval_sample_rejected"),
                                root.get(prefix + "dwt_interval_sample_rejected")),
        "ema_updated": _first_bool(gate.get("ema_updated"), f.get("dwt_interval_ema_updated"),
                                   frag.get(prefix + "dwt_interval_ema_updated"),
                                   root.get(prefix + "dwt_interval_ema_updated")),
        "observed_cycles": _first_int(gate.get("observed_cycles"), f.get("dwt_interval_observed_cycles"),
                                      frag.get(prefix + "dwt_interval_observed_cycles"),
                                      root.get(prefix + "dwt_interval_observed_cycles")),
        "prediction_cycles": _first_int(gate.get("prediction_cycles"), f.get("dwt_interval_prediction_cycles"),
                                        frag.get(prefix + "dwt_interval_prediction_cycles"),
                                        root.get(prefix + "dwt_interval_prediction_cycles")),
        "effective_cycles": _first_int(gate.get("effective_cycles"), f.get("dwt_interval_effective_cycles"),
                                       frag.get(prefix + "dwt_interval_effective_cycles"),
                                       root.get(prefix + "dwt_interval_effective_cycles")),
        "residual_cycles": _first_int(gate.get("residual_cycles"), f.get("dwt_interval_residual_cycles"),
                                      frag.get(prefix + "dwt_interval_residual_cycles"),
                                      root.get(prefix + "dwt_interval_residual_cycles")),
        "threshold_cycles": _first_int(gate.get("threshold_cycles"), f.get("dwt_interval_gate_threshold_cycles"),
                                       frag.get(prefix + "dwt_interval_gate_threshold_cycles"),
                                       root.get(prefix + "dwt_interval_gate_threshold_cycles")),
        "accept_count": _first_int(gate.get("accept_count"), f.get("dwt_interval_accept_count"),
                                   frag.get(prefix + "dwt_interval_accept_count"),
                                   root.get(prefix + "dwt_interval_accept_count")),
        "reject_count": _first_int(gate.get("reject_count"), f.get("dwt_interval_reject_count"),
                                   frag.get(prefix + "dwt_interval_reject_count"),
                                   root.get(prefix + "dwt_interval_reject_count")),
        "resync_applied": _first_bool(gate.get("resync_applied"), f.get("dwt_interval_resync_applied"),
                                      frag.get(prefix + "dwt_interval_resync_applied"),
                                      root.get(prefix + "dwt_interval_resync_applied")),
        "resync_count": _first_int(gate.get("resync_count"), f.get("dwt_interval_resync_count"),
                                   frag.get(prefix + "dwt_interval_resync_count"),
                                   root.get(prefix + "dwt_interval_resync_count")),
        "reject_streak": _first_int(gate.get("reject_streak"), f.get("dwt_interval_reject_streak"),
                                    frag.get(prefix + "dwt_interval_reject_streak"),
                                    root.get(prefix + "dwt_interval_reject_streak")),
        "synthetic": _first_bool(f.get("dwt_synthetic"),
                                 frag.get(prefix + "dwt_synthetic"),
                                 root.get(prefix + "dwt_synthetic")),
        "synthetic_error": _first_int(f.get("dwt_synthetic_error_cycles"),
                                      frag.get(prefix + "dwt_synthetic_error_cycles"),
                                      root.get(prefix + "dwt_synthetic_error_cycles")),
        "original_dwt": _first_int(f.get("dwt_original_at_event"),
                                   frag.get(prefix + "dwt_original_at_event"),
                                   root.get(prefix + "dwt_original_at_event")),
        "used_dwt": _first_int(f.get("dwt_used_at_event"),
                               frag.get(prefix + "dwt_used_at_event"),
                               root.get(prefix + "dwt_used_at_event")),
    }


def adjacency_diag(root: Dict[str, Any],
                   frag: Dict[str, Any],
                   forensic: Dict[str, Any],
                   lane: str) -> Dict[str, Any]:
    f = lane_forensics_obj(root, frag, forensic, lane)
    adj = f.get("dwt_interval_adjacency") if isinstance(f.get("dwt_interval_adjacency"), dict) else {}
    prefix = f"{lane}_forensics_"

    valid = _first_bool(
        adj.get("valid"),
        f.get("dwt_interval_adjacency_gate_valid"),
        frag.get(prefix + "dwt_interval_adjacency_gate_valid"),
        root.get(prefix + "dwt_interval_adjacency_gate_valid"),
    )
    ok = _first_bool(
        adj.get("ok"),
        f.get("dwt_interval_adjacency_ok"),
        frag.get(prefix + "dwt_interval_adjacency_ok"),
        root.get(prefix + "dwt_interval_adjacency_ok"),
    )
    rejected = _first_bool(
        adj.get("rejected"),
        f.get("dwt_interval_adjacency_rejected"),
        frag.get(prefix + "dwt_interval_adjacency_rejected"),
        root.get(prefix + "dwt_interval_adjacency_rejected"),
    )
    counter_delta = _first_int(
        adj.get("counter_delta_ticks"),
        f.get("dwt_interval_counter_delta_ticks"),
        frag.get(prefix + "dwt_interval_counter_delta_ticks"),
        root.get(prefix + "dwt_interval_counter_delta_ticks"),
    )
    expected = _first_int(
        adj.get("expected_counter_delta_ticks"),
        f.get("dwt_interval_expected_counter_delta_ticks"),
        frag.get(prefix + "dwt_interval_expected_counter_delta_ticks"),
        root.get(prefix + "dwt_interval_expected_counter_delta_ticks"),
    )
    reject_count = _first_int(
        adj.get("reject_count"),
        f.get("dwt_interval_adjacency_reject_count"),
        frag.get(prefix + "dwt_interval_adjacency_reject_count"),
        root.get(prefix + "dwt_interval_adjacency_reject_count"),
    )

    delta_error = (
        counter_delta - expected
        if counter_delta is not None and expected is not None
        else None
    )

    return {
        "valid": valid,
        "ok": ok,
        "rejected": rejected,
        "counter_delta": counter_delta,
        "expected": expected,
        "delta_error": delta_error,
        "reject_count": reject_count,
    }


def adjacency_tag(adj: Dict[str, Any]) -> str:
    if not adj.get("valid"):
        return "---"
    if adj.get("rejected"):
        return "REJ"
    if adj.get("ok"):
        return "OK"
    return "BAD"


def service_diag(root: Dict[str, Any],
                 frag: Dict[str, Any],
                 forensic: Dict[str, Any],
                 lane: str) -> Dict[str, Optional[int]]:
    service = lane_service_obj(root, frag, forensic, lane)
    prefix = f"{lane}_forensics_"
    return {
        "service_class": _first_int(
            service.get("class"),
            service.get("service_class"),
            frag.get(prefix + "service_class"),
            root.get(prefix + "service_class"),
        ),
        "service_offset": _first_int(
            service.get("offset_ticks"),
            service.get("offset_signed_ticks"),
            frag.get(prefix + "service_offset_ticks"),
            root.get(prefix + "service_offset_ticks"),
        ),
        "arm_to_isr": _first_int(
            service.get("arm_to_isr_ticks"),
            frag.get(prefix + "arm_to_isr_ticks"),
            root.get(prefix + "arm_to_isr_ticks"),
        ),
        "last_counter_delta": _first_int(
            service.get("last_counter_delta_ticks"),
            frag.get(prefix + "last_counter_delta_ticks"),
            root.get(prefix + "last_counter_delta_ticks"),
        ),
    }


def dwt_ppb_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[float]:
    return _first_float(
        _nested_get(frag, "stats", "dwt", "ppb"),
        _nested_get(root, "fragment", "stats", "dwt", "ppb"),
        frag.get("dwt_ppb"),
        root.get("dwt_ppb"),
    )


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



def _interval_delta(current: Optional[int], previous: Optional[int]) -> Optional[int]:
    if current is None or previous is None:
        return None
    return current - previous


def _lane_cycle_surfaces(gate: Dict[str, Any],
                         pub_actual: Optional[int],
                         pub_pred: Optional[int],
                         pub_res: Optional[int]) -> Dict[str, Any]:
    """Return the raw, gate, and published interval surfaces for one lane.

    The key doctrine is that these are three different questions:

      rawΔ  : did the raw evidence interval change from the prior raw interval?
      gateR : did raw evidence disagree with the EMA/gate prediction?
      pubΔ  : did the published interval change from the prior published interval?

    Older rows may not expose dwt_interval_gate. In that case raw falls back to
    the published/static surface so legacy reports still render, while gateR is
    left blank unless a real gate prediction exists.
    """
    gate_valid = bool(gate.get("valid"))

    raw = gate.get("observed_cycles") if gate_valid else None
    if raw is None:
        raw = pub_actual

    gate_pred = gate.get("prediction_cycles") if gate_valid else None
    gate_res = gate.get("residual_cycles") if gate_valid else None
    if gate_res is None and raw is not None and gate_pred is not None:
        gate_res = raw - gate_pred

    published = pub_actual
    if published is None and gate_valid:
        published = gate.get("effective_cycles")
    if published is None and pub_pred is not None and pub_res is not None:
        published = pub_pred + pub_res

    return {
        "raw": raw,
        "gate_pred": gate_pred,
        "gate_res": gate_res,
        "published": published,
        "gate_valid": gate_valid,
        "gate_accepted": gate.get("accepted"),
        "gate_rejected": gate.get("rejected"),
        "gate_reject_count": gate.get("reject_count"),
        "gate_resync_applied": gate.get("resync_applied"),
        "synthetic": gate.get("synthetic"),
        "synthetic_error": gate.get("synthetic_error"),
    }


def _headers_for(clock_filter: Optional[str]) -> Tuple[str, str]:
    parts = [
        f"{'pps':>6s}",
        f"{'p_pred':>13s}",
        f"{'p_act':>13s}",
        f"{'p_Δ':>8s}",
    ]

    if clock_filter is None or clock_filter == "VCLOCK":
        parts.extend([
            f"{'v_raw':>13s}",
            f"{'v_rawΔ':>8s}",
            f"{'v_gPred':>13s}",
            f"{'v_gateR':>8s}",
            f"{'v_pub':>13s}",
            f"{'v_pubΔ':>8s}",
            f"{'phase':>5s}",
            f"{'v_cΔ':>11s}",
        ])

    for lane in ("OCXO1", "OCXO2"):
        if clock_filter is not None and clock_filter != lane:
            continue
        prefix = "o1" if lane == "OCXO1" else "o2"
        parts.extend([
            f"{prefix + '_raw':>13s}",
            f"{prefix + '_rawΔ':>8s}",
            f"{prefix + '_gPred':>13s}",
            f"{prefix + '_gateR':>8s}",
            f"{prefix + '_pub':>13s}",
            f"{prefix + '_pubΔ':>8s}",
            f"{prefix + '_adj':>5s}",
            f"{prefix + '_aΔ':>9s}",
            f"{prefix + '_aN':>6s}",
            f"{prefix + '_svc':>7s}",
            f"{prefix + '_cΔ':>11s}",
        ])

    parts.append(f"{'dwt_ppb':>10s}")
    header = "  ".join(parts)
    sep = "  ".join("─" * len(p) for p in parts)
    return header, sep


def _row_line(row: Dict[str, Any], clock_filter: Optional[str]) -> str:
    parts = [
        f"{row['pps_count']:>6d}",
        _fmt_int(row["pps_pred"], 13),
        _fmt_int(row["pps_actual"], 13),
        _fmt_int(row["pps_delta"], 8, signed=True),
    ]

    if clock_filter is None or clock_filter == "VCLOCK":
        parts.extend([
            _fmt_int(row["v_raw"], 13),
            _fmt_int(row["v_raw_delta"], 8, signed=True),
            _fmt_int(row["v_gate_pred"], 13),
            _fmt_int(row["v_gate_res"], 8, signed=True),
            _fmt_int(row["v_pub"], 13),
            _fmt_int(row["v_pub_delta"], 8, signed=True),
            _fmt_int(row["phase"], 5),
            _fmt_int(row["v_cdelta"], 11),
        ])

    for lane in ("OCXO1", "OCXO2"):
        if clock_filter is not None and clock_filter != lane:
            continue
        key = "o1" if lane == "OCXO1" else "o2"
        data = row[key]
        parts.extend([
            _fmt_int(data["raw"], 13),
            _fmt_int(data["raw_delta"], 8, signed=True),
            _fmt_int(data["gate_pred"], 13),
            _fmt_int(data["gate_res"], 8, signed=True),
            _fmt_int(data["pub"], 13),
            _fmt_int(data["pub_delta"], 8, signed=True),
            _fmt_str(data["adj_tag"], 5),
            _fmt_int(data["adj_delta_error"], 9, signed=True),
            _fmt_int(data["adj_reject_count"], 6),
            _fmt_int(data["service_offset"], 7, signed=True),
            _fmt_int(data["counter_delta"], 11),
        ])

    parts.append(_fmt_float(row["dwt_ppb"], 10, 3, signed=True))
    return "  ".join(parts)


def collect_rows(rows: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int]:
    out: List[Dict[str, Any]] = []
    gaps = 0

    prev_pps_count: Optional[int] = None
    prev_physical_pps_dwt: Optional[int] = None
    prev_vclock_dwt: Optional[int] = None
    prev_vclock_counter32: Optional[int] = None

    prev_pps_interval: Optional[int] = None
    prev_raw_interval: Dict[str, Optional[int]] = {
        "vclock": None,
        "ocxo1": None,
        "ocxo2": None,
    }
    prev_pub_interval: Dict[str, Optional[int]] = {
        "vclock": None,
        "ocxo1": None,
        "ocxo2": None,
    }

    def reset_deltas_after_gap() -> None:
        nonlocal prev_physical_pps_dwt, prev_vclock_dwt, prev_vclock_counter32
        nonlocal prev_pps_interval, prev_raw_interval, prev_pub_interval
        prev_physical_pps_dwt = None
        prev_vclock_dwt = None
        prev_vclock_counter32 = None
        prev_pps_interval = None
        prev_raw_interval = {"vclock": None, "ocxo1": None, "ocxo2": None}
        prev_pub_interval = {"vclock": None, "ocxo1": None, "ocxo2": None}

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics_root(rec)
        pred = _prediction(frag)

        pps_count = _first_int(
            root.get("pps_count"),
            frag.get("pps_count"),
            forensic.get("pps_count"),
            frag.get("teensy_pps_vclock_count"),
            frag.get("teensy_pps_count"),
        )
        if pps_count is None:
            continue

        if prev_pps_count is not None and pps_count != prev_pps_count + 1:
            gaps += 1
            reset_deltas_after_gap()

        physical_pps_dwt = physical_pps_dwt_from_schema(root, frag, forensic)
        vclock_dwt, _ = vclock_preferred_dwt_from_schema(root, frag, forensic)

        phase = pps_vclock_phase_cycles_from_schema(root, frag, forensic)
        if phase is None and physical_pps_dwt is not None and vclock_dwt is not None:
            phase = abs(_signed_delta_u32(vclock_dwt, physical_pps_dwt))

        pps_actual, pps_pred, pps_static_res = lane_prediction(frag, pred, "pps")
        v_pub_actual, v_pub_pred, _v_pub_res = lane_prediction(frag, pred, "vclock")
        o1_pub_actual, o1_pub_pred, _o1_pub_res = lane_prediction(frag, pred, "ocxo1")
        o2_pub_actual, o2_pub_pred, _o2_pub_res = lane_prediction(frag, pred, "ocxo2")

        if pps_actual is None and physical_pps_dwt is not None and prev_physical_pps_dwt is not None:
            pps_actual = _delta_u32(physical_pps_dwt, prev_physical_pps_dwt)
        pps_delta = _interval_delta(pps_actual, prev_pps_interval)

        v_gate = dwt_gate_diag(root, frag, forensic, "vclock")
        v_surface = _lane_cycle_surfaces(v_gate, v_pub_actual, v_pub_pred, _v_pub_res)
        v_raw_delta = _interval_delta(v_surface["raw"], prev_raw_interval["vclock"])
        v_pub_delta = _interval_delta(v_surface["published"], prev_pub_interval["vclock"])

        vclock_event_counter32 = vclock_counter32_at_event(root, frag, forensic)
        v_cdelta = lane_counter_delta(root, frag, forensic, "vclock")
        if v_cdelta is None and vclock_event_counter32 is not None and prev_vclock_counter32 is not None:
            v_cdelta = _delta_u32(vclock_event_counter32, prev_vclock_counter32)

        def lane_data(lane_name: str,
                      pub_actual: Optional[int],
                      pub_pred: Optional[int],
                      pub_res: Optional[int]) -> Dict[str, Any]:
            gate = dwt_gate_diag(root, frag, forensic, lane_name)
            adj = adjacency_diag(root, frag, forensic, lane_name)
            svc = service_diag(root, frag, forensic, lane_name)
            cdelta = lane_counter_delta(root, frag, forensic, lane_name)
            if cdelta is None:
                cdelta = svc.get("last_counter_delta")

            surface = _lane_cycle_surfaces(gate, pub_actual, pub_pred, pub_res)
            raw_delta = _interval_delta(surface["raw"], prev_raw_interval[lane_name])
            pub_delta = _interval_delta(surface["published"], prev_pub_interval[lane_name])

            return {
                "raw": surface["raw"],
                "raw_delta": raw_delta,
                "gate_pred": surface["gate_pred"],
                "gate_res": surface["gate_res"],
                "pub": surface["published"],
                "pub_delta": pub_delta,
                "gate_valid": surface["gate_valid"],
                "gate_accepted": surface["gate_accepted"],
                "gate_rejected": surface["gate_rejected"],
                "gate_reject_count": surface["gate_reject_count"],
                "gate_resync_applied": surface["gate_resync_applied"],
                "synthetic": surface["synthetic"],
                "synthetic_error": surface["synthetic_error"],
                "adj_valid": adj["valid"],
                "adj_ok": adj["ok"],
                "adj_rejected": adj["rejected"],
                "adj_tag": adjacency_tag(adj),
                "adj_counter_delta": adj["counter_delta"],
                "adj_expected": adj["expected"],
                "adj_delta_error": adj["delta_error"],
                "adj_reject_count": adj["reject_count"],
                "service_offset": svc["service_offset"],
                "service_class": svc["service_class"],
                "counter_delta": cdelta,
            }

        o1 = lane_data("ocxo1", o1_pub_actual, o1_pub_pred, _o1_pub_res)
        o2 = lane_data("ocxo2", o2_pub_actual, o2_pub_pred, _o2_pub_res)

        out.append({
            "pps_count": pps_count,
            "pps_actual": pps_actual,
            "pps_pred": pps_pred,
            "pps_delta": pps_delta,
            "phase": phase,
            "v_raw": v_surface["raw"],
            "v_raw_delta": v_raw_delta,
            "v_gate_pred": v_surface["gate_pred"],
            "v_gate_res": v_surface["gate_res"],
            "v_pub": v_surface["published"],
            "v_pub_delta": v_pub_delta,
            "v_gate_valid": v_surface["gate_valid"],
            "v_gate_accepted": v_surface["gate_accepted"],
            "v_gate_rejected": v_surface["gate_rejected"],
            "v_gate_reject_count": v_surface["gate_reject_count"],
            "v_cdelta": v_cdelta,
            "o1": o1,
            "o2": o2,
            "dwt_ppb": dwt_ppb_from_schema(root, frag),
        })

        prev_pps_count = pps_count
        if physical_pps_dwt is not None:
            prev_physical_pps_dwt = physical_pps_dwt
        if vclock_dwt is not None:
            prev_vclock_dwt = vclock_dwt
        if vclock_event_counter32 is not None:
            prev_vclock_counter32 = vclock_event_counter32
        if pps_actual is not None:
            prev_pps_interval = pps_actual

        if v_surface["raw"] is not None:
            prev_raw_interval["vclock"] = v_surface["raw"]
        if v_surface["published"] is not None:
            prev_pub_interval["vclock"] = v_surface["published"]
        for lane_name, data in (("ocxo1", o1), ("ocxo2", o2)):
            if data["raw"] is not None:
                prev_raw_interval[lane_name] = data["raw"]
            if data["pub"] is not None:
                prev_pub_interval[lane_name] = data["pub"]

    return out, gaps


def _residual_hit(*values: Optional[int]) -> bool:
    return any(v is not None and abs(v) >= EXCURSION_THRESHOLD_CYCLES for v in values)


def analyze(campaign: str, limit: int = 0, clock_filter: Optional[str] = None) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    clock_filter = normalize_clock_filter(clock_filter) if clock_filter else None
    collected, gaps = collect_rows(rows)
    if limit:
        collected = collected[:limit]

    clock_label = "ALL" if clock_filter is None else clock_filter
    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={clock_label})")
    print()
    print("Residual-separated raw cycle + adjacency audit")
    print("═══════════════════════════════════════════════")
    print("  *_rawΔ  = raw interval[n] - raw interval[n-1]  (old-style raw residual).")
    print("  *_gateR = raw interval - EMA/gate prediction   (gate disagreement).")
    print("  *_pubΔ  = published interval[n] - published interval[n-1].")
    print("  *_raw is the raw/evidence interval; *_pub is subscriber-facing published/smoothed.")
    print("  *_gPred is the EMA/gate prediction interval used for *_gateR.")
    print("  *_adj is OCXO counter-adjacency: OK, REJ, BAD, or ---.")
    print("  *_aΔ is counter_delta_ticks - expected_counter_delta_ticks; normal is 0.")
    print("  *_aN is cumulative adjacency reject count.")
    print("  *_svc is compact OCXO compare-service offset in 10 MHz ticks.")
    print()

    header, sep = _headers_for(clock_filter)
    print(header)
    print(sep)

    for row in collected:
        print(_row_line(row, clock_filter))

    print()
    print(f"Rows shown: {len(collected):,}")
    print(f"Gaps:       {gaps:,}")
    print()

    stats: Dict[str, Welford] = {
        "pps_actual": Welford(),
        "pps_delta": Welford(),
        "phase": Welford(),
        "vclock_raw_interval": Welford(),
        "vclock_raw_delta": Welford(),
        "vclock_gate_residual": Welford(),
        "vclock_published_interval": Welford(),
        "vclock_published_delta": Welford(),
        "vclock_counter32_delta": Welford(),
        "ocxo1_raw_interval": Welford(),
        "ocxo1_raw_delta": Welford(),
        "ocxo1_gate_residual": Welford(),
        "ocxo1_published_interval": Welford(),
        "ocxo1_published_delta": Welford(),
        "ocxo1_adjacency_delta": Welford(),
        "ocxo1_adjacency_reject_count": Welford(),
        "ocxo1_counter32_delta": Welford(),
        "ocxo1_service_offset": Welford(),
        "ocxo2_raw_interval": Welford(),
        "ocxo2_raw_delta": Welford(),
        "ocxo2_gate_residual": Welford(),
        "ocxo2_published_interval": Welford(),
        "ocxo2_published_delta": Welford(),
        "ocxo2_adjacency_delta": Welford(),
        "ocxo2_adjacency_reject_count": Welford(),
        "ocxo2_counter32_delta": Welford(),
        "ocxo2_service_offset": Welford(),
    }

    coverage = {
        "rows": 0,
        "vclock_gate": 0,
        "vclock_counter32_delta": 0,
        "ocxo1_gate": 0,
        "ocxo2_gate": 0,
        "ocxo1_adjacency": 0,
        "ocxo2_adjacency": 0,
        "ocxo1_adjacency_rejected": 0,
        "ocxo2_adjacency_rejected": 0,
        "ocxo1_counter32_delta": 0,
        "ocxo2_counter32_delta": 0,
        "ocxo1_service": 0,
        "ocxo2_service": 0,
    }

    service_class_counts = {"ocxo1": {}, "ocxo2": {}}
    excursions: List[Dict[str, Any]] = []
    vclock_excursions: List[Dict[str, Any]] = []

    for row in collected:
        coverage["rows"] += 1
        add_optional(stats["pps_actual"], row["pps_actual"])
        add_optional(stats["pps_delta"], row["pps_delta"])
        add_optional(stats["phase"], row["phase"])
        add_optional(stats["vclock_raw_interval"], row["v_raw"])
        add_optional(stats["vclock_raw_delta"], row["v_raw_delta"])
        add_optional(stats["vclock_gate_residual"], row["v_gate_res"])
        add_optional(stats["vclock_published_interval"], row["v_pub"])
        add_optional(stats["vclock_published_delta"], row["v_pub_delta"])
        add_optional(stats["vclock_counter32_delta"], row["v_cdelta"])
        if row["v_gate_valid"]:
            coverage["vclock_gate"] += 1
        if row["v_cdelta"] is not None:
            coverage["vclock_counter32_delta"] += 1

        for key, lane_name in (("o1", "ocxo1"), ("o2", "ocxo2")):
            d = row[key]
            add_optional(stats[f"{lane_name}_raw_interval"], d["raw"])
            add_optional(stats[f"{lane_name}_raw_delta"], d["raw_delta"])
            add_optional(stats[f"{lane_name}_gate_residual"], d["gate_res"])
            add_optional(stats[f"{lane_name}_published_interval"], d["pub"])
            add_optional(stats[f"{lane_name}_published_delta"], d["pub_delta"])
            add_optional(stats[f"{lane_name}_adjacency_delta"], d["adj_delta_error"])
            add_optional(stats[f"{lane_name}_adjacency_reject_count"], d["adj_reject_count"])
            add_optional(stats[f"{lane_name}_counter32_delta"], d["counter_delta"])
            add_optional(stats[f"{lane_name}_service_offset"], d["service_offset"])

            if d["gate_valid"]:
                coverage[f"{lane_name}_gate"] += 1
            if d["adj_valid"]:
                coverage[f"{lane_name}_adjacency"] += 1
            if d["adj_rejected"]:
                coverage[f"{lane_name}_adjacency_rejected"] += 1
            if d["counter_delta"] is not None:
                coverage[f"{lane_name}_counter32_delta"] += 1
            if d["service_offset"] is not None:
                coverage[f"{lane_name}_service"] += 1
            cls = service_class_short(d["service_class"])
            service_class_counts[lane_name][cls] = service_class_counts[lane_name].get(cls, 0) + 1

        if _residual_hit(row["v_raw_delta"], row["v_gate_res"], row["v_pub_delta"]):
            vclock_excursions.append(row)

        for key in ("o1", "o2"):
            d = row[key]
            if _residual_hit(d["raw_delta"], d["gate_res"], d["pub_delta"]):
                excursions.append({"row": row, "lane": key, "data": d})

    print("Schema coverage")
    print("═══════════════")
    print(f"  rows                         = {coverage['rows']:,}")
    print(f"  VCLOCK gate rows             = {coverage['vclock_gate']:,}")
    print(f"  VCLOCK counter32 delta rows  = {coverage['vclock_counter32_delta']:,}")
    print(f"  OCXO1 gate rows              = {coverage['ocxo1_gate']:,}")
    print(f"  OCXO2 gate rows              = {coverage['ocxo2_gate']:,}")
    print(f"  OCXO1 adjacency rows         = {coverage['ocxo1_adjacency']:,}")
    print(f"  OCXO2 adjacency rows         = {coverage['ocxo2_adjacency']:,}")
    print(f"  OCXO1 adjacency rejects      = {coverage['ocxo1_adjacency_rejected']:,}")
    print(f"  OCXO2 adjacency rejects      = {coverage['ocxo2_adjacency_rejected']:,}")
    print(f"  OCXO1 counter32 delta rows   = {coverage['ocxo1_counter32_delta']:,}")
    print(f"  OCXO2 counter32 delta rows   = {coverage['ocxo2_counter32_delta']:,}")
    print(f"  OCXO1 service rows           = {coverage['ocxo1_service']:,}")
    print(f"  OCXO2 service rows           = {coverage['ocxo2_service']:,}")
    print(f"  OCXO1 service classes        = {service_class_counts['ocxo1']}")
    print(f"  OCXO2 service classes        = {service_class_counts['ocxo2']}")
    print()

    print("Summary")
    print("═══════")
    _print_welford("PPS actual cycles", stats["pps_actual"])
    _print_welford("PPS first-difference residual", stats["pps_delta"])
    _print_welford("PPS→VCLOCK phase offset", stats["phase"])

    if clock_filter is None or clock_filter == "VCLOCK":
        _print_welford("VCLOCK raw interval", stats["vclock_raw_interval"])
        _print_welford("VCLOCK raw first-difference residual", stats["vclock_raw_delta"])
        _print_welford("VCLOCK EMA gate residual", stats["vclock_gate_residual"])
        _print_welford("VCLOCK published interval", stats["vclock_published_interval"])
        _print_welford("VCLOCK published first-difference residual", stats["vclock_published_delta"])
        _print_welford("VCLOCK counter32 delta", stats["vclock_counter32_delta"], unit="ticks")

    for lane_name, label in (("ocxo1", "OCXO1"), ("ocxo2", "OCXO2")):
        if clock_filter is not None and clock_filter != label:
            continue
        _print_welford(f"{label} raw interval", stats[f"{lane_name}_raw_interval"])
        _print_welford(f"{label} raw first-difference residual", stats[f"{lane_name}_raw_delta"])
        _print_welford(f"{label} EMA gate residual", stats[f"{lane_name}_gate_residual"])
        _print_welford(f"{label} published interval", stats[f"{lane_name}_published_interval"])
        _print_welford(f"{label} published first-difference residual", stats[f"{lane_name}_published_delta"])
        _print_welford(f"{label} adjacency delta", stats[f"{lane_name}_adjacency_delta"], unit="ticks")
        _print_welford(f"{label} adjacency reject count", stats[f"{lane_name}_adjacency_reject_count"], unit="rejects")
        _print_welford(f"{label} counter32 delta", stats[f"{lane_name}_counter32_delta"], unit="ticks")
        _print_welford(f"{label} service offset", stats[f"{lane_name}_service_offset"], unit="ticks")
    print()

    if clock_filter is None or clock_filter == "VCLOCK":
        print(f"VCLOCK residual excursions (|rawΔ| or |gateR| or |pubΔ| ≥ {EXCURSION_THRESHOLD_CYCLES} cycles)")
        print("════════════════════════════════════════════════════════════════════════════")
        if not vclock_excursions:
            print("  none")
        else:
            h = (
                f"{'pps':>6s} {'rawΔ':>8s} {'gateR':>8s} {'pubΔ':>8s} "
                f"{'raw':>13s} {'gPred':>13s} {'pub':>13s} "
                f"{'v_cΔ':>11s} {'phase':>5s} {'p_Δ':>8s}"
            )
            print(h)
            print(f"{'─'*6} {'─'*8} {'─'*8} {'─'*8} {'─'*13} {'─'*13} {'─'*13} {'─'*11} {'─'*5} {'─'*8}")
            for row in vclock_excursions[:60]:
                print(
                    f"{row['pps_count']:>6d} "
                    f"{_fmt_int(row['v_raw_delta'], 8, signed=True)} "
                    f"{_fmt_int(row['v_gate_res'], 8, signed=True)} "
                    f"{_fmt_int(row['v_pub_delta'], 8, signed=True)} "
                    f"{_fmt_int(row['v_raw'], 13)} "
                    f"{_fmt_int(row['v_gate_pred'], 13)} "
                    f"{_fmt_int(row['v_pub'], 13)} "
                    f"{_fmt_int(row['v_cdelta'], 11)} "
                    f"{_fmt_int(row['phase'], 5)} "
                    f"{_fmt_int(row['pps_delta'], 8, signed=True)}"
                )
            if len(vclock_excursions) > 60:
                print(f"  ... {len(vclock_excursions) - 60:,} more VCLOCK excursion rows omitted")
        print()

    if clock_filter is None or clock_filter in ("OCXO1", "OCXO2"):
        print(f"OCXO residual excursions (|rawΔ| or |gateR| or |pubΔ| ≥ {EXCURSION_THRESHOLD_CYCLES} cycles)")
        print("════════════════════════════════════════════════════════════════════════")
        filtered = [
            ex for ex in excursions
            if clock_filter is None or
               (clock_filter == "OCXO1" and ex["lane"] == "o1") or
               (clock_filter == "OCXO2" and ex["lane"] == "o2")
        ]
        if not filtered:
            print("  none")
        else:
            h = (
                f"{'pps':>6s} {'lane':>5s} {'rawΔ':>8s} {'gateR':>8s} {'pubΔ':>8s} "
                f"{'raw':>13s} {'gPred':>13s} {'pub':>13s} {'adj':>5s} {'aΔ':>9s} {'aN':>6s} "
                f"{'cΔ':>11s} {'svc':>7s} {'p_Δ':>8s} {'v_rawΔ':>8s} {'v_gateR':>8s} {'v_pubΔ':>8s}"
            )
            print(h)
            print(f"{'─'*6} {'─'*5} {'─'*8} {'─'*8} {'─'*8} {'─'*13} {'─'*13} {'─'*13} {'─'*5} {'─'*9} {'─'*6} {'─'*11} {'─'*7} {'─'*8} {'─'*8} {'─'*8} {'─'*8}")
            for ex in filtered[:60]:
                row = ex["row"]
                d = ex["data"]
                print(
                    f"{row['pps_count']:>6d} {ex['lane']:>5s} "
                    f"{_fmt_int(d['raw_delta'], 8, signed=True)} "
                    f"{_fmt_int(d['gate_res'], 8, signed=True)} "
                    f"{_fmt_int(d['pub_delta'], 8, signed=True)} "
                    f"{_fmt_int(d['raw'], 13)} "
                    f"{_fmt_int(d['gate_pred'], 13)} "
                    f"{_fmt_int(d['pub'], 13)} "
                    f"{_fmt_str(d['adj_tag'], 5)} "
                    f"{_fmt_int(d['adj_delta_error'], 9, signed=True)} "
                    f"{_fmt_int(d['adj_reject_count'], 6)} "
                    f"{_fmt_int(d['counter_delta'], 11)} "
                    f"{_fmt_int(d['service_offset'], 7, signed=True)} "
                    f"{_fmt_int(row['pps_delta'], 8, signed=True)} "
                    f"{_fmt_int(row['v_raw_delta'], 8, signed=True)} "
                    f"{_fmt_int(row['v_gate_res'], 8, signed=True)} "
                    f"{_fmt_int(row['v_pub_delta'], 8, signed=True)}"
                )
            if len(filtered) > 60:
                print(f"  ... {len(filtered) - 60:,} more excursion rows omitted")
        print()

    print("Notes")
    print("═════")
    print("  • *_raw is the raw/evidence interval from dwt_interval_gate.observed_cycles when present.")
    print("  • *_rawΔ = raw[n] - raw[n-1]. This is the old-style raw first-difference residual.")
    print("  • *_gPred is process_interrupt's EMA/gate prediction interval.")
    print("  • *_gateR = raw - gPred. This is predictor disagreement, not first-difference jitter.")
    print("  • *_pub is the subscriber-facing published/smoothed interval.")
    print("  • *_pubΔ = pub[n] - pub[n-1]. This is the old-style published residual.")
    print("  • p_* columns are the physical PPS rail; PPS has no separate gate/published rail here.")
    print("  • *_adj is OCXO adjacency status: OK=counter lineage intact, REJ=guard rejected")
    print("    the event, BAD=valid adjacency surface but not OK/rejected, ---=not applicable/missing.")
    print("  • *_aΔ is counter_delta_ticks - expected_counter_delta_ticks. Normal is 0.")
    print("  • *_aN is cumulative adjacency reject count.")
    print("  • *_cΔ is Alpha/process_interrupt counter32 delta since the previous event.")
    print("  • *_svc is compact OCXO compare-service offset in 10 MHz ticks.")
    print("  • Use a clock filter for narrower one-line rows, e.g. raw_cycles Symmetry2 VCLOCK.")
    print()


def parse_args(argv: List[str]) -> Tuple[str, int, Optional[str]]:
    if len(argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit] [clock]")
        print("       raw_cycles <campaign_name> --clock OCXO2 [limit]")
        raise SystemExit(1)

    campaign = argv[1]
    limit = 0
    clock: Optional[str] = None

    i = 2
    while i < len(argv):
        arg = argv[i]
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
        if arg.upper() in CLOCKS or arg.upper() in ("V", "VC", "O1", "O2"):
            clock = arg
            i += 1
            continue
        try:
            limit = int(arg)
        except ValueError as exc:
            raise SystemExit(f"unknown argument '{arg}'") from exc
        i += 1

    return campaign, limit, normalize_clock_filter(clock) if clock else None


def main() -> None:
    campaign, limit, clock = parse_args(sys.argv)
    analyze(campaign, limit, clock)


if __name__ == "__main__":
    main()
