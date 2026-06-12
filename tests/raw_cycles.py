"""
ZPNet Raw Cycles — EMA vs PPS-Yardstick side-by-side cycle audit.

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

    *_raw      is the raw/evidence interval from TIMEBASE_FORENSICS
               dwt_interval_gate.observed_cycles when available.
    *_rawΔ     is the raw first-difference residual: raw[n] - raw[n-1].
    *_ema      is the subscriber-facing published interval.  Today this is
               EMA-authored (formerly printed as *_pub); the rename tells
               the truth about its provenance.
    *_emaΔ     is ema[n] - ema[n-1].

    *_inferred is the PPS-Yardstick inferred interval (heir apparent to the
               EMA): prior chain interval scaled by the physical PPS-to-PPS
               yardstick ratio, carried in Q16, validated by raw.
    *_imo      is inferred - observed for this second: the yardstick
               scorecard.  Sub-lattice |imo| means the inference is tracking
               truth tighter than the 4-cycle QTimer quantization floor.
    *_walk     is the free-running yardstick chain endpoint minus the
               observed endpoint: cumulative chain-walk audit.
    *_y        is the yardstick row tag: OK, EXC (gate excursion), STAL
               (stale PPS yardstick, interval held), SEED (chain reseeded
               this row), or --- (rail not valid on this row).

    *_cΔ       is counter32 delta since the previous event (lineage proof).
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


def yardstick_diag(root: Dict[str, Any],
                   frag: Dict[str, Any],
                   forensic: Dict[str, Any],
                   lane: str) -> Dict[str, Any]:
    """Extract the PPS-Yardstick inference object for one OCXO lane.

    Source of truth is TIMEBASE_FORENSICS lane forensics dwt_yardstick.
    Flat-key fallbacks follow the house extraction convention so older or
    transitional rows still render with --- rather than crashing.
    """
    f = lane_forensics_obj(root, frag, forensic, lane)
    y = f.get("dwt_yardstick") if isinstance(f.get("dwt_yardstick"), dict) else {}
    prefix = f"{lane}_forensics_dwt_yardstick_"

    return {
        "valid": _first_bool(y.get("valid"), f.get("dwt_yardstick_valid"),
                             frag.get(prefix + "valid"),
                             root.get(prefix + "valid")),
        "stale": _first_bool(y.get("stale"), f.get("dwt_yardstick_stale"),
                             frag.get(prefix + "stale"),
                             root.get(prefix + "stale")),
        "seeded": _first_bool(y.get("seeded"), f.get("dwt_yardstick_seeded"),
                              frag.get(prefix + "seeded"),
                              root.get(prefix + "seeded")),
        "excursion": _first_bool(y.get("excursion"), f.get("dwt_yardstick_excursion"),
                                 frag.get(prefix + "excursion"),
                                 root.get(prefix + "excursion")),
        "pps_sequence": _first_int(y.get("pps_sequence"),
                                   f.get("dwt_yardstick_pps_sequence")),
        "pps_seq_delta": _first_int(y.get("pps_seq_delta"),
                                    f.get("dwt_yardstick_pps_seq_delta")),
        "g_now": _first_int(y.get("g_now_cycles"),
                            f.get("dwt_yardstick_g_now_cycles")),
        "g_prev": _first_int(y.get("g_prev_cycles"),
                             f.get("dwt_yardstick_g_prev_cycles")),
        "inferred": _first_int(y.get("inferred_interval_cycles"),
                               f.get("dwt_yardstick_inferred_interval_cycles"),
                               frag.get(prefix + "inferred_interval_cycles"),
                               root.get(prefix + "inferred_interval_cycles")),
        "observed": _first_int(y.get("observed_interval_cycles"),
                               f.get("dwt_yardstick_observed_interval_cycles")),
        "imo": _first_int(y.get("inferred_minus_observed_cycles"),
                          f.get("dwt_yardstick_inferred_minus_observed_cycles"),
                          frag.get(prefix + "inferred_minus_observed_cycles"),
                          root.get(prefix + "inferred_minus_observed_cycles")),
        "endpoint_dwt": _first_int(y.get("inferred_endpoint_dwt"),
                                   f.get("dwt_yardstick_inferred_endpoint_dwt")),
        "endpoint_frac_q16": _first_int(y.get("inferred_endpoint_frac_q16"),
                                        f.get("dwt_yardstick_inferred_endpoint_frac_q16")),
        "walk": _first_int(y.get("endpoint_minus_observed_cycles"),
                           f.get("dwt_yardstick_endpoint_minus_observed_cycles"),
                           frag.get(prefix + "endpoint_minus_observed_cycles"),
                           root.get(prefix + "endpoint_minus_observed_cycles")),
        "threshold": _first_int(y.get("gate_threshold_cycles"),
                                f.get("dwt_yardstick_gate_threshold_cycles")),
        "agree_count": _first_int(y.get("gate_agree_count"),
                                  f.get("dwt_yardstick_gate_agree_count")),
        "excursion_count": _first_int(y.get("gate_excursion_count"),
                                      f.get("dwt_yardstick_gate_excursion_count")),
    }


def yardstick_tag(y: Dict[str, Any]) -> str:
    if not y.get("valid"):
        return "---"
    if y.get("seeded"):
        return "SEED"
    if y.get("excursion"):
        return "EXC"
    if y.get("stale"):
        return "STAL"
    return "OK"



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
    """Return the raw and EMA-published interval surfaces for one lane.

    The key doctrine is that these are two different questions:

      rawΔ : did the raw evidence interval change from the prior raw interval?
      emaΔ : did the EMA-published interval change from the prior one?

    (The yardstick surface is extracted separately by yardstick_diag; its
    scorecard *_imo is firmware-computed, not derived here.)

    Older rows may not expose dwt_interval_gate. In that case raw falls back
    to the published/static surface so legacy reports still render.
    """
    gate_valid = bool(gate.get("valid"))

    raw = gate.get("observed_cycles") if gate_valid else None
    if raw is None:
        raw = pub_actual

    # Stage 2 authority note: the firmware static prediction surface
    # (pub_actual) now carries the yardstick-published interval, so the EMA
    # column must source from the EMA rail itself: dwt_interval_gate
    # effective_cycles remains EMA-authored in every firmware version.
    ema = gate.get("effective_cycles") if gate_valid else None
    if ema is None:
        ema = pub_actual
    if ema is None and pub_pred is not None and pub_res is not None:
        ema = pub_pred + pub_res

    return {
        "raw": raw,
        "ema": ema,
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
            f"{'v_ema':>13s}",
            f"{'v_emaΔ':>8s}",
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
            f"{prefix + '_ema':>13s}",
            f"{prefix + '_emaΔ':>8s}",
            f"{prefix + '_inferred':>13s}",
            f"{prefix + '_imo':>7s}",
            f"{prefix + '_walk':>8s}",
            f"{prefix + '_y':>5s}",
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
            _fmt_int(row["v_ema"], 13),
            _fmt_int(row["v_ema_delta"], 8, signed=True),
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
            _fmt_int(data["ema"], 13),
            _fmt_int(data["ema_delta"], 8, signed=True),
            _fmt_int(data["inferred"], 13),
            _fmt_int(data["imo"], 7, signed=True),
            _fmt_int(data["walk"], 8, signed=True),
            _fmt_str(data["y_tag"], 5),
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
    prev_ema_interval: Dict[str, Optional[int]] = {
        "vclock": None,
        "ocxo1": None,
        "ocxo2": None,
    }

    def reset_deltas_after_gap() -> None:
        nonlocal prev_physical_pps_dwt, prev_vclock_dwt, prev_vclock_counter32
        nonlocal prev_pps_interval, prev_raw_interval, prev_ema_interval
        prev_physical_pps_dwt = None
        prev_vclock_dwt = None
        prev_vclock_counter32 = None
        prev_pps_interval = None
        prev_raw_interval = {"vclock": None, "ocxo1": None, "ocxo2": None}
        prev_ema_interval = {"vclock": None, "ocxo1": None, "ocxo2": None}

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
        v_ema_delta = _interval_delta(v_surface["ema"], prev_ema_interval["vclock"])

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
            yard = yardstick_diag(root, frag, forensic, lane_name)
            cdelta = lane_counter_delta(root, frag, forensic, lane_name)
            if cdelta is None:
                cdelta = svc.get("last_counter_delta")

            surface = _lane_cycle_surfaces(gate, pub_actual, pub_pred, pub_res)
            raw_delta = _interval_delta(surface["raw"], prev_raw_interval[lane_name])
            ema_delta = _interval_delta(surface["ema"], prev_ema_interval[lane_name])

            # Firmware-computed scorecards; fall back to local arithmetic only
            # when the firmware surface is present but a field is missing.
            imo = yard.get("imo")
            if imo is None and yard.get("inferred") is not None and surface["raw"] is not None:
                imo = yard["inferred"] - surface["raw"]

            # Derived side-by-side error: EMA published interval minus raw
            # evidence interval, the direct competitor of imo.
            emo = (
                surface["ema"] - surface["raw"]
                if surface["ema"] is not None and surface["raw"] is not None
                else None
            )

            return {
                "raw": surface["raw"],
                "raw_delta": raw_delta,
                "ema": surface["ema"],
                "ema_delta": ema_delta,
                "emo": emo,
                "inferred": yard.get("inferred"),
                "imo": imo,
                "walk": yard.get("walk"),
                "y_tag": yardstick_tag(yard),
                "y_valid": yard.get("valid"),
                "y_stale": yard.get("stale"),
                "y_seeded": yard.get("seeded"),
                "y_excursion": yard.get("excursion"),
                "y_agree_count": yard.get("agree_count"),
                "y_excursion_count": yard.get("excursion_count"),
                "y_threshold": yard.get("threshold"),
                "y_pps_seq_delta": yard.get("pps_seq_delta"),
                "gate_valid": surface["gate_valid"],
                "gate_accepted": surface["gate_accepted"],
                "gate_rejected": surface["gate_rejected"],
                "gate_reject_count": surface["gate_reject_count"],
                "gate_resync_applied": surface["gate_resync_applied"],
                "synthetic": surface["synthetic"],
                "synthetic_error": surface["synthetic_error"],
                "adj_valid": adj["valid"],
                "adj_rejected": adj["rejected"],
                "adj_reject_count": adj["reject_count"],
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
            "v_ema": v_surface["ema"],
            "v_ema_delta": v_ema_delta,
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
        if v_surface["ema"] is not None:
            prev_ema_interval["vclock"] = v_surface["ema"]
        for lane_name, data in (("ocxo1", o1), ("ocxo2", o2)):
            if data["raw"] is not None:
                prev_raw_interval[lane_name] = data["raw"]
            if data["ema"] is not None:
                prev_ema_interval[lane_name] = data["ema"]

    return out, gaps

def _residual_hit(*values: Optional[int]) -> bool:
    return any(v is not None and abs(v) >= EXCURSION_THRESHOLD_CYCLES for v in values)


def _walk_slope_cycles_per_second(samples: List[Tuple[int, int]]) -> Optional[float]:
    """Least-squares slope of chain walk vs pps_count, in cycles/second.

    samples are (pps_count, walk_cycles) pairs from rows where the yardstick
    rail was valid.  The slope is the campaign-scale chain-walk rate: the
    decisive Stage 2/3 anchor-policy datum.
    """
    n = len(samples)
    if n < 2:
        return None
    mean_x = sum(x for x, _ in samples) / n
    mean_y = sum(y for _, y in samples) / n
    sxx = sum((x - mean_x) ** 2 for x, _ in samples)
    if sxx == 0:
        return None
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in samples)
    return sxy / sxx


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
    print("EMA vs PPS-Yardstick side-by-side cycle audit")
    print("═════════════════════════════════════════════")
    print("  *_rawΔ     = raw interval[n] - raw interval[n-1].")
    print("  *_ema      = subscriber-facing published interval (EMA-authored today).")
    print("  *_emaΔ     = ema[n] - ema[n-1].")
    print("  *_inferred = PPS-Yardstick inferred interval (heir apparent).")
    print("  *_imo      = inferred - observed: the yardstick per-second scorecard.")
    print("  *_walk     = free-running yardstick chain endpoint - observed endpoint.")
    print("  *_y        = yardstick tag: OK / EXC / STAL / SEED / ---.")
    print("  *_cΔ       = counter32 delta since previous event (lineage proof).")
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
        "vclock_ema_interval": Welford(),
        "vclock_ema_delta": Welford(),
        "vclock_counter32_delta": Welford(),
    }
    for lane_name in ("ocxo1", "ocxo2"):
        stats[f"{lane_name}_raw_interval"] = Welford()
        stats[f"{lane_name}_raw_delta"] = Welford()
        stats[f"{lane_name}_ema_interval"] = Welford()
        stats[f"{lane_name}_ema_delta"] = Welford()
        stats[f"{lane_name}_inferred_interval"] = Welford()
        stats[f"{lane_name}_imo"] = Welford()
        stats[f"{lane_name}_emo"] = Welford()
        stats[f"{lane_name}_abs_imo"] = Welford()
        stats[f"{lane_name}_abs_emo"] = Welford()
        stats[f"{lane_name}_walk"] = Welford()
        stats[f"{lane_name}_counter32_delta"] = Welford()

    coverage = {
        "rows": 0,
        "vclock_gate": 0,
        "ocxo1_gate": 0,
        "ocxo2_gate": 0,
        "ocxo1_yardstick": 0,
        "ocxo2_yardstick": 0,
        "ocxo1_yardstick_stale": 0,
        "ocxo2_yardstick_stale": 0,
        "ocxo1_yardstick_seeded": 0,
        "ocxo2_yardstick_seeded": 0,
        "ocxo1_yardstick_excursions": 0,
        "ocxo2_yardstick_excursions": 0,
        "ocxo1_adjacency_rejected": 0,
        "ocxo2_adjacency_rejected": 0,
    }
    last_counts = {
        "ocxo1": {"agree": None, "excursion": None, "threshold": None},
        "ocxo2": {"agree": None, "excursion": None, "threshold": None},
    }

    service_class_counts = {"ocxo1": {}, "ocxo2": {}}
    excursions: List[Dict[str, Any]] = []
    vclock_excursions: List[Dict[str, Any]] = []
    walk_samples = {"ocxo1": [], "ocxo2": []}

    for row in collected:
        coverage["rows"] += 1
        add_optional(stats["pps_actual"], row["pps_actual"])
        add_optional(stats["pps_delta"], row["pps_delta"])
        add_optional(stats["phase"], row["phase"])
        add_optional(stats["vclock_raw_interval"], row["v_raw"])
        add_optional(stats["vclock_raw_delta"], row["v_raw_delta"])
        add_optional(stats["vclock_ema_interval"], row["v_ema"])
        add_optional(stats["vclock_ema_delta"], row["v_ema_delta"])
        add_optional(stats["vclock_counter32_delta"], row["v_cdelta"])
        if row["v_gate_valid"]:
            coverage["vclock_gate"] += 1

        for key, lane_name in (("o1", "ocxo1"), ("o2", "ocxo2")):
            d = row[key]
            add_optional(stats[f"{lane_name}_raw_interval"], d["raw"])
            add_optional(stats[f"{lane_name}_raw_delta"], d["raw_delta"])
            add_optional(stats[f"{lane_name}_ema_interval"], d["ema"])
            add_optional(stats[f"{lane_name}_ema_delta"], d["ema_delta"])
            add_optional(stats[f"{lane_name}_counter32_delta"], d["counter_delta"])

            if d["gate_valid"]:
                coverage[f"{lane_name}_gate"] += 1
            if d["adj_rejected"]:
                coverage[f"{lane_name}_adjacency_rejected"] += 1
            cls = service_class_short(d["service_class"])
            service_class_counts[lane_name][cls] = service_class_counts[lane_name].get(cls, 0) + 1

            if d["y_valid"]:
                coverage[f"{lane_name}_yardstick"] += 1
                add_optional(stats[f"{lane_name}_inferred_interval"], d["inferred"])
                add_optional(stats[f"{lane_name}_walk"], d["walk"])
                if d["walk"] is not None:
                    walk_samples[lane_name].append((row["pps_count"], d["walk"]))
                if d["y_stale"]:
                    coverage[f"{lane_name}_yardstick_stale"] += 1
                if d["y_seeded"]:
                    coverage[f"{lane_name}_yardstick_seeded"] += 1
                if d["y_excursion"]:
                    coverage[f"{lane_name}_yardstick_excursions"] += 1
                else:
                    # Head-to-head scorecards on agree seconds only: excursion
                    # seconds are observed-side corruption by definition, and
                    # both rails are judged against the same clean evidence.
                    if not d["y_seeded"]:
                        add_optional(stats[f"{lane_name}_imo"], d["imo"])
                        add_optional(stats[f"{lane_name}_emo"], d["emo"])
                        if d["imo"] is not None:
                            stats[f"{lane_name}_abs_imo"].update(abs(float(d["imo"])))
                        if d["emo"] is not None:
                            stats[f"{lane_name}_abs_emo"].update(abs(float(d["emo"])))
                if d["y_agree_count"] is not None:
                    last_counts[lane_name]["agree"] = d["y_agree_count"]
                if d["y_excursion_count"] is not None:
                    last_counts[lane_name]["excursion"] = d["y_excursion_count"]
                if d["y_threshold"] is not None:
                    last_counts[lane_name]["threshold"] = d["y_threshold"]

        if _residual_hit(row["v_raw_delta"], row["v_ema_delta"]):
            vclock_excursions.append(row)

        for key in ("o1", "o2"):
            d = row[key]
            if _residual_hit(d["raw_delta"], d["ema_delta"]) or d["y_excursion"]:
                excursions.append({"row": row, "lane": key, "data": d})

    print("Schema coverage")
    print("═══════════════")
    print(f"  rows                          = {coverage['rows']:,}")
    print(f"  VCLOCK gate rows              = {coverage['vclock_gate']:,}")
    print(f"  OCXO1 gate rows               = {coverage['ocxo1_gate']:,}")
    print(f"  OCXO2 gate rows               = {coverage['ocxo2_gate']:,}")
    print(f"  OCXO1 yardstick rows          = {coverage['ocxo1_yardstick']:,}")
    print(f"  OCXO2 yardstick rows          = {coverage['ocxo2_yardstick']:,}")
    print(f"  OCXO1 yardstick stale/seeded  = {coverage['ocxo1_yardstick_stale']:,} / {coverage['ocxo1_yardstick_seeded']:,}")
    print(f"  OCXO2 yardstick stale/seeded  = {coverage['ocxo2_yardstick_stale']:,} / {coverage['ocxo2_yardstick_seeded']:,}")
    print(f"  OCXO1 yardstick excursions    = {coverage['ocxo1_yardstick_excursions']:,}")
    print(f"  OCXO2 yardstick excursions    = {coverage['ocxo2_yardstick_excursions']:,}")
    print(f"  OCXO1 adjacency rejects       = {coverage['ocxo1_adjacency_rejected']:,}")
    print(f"  OCXO2 adjacency rejects       = {coverage['ocxo2_adjacency_rejected']:,}")
    print(f"  OCXO1 service classes         = {service_class_counts['ocxo1']}")
    print(f"  OCXO2 service classes         = {service_class_counts['ocxo2']}")
    print()

    print("Summary")
    print("═══════")
    _print_welford("PPS actual cycles", stats["pps_actual"])
    _print_welford("PPS first-difference residual", stats["pps_delta"])
    _print_welford("PPS→VCLOCK phase offset", stats["phase"])

    if clock_filter is None or clock_filter == "VCLOCK":
        _print_welford("VCLOCK raw interval", stats["vclock_raw_interval"])
        _print_welford("VCLOCK raw first-difference residual", stats["vclock_raw_delta"])
        _print_welford("VCLOCK EMA published interval", stats["vclock_ema_interval"])
        _print_welford("VCLOCK EMA published first-difference", stats["vclock_ema_delta"])
        _print_welford("VCLOCK counter32 delta", stats["vclock_counter32_delta"], unit="ticks")

    for lane_name, label in (("ocxo1", "OCXO1"), ("ocxo2", "OCXO2")):
        if clock_filter is not None and clock_filter != label:
            continue
        _print_welford(f"{label} raw interval", stats[f"{lane_name}_raw_interval"])
        _print_welford(f"{label} raw first-difference residual", stats[f"{lane_name}_raw_delta"])
        _print_welford(f"{label} EMA published interval", stats[f"{lane_name}_ema_interval"])
        _print_welford(f"{label} EMA published first-difference", stats[f"{lane_name}_ema_delta"])
        _print_welford(f"{label} yardstick inferred interval", stats[f"{lane_name}_inferred_interval"])
        _print_welford(f"{label} counter32 delta", stats[f"{lane_name}_counter32_delta"], unit="ticks")
    print()

    if clock_filter is None or clock_filter in ("OCXO1", "OCXO2"):
        print("Head-to-head: EMA vs PPS-Yardstick (agree seconds, same raw evidence)")
        print("══════════════════════════════════════════════════════════════════════")
        print("  emo = ema published - raw observed   (EMA per-second error)")
        print("  imo = inferred - raw observed        (yardstick per-second error)")
        print()
        for lane_name, label in (("ocxo1", "OCXO1"), ("ocxo2", "OCXO2")):
            if clock_filter is not None and clock_filter != label:
                continue
            _print_welford(f"{label} EMA error (emo)", stats[f"{lane_name}_emo"])
            _print_welford(f"{label} EMA |error|", stats[f"{lane_name}_abs_emo"])
            _print_welford(f"{label} yardstick error (imo)", stats[f"{lane_name}_imo"])
            _print_welford(f"{label} yardstick |error|", stats[f"{lane_name}_abs_imo"])
            _print_welford(f"{label} chain walk (endpoint - observed)", stats[f"{lane_name}_walk"])
            slope = _walk_slope_cycles_per_second(walk_samples[lane_name])
            counts = last_counts[lane_name]
            slope_text = "---" if slope is None else f"{slope:+.4f} cycles/s"
            print(f"  {label} chain walk slope (least squares)        {slope_text}")
            print(
                f"  {label} cumulative gate counts                   "
                f"agree={counts['agree']}  excursion={counts['excursion']}  "
                f"threshold={counts['threshold']}"
            )
            print()

    if clock_filter is None or clock_filter == "VCLOCK":
        print(f"VCLOCK residual excursions (|rawΔ| or |emaΔ| ≥ {EXCURSION_THRESHOLD_CYCLES} cycles)")
        print("════════════════════════════════════════════════════════════")
        if not vclock_excursions:
            print("  none")
        else:
            h = (
                f"{'pps':>6s} {'rawΔ':>8s} {'emaΔ':>8s} "
                f"{'raw':>13s} {'ema':>13s} "
                f"{'v_cΔ':>11s} {'phase':>5s} {'p_Δ':>8s}"
            )
            print(h)
            print(f"{'─'*6} {'─'*8} {'─'*8} {'─'*13} {'─'*13} {'─'*11} {'─'*5} {'─'*8}")
            for row in vclock_excursions[:60]:
                print(
                    f"{row['pps_count']:>6d} "
                    f"{_fmt_int(row['v_raw_delta'], 8, signed=True)} "
                    f"{_fmt_int(row['v_ema_delta'], 8, signed=True)} "
                    f"{_fmt_int(row['v_raw'], 13)} "
                    f"{_fmt_int(row['v_ema'], 13)} "
                    f"{_fmt_int(row['v_cdelta'], 11)} "
                    f"{_fmt_int(row['phase'], 5)} "
                    f"{_fmt_int(row['pps_delta'], 8, signed=True)}"
                )
            if len(vclock_excursions) > 60:
                print(f"  ... {len(vclock_excursions) - 60:,} more VCLOCK excursion rows omitted")
        print()

    if clock_filter is None or clock_filter in ("OCXO1", "OCXO2"):
        print(f"OCXO residual excursions (|rawΔ| or |emaΔ| ≥ {EXCURSION_THRESHOLD_CYCLES} cycles, or yardstick EXC)")
        print("══════════════════════════════════════════════════════════════════════════════")
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
                f"{'pps':>6s} {'lane':>5s} {'rawΔ':>8s} {'emaΔ':>8s} {'imo':>7s} {'walk':>8s} "
                f"{'raw':>13s} {'ema':>13s} {'inferred':>13s} {'y':>5s} "
                f"{'cΔ':>11s} {'p_Δ':>8s} {'v_rawΔ':>8s}"
            )
            print(h)
            print(f"{'─'*6} {'─'*5} {'─'*8} {'─'*8} {'─'*7} {'─'*8} {'─'*13} {'─'*13} {'─'*13} {'─'*5} {'─'*11} {'─'*8} {'─'*8}")
            for ex in filtered[:60]:
                row = ex["row"]
                d = ex["data"]
                print(
                    f"{row['pps_count']:>6d} {ex['lane']:>5s} "
                    f"{_fmt_int(d['raw_delta'], 8, signed=True)} "
                    f"{_fmt_int(d['ema_delta'], 8, signed=True)} "
                    f"{_fmt_int(d['imo'], 7, signed=True)} "
                    f"{_fmt_int(d['walk'], 8, signed=True)} "
                    f"{_fmt_int(d['raw'], 13)} "
                    f"{_fmt_int(d['ema'], 13)} "
                    f"{_fmt_int(d['inferred'], 13)} "
                    f"{_fmt_str(d['y_tag'], 5)} "
                    f"{_fmt_int(d['counter_delta'], 11)} "
                    f"{_fmt_int(row['pps_delta'], 8, signed=True)} "
                    f"{_fmt_int(row['v_raw_delta'], 8, signed=True)}"
                )
            if len(filtered) > 60:
                print(f"  ... {len(filtered) - 60:,} more excursion rows omitted")
        print()

    print("Notes")
    print("═════")
    print("  • *_raw is the raw/evidence interval from dwt_interval_gate.observed_cycles when present.")
    print("  • *_rawΔ = raw[n] - raw[n-1]. This is the old-style raw first-difference residual.")
    print("  • *_ema is the subscriber-facing published interval; today it is EMA-authored")
    print("    (this column was previously printed as *_pub).")
    print("  • *_emaΔ = ema[n] - ema[n-1].")
    print("  • *_inferred is the PPS-Yardstick inferred interval: prior chain interval scaled")
    print("    by the physical PPS-to-PPS yardstick ratio, carried in Q16 fixed point.")
    print("  • *_imo = inferred - observed (firmware-computed). Sub-lattice |imo| means the")
    print("    inference tracks truth tighter than the 4-cycle QTimer quantization floor.")
    print("  • *_walk is the free-running chain endpoint minus observed endpoint: the")
    print("    cumulative chain-walk audit that decides the Stage 2/3 anchor policy.")
    print("  • *_y tags: OK, EXC (gate excursion: observed disagreed with inference beyond")
    print("    threshold), STAL (no fresh PPS yardstick; interval held), SEED (chain")
    print("    reseeded this row), --- (rail not valid).")
    print("  • Head-to-head emo/imo are computed over agree seconds only: both rails judged")
    print("    against the same clean raw evidence.")
    print("  • p_* columns are the physical PPS rail; PPS has no separate gate/published rail here.")
    print("  • *_cΔ is Alpha/process_interrupt counter32 delta since the previous event.")
    print("  • Use a clock filter for narrower one-line rows, e.g. raw_cycles Plover1 OCXO2.")
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