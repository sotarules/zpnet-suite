"""
ZPNet Raw Cycle Excursions — focused forensic report.

This is a companion to raw_cycles.py.  It scans TIMEBASE rows for the same
OCXO raw-cycle excursion signatures that raw_cycles exposes, but prints only
excursion rows and expands each one with the DWT/service/counter custody facts
needed to decide where the impossible interval was born.

Usage:
    python raw_cycle_excursions.py <campaign>
    python raw_cycle_excursions.py <campaign> --clock OCXO1
    python raw_cycle_excursions.py <campaign> --threshold 100 --max 80

Doctrine:
    *_raw in raw_cycles is TIMEBASE_FORENSICS lane forensics
    dwt_interval_gate.observed_cycles when present.  It is not the ISR-entry
    raw DWT.  It is the observed/event-coordinate interval after
    process_interrupt has converted priority-0 ISR evidence into an event DWT.

    The key diagnostic split in this report is therefore:

        isr_raw_interval       = dwt_isr_entry_raw[n] - dwt_isr_entry_raw[n-1]
        event_raw_interval     = dwt_event_from_isr_entry_raw[n] - previous
        original_dwt_interval  = dwt_original_at_event[n] - previous
        used_dwt_interval      = dwt_used_at_event[n] - previous
        observed_interval      = dwt_interval_gate.observed_cycles
        inferred_interval      = dwt_yardstick.inferred_interval_cycles

    If ISR-entry intervals are normal but event/observed intervals explode,
    the fault is almost certainly service-offset correction / low-word
    generation math.  If ISR-entry intervals explode too, the compare event
    was actually presented late or the target was armed in the wrong generation.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


EXPECTED_OCXO_CDELTA = 10_000_000
DEFAULT_EXCURSION_THRESHOLD_CYCLES = 100
DEFAULT_QTIMER_ENTRY_LATENCY_CYCLES = 41
LANES = ("OCXO1", "OCXO2")
LOW16_MOD = 65_536


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
    frag = _root(rec).get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics_root(rec: Dict[str, Any]) -> Dict[str, Any]:
    f = _root(rec).get("forensics")
    return f if isinstance(f, dict) else {}


def _nested_get(obj: Any, *path: str) -> Any:
    cur = obj
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
        out = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(out) else out


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


def _u32_delta(now: Optional[int], prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    return (now - prev) & 0xFFFFFFFF


def _interval_delta(now: Optional[int], prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    return now - prev


def _fmt_int(v: Optional[int], signed: bool = False) -> str:
    if v is None:
        return "---"
    return f"{v:+,d}" if signed else f"{v:,d}"


def _fmt_bool(v: Optional[bool]) -> str:
    if v is None:
        return "---"
    return "Y" if v else "N"


def _fmt_float(v: Optional[float], decimals: int = 3, signed: bool = False) -> str:
    if v is None:
        return "---"
    return f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"


def _fmt_kv(label: str, value: Any) -> str:
    if isinstance(value, bool) or value is None:
        text = _fmt_bool(value) if isinstance(value, bool) or value is None else str(value)
    elif isinstance(value, int):
        text = _fmt_int(value, signed=value < 0)
    elif isinstance(value, float):
        text = _fmt_float(value, signed=value < 0)
    else:
        text = str(value)
    return f"{label}={text}"


def lane_key(lane: str) -> str:
    return lane.strip().lower()


def lane_short(lane: str) -> str:
    l = lane_key(lane)
    return "o1" if l == "ocxo1" else "o2"


def lane_forensics_obj(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Dict[str, Any]:
    l = lane_key(lane)
    candidates = (
        _nested_get(forensic, l, "forensics"),
        _nested_get(frag, l, "forensics"),
        _nested_get(root, l, "forensics"),
        _nested_get(root, "fragment", l, "forensics"),
        _nested_get(root, "clock_forensics", l, "alpha_event"),
        _nested_get(frag, "clock_forensics", l, "alpha_event"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_service_obj(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Dict[str, Any]:
    l = lane_key(lane)
    candidates = (
        _nested_get(forensic, l, "service"),
        _nested_get(frag, l, "service"),
        _nested_get(root, l, "service"),
        _nested_get(root, "fragment", l, "service"),
        _nested_get(forensic, l, "ocxo_service"),
        _nested_get(frag, l, "ocxo_service"),
        _nested_get(root, l, "ocxo_service"),
        _nested_get(root, "clock_forensics", l, "alpha_event", "ocxo_service"),
        _nested_get(frag, "clock_forensics", l, "alpha_event", "ocxo_service"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def prediction_obj(frag: Dict[str, Any], lane: str) -> Dict[str, Any]:
    pred = frag.get("prediction")
    if not isinstance(pred, dict):
        return {}
    lane_pred = pred.get(lane_key(lane)) or pred.get(lane.upper())
    return lane_pred if isinstance(lane_pred, dict) else {}


def pps_count_from_schema(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def cps_from_schema(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        forensic.get("dwt_cycles_between_pps_vclock"),
        frag.get("dwt_cycles_between_pps_vclock"),
        _nested_get(frag, "dwt", "cycles_between_pps_vclock"),
        root.get("dwt_cycles_between_pps_vclock"),
        _nested_get(root, "payload", "dwt_cycles_between_pps_vclock"),
    )


def pps_delta_from_schema(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    pps_pred = prediction_obj(frag, "pps")
    return _first_int(
        pps_pred.get("residual_cycles"),
        pps_pred.get("static_residual_cycles"),
        frag.get("pps_residual_cycles"),
    )


def service_class_name(v: Optional[int]) -> str:
    if v == 1:
        return "FALSE_IRQ"
    if v == 2:
        return "EARLY"
    if v == 3:
        return "OK"
    if v == 0:
        return "NONE"
    return "---"


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


def signed_cycles_for_ticks(ticks: Optional[int], cps: Optional[int]) -> Optional[int]:
    if ticks is None or cps is None:
        return None
    numerator = ticks * cps
    denom = EXPECTED_OCXO_CDELTA
    if numerator >= 0:
        return int((numerator + denom // 2) // denom)
    return -int(((-numerator) + denom // 2) // denom)


@dataclass
class LaneRecord:
    pps: int
    lane: str
    cps: Optional[int]
    pps_delta: Optional[int]

    raw: Optional[int]
    raw_delta: Optional[int]
    ema: Optional[int]
    ema_delta: Optional[int]
    inferred: Optional[int]
    imo: Optional[int]
    aerr: Optional[int]
    walk: Optional[int]
    y_tag: str

    gate_valid: Optional[bool]
    gate_accepted: Optional[bool]
    gate_rejected: Optional[bool]
    gate_residual: Optional[int]
    gate_threshold: Optional[int]
    gate_reject_count: Optional[int]
    gate_reject_streak: Optional[int]

    adjacency_valid: Optional[bool]
    adjacency_ok: Optional[bool]
    adjacency_rejected: Optional[bool]
    counter_delta: Optional[int]
    expected_counter_delta: Optional[int]
    counter_delta_error: Optional[int]
    adjacency_reject_count: Optional[int]

    dwt_isr_entry_raw: Optional[int]
    dwt_event_from_isr_entry_raw: Optional[int]
    dwt_original_at_event: Optional[int]
    dwt_used_at_event: Optional[int]
    dwt_predicted_at_event: Optional[int]
    dwt_ema_dwt_at_event: Optional[int]
    dwt_isr_to_event_correction: Optional[int]
    dwt_published_minus_event: Optional[int]
    dwt_used_minus_event: Optional[int]
    dwt_synthetic: Optional[bool]
    dwt_synthetic_error: Optional[int]
    dwt_synthetic_threshold: Optional[int]

    isr_raw_interval: Optional[int]
    event_from_raw_interval: Optional[int]
    original_dwt_interval: Optional[int]
    used_dwt_interval: Optional[int]
    predicted_dwt_interval: Optional[int]
    ema_dwt_interval: Optional[int]

    service_class: Optional[int]
    service_offset_ticks: Optional[int]
    service_offset_abs_ticks: Optional[int]
    interpreted_late_ticks: Optional[int]
    early_ticks: Optional[int]
    target_delta_mod65536_ticks: Optional[int]
    arm_remaining_ticks: Optional[int]
    arm_to_isr_ticks: Optional[int]
    arm_to_isr_dwt_cycles: Optional[int]
    perishable_fact_sequence: Optional[int]
    service_correction_cycles: Optional[int]
    service_corrected_dwt_at_event: Optional[int]
    fact_ring_overflow_count: Optional[int]
    counter_delta_violation_count: Optional[int]
    last_bad_counter_delta: Optional[int]
    sample_phase_valid: Optional[bool]
    sample_phase_ticks: Optional[int]
    sample_period_ticks: Optional[int]
    sample_dwt_at_event: Optional[int]
    sample_counter32_at_event: Optional[int]

    derived_service_correction_cycles: Optional[int]
    derived_total_isr_to_event_correction: Optional[int]
    correction_mismatch_cycles: Optional[int]
    arm_to_isr_expected_cycles: Optional[int]
    arm_to_isr_cycle_mismatch: Optional[int]
    arm_to_isr_ticks_mod65536: Optional[int]
    possible_low16_wraps_from_dwt: Optional[float]
    offset_delta_ticks: Optional[int]
    offset_delta_cycles: Optional[int]

    trigger_reasons: List[str] = field(default_factory=list)


def extract_lane_record(
    pps: int,
    lane: str,
    root: Dict[str, Any],
    frag: Dict[str, Any],
    forensic: Dict[str, Any],
    prev: Dict[str, Optional[int]],
    threshold: int,
    qtimer_latency_cycles: int,
) -> LaneRecord:
    l = lane_key(lane)
    f = lane_forensics_obj(root, frag, forensic, lane)
    s = lane_service_obj(root, frag, forensic, lane)
    pred = prediction_obj(frag, lane)
    gate = f.get("dwt_interval_gate") if isinstance(f.get("dwt_interval_gate"), dict) else {}
    adj = f.get("dwt_interval_adjacency") if isinstance(f.get("dwt_interval_adjacency"), dict) else {}
    y = f.get("dwt_yardstick") if isinstance(f.get("dwt_yardstick"), dict) else {}
    prefix = f"{l}_forensics_"

    cps = cps_from_schema(root, frag, forensic)
    pps_delta = pps_delta_from_schema(root, frag, forensic)

    raw = _first_int(
        gate.get("observed_cycles"),
        f.get("dwt_interval_observed_cycles"),
        y.get("observed_interval_cycles"),
        pred.get("actual_cycles"),
        pred.get("static_actual_cycles"),
    )
    ema = _first_int(
        gate.get("effective_cycles"),
        f.get("dwt_interval_effective_cycles"),
        pred.get("actual_cycles"),
    )
    inferred = _first_int(y.get("inferred_interval_cycles"), f.get("dwt_yardstick_inferred_interval_cycles"))
    imo = _first_int(y.get("inferred_minus_observed_cycles"), f.get("dwt_yardstick_inferred_minus_observed_cycles"))
    if imo is None and inferred is not None and raw is not None:
        imo = inferred - raw

    aerr = _first_int(y.get("auth_error_cycles"), f.get("dwt_yardstick_auth_error_cycles"))
    walk = _first_int(y.get("endpoint_minus_observed_cycles"), f.get("dwt_yardstick_endpoint_minus_observed_cycles"))
    y_valid = _first_bool(y.get("valid"), f.get("dwt_yardstick_valid"))
    y_obj = {
        "valid": y_valid,
        "seeded": _first_bool(y.get("seeded"), f.get("dwt_yardstick_seeded")),
        "stale": _first_bool(y.get("stale"), f.get("dwt_yardstick_stale")),
        "excursion": _first_bool(y.get("excursion"), f.get("dwt_yardstick_excursion")),
    }

    dwt_isr_entry_raw = _first_int(f.get("dwt_isr_entry_raw"), frag.get(prefix + "dwt_isr_entry_raw"), root.get(prefix + "dwt_isr_entry_raw"))
    dwt_event_from_isr_entry_raw = _first_int(f.get("dwt_event_from_isr_entry_raw"), frag.get(prefix + "dwt_event_from_isr_entry_raw"), root.get(prefix + "dwt_event_from_isr_entry_raw"))
    dwt_original_at_event = _first_int(f.get("dwt_original_at_event"), frag.get(prefix + "dwt_original_at_event"), root.get(prefix + "dwt_original_at_event"))
    dwt_used_at_event = _first_int(f.get("dwt_used_at_event"), frag.get(prefix + "dwt_used_at_event"), root.get(prefix + "dwt_used_at_event"))
    dwt_predicted_at_event = _first_int(f.get("dwt_predicted_at_event"), frag.get(prefix + "dwt_predicted_at_event"), root.get(prefix + "dwt_predicted_at_event"))
    dwt_ema_dwt_at_event = _first_int(f.get("dwt_ema_dwt_at_event"), frag.get(prefix + "dwt_ema_dwt_at_event"), root.get(prefix + "dwt_ema_dwt_at_event"))

    service_offset_ticks = _first_int(
        s.get("offset_ticks"),
        s.get("offset_signed_ticks"),
        s.get("service_offset"),
        f.get("ocxo_service_offset_signed_ticks"),
        f.get("diag_service_offset_signed_ticks"),
        frag.get(prefix + "service_offset_ticks"),
        root.get(prefix + "service_offset_ticks"),
    )
    service_correction_cycles = _first_int(
        s.get("correction_cycles"),
        s.get("service_correction_cycles"),
        f.get("ocxo_service_correction_cycles"),
        f.get("diag_service_correction_cycles"),
        frag.get(prefix + "service_correction_cycles"),
        root.get(prefix + "service_correction_cycles"),
    )
    dwt_isr_to_event_correction = _first_int(f.get("dwt_isr_entry_to_event_correction_cycles"), frag.get(prefix + "dwt_isr_entry_to_event_correction_cycles"), root.get(prefix + "dwt_isr_entry_to_event_correction_cycles"))

    derived_service_correction_cycles = signed_cycles_for_ticks(service_offset_ticks, cps)
    derived_total_isr_to_event_correction = (
        -qtimer_latency_cycles - derived_service_correction_cycles
        if derived_service_correction_cycles is not None
        else None
    )
    correction_mismatch_cycles = (
        dwt_isr_to_event_correction - derived_total_isr_to_event_correction
        if dwt_isr_to_event_correction is not None and derived_total_isr_to_event_correction is not None
        else None
    )

    arm_to_isr_ticks = _first_int(s.get("arm_to_isr_ticks"), f.get("ocxo_arm_to_isr_ticks"), f.get("diag_arm_to_isr_ticks"), frag.get(prefix + "arm_to_isr_ticks"), root.get(prefix + "arm_to_isr_ticks"))
    arm_to_isr_dwt_cycles = _first_int(s.get("arm_to_isr_dwt_cycles"), f.get("ocxo_arm_to_isr_dwt_cycles"), f.get("diag_arm_to_isr_dwt_cycles"), frag.get(prefix + "arm_to_isr_dwt_cycles"), root.get(prefix + "arm_to_isr_dwt_cycles"))
    arm_to_isr_expected_cycles = signed_cycles_for_ticks(arm_to_isr_ticks, cps)
    arm_to_isr_cycle_mismatch = (
        arm_to_isr_dwt_cycles - arm_to_isr_expected_cycles
        if arm_to_isr_dwt_cycles is not None and arm_to_isr_expected_cycles is not None
        else None
    )
    possible_low16_wraps_from_dwt = None
    if arm_to_isr_dwt_cycles is not None and cps:
        elapsed_ticks_from_dwt = arm_to_isr_dwt_cycles * EXPECTED_OCXO_CDELTA / cps
        if arm_to_isr_ticks is not None:
            possible_low16_wraps_from_dwt = (elapsed_ticks_from_dwt - arm_to_isr_ticks) / LOW16_MOD

    offset_delta_ticks = _interval_delta(service_offset_ticks, prev.get("service_offset_ticks"))
    offset_delta_cycles = signed_cycles_for_ticks(offset_delta_ticks, cps)

    counter_delta = _first_int(
        adj.get("counter_delta_ticks"),
        f.get("dwt_interval_counter_delta_ticks"),
        s.get("last_counter_delta_ticks"),
        f.get("ocxo_last_counter_delta_ticks"),
        f.get("diag_last_counter_delta_ticks"),
    )
    expected_counter_delta = _first_int(
        adj.get("expected_counter_delta_ticks"),
        f.get("dwt_interval_expected_counter_delta_ticks"),
        f.get("ocxo_expected_counter_delta_ticks"),
        EXPECTED_OCXO_CDELTA,
    )
    counter_delta_error = (
        counter_delta - expected_counter_delta
        if counter_delta is not None and expected_counter_delta is not None
        else None
    )

    rec = LaneRecord(
        pps=pps,
        lane=lane,
        cps=cps,
        pps_delta=pps_delta,
        raw=raw,
        raw_delta=_interval_delta(raw, prev.get("raw")),
        ema=ema,
        ema_delta=_interval_delta(ema, prev.get("ema")),
        inferred=inferred,
        imo=imo,
        aerr=aerr,
        walk=walk,
        y_tag=yardstick_tag(y_obj),
        gate_valid=_first_bool(gate.get("valid"), f.get("dwt_interval_gate_valid")),
        gate_accepted=_first_bool(gate.get("accepted"), f.get("dwt_interval_sample_accepted")),
        gate_rejected=_first_bool(gate.get("rejected"), f.get("dwt_interval_sample_rejected")),
        gate_residual=_first_int(gate.get("residual_cycles"), f.get("dwt_interval_residual_cycles")),
        gate_threshold=_first_int(gate.get("threshold_cycles"), f.get("dwt_interval_gate_threshold_cycles")),
        gate_reject_count=_first_int(gate.get("reject_count"), f.get("dwt_interval_reject_count")),
        gate_reject_streak=_first_int(gate.get("reject_streak"), f.get("dwt_interval_reject_streak")),
        adjacency_valid=_first_bool(adj.get("valid"), f.get("dwt_interval_adjacency_gate_valid")),
        adjacency_ok=_first_bool(adj.get("ok"), f.get("dwt_interval_adjacency_ok")),
        adjacency_rejected=_first_bool(adj.get("rejected"), f.get("dwt_interval_adjacency_rejected")),
        counter_delta=counter_delta,
        expected_counter_delta=expected_counter_delta,
        counter_delta_error=counter_delta_error,
        adjacency_reject_count=_first_int(adj.get("reject_count"), f.get("dwt_interval_adjacency_reject_count")),
        dwt_isr_entry_raw=dwt_isr_entry_raw,
        dwt_event_from_isr_entry_raw=dwt_event_from_isr_entry_raw,
        dwt_original_at_event=dwt_original_at_event,
        dwt_used_at_event=dwt_used_at_event,
        dwt_predicted_at_event=dwt_predicted_at_event,
        dwt_ema_dwt_at_event=dwt_ema_dwt_at_event,
        dwt_isr_to_event_correction=dwt_isr_to_event_correction,
        dwt_published_minus_event=_first_int(f.get("dwt_published_minus_event_cycles"), frag.get(prefix + "dwt_published_minus_event_cycles"), root.get(prefix + "dwt_published_minus_event_cycles")),
        dwt_used_minus_event=_first_int(f.get("dwt_used_minus_event_cycles"), frag.get(prefix + "dwt_used_minus_event_cycles"), root.get(prefix + "dwt_used_minus_event_cycles")),
        dwt_synthetic=_first_bool(f.get("dwt_synthetic"), frag.get(prefix + "dwt_synthetic"), root.get(prefix + "dwt_synthetic")),
        dwt_synthetic_error=_first_int(f.get("dwt_synthetic_error_cycles"), frag.get(prefix + "dwt_synthetic_error_cycles"), root.get(prefix + "dwt_synthetic_error_cycles")),
        dwt_synthetic_threshold=_first_int(f.get("dwt_synthetic_threshold_cycles"), frag.get(prefix + "dwt_synthetic_threshold_cycles"), root.get(prefix + "dwt_synthetic_threshold_cycles")),
        isr_raw_interval=_u32_delta(dwt_isr_entry_raw, prev.get("dwt_isr_entry_raw")),
        event_from_raw_interval=_u32_delta(dwt_event_from_isr_entry_raw, prev.get("dwt_event_from_isr_entry_raw")),
        original_dwt_interval=_u32_delta(dwt_original_at_event, prev.get("dwt_original_at_event")),
        used_dwt_interval=_u32_delta(dwt_used_at_event, prev.get("dwt_used_at_event")),
        predicted_dwt_interval=_u32_delta(dwt_predicted_at_event, prev.get("dwt_predicted_at_event")),
        ema_dwt_interval=_u32_delta(dwt_ema_dwt_at_event, prev.get("dwt_ema_dwt_at_event")),
        service_class=_first_int(s.get("class"), s.get("service_class"), f.get("ocxo_service_class"), f.get("diag_service_class")),
        service_offset_ticks=service_offset_ticks,
        service_offset_abs_ticks=_first_int(s.get("offset_abs_ticks"), f.get("ocxo_service_offset_abs_ticks"), f.get("diag_service_offset_abs_ticks")),
        interpreted_late_ticks=_first_int(s.get("interpreted_late_ticks"), f.get("ocxo_interpreted_late_ticks"), f.get("diag_interpreted_late_ticks")),
        early_ticks=_first_int(s.get("early_ticks"), f.get("ocxo_early_ticks"), f.get("diag_early_ticks")),
        target_delta_mod65536_ticks=_first_int(s.get("target_delta_mod65536_ticks"), f.get("ocxo_target_delta_mod65536_ticks"), f.get("diag_target_delta_mod65536_ticks")),
        arm_remaining_ticks=_first_int(s.get("arm_remaining_ticks"), f.get("ocxo_arm_remaining_ticks"), f.get("diag_arm_remaining_ticks")),
        arm_to_isr_ticks=arm_to_isr_ticks,
        arm_to_isr_dwt_cycles=arm_to_isr_dwt_cycles,
        perishable_fact_sequence=_first_int(s.get("perishable_fact_sequence"), f.get("ocxo_perishable_fact_sequence"), f.get("diag_perishable_fact_sequence")),
        service_correction_cycles=service_correction_cycles,
        service_corrected_dwt_at_event=_first_int(s.get("corrected_dwt_at_event"), s.get("service_corrected_dwt_at_event"), f.get("ocxo_service_corrected_dwt_at_event"), f.get("diag_service_corrected_dwt_at_event")),
        fact_ring_overflow_count=_first_int(s.get("fact_ring_overflow_count"), f.get("ocxo_fact_ring_overflow_count"), f.get("diag_fact_ring_overflow_count")),
        counter_delta_violation_count=_first_int(s.get("counter_delta_violation_count"), f.get("ocxo_counter_delta_violation_count"), f.get("diag_counter_delta_violation_count")),
        last_bad_counter_delta=_first_int(s.get("last_bad_counter_delta"), f.get("ocxo_last_bad_counter_delta"), f.get("diag_last_bad_counter_delta")),
        sample_phase_valid=_first_bool(s.get("sample_phase_valid"), f.get("ocxo_sample_phase_valid")),
        sample_phase_ticks=_first_int(s.get("sample_phase_ticks"), f.get("ocxo_sample_phase_ticks")),
        sample_period_ticks=_first_int(s.get("sample_period_ticks"), f.get("ocxo_sample_period_ticks")),
        sample_dwt_at_event=_first_int(s.get("sample_dwt_at_event"), f.get("ocxo_sample_dwt_at_event")),
        sample_counter32_at_event=_first_int(s.get("sample_counter32_at_event"), f.get("ocxo_sample_counter32_at_event")),
        derived_service_correction_cycles=derived_service_correction_cycles,
        derived_total_isr_to_event_correction=derived_total_isr_to_event_correction,
        correction_mismatch_cycles=correction_mismatch_cycles,
        arm_to_isr_expected_cycles=arm_to_isr_expected_cycles,
        arm_to_isr_cycle_mismatch=arm_to_isr_cycle_mismatch,
        arm_to_isr_ticks_mod65536=(arm_to_isr_ticks % LOW16_MOD) if arm_to_isr_ticks is not None else None,
        possible_low16_wraps_from_dwt=possible_low16_wraps_from_dwt,
        offset_delta_ticks=offset_delta_ticks,
        offset_delta_cycles=offset_delta_cycles,
    )

    reasons: List[str] = []
    if rec.raw_delta is not None and abs(rec.raw_delta) >= threshold:
        reasons.append("rawΔ")
    if rec.ema_delta is not None and abs(rec.ema_delta) >= threshold:
        reasons.append("emaΔ")
    if y_obj["excursion"]:
        reasons.append("yardstick_EXC")
    if rec.imo is not None and rec.gate_threshold is not None and abs(rec.imo) > rec.gate_threshold:
        reasons.append("imo>gate")
    if rec.gate_rejected:
        reasons.append("interval_gate_REJ")
    if rec.adjacency_rejected:
        reasons.append("adjacency_REJ")
    if rec.counter_delta_error not in (None, 0):
        reasons.append("counterΔ_BAD")
    rec.trigger_reasons = reasons
    return rec


def update_prev(prev: Dict[str, Optional[int]], rec: LaneRecord) -> None:
    prev["raw"] = rec.raw
    prev["ema"] = rec.ema
    prev["dwt_isr_entry_raw"] = rec.dwt_isr_entry_raw
    prev["dwt_event_from_isr_entry_raw"] = rec.dwt_event_from_isr_entry_raw
    prev["dwt_original_at_event"] = rec.dwt_original_at_event
    prev["dwt_used_at_event"] = rec.dwt_used_at_event
    prev["dwt_predicted_at_event"] = rec.dwt_predicted_at_event
    prev["dwt_ema_dwt_at_event"] = rec.dwt_ema_dwt_at_event
    prev["service_offset_ticks"] = rec.service_offset_ticks


def classify(rec: LaneRecord, threshold: int) -> str:
    if not rec.trigger_reasons:
        return "not_an_excursion"

    isr_err = None
    if rec.isr_raw_interval is not None and rec.inferred is not None:
        isr_err = rec.isr_raw_interval - rec.inferred
    event_err = None
    if rec.event_from_raw_interval is not None and rec.inferred is not None:
        event_err = rec.event_from_raw_interval - rec.inferred

    if isr_err is not None and abs(isr_err) >= threshold:
        return "compare_presented_late_or_wrong_generation"
    if event_err is not None and abs(event_err) >= threshold:
        return "service_correction_or_event_coordinate_math"
    if rec.imo is not None and abs(rec.imo) >= threshold:
        return "yardstick_observed_interval_disagreement"
    return "small_or_incomplete_evidence"


def missing_fields_for(rec: LaneRecord) -> List[str]:
    required = {
        "target_delta_mod65536_ticks": rec.target_delta_mod65536_ticks,
        "interpreted_late_ticks": rec.interpreted_late_ticks,
        "early_ticks": rec.early_ticks,
        "arm_remaining_ticks": rec.arm_remaining_ticks,
        "arm_to_isr_dwt_cycles": rec.arm_to_isr_dwt_cycles,
        "service_correction_cycles": rec.service_correction_cycles,
        "service_corrected_dwt_at_event": rec.service_corrected_dwt_at_event,
        "perishable_fact_sequence": rec.perishable_fact_sequence,
        "fact_ring_overflow_count": rec.fact_ring_overflow_count,
        "counter_delta_violation_count": rec.counter_delta_violation_count,
        "sample_dwt_at_event": rec.sample_dwt_at_event,
        "sample_counter32_at_event": rec.sample_counter32_at_event,
    }
    return [k for k, v in required.items() if v is None]


def collect_excursions(
    rows: List[Dict[str, Any]],
    threshold: int,
    clock_filter: Optional[str],
    qtimer_latency_cycles: int,
) -> Tuple[List[LaneRecord], Dict[str, int], int]:
    lanes = LANES if clock_filter is None else (clock_filter,)
    prev_by_lane: Dict[str, Dict[str, Optional[int]]] = {lane: {} for lane in lanes}
    excursions: List[LaneRecord] = []
    coverage_missing: Dict[str, int] = {}
    gaps = 0
    prev_pps: Optional[int] = None

    for row in rows:
        root = _root(row)
        frag = _frag(row)
        forensic = _forensics_root(row)
        pps = pps_count_from_schema(root, frag, forensic)
        if pps is None:
            continue
        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            # Deltas across campaign gaps are not meaningful.
            for state in prev_by_lane.values():
                state.clear()

        for lane in lanes:
            rec = extract_lane_record(
                pps=pps,
                lane=lane,
                root=root,
                frag=frag,
                forensic=forensic,
                prev=prev_by_lane[lane],
                threshold=threshold,
                qtimer_latency_cycles=qtimer_latency_cycles,
            )
            if rec.trigger_reasons:
                excursions.append(rec)
                for name in missing_fields_for(rec):
                    coverage_missing[name] = coverage_missing.get(name, 0) + 1
            update_prev(prev_by_lane[lane], rec)
        prev_pps = pps

    return excursions, coverage_missing, gaps


def print_summary(campaign: str, row_count: int, excursions: List[LaneRecord], threshold: int, gaps: int) -> None:
    print(f"Campaign: {campaign}  ({row_count:,} TIMEBASE rows scanned)")
    print(f"Excursion threshold: |rawΔ| or |emaΔ| >= {threshold:,} cycles, plus yardstick/gate/adjacency rejects")
    print(f"Gaps reset delta baselines: {gaps:,}")
    print(f"Excursion lane-rows: {len(excursions):,}")
    by_lane: Dict[str, int] = {}
    by_class: Dict[str, int] = {}
    for rec in excursions:
        by_lane[rec.lane] = by_lane.get(rec.lane, 0) + 1
        c = classify(rec, threshold)
        by_class[c] = by_class.get(c, 0) + 1
    print(f"By lane: {by_lane or '{}'}")
    print(f"By first-pass classification: {by_class or '{}'}")
    print()


def print_compact_table(excursions: List[LaneRecord], threshold: int, max_rows: int) -> None:
    print("Compact excursion index")
    print("═══════════════════════")
    if not excursions:
        print("  none")
        print()
        return
    h = (
        f"{'#':>3s} {'pps':>7s} {'lane':>5s} {'triggers':<30s} "
        f"{'class':<42s} {'rawΔ':>12s} {'imo':>9s} {'isr-int':>13s} "
        f"{'event-int':>13s} {'svc_off':>9s} {'offΔ':>9s} {'cΔ':>11s}"
    )
    print(h)
    print("─" * len(h))
    shown = excursions[:max_rows] if max_rows else excursions
    for i, rec in enumerate(shown, start=1):
        triggers = "+".join(rec.trigger_reasons)
        print(
            f"{i:>3d} {rec.pps:>7d} {lane_short(rec.lane):>5s} {triggers:<30.30s} "
            f"{classify(rec, threshold):<42.42s} "
            f"{_fmt_int(rec.raw_delta, signed=True):>12s} "
            f"{_fmt_int(rec.imo, signed=True):>9s} "
            f"{_fmt_int(rec.isr_raw_interval):>13s} "
            f"{_fmt_int(rec.event_from_raw_interval):>13s} "
            f"{_fmt_int(rec.service_offset_ticks, signed=True):>9s} "
            f"{_fmt_int(rec.offset_delta_ticks, signed=True):>9s} "
            f"{_fmt_int(rec.counter_delta):>11s}"
        )
    if max_rows and len(excursions) > max_rows:
        print(f"  ... {len(excursions) - max_rows:,} more omitted by --max")
    print()


def print_detail(rec: LaneRecord, idx: int, threshold: int) -> None:
    print(f"Excursion {idx}: pps={rec.pps:,} lane={rec.lane} triggers={'+'.join(rec.trigger_reasons)}")
    print("─" * 96)
    print(f"  classification: {classify(rec, threshold)}")
    print(
        "  intervals:      "
        + "  ".join([
            _fmt_kv("raw", rec.raw),
            _fmt_kv("rawΔ", rec.raw_delta),
            _fmt_kv("ema", rec.ema),
            _fmt_kv("emaΔ", rec.ema_delta),
            _fmt_kv("inferred", rec.inferred),
            _fmt_kv("imo", rec.imo),
            _fmt_kv("aerr", rec.aerr),
            _fmt_kv("walk", rec.walk),
            f"y={rec.y_tag}",
        ])
    )
    print(
        "  gate:           "
        + "  ".join([
            _fmt_kv("valid", rec.gate_valid),
            _fmt_kv("accepted", rec.gate_accepted),
            _fmt_kv("rejected", rec.gate_rejected),
            _fmt_kv("resid", rec.gate_residual),
            _fmt_kv("threshold", rec.gate_threshold),
            _fmt_kv("reject_count", rec.gate_reject_count),
            _fmt_kv("reject_streak", rec.gate_reject_streak),
        ])
    )
    print(
        "  adjacency:      "
        + "  ".join([
            _fmt_kv("valid", rec.adjacency_valid),
            _fmt_kv("ok", rec.adjacency_ok),
            _fmt_kv("rejected", rec.adjacency_rejected),
            _fmt_kv("cΔ", rec.counter_delta),
            _fmt_kv("expected", rec.expected_counter_delta),
            _fmt_kv("cΔ_err", rec.counter_delta_error),
            _fmt_kv("reject_count", rec.adjacency_reject_count),
        ])
    )
    print(
        "  dwt endpoints:  "
        + "  ".join([
            _fmt_kv("isr_raw", rec.dwt_isr_entry_raw),
            _fmt_kv("event_from_raw", rec.dwt_event_from_isr_entry_raw),
            _fmt_kv("original", rec.dwt_original_at_event),
            _fmt_kv("used", rec.dwt_used_at_event),
            _fmt_kv("pred", rec.dwt_predicted_at_event),
            _fmt_kv("ema_dwt", rec.dwt_ema_dwt_at_event),
        ])
    )
    print(
        "  dwt intervals:  "
        + "  ".join([
            _fmt_kv("isr_raw_int", rec.isr_raw_interval),
            _fmt_kv("event_int", rec.event_from_raw_interval),
            _fmt_kv("original_int", rec.original_dwt_interval),
            _fmt_kv("used_int", rec.used_dwt_interval),
            _fmt_kv("pred_int", rec.predicted_dwt_interval),
            _fmt_kv("ema_int", rec.ema_dwt_interval),
        ])
    )
    print(
        "  dwt correction: "
        + "  ".join([
            _fmt_kv("isr→event", rec.dwt_isr_to_event_correction),
            _fmt_kv("pub-event", rec.dwt_published_minus_event),
            _fmt_kv("used-event", rec.dwt_used_minus_event),
            _fmt_kv("synthetic", rec.dwt_synthetic),
            _fmt_kv("synthetic_err", rec.dwt_synthetic_error),
            _fmt_kv("synthetic_gate", rec.dwt_synthetic_threshold),
        ])
    )
    print(
        "  service:        "
        + "  ".join([
            f"class={service_class_name(rec.service_class)}({rec.service_class if rec.service_class is not None else '---'})",
            _fmt_kv("offset_ticks", rec.service_offset_ticks),
            _fmt_kv("offset_abs", rec.service_offset_abs_ticks),
            _fmt_kv("late", rec.interpreted_late_ticks),
            _fmt_kv("early", rec.early_ticks),
            _fmt_kv("targetΔmod", rec.target_delta_mod65536_ticks),
            _fmt_kv("arm_rem", rec.arm_remaining_ticks),
            _fmt_kv("arm→isr_ticks", rec.arm_to_isr_ticks),
        ])
    )
    print(
        "  service cont.:  "
        + "  ".join([
            _fmt_kv("arm→isr_dwt", rec.arm_to_isr_dwt_cycles),
            _fmt_kv("fact_seq", rec.perishable_fact_sequence),
            _fmt_kv("svc_corr", rec.service_correction_cycles),
            _fmt_kv("svc_corr_dwt", rec.service_corrected_dwt_at_event),
            _fmt_kv("ring_ovf", rec.fact_ring_overflow_count),
            _fmt_kv("cΔ_viol", rec.counter_delta_violation_count),
            _fmt_kv("last_bad_cΔ", rec.last_bad_counter_delta),
        ])
    )
    print(
        "  sample phase:   "
        + "  ".join([
            _fmt_kv("phase_valid", rec.sample_phase_valid),
            _fmt_kv("phase_ticks", rec.sample_phase_ticks),
            _fmt_kv("period_ticks", rec.sample_period_ticks),
            _fmt_kv("sample_dwt", rec.sample_dwt_at_event),
            _fmt_kv("sample_counter32", rec.sample_counter32_at_event),
        ])
    )
    print(
        "  derived:        "
        + "  ".join([
            _fmt_kv("cps", rec.cps),
            _fmt_kv("svc_corr_from_offset", rec.derived_service_correction_cycles),
            _fmt_kv("expected_isr→event", rec.derived_total_isr_to_event_correction),
            _fmt_kv("corr_mismatch", rec.correction_mismatch_cycles),
            _fmt_kv("arm→isr_exp_dwt", rec.arm_to_isr_expected_cycles),
            _fmt_kv("arm→isr_mismatch", rec.arm_to_isr_cycle_mismatch),
            _fmt_kv("arm→isr_ticks_mod", rec.arm_to_isr_ticks_mod65536),
            _fmt_kv("offsetΔ_ticks", rec.offset_delta_ticks),
            _fmt_kv("offsetΔ_cycles", rec.offset_delta_cycles),
        ])
    )
    if rec.possible_low16_wraps_from_dwt is not None:
        print(f"  low16-wrap estimate from arm→isr DWT/ticks: {rec.possible_low16_wraps_from_dwt:+.3f} wraps")

    missing = missing_fields_for(rec)
    if missing:
        print("  missing TIMEBASE fields for full diagnosis: " + ", ".join(missing))
    print()


def print_missing_coverage(missing_counts: Dict[str, int], excursion_count: int) -> None:
    print("Missing forensic field coverage on excursion rows")
    print("══════════════════════════════════════════════")
    if not missing_counts:
        print("  none — TIMEBASE has the full requested service-custody surface")
        print()
        return
    for name in sorted(missing_counts):
        print(f"  {name:<38s} missing on {missing_counts[name]:>5,d} / {excursion_count:,} excursion rows")
    print()


def normalize_clock(value: Optional[str]) -> Optional[str]:
    if not value:
        return None
    v = value.strip().upper()
    if v in ("O1", "OCXO1"):
        return "OCXO1"
    if v in ("O2", "OCXO2"):
        return "OCXO2"
    raise ValueError("--clock must be OCXO1 or OCXO2")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Focused OCXO raw-cycle excursion forensics")
    parser.add_argument("campaign", help="campaign name")
    parser.add_argument("--threshold", type=int, default=DEFAULT_EXCURSION_THRESHOLD_CYCLES,
                        help=f"cycle threshold for rawΔ/emaΔ triggers (default {DEFAULT_EXCURSION_THRESHOLD_CYCLES})")
    parser.add_argument("--clock", default=None, help="OCXO1 or OCXO2")
    parser.add_argument("--max", type=int, default=0,
                        help="maximum detailed excursion rows to print; 0 means all")
    parser.add_argument("--qtimer-latency", type=int, default=DEFAULT_QTIMER_ENTRY_LATENCY_CYCLES,
                        help=f"event-coordinate correction from ISR entry before service offset (default {DEFAULT_QTIMER_ENTRY_LATENCY_CYCLES})")
    args = parser.parse_args(argv)

    clock_filter = normalize_clock(args.clock)
    rows = fetch_timebase(args.campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign '{args.campaign}'")
        return 1

    excursions, missing_counts, gaps = collect_excursions(
        rows=rows,
        threshold=args.threshold,
        clock_filter=clock_filter,
        qtimer_latency_cycles=args.qtimer_latency,
    )

    print_summary(args.campaign, len(rows), excursions, args.threshold, gaps)
    print_compact_table(excursions, args.threshold, args.max)
    print_missing_coverage(missing_counts, len(excursions))

    detail_rows = excursions if args.max == 0 else excursions[:args.max]
    print("Detailed excursion dossiers")
    print("═══════════════════════════")
    if not detail_rows:
        print("  none")
    for idx, rec in enumerate(detail_rows, start=1):
        print_detail(rec, idx, args.threshold)
    if args.max and len(excursions) > args.max:
        print(f"... {len(excursions) - args.max:,} more detailed dossiers omitted by --max")
        print()

    print("Interpretation guide")
    print("════════════════════")
    print("  • If isr_raw_int is normal but event_int/original/raw are bad, the bad value was")
    print("    manufactured after priority-0 capture, likely in service-offset correction or")
    print("    low-word generation math.")
    print("  • If isr_raw_int is bad too, the compare interrupt itself was presented late or")
    print("    the compare target was armed in the wrong 16-bit generation.")
    print("  • If cΔ remains exactly 10,000,000 while raw explodes, event identity is adjacent;")
    print("    the wound is the DWT coordinate attached to that identity.")
    print("  • arm→isr_dwt and targetΔmod are the decisive fields.  If they are missing, patch")
    print("    TIMEBASE service forensics to publish the existing interrupt_capture_diag_t fields.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
