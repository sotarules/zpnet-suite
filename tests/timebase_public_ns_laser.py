#!/usr/bin/env python3
"""
TIMEBASE Public-NS Laser

Purpose
-------
This is a narrow forensic tool for the specific pathology seen in IRQ3:

    raw/counter lineage is clean
    measured_gnss_ns is sane
    public clock ns jumps by a huge amount
    Welford/PPB/TAU then ingest the bad public residual

This is NOT a general shotgun report.

It hunts:

    1. Inside-segment public ns interval jumps
    2. Public ns residual disagreement with pps_residual.fast_residual_ns
    3. Measured-vs-public divergence jumps
    4. Stats jumps that occur near a public-ns failure
    5. Whether counter/adja­cency/gate were innocent at the event

Native DB shape matches raw_cycles.py:

    from zpnet.shared.db import open_db

    SELECT payload
    FROM timebase
    WHERE campaign = %s
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


REPORT_VERSION = "2026-06-06-public-ns-laser-v1"

NSEC = 1_000_000_000
EXPECTED_OCXO_COUNTER_DELTA = 10_000_000


# ══════════════════════════════════════════════════════════════════════════════
# DB loader
# ══════════════════════════════════════════════════════════════════════════════

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

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        out.append(payload)
    return out


# ══════════════════════════════════════════════════════════════════════════════
# Generic helpers
# ══════════════════════════════════════════════════════════════════════════════

def nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def as_bool(v: Any) -> Optional[bool]:
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


def first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = as_int(v)
        if out is not None:
            return out
    return None


def first_float(*values: Any) -> Optional[float]:
    for v in values:
        out = as_float(v)
        if out is not None:
            return out
    return None


def first_bool(*values: Any) -> Optional[bool]:
    for v in values:
        out = as_bool(v)
        if out is not None:
            return out
    return None


def fmt_i(v: Optional[int]) -> str:
    return "---" if v is None else f"{v:,}"


def fmt_si(v: Optional[int]) -> str:
    return "---" if v is None else f"{v:+,}"


def fmt_f(v: Optional[float], digits: int = 6) -> str:
    return "---" if v is None else f"{v:+,.{digits}f}"


def fmt_tau(v: Optional[float]) -> str:
    return "---" if v is None else f"{v:.12f}"


def norm_lane(lane: str) -> str:
    t = lane.strip().lower()
    if t in ("all", "*"):
        return "all"
    if t in ("1", "o1", "ocxo1"):
        return "ocxo1"
    if t in ("2", "o2", "ocxo2"):
        return "ocxo2"
    if t in ("v", "vc", "vclk", "vclock"):
        return "vclock"
    raise ValueError(f"unknown lane '{lane}', expected OCXO1, OCXO2, VCLOCK, or all")


def resolve_lanes(lane_arg: str) -> List[str]:
    lane = norm_lane(lane_arg)
    if lane == "all":
        return ["ocxo1", "ocxo2", "vclock"]
    return [lane]


# ══════════════════════════════════════════════════════════════════════════════
# Schema helpers
# ══════════════════════════════════════════════════════════════════════════════

def root_of(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def frag_of(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = root_of(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def forensics_root_of(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = root_of(rec)
    f = root.get("forensics")
    return f if isinstance(f, dict) else {}


def prediction_of(rec: Dict[str, Any]) -> Dict[str, Any]:
    frag = frag_of(rec)
    pred = frag.get("prediction")
    return pred if isinstance(pred, dict) else {}


def get_pps(rec: Dict[str, Any]) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)
    return first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def get_gnss_ns(rec: Dict[str, Any]) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)
    return first_int(
        nested_get(root, "gnss", "ns"),
        root.get("gnss_ns"),
        frag.get("gnss_ns"),
        forensic.get("gnss_ns"),
    )


def lane_forensics_obj(rec: Dict[str, Any], lane: str) -> Dict[str, Any]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    candidates = (
        nested_get(forensic, lane, "forensics"),
        nested_get(frag, lane, "forensics"),
        nested_get(root, lane, "forensics"),
        nested_get(root, "fragment", lane, "forensics"),
        nested_get(root, "clock_forensics", lane, "alpha_event"),
        nested_get(frag, "clock_forensics", lane, "alpha_event"),
    )

    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_service_obj(rec: Dict[str, Any], lane: str) -> Dict[str, Any]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    candidates = (
        nested_get(forensic, lane, "service"),
        nested_get(frag, lane, "service"),
        nested_get(root, lane, "service"),
        nested_get(root, "fragment", lane, "service"),
        nested_get(root, "clock_forensics", lane, "alpha_event", "ocxo_service"),
        nested_get(frag, "clock_forensics", lane, "alpha_event", "ocxo_service"),
    )

    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_prediction(rec: Dict[str, Any], lane: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    frag = frag_of(rec)
    pred = prediction_of(rec)
    lane_pred = pred.get(lane) if isinstance(pred.get(lane), dict) else {}

    actual = first_int(
        lane_pred.get("actual_cycles"),
        frag.get(f"{lane}_actual_cycles"),
        pred.get(f"{lane}_actual_cycles"),
    )

    prediction = first_int(
        lane_pred.get("prediction_cycles"),
        lane_pred.get("static_prediction_cycles"),
        frag.get(f"{lane}_prediction_cycles"),
        pred.get(f"{lane}_prediction_cycles"),
        pred.get(f"{lane}_static_prediction_cycles"),
    )

    residual = first_int(
        lane_pred.get("residual_cycles"),
        lane_pred.get("static_residual_cycles"),
        frag.get(f"{lane}_residual_cycles"),
        pred.get(f"{lane}_residual_cycles"),
        pred.get(f"{lane}_static_residual_cycles"),
    )

    if residual is None and actual is not None and prediction is not None:
        residual = actual - prediction

    return actual, prediction, residual


def gate_diag(rec: Dict[str, Any], lane: str) -> Dict[str, Any]:
    root = root_of(rec)
    frag = frag_of(rec)
    f = lane_forensics_obj(rec, lane)
    gate = f.get("dwt_interval_gate") if isinstance(f.get("dwt_interval_gate"), dict) else {}
    prefix = f"{lane}_forensics_"

    return {
        "valid": first_bool(
            gate.get("valid"),
            f.get("dwt_interval_gate_valid"),
            frag.get(prefix + "dwt_interval_gate_valid"),
            root.get(prefix + "dwt_interval_gate_valid"),
        ),
        "accepted": first_bool(
            gate.get("accepted"),
            f.get("dwt_interval_sample_accepted"),
            frag.get(prefix + "dwt_interval_sample_accepted"),
            root.get(prefix + "dwt_interval_sample_accepted"),
        ),
        "rejected": first_bool(
            gate.get("rejected"),
            f.get("dwt_interval_sample_rejected"),
            frag.get(prefix + "dwt_interval_sample_rejected"),
            root.get(prefix + "dwt_interval_sample_rejected"),
        ),
        "observed_cycles": first_int(
            gate.get("observed_cycles"),
            f.get("dwt_interval_observed_cycles"),
            frag.get(prefix + "dwt_interval_observed_cycles"),
            root.get(prefix + "dwt_interval_observed_cycles"),
        ),
        "prediction_cycles": first_int(
            gate.get("prediction_cycles"),
            f.get("dwt_interval_prediction_cycles"),
            frag.get(prefix + "dwt_interval_prediction_cycles"),
            root.get(prefix + "dwt_interval_prediction_cycles"),
        ),
        "effective_cycles": first_int(
            gate.get("effective_cycles"),
            f.get("dwt_interval_effective_cycles"),
            frag.get(prefix + "dwt_interval_effective_cycles"),
            root.get(prefix + "dwt_interval_effective_cycles"),
        ),
        "residual_cycles": first_int(
            gate.get("residual_cycles"),
            f.get("dwt_interval_residual_cycles"),
            frag.get(prefix + "dwt_interval_residual_cycles"),
            root.get(prefix + "dwt_interval_residual_cycles"),
        ),
        "synthetic": first_bool(
            f.get("dwt_synthetic"),
            frag.get(prefix + "dwt_synthetic"),
            root.get(prefix + "dwt_synthetic"),
        ),
        "synthetic_error": first_int(
            f.get("dwt_synthetic_error_cycles"),
            frag.get(prefix + "dwt_synthetic_error_cycles"),
            root.get(prefix + "dwt_synthetic_error_cycles"),
        ),
        "synthetic_threshold": first_int(
            f.get("dwt_synthetic_threshold_cycles"),
            frag.get(prefix + "dwt_synthetic_threshold_cycles"),
            root.get(prefix + "dwt_synthetic_threshold_cycles"),
        ),
        "original_dwt": first_int(
            f.get("dwt_original_at_event"),
            frag.get(prefix + "dwt_original_at_event"),
            root.get(prefix + "dwt_original_at_event"),
        ),
        "predicted_dwt": first_int(
            f.get("dwt_predicted_at_event"),
            frag.get(prefix + "dwt_predicted_at_event"),
            root.get(prefix + "dwt_predicted_at_event"),
        ),
        "used_dwt": first_int(
            f.get("dwt_used_at_event"),
            frag.get(prefix + "dwt_used_at_event"),
            root.get(prefix + "dwt_used_at_event"),
        ),
        "isr_entry_dwt": first_int(
            f.get("dwt_isr_entry_raw"),
            frag.get(prefix + "dwt_isr_entry_raw"),
            root.get(prefix + "dwt_isr_entry_raw"),
        ),
        "last_event_counter32": first_int(
            f.get("last_event_counter32"),
            frag.get(prefix + "last_event_counter32"),
            root.get(prefix + "last_event_counter32"),
        ),
        "last_event_dwt": first_int(
            f.get("last_event_dwt"),
            frag.get(prefix + "last_event_dwt"),
            root.get(prefix + "last_event_dwt"),
        ),
    }


def adjacency_diag(rec: Dict[str, Any], lane: str) -> Dict[str, Any]:
    root = root_of(rec)
    frag = frag_of(rec)
    f = lane_forensics_obj(rec, lane)
    adj = f.get("dwt_interval_adjacency") if isinstance(f.get("dwt_interval_adjacency"), dict) else {}
    prefix = f"{lane}_forensics_"

    counter_delta = first_int(
        adj.get("counter_delta_ticks"),
        f.get("dwt_interval_counter_delta_ticks"),
        frag.get(prefix + "dwt_interval_counter_delta_ticks"),
        root.get(prefix + "dwt_interval_counter_delta_ticks"),
    )

    expected = first_int(
        adj.get("expected_counter_delta_ticks"),
        f.get("dwt_interval_expected_counter_delta_ticks"),
        frag.get(prefix + "dwt_interval_expected_counter_delta_ticks"),
        root.get(prefix + "dwt_interval_expected_counter_delta_ticks"),
    )

    return {
        "valid": first_bool(
            adj.get("valid"),
            f.get("dwt_interval_adjacency_gate_valid"),
            frag.get(prefix + "dwt_interval_adjacency_gate_valid"),
            root.get(prefix + "dwt_interval_adjacency_gate_valid"),
        ),
        "ok": first_bool(
            adj.get("ok"),
            f.get("dwt_interval_adjacency_ok"),
            frag.get(prefix + "dwt_interval_adjacency_ok"),
            root.get(prefix + "dwt_interval_adjacency_ok"),
        ),
        "rejected": first_bool(
            adj.get("rejected"),
            f.get("dwt_interval_adjacency_rejected"),
            frag.get(prefix + "dwt_interval_adjacency_rejected"),
            root.get(prefix + "dwt_interval_adjacency_rejected"),
        ),
        "counter_delta": counter_delta,
        "expected": expected,
        "delta_error": (
            counter_delta - expected
            if counter_delta is not None and expected is not None
            else None
        ),
        "reject_count": first_int(
            adj.get("reject_count"),
            f.get("dwt_interval_adjacency_reject_count"),
            frag.get(prefix + "dwt_interval_adjacency_reject_count"),
            root.get(prefix + "dwt_interval_adjacency_reject_count"),
        ),
    }


def service_diag(rec: Dict[str, Any], lane: str) -> Dict[str, Optional[int]]:
    root = root_of(rec)
    frag = frag_of(rec)
    service = lane_service_obj(rec, lane)
    prefix = f"{lane}_forensics_"

    return {
        "class": first_int(
            service.get("class"),
            service.get("service_class"),
            frag.get(prefix + "service_class"),
            root.get(prefix + "service_class"),
        ),
        "offset_ticks": first_int(
            service.get("offset_ticks"),
            service.get("offset_signed_ticks"),
            frag.get(prefix + "service_offset_ticks"),
            root.get(prefix + "service_offset_ticks"),
        ),
        "arm_to_isr_ticks": first_int(
            service.get("arm_to_isr_ticks"),
            frag.get(prefix + "arm_to_isr_ticks"),
            root.get(prefix + "arm_to_isr_ticks"),
        ),
        "last_counter_delta_ticks": first_int(
            service.get("last_counter_delta_ticks"),
            frag.get(prefix + "last_counter_delta_ticks"),
            root.get(prefix + "last_counter_delta_ticks"),
        ),
    }


def lane_counter_delta(rec: Dict[str, Any], lane: str) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    f = lane_forensics_obj(rec, lane)
    service = service_diag(rec, lane)
    prefix = f"{lane}_forensics_"

    return first_int(
        f.get("counter32_delta_since_previous_event"),
        frag.get(prefix + "counter32_delta_since_previous_event"),
        root.get(prefix + "counter32_delta_since_previous_event"),
        frag.get(f"{lane}_counter32_delta_since_previous_event"),
        root.get(f"{lane}_counter32_delta_since_previous_event"),
        service.get("last_counter_delta_ticks"),
    )


def lane_public_ns(rec: Dict[str, Any], lane: str) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    if lane == "vclock":
        return first_int(
            nested_get(frag, "vclock", "ns"),
            nested_get(forensic, "vclock", "ns"),
            nested_get(root, "vclock", "ns"),
        )

    return first_int(
        nested_get(frag, lane, "ns"),
        nested_get(forensic, lane, "ns"),
        nested_get(root, lane, "ns"),
    )


def lane_measured_gnss_ns(rec: Dict[str, Any], lane: str) -> Optional[int]:
    if lane == "vclock":
        return None

    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    return first_int(
        nested_get(frag, lane, "measured_gnss_ns"),
        nested_get(forensic, lane, "measured_gnss_ns"),
        nested_get(root, lane, "measured_gnss_ns"),
    )


def lane_pps_residual_ns(rec: Dict[str, Any], lane: str) -> Optional[int]:
    if lane == "vclock":
        return None

    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    return first_int(
        nested_get(frag, lane, "pps_residual", "fast_residual_ns"),
        nested_get(forensic, lane, "pps_residual", "fast_residual_ns"),
        nested_get(root, lane, "pps_residual", "fast_residual_ns"),
    )


def lane_ns_source_id(rec: Dict[str, Any], lane: str) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    return first_int(
        nested_get(frag, lane, "ns_source_id"),
        nested_get(forensic, lane, "ns_source_id"),
        nested_get(root, lane, "ns_source_id"),
    )


def lane_projection_source(rec: Dict[str, Any], lane: str) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    return first_int(
        nested_get(frag, lane, "pps_projection_source"),
        nested_get(forensic, lane, "pps_projection_source"),
        nested_get(root, lane, "pps_projection_source"),
    )


def lane_projection_valid(rec: Dict[str, Any], lane: str) -> Optional[bool]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    return first_bool(
        nested_get(frag, lane, "pps_projected_valid"),
        nested_get(forensic, lane, "pps_projected_valid"),
        nested_get(root, lane, "pps_projected_valid"),
    )


def lane_projection_raw_ns(rec: Dict[str, Any], lane: str) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    return first_int(
        nested_get(frag, lane, "pps_projected_raw_ns"),
        nested_get(forensic, lane, "pps_projected_raw_ns"),
        nested_get(root, lane, "pps_projected_raw_ns"),
    )


def lane_stats_obj(rec: Dict[str, Any], lane: str) -> Dict[str, Any]:
    root = root_of(rec)
    frag = frag_of(rec)

    candidates = (
        nested_get(frag, "stats", lane),
        nested_get(root, "stats", lane),
        nested_get(root, "fragment", "stats", lane),
    )

    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def stat_ppb(rec: Dict[str, Any], lane: str) -> Optional[float]:
    return as_float(lane_stats_obj(rec, lane).get("ppb"))


def stat_tau(rec: Dict[str, Any], lane: str) -> Optional[float]:
    return as_float(lane_stats_obj(rec, lane).get("tau"))


def stat_welford(rec: Dict[str, Any], lane: str, key: str) -> Optional[float]:
    w = lane_stats_obj(rec, lane).get("welford")
    if not isinstance(w, dict):
        return None
    return as_float(w.get(key))


def stat_welford_n(rec: Dict[str, Any], lane: str) -> Optional[int]:
    w = lane_stats_obj(rec, lane).get("welford")
    if not isinstance(w, dict):
        return None
    return as_int(w.get("n"))


# ══════════════════════════════════════════════════════════════════════════════
# Row model
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class LaneRow:
    index: int
    pps: Optional[int]
    lane: str
    segment: int

    gnss_ns: Optional[int]

    public_ns: Optional[int]
    public_interval: Optional[int]
    public_residual: Optional[int]

    measured_ns: Optional[int]
    measured_interval: Optional[int]
    measured_residual: Optional[int]

    measured_minus_public: Optional[int]
    measured_minus_public_delta: Optional[int]

    pps_residual_ns: Optional[int]

    ns_source_id: Optional[int]
    projection_source: Optional[int]
    projection_valid: Optional[bool]
    projection_raw_ns: Optional[int]

    counter_delta: Optional[int]
    adjacency_ok: Optional[bool]
    adjacency_delta_error: Optional[int]
    adjacency_reject_count: Optional[int]

    gate_observed: Optional[int]
    gate_prediction: Optional[int]
    gate_effective: Optional[int]
    gate_residual: Optional[int]
    gate_accepted: Optional[bool]
    gate_rejected: Optional[bool]

    pub_actual_cycles: Optional[int]
    pub_prediction_cycles: Optional[int]
    pub_residual_cycles: Optional[int]
    pub_minus_gate_effective: Optional[int]

    service_class: Optional[int]
    service_offset_ticks: Optional[int]
    arm_to_isr_ticks: Optional[int]

    dwt_original: Optional[int]
    dwt_predicted: Optional[int]
    dwt_used: Optional[int]
    dwt_isr_entry: Optional[int]
    dwt_synthetic: Optional[bool]
    dwt_synthetic_error: Optional[int]

    ppb: Optional[float]
    tau: Optional[float]
    w_mean: Optional[float]
    w_min: Optional[float]
    w_max: Optional[float]
    w_stddev: Optional[float]
    w_stderr: Optional[float]
    w_n: Optional[int]


@dataclass
class Event:
    lane: str
    pps: Optional[int]
    index: int
    segment: int
    severity: str
    reason: str
    detail: str


# ══════════════════════════════════════════════════════════════════════════════
# Analysis
# ══════════════════════════════════════════════════════════════════════════════

def is_segment_break(prev: Dict[str, Any], cur: Dict[str, Any], ns_threshold: int) -> Tuple[bool, str]:
    prev_pps = get_pps(prev)
    cur_pps = get_pps(cur)
    prev_gnss = get_gnss_ns(prev)
    cur_gnss = get_gnss_ns(cur)

    reasons: List[str] = []

    if prev_pps is not None and cur_pps is not None and cur_pps != prev_pps + 1:
        reasons.append(f"pps_gap {prev_pps:,}->{cur_pps:,}")

    if prev_gnss is not None and cur_gnss is not None:
        d = cur_gnss - prev_gnss
        if abs(d - NSEC) > ns_threshold:
            reasons.append(f"gnss_interval={d:+,}ns")

    return bool(reasons), "; ".join(reasons)


def build_lane_rows(
    rows: Sequence[Dict[str, Any]],
    lane: str,
    *,
    ns_threshold: int,
) -> Tuple[List[LaneRow], List[Event]]:
    out: List[LaneRow] = []
    events: List[Event] = []

    segment = 0
    prev_global: Optional[Dict[str, Any]] = None
    prev_lane: Optional[LaneRow] = None

    for index, rec in enumerate(rows):
        pps = get_pps(rec)
        gnss_ns = get_gnss_ns(rec)

        if prev_global is not None:
            broke, reason = is_segment_break(prev_global, rec, ns_threshold)
            if broke:
                segment += 1
                prev_lane = None
                events.append(Event(
                    lane="root",
                    pps=pps,
                    index=index,
                    segment=segment,
                    severity="SEGMENT",
                    reason="SEGMENT_BREAK",
                    detail=reason,
                ))

        gate = gate_diag(rec, lane)
        adj = adjacency_diag(rec, lane)
        svc = service_diag(rec, lane)

        public_ns = lane_public_ns(rec, lane)
        measured_ns = lane_measured_gnss_ns(rec, lane)

        public_interval = None
        public_residual = None
        measured_interval = None
        measured_residual = None
        measured_minus_public = None
        measured_minus_public_delta = None

        if measured_ns is not None and public_ns is not None:
            measured_minus_public = measured_ns - public_ns

        if prev_lane is not None:
            if public_ns is not None and prev_lane.public_ns is not None:
                public_interval = public_ns - prev_lane.public_ns
                public_residual = public_interval - NSEC

            if measured_ns is not None and prev_lane.measured_ns is not None:
                measured_interval = measured_ns - prev_lane.measured_ns
                measured_residual = measured_interval - NSEC

            if measured_minus_public is not None and prev_lane.measured_minus_public is not None:
                measured_minus_public_delta = measured_minus_public - prev_lane.measured_minus_public

        pub_actual, pub_pred, pub_res = lane_prediction(rec, lane)

        gate_effective = gate["effective_cycles"]
        pub_minus_gate_effective = None
        if pub_actual is not None and gate_effective is not None:
            pub_minus_gate_effective = pub_actual - gate_effective

        row = LaneRow(
            index=index,
            pps=pps,
            lane=lane,
            segment=segment,
            gnss_ns=gnss_ns,
            public_ns=public_ns,
            public_interval=public_interval,
            public_residual=public_residual,
            measured_ns=measured_ns,
            measured_interval=measured_interval,
            measured_residual=measured_residual,
            measured_minus_public=measured_minus_public,
            measured_minus_public_delta=measured_minus_public_delta,
            pps_residual_ns=lane_pps_residual_ns(rec, lane),
            ns_source_id=lane_ns_source_id(rec, lane),
            projection_source=lane_projection_source(rec, lane),
            projection_valid=lane_projection_valid(rec, lane),
            projection_raw_ns=lane_projection_raw_ns(rec, lane),
            counter_delta=lane_counter_delta(rec, lane),
            adjacency_ok=adj["ok"],
            adjacency_delta_error=adj["delta_error"],
            adjacency_reject_count=adj["reject_count"],
            gate_observed=gate["observed_cycles"],
            gate_prediction=gate["prediction_cycles"],
            gate_effective=gate["effective_cycles"],
            gate_residual=(
                gate["observed_cycles"] - gate["prediction_cycles"]
                if gate["observed_cycles"] is not None and gate["prediction_cycles"] is not None
                else gate["residual_cycles"]
            ),
            gate_accepted=gate["accepted"],
            gate_rejected=gate["rejected"],
            pub_actual_cycles=pub_actual,
            pub_prediction_cycles=pub_pred,
            pub_residual_cycles=pub_res,
            pub_minus_gate_effective=pub_minus_gate_effective,
            service_class=svc["class"],
            service_offset_ticks=svc["offset_ticks"],
            arm_to_isr_ticks=svc["arm_to_isr_ticks"],
            dwt_original=gate["original_dwt"],
            dwt_predicted=gate["predicted_dwt"],
            dwt_used=gate["used_dwt"],
            dwt_isr_entry=gate["isr_entry_dwt"],
            dwt_synthetic=gate["synthetic"],
            dwt_synthetic_error=gate["synthetic_error"],
            ppb=stat_ppb(rec, lane),
            tau=stat_tau(rec, lane),
            w_mean=stat_welford(rec, lane, "mean"),
            w_min=stat_welford(rec, lane, "min"),
            w_max=stat_welford(rec, lane, "max"),
            w_stddev=stat_welford(rec, lane, "stddev"),
            w_stderr=stat_welford(rec, lane, "stderr"),
            w_n=stat_welford_n(rec, lane),
        )

        out.append(row)

        prev_lane = row
        prev_global = rec

    return out, events


def detect_events(
    lane_rows: Sequence[LaneRow],
    *,
    public_ns_threshold: int,
    measured_public_delta_threshold: int,
    stats_ppb_threshold: float,
    stats_welford_mean_threshold: float,
    stats_welford_extreme_threshold: float,
    stats_welford_stddev_threshold: float,
    bootstrap_n: int,
) -> List[Event]:
    events: List[Event] = []
    prev: Optional[LaneRow] = None

    for row in lane_rows:
        if prev is None or row.segment != prev.segment:
            prev = row
            continue

        if row.public_residual is not None and abs(row.public_residual) > public_ns_threshold:
            events.append(Event(
                lane=row.lane,
                pps=row.pps,
                index=row.index,
                segment=row.segment,
                severity="FAIL",
                reason="PUBLIC_NS_INTERVAL",
                detail=f"public_interval={fmt_i(row.public_interval)} ns residual={fmt_si(row.public_residual)} ns",
            ))

        if (
            row.public_residual is not None
            and row.pps_residual_ns is not None
            and abs(row.public_residual - row.pps_residual_ns) > public_ns_threshold
        ):
            events.append(Event(
                lane=row.lane,
                pps=row.pps,
                index=row.index,
                segment=row.segment,
                severity="FAIL",
                reason="PUBLIC_NS_VS_PPS_RESIDUAL",
                detail=f"public_res={fmt_si(row.public_residual)} ns pps_res={fmt_si(row.pps_residual_ns)} ns",
            ))

        if (
            row.measured_minus_public_delta is not None
            and abs(row.measured_minus_public_delta) > measured_public_delta_threshold
        ):
            events.append(Event(
                lane=row.lane,
                pps=row.pps,
                index=row.index,
                segment=row.segment,
                severity="WARN",
                reason="MEASURED_MINUS_PUBLIC_JUMP",
                detail=f"delta={fmt_si(row.measured_minus_public_delta)} ns now={fmt_si(row.measured_minus_public)} ns",
            ))

        mature = (
            prev.w_n is not None
            and row.w_n is not None
            and prev.w_n >= bootstrap_n
            and row.w_n >= bootstrap_n
        )

        if mature:
            if row.ppb is not None and prev.ppb is not None:
                d = row.ppb - prev.ppb
                if abs(d) > stats_ppb_threshold:
                    events.append(Event(
                        lane=row.lane,
                        pps=row.pps,
                        index=row.index,
                        segment=row.segment,
                        severity="WARN",
                        reason="STAT_PPB_JUMP",
                        detail=f"{fmt_f(prev.ppb)} -> {fmt_f(row.ppb)} delta={fmt_f(d)}",
                    ))

            if row.tau is not None and prev.tau is not None:
                d_ppb = (row.tau - prev.tau) * 1_000_000_000.0
                if abs(d_ppb) > stats_ppb_threshold:
                    events.append(Event(
                        lane=row.lane,
                        pps=row.pps,
                        index=row.index,
                        segment=row.segment,
                        severity="WARN",
                        reason="STAT_TAU_JUMP",
                        detail=f"{fmt_tau(prev.tau)} -> {fmt_tau(row.tau)} delta_ppb_equiv={fmt_f(d_ppb)}",
                    ))

            if row.w_mean is not None and prev.w_mean is not None:
                d = row.w_mean - prev.w_mean
                if abs(d) > stats_welford_mean_threshold:
                    events.append(Event(
                        lane=row.lane,
                        pps=row.pps,
                        index=row.index,
                        segment=row.segment,
                        severity="WARN",
                        reason="WELFORD_MEAN_JUMP",
                        detail=f"{fmt_f(prev.w_mean)} -> {fmt_f(row.w_mean)} delta={fmt_f(d)}",
                    ))

            if row.w_min is not None and prev.w_min is not None:
                d = row.w_min - prev.w_min
                if abs(d) > stats_welford_extreme_threshold:
                    events.append(Event(
                        lane=row.lane,
                        pps=row.pps,
                        index=row.index,
                        segment=row.segment,
                        severity="WARN",
                        reason="WELFORD_MIN_JUMP",
                        detail=f"{fmt_f(prev.w_min)} -> {fmt_f(row.w_min)} delta={fmt_f(d)}",
                    ))

            if row.w_max is not None and prev.w_max is not None:
                d = row.w_max - prev.w_max
                if abs(d) > stats_welford_extreme_threshold:
                    events.append(Event(
                        lane=row.lane,
                        pps=row.pps,
                        index=row.index,
                        segment=row.segment,
                        severity="WARN",
                        reason="WELFORD_MAX_JUMP",
                        detail=f"{fmt_f(prev.w_max)} -> {fmt_f(row.w_max)} delta={fmt_f(d)}",
                    ))

            if row.w_stddev is not None and prev.w_stddev is not None:
                d = row.w_stddev - prev.w_stddev
                if abs(d) > stats_welford_stddev_threshold:
                    events.append(Event(
                        lane=row.lane,
                        pps=row.pps,
                        index=row.index,
                        segment=row.segment,
                        severity="WARN",
                        reason="WELFORD_STDDEV_JUMP",
                        detail=f"{fmt_f(prev.w_stddev)} -> {fmt_f(row.w_stddev)} delta={fmt_f(d)}",
                    ))

        prev = row

    return events


# ══════════════════════════════════════════════════════════════════════════════
# Printing
# ══════════════════════════════════════════════════════════════════════════════

def print_header(title: str) -> None:
    print()
    print(title)
    print("═" * len(title))


def print_event_table(events: Sequence[Event], show: int) -> None:
    if not events:
        print("  none")
        return

    print(f"{'pps':>8} {'seg':>4} {'lane':>7} {'sev':>6} {'reason':<32} detail")
    print("─" * 120)
    for ev in events[:show]:
        print(
            f"{fmt_i(ev.pps):>8} "
            f"{ev.segment:>4} "
            f"{ev.lane:>7} "
            f"{ev.severity:>6} "
            f"{ev.reason:<32} "
            f"{ev.detail}"
        )

    if len(events) > show:
        print(f"... {len(events) - show:,} more suppressed")


def print_lane_summary(lane: str, rows: Sequence[LaneRow], events: Sequence[Event]) -> None:
    counter_bad = [r for r in rows if r.counter_delta is not None and r.lane.startswith("ocxo") and r.counter_delta != EXPECTED_OCXO_COUNTER_DELTA]
    adjacency_bad = [r for r in rows if r.adjacency_delta_error not in (None, 0)]
    public_bad = [e for e in events if e.lane == lane and e.reason == "PUBLIC_NS_INTERVAL"]
    stats_bad = [e for e in events if e.lane == lane and (e.reason.startswith("STAT_") or e.reason.startswith("WELFORD_"))]

    print(f"  {lane.upper():<7} counter_lineage={ 'CHECK' if counter_bad else 'CLEAN' } "
          f"adjacency={ 'CHECK' if adjacency_bad else 'CLEAN' } "
          f"public_ns={ 'CHECK' if public_bad else 'CLEAN' } "
          f"stats={ 'CHECK' if stats_bad else 'CLEAN' }")


def print_context_row(row: LaneRow, mark: str = " ") -> None:
    print(
        f"{mark} "
        f"pps={fmt_i(row.pps):>8} "
        f"seg={row.segment:<2} "
        f"lane={row.lane:<6} "
        f"pub_ns={fmt_i(row.public_ns):>20} "
        f"pub_res={fmt_si(row.public_residual):>22} "
        f"meas_ns={fmt_i(row.measured_ns):>20} "
        f"meas_res={fmt_si(row.measured_residual):>10} "
        f"m-p={fmt_si(row.measured_minus_public):>18} "
        f"m-pΔ={fmt_si(row.measured_minus_public_delta):>18}"
    )


def print_deep_row(row: LaneRow) -> None:
    print("    ledger/projection:")
    print(f"      public_ns              = {fmt_i(row.public_ns)}")
    print(f"      public_interval        = {fmt_i(row.public_interval)}")
    print(f"      public_residual        = {fmt_si(row.public_residual)} ns")
    print(f"      measured_gnss_ns       = {fmt_i(row.measured_ns)}")
    print(f"      measured_interval      = {fmt_i(row.measured_interval)}")
    print(f"      measured_residual      = {fmt_si(row.measured_residual)} ns")
    print(f"      measured_minus_public  = {fmt_si(row.measured_minus_public)} ns")
    print(f"      measured_minus_pub Δ   = {fmt_si(row.measured_minus_public_delta)} ns")
    print(f"      pps_residual_ns        = {fmt_si(row.pps_residual_ns)} ns")
    print(f"      ns_source_id           = {fmt_i(row.ns_source_id)}")
    print(f"      projection_source      = {fmt_i(row.projection_source)}")
    print(f"      projection_valid       = {row.projection_valid}")
    print(f"      projection_raw_ns      = {fmt_i(row.projection_raw_ns)}")

    print("    capture/counter/gate:")
    print(f"      counter_delta          = {fmt_i(row.counter_delta)}")
    print(f"      adjacency_ok           = {row.adjacency_ok}")
    print(f"      adjacency_delta_error  = {fmt_si(row.adjacency_delta_error)}")
    print(f"      adjacency_reject_count = {fmt_i(row.adjacency_reject_count)}")
    print(f"      gate_observed          = {fmt_i(row.gate_observed)}")
    print(f"      gate_prediction        = {fmt_i(row.gate_prediction)}")
    print(f"      gate_effective         = {fmt_i(row.gate_effective)}")
    print(f"      gate_residual          = {fmt_si(row.gate_residual)} cycles")
    print(f"      gate_accepted/rejected = {row.gate_accepted}/{row.gate_rejected}")
    print(f"      pub_actual_cycles      = {fmt_i(row.pub_actual_cycles)}")
    print(f"      pub_pred_cycles        = {fmt_i(row.pub_prediction_cycles)}")
    print(f"      pub_res_cycles         = {fmt_si(row.pub_residual_cycles)}")
    print(f"      pub-gate-effective     = {fmt_si(row.pub_minus_gate_effective)} cycles")

    print("    service/dwt:")
    print(f"      service_class          = {fmt_i(row.service_class)}")
    print(f"      service_offset_ticks   = {fmt_si(row.service_offset_ticks)}")
    print(f"      arm_to_isr_ticks       = {fmt_i(row.arm_to_isr_ticks)}")
    print(f"      dwt_original           = {fmt_i(row.dwt_original)}")
    print(f"      dwt_predicted          = {fmt_i(row.dwt_predicted)}")
    print(f"      dwt_used               = {fmt_i(row.dwt_used)}")
    print(f"      dwt_isr_entry          = {fmt_i(row.dwt_isr_entry)}")
    print(f"      dwt_synthetic          = {row.dwt_synthetic}")
    print(f"      dwt_synth_error        = {fmt_si(row.dwt_synthetic_error)} cycles")

    print("    stats:")
    print(f"      ppb                    = {fmt_f(row.ppb)}")
    print(f"      tau                    = {fmt_tau(row.tau)}")
    print(f"      welford.mean           = {fmt_f(row.w_mean)}")
    print(f"      welford.min            = {fmt_f(row.w_min)}")
    print(f"      welford.max            = {fmt_f(row.w_max)}")
    print(f"      welford.stddev         = {fmt_f(row.w_stddev)}")
    print(f"      welford.stderr         = {fmt_f(row.w_stderr)}")
    print(f"      welford.n              = {fmt_i(row.w_n)}")


def print_crime_scene(
    lane: str,
    rows: Sequence[LaneRow],
    events: Sequence[Event],
    *,
    window: int,
    show_scenes: int,
) -> None:
    trigger_indices = []
    for ev in events:
        if ev.lane != lane:
            continue
        if ev.reason in (
            "PUBLIC_NS_INTERVAL",
            "PUBLIC_NS_VS_PPS_RESIDUAL",
            "MEASURED_MINUS_PUBLIC_JUMP",
            "WELFORD_MAX_JUMP",
            "WELFORD_STDDEV_JUMP",
            "STAT_PPB_JUMP",
            "STAT_TAU_JUMP",
        ):
            if ev.index not in trigger_indices:
                trigger_indices.append(ev.index)

    if not trigger_indices:
        print("  none")
        return

    shown = 0
    for idx in trigger_indices:
        if shown >= show_scenes:
            print(f"  ... {len(trigger_indices) - shown:,} additional scene(s) suppressed")
            break

        pps = rows[idx].pps if 0 <= idx < len(rows) else None
        print_header(f"Crime scene: {lane.upper()} around PPS {fmt_i(pps)} / row index {idx}")

        start = max(0, idx - window)
        end = min(len(rows), idx + window + 1)

        for i in range(start, end):
            mark = ">" if i == idx else " "
            print_context_row(rows[i], mark=mark)

        print()
        print("  Deep row detail")
        print("  ───────────────")
        print_deep_row(rows[idx])

        shown += 1


def print_report(
    *,
    campaign: str,
    lane_rows_by_lane: Dict[str, List[LaneRow]],
    events: List[Event],
    segment_events: List[Event],
    show: int,
    window: int,
    show_scenes: int,
) -> None:
    print(f"timebase_public_ns_laser {REPORT_VERSION}")
    print(f"Campaign: {campaign}")

    print_header("Verdict")
    for lane, rows in lane_rows_by_lane.items():
        lane_events = [e for e in events if e.lane == lane]
        print_lane_summary(lane, rows, lane_events)

    public_fail_count = sum(1 for e in events if e.reason == "PUBLIC_NS_INTERVAL")
    stats_fail_count = sum(1 for e in events if e.reason.startswith("STAT_") or e.reason.startswith("WELFORD_"))

    print(f"  segment breaks:        {len(segment_events)}")
    print(f"  public-ns failures:    {public_fail_count}")
    print(f"  stats-corruption hits: {stats_fail_count}")

    print_header("Segment breaks")
    print_event_table(segment_events, show=show)

    print_header("Laser hits")
    print_event_table(events, show=show)

    for lane, rows in lane_rows_by_lane.items():
        lane_events = [e for e in events if e.lane == lane]
        print_crime_scene(
            lane,
            rows,
            lane_events,
            window=window,
            show_scenes=show_scenes,
        )

    print_header("Interpretation")
    print("  • PUBLIC_NS_INTERVAL means the public ns ledger jumped inside a continuous segment.")
    print("  • If counter/adja­cency is clean while PUBLIC_NS_INTERVAL fires, the edge/counter lineage is innocent.")
    print("  • measured_gnss_ns staying sane while public_ns jumps implicates projection/public-ledger construction.")
    print("  • WELFORD_* and STAT_* after a public-ns jump are likely victims, not the root cause.")
    print("  • The crime scene section prints ±window rows plus deep projection/DWT/service/stat context.")


# ══════════════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════════════

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Laser-focused TIMEBASE public-ns projection pathology hunter."
    )

    p.add_argument("campaign", help="Campaign name, e.g. IRQ3")
    p.add_argument("--lane", default="all", help="OCXO1, OCXO2, VCLOCK, or all")
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--show", type=int, default=200)
    p.add_argument("--window", type=int, default=5, help="Rows before/after each hit")
    p.add_argument("--show-scenes", type=int, default=8)

    p.add_argument("--ns-threshold", type=int, default=10_000)
    p.add_argument("--measured-public-delta-threshold", type=int, default=10_000)
    p.add_argument("--stat-ppb-threshold", type=float, default=25.0)
    p.add_argument("--stat-welford-mean-threshold", type=float, default=25.0)
    p.add_argument("--stat-welford-extreme-threshold", type=float, default=500.0)
    p.add_argument("--stat-welford-stddev-threshold", type=float, default=25.0)
    p.add_argument("--bootstrap-n", type=int, default=10)

    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    print(f"timebase_public_ns_laser {REPORT_VERSION} entered")

    args = parse_args(argv)

    lanes = resolve_lanes(args.lane)

    rows = fetch_timebase(args.campaign)
    if args.limit:
        rows = rows[:args.limit]

    if not rows:
        print(f"No TIMEBASE rows found for campaign {args.campaign!r}")
        return 2

    rows = sorted(
        rows,
        key=lambda r: (
            get_pps(r) if get_pps(r) is not None else 10**18,
            get_gnss_ns(r) if get_gnss_ns(r) is not None else 10**18,
        ),
    )

    lane_rows_by_lane: Dict[str, List[LaneRow]] = {}
    all_events: List[Event] = []
    segment_events: List[Event] = []

    for lane in lanes:
        lane_rows, seg_events = build_lane_rows(
            rows,
            lane,
            ns_threshold=args.ns_threshold,
        )

        events = detect_events(
            lane_rows,
            public_ns_threshold=args.ns_threshold,
            measured_public_delta_threshold=args.measured_public_delta_threshold,
            stats_ppb_threshold=args.stat_ppb_threshold,
            stats_welford_mean_threshold=args.stat_welford_mean_threshold,
            stats_welford_extreme_threshold=args.stat_welford_extreme_threshold,
            stats_welford_stddev_threshold=args.stat_welford_stddev_threshold,
            bootstrap_n=args.bootstrap_n,
        )

        lane_rows_by_lane[lane] = lane_rows
        all_events.extend(events)

        # Segment events are identical per lane, so keep only the first lane's segment list.
        if not segment_events:
            segment_events = seg_events

    all_events = sorted(all_events, key=lambda e: (e.index, e.lane, e.reason))

    print_report(
        campaign=args.campaign,
        lane_rows_by_lane=lane_rows_by_lane,
        events=all_events,
        segment_events=segment_events,
        show=args.show,
        window=args.window,
        show_scenes=args.show_scenes,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))