#!/usr/bin/env python3
"""
TIMEBASE Forensic Sanity — segment-aware timing + statistics audit.

Native to the same local ZPNet test environment used by raw_cycles.py.

Reads:
    SELECT payload
    FROM timebase
    WHERE campaign = %s

using:
    from zpnet.shared.db import open_db

Doctrine
--------
A clock can have clean counter/public-ledger lineage while its accumulated
statistics become poisoned. This report audits both:

    1. Counter/adjaency lineage
    2. Gate/published interval lineage
    3. Public nanosecond ledger lineage
    4. PPB / TAU / Welford statistical state continuity

Important:
    Do not compare row-to-row ledgers or statistics across a segment break.

A segment break is declared when:
    pps_count is not previous + 1
    OR GNSS ns interval is not approximately +1 second
    OR GNSS/PPS epoch offset jumps
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


REPORT_VERSION = "2026-06-06-open-db-stats-v5"

NSEC = 1_000_000_000
OCXO_EXPECTED_COUNTER_TICKS = 10_000_000


# ══════════════════════════════════════════════════════════════════════════════
# DB loader — same local doctrine as raw_cycles.py
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


def fmt_num(v: Any) -> str:
    if v is None:
        return "None"
    if isinstance(v, int):
        return f"{v:,}"
    if isinstance(v, float):
        if math.isfinite(v):
            return f"{v:,.6f}" if abs(v) >= 1000 else f"{v:+.6f}"
        return str(v)
    return str(v)


def fmt_delta(v: Any, unit: str = "") -> str:
    if v is None:
        return "None"
    try:
        f = float(v)
    except Exception:
        return str(v)
    if abs(f - round(f)) < 1e-9:
        s = f"{int(round(f)):+,}"
    else:
        s = f"{f:+,.6f}"
    return f"{s} {unit}".rstrip()


def norm_lane(lane: str) -> str:
    lane = lane.strip().lower()
    if lane in ("all", "*"):
        return "all"
    if lane in ("1", "o1", "ocxo1"):
        return "ocxo1"
    if lane in ("2", "o2", "ocxo2"):
        return "ocxo2"
    if lane in ("v", "vc", "vclk", "vclock"):
        return "vclock"
    if lane == "dwt":
        return "dwt"
    raise ValueError(f"unknown lane '{lane}', expected OCXO1, OCXO2, VCLOCK, DWT, or all")


def resolve_lanes(lane_arg: str) -> List[str]:
    lane = norm_lane(lane_arg)
    if lane == "all":
        return ["ocxo1", "ocxo2", "vclock"]
    return [lane]


def stable_stats(values: Sequence[float]) -> Optional[Tuple[int, float, float, float, float]]:
    vals = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not vals:
        return None
    n = len(vals)
    mean = statistics.fmean(vals)
    sd = statistics.pstdev(vals) if n > 1 else 0.0
    return n, mean, sd, min(vals), max(vals)


# ══════════════════════════════════════════════════════════════════════════════
# TIMEBASE schema helpers
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


def prediction_of(rec: Dict[str, Any]) -> Dict[str, Any]:
    frag = frag_of(rec)
    pred = frag.get("prediction")
    return pred if isinstance(pred, dict) else {}


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

    static = first_int(
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

    if residual is None and actual is not None and static is not None:
        residual = actual - static

    return actual, static, residual


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
        "reject_count": first_int(
            gate.get("reject_count"),
            f.get("dwt_interval_reject_count"),
            frag.get(prefix + "dwt_interval_reject_count"),
            root.get(prefix + "dwt_interval_reject_count"),
        ),
        "resync_applied": first_bool(
            gate.get("resync_applied"),
            f.get("dwt_interval_resync_applied"),
            frag.get(prefix + "dwt_interval_resync_applied"),
            root.get(prefix + "dwt_interval_resync_applied"),
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
    }


def adjacency_diag(rec: Dict[str, Any], lane: str) -> Dict[str, Any]:
    root = root_of(rec)
    frag = frag_of(rec)
    f = lane_forensics_obj(rec, lane)
    adj = f.get("dwt_interval_adjacency") if isinstance(f.get("dwt_interval_adjacency"), dict) else {}
    prefix = f"{lane}_forensics_"

    valid = first_bool(
        adj.get("valid"),
        f.get("dwt_interval_adjacency_gate_valid"),
        frag.get(prefix + "dwt_interval_adjacency_gate_valid"),
        root.get(prefix + "dwt_interval_adjacency_gate_valid"),
    )

    ok = first_bool(
        adj.get("ok"),
        f.get("dwt_interval_adjacency_ok"),
        frag.get(prefix + "dwt_interval_adjacency_ok"),
        root.get(prefix + "dwt_interval_adjacency_ok"),
    )

    rejected = first_bool(
        adj.get("rejected"),
        f.get("dwt_interval_adjacency_rejected"),
        frag.get(prefix + "dwt_interval_adjacency_rejected"),
        root.get(prefix + "dwt_interval_adjacency_rejected"),
    )

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

    reject_count = first_int(
        adj.get("reject_count"),
        f.get("dwt_interval_adjacency_reject_count"),
        frag.get(prefix + "dwt_interval_adjacency_reject_count"),
        root.get(prefix + "dwt_interval_adjacency_reject_count"),
    )

    delta_error = counter_delta - expected if counter_delta is not None and expected is not None else None

    return {
        "valid": valid,
        "ok": ok,
        "rejected": rejected,
        "counter_delta": counter_delta,
        "expected": expected,
        "delta_error": delta_error,
        "reject_count": reject_count,
    }


def lane_counter_delta(rec: Dict[str, Any], lane: str) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    f = lane_forensics_obj(rec, lane)
    prefix = f"{lane}_forensics_"

    return first_int(
        f.get("counter32_delta_since_previous_event"),
        frag.get(prefix + "counter32_delta_since_previous_event"),
        root.get(prefix + "counter32_delta_since_previous_event"),
        frag.get(f"{lane}_counter32_delta_since_previous_event"),
        root.get(f"{lane}_counter32_delta_since_previous_event"),
    )


def service_diag(rec: Dict[str, Any], lane: str) -> Dict[str, Optional[int]]:
    root = root_of(rec)
    frag = frag_of(rec)
    service = lane_service_obj(rec, lane)
    prefix = f"{lane}_forensics_"

    return {
        "service_class": first_int(
            service.get("class"),
            service.get("service_class"),
            frag.get(prefix + "service_class"),
            root.get(prefix + "service_class"),
        ),
        "service_offset": first_int(
            service.get("offset_ticks"),
            service.get("offset_signed_ticks"),
            frag.get(prefix + "service_offset_ticks"),
            root.get(prefix + "service_offset_ticks"),
        ),
        "arm_to_isr": first_int(
            service.get("arm_to_isr_ticks"),
            frag.get(prefix + "arm_to_isr_ticks"),
            root.get(prefix + "arm_to_isr_ticks"),
        ),
        "last_counter_delta": first_int(
            service.get("last_counter_delta_ticks"),
            frag.get(prefix + "last_counter_delta_ticks"),
            root.get(prefix + "last_counter_delta_ticks"),
        ),
    }


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


def pps_interval_cycles(rec: Dict[str, Any]) -> Optional[int]:
    root = root_of(rec)
    frag = frag_of(rec)
    forensic = forensics_root_of(rec)

    return first_int(
        nested_get(frag, "pps", "dwt_cycles_between_edges"),
        forensic.get("dwt_cycles_between_pps_vclock"),
        root.get("dwt_cycles_between_pps_vclock"),
    )


# ══════════════════════════════════════════════════════════════════════════════
# Stats helpers
# ══════════════════════════════════════════════════════════════════════════════

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


def extra_clock_stats_obj(rec: Dict[str, Any], name: str) -> Dict[str, Any]:
    root = root_of(rec)
    extra = root.get("extra_clocks")
    if not isinstance(extra, dict):
        return {}

    prefix = f"{name}_"
    return {
        "ppb": extra.get(prefix + "ppb"),
        "tau": extra.get(prefix + "tau"),
        "welford": {
            "mean": extra.get(prefix + "welford_mean"),
            "min": extra.get(prefix + "welford_min"),
            "max": extra.get(prefix + "welford_max"),
            "stddev": extra.get(prefix + "welford_stddev"),
            "stderr": extra.get(prefix + "welford_stderr"),
            "n": extra.get(prefix + "welford_n"),
        },
    }


def stat_obj(rec: Dict[str, Any], lane: str) -> Dict[str, Any]:
    if lane == "gnss_raw":
        return extra_clock_stats_obj(rec, "gnss_raw")
    return lane_stats_obj(rec, lane)


def stat_ppb(rec: Dict[str, Any], lane: str) -> Optional[float]:
    return as_float(stat_obj(rec, lane).get("ppb"))


def stat_tau(rec: Dict[str, Any], lane: str) -> Optional[float]:
    return as_float(stat_obj(rec, lane).get("tau"))


def stat_welford(rec: Dict[str, Any], lane: str, key: str) -> Optional[float]:
    w = stat_obj(rec, lane).get("welford")
    if not isinstance(w, dict):
        return None
    return as_float(w.get(key))


def stat_welford_n(rec: Dict[str, Any], lane: str) -> Optional[int]:
    w = stat_obj(rec, lane).get("welford")
    if not isinstance(w, dict):
        return None
    return as_int(w.get("n"))


def tau_to_ppb_delta(delta_tau: float) -> float:
    return delta_tau * 1_000_000_000.0


# ══════════════════════════════════════════════════════════════════════════════
# Data structures
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class Issue:
    pps: Optional[int]
    lane: str
    severity: str
    reason: str
    detail: str


@dataclass
class Segment:
    index: int
    start_pps: Optional[int]
    end_pps: Optional[int]
    rows: int
    gnss_pps_offset_ns: Optional[int]


@dataclass
class SyntheticRun:
    lane: str
    start_pps: Optional[int]
    end_pps: Optional[int]
    rows: int
    min_error: Optional[int]
    max_error: Optional[int]
    start_error: Optional[int]
    end_error: Optional[int]
    max_step: Optional[int]


# ══════════════════════════════════════════════════════════════════════════════
# Analysis
# ══════════════════════════════════════════════════════════════════════════════

class SanityReport:
    def __init__(
        self,
        *,
        cycle_threshold: int,
        ns_threshold: int,
        gate_row_warnings: bool,
        published_tolerance_cycles: int,
        gnss_offset_tolerance_ns: int,
        synthetic_warn_cycles: int,
        synthetic_fail_cycles: int,
        synthetic_step_warn_cycles: int,
        stat_ppb_jump: float,
        stat_tau_jump_ppb: float,
        stat_welford_mean_jump: float,
        stat_welford_extreme_jump: float,
        stat_welford_stddev_jump: float,
        stat_n_strict: bool,
    ) -> None:
        self.cycle_threshold = cycle_threshold
        self.ns_threshold = ns_threshold
        self.gate_row_warnings = gate_row_warnings
        self.published_tolerance_cycles = published_tolerance_cycles
        self.gnss_offset_tolerance_ns = gnss_offset_tolerance_ns
        self.synthetic_warn_cycles = synthetic_warn_cycles
        self.synthetic_fail_cycles = synthetic_fail_cycles
        self.synthetic_step_warn_cycles = synthetic_step_warn_cycles

        self.stat_ppb_jump = stat_ppb_jump
        self.stat_tau_jump_ppb = stat_tau_jump_ppb
        self.stat_welford_mean_jump = stat_welford_mean_jump
        self.stat_welford_extreme_jump = stat_welford_extreme_jump
        self.stat_welford_stddev_jump = stat_welford_stddev_jump
        self.stat_n_strict = stat_n_strict

        self.issues: List[Issue] = []
        self.series: Dict[str, List[float]] = defaultdict(list)
        self.coverage: Counter[str] = Counter()
        self.segments: List[Segment] = []
        self.synthetic_runs: List[SyntheticRun] = []

    def issue(self, pps: Optional[int], lane: str, severity: str, reason: str, detail: str) -> None:
        self.issues.append(Issue(pps, lane, severity, reason, detail))

    def add_series(self, key: str, value: Any) -> None:
        v = as_float(value)
        if v is not None and math.isfinite(v):
            self.series[key].append(v)

    def analyze(self, rows: Sequence[Dict[str, Any]], lanes: Sequence[str]) -> None:
        rows_sorted = sorted(
            rows,
            key=lambda r: (
                get_pps(r) if get_pps(r) is not None else 10**18,
                get_gnss_ns(r) if get_gnss_ns(r) is not None else 10**18,
            ),
        )

        prev_rec: Optional[Dict[str, Any]] = None
        prev_by_lane: Dict[str, Dict[str, Any]] = {}

        segment_index = 0
        segment_start_pps: Optional[int] = None
        segment_end_pps: Optional[int] = None
        segment_rows = 0
        segment_offset_ns: Optional[int] = None

        synth_state: Dict[str, Optional[Dict[str, Any]]] = {lane: None for lane in lanes}

        def close_segment() -> None:
            if segment_rows > 0:
                self.segments.append(
                    Segment(
                        index=segment_index,
                        start_pps=segment_start_pps,
                        end_pps=segment_end_pps,
                        rows=segment_rows,
                        gnss_pps_offset_ns=segment_offset_ns,
                    )
                )

        for rec in rows_sorted:
            pps = get_pps(rec)
            gnss_ns = get_gnss_ns(rec)
            offset_ns = gnss_ns - pps * NSEC if gnss_ns is not None and pps is not None else None

            segment_break = False
            reasons: List[str] = []

            if prev_rec is not None:
                prev_pps = get_pps(prev_rec)
                prev_gnss_ns = get_gnss_ns(prev_rec)

                if prev_pps is not None and pps is not None and pps != prev_pps + 1:
                    segment_break = True
                    reasons.append(f"pps_count {prev_pps:,} -> {pps:,}")
                    self.issue(pps, "root", "FAIL", "SEGMENT_BREAK_PPS_GAP", reasons[-1])

                if prev_gnss_ns is not None and gnss_ns is not None:
                    gnss_delta = gnss_ns - prev_gnss_ns
                    if abs(gnss_delta - NSEC) > self.ns_threshold:
                        segment_break = True
                        reasons.append(f"gnss interval {gnss_delta:,} ns")
                        self.issue(pps, "root", "FAIL", "SEGMENT_BREAK_GNSS_INTERVAL", reasons[-1])

                if (
                    segment_offset_ns is not None
                    and offset_ns is not None
                    and abs(offset_ns - segment_offset_ns) > self.gnss_offset_tolerance_ns
                ):
                    segment_break = True
                    reasons.append(f"gnss_pps_offset {segment_offset_ns:,} -> {offset_ns:,} ns")
                    self.issue(pps, "root", "WARN", "SEGMENT_BREAK_GNSS_PPS_OFFSET", reasons[-1])

            if segment_break:
                close_segment()
                segment_index += 1
                segment_start_pps = pps
                segment_end_pps = pps
                segment_rows = 0
                segment_offset_ns = offset_ns
                prev_by_lane = {}

                for lane in lanes:
                    self.close_synth_run(lane, synth_state)

                self.issue(pps, "root", "FAIL", "SEGMENT_BREAK", "; ".join(reasons))

            if segment_rows == 0:
                segment_start_pps = pps
                segment_offset_ns = offset_ns

            segment_rows += 1
            segment_end_pps = pps

            if offset_ns is not None:
                self.add_series("gnss_pps_offset_ns", offset_ns)

            pi = pps_interval_cycles(rec)
            if pi is not None:
                self.add_series("pps_interval_cycles", pi)

            for lane in lanes:
                self.analyze_lane(
                    rec=rec,
                    prev_rec=prev_by_lane.get(lane),
                    lane=lane,
                    pps=pps,
                    segment_break=segment_break,
                    synth_state=synth_state,
                )

            if not segment_break:
                for lane in lanes:
                    prev_by_lane[lane] = rec

            prev_rec = rec

        close_segment()

        for lane in lanes:
            self.close_synth_run(lane, synth_state)

    def analyze_lane(
        self,
        *,
        rec: Dict[str, Any],
        prev_rec: Optional[Dict[str, Any]],
        lane: str,
        pps: Optional[int],
        segment_break: bool,
        synth_state: Dict[str, Optional[Dict[str, Any]]],
    ) -> None:
        f = lane_forensics_obj(rec, lane)
        gate = gate_diag(rec, lane)
        adj = adjacency_diag(rec, lane)
        svc = service_diag(rec, lane)

        if f or gate:
            self.coverage[f"{lane}_rows"] += 1

        cdelta = lane_counter_delta(rec, lane)
        if cdelta is None:
            cdelta = svc.get("last_counter_delta")

        if cdelta is not None:
            self.coverage[f"{lane}_counter_delta"] += 1
            self.add_series(f"{lane}_counter_delta_ticks", cdelta)
            if lane.startswith("ocxo") and cdelta != OCXO_EXPECTED_COUNTER_TICKS:
                self.issue(
                    pps,
                    lane,
                    "FAIL",
                    "COUNTER_DELTA",
                    f"counter_delta={cdelta:,}, expected={OCXO_EXPECTED_COUNTER_TICKS:,}",
                )

        if adj.get("valid") is not None:
            self.coverage[f"{lane}_adjacency"] += 1
            if adj.get("valid") is True and adj.get("ok") is not True and adj.get("rejected") is not True:
                self.issue(
                    pps,
                    lane,
                    "FAIL",
                    "ADJACENCY_BAD",
                    f"counter_delta={adj.get('counter_delta')}, expected={adj.get('expected')}",
                )

        if adj.get("delta_error") is not None:
            self.add_series(f"{lane}_adjacency_delta_ticks", adj["delta_error"])

        if adj.get("reject_count") is not None:
            self.add_series(f"{lane}_adjacency_reject_count", adj["reject_count"])

        gate_valid = bool(gate.get("valid"))
        if gate_valid:
            self.coverage[f"{lane}_gate"] += 1

        observed = gate.get("observed_cycles")
        gpred = gate.get("prediction_cycles")
        effective = gate.get("effective_cycles")
        gres = gate.get("residual_cycles")

        if observed is not None and gpred is not None:
            gate_residual = observed - gpred
            self.add_series(f"{lane}_gateR_cycles", gate_residual)
            if self.gate_row_warnings and abs(gate_residual) > self.cycle_threshold:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "GATE_RESIDUAL",
                    f"observed-prediction={gate_residual:+,} cycles",
                )

        if gres is not None:
            self.add_series(f"{lane}_gate_residual_field_cycles", gres)

        if gate.get("rejected") is True or gate.get("accepted") is False:
            self.issue(
                pps,
                lane,
                "FAIL",
                "GATE_REJECTED",
                f"accepted={gate.get('accepted')}, rejected={gate.get('rejected')}",
            )

        pub_actual, pub_pred, pub_res = lane_prediction(rec, lane)

        if pub_actual is None and effective is not None:
            pub_actual = effective

        if pub_actual is None and pub_pred is not None and pub_res is not None:
            pub_actual = pub_pred + pub_res

        if pub_actual is not None:
            self.add_series(f"{lane}_published_actual_cycles", pub_actual)

        if pub_pred is not None:
            self.add_series(f"{lane}_published_prediction_cycles", pub_pred)

        if pub_res is not None:
            self.add_series(f"{lane}_published_residual_cycles", pub_res)

        if pub_actual is not None and effective is not None:
            diff = pub_actual - effective
            self.add_series(f"{lane}_pub_minus_gate_effective_cycles", diff)
            if abs(diff) > self.published_tolerance_cycles:
                self.issue(
                    pps,
                    lane,
                    "FAIL",
                    "PUBLISHED_INTERVAL_MISMATCH",
                    f"pub_actual={pub_actual:,}, gate_effective={effective:,}, diff={diff:+,}",
                )

        public_ns = lane_public_ns(rec, lane)
        measured_ns = lane_measured_gnss_ns(rec, lane)
        pps_res_ns = lane_pps_residual_ns(rec, lane)

        if public_ns is not None:
            self.coverage[f"{lane}_ns"] += 1

        if measured_ns is not None and public_ns is not None:
            self.add_series(f"{lane}_measured_minus_public_ns", measured_ns - public_ns)

        if pps_res_ns is not None:
            self.add_series(f"{lane}_pps_fast_res_ns", pps_res_ns)

        if prev_rec is not None and not segment_break:
            prev_public_ns = lane_public_ns(prev_rec, lane)
            prev_measured_ns = lane_measured_gnss_ns(prev_rec, lane)

            if public_ns is not None and prev_public_ns is not None:
                pub_interval = public_ns - prev_public_ns
                residual = pub_interval - NSEC
                self.add_series(f"{lane}_public_ns_interval_residual_ns", residual)

                if abs(residual) > self.ns_threshold:
                    self.issue(
                        pps,
                        lane,
                        "FAIL",
                        "PUBLIC_NS_INTERVAL",
                        f"pub_interval={pub_interval:,} ns, residual={residual:+,} ns",
                    )

                if pps_res_ns is not None and abs(residual - pps_res_ns) > self.ns_threshold:
                    self.issue(
                        pps,
                        lane,
                        "FAIL",
                        "PUBLIC_NS_VS_PPS_RESIDUAL",
                        f"row_ns_res={residual:+,}, pps_res={pps_res_ns:+,}",
                    )

            if measured_ns is not None and prev_measured_ns is not None:
                measured_interval = measured_ns - prev_measured_ns
                measured_residual = measured_interval - NSEC
                self.add_series(f"{lane}_measured_ns_interval_residual_ns", measured_residual)

                if abs(measured_residual) > self.ns_threshold:
                    self.issue(
                        pps,
                        lane,
                        "WARN",
                        "MEASURED_NS_INTERVAL",
                        f"measured_interval={measured_interval:,} ns, residual={measured_residual:+,} ns",
                    )

        synth_error = gate.get("synthetic_error")
        synth_threshold = gate.get("synthetic_threshold")

        if synth_error is not None:
            self.add_series(f"{lane}_synth_error_cycles", synth_error)
            self.update_synth_run(
                lane=lane,
                pps=pps,
                synth_error=synth_error,
                synth_threshold=synth_threshold,
                synth_state=synth_state,
            )

        if svc.get("service_offset") is not None:
            self.add_series(f"{lane}_service_offset_ticks", svc["service_offset"])
            if abs(svc["service_offset"]) > 2:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "SERVICE_OFFSET",
                    f"offset_ticks={svc['service_offset']:+,}",
                )

        if svc.get("service_class") is not None:
            self.add_series(f"{lane}_service_class", svc["service_class"])

        if svc.get("arm_to_isr") is not None:
            self.add_series(f"{lane}_arm_to_isr_ticks", svc["arm_to_isr"])

        self.analyze_stats_surface(
            rec=rec,
            prev_rec=prev_rec,
            lane=lane,
            pps=pps,
            segment_break=segment_break,
        )

    def analyze_stats_surface(
        self,
        *,
        rec: Dict[str, Any],
        prev_rec: Optional[Dict[str, Any]],
        lane: str,
        pps: Optional[int],
        segment_break: bool,
    ) -> None:
        ppb = stat_ppb(rec, lane)
        tau = stat_tau(rec, lane)
        w_mean = stat_welford(rec, lane, "mean")
        w_min = stat_welford(rec, lane, "min")
        w_max = stat_welford(rec, lane, "max")
        w_stddev = stat_welford(rec, lane, "stddev")
        w_stderr = stat_welford(rec, lane, "stderr")
        w_n = stat_welford_n(rec, lane)

        if ppb is not None:
            self.coverage[f"{lane}_stats_ppb"] += 1
            self.add_series(f"{lane}_stats_ppb", ppb)

        if tau is not None:
            self.coverage[f"{lane}_stats_tau"] += 1
            self.add_series(f"{lane}_stats_tau_minus_1_ppb_equiv", (tau - 1.0) * 1_000_000_000.0)

        if w_mean is not None:
            self.coverage[f"{lane}_welford_mean"] += 1
            self.add_series(f"{lane}_welford_mean", w_mean)

        if w_min is not None:
            self.add_series(f"{lane}_welford_min", w_min)

        if w_max is not None:
            self.add_series(f"{lane}_welford_max", w_max)

        if w_stddev is not None:
            self.add_series(f"{lane}_welford_stddev", w_stddev)

        if w_stderr is not None:
            self.add_series(f"{lane}_welford_stderr", w_stderr)

        if w_n is not None:
            self.add_series(f"{lane}_welford_n", w_n)

        if prev_rec is None or segment_break:
            return

        prev_ppb = stat_ppb(prev_rec, lane)
        prev_tau = stat_tau(prev_rec, lane)
        prev_w_mean = stat_welford(prev_rec, lane, "mean")
        prev_w_min = stat_welford(prev_rec, lane, "min")
        prev_w_max = stat_welford(prev_rec, lane, "max")
        prev_w_stddev = stat_welford(prev_rec, lane, "stddev")
        prev_w_stderr = stat_welford(prev_rec, lane, "stderr")
        prev_w_n = stat_welford_n(prev_rec, lane)

        if ppb is not None and prev_ppb is not None:
            d = ppb - prev_ppb
            self.add_series(f"{lane}_stats_ppb_delta", d)
            if abs(d) > self.stat_ppb_jump:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "STAT_PPB_JUMP",
                    f"ppb {prev_ppb:+.6f} -> {ppb:+.6f}, delta={d:+.6f}",
                )

        if tau is not None and prev_tau is not None:
            d_tau = tau - prev_tau
            d_ppb_equiv = tau_to_ppb_delta(d_tau)
            self.add_series(f"{lane}_stats_tau_delta_ppb_equiv", d_ppb_equiv)
            if abs(d_ppb_equiv) > self.stat_tau_jump_ppb:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "STAT_TAU_JUMP",
                    f"tau {prev_tau:.12f} -> {tau:.12f}, delta_ppb_equiv={d_ppb_equiv:+.6f}",
                )

        if w_mean is not None and prev_w_mean is not None:
            d = w_mean - prev_w_mean
            self.add_series(f"{lane}_welford_mean_delta", d)
            if abs(d) > self.stat_welford_mean_jump:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "WELFORD_MEAN_JUMP",
                    f"mean {prev_w_mean:+.6f} -> {w_mean:+.6f}, delta={d:+.6f}",
                )

        if w_min is not None and prev_w_min is not None:
            d = w_min - prev_w_min
            self.add_series(f"{lane}_welford_min_delta", d)
            if abs(d) > self.stat_welford_extreme_jump:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "WELFORD_MIN_JUMP",
                    f"min {prev_w_min:+.6f} -> {w_min:+.6f}, delta={d:+.6f}",
                )

        if w_max is not None and prev_w_max is not None:
            d = w_max - prev_w_max
            self.add_series(f"{lane}_welford_max_delta", d)
            if abs(d) > self.stat_welford_extreme_jump:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "WELFORD_MAX_JUMP",
                    f"max {prev_w_max:+.6f} -> {w_max:+.6f}, delta={d:+.6f}",
                )

        if w_stddev is not None and prev_w_stddev is not None:
            d = w_stddev - prev_w_stddev
            self.add_series(f"{lane}_welford_stddev_delta", d)
            if abs(d) > self.stat_welford_stddev_jump:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "WELFORD_STDDEV_JUMP",
                    f"stddev {prev_w_stddev:+.6f} -> {w_stddev:+.6f}, delta={d:+.6f}",
                )

        if w_stderr is not None and prev_w_stderr is not None:
            d = w_stderr - prev_w_stderr
            self.add_series(f"{lane}_welford_stderr_delta", d)

        if w_n is not None and prev_w_n is not None:
            dn = w_n - prev_w_n
            self.add_series(f"{lane}_welford_n_delta", dn)

            if self.stat_n_strict and dn != 1:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "WELFORD_N_STEP",
                    f"n {prev_w_n:,} -> {w_n:,}, delta={dn:+,}",
                )
            elif dn < 0 or dn > 10:
                self.issue(
                    pps,
                    lane,
                    "WARN",
                    "WELFORD_N_ANOMALY",
                    f"n {prev_w_n:,} -> {w_n:,}, delta={dn:+,}",
                )

    def update_synth_run(
        self,
        *,
        lane: str,
        pps: Optional[int],
        synth_error: int,
        synth_threshold: Optional[int],
        synth_state: Dict[str, Optional[Dict[str, Any]]],
    ) -> None:
        threshold = synth_threshold if synth_threshold is not None else self.synthetic_warn_cycles
        active = abs(synth_error) > threshold

        state = synth_state.get(lane)

        if not active:
            if state is not None:
                self.close_synth_run(lane, synth_state)
            return

        if state is None:
            synth_state[lane] = {
                "start_pps": pps,
                "end_pps": pps,
                "rows": 1,
                "min_error": synth_error,
                "max_error": synth_error,
                "start_error": synth_error,
                "end_error": synth_error,
                "prev_error": synth_error,
                "max_step": 0,
            }
            return

        prev_error = state["prev_error"]
        step = synth_error - prev_error

        state["end_pps"] = pps
        state["rows"] += 1
        state["min_error"] = min(state["min_error"], synth_error)
        state["max_error"] = max(state["max_error"], synth_error)
        state["end_error"] = synth_error
        state["prev_error"] = synth_error
        state["max_step"] = max(state["max_step"], abs(step))

        if abs(step) > self.synthetic_step_warn_cycles:
            self.issue(
                pps,
                lane,
                "WARN",
                "SYNTHETIC_ENDPOINT_STEP",
                f"step={step:+,} cycles, error={synth_error:+,}",
            )

    def close_synth_run(self, lane: str, synth_state: Dict[str, Optional[Dict[str, Any]]]) -> None:
        state = synth_state.get(lane)
        if not state:
            return

        self.synthetic_runs.append(
            SyntheticRun(
                lane=lane,
                start_pps=state["start_pps"],
                end_pps=state["end_pps"],
                rows=state["rows"],
                min_error=state["min_error"],
                max_error=state["max_error"],
                start_error=state["start_error"],
                end_error=state["end_error"],
                max_step=state["max_step"],
            )
        )
        synth_state[lane] = None


# ══════════════════════════════════════════════════════════════════════════════
# Printing
# ══════════════════════════════════════════════════════════════════════════════

def print_header(title: str) -> None:
    print()
    print(title)
    print("═" * len(title))


def has_issue(report: SanityReport, reason: str) -> bool:
    return any(i.reason == reason for i in report.issues)


def print_verdict(lanes: Sequence[str], report: SanityReport) -> None:
    print_header("Verdict")

    for lane in lanes:
        counter_clean = not any(i.lane == lane and i.reason == "COUNTER_DELTA" for i in report.issues)
        public_clean = not any(i.lane == lane and i.reason in ("PUBLIC_NS_INTERVAL", "PUBLIC_NS_VS_PPS_RESIDUAL") for i in report.issues)
        stats_clean = not any(
            i.lane == lane and (
                i.reason.startswith("STAT_") or i.reason.startswith("WELFORD_")
            )
            for i in report.issues
        )

        print(f"  {lane.upper()} counter lineage:       {'CLEAN' if counter_clean else 'CHECK'}")
        print(f"  {lane.upper()} public ns ledger:      {'CLEAN' if public_clean else 'CHECK'}")
        print(f"  {lane.upper()} PPB/TAU/Welford state: {'CLEAN' if stats_clean else 'CHECK'}")

    seg_count = sum(1 for i in report.issues if i.reason == "SEGMENT_BREAK")
    print(f"  segment breaks:              {seg_count}")
    print(f"  tooth-jump evidence:         {'YES' if has_issue(report, 'PUBLIC_NS_INTERVAL') else 'NONE'}")
    print(f"  stats-jump evidence:         {'YES' if any(i.reason.startswith('STAT_') or i.reason.startswith('WELFORD_') for i in report.issues) else 'NONE'}")


def print_report(
    *,
    campaign: str,
    rows: Sequence[Dict[str, Any]],
    lanes: Sequence[str],
    report: SanityReport,
    show: int,
) -> None:
    print(f"timebase_forensic_sanity {REPORT_VERSION}")
    print(f"Campaign: {campaign}  rows={len(rows):,}  lanes={','.join(lanes)}")

    print_verdict(lanes, report)

    print_header("TIMEBASE forensic sanity report")
    print(f"  cycle threshold            = {report.cycle_threshold:,} cycles")
    print(f"  gate row warnings          = {report.gate_row_warnings}")
    print(f"  ns threshold               = {report.ns_threshold:,} ns")
    print(f"  published tolerance        = ±{report.published_tolerance_cycles:,} cycles")
    print(f"  GNSS/PPS offset tolerance  = ±{report.gnss_offset_tolerance_ns:,} ns")
    print(f"  synthetic warn threshold   = {report.synthetic_warn_cycles:,} cycles")
    print(f"  synthetic fail threshold   = {report.synthetic_fail_cycles:,} cycles")
    print(f"  synthetic step warning     = {report.synthetic_step_warn_cycles:,} cycles")
    print(f"  stat ppb jump              = {report.stat_ppb_jump:g} ppb")
    print(f"  stat tau jump              = {report.stat_tau_jump_ppb:g} ppb-equivalent")
    print(f"  Welford mean jump          = {report.stat_welford_mean_jump:g}")
    print(f"  Welford extreme jump       = {report.stat_welford_extreme_jump:g}")
    print(f"  Welford stddev jump        = {report.stat_welford_stddev_jump:g}")

    print_header("Issue counts")
    sev_counts = Counter(i.severity for i in report.issues)
    if sev_counts:
        for sev in ("FAIL", "WARN", "INFO"):
            if sev_counts.get(sev):
                print(f"  {sev:<7} {sev_counts[sev]:,}")
    else:
        print("  none")

    reason_counts = Counter(i.reason for i in report.issues)
    if reason_counts:
        print()
        print("Top reasons")
        for reason, count in reason_counts.most_common(30):
            print(f"  {reason:<40} {count:>8,}")

    print_header("Segments")
    if not report.segments:
        print("  none")
    else:
        print(f"{'idx':>4} {'start':>10} {'end':>10} {'rows':>8} {'gnss_pps_offset_ns':>24}")
        print("─" * 64)
        for seg in report.segments:
            print(
                f"{seg.index:>4} "
                f"{fmt_num(seg.start_pps):>10} "
                f"{fmt_num(seg.end_pps):>10} "
                f"{seg.rows:>8,} "
                f"{fmt_num(seg.gnss_pps_offset_ns):>24}"
            )

    print_header("Schema coverage")
    if report.coverage:
        for key, count in sorted(report.coverage.items()):
            print(f"  {key:<42} {count:,}")
    else:
        print("  none")

    print_header("Summary")
    if report.series:
        for key in sorted(report.series):
            stats = stable_stats(report.series[key])
            if not stats:
                continue

            n, mean, sd, mn, mx = stats

            unit = ""
            if key.endswith("_ns"):
                unit = "ns"
            elif key.endswith("_cycles"):
                unit = "cycles"
            elif key.endswith("_ticks"):
                unit = "ticks"

            print(
                f"  {key:<42} "
                f"n={n:>7,}  "
                f"mean={fmt_delta(mean, unit):>20}  "
                f"sd={sd:>14,.6f}  "
                f"min={fmt_delta(mn, unit):>20}  "
                f"max={fmt_delta(mx, unit):>20}"
            )
    else:
        print("  none")

    print_header("Synthetic endpoint repair runs")
    if not report.synthetic_runs:
        print("  none")
    else:
        print(
            f"{'lane':>7} {'start':>10} {'end':>10} {'rows':>8} "
            f"{'start_err':>12} {'end_err':>12} {'min_err':>12} {'max_err':>12} {'max_step':>12}"
        )
        print("─" * 102)

        for run in report.synthetic_runs:
            max_abs = max(abs(run.min_error or 0), abs(run.max_error or 0))
            label = "LARGE" if max_abs > report.synthetic_fail_cycles else "ACTIVE"

            print(
                f"{run.lane:>7} "
                f"{fmt_num(run.start_pps):>10} "
                f"{fmt_num(run.end_pps):>10} "
                f"{run.rows:>8,} "
                f"{fmt_delta(run.start_error):>12} "
                f"{fmt_delta(run.end_error):>12} "
                f"{fmt_delta(run.min_error):>12} "
                f"{fmt_delta(run.max_error):>12} "
                f"{fmt_delta(run.max_step):>12} "
                f" {label}"
            )

    print_header(f"First {show} issues")
    if not report.issues:
        print("  none")
    else:
        print(f"{'pps':>10} {'lane':>7} {'sev':>5} {'reason':<36} detail")
        print("─" * 116)

        for issue in report.issues[:show]:
            print(
                f"{fmt_num(issue.pps):>10} "
                f"{issue.lane:>7} "
                f"{issue.severity:>5} "
                f"{issue.reason:<36} "
                f"{issue.detail}"
            )

        if len(report.issues) > show:
            print(f"... {len(report.issues) - show:,} more issues suppressed")

    print_header("Interpretation")
    print("  • SEGMENT_BREAK_* is a campaign/timebase continuity break. Row-to-row ledger/stat checks reset there.")
    print("  • COUNTER_DELTA / ADJ_* failures point at target lineage or counter-ladder trouble.")
    print("  • GATE_* summaries point at ProcessInterrupt raw interval gate/predictor surface.")
    print("  • PUBLIC_NS_INTERVAL is the inside-segment tooth-jump detector.")
    print("  • STAT_* / WELFORD_* warnings mean accumulated statistics moved abruptly inside a continuous segment.")
    print("  • WELFORD_MIN/MAX jumps are especially important because one poisoned residual can blow open the envelope.")
    print("  • Synthetic repair is summarized as runs; row-level synthetic spam is intentionally suppressed.")


# ══════════════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════════════

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Segment-aware TIMEBASE forensic sanity report with PPB/TAU/Welford audit."
    )

    p.add_argument("campaign", help="Campaign name, e.g. IRQ3")
    p.add_argument("--lane", default="all", help="OCXO1, OCXO2, VCLOCK, DWT, or all")
    p.add_argument("--show", type=int, default=200, help="Number of issues to print")
    p.add_argument("--limit", type=int, default=0, help="Limit rows after loading")

    p.add_argument("--cycle-threshold", type=int, default=100)
    p.add_argument("--gate-row-warnings", action="store_true",
                   help="Emit row-level GATE_RESIDUAL warnings. Default is summary-only.")
    p.add_argument("--ns-threshold", type=int, default=10_000)
    p.add_argument("--published-tolerance-cycles", type=int, default=4)
    p.add_argument("--gnss-offset-tolerance-ns", type=int, default=10_000)

    p.add_argument("--synthetic-warn-cycles", type=int, default=500)
    p.add_argument("--synthetic-fail-cycles", type=int, default=20_000)
    p.add_argument("--synthetic-step-warn-cycles", type=int, default=10_000)

    p.add_argument("--stat-ppb-jump", type=float, default=25.0)
    p.add_argument("--stat-tau-jump-ppb", type=float, default=25.0)
    p.add_argument("--stat-welford-mean-jump", type=float, default=25.0)
    p.add_argument("--stat-welford-extreme-jump", type=float, default=500.0)
    p.add_argument("--stat-welford-stddev-jump", type=float, default=25.0)
    p.add_argument("--stat-n-strict", action="store_true")

    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    print(f"timebase_forensic_sanity {REPORT_VERSION} entered")

    args = parse_args(argv)
    lanes = resolve_lanes(args.lane)

    rows = fetch_timebase(args.campaign)
    if args.limit:
        rows = rows[:args.limit]

    if not rows:
        print(f"No TIMEBASE rows found for campaign {args.campaign!r}.")
        return 2

    report = SanityReport(
        cycle_threshold=args.cycle_threshold,
        ns_threshold=args.ns_threshold,
        gate_row_warnings=args.gate_row_warnings,
        published_tolerance_cycles=args.published_tolerance_cycles,
        gnss_offset_tolerance_ns=args.gnss_offset_tolerance_ns,
        synthetic_warn_cycles=args.synthetic_warn_cycles,
        synthetic_fail_cycles=args.synthetic_fail_cycles,
        synthetic_step_warn_cycles=args.synthetic_step_warn_cycles,
        stat_ppb_jump=args.stat_ppb_jump,
        stat_tau_jump_ppb=args.stat_tau_jump_ppb,
        stat_welford_mean_jump=args.stat_welford_mean_jump,
        stat_welford_extreme_jump=args.stat_welford_extreme_jump,
        stat_welford_stddev_jump=args.stat_welford_stddev_jump,
        stat_n_strict=args.stat_n_strict,
    )

    report.analyze(rows, lanes)

    print_report(
        campaign=args.campaign,
        rows=rows,
        lanes=lanes,
        report=report,
        show=args.show,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))