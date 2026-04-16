"""
Dedicated OCXO campaign analyzer.

Focused only on OCXO second-edge forensics.

This version prefers the synthetic/corrected OCXO second-boundary fields when
present, falls back to raw boundary fields when necessary, and preserves both
views so remaining pathology can be reasoned about explicitly.

Canonical row model per OCXO:
  - pps_count
  - start_gnss_ns / end_gnss_ns     (prefer synthetic/final boundary)
  - delta_ns                        (prefer synthetic/final boundary delta)
  - predicted_delta_ns             (last observed delta_ns, i.e. "last is best")
  - residual_a = delta_ns - predicted_delta_ns
  - residual_b = delta_ns - 1_000_000_000

Useful forensic side-channels:
  - raw start/end boundary and raw delta
  - raw-minus-final corrections at start/end
  - bridge raw/final errors
  - compare-latency deltas
  - bucket observed/predicted GNSS ns and cycles

Usage:
    python -m zpnet.tests.ocxo_campaign_analyzer <campaign_name>
    .zt ocxo_campaign_analyzer Calibrate12
"""

from __future__ import annotations

import json
import math
import sys
import traceback
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000

DOUBLED_SECOND_THRESHOLD_NS = 1_500_000_000
HALF_SECOND_THRESHOLD_NS = 500_000_000
DELTA_REASONABLE_WARN_NS = 10_000
RESIDUAL_JUMP_WARN_NS = 250
RESIDUAL_JUMP_ALARM_NS = 1_000
CANONICAL_OBSERVED_MISMATCH_WARN_NS = 50

MAX_TABLE_ROWS = 120
MAX_ISSUES = 20


@dataclass
class OcxoRow:
    pps_count: int
    gnss_ns: int

    start_gnss_ns: Optional[int]
    end_gnss_ns: Optional[int]
    delta_ns: Optional[int]
    predicted_delta_ns: Optional[int]

    residual_a_ns: Optional[int]
    residual_b_ns: Optional[int]

    bridge_raw_error_ns: Optional[int]
    bridge_final_error_ns: Optional[int]

    dwt_at_edge: Optional[int]
    dwt_cycles_between_edges: Optional[int]

    phase_offset_ns: Optional[int]
    ppb: Optional[float]
    window_error_ns: Optional[int]
    second_residual_ns: Optional[int]

    second_gnss_ns_observed: Optional[int]
    second_gnss_ns_prediction: Optional[int]
    second_gnss_ns_prediction_error: Optional[int]

    second_cycles_observed: Optional[int]
    second_cycles_prediction: Optional[int]
    second_cycles_prediction_error: Optional[int]

    second_start_gnss_ns_raw: Optional[int]
    second_end_gnss_ns_raw: Optional[int]
    second_start_dwt_raw: Optional[int]
    second_end_dwt_raw: Optional[int]

    second_start_gnss_ns_final: Optional[int]
    second_end_gnss_ns_final: Optional[int]
    second_start_dwt_final: Optional[int]
    second_end_dwt_final: Optional[int]

    second_start_raw_minus_final_ns: Optional[int]
    second_end_raw_minus_final_ns: Optional[int]

    raw_delta_ns: Optional[int]
    final_delta_ns: Optional[int]
    canonical_minus_observed_ns: Optional[int]

    compare_delta_ns: Optional[int]
    compare_delta_ticks: Optional[int]

    bucket_interval_counts: Optional[int]
    current_window_bucket_count: Optional[int]
    last_second_bucket_count: Optional[int]
    last_bucket_cycles: Optional[int]
    last_bucket_gnss_ns: Optional[int]
    current_window_cycles_sum: Optional[int]
    current_window_gnss_ns_sum: Optional[int]

    delta_source: str


class WelfordStats:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float):
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        if x < self.min_val:
            self.min_val = x
        if x > self.max_val:
            self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    def summary(self, fmt: str = ".3f") -> str:
        if self.n == 0:
            return "no data"
        sd = f"{self.stddev:{fmt}}" if self.n >= 2 else "---"
        return (
            f"n={self.n}  mean={self.mean:+{fmt}}  sd={sd}  "
            f"min={self.min_val:{fmt}}  max={self.max_val:{fmt}}"
        )


def _frag(row: Dict[str, Any], key: str, default=None):
    frag = row.get("fragment")
    if not isinstance(frag, dict):
        return default
    return frag.get(key, default)


def _frag_int(row: Dict[str, Any], key: str) -> Optional[int]:
    v = _frag(row, key)
    return int(v) if v is not None else None


def _frag_float(row: Dict[str, Any], key: str) -> Optional[float]:
    v = _frag(row, key)
    return float(v) if v is not None else None


def _resolved_final_boundary(
    row: Dict[str, Any],
    prefix: str,
    which: str,
) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    """
    Resolve the synthetic/final GNSS boundary if available. If the explicit
    *_final field is absent, reconstruct it from raw - raw_minus_final.
    Returns: final_ns, raw_ns, raw_minus_final_ns
    """
    final_ns = _frag_int(row, f"{prefix}_diag_ocxo_second_{which}_gnss_ns_final")
    raw_ns = _frag_int(row, f"{prefix}_diag_ocxo_second_{which}_gnss_ns_raw")
    raw_minus_final_ns = _frag_int(row, f"{prefix}_diag_ocxo_second_{which}_raw_minus_final_ns")

    if final_ns is None and raw_ns is not None and raw_minus_final_ns is not None:
        final_ns = raw_ns - raw_minus_final_ns

    return final_ns, raw_ns, raw_minus_final_ns


def _resolved_final_dwt(
    row: Dict[str, Any],
    prefix: str,
    which: str,
) -> Tuple[Optional[int], Optional[int]]:
    """
    Resolve synthetic/final DWT boundary when present, otherwise use raw.
    Returns: final_dwt, raw_dwt
    """
    final_dwt = _frag_int(row, f"{prefix}_diag_ocxo_second_{which}_dwt_final")
    raw_dwt = _frag_int(row, f"{prefix}_diag_ocxo_second_{which}_dwt_raw")
    if final_dwt is None:
        final_dwt = raw_dwt
    return final_dwt, raw_dwt


def _choose_best_delta_fields(
    row: Dict[str, Any],
    prefix: str,
) -> Tuple[Optional[int], Optional[int], Optional[int], Optional[int], str]:
    """
    Prefer synthetic/final boundaries, then bucket-observed, then raw, then
    legacy coarse fields.

    Returns:
        start_gnss_ns, end_gnss_ns, delta_ns, second_residual_ns, delta_source
    """
    start_final, start_raw, _ = _resolved_final_boundary(row, prefix, "start")
    end_final, end_raw, _ = _resolved_final_boundary(row, prefix, "end")
    observed = _frag_int(row, f"{prefix}_diag_ocxo_second_gnss_ns_observed")
    residual = _frag_int(row, f"{prefix}_diag_ocxo_second_residual_ns")

    if start_final is not None and end_final is not None:
        return start_final, end_final, end_final - start_final, residual, "bucket_final"

    if end_final is not None and observed is not None:
        return end_final - observed, end_final, observed, residual, "bucket_final_observed"

    if start_raw is not None and end_raw is not None:
        delta_ns = observed
        if delta_ns is None:
            delta_ns = end_raw - start_raw
        return start_raw, end_raw, delta_ns, residual, "bucket_raw"

    if end_raw is not None and observed is not None:
        return end_raw - observed, end_raw, observed, residual, "bucket_observed"

    end_final = _frag_int(row, f"{prefix}_diag_gnss_ns_at_event_final")
    coarse_delta = _frag_int(row, f"{prefix}_gnss_ns_between_edges")
    coarse_residual = _frag_int(row, f"{prefix}_second_residual_ns")

    if end_final is not None and coarse_delta is not None:
        return end_final - coarse_delta, end_final, coarse_delta, coarse_residual, "coarse"

    return None, end_final, coarse_delta, coarse_residual, "missing"


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        payload["_db_id"] = row["id"]
        payload["_db_ts"] = str(row["ts"])
        result.append(payload)
    return result


def find_recovery_boundaries(rows: List[Dict[str, Any]]) -> Set[int]:
    boundaries: Set[int] = set()
    for i in range(1, len(rows)):
        prev_pps = int(rows[i - 1]["pps_count"])
        curr_pps = int(rows[i]["pps_count"])
        if curr_pps > prev_pps + 1:
            boundaries.add(curr_pps)
    return boundaries


def print_campaign_header(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> None:
    pps_counts = [int(r["pps_count"]) for r in rows]
    pps_min, pps_max = min(pps_counts), max(pps_counts)

    print("=" * 110)
    print(f"OCXO CAMPAIGN ANALYZER: {rows[0].get('campaign', '?')}")
    print("=" * 110)
    print(f"pps_count range:   {pps_min} -> {pps_max}")
    print(f"records:           {len(rows):,}")
    print(f"recoveries:        {len(recovery_boundaries)}")
    print(f"servo mode:        {_frag(rows[-1], 'calibrate_ocxo', '?')}")
    print()


def build_ocxo_rows(
    rows: List[Dict[str, Any]],
    prefix: str,
    recovery_boundaries: Set[int],
) -> List[OcxoRow]:
    out: List[OcxoRow] = []
    prev_delta_ns: Optional[int] = None

    for row in rows:
        pps = int(row["pps_count"])

        if pps in recovery_boundaries:
            prev_delta_ns = None

        start_gnss_ns, end_gnss_ns, delta_ns, second_residual_ns, delta_source = (
            _choose_best_delta_fields(row, prefix)
        )

        second_start_gnss_ns_final, second_start_gnss_ns_raw, second_start_raw_minus_final_ns = (
            _resolved_final_boundary(row, prefix, "start")
        )
        second_end_gnss_ns_final, second_end_gnss_ns_raw, second_end_raw_minus_final_ns = (
            _resolved_final_boundary(row, prefix, "end")
        )
        second_start_dwt_final, second_start_dwt_raw = _resolved_final_dwt(row, prefix, "start")
        second_end_dwt_final, second_end_dwt_raw = _resolved_final_dwt(row, prefix, "end")

        raw_delta_ns = None
        if second_start_gnss_ns_raw is not None and second_end_gnss_ns_raw is not None:
            raw_delta_ns = second_end_gnss_ns_raw - second_start_gnss_ns_raw

        final_delta_ns = None
        if second_start_gnss_ns_final is not None and second_end_gnss_ns_final is not None:
            final_delta_ns = second_end_gnss_ns_final - second_start_gnss_ns_final

        second_gnss_ns_observed = _frag_int(row, f"{prefix}_diag_ocxo_second_gnss_ns_observed")
        canonical_minus_observed_ns = None
        if final_delta_ns is not None and second_gnss_ns_observed is not None:
            canonical_minus_observed_ns = final_delta_ns - second_gnss_ns_observed

        predicted_delta_ns = prev_delta_ns

        residual_a_ns = None
        if delta_ns is not None and predicted_delta_ns is not None:
            residual_a_ns = delta_ns - predicted_delta_ns

        residual_b_ns = None
        if delta_ns is not None:
            residual_b_ns = delta_ns - NS_PER_SECOND

        if second_residual_ns is None and delta_ns is not None:
            second_residual_ns = NS_PER_SECOND - delta_ns

        out.append(
            OcxoRow(
                pps_count=pps,
                gnss_ns=_frag_int(row, "gnss_ns") or 0,
                start_gnss_ns=start_gnss_ns,
                end_gnss_ns=end_gnss_ns,
                delta_ns=delta_ns,
                predicted_delta_ns=predicted_delta_ns,
                residual_a_ns=residual_a_ns,
                residual_b_ns=residual_b_ns,
                bridge_raw_error_ns=_frag_int(row, f"{prefix}_diag_bridge_raw_error_ns"),
                bridge_final_error_ns=_frag_int(row, f"{prefix}_diag_bridge_final_error_ns"),
                dwt_at_edge=_frag_int(row, f"{prefix}_dwt_at_edge"),
                dwt_cycles_between_edges=_frag_int(row, f"{prefix}_dwt_cycles_between_edges"),
                phase_offset_ns=_frag_int(row, f"{prefix}_phase_offset_ns"),
                ppb=_frag_float(row, f"{prefix}_ppb"),
                window_error_ns=_frag_int(row, f"{prefix}_window_error_ns"),
                second_residual_ns=second_residual_ns,
                second_gnss_ns_observed=second_gnss_ns_observed,
                second_gnss_ns_prediction=_frag_int(row, f"{prefix}_diag_ocxo_second_gnss_ns_prediction"),
                second_gnss_ns_prediction_error=_frag_int(row, f"{prefix}_diag_ocxo_second_gnss_ns_prediction_error"),
                second_cycles_observed=_frag_int(row, f"{prefix}_diag_ocxo_second_cycles_observed"),
                second_cycles_prediction=_frag_int(row, f"{prefix}_diag_ocxo_second_cycles_prediction"),
                second_cycles_prediction_error=_frag_int(row, f"{prefix}_diag_ocxo_second_cycles_prediction_error"),
                second_start_gnss_ns_raw=second_start_gnss_ns_raw,
                second_end_gnss_ns_raw=second_end_gnss_ns_raw,
                second_start_dwt_raw=second_start_dwt_raw,
                second_end_dwt_raw=second_end_dwt_raw,
                second_start_gnss_ns_final=second_start_gnss_ns_final,
                second_end_gnss_ns_final=second_end_gnss_ns_final,
                second_start_dwt_final=second_start_dwt_final,
                second_end_dwt_final=second_end_dwt_final,
                second_start_raw_minus_final_ns=second_start_raw_minus_final_ns,
                second_end_raw_minus_final_ns=second_end_raw_minus_final_ns,
                raw_delta_ns=raw_delta_ns,
                final_delta_ns=final_delta_ns,
                canonical_minus_observed_ns=canonical_minus_observed_ns,
                compare_delta_ns=_frag_int(row, f"{prefix}_diag_counter16_minus_compare_ns"),
                compare_delta_ticks=_frag_int(row, f"{prefix}_diag_counter16_minus_compare_ticks"),
                bucket_interval_counts=_frag_int(row, f"{prefix}_diag_ocxo_bucket_interval_counts"),
                current_window_bucket_count=_frag_int(row, f"{prefix}_diag_ocxo_current_window_bucket_count"),
                last_second_bucket_count=_frag_int(row, f"{prefix}_diag_ocxo_last_second_bucket_count"),
                last_bucket_cycles=_frag_int(row, f"{prefix}_diag_ocxo_last_bucket_cycles"),
                last_bucket_gnss_ns=_frag_int(row, f"{prefix}_diag_ocxo_last_bucket_gnss_ns"),
                current_window_cycles_sum=_frag_int(row, f"{prefix}_diag_ocxo_current_window_cycles_sum"),
                current_window_gnss_ns_sum=_frag_int(row, f"{prefix}_diag_ocxo_current_window_gnss_ns_sum"),
                delta_source=delta_source,
            )
        )

        if delta_ns is not None and delta_ns > 0:
            prev_delta_ns = delta_ns

    return out


def print_ocxo_table(label: str, data: List[OcxoRow], max_rows: int = MAX_TABLE_ROWS) -> None:
    rows = data[:max_rows]

    print("-" * 110)
    print(label)
    print("-" * 110)
    print(
        f"{'PPS':>7s}  {'START_GNSS_NS':>16s}  {'END_GNSS_NS':>16s}  "
        f"{'DELTA_NS':>14s}  {'PRED_DELTA_NS':>14s}  {'RES_A':>10s}  {'RES_B':>10s}"
    )
    print(
        f"{'---':>7s}  {'-------------':>16s}  {'-----------':>16s}  "
        f"{'--------':>14s}  {'-------------':>14s}  {'-----':>10s}  {'-----':>10s}"
    )

    for row in rows:
        print(
            f"{row.pps_count:7d}  "
            f"{str(row.start_gnss_ns) if row.start_gnss_ns is not None else '---':>16s}  "
            f"{str(row.end_gnss_ns) if row.end_gnss_ns is not None else '---':>16s}  "
            f"{str(row.delta_ns) if row.delta_ns is not None else '---':>14s}  "
            f"{str(row.predicted_delta_ns) if row.predicted_delta_ns is not None else '---':>14s}  "
            f"{str(row.residual_a_ns) if row.residual_a_ns is not None else '---':>10s}  "
            f"{str(row.residual_b_ns) if row.residual_b_ns is not None else '---':>10s}"
        )

    if len(data) > max_rows:
        print(f"... truncated table: showing first {max_rows} of {len(data)} rows")
    print()


def analyze_ocxo(label: str, data: List[OcxoRow]) -> List[str]:
    anomalies: List[str] = []

    delta_stats = WelfordStats()
    pred_stats = WelfordStats()
    residual_a_stats = WelfordStats()
    residual_b_stats = WelfordStats()
    bridge_raw_stats = WelfordStats()
    bridge_final_stats = WelfordStats()
    bucket_obs_stats = WelfordStats()
    bucket_pred_stats = WelfordStats()
    bucket_err_stats = WelfordStats()
    bucket_residual_stats = WelfordStats()
    cycle_err_stats = WelfordStats()
    raw_delta_stats = WelfordStats()
    final_delta_stats = WelfordStats()
    canon_minus_obs_stats = WelfordStats()
    start_corr_stats = WelfordStats()
    end_corr_stats = WelfordStats()
    compare_ns_stats = WelfordStats()
    compare_ticks_stats = WelfordStats()

    prev_residual_a: Optional[int] = None
    prev_residual_b: Optional[int] = None

    edge_issues: List[str] = []
    delta_warns: List[str] = []
    doubled_seconds: List[str] = []
    short_seconds: List[str] = []
    residual_jump_warns: List[str] = []
    residual_jump_alarms: List[str] = []
    correction_warns: List[str] = []
    canonical_mismatch_warns: List[str] = []

    for row in data:
        if row.delta_ns is not None:
            delta_stats.update(float(row.delta_ns))
        if row.predicted_delta_ns is not None:
            pred_stats.update(float(row.predicted_delta_ns))
        if row.residual_a_ns is not None:
            residual_a_stats.update(float(row.residual_a_ns))
        if row.residual_b_ns is not None:
            residual_b_stats.update(float(row.residual_b_ns))
        if row.bridge_raw_error_ns is not None:
            bridge_raw_stats.update(float(row.bridge_raw_error_ns))
        if row.bridge_final_error_ns is not None:
            bridge_final_stats.update(float(row.bridge_final_error_ns))
        if row.second_gnss_ns_observed is not None:
            bucket_obs_stats.update(float(row.second_gnss_ns_observed))
        if row.second_gnss_ns_prediction is not None:
            bucket_pred_stats.update(float(row.second_gnss_ns_prediction))
        if row.second_gnss_ns_prediction_error is not None:
            bucket_err_stats.update(float(row.second_gnss_ns_prediction_error))
        if row.second_residual_ns is not None:
            bucket_residual_stats.update(float(row.second_residual_ns))
        if row.second_cycles_prediction_error is not None:
            cycle_err_stats.update(float(row.second_cycles_prediction_error))
        if row.raw_delta_ns is not None:
            raw_delta_stats.update(float(row.raw_delta_ns))
        if row.final_delta_ns is not None:
            final_delta_stats.update(float(row.final_delta_ns))
        if row.canonical_minus_observed_ns is not None:
            canon_minus_obs_stats.update(float(row.canonical_minus_observed_ns))
        if row.second_start_raw_minus_final_ns is not None:
            start_corr_stats.update(float(row.second_start_raw_minus_final_ns))
        if row.second_end_raw_minus_final_ns is not None:
            end_corr_stats.update(float(row.second_end_raw_minus_final_ns))
        if row.compare_delta_ns is not None:
            compare_ns_stats.update(float(row.compare_delta_ns))
        if row.compare_delta_ticks is not None:
            compare_ticks_stats.update(float(row.compare_delta_ticks))

        if row.start_gnss_ns is not None and row.end_gnss_ns is not None:
            if row.end_gnss_ns < row.start_gnss_ns:
                edge_issues.append(
                    f"pps={row.pps_count}: end_gnss_ns < start_gnss_ns "
                    f"({row.end_gnss_ns} < {row.start_gnss_ns})"
                )

        if row.delta_ns is not None:
            if row.delta_ns >= DOUBLED_SECOND_THRESHOLD_NS:
                doubled_seconds.append(
                    f"pps={row.pps_count}: delta_ns looks doubled ({row.delta_ns:,})"
                )
            elif row.delta_ns <= HALF_SECOND_THRESHOLD_NS:
                short_seconds.append(
                    f"pps={row.pps_count}: delta_ns implausibly short ({row.delta_ns:,})"
                )
            elif abs(row.delta_ns - NS_PER_SECOND) > DELTA_REASONABLE_WARN_NS:
                delta_warns.append(
                    f"pps={row.pps_count}: delta_ns dev from 1e9 = {row.delta_ns - NS_PER_SECOND:+,d}"
                )

        if row.second_start_raw_minus_final_ns is not None or row.second_end_raw_minus_final_ns is not None:
            s = row.second_start_raw_minus_final_ns or 0
            e = row.second_end_raw_minus_final_ns or 0
            if abs(s - e) > CANONICAL_OBSERVED_MISMATCH_WARN_NS:
                correction_warns.append(
                    f"pps={row.pps_count}: start/end raw-minus-final differ ({s:+d} vs {e:+d})"
                )

        if row.canonical_minus_observed_ns is not None and abs(row.canonical_minus_observed_ns) > CANONICAL_OBSERVED_MISMATCH_WARN_NS:
            canonical_mismatch_warns.append(
                f"pps={row.pps_count}: canonical-final delta vs observed mismatch = {row.canonical_minus_observed_ns:+d} ns"
            )

        if row.residual_a_ns is not None and prev_residual_a is not None:
            jump = row.residual_a_ns - prev_residual_a
            if abs(jump) >= RESIDUAL_JUMP_ALARM_NS:
                residual_jump_alarms.append(
                    f"pps={row.pps_count}: residual_a jump {jump:+,d} ns "
                    f"({prev_residual_a:+,d} -> {row.residual_a_ns:+,d})"
                )
            elif abs(jump) >= RESIDUAL_JUMP_WARN_NS:
                residual_jump_warns.append(
                    f"pps={row.pps_count}: residual_a jump {jump:+,d} ns"
                )
        if row.residual_a_ns is not None:
            prev_residual_a = row.residual_a_ns

        if row.residual_b_ns is not None and prev_residual_b is not None:
            jump_b = row.residual_b_ns - prev_residual_b
            if abs(jump_b) >= RESIDUAL_JUMP_ALARM_NS:
                residual_jump_alarms.append(
                    f"pps={row.pps_count}: residual_b jump {jump_b:+,d} ns "
                    f"({prev_residual_b:+,d} -> {row.residual_b_ns:+,d})"
                )
            elif abs(jump_b) >= RESIDUAL_JUMP_WARN_NS:
                residual_jump_warns.append(
                    f"pps={row.pps_count}: residual_b jump {jump_b:+,d} ns"
                )
        if row.residual_b_ns is not None:
            prev_residual_b = row.residual_b_ns

    source_counts = {}
    for row in data:
        source_counts[row.delta_source] = source_counts.get(row.delta_source, 0) + 1
    source_mix = "  ".join(f"{k}={v}" for k, v in sorted(source_counts.items()))

    print(f"{label} SUMMARY")
    print(f"  source_mix:        {source_mix if source_mix else 'no data'}")
    print(f"  delta_ns:          {delta_stats.summary('.3f')}")
    print(f"  predicted_delta:   {pred_stats.summary('.3f')}")
    print(f"  residual_a:        {residual_a_stats.summary('.3f')}")
    print(f"  residual_b:        {residual_b_stats.summary('.3f')}")
    print(f"  bridge_raw_err:    {bridge_raw_stats.summary('.3f')}")
    print(f"  bridge_final_err:  {bridge_final_stats.summary('.3f')}")
    print(f"  bucket_obs_ns:     {bucket_obs_stats.summary('.3f')}")
    print(f"  bucket_pred_ns:    {bucket_pred_stats.summary('.3f')}")
    print(f"  bucket_pred_err:   {bucket_err_stats.summary('.3f')}")
    print(f"  bucket_residual:   {bucket_residual_stats.summary('.3f')}")
    print(f"  cycle_pred_err:    {cycle_err_stats.summary('.3f')}")
    print(f"  raw_delta_ns:      {raw_delta_stats.summary('.3f')}")
    print(f"  final_delta_ns:    {final_delta_stats.summary('.3f')}")
    print(f"  canon-observed:    {canon_minus_obs_stats.summary('.3f')}")
    print(f"  start_raw-final:   {start_corr_stats.summary('.3f')}")
    print(f"  end_raw-final:     {end_corr_stats.summary('.3f')}")
    print(f"  compare_delta_ns:  {compare_ns_stats.summary('.3f')}")
    print(f"  compare_delta_tk:  {compare_ticks_stats.summary('.3f')}")
    print()

    def emit(title: str, items: List[str], cap: int = MAX_ISSUES) -> None:
        if not items:
            return
        print(f"{title}: {len(items)}")
        for msg in items[:cap]:
            print(f"  {msg}")
        if len(items) > cap:
            print(f"  ... and {len(items) - cap} more")
        print()

    emit("EDGE ISSUES", edge_issues)
    emit("DELTA WARNS", delta_warns)
    emit("DOUBLED-SECOND SUSPECTS", doubled_seconds)
    emit("SHORT-SECOND SUSPECTS", short_seconds)
    emit("CORRECTION WARNS", correction_warns)
    emit("CANONICAL MISMATCH WARNS", canonical_mismatch_warns)
    emit("RESIDUAL JUMP WARNS", residual_jump_warns)
    emit("RESIDUAL JUMP ALARMS", residual_jump_alarms)

    if edge_issues:
        anomalies.append(f"{label}: {len(edge_issues)} edge issues")
    if doubled_seconds:
        anomalies.append(f"{label}: {len(doubled_seconds)} doubled-second suspects")
    if short_seconds:
        anomalies.append(f"{label}: {len(short_seconds)} short-second suspects")
    if residual_jump_alarms:
        anomalies.append(f"{label}: {len(residual_jump_alarms)} residual jump alarms")
    if canonical_mismatch_warns:
        anomalies.append(f"{label}: {len(canonical_mismatch_warns)} canonical mismatch warns")

    if anomalies:
        print(f"{label} VERDICT: NEEDS ATTENTION")
    else:
        print(f"{label} VERDICT: CLEAN")
    print()

    return anomalies


def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)
    print(f"fetched rows: {len(rows)}")

    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    recovery_boundaries = find_recovery_boundaries(rows)
    print_campaign_header(rows, recovery_boundaries)

    ocxo1_rows = build_ocxo_rows(rows, "ocxo1", recovery_boundaries)
    ocxo2_rows = build_ocxo_rows(rows, "ocxo2", recovery_boundaries)

    print_ocxo_table("OCXO1", ocxo1_rows)
    anomalies_1 = analyze_ocxo("OCXO1", ocxo1_rows)

    print_ocxo_table("OCXO2", ocxo2_rows)
    anomalies_2 = analyze_ocxo("OCXO2", ocxo2_rows)

    anomalies = anomalies_1 + anomalies_2

    print("=" * 110)
    if anomalies:
        print(f"VERDICT: ANOMALIES FOUND ({len(anomalies)})")
        for item in anomalies:
            print(f"  * {item}")
    else:
        print("VERDICT: CLEAN")
    print("=" * 110)


def main() -> None:
    print("ocxo_campaign_analyzer starting...")

    if len(sys.argv) < 2:
        print("Usage: ocxo_campaign_analyzer <campaign_name>")
        sys.exit(1)

    campaign = sys.argv[1]
    print(f"campaign={campaign}")
    analyze(campaign)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("ocxo_campaign_analyzer crashed:")
        traceback.print_exc()
        raise
