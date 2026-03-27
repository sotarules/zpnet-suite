from __future__ import annotations

"""
ZPNet EDGE TDC ANALYZER — RAW / ADJUSTED TWO-PHASE FORENSICS

Purpose
-------
Analyze the newer OCXO phase-capture telemetry that records TWO observed
post-PPS edges per OCXO, along with raw phase, adjusted phase, bias, and
per-phase bracket topology.

This tool is meant to answer questions like:
  * What bracket widths are we actually seeing for phase-1 and phase-2?
  * Are the bracket topologies stable?
  * What raw pair deltas (phase2 - phase1) occur for each topology?
  * Is the bias map doing anything useful yet?
  * Are raw and adjusted pair deltas collapsing or diverging?

Fields consumed from TIMEBASE payloads
--------------------------------------
Phase 1 (OCXO1 example):
  ocxo1_dwt_bracket_cycles
  ocxo1_raw_elapsed_ns
  ocxo1_raw_phase_offset_ns
  ocxo1_phase_bias_ns
  ocxo1_adjusted_phase_signed_ns
  ocxo1_phase_offset_ns

Phase 2:
  ocxo1_phase2_dwt_bracket_cycles
  ocxo1_phase2_raw_elapsed_ns
  ocxo1_phase2_raw_phase_offset_ns
  ocxo1_phase2_phase_bias_ns
  ocxo1_phase2_adjusted_phase_signed_ns
  ocxo1_phase2_phase_offset_ns

Pair diagnostics:
  ocxo1_phase_pair_delta_ns
  ocxo1_adjusted_phase_pair_delta_ns
  phase_pair_valid
  ocxo1_residual_ns

Same field family exists for OCXO2.
"""

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

DWT_NS_PER_CYCLE = 125.0 / 126.0


class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val: Optional[float] = None
        self.max_val: Optional[float] = None

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        if self.min_val is None or x < self.min_val:
            self.min_val = x
        if self.max_val is None or x > self.max_val:
            self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def range(self) -> float:
        if self.min_val is None or self.max_val is None:
            return 0.0
        return self.max_val - self.min_val


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
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
        result.append(payload)
    return result


def to_stats(values: Sequence[int]) -> Welford:
    w = Welford()
    for v in values:
        w.update(float(v))
    return w


def print_stats_block(title: str, values: Sequence[int], unit: str = "", ns_from_cycles: bool = False) -> None:
    print("-" * 78)
    print(title)
    print("-" * 78)
    print()

    if not values:
        print("  no samples")
        print()
        return

    w = to_stats(values)
    min_v = int(w.min_val) if w.min_val is not None else 0
    max_v = int(w.max_val) if w.max_val is not None else 0

    if ns_from_cycles:
        print(f"  n       = {w.n:,}")
        print(f"  mean    = {w.mean:,.2f} cycles  ({w.mean * DWT_NS_PER_CYCLE:,.1f} ns)")
        print(f"  stddev  = {w.stddev:,.2f} cycles  ({w.stddev * DWT_NS_PER_CYCLE:,.1f} ns)")
        print(f"  min     = {min_v:,} cycles  ({min_v * DWT_NS_PER_CYCLE:.1f} ns)")
        print(f"  max     = {max_v:,} cycles  ({max_v * DWT_NS_PER_CYCLE:.1f} ns)")
        print(f"  range   = {int(w.range):,} cycles  ({w.range * DWT_NS_PER_CYCLE:,.1f} ns)")
    else:
        suffix = f" {unit}" if unit else ""
        sign = unit == "ns"
        if sign:
            print(f"  n       = {w.n:,}")
            print(f"  mean    = {w.mean:,.2f}")
            print(f"  stddev  = {w.stddev:,.2f}")
            print(f"  min     = {min_v:+d}")
            print(f"  max     = {max_v:+d}")
            print(f"  range   = {int(w.range):,}")
        else:
            print(f"  n       = {w.n:,}")
            print(f"  mean    = {w.mean:,.2f}{suffix}")
            print(f"  stddev  = {w.stddev:,.2f}{suffix}")
            print(f"  min     = {min_v}{suffix}")
            print(f"  max     = {max_v}{suffix}")
            print(f"  range   = {int(w.range)}{suffix}")
    print()


def histogram(values: Sequence[int], title: str, show_ns_for_cycles: bool = False) -> None:
    print("-" * 78)
    print(title)
    print("-" * 78)
    print()
    if not values:
        print("  no samples")
        print()
        return

    counts = Counter(values)
    ordered = sorted(counts.keys())
    max_count = max(counts.values())
    width = 48
    last = None
    for val in ordered:
        if last is not None and val - last > 3:
            print(f"       │ {'.' * width}  ···")
        cnt = counts[val]
        bar = "█" * max(1, int(round((cnt / max_count) * width))) if cnt else ""
        pct = 100.0 * cnt / len(values)
        if show_ns_for_cycles:
            print(f"  {val:4d} │ {bar:<{width}s} {cnt:5d} ({pct:5.1f}%)  [{val * DWT_NS_PER_CYCLE:6.1f} ns]")
        else:
            print(f"  {val:4d} │ {bar:<{width}s} {cnt:5d} ({pct:5.1f}%)")
        last = val
    print()


def topology_histogram(pairs: Sequence[Tuple[int, int]], title: str) -> None:
    print("-" * 78)
    print(title)
    print("-" * 78)
    print()
    if not pairs:
        print("  no samples")
        print()
        return
    counts = Counter(pairs)
    total = len(pairs)
    for (b1, b2), cnt in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])):
        pct = 100.0 * cnt / total
        print(
            f"  ({b1:>3d}, {b2:>3d}) cycles  ({b1 * DWT_NS_PER_CYCLE:6.1f}, {b2 * DWT_NS_PER_CYCLE:6.1f} ns):"
            f" {cnt:5d} ({pct:5.1f}%)"
        )
    print()


def cluster_block(values: Sequence[int], title: str) -> None:
    print("-" * 78)
    print(title)
    print("-" * 78)
    print()
    if not values:
        print("  no samples")
        print()
        return
    counts = Counter(values)
    for val, cnt in sorted(counts.items()):
        pct = 100.0 * cnt / len(values)
        print(f"  {val:>4d} cycles ({val * DWT_NS_PER_CYCLE:6.1f} ns): {cnt:5d} ({pct:5.1f}%)")
    print()


def wrap_pair_delta(p1: int, p2: int) -> int:
    d = int(p2) - int(p1)
    if d > 50:
        d -= 100
    elif d < -50:
        d += 100
    return d


def _payload_int(row: Dict[str, Any], field: str) -> Optional[int]:
    value = row.get(field)
    if value is None:
        return None
    return int(value)


def analyze_ocxo(rows: List[Dict[str, Any]], key: str, limit: int = 0) -> None:
    upper = key.upper()
    samples: List[Dict[str, int]] = []
    skipped = 0

    required = [
        f"{key}_raw_phase_offset_ns",
        f"{key}_phase_offset_ns",
        f"{key}_raw_elapsed_ns",
        f"{key}_dwt_bracket_cycles",
        f"{key}_phase_bias_ns",
        f"{key}_phase2_raw_phase_offset_ns",
        f"{key}_phase2_phase_offset_ns",
        f"{key}_phase2_raw_elapsed_ns",
        f"{key}_phase2_dwt_bracket_cycles",
        f"{key}_phase2_phase_bias_ns",
        f"{key}_phase_pair_delta_ns",
        f"{key}_adjusted_phase_pair_delta_ns",
        f"{key}_residual_ns",
        "phase_pair_valid",
    ]

    for row in rows:
        if limit and len(samples) >= limit:
            break
        vals: Dict[str, int] = {}
        missing = False
        for field in required:
            if field == "phase_pair_valid":
                if row.get(field) is None:
                    missing = True
                    break
                vals[field] = 1 if bool(row.get(field)) else 0
                continue
            v = _payload_int(row, field)
            if v is None:
                missing = True
                break
            vals[field] = v
        pps = _payload_int(row, "pps_count")
        if pps is None:
            pps = _payload_int(row, "teensy_pps_count")
        if pps is None:
            missing = True
        else:
            vals["pps_count"] = pps

        if missing:
            skipped += 1
            continue
        samples.append(vals)

    print("=" * 78)
    print(f"EDGE TDC ANALYZER — {upper}")
    print("=" * 78)
    print()
    print(f"  Samples:               {len(samples)}")
    print(f"  Skipped missing:       {skipped}")
    print()

    if not samples:
        return

    b1 = [s[f"{key}_dwt_bracket_cycles"] for s in samples]
    b2 = [s[f"{key}_phase2_dwt_bracket_cycles"] for s in samples]
    topo = list(zip(b1, b2))

    r1 = [s[f"{key}_raw_phase_offset_ns"] for s in samples]
    r2 = [s[f"{key}_phase2_raw_phase_offset_ns"] for s in samples]
    a1 = [s[f"{key}_phase_offset_ns"] for s in samples]
    a2 = [s[f"{key}_phase2_phase_offset_ns"] for s in samples]
    e1 = [s[f"{key}_raw_elapsed_ns"] for s in samples]
    e2 = [s[f"{key}_phase2_raw_elapsed_ns"] for s in samples]
    x1 = [s[f"{key}_phase_bias_ns"] for s in samples]
    x2 = [s[f"{key}_phase2_phase_bias_ns"] for s in samples]
    d_raw = [s[f"{key}_phase_pair_delta_ns"] for s in samples]
    d_adj = [s[f"{key}_adjusted_phase_pair_delta_ns"] for s in samples]
    residuals = [s[f"{key}_residual_ns"] for s in samples]

    print_stats_block("PHASE-1 BRACKET WIDTH", b1, ns_from_cycles=True)
    print_stats_block("PHASE-2 BRACKET WIDTH", b2, ns_from_cycles=True)
    histogram(b1, f"{upper} PHASE-1 BRACKET HISTOGRAM (cycles)", show_ns_for_cycles=True)
    histogram(b2, f"{upper} PHASE-2 BRACKET HISTOGRAM (cycles)", show_ns_for_cycles=True)
    topology_histogram(topo, "BRACKET TOPOLOGY PAIRS")
    cluster_block(b1, "PHASE-1 BRACKET CLUSTERS")
    cluster_block(b2, "PHASE-2 BRACKET CLUSTERS")

    print_stats_block("RAW PHASE PAIR DELTA (phase2 - phase1)", d_raw, unit="ns")
    histogram(d_raw, f"{upper} RAW PHASE PAIR DELTA HISTOGRAM (ns)")
    print_stats_block("ADJUSTED PHASE PAIR DELTA (phase2 - phase1)", d_adj, unit="ns")
    histogram(d_adj, f"{upper} ADJUSTED PHASE PAIR DELTA HISTOGRAM (ns)")

    print_stats_block("PHASE-1 BIAS DISTRIBUTION", x1, unit="ns")
    histogram(x1, f"{upper} PHASE-1 BIAS HISTOGRAM (ns)")
    print_stats_block("PHASE-2 BIAS DISTRIBUTION", x2, unit="ns")
    histogram(x2, f"{upper} PHASE-2 BIAS HISTOGRAM (ns)")

    print_stats_block("RESIDUAL STABILITY", residuals, unit="ns")

    print("-" * 78)
    print("RAW ELAPSED STABILITY")
    print("-" * 78)
    print()
    e1s = to_stats(e1)
    e2s = to_stats(e2)
    print(f"  phase1 elapsed mean    = {e1s.mean:,.2f} ns")
    print(f"  phase1 elapsed stddev  = {e1s.stddev:,.2f} ns")
    print(f"  phase1 elapsed range   = {int(e1s.range):,} ns")
    print()
    print(f"  phase2 elapsed mean    = {e2s.mean:,.2f} ns")
    print(f"  phase2 elapsed stddev  = {e2s.stddev:,.2f} ns")
    print(f"  phase2 elapsed range   = {int(e2s.range):,} ns")
    print()

    print("-" * 78)
    print("PER-SECOND DETAIL (first 30)")
    print("-" * 78)
    print()
    print(
        f"  {'pps':>6s}  {'R1':>4s} {'R2':>4s} {'ΔR':>5s}  {'A1':>4s} {'A2':>4s} {'ΔA':>5s}  "
        f"{'E1':>6s} {'E2':>6s}  {'B1':>4s} {'B2':>4s}  {'X1':>4s} {'X2':>4s}  {'RES':>6s}  {'PAIR':>4s}"
    )
    print(
        f"  {'─'*6}  {'─'*4} {'─'*4} {'─'*5}  {'─'*4} {'─'*4} {'─'*5}  "
        f"{'─'*6} {'─'*6}  {'─'*4} {'─'*4}  {'─'*4} {'─'*4}  {'─'*6}  {'─'*4}"
    )

    shown = 0
    for s in samples:
        print(
            f"  {s['pps_count']:>6d}  "
            f"{s[f'{key}_raw_phase_offset_ns']:>4d} {s[f'{key}_phase2_raw_phase_offset_ns']:>4d} {s[f'{key}_phase_pair_delta_ns']:+5d}  "
            f"{s[f'{key}_phase_offset_ns']:>4d} {s[f'{key}_phase2_phase_offset_ns']:>4d} {s[f'{key}_adjusted_phase_pair_delta_ns']:+5d}  "
            f"{s[f'{key}_raw_elapsed_ns']:>6d} {s[f'{key}_phase2_raw_elapsed_ns']:>6d}  "
            f"{s[f'{key}_dwt_bracket_cycles']:>4d} {s[f'{key}_phase2_dwt_bracket_cycles']:>4d}  "
            f"{s[f'{key}_phase_bias_ns']:+4d} {s[f'{key}_phase2_phase_bias_ns']:+4d}  "
            f"{s[f'{key}_residual_ns']:+6d}  {'YES' if s['phase_pair_valid'] else 'NO':>4s}"
        )
        shown += 1
        if shown >= 30:
            rem = len(samples) - shown
            if rem > 0:
                print(f"  ... {rem} more rows ...")
            break
    print()


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print("=" * 78)
    print("ZPNet EDGE TDC ANALYZER")
    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE records)")
    print("=" * 78)
    print()

    analyze_ocxo(rows, "ocxo1", limit=limit)
    analyze_ocxo(rows, "ocxo2", limit=limit)

    print("=" * 78)
    print("SUMMARY")
    print("=" * 78)
    print()
    print("  Goal:")
    print("    Surface the new two-phase bracket topology and show how raw phase,")
    print("    adjusted phase, and bias maps relate to the observed bracket classes.")
    print()
    print("  Things to watch:")
    print("    - phase-1 and phase-2 bracket clusters")
    print("    - stable topology pairs such as (275,256) or (279,256)")
    print("    - raw delta distribution vs adjusted delta distribution")
    print("    - whether bias application collapses adjusted pair deltas")
    print()
    print("=" * 78)


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: edge_tdc_analyzer <campaign_name> [limit]")
        sys.exit(1)
    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
