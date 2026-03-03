"""
ZPNet TDC Analyzer — Derive TDC correction constants from TIMEBASE data

Usage:
    python -m zpnet.tests.tdc_analyzer <campaign_name>
    .zt tdc_analyzer Test2

Reads all TIMEBASE rows for the named campaign and analyzes the
dispatch delta (ISR DWT snapshot - spin loop shadow DWT) to derive
the TDC correction constants for the current clock speed.

Two data sources, in priority order:

  1. Raw cycle counts: diag_raw_isr_cyc - diag_raw_shadow_cyc
     (exact, no conversion loss — requires Pi-side passthrough)

  2. Fallback: diag_teensy_dispatch_delta_ns back-derived to cycles
     via cycles = round(ns * 126 / 125).  Has ±1 cycle quantization
     noise from the integer division round trip.

At any clock speed, the delta should cluster at discrete values
separated by the loop cycle count (TDC_LOOP_CYCLES).  The minimum
cluster center is the fixed overhead (TDC_FIXED_OVERHEAD).

This tool:
  1. Extracts delta_cycles from raw counts (preferred) or ns (fallback)
  2. Builds a histogram of delta values
  3. Identifies clusters (discrete peaks)
  4. Computes TDC_FIXED_OVERHEAD (minimum cluster center)
  5. Computes TDC_LOOP_CYCLES (spacing between clusters)
  6. Computes TDC_MAX_CORRECTION (number of primary clusters = distinct
     landing positions within one loop iteration)
  7. Outputs recommended C++ constants
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from functools import reduce
from math import gcd
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


# ---------------------------------------------------------------------
# Fetch campaign TIMEBASE rows
# ---------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch all TIMEBASE rows for a campaign, ordered by pps_count."""
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

    result = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        result.append(p)

    return result


# ---------------------------------------------------------------------
# Welford's online stats
# ---------------------------------------------------------------------

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


# ---------------------------------------------------------------------
# Cluster detection
# ---------------------------------------------------------------------

def find_clusters(
    values: List[int],
    min_count: int = 3,
) -> List[Tuple[int, int]]:
    """
    Find discrete clusters in a list of integer values.

    Returns list of (value, count) for values that appear at least
    min_count times, sorted by value.
    """
    counter = Counter(values)
    clusters = [
        (val, cnt)
        for val, cnt in counter.items()
        if cnt >= min_count
    ]
    clusters.sort(key=lambda x: x[0])
    return clusters


def compute_spacings(clusters: List[Tuple[int, int]]) -> List[int]:
    """Compute spacings between consecutive cluster centers."""
    if len(clusters) < 2:
        return []
    return [clusters[i + 1][0] - clusters[i][0] for i in range(len(clusters) - 1)]


# ---------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"❌ No TIMEBASE rows found for campaign '{campaign}'")
        return

    print("=" * 70)
    print(f"TDC ANALYSIS: {campaign}")
    print("=" * 70)
    print()
    print(f"  Total TIMEBASE records: {len(rows)}")

    # --- Extract delta cycles ---
    #
    # Priority 1: Raw cycle counts (exact).
    #   delta = (isr_cyc - shadow_cyc) & 0xFFFFFFFF  (unsigned 32-bit)
    #
    # Priority 2: Back-derive from dispatch_delta_ns (±1 cycle noise).
    #   cycles = round(ns * 126 / 125)
    #
    deltas: List[int] = []
    raw_count = 0
    ns_count = 0
    skipped_none = 0
    skipped_zero = 0
    skipped_huge = 0

    for r in rows:
        # Try raw cycle counts first
        isr_cyc = r.get("diag_raw_isr_cyc")
        shadow_cyc = r.get("diag_raw_shadow_cyc")

        if isr_cyc is not None and shadow_cyc is not None:
            delta = (int(isr_cyc) - int(shadow_cyc)) & 0xFFFFFFFF
            if delta == 0:
                skipped_zero += 1
                continue
            if delta > 1000:
                skipped_huge += 1
                continue
            deltas.append(delta)
            raw_count += 1
            continue

        # Fallback: back-derive from ns
        delta_ns = r.get("diag_teensy_dispatch_delta_ns")

        if delta_ns is None:
            skipped_none += 1
            continue

        delta_ns = int(delta_ns)

        if delta_ns <= 0:
            skipped_zero += 1
            continue

        # Back-derive cycles: ns * 126 / 125
        delta_cycles = round(delta_ns * 126 / 125)

        if delta_cycles > 1000:
            skipped_huge += 1
            continue

        deltas.append(delta_cycles)
        ns_count += 1

    print(f"  Valid delta samples:    {len(deltas)}")
    print(f"    From raw cycles:      {raw_count}")
    print(f"    From ns (fallback):   {ns_count}")
    if ns_count > 0 and raw_count == 0:
        print(f"    ⚠️  All samples from ns fallback — ±1 cycle quantization noise expected")
        print(f"       Deploy Pi-side diag_raw_isr_cyc/diag_raw_shadow_cyc for exact data")
    print(f"  Skipped (null fields):  {skipped_none}")
    print(f"  Skipped (zero):         {skipped_zero}")
    print(f"  Skipped (> 1000 cyc):   {skipped_huge}")
    print()

    if len(deltas) < 10:
        print("❌ Insufficient data for TDC analysis")
        return

    # --- Basic statistics ---
    stats = WelfordStats()
    for d in deltas:
        stats.update(float(d))

    print("-" * 70)
    print("RAW DELTA STATISTICS (cycles)")
    print("-" * 70)
    print(f"  n      = {stats.n}")
    print(f"  mean   = {stats.mean:.2f}")
    print(f"  stddev = {stats.stddev:.2f}")
    print(f"  min    = {int(stats.min_val)}")
    print(f"  max    = {int(stats.max_val)}")
    print()

    # --- Histogram ---
    print("-" * 70)
    print("DELTA HISTOGRAM")
    print("-" * 70)

    counter = Counter(deltas)
    all_vals = sorted(counter.keys())
    max_count = max(counter.values())
    bar_width = 50

    # Show all values but visually separate gaps > 10
    last_val = None
    for val in all_vals:
        if last_val is not None and val - last_val > 10:
            print(f"       │ {'':.<{bar_width}s}  ···")
        cnt = counter[val]
        bar_len = int((cnt / max_count) * bar_width)
        bar = "█" * bar_len
        pct = (cnt / len(deltas)) * 100
        print(f"  {val:4d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)")
        last_val = val

    print()

    # --- Cluster detection ---
    print("-" * 70)
    print("CLUSTER ANALYSIS")
    print("-" * 70)

    # Adaptive threshold: at least 1% of samples or 3 hits
    min_hits = max(3, len(deltas) // 100)
    significant = find_clusters(deltas, min_count=min_hits)

    if not significant:
        significant = find_clusters(deltas, min_count=2)

    if not significant:
        print("  ❌ No significant clusters found")
        return

    print(f"  Significant clusters (≥ {min_hits} hits):")
    print()

    total_in_clusters = 0
    for val, cnt in significant:
        pct = (cnt / len(deltas)) * 100
        total_in_clusters += cnt
        ns_approx = val * 125 / 126
        print(f"    Cluster at {val:4d} cycles ({ns_approx:6.1f} ns):  {cnt:5d} hits ({pct:5.1f}%)")

    unclustered = len(deltas) - total_in_clusters
    print(f"\n  In clusters:  {total_in_clusters} ({total_in_clusters / len(deltas) * 100:.1f}%)")
    print(f"  Unclustered:  {unclustered} ({unclustered / len(deltas) * 100:.1f}%)")
    print()

    # --- Derive TDC constants ---
    print("-" * 70)
    print("TDC CONSTANT DERIVATION")
    print("-" * 70)

    cluster_values = [v for v, _ in significant]

    # Separate primary clusters (the main group) from outliers.
    # The main group is the tightly-spaced set; outliers are far away.
    # Use a gap threshold of 20 cycles to distinguish.
    primary = [cluster_values[0]]
    outlier_clusters = []
    for i in range(1, len(cluster_values)):
        if cluster_values[i] - cluster_values[i - 1] > 20:
            outlier_clusters.extend(cluster_values[i:])
            break
        primary.append(cluster_values[i])

    if outlier_clusters:
        print(f"  Primary clusters: {primary}")
        print(f"  Outlier clusters: {outlier_clusters} (timer late, excluded from derivation)")
        print()

    # TDC_FIXED_OVERHEAD = minimum primary cluster center
    tdc_fixed_overhead = primary[0]

    # TDC_LOOP_CYCLES = spacing between primary clusters
    primary_clusters = [(v, c) for v, c in significant if v in primary]
    spacings = compute_spacings(primary_clusters)

    if spacings:
        spacing_counter = Counter(spacings)
        most_common_spacing = spacing_counter.most_common(1)[0][0]
        spacing_gcd = reduce(gcd, spacings)

        print(f"  Primary cluster spacings: {spacings}")
        print(f"  Most common spacing: {most_common_spacing}")
        print(f"  GCD of spacings: {spacing_gcd}")
        print()

        tdc_loop_cycles = most_common_spacing

        irregular = [s for s in spacings if s % tdc_loop_cycles != 0]
        if irregular:
            print(f"  ⚠️  Irregular spacings (not multiples of {tdc_loop_cycles}): {irregular}")
            print(f"      Using GCD ({spacing_gcd}) instead")
            tdc_loop_cycles = spacing_gcd
    else:
        if len(primary) == 1:
            print("  Only one primary cluster — cannot determine loop cycle count")
            print("  ℹ️  This may indicate all interrupts land at the same loop phase")
            print("      (possible if the loop is very tight relative to interrupt jitter)")
        tdc_loop_cycles = None

    # TDC_MAX_CORRECTION = number of primary clusters.
    #
    # Each primary cluster represents a distinct landing position within
    # one loop iteration.  The correction value for each is:
    #   correction = delta_cycles - TDC_FIXED_OVERHEAD
    # which ranges from 0 to (len(primary) - 1).  We reject any
    # correction >= len(primary).
    #
    # At 600 MHz: 6 primary clusters (overhead=50, values 50-55, loop=6)
    #   → corrections 0-5, reject >=6, so MAX_CORRECTION=6
    #
    # At 1008 MHz: 5 primary clusters (overhead=48, values 48-52, loop=1)
    #   → corrections 0-4, reject >=5, so MAX_CORRECTION=5
    #
    # This is correct regardless of TDC_LOOP_CYCLES because the number
    # of valid landing positions is a property of the interrupt pipeline,
    # not the loop structure.
    tdc_max_correction = len(primary)

    print()
    print(f"  TDC_FIXED_OVERHEAD = {tdc_fixed_overhead}")
    if tdc_loop_cycles is not None:
        print(f"  TDC_LOOP_CYCLES    = {tdc_loop_cycles}")
    else:
        print(f"  TDC_LOOP_CYCLES    = (undetermined — need raw cycle data)")
    print(f"  TDC_MAX_CORRECTION = {tdc_max_correction}  (from {len(primary)} primary clusters)")
    print(f"  Primary clusters   = {primary}")
    print(f"  Valid corrections  = 0..{tdc_max_correction - 1}  (reject >= {tdc_max_correction})")
    print()

    # --- Verify correction distribution ---
    if tdc_loop_cycles and tdc_loop_cycles > 0:
        print("-" * 70)
        print("CORRECTION DISTRIBUTION")
        print("-" * 70)
        print()
        print(f"  correction = (delta - {tdc_fixed_overhead}) % {tdc_loop_cycles}")
        print()

        correction_counter: Counter = Counter()
        out_of_range = 0

        for d in deltas:
            if d < tdc_fixed_overhead:
                out_of_range += 1
                continue
            # Exclude outliers (> 20 cycles above last primary cluster)
            if d > primary[-1] + 20:
                continue
            adjusted = d - tdc_fixed_overhead
            correction = adjusted % tdc_loop_cycles
            correction_counter[correction] += 1

        for corr in sorted(correction_counter.keys()):
            cnt = correction_counter[corr]
            pct = (cnt / len(deltas)) * 100
            print(f"    correction={corr}: {cnt:5d} ({pct:5.1f}%)")

        if out_of_range:
            print(f"\n    Below overhead: {out_of_range}")

        # Uniformity check
        if len(correction_counter) > 1:
            counts = list(correction_counter.values())
            mean_cnt = sum(counts) / len(counts)
            max_dev = max(abs(c - mean_cnt) / mean_cnt for c in counts) * 100
            if max_dev < 20:
                print(f"\n  ✅ Corrections roughly uniform (max deviation {max_dev:.0f}% from mean)")
            else:
                print(f"\n  ⚠️  Corrections non-uniform (max deviation {max_dev:.0f}% from mean)")

    elif len(primary) > 1:
        # TDC_LOOP_CYCLES undetermined but we have multiple clusters —
        # show direct correction values (delta - overhead)
        print("-" * 70)
        print("CORRECTION DISTRIBUTION (direct, no modulo)")
        print("-" * 70)
        print()
        print(f"  correction = delta - {tdc_fixed_overhead}")
        print()

        correction_counter = Counter()
        out_of_range = 0

        for d in deltas:
            if d < tdc_fixed_overhead:
                out_of_range += 1
                continue
            if d > primary[-1] + 20:
                continue
            correction = d - tdc_fixed_overhead
            correction_counter[correction] += 1

        for corr in sorted(correction_counter.keys()):
            cnt = correction_counter[corr]
            pct_val = (cnt / len(deltas)) * 100
            print(f"    correction={corr}: {cnt:5d} ({pct_val:5.1f}%)")

        if out_of_range:
            print(f"\n    Below overhead: {out_of_range}")

        if len(correction_counter) > 1:
            counts = list(correction_counter.values())
            mean_cnt = sum(counts) / len(counts)
            max_dev = max(abs(c - mean_cnt) / mean_cnt for c in counts) * 100
            if max_dev < 20:
                print(f"\n  ✅ Corrections roughly uniform (max deviation {max_dev:.0f}% from mean)")
            else:
                print(f"\n  ⚠️  Corrections non-uniform (max deviation {max_dev:.0f}% from mean)")

    # --- Data quality assessment ---
    print()
    print("-" * 70)
    print("DATA QUALITY")
    print("-" * 70)
    print()
    if raw_count > 0:
        print(f"  ✅ Using raw cycle counts ({raw_count} samples) — exact, no conversion loss")
        if ns_count > 0:
            print(f"     ({ns_count} samples from ns fallback — mixed precision)")
    else:
        print(f"  ⚠️  All {ns_count} samples from ns fallback — ±1 cycle quantization noise")
        print(f"      The histogram may show artificial spreading of clusters")
        print(f"      Deploy diag_raw_isr_cyc/diag_raw_shadow_cyc for definitive results")

    # --- Recommended C++ constants ---
    print()
    print("-" * 70)
    print("RECOMMENDED C++ CONSTANTS")
    print("-" * 70)
    print()

    if raw_count == 0 and tdc_loop_cycles and tdc_loop_cycles <= 1:
        print("  ⚠️  Loop cycle count unreliable from ns-derived data.")
        print("      Re-run with raw cycle data before updating firmware.")
        print()
        print(f"  // PROVISIONAL — re-derive with raw cycle data")
        print(f"  static constexpr bool     TDC_NEEDS_RECALIBRATION = true;")
    else:
        print(f"  static constexpr bool     TDC_NEEDS_RECALIBRATION = false;")
    print()
    print(f"  static constexpr uint32_t TDC_FIXED_OVERHEAD   = {tdc_fixed_overhead};")
    if tdc_loop_cycles is not None:
        print(f"  static constexpr uint32_t TDC_LOOP_CYCLES      = {tdc_loop_cycles};")
    else:
        print(f"  static constexpr uint32_t TDC_LOOP_CYCLES      = 6;   // PLACEHOLDER — undetermined")
    print(f"  static constexpr uint32_t TDC_MAX_CORRECTION   = {tdc_max_correction};")
    print()

    # --- Compare with old 600 MHz constants ---
    print("-" * 70)
    print("COMPARISON WITH 600 MHz CONSTANTS")
    print("-" * 70)
    print()
    print(f"  {'':20s} {'600 MHz':>10s}  {'1008 MHz':>10s}  {'Change':>10s}")
    print(f"  {'─' * 20} {'─' * 10}  {'─' * 10}  {'─' * 10}")
    print(f"  {'TDC_FIXED_OVERHEAD':20s} {'50':>10s}  {tdc_fixed_overhead:>10d}  {tdc_fixed_overhead - 50:>+10d}")
    if tdc_loop_cycles is not None:
        print(f"  {'TDC_LOOP_CYCLES':20s} {'6':>10s}  {tdc_loop_cycles:>10d}  {tdc_loop_cycles - 6:>+10d}")
    print(f"  {'TDC_MAX_CORRECTION':20s} {'5':>10s}  {tdc_max_correction:>10d}  {tdc_max_correction - 5:>+10d}")
    print()

    print(f"  At 600 MHz:  overhead = 50 cyc × 1.667 ns = {50 * 5 / 3:.1f} ns")
    if tdc_loop_cycles is not None:
        print(f"  At 1008 MHz: overhead = {tdc_fixed_overhead} cyc × 0.992 ns = {tdc_fixed_overhead * 125 / 126:.1f} ns")
        print(f"  At 600 MHz:  loop    = 6 cyc × 1.667 ns = {6 * 5 / 3:.1f} ns")
        print(f"  At 1008 MHz: loop    = {tdc_loop_cycles} cyc × 0.992 ns = {tdc_loop_cycles * 125 / 126:.1f} ns")

    # --- Correction precision summary ---
    print()
    print("-" * 70)
    print("CORRECTION PRECISION")
    print("-" * 70)
    print()
    if tdc_loop_cycles is not None:
        correction_resolution_ns = tdc_loop_cycles * 125 / 126
        uncorrected_window_ns = tdc_max_correction * 125 / 126
        print(f"  Correction resolution:   {correction_resolution_ns:.2f} ns per correction step")
        print(f"  Uncorrected ISR window:  {uncorrected_window_ns:.2f} ns ({tdc_max_correction} positions × {correction_resolution_ns:.2f} ns)")
        print(f"  With TDC correction:     ±{correction_resolution_ns / 2:.2f} ns (quantization limit)")

    print()
    print("=" * 70)
    print("DONE")
    print("=" * 70)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: tdc_analyzer <campaign_name>")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, count(*) as cnt,
                           count(*) FILTER (
                               WHERE (payload->>'diag_raw_isr_cyc') IS NOT NULL
                           ) as has_raw,
                           count(*) FILTER (
                               WHERE (payload->>'diag_teensy_dispatch_delta_ns') IS NOT NULL
                           ) as has_ns
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s} {'RAW CYC':>8s} {'NS DIAG':>8s}")
                print(f"  {'─' * 20} {'─' * 8} {'─' * 8} {'─' * 8}")
                for r in rows:
                    print(f"  {r['campaign']:<20s} {r['cnt']:>8d} {r['has_raw']:>8d} {r['has_ns']:>8d}")
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    analyze(campaign)


if __name__ == "__main__":
    main()