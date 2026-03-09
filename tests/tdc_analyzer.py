"""
ZPNet TDC Analyzer — Derive TDC correction constants from TIMEBASE data

Usage:
    python -m zpnet.tests.tdc_analyzer <campaign_name>
    .zt tdc_analyzer Test2

Reads all TIMEBASE rows for the named campaign and analyzes Teensy
dispatch / latch latency in the DWT cycle domain.

  The dispatch delta (ISR DWT snapshot − spin loop shadow DWT) tells
  you how many DWT cycles elapsed between the last shadow read and
  the PPS ISR entry.  Two data sources, in priority order:
    1. Raw cycle counts: diag_raw_isr_cyc − diag_raw_shadow_cyc
    2. Fallback: diag_teensy_dispatch_delta_ns back-derived to cycles

At 1008 MHz the delta should cluster at discrete values separated by
the loop cycle count.  The minimum cluster center is the fixed overhead.

This tool:
  1. Extracts delta values from TIMEBASE records
  2. Builds a histogram of delta values
  3. Identifies clusters (discrete peaks)
  4. Computes fixed overhead (minimum cluster center)
  5. Computes loop cycle spacing between clusters
  6. Computes max correction (number of primary clusters)
  7. Outputs recommended C++ constants / correction tables

v5: Pi clock domain removed.  Teensy-only analysis.
    DWT PPS period stability retained for interpolation quality.
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
# Configuration
# ---------------------------------------------------------------------

DWT_FREQ_HZ = 1_008_000_000
DWT_NS_PER_CYCLE = 125.0 / 126.0   # 1008 MHz → 0.992 ns/cycle
MAX_DELTA = 1000

# Reference constants for comparison (update after each recalibration)
REFERENCE_OVERHEAD = 50
REFERENCE_LOOP = 6
REFERENCE_MAX_CORR = 5
REFERENCE_LABEL = "600 MHz"


# ---------------------------------------------------------------------
# Delta extraction
# ---------------------------------------------------------------------

def _extract_delta(r: Dict[str, Any]) -> Tuple[Optional[int], str]:
    """Extract Teensy dispatch delta in DWT cycles.

    Returns (delta_cycles, source) or (None, reason).
    """
    # Priority 1: Raw cycle counts (exact)
    isr_cyc = r.get("diag_raw_isr_cyc")
    shadow_cyc = r.get("diag_raw_shadow_cyc")

    if isr_cyc is not None and shadow_cyc is not None:
        delta = (int(isr_cyc) - int(shadow_cyc)) & 0xFFFFFFFF
        return (delta, "raw") if delta > 0 else (None, "zero")

    # Priority 2: Back-derive from ns
    delta_ns = r.get("diag_teensy_dispatch_delta_ns")
    if delta_ns is None:
        return (None, "null")

    delta_ns = int(delta_ns)
    if delta_ns <= 0:
        return (None, "zero")

    # Back-derive cycles: ns × 126 / 125 (1008 MHz DWT)
    delta_cycles = round(delta_ns * 126 / 125)
    return (delta_cycles, "ns")


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
# TDC analysis
# ---------------------------------------------------------------------

def analyze_tdc(rows: List[Dict[str, Any]]) -> None:
    """Run full TDC analysis on Teensy dispatch delta data."""

    unit = "cycles"
    ns_per = DWT_NS_PER_CYCLE

    print()
    print("=" * 70)
    print(f"TDC ANALYSIS: Teensy (DWT domain, {DWT_FREQ_HZ/1e6:.0f} MHz)")
    print("=" * 70)
    print()

    # --- Extract deltas ---
    deltas: List[int] = []
    raw_count = 0
    ns_count = 0
    skipped_none = 0
    skipped_zero = 0
    skipped_huge = 0

    for r in rows:
        result, source = _extract_delta(r)

        if result is None:
            if source == "null":
                skipped_none += 1
            elif source == "zero":
                skipped_zero += 1
            continue

        if result > MAX_DELTA:
            skipped_huge += 1
            continue

        deltas.append(result)
        if source == "raw":
            raw_count += 1
        else:
            ns_count += 1

    print(f"  Total TIMEBASE records: {len(rows)}")
    print(f"  Valid delta samples:    {len(deltas)}")
    print(f"    From raw {unit}:      {raw_count}")
    print(f"    From ns (fallback):   {ns_count}")
    if ns_count > 0 and raw_count == 0:
        print(f"    ⚠️  All samples from ns fallback — quantization noise expected")
    print(f"  Skipped (null fields):  {skipped_none}")
    print(f"  Skipped (zero):         {skipped_zero}")
    print(f"  Skipped (> {MAX_DELTA} {unit}):  {skipped_huge}")
    print()

    if len(deltas) < 10:
        print(f"  ❌ Insufficient data for TDC analysis (need ≥ 10, have {len(deltas)})")
        return

    # --- Basic statistics ---
    stats = WelfordStats()
    for d in deltas:
        stats.update(float(d))

    print("-" * 70)
    print(f"RAW DELTA STATISTICS ({unit})")
    print("-" * 70)
    print(f"  n      = {stats.n}")
    print(f"  mean   = {stats.mean:.2f} {unit}  ({stats.mean * ns_per:.1f} ns)")
    print(f"  stddev = {stats.stddev:.2f} {unit}  ({stats.stddev * ns_per:.1f} ns)")
    print(f"  min    = {int(stats.min_val)} {unit}  ({int(stats.min_val) * ns_per:.1f} ns)")
    print(f"  max    = {int(stats.max_val)} {unit}  ({int(stats.max_val) * ns_per:.1f} ns)")
    print()

    # --- Histogram ---
    print("-" * 70)
    print("DELTA HISTOGRAM")
    print("-" * 70)

    counter = Counter(deltas)
    all_vals = sorted(counter.keys())
    max_count = max(counter.values())
    bar_width = 50

    last_val = None
    for val in all_vals:
        if last_val is not None and val - last_val > 10:
            print(f"       │ {'':.<{bar_width}s}  ···")
        cnt = counter[val]
        bar_len = int((cnt / max_count) * bar_width)
        bar = "█" * bar_len
        pct = (cnt / len(deltas)) * 100
        ns_approx = val * ns_per
        print(f"  {val:4d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)  [{ns_approx:.1f} ns]")
        last_val = val

    print()

    # --- Cluster detection ---
    print("-" * 70)
    print("CLUSTER ANALYSIS")
    print("-" * 70)

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
        ns_approx = val * ns_per
        print(f"    Cluster at {val:4d} {unit} ({ns_approx:6.1f} ns):  {cnt:5d} hits ({pct:5.1f}%)")

    unclustered = len(deltas) - total_in_clusters
    print(f"\n  In clusters:  {total_in_clusters} ({total_in_clusters / len(deltas) * 100:.1f}%)")
    print(f"  Unclustered:  {unclustered} ({unclustered / len(deltas) * 100:.1f}%)")
    print()

    # --- Derive TDC constants ---
    print("-" * 70)
    print("TDC CONSTANT DERIVATION")
    print("-" * 70)

    cluster_values = [v for v, _ in significant]

    # Separate primary clusters from outliers (gap > 20 units)
    primary = [cluster_values[0]]
    outlier_clusters = []
    for i in range(1, len(cluster_values)):
        if cluster_values[i] - cluster_values[i - 1] > 20:
            outlier_clusters.extend(cluster_values[i:])
            break
        primary.append(cluster_values[i])

    if outlier_clusters:
        print(f"  Primary clusters: {primary}")
        print(f"  Outlier clusters: {outlier_clusters} (late, excluded from derivation)")
        print()

    tdc_fixed_overhead = primary[0]

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
            print(f"  Only one primary cluster — cannot determine loop cycle count")
            print(f"  ℹ️  This may indicate all captures land at the same loop phase")
            print(f"      (possible if the loop is very tight relative to capture jitter)")
        tdc_loop_cycles = None

    tdc_max_correction = len(primary)

    print()
    print(f"  FIXED_OVERHEAD  = {tdc_fixed_overhead} {unit}  ({tdc_fixed_overhead * ns_per:.1f} ns)")
    if tdc_loop_cycles is not None:
        print(f"  LOOP_CYCLES     = {tdc_loop_cycles} {unit}  ({tdc_loop_cycles * ns_per:.1f} ns)")
    else:
        print(f"  LOOP_CYCLES     = (undetermined — need more data or raw counts)")
    print(f"  MAX_CORRECTION  = {tdc_max_correction}  (from {len(primary)} primary clusters)")
    print(f"  Primary clusters = {primary}")
    print(f"  Valid corrections = 0..{tdc_max_correction - 1}  (reject >= {tdc_max_correction})")
    print()

    # --- Correction distribution ---
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
            if d > primary[-1] + 20:
                continue
            adjusted = d - tdc_fixed_overhead
            correction = adjusted % tdc_loop_cycles
            correction_counter[correction] += 1

        for corr in sorted(correction_counter.keys()):
            cnt = correction_counter[corr]
            pct = (cnt / len(deltas)) * 100
            ns_corr = corr * ns_per
            print(f"    correction={corr}: {cnt:5d} ({pct:5.1f}%)  [{ns_corr:.1f} ns]")

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

    elif len(primary) > 1:
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
            ns_corr = corr * ns_per
            print(f"    correction={corr}: {cnt:5d} ({pct_val:5.1f}%)  [{ns_corr:.1f} ns]")

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
        print(f"  ✅ Using raw cycles ({raw_count} samples) — exact, no conversion loss")
        if ns_count > 0:
            print(f"     ({ns_count} samples from ns fallback — mixed precision)")
    else:
        print(f"  ⚠️  All {ns_count} samples from ns fallback — quantization noise")
        print(f"      The histogram may show artificial spreading of clusters")

    # --- Correction precision ---
    print()
    print("-" * 70)
    print("CORRECTION PRECISION")
    print("-" * 70)
    print()
    if tdc_loop_cycles is not None:
        correction_resolution_ns = tdc_loop_cycles * ns_per
        uncorrected_window_ns = tdc_max_correction * tdc_loop_cycles * ns_per
        print(f"  Correction resolution:   {correction_resolution_ns:.2f} ns per correction step")
        print(f"  Uncorrected window:      {uncorrected_window_ns:.2f} ns ({tdc_max_correction} positions × {correction_resolution_ns:.2f} ns)")
        print(f"  With correction:         ±{correction_resolution_ns / 2:.2f} ns (quantization limit)")
    else:
        total_span_ns = (primary[-1] - primary[0]) * ns_per if len(primary) > 1 else 0
        print(f"  Uncorrected window:      {total_span_ns:.2f} ns (span of primary clusters)")
        print(f"  Cannot determine correction resolution without loop spacing")

    # --- Reference comparison ---
    print()
    print("-" * 70)
    print(f"COMPARISON WITH {REFERENCE_LABEL} CONSTANTS")
    print("-" * 70)
    print()
    print(f"  {'':24s} {'Reference':>10s}  {'Current':>10s}  {'Change':>10s}")
    print(f"  {'─' * 24} {'─' * 10}  {'─' * 10}  {'─' * 10}")
    print(f"  {'FIXED_OVERHEAD':24s} {REFERENCE_OVERHEAD:>10d}  {tdc_fixed_overhead:>10d}  {tdc_fixed_overhead - REFERENCE_OVERHEAD:>+10d}")
    if REFERENCE_LOOP is not None and tdc_loop_cycles is not None:
        print(f"  {'LOOP_CYCLES':24s} {REFERENCE_LOOP:>10d}  {tdc_loop_cycles:>10d}  {tdc_loop_cycles - REFERENCE_LOOP:>+10d}")
    if REFERENCE_MAX_CORR is not None:
        print(f"  {'MAX_CORRECTION':24s} {REFERENCE_MAX_CORR:>10d}  {tdc_max_correction:>10d}  {tdc_max_correction - REFERENCE_MAX_CORR:>+10d}")

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
        print(f"  static constexpr uint32_t TDC_LOOP_CYCLES      = 6;   // PLACEHOLDER")
    print(f"  static constexpr uint32_t TDC_MAX_CORRECTION   = {tdc_max_correction};")


# ---------------------------------------------------------------------
# DWT PPS period stability (interpolation quality)
# ---------------------------------------------------------------------

def analyze_period_stability(rows: List[Dict[str, Any]]) -> None:
    """Analyze DWT PPS period stability for sub-second interpolation quality."""

    print()
    print()
    print("=" * 70)
    print("DWT PPS PERIOD STABILITY (INTERPOLATION BASE)")
    print("=" * 70)
    print()

    # Extract consecutive shadow pairs
    teensy_shadows = []
    for r in rows:
        sc = r.get("diag_raw_shadow_cyc")
        if sc is not None:
            teensy_shadows.append(int(sc))

    periods = []
    for i in range(1, len(teensy_shadows)):
        diff = (teensy_shadows[i] - teensy_shadows[i - 1]) & 0xFFFFFFFF
        # Sanity: should be close to 1,008,000,000 (± 0.1%)
        if 1_007_000_000 < diff < 1_009_000_000:
            periods.append(diff)

    if len(periods) < 2:
        print(f"  ❌ Insufficient data ({len(periods)} periods, need ≥ 2)")
        print(f"     from {len(teensy_shadows)} consecutive shadow_cyc values")
        return

    stats = WelfordStats()
    for p in periods:
        stats.update(float(p))

    mean_period = stats.mean
    std_period = stats.stddev
    mean_ns = mean_period * DWT_NS_PER_CYCLE
    std_ns = std_period * DWT_NS_PER_CYCLE

    # Drift from nominal in PPB
    drift_ppb = ((mean_period - DWT_FREQ_HZ) / DWT_FREQ_HZ) * 1e9

    # Allan-deviation-like: RMS of consecutive period differences
    consec_diffs = [float(periods[i] - periods[i - 1]) for i in range(1, len(periods))]
    if consec_diffs:
        adev_like = math.sqrt(sum(d * d for d in consec_diffs) / (2 * len(consec_diffs)))
        adev_ns = adev_like * DWT_NS_PER_CYCLE
        adev_ppb = (adev_like / DWT_FREQ_HZ) * 1e9
    else:
        adev_like = adev_ns = adev_ppb = None

    print(f"  Samples:           {len(periods)}  (from {len(teensy_shadows)} consecutive shadow_cyc diffs)")
    print(f"  Nominal frequency: {DWT_FREQ_HZ:,} Hz")
    print()

    print("-" * 70)
    print("PERIOD STATISTICS")
    print("-" * 70)
    print(f"  mean   = {mean_period:,.2f} cycles  ({mean_ns:,.2f} ns)")
    print(f"  stddev = {std_period:,.4f} cycles  ({std_ns:,.2f} ns)")
    print(f"  min    = {int(stats.min_val):,} cycles")
    print(f"  max    = {int(stats.max_val):,} cycles")
    print(f"  range  = {int(stats.max_val) - int(stats.min_val):,} cycles"
          f"  ({(int(stats.max_val) - int(stats.min_val)) * DWT_NS_PER_CYCLE:,.1f} ns)")
    print()
    print(f"  Mean drift from nominal: {drift_ppb:+.3f} PPB")
    print(f"  Period stddev:           {std_ns:.2f} ns  ({std_period / DWT_FREQ_HZ * 1e9:.3f} PPB)")
    print()

    if adev_like is not None:
        print(f"  Successive-diff RMS:     {adev_ns:.2f} ns  ({adev_ppb:.3f} PPB)")
        print(f"    (Allan deviation proxy at τ=1s)")
        print()

    # --- Interpolation quality ---
    print("-" * 70)
    print("INTERPOLATION QUALITY")
    print("-" * 70)
    print()

    mid_err_ns = std_ns / 2.0
    quant_ns = DWT_NS_PER_CYCLE
    total_err = math.sqrt(mid_err_ns**2 + (quant_ns / 2)**2)

    print(f"  Mid-second interpolation error:  ±{mid_err_ns:.2f} ns (from period stddev)")
    print(f"  TDC quantization:                ±{quant_ns / 2:.2f} ns (DWT @ {DWT_FREQ_HZ/1e6:.0f} MHz)")
    print(f"  Combined (RSS):                  ±{total_err:.2f} ns")
    print()

    tau = mean_period / DWT_FREQ_HZ
    print(f"  τ (ticks-per-second ratio):      {tau:.12f}")
    print(f"  τ deviation from unity:          {(tau - 1.0) * 1e9:+.3f} PPB")
    print()

    # --- Period histogram ---
    offsets = [p - int(round(mean_period)) for p in periods]
    offset_counter = Counter(offsets)
    offset_vals = sorted(offset_counter.keys())

    if len(offset_vals) <= 40:
        print("-" * 70)
        print(f"PERIOD HISTOGRAM (offset from {int(round(mean_period)):,})")
        print("-" * 70)

        max_cnt = max(offset_counter.values())
        bar_width = 50
        for val in offset_vals:
            cnt = offset_counter[val]
            bar_len = int((cnt / max_cnt) * bar_width) if max_cnt > 0 else 0
            bar = "█" * bar_len
            pct = (cnt / len(periods)) * 100
            ns_off = val * DWT_NS_PER_CYCLE
            print(f"  {val:+5d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)  [{ns_off:+.1f} ns]")
        print()
    else:
        print(f"  (Period spread too wide for histogram: {len(offset_vals)} distinct values)")
        print()


# ---------------------------------------------------------------------
# Top-level analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"❌ No TIMEBASE rows found for campaign '{campaign}'")
        return

    print("=" * 70)
    print(f"ZPNet TDC ANALYZER — Campaign: {campaign}")
    print(f"  Total TIMEBASE records: {len(rows)}")
    print(f"  Three clock domains: GNSS (reference), DWT, OCXO")
    print("=" * 70)

    # --- TDC analysis ---
    analyze_tdc(rows)

    # --- Period stability ---
    analyze_period_stability(rows)

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
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s} {'RAW_CYC':>8s} {'NS':>8s}")
                print(f"  {'─' * 20} {'─' * 8} {'─' * 8} {'─' * 8}")
                for r in rows:
                    print(
                        f"  {r['campaign']:<20s} {r['cnt']:>8d} "
                        f"{r['has_raw']:>8d} {r['has_ns']:>8d}"
                    )
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    analyze(campaign)


if __name__ == "__main__":
    main()