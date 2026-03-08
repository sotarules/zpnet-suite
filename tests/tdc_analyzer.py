"""
ZPNet TDC Analyzer — Derive TDC correction constants from TIMEBASE data

Usage:
    python -m zpnet.tests.tdc_analyzer <campaign_name>
    .zt tdc_analyzer Test2

Reads all TIMEBASE rows for the named campaign and analyzes dispatch /
latch latency for BOTH the Teensy and Pi timing instruments:

  PASS 1 — Teensy (DWT cycle domain)
    The dispatch delta (ISR DWT snapshot − spin loop shadow DWT) tells
    you how many DWT cycles elapsed between the last shadow read and
    the PPS ISR entry.  Two data sources, in priority order:
      1. Raw cycle counts: diag_raw_isr_cyc − diag_raw_shadow_cyc
      2. Fallback: diag_teensy_dispatch_delta_ns back-derived to cycles

  PASS 2 — Pi (CNTVCT tick domain)
    The latch delta (captured CNTVCT − shadow CNTVCT from the C spin
    loop on core 3) tells you how many 54 MHz ticks elapsed between
    the last shadow read and the sacred PPS capture.
      1. Primary: diag_pi_latch_delta_ticks (direct from ppslatch ring)
      2. Fallback: diag_pi_latch_delta_ns back-derived to ticks

At any clock speed, the delta should cluster at discrete values
separated by the loop cycle count.  The minimum cluster center is
the fixed overhead.

This tool:
  1. Extracts delta values for each instrument
  2. Builds a histogram of delta values
  3. Identifies clusters (discrete peaks)
  4. Computes fixed overhead (minimum cluster center)
  5. Computes loop cycle spacing between clusters
  6. Computes max correction (number of primary clusters)
  7. Outputs recommended constants / correction tables
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
# Instrument profiles
# ---------------------------------------------------------------------

class InstrumentProfile:
    """Describes how to extract and interpret TDC data for one instrument."""

    def __init__(
        self,
        name: str,
        domain: str,
        unit: str,
        freq_hz: float,
        ns_per_unit: float,
        extract_fn,
        max_delta: int = 1000,
        reference_overhead: Optional[int] = None,
        reference_loop: Optional[int] = None,
        reference_max_corr: Optional[int] = None,
        reference_label: Optional[str] = None,
    ):
        self.name = name
        self.domain = domain
        self.unit = unit
        self.freq_hz = freq_hz
        self.ns_per_unit = ns_per_unit
        self.extract_fn = extract_fn
        self.max_delta = max_delta
        self.reference_overhead = reference_overhead
        self.reference_loop = reference_loop
        self.reference_max_corr = reference_max_corr
        self.reference_label = reference_label


def _extract_teensy(r: Dict[str, Any]) -> Tuple[Optional[int], str]:
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


def _extract_pi(r: Dict[str, Any]) -> Tuple[Optional[int], str]:
    """Extract Pi latch delta in CNTVCT ticks.

    Returns (delta_ticks, source) or (None, reason).
    """
    # Priority 1: Direct latch delta ticks from ppslatch ring
    delta_ticks = r.get("diag_pi_latch_delta_ticks")
    if delta_ticks is not None:
        delta_ticks = int(delta_ticks)
        return (delta_ticks, "raw") if delta_ticks > 0 else (None, "zero")

    # Priority 2: Back-derive from ns
    delta_ns = r.get("diag_pi_latch_delta_ns")
    if delta_ns is None:
        return (None, "null")

    delta_ns = int(delta_ns)
    if delta_ns <= 0:
        return (None, "zero")

    # Back-derive ticks: ns × 54 / 1000
    delta_ticks = round(delta_ns * 54 / 1000)
    return (delta_ticks, "ns")


TEENSY_PROFILE = InstrumentProfile(
    name="Teensy",
    domain="DWT",
    unit="cycles",
    freq_hz=1_008_000_000,
    ns_per_unit=125 / 126,      # 1008 MHz → 0.992 ns/cycle
    extract_fn=_extract_teensy,
    max_delta=1000,
    reference_overhead=50,
    reference_loop=6,
    reference_max_corr=5,
    reference_label="600 MHz",
)

PI_PROFILE = InstrumentProfile(
    name="Pi",
    domain="CNTVCT",
    unit="ticks",
    freq_hz=54_000_000,
    ns_per_unit=1e9 / 54_000_000,   # 54 MHz → 18.519 ns/tick
    extract_fn=_extract_pi,
    max_delta=200,              # Pi ticks are coarser; 200 ticks ≈ 3.7 µs
)


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
# Single-instrument analysis pass
# ---------------------------------------------------------------------

def analyze_instrument(
    rows: List[Dict[str, Any]],
    profile: InstrumentProfile,
) -> None:
    """Run full TDC analysis for one instrument."""

    name = profile.name
    unit = profile.unit
    ns_per = profile.ns_per_unit

    print()
    print("=" * 70)
    print(f"TDC ANALYSIS: {name} ({profile.domain} domain, {profile.freq_hz/1e6:.0f} MHz)")
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
        result, source = profile.extract_fn(r)

        if result is None:
            if source == "null":
                skipped_none += 1
            elif source == "zero":
                skipped_zero += 1
            continue

        if result > profile.max_delta:
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
    print(f"  Skipped (> {profile.max_delta} {unit}):  {skipped_huge}")
    print()

    if len(deltas) < 10:
        print(f"  ❌ Insufficient data for {name} TDC analysis (need ≥ 10, have {len(deltas)})")
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
            print(f"  Only one primary cluster — cannot determine loop {unit} count")
            print(f"  ℹ️  This may indicate all captures land at the same loop phase")
            print(f"      (possible if the loop is very tight relative to capture jitter)")
        tdc_loop_cycles = None

    tdc_max_correction = len(primary)

    print()
    print(f"  FIXED_OVERHEAD  = {tdc_fixed_overhead} {unit}  ({tdc_fixed_overhead * ns_per:.1f} ns)")
    if tdc_loop_cycles is not None:
        print(f"  LOOP_{unit.upper()}   = {tdc_loop_cycles} {unit}  ({tdc_loop_cycles * ns_per:.1f} ns)")
    else:
        print(f"  LOOP_{unit.upper()}   = (undetermined — need more data or raw counts)")
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
        print(f"  ✅ Using raw {unit} ({raw_count} samples) — exact, no conversion loss")
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
    if profile.reference_overhead is not None:
        print()
        print("-" * 70)
        print(f"COMPARISON WITH {profile.reference_label or 'REFERENCE'} CONSTANTS")
        print("-" * 70)
        print()
        ref_oh = profile.reference_overhead
        ref_loop = profile.reference_loop
        ref_max = profile.reference_max_corr

        print(f"  {'':24s} {'Reference':>10s}  {'Current':>10s}  {'Change':>10s}")
        print(f"  {'─' * 24} {'─' * 10}  {'─' * 10}  {'─' * 10}")
        print(f"  {'FIXED_OVERHEAD':24s} {ref_oh:>10d}  {tdc_fixed_overhead:>10d}  {tdc_fixed_overhead - ref_oh:>+10d}")
        if ref_loop is not None and tdc_loop_cycles is not None:
            print(f"  {'LOOP_' + unit.upper():24s} {ref_loop:>10d}  {tdc_loop_cycles:>10d}  {tdc_loop_cycles - ref_loop:>+10d}")
        if ref_max is not None:
            print(f"  {'MAX_CORRECTION':24s} {ref_max:>10d}  {tdc_max_correction:>10d}  {tdc_max_correction - ref_max:>+10d}")

    # --- Recommended constants (Teensy only — C++ style) ---
    if name == "Teensy":
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

    # --- Recommended constants (Pi — Python style) ---
    if name == "Pi":
        print()
        print("-" * 70)
        print("RECOMMENDED Pi CONSTANTS")
        print("-" * 70)
        print()

        print(f"  PI_LATCH_FIXED_OVERHEAD   = {tdc_fixed_overhead}    # {unit} ({tdc_fixed_overhead * ns_per:.1f} ns)")
        if tdc_loop_cycles is not None:
            print(f"  PI_LATCH_LOOP_TICKS       = {tdc_loop_cycles}    # {unit} ({tdc_loop_cycles * ns_per:.1f} ns)")
        else:
            print(f"  PI_LATCH_LOOP_TICKS       = None  # undetermined")
        print(f"  PI_LATCH_MAX_CORRECTION   = {tdc_max_correction}")
        print()
        print(f"  # Apply correction: corrected = captured_cntvct - PI_LATCH_FIXED_OVERHEAD")
        if tdc_loop_cycles is not None:
            print(f"  # Fine correction:  adjustment = (delta - OVERHEAD) % LOOP_TICKS")


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
    print("=" * 70)

    # --- Pass 1: Teensy ---
    analyze_instrument(rows, TEENSY_PROFILE)

    # --- Comparison divider ---
    print()
    print()
    print("╔" + "═" * 68 + "╗")
    print("║" + "  Pi LATCH ANALYSIS FOLLOWS".center(68) + "║")
    print("╚" + "═" * 68 + "╝")

    # --- Pass 2: Pi ---
    analyze_instrument(rows, PI_PROFILE)

    # --- Cross-instrument summary ---
    print()
    print()
    print("=" * 70)
    print("CROSS-INSTRUMENT SUMMARY")
    print("=" * 70)
    print()

    # Quick extraction for summary
    teensy_deltas = []
    pi_deltas = []
    for r in rows:
        t_result, t_src = _extract_teensy(r)
        if t_result is not None and t_result <= TEENSY_PROFILE.max_delta:
            teensy_deltas.append(t_result)
        p_result, p_src = _extract_pi(r)
        if p_result is not None and p_result <= PI_PROFILE.max_delta:
            pi_deltas.append(p_result)

    if teensy_deltas:
        t_stats = WelfordStats()
        for d in teensy_deltas:
            t_stats.update(float(d))
        t_mean_ns = t_stats.mean * TEENSY_PROFILE.ns_per_unit
        t_std_ns = t_stats.stddev * TEENSY_PROFILE.ns_per_unit
    else:
        t_mean_ns = t_std_ns = None

    if pi_deltas:
        p_stats = WelfordStats()
        for d in pi_deltas:
            p_stats.update(float(d))
        p_mean_ns = p_stats.mean * PI_PROFILE.ns_per_unit
        p_std_ns = p_stats.stddev * PI_PROFILE.ns_per_unit
    else:
        p_mean_ns = p_std_ns = None

    print(f"  {'':20s} {'Teensy':>14s}  {'Pi':>14s}")
    print(f"  {'─' * 20} {'─' * 14}  {'─' * 14}")

    if t_mean_ns is not None:
        print(f"  {'Samples':20s} {len(teensy_deltas):>14d}  ", end="")
    else:
        print(f"  {'Samples':20s} {'(no data)':>14s}  ", end="")
    if p_mean_ns is not None:
        print(f"{len(pi_deltas):>14d}")
    else:
        print(f"{'(no data)':>14s}")

    if t_mean_ns is not None:
        print(f"  {'Mean latency (ns)':20s} {t_mean_ns:>14.1f}  ", end="")
    else:
        print(f"  {'Mean latency (ns)':20s} {'—':>14s}  ", end="")
    if p_mean_ns is not None:
        print(f"{p_mean_ns:>14.1f}")
    else:
        print(f"{'—':>14s}")

    if t_std_ns is not None:
        print(f"  {'Stddev (ns)':20s} {t_std_ns:>14.2f}  ", end="")
    else:
        print(f"  {'Stddev (ns)':20s} {'—':>14s}  ", end="")
    if p_std_ns is not None:
        print(f"{p_std_ns:>14.2f}")
    else:
        print(f"{'—':>14s}")

    if t_mean_ns is not None:
        print(f"  {'Clock (MHz)':20s} {TEENSY_PROFILE.freq_hz/1e6:>14.0f}  ", end="")
    else:
        print(f"  {'Clock (MHz)':20s} {'—':>14s}  ", end="")
    if p_mean_ns is not None:
        print(f"{PI_PROFILE.freq_hz/1e6:>14.0f}")
    else:
        print(f"{'—':>14s}")

    if t_mean_ns is not None:
        print(f"  {'Resolution (ns)':20s} {TEENSY_PROFILE.ns_per_unit:>14.2f}  ", end="")
    else:
        print(f"  {'Resolution (ns)':20s} {'—':>14s}  ", end="")
    if p_mean_ns is not None:
        print(f"{PI_PROFILE.ns_per_unit:>14.2f}")
    else:
        print(f"{'—':>14s}")

    # --- PPS Period Stability Analysis ---
    # The interpolation base: how many cycles fit in one PPS second?
    # This is the number that determines interpolation accuracy.
    # Latch latency cancels out because it appears in both consecutive
    # shadow values — only genuine clock frequency variation survives.

    print()
    print()
    print("╔" + "═" * 68 + "╗")
    print("║" + "  PPS PERIOD STABILITY (INTERPOLATION BASE)".center(68) + "║")
    print("╚" + "═" * 68 + "╝")
    print()

    # --- Extract consecutive shadow pairs for each domain ---

    # Teensy DWT period: consecutive diag_raw_shadow_cyc differences
    teensy_shadows = []
    for r in rows:
        sc = r.get("diag_raw_shadow_cyc")
        if sc is not None:
            teensy_shadows.append(int(sc))

    # Pi CNTVCT period: diag_pi_same_edge_period_ticks (already computed
    # by CLOCKS as current_shadow - prior_shadow), OR we can derive from
    # consecutive diag_pi_latch_shadow_cntvct values.
    pi_period_ticks = []
    pi_period_from_field = 0
    pi_period_from_diff = 0

    for r in rows:
        # Priority 1: pre-computed period field
        pt = r.get("diag_pi_same_edge_period_ticks")
        if pt is not None:
            pt = int(pt)
            if pt > 0:
                pi_period_ticks.append(pt)
                pi_period_from_field += 1
                continue

    # Priority 2: diff consecutive shadow CNTVCT values
    if not pi_period_ticks:
        pi_shadows = []
        for r in rows:
            sv = r.get("diag_pi_latch_shadow_cntvct")
            if sv is not None:
                pi_shadows.append(int(sv))
        for i in range(1, len(pi_shadows)):
            diff = pi_shadows[i] - pi_shadows[i - 1]
            # Sanity: should be close to 54 MHz (± 0.1%)
            if 53_900_000 < diff < 54_100_000:
                pi_period_ticks.append(diff)
                pi_period_from_diff += 1

    # Teensy DWT period from consecutive shadows
    teensy_periods = []
    for i in range(1, len(teensy_shadows)):
        diff = (teensy_shadows[i] - teensy_shadows[i - 1]) & 0xFFFFFFFF
        # Sanity: should be close to 1,008,000,000 (± 0.1%)
        if 1_007_000_000 < diff < 1_009_000_000:
            teensy_periods.append(diff)

    # --- Analyze each domain ---

    for domain_name, periods, nominal_hz, ns_per, source_note in [
        ("Teensy DWT", teensy_periods, 1_008_000_000, 125.0 / 126.0,
         f"from {len(teensy_shadows)} consecutive shadow_cyc diffs"),
        ("Pi CNTVCT", pi_period_ticks, 54_000_000, 1e9 / 54_000_000,
         f"from {pi_period_from_field} period fields, {pi_period_from_diff} shadow diffs"
         if pi_period_from_diff > 0
         else f"from {pi_period_from_field} period fields"),
    ]:
        print("=" * 70)
        print(f"PPS PERIOD STABILITY: {domain_name}")
        print("=" * 70)
        print()

        if len(periods) < 2:
            print(f"  ❌ Insufficient data ({len(periods)} periods, need ≥ 2)")
            print(f"     {source_note}")
            print()
            continue

        stats = WelfordStats()
        for p in periods:
            stats.update(float(p))

        mean_period = stats.mean
        std_period = stats.stddev
        mean_ns = mean_period * ns_per
        std_ns = std_period * ns_per

        # Drift from nominal in PPB
        drift_ppb = ((mean_period - nominal_hz) / nominal_hz) * 1e9

        # Allan-deviation-like: RMS of consecutive period differences
        consec_diffs = []
        for i in range(1, len(periods)):
            consec_diffs.append(float(periods[i] - periods[i - 1]))
        if consec_diffs:
            adev_like = math.sqrt(sum(d * d for d in consec_diffs) / (2 * len(consec_diffs)))
            adev_ns = adev_like * ns_per
            adev_ppb = (adev_like / nominal_hz) * 1e9
        else:
            adev_like = adev_ns = adev_ppb = None

        print(f"  Samples:           {len(periods)}  ({source_note})")
        print(f"  Nominal frequency: {nominal_hz:,} Hz")
        print()
        print("-" * 70)
        print(f"PERIOD STATISTICS ({domain_name})")
        print("-" * 70)
        print(f"  mean   = {mean_period:,.2f} cycles  ({mean_ns:,.2f} ns)")
        print(f"  stddev = {std_period:,.4f} cycles  ({std_ns:,.2f} ns)")
        print(f"  min    = {int(stats.min_val):,} cycles")
        print(f"  max    = {int(stats.max_val):,} cycles")
        print(f"  range  = {int(stats.max_val) - int(stats.min_val):,} cycles"
              f"  ({(int(stats.max_val) - int(stats.min_val)) * ns_per:,.1f} ns)")
        print()
        print(f"  Mean drift from nominal: {drift_ppb:+.3f} PPB")
        print(f"  Period stddev:           {std_ns:.2f} ns  ({std_period / nominal_hz * 1e9:.3f} PPB)")
        print()

        if adev_like is not None:
            print(f"  Successive-diff RMS:     {adev_ns:.2f} ns  ({adev_ppb:.3f} PPB)")
            print(f"    (Allan deviation proxy at τ=1s)")
            print()

        # --- Interpolation quality assessment ---
        print("-" * 70)
        print(f"INTERPOLATION QUALITY ({domain_name})")
        print("-" * 70)
        print()

        # Max interpolation error at mid-second (worst case)
        # If period varies by ±std_period, the mid-second error is ±std_ns/2
        mid_err_ns = std_ns / 2.0
        print(f"  Mid-second interpolation error:  ±{mid_err_ns:.2f} ns (from period stddev)")

        # How this compares to the latch quantization
        if domain_name.startswith("Pi"):
            quant_ns = 1e9 / 54_000_000  # 18.52 ns
            print(f"  Latch quantization:              ±{quant_ns / 2:.2f} ns (CNTVCT @ 54 MHz)")
        else:
            quant_ns = 125.0 / 126.0  # 0.992 ns
            print(f"  TDC quantization:                ±{quant_ns / 2:.2f} ns (DWT @ 1008 MHz)")

        total_err = math.sqrt(mid_err_ns**2 + (quant_ns / 2)**2)
        print(f"  Combined (RSS):                  ±{total_err:.2f} ns")
        print()

        # Tau quality
        tau = mean_period / nominal_hz
        print(f"  τ (ticks-per-second ratio):      {tau:.12f}")
        print(f"  τ deviation from unity:          {(tau - 1.0) * 1e9:+.3f} PPB")
        print()

        # --- Period histogram (coarse buckets) ---
        # Show distribution of periods relative to mean
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
                ns_off = val * ns_per
                print(f"  {val:+5d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)  [{ns_off:+.1f} ns]")
            print()
        else:
            print(f"  (Period spread too wide for histogram: {len(offset_vals)} distinct values)")
            print()

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
                           ) as has_raw_teensy,
                           count(*) FILTER (
                               WHERE (payload->>'diag_teensy_dispatch_delta_ns') IS NOT NULL
                           ) as has_ns_teensy,
                           count(*) FILTER (
                               WHERE (payload->>'diag_pi_latch_delta_ticks') IS NOT NULL
                           ) as has_raw_pi,
                           count(*) FILTER (
                               WHERE (payload->>'diag_pi_latch_delta_ns') IS NOT NULL
                           ) as has_ns_pi
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s} {'T:RAW':>8s} {'T:NS':>8s} {'Pi:RAW':>8s} {'Pi:NS':>8s}")
                print(f"  {'─' * 20} {'─' * 8} {'─' * 8} {'─' * 8} {'─' * 8} {'─' * 8}")
                for r in rows:
                    print(
                        f"  {r['campaign']:<20s} {r['cnt']:>8d} "
                        f"{r['has_raw_teensy']:>8d} {r['has_ns_teensy']:>8d} "
                        f"{r['has_raw_pi']:>8d} {r['has_ns_pi']:>8d}"
                    )
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    analyze(campaign)


if __name__ == "__main__":
    main()