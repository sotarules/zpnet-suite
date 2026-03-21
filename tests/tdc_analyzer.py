"""
ZPNet TDC Analyzer — v7 (TimePop v8.1 Priority Queue Edition)

Analyzes the PPS spin capture data to derive TDC correction constants
at 1008 MHz for the nested ISR shadow-write architecture.

Architecture (v8.1):

  TimePop's priority queue scheduler fires an ISR callback (priority 16)
  ~5-50 µs before each PPS edge.  The callback enters a tight shadow-write
  loop, continuously writing DWT_CYCCNT to dispatch_shadow_dwt.

  The PPS ISR (priority 0) preempts via NVIC nesting, captures the shadow
  DWT, and sets pps_fired.  The delta (isr_dwt - shadow_dwt) reveals the
  nested ISR entry latency fingerprint.

  The DWT spin lands on the predicted cycle count with zero jitter (the
  TimePop scheduler delivers sub-nanosecond GNSS-phase-locked precision).
  The shadow-write loop then runs until PPS arrives.

Key fields from TIMEBASE rows:

  spin_valid            — true if both spin and ISR captured successfully
  spin_delta_cycles     — isr_dwt - shadow_dwt (the TDC measurement)
  spin_error_cycles     — landed_dwt - target_dwt (spin landing precision)
  spin_approach_cycles  — isr_dwt - landed_dwt (total time from spin to PPS)
  spin_landed_dwt       — DWT where the spin landed
  spin_isr_dwt          — DWT captured by the PPS ISR
  spin_tdc_correction   — correction applied by tdc_correct()
  spin_nano_timed_out   — true if DWT spin exceeded budget (should be false)
  spin_shadow_timed_out — true if shadow loop timed out (should be false)

The spin_delta_cycles histogram should show values determined by the
nested ISR entry latency and the instruction that was executing when
the PPS interrupt preempted the shadow-write loop.

This tool:
  1. Extracts spin_delta_cycles from valid TIMEBASE records
  2. Builds a histogram and identifies the delta distribution
  3. Analyzes spin_error_cycles for DWT spin precision
  4. Analyzes approach_cycles for shadow loop margin
  5. Derives recommended TDC constants for tdc_correction.h
  6. Reports second-to-second stability

Usage:
    python -m zpnet.tests.tdc_analyzer <campaign_name> [limit]
    .zt tdc_analyzer Shakeout4
    .zt tdc_analyzer Shakeout4 200
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db

DWT_FREQ_HZ = 1_008_000_000
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
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0

    @property
    def range(self) -> float:
        if self.min_val is not None and self.max_val is not None:
            return self.max_val - self.min_val
        return 0.0


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

    result = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        result.append(p)

    return result


def find_clusters(
    values: List[int],
    min_count: int = 3,
) -> List[Tuple[int, int]]:
    counter = Counter(values)
    clusters = [
        (val, cnt)
        for val, cnt in counter.items()
        if cnt >= min_count
    ]
    clusters.sort(key=lambda x: x[0])
    return clusters


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print("=" * 70)
    print(f"ZPNet TDC ANALYZER v7 — TimePop v8.1 Priority Queue Edition")
    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE records)")
    print("=" * 70)
    print()

    # ── Extract spin capture data ──

    deltas: List[int] = []
    spin_errors: List[int] = []
    approach_values: List[int] = []
    tdc_corrections: List[int] = []
    skipped_no_field = 0
    skipped_invalid = 0
    skipped_timeout = 0

    for i, r in enumerate(rows):
        if limit and len(deltas) >= limit:
            break

        valid = r.get("spin_valid")
        if valid is None:
            skipped_no_field += 1
            continue

        if not valid:
            skipped_invalid += 1
            continue

        # Check both timeout fields
        if r.get("spin_nano_timed_out") or r.get("spin_shadow_timed_out"):
            skipped_timeout += 1
            continue

        delta = r.get("spin_delta_cycles")
        if delta is None:
            skipped_no_field += 1
            continue

        deltas.append(int(delta))

        err = r.get("spin_error_cycles")
        if err is not None:
            spin_errors.append(int(err))

        approach = r.get("spin_approach_cycles")
        if approach is not None:
            approach_values.append(int(approach))

        tdc = r.get("spin_tdc_correction")
        if tdc is not None:
            tdc_corrections.append(int(tdc))

    print(f"  Valid spin captures:    {len(deltas)}")
    print(f"  Skipped (no fields):    {skipped_no_field}")
    print(f"  Skipped (invalid):      {skipped_invalid}")
    print(f"  Skipped (timeout):      {skipped_timeout}")
    print()

    if len(deltas) < 5:
        print(f"  Insufficient data (need >= 5, have {len(deltas)})")
        return

    # ── Delta statistics (the TDC measurement) ──

    print("-" * 70)
    print("TDC DELTA STATISTICS (isr_dwt - shadow_dwt)")
    print("-" * 70)
    print()

    w_delta = Welford()
    for d in deltas:
        w_delta.update(float(d))

    mean_ns = w_delta.mean * DWT_NS_PER_CYCLE
    std_ns = w_delta.stddev * DWT_NS_PER_CYCLE
    min_delta = int(w_delta.min_val)
    max_delta = int(w_delta.max_val)

    print(f"  n       = {w_delta.n:,}")
    print(f"  mean    = {w_delta.mean:,.2f} cycles  ({mean_ns:,.1f} ns)")
    print(f"  stddev  = {w_delta.stddev:,.2f} cycles  ({std_ns:,.1f} ns)")
    print(f"  min     = {min_delta:,} cycles")
    print(f"  max     = {max_delta:,} cycles")
    print(f"  range   = {int(w_delta.range):,} cycles  ({w_delta.range * DWT_NS_PER_CYCLE:,.1f} ns)")
    print()

    # ── DWT spin precision ──

    if spin_errors:
        print("-" * 70)
        print("DWT SPIN PRECISION (landed_dwt - target_dwt)")
        print("-" * 70)
        print()

        w_err = Welford()
        for e in spin_errors:
            w_err.update(float(e))

        print(f"  n       = {w_err.n:,}")
        print(f"  mean    = {w_err.mean:,.2f} cycles  ({w_err.mean * DWT_NS_PER_CYCLE:,.1f} ns)")
        print(f"  stddev  = {w_err.stddev:,.2f} cycles  ({w_err.stddev * DWT_NS_PER_CYCLE:,.1f} ns)")
        print(f"  min     = {int(w_err.min_val):,} cycles")
        print(f"  max     = {int(w_err.max_val):,} cycles")
        print(f"  range   = {int(w_err.range):,} cycles")
        print()

        if w_err.range == 0 and w_err.mean == 0:
            print(f"  Assessment: PERFECT — zero landing error on every sample")
        elif w_err.stddev <= 1.0:
            print(f"  Assessment: EXCELLENT — sub-cycle jitter")
        else:
            print(f"  Assessment: NOMINAL — {w_err.stddev:.1f} cycle jitter")
        print()

    # ── Approach time (shadow loop margin) ──

    if approach_values:
        print("-" * 70)
        print("APPROACH TIME (isr_dwt - landed_dwt)")
        print("-" * 70)
        print()

        w_approach = Welford()
        for a in approach_values:
            w_approach.update(float(a))

        approach_mean_us = w_approach.mean * DWT_NS_PER_CYCLE / 1000.0
        approach_min_us = w_approach.min_val * DWT_NS_PER_CYCLE / 1000.0
        approach_max_us = w_approach.max_val * DWT_NS_PER_CYCLE / 1000.0

        print(f"  n       = {w_approach.n:,}")
        print(f"  mean    = {w_approach.mean:,.0f} cycles  ({approach_mean_us:,.1f} µs)")
        print(f"  min     = {int(w_approach.min_val):,} cycles  ({approach_min_us:,.1f} µs)")
        print(f"  max     = {int(w_approach.max_val):,} cycles  ({approach_max_us:,.1f} µs)")
        print()
        print(f"  This is how long the shadow-write loop ran before PPS arrived.")
        print(f"  It must be long enough for the loop to write at least one")
        print(f"  fresh DWT value before the PPS ISR captures it.")
        print()

        if approach_min_us < 1.0:
            print(f"  ⚠️  Minimum approach {approach_min_us:.1f} µs is dangerously tight")
            print(f"     Consider increasing SPIN_EARLY_NS for more headroom")
        elif approach_min_us < 3.0:
            print(f"  ℹ️  Minimum approach {approach_min_us:.1f} µs — adequate but tight")
        else:
            print(f"  ✅ Minimum approach {approach_min_us:.1f} µs — comfortable margin")
        print()

    # ── Delta histogram ──
    #
    # The delta is the TDC measurement: isr_dwt - shadow_dwt.
    # Subtract the minimum to show the correction distribution.

    residuals = [d - min_delta for d in deltas]

    print("-" * 70)
    print(f"DELTA HISTOGRAM (offset from minimum {min_delta:,})")
    print("-" * 70)
    print()

    counter = Counter(residuals)
    all_vals = sorted(counter.keys())
    max_count = max(counter.values()) if counter else 1
    bar_width = 50

    # Show compact histogram for values up to reasonable range
    display_vals = [v for v in all_vals if v <= 200]
    hidden = len(all_vals) - len(display_vals)

    last_val = None
    for val in display_vals:
        if last_val is not None and val - last_val > 3:
            print(f"       │ {'':.<{bar_width}s}  ···")
        cnt = counter[val]
        bar_len = int((cnt / max_count) * bar_width)
        bar = "█" * bar_len
        pct = (cnt / len(residuals)) * 100
        ns_off = val * DWT_NS_PER_CYCLE
        print(f"  {val:4d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)  [{ns_off:+.1f} ns]")
        last_val = val

    if hidden > 0:
        print(f"       │  ... {hidden} additional values beyond +200 cycles")
    print()

    # ── Cluster detection ──

    print("-" * 70)
    print("CLUSTER ANALYSIS")
    print("-" * 70)
    print()

    min_hits = max(2, len(residuals) // 50)
    clusters = find_clusters(residuals, min_count=min_hits)

    if not clusters:
        clusters = find_clusters(residuals, min_count=1)

    if not clusters:
        print("  No clusters found")
        return

    print(f"  Clusters (>= {min_hits} hits):")
    print()

    total_in_clusters = 0
    for val, cnt in clusters:
        pct = (cnt / len(residuals)) * 100
        total_in_clusters += cnt
        abs_cycles = val + min_delta
        ns_off = val * DWT_NS_PER_CYCLE
        print(f"    offset {val:4d} (abs {abs_cycles:,}):  {cnt:5d} hits ({pct:5.1f}%)  [{ns_off:.1f} ns from min]")

    unclustered = len(residuals) - total_in_clusters
    print()
    print(f"  In clusters:  {total_in_clusters} ({total_in_clusters / len(residuals) * 100:.1f}%)")
    print(f"  Unclustered:  {unclustered} ({unclustered / len(residuals) * 100:.1f}%)")

    # ── Spacing analysis ──

    cluster_vals = [v for v, _ in clusters]

    if len(cluster_vals) >= 2:
        spacings = [cluster_vals[i + 1] - cluster_vals[i] for i in range(len(cluster_vals) - 1)]
        spacing_counter = Counter(spacings)
        most_common = spacing_counter.most_common(1)[0][0]

        print()
        print(f"  Cluster spacings: {spacings}")
        print(f"  Most common spacing: {most_common} cycles ({most_common * DWT_NS_PER_CYCLE:.1f} ns)")
        print(f"    (this is the loop iteration cycle count at 1008 MHz)")

    # ── Decomposition ──

    print()
    print("-" * 70)
    print("DECOMPOSITION")
    print("-" * 70)
    print()
    print(f"  spin_delta = nested_ISR_entry_latency + loop_position_jitter")
    print()
    print(f"  Delta range:            {int(w_delta.range):,} cycles ({w_delta.range * DWT_NS_PER_CYCLE:.1f} ns)")
    print(f"  Delta stddev:           {w_delta.stddev:.2f} cycles ({std_ns:.1f} ns)")
    print()

    if spin_errors:
        print(f"  DWT spin jitter:        {w_err.stddev:.2f} cycles ({w_err.stddev * DWT_NS_PER_CYCLE:.1f} ns)")
        # The ISR latency jitter is the delta jitter minus the spin jitter (RSS)
        if w_delta.stddev > w_err.stddev:
            isr_jitter = math.sqrt(w_delta.stddev**2 - w_err.stddev**2)
            print(f"  ISR nesting jitter:     {isr_jitter:.2f} cycles ({isr_jitter * DWT_NS_PER_CYCLE:.1f} ns)")
            print(f"    (delta jitter with spin jitter removed via RSS)")
        print()

    # ── TDC constant recommendations ──

    print("-" * 70)
    print("TDC CONSTANT RECOMMENDATIONS (for tdc_correction.h)")
    print("-" * 70)
    print()

    delta_range = max_delta - min_delta

    print(f"  TDC_FIXED_OVERHEAD  = {min_delta}    // minimum observed delta")
    print(f"  TDC_LOOP_CYCLES     = 1     // 1 cycle per loop iteration at 1008 MHz")
    print(f"  TDC_MAX_CORRECTION  = {delta_range}    // max correction (delta range {min_delta}-{max_delta})")
    print()

    if delta_range <= 12:
        print(f"  Assessment: GOOD — {delta_range + 1} discrete correction values")
        print(f"  Correction precision: ±0.5 cycles (±{0.5 * DWT_NS_PER_CYCLE:.1f} ns)")
    else:
        print(f"  Assessment: WIDE — {delta_range + 1} values, consider investigating")
        print(f"  NVIC nesting jitter may be higher than expected")
    print()

    if tdc_corrections:
        w_tdc = Welford()
        for t in tdc_corrections:
            w_tdc.update(float(t))

        tdc_counter = Counter(tdc_corrections)
        print(f"  Applied corrections (from TIMEBASE spin_tdc_correction):")
        print(f"    n={w_tdc.n}  mean={w_tdc.mean:.1f}  stddev={w_tdc.stddev:.1f}")
        print(f"    min={int(w_tdc.min_val)}  max={int(w_tdc.max_val)}")
        print()
        print(f"    Distribution:")
        for val in sorted(tdc_counter.keys()):
            cnt = tdc_counter[val]
            pct = cnt / len(tdc_corrections) * 100
            print(f"      correction {val:>3d}: {cnt:>5d} ({pct:5.1f}%)")
        print()

    # ── Consecutive delta differences ──

    if len(deltas) >= 3:
        print("-" * 70)
        print("SECOND-TO-SECOND STABILITY")
        print("-" * 70)
        print()

        w_diff = Welford()
        for i in range(1, len(deltas)):
            w_diff.update(float(deltas[i] - deltas[i - 1]))

        print(f"  Consecutive delta differences:")
        print(f"    n       = {w_diff.n:,}")
        print(f"    mean    = {w_diff.mean:+.2f} cycles")
        print(f"    stddev  = {w_diff.stddev:.2f} cycles ({w_diff.stddev * DWT_NS_PER_CYCLE:.1f} ns)")
        print(f"    min     = {int(w_diff.min_val):+,} cycles")
        print(f"    max     = {int(w_diff.max_val):+,} cycles")
        print()

    # ── Per-second detail table ──

    print("-" * 70)
    print("PER-SECOND DETAIL (first 20)")
    print("-" * 70)
    print()
    print(f"  {'pps':>6s}  {'delta':>8s}  {'offset':>7s}  {'spin_err':>9s}  {'approach':>9s}  {'tdc':>4s}  {'delta_ns':>10s}")
    print(f"  {'─'*6}  {'─'*8}  {'─'*7}  {'─'*9}  {'─'*9}  {'─'*4}  {'─'*10}")

    shown = 0
    for i, r in enumerate(rows):
        if not r.get("spin_valid"):
            continue

        delta = r.get("spin_delta_cycles")
        err = r.get("spin_error_cycles", 0)
        approach = r.get("spin_approach_cycles", 0)
        tdc = r.get("spin_tdc_correction", "?")
        pps = r.get("pps_count", r.get("teensy_pps_count", "?"))

        if delta is None:
            continue

        delta = int(delta)
        offset = delta - min_delta
        delta_ns = delta * DWT_NS_PER_CYCLE

        tdc_str = str(int(tdc)) if isinstance(tdc, (int, float)) else str(tdc)

        print(f"  {pps:>6}  {delta:>8,}  {offset:>+7d}  {int(err):>+9d}  {int(approach):>9,}  {tdc_str:>4s}  {delta_ns:>10.1f}")

        shown += 1
        if shown >= 20:
            remaining = len(deltas) - shown
            if remaining > 0:
                print(f"  ... {remaining} more rows ...")
            break

    print()

    # ── Summary ──

    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print()
    print(f"  Spin capture success rate:  {len(deltas)}/{len(deltas) + skipped_invalid} "
          f"({100 * len(deltas) / max(1, len(deltas) + skipped_invalid):.1f}%)")

    if spin_errors:
        if w_err.range == 0 and w_err.mean == 0:
            print(f"  DWT spin precision:         PERFECT (zero error, all samples)")
        else:
            print(f"  DWT spin precision:         {w_err.stddev:.1f} cycles ({w_err.stddev * DWT_NS_PER_CYCLE:.1f} ns) stddev")

    print(f"  TDC delta stability:        {w_delta.stddev:.1f} cycles ({std_ns:.1f} ns) stddev")
    print(f"  TDC delta range:            {int(w_delta.range)} cycles ({w_delta.range * DWT_NS_PER_CYCLE:.1f} ns)")

    if approach_values:
        print(f"  Shadow loop margin:         {w_approach.min_val:,.0f}-{w_approach.max_val:,.0f} cycles "
              f"({w_approach.min_val * DWT_NS_PER_CYCLE / 1000:.1f}-{w_approach.max_val * DWT_NS_PER_CYCLE / 1000:.1f} µs)")

    print()
    print(f"  Recommended constants:")
    print(f"    TDC_FIXED_OVERHEAD  = {min_delta}")
    print(f"    TDC_LOOP_CYCLES     = 1")
    print(f"    TDC_MAX_CORRECTION  = {delta_range}")

    if len(cluster_vals) >= 2:
        print()
        print(f"  Cluster spacing:            {most_common} cycles ({most_common * DWT_NS_PER_CYCLE:.1f} ns)")
        print(f"    → Shadow-write loop iteration time at 1008 MHz")

    print()
    print("=" * 70)


def main():
    if len(sys.argv) < 2:
        print("Usage: tdc_analyzer <campaign_name> [limit]")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, count(*) as cnt,
                           count(*) FILTER (
                               WHERE (payload->>'spin_valid')::boolean = true
                           ) as spin_valid
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s} {'SPIN OK':>8s}")
                print(f"  {'─' * 20} {'─' * 8} {'─' * 8}")
                for r in rows:
                    print(f"  {r['campaign']:<20s} {r['cnt']:>8d} {r['spin_valid']:>8d}")
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    limit_val = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit_val)


if __name__ == "__main__":
    main()