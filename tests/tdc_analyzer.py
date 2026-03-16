"""
ZPNet TDC Analyzer — v6 Spin Capture Edition

Analyzes the spin capture data from process_clocks_alpha v17 to derive
TDC correction constants at 1008 MHz.

The spin capture fires a nano-precise TimePop callback ~50 µs before
each PPS edge, landing on a predicted DWT cycle count with zero jitter.
The PPS ISR then captures its own DWT snapshot.  The delta between
the two reveals the ISR entry latency fingerprint.

Key fields from TIMEBASE rows:

  spin_valid          — true if both spin and ISR captured successfully
  spin_delta_cycles   — isr_dwt - landed_dwt (the TDC measurement)
  spin_error_cycles   — landed_dwt - target_dwt (nano-spin precision)
  spin_landed_dwt     — DWT where the nano-spin landed
  spin_isr_dwt        — DWT captured by the PPS ISR
  spin_timed_out      — true if the nano-spin timed out

The spin_delta_cycles histogram should cluster at discrete values
determined by the ISR entry latency and the instruction that was
executing when the PPS interrupt fired.

This tool:
  1. Extracts spin_delta_cycles from valid TIMEBASE records
  2. Builds a histogram and identifies clusters
  3. Separates the fixed early-arrival component (~50 µs) from
     the ISR latency jitter component
  4. Analyzes spin_error_cycles for nano-spin precision
  5. Derives recommended TDC constants for firmware

Usage:
    python -m zpnet.tests.tdc_analyzer <campaign_name> [limit]
    .zt tdc_analyzer Baseline8
    .zt tdc_analyzer Baseline8 200
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
EARLY_MARGIN_NS = 50_000  # 50 µs early arrival target


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
    print(f"ZPNet TDC ANALYZER v6 — Spin Capture Edition")
    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE records)")
    print("=" * 70)
    print()

    # ── Extract spin capture data ──

    deltas: List[int] = []
    spin_errors: List[int] = []
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

        if r.get("spin_timed_out"):
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

    print(f"  Valid spin captures:    {len(deltas)}")
    print(f"  Skipped (no fields):    {skipped_no_field}")
    print(f"  Skipped (invalid):      {skipped_invalid}")
    print(f"  Skipped (timeout):      {skipped_timeout}")
    print()

    if len(deltas) < 5:
        print(f"  Insufficient data (need >= 5, have {len(deltas)})")
        return

    # ── Delta statistics ──

    print("-" * 70)
    print("SPIN DELTA STATISTICS (isr_dwt - landed_dwt)")
    print("-" * 70)
    print()

    w_delta = Welford()
    for d in deltas:
        w_delta.update(float(d))

    mean_ns = w_delta.mean * DWT_NS_PER_CYCLE
    std_ns = w_delta.stddev * DWT_NS_PER_CYCLE

    print(f"  n       = {w_delta.n:,}")
    print(f"  mean    = {w_delta.mean:,.2f} cycles  ({mean_ns:,.1f} ns)")
    print(f"  stddev  = {w_delta.stddev:,.2f} cycles  ({std_ns:,.1f} ns)")
    print(f"  min     = {int(w_delta.min_val):,} cycles")
    print(f"  max     = {int(w_delta.max_val):,} cycles")
    print(f"  range   = {int(w_delta.range):,} cycles  ({w_delta.range * DWT_NS_PER_CYCLE:,.1f} ns)")
    print()

    # The delta is ~50,400 cycles (50 µs early margin) plus ISR entry latency.
    # The variation is what we care about — it contains the TDC fingerprint.
    expected_early_cycles = int(EARLY_MARGIN_NS * DWT_FREQ_HZ / 1_000_000_000)
    isr_overhead_mean = w_delta.mean - expected_early_cycles
    print(f"  Expected early margin:  {expected_early_cycles:,} cycles ({EARLY_MARGIN_NS / 1000:.0f} µs)")
    print(f"  Implied ISR overhead:   {isr_overhead_mean:,.1f} cycles ({isr_overhead_mean * DWT_NS_PER_CYCLE:,.1f} ns)")
    print(f"    (mean delta - early margin; includes prediction error)")
    print()

    # ── Nano-spin precision ──

    if spin_errors:
        print("-" * 70)
        print("NANO-SPIN PRECISION (landed_dwt - target_dwt)")
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

    # ── Delta histogram ──
    #
    # To reveal the ISR latency fingerprint, we subtract the minimum
    # delta (the fixed component) and look at the residual distribution.

    min_delta = int(w_delta.min_val)
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
    #
    # The spin_delta_cycles = early_margin + prediction_error + ISR_latency
    #
    # early_margin: ~50,400 cycles (constant, known)
    # prediction_error: the random walk residual (~3 cycles stddev)
    # ISR_latency: the TDC fingerprint (clusters at discrete values)
    #
    # Since the prediction error is small (~3 cycles) relative to the
    # ISR latency jitter (~10-50 cycles range), the cluster structure
    # in the histogram is dominated by the ISR latency fingerprint.

    print()
    print("-" * 70)
    print("DECOMPOSITION")
    print("-" * 70)
    print()
    print(f"  spin_delta = early_margin + prediction_error + ISR_latency")
    print()
    print(f"  Early margin (target):  {expected_early_cycles:,} cycles ({EARLY_MARGIN_NS / 1000:.0f} µs)")
    print(f"  Delta range:            {int(w_delta.range):,} cycles ({w_delta.range * DWT_NS_PER_CYCLE:.1f} ns)")
    print(f"  Delta stddev:           {w_delta.stddev:.2f} cycles ({std_ns:.1f} ns)")
    print()

    if spin_errors:
        print(f"  Spin landing jitter:    {w_err.stddev:.2f} cycles ({w_err.stddev * DWT_NS_PER_CYCLE:.1f} ns)")
        # The ISR latency jitter is the delta jitter minus the spin jitter (RSS)
        if w_delta.stddev > w_err.stddev:
            isr_jitter = math.sqrt(w_delta.stddev**2 - w_err.stddev**2)
            print(f"  ISR latency jitter:     {isr_jitter:.2f} cycles ({isr_jitter * DWT_NS_PER_CYCLE:.1f} ns)")
            print(f"    (delta jitter with spin jitter removed via RSS)")
        print()

    # ── Consecutive delta differences ──
    #
    # How much does the delta change from one PPS to the next?
    # This reveals the second-to-second variability.

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

    # ── Per-second detail table (first 20 and last 10) ──

    print("-" * 70)
    print("PER-SECOND DETAIL (first 20)")
    print("-" * 70)
    print()
    print(f"  {'pps':>6s}  {'delta':>8s}  {'offset':>7s}  {'spin_err':>9s}  {'delta_ns':>10s}  {'err_ns':>8s}")
    print(f"  {'─'*6}  {'─'*8}  {'─'*7}  {'─'*9}  {'─'*10}  {'─'*8}")

    shown = 0
    for i, r in enumerate(rows):
        if not r.get("spin_valid"):
            continue

        delta = r.get("spin_delta_cycles")
        err = r.get("spin_error_cycles", 0)
        pps = r.get("pps_count", r.get("teensy_pps_count", "?"))

        if delta is None:
            continue

        delta = int(delta)
        offset = delta - min_delta
        delta_ns = delta * DWT_NS_PER_CYCLE
        err_ns = int(err) * DWT_NS_PER_CYCLE

        print(f"  {pps:>6}  {delta:>8,}  {offset:>+7d}  {int(err):>+9d}  {delta_ns:>10.1f}  {err_ns:>8.1f}")

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
    print(f"  Nano-spin precision:        {w_err.stddev:.1f} cycles ({w_err.stddev * DWT_NS_PER_CYCLE:.1f} ns) stddev" if spin_errors else "")
    print(f"  Delta stability:            {w_delta.stddev:.1f} cycles ({std_ns:.1f} ns) stddev")
    print(f"  Delta range:                {int(w_delta.range)} cycles ({w_delta.range * DWT_NS_PER_CYCLE:.1f} ns)")
    print()

    if len(cluster_vals) >= 2:
        print(f"  Cluster spacing:            {most_common} cycles ({most_common * DWT_NS_PER_CYCLE:.1f} ns)")
        print(f"    → This is the spin loop iteration time at 1008 MHz")
        print()

    print(f"  Next steps:")
    print(f"    1. Collect 500+ seconds of data for robust histogram")
    print(f"    2. If clusters are visible, derive TDC_FIXED_OVERHEAD")
    print(f"       and TDC_LOOP_CYCLES from the cluster positions")
    print(f"    3. If clusters are NOT visible (prediction error dominates),")
    print(f"       the spin capture is already better than the old TDC —")
    print(f"       the landed_dwt IS the corrected value, no table needed")
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