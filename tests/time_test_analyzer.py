"""
ZPNet TIME_TEST Analyzer — Continuous time_dwt_to_gnss_ns() Validation

Analyzes TIME_TEST data from TIMEBASE records to:

  1. Characterize CH3 ISR entry latency (isr_delta_cycles histogram)
     — The minimum delta is the true fixed hardware overhead.
     — Derives recommended constant for isr_dwt_compensate.h.

  2. Validate time_dwt_to_gnss_ns() accuracy (residual distribution)
     — residual_ns = computed_gnss_ns - vclock_gnss_ns
     — The VCLOCK-derived value is ground truth (hardware counter).

  3. Detect position-in-second dependence (residual vs phase)
     — DWT rate prediction error accumulates linearly across the second.
     — Early-in-second samples should have smaller residuals.
     — The slope reveals instantaneous DWT frequency error in ppb.

Architecture:

  Every second, alpha arms a TimePop one-shot at a random delay
  (100–900 ms).  The callback arms QTimer1 CH3 to compare on the
  next VCLOCK edge.  CH3's ISR captures DWT_CYCCNT, the shadow DWT
  from the spin loop, and the QTimer1 32-bit value.

  The callback computes edge_dwt = isr_dwt - ISR_OVERHEAD, feeds it
  through time_dwt_to_gnss_ns(), and compares against the QTimer-
  derived ground truth.

Key TIMEBASE fields (inside fragment sub-dict):

  time_test_valid              — true if entire chain succeeded
  time_test_residual_ns        — computed_gnss_ns - vclock_gnss_ns
  time_test_computed_gnss_ns   — time_dwt_to_gnss_ns(edge_dwt)
  time_test_vclock_gnss_ns     — ground truth from QTimer
  time_test_isr_dwt            — DWT_CYCCNT first instruction in CH3 ISR
  time_test_isr_shadow_dwt     — shadow DWT captured by CH3 ISR
  time_test_isr_delta_cycles   — isr_dwt - isr_shadow_dwt (ISR latency)
  time_test_edge_dwt           — corrected DWT at VCLOCK edge
  time_test_vclock_at_fire     — QTimer1 32-bit value at ISR entry
  time_test_tests_run          — lifetime arm attempts
  time_test_tests_valid        — lifetime successful validations

Usage:
    python -m zpnet.tests.time_test_analyzer <campaign_name> [limit]
    .zt time_test_analyzer Alpha10
    .zt time_test_analyzer Alpha10 500
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
NS_PER_SECOND = 1_000_000_000


# ─────────────────────────────────────────────────────────────────────
# Welford online accumulator
# ─────────────────────────────────────────────────────────────────────

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


# ─────────────────────────────────────────────────────────────────────
# Database fetch
# ─────────────────────────────────────────────────────────────────────

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


def _frag(row: Dict[str, Any], key: str, default=None):
    """Read a field from the fragment sub-dict, falling back to top-level."""
    frag = row.get("fragment")
    if frag and key in frag:
        return frag[key]
    return row.get(key, default)


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


# ─────────────────────────────────────────────────────────────────────
# Analysis
# ─────────────────────────────────────────────────────────────────────

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print("=" * 72)
    print("ZPNet TIME_TEST ANALYZER — time_dwt_to_gnss_ns() Validation")
    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE records)")
    print("=" * 72)
    print()

    # ── Extract TIME_TEST data ──

    deltas: List[int] = []            # isr_delta_cycles
    residuals: List[int] = []         # residual_ns
    positions: List[int] = []         # ns into second (vclock_gnss_ns % 1e9)
    residual_position_pairs: List[Tuple[int, int]] = []  # (position_ns, residual_ns)

    skipped_no_field = 0
    skipped_invalid = 0
    count = 0

    for r in rows:
        if limit and count >= limit:
            break

        valid = _frag(r, "time_test_valid")
        if valid is None:
            skipped_no_field += 1
            continue

        if not valid:
            skipped_invalid += 1
            continue

        delta = _frag(r, "time_test_isr_delta_cycles")
        residual = _frag(r, "time_test_residual_ns")
        vclock_gnss = _frag(r, "time_test_vclock_gnss_ns")

        if delta is None or residual is None:
            skipped_no_field += 1
            continue

        delta = int(delta)
        residual = int(residual)

        deltas.append(delta)
        residuals.append(residual)

        if vclock_gnss is not None:
            pos_ns = int(vclock_gnss) % NS_PER_SECOND
            positions.append(pos_ns)
            residual_position_pairs.append((pos_ns, residual))

        count += 1

    # ── Lifetime stats from most recent record ──

    last = rows[-1] if rows else {}
    tests_run = _frag(last, "time_test_tests_run", 0)
    tests_valid = _frag(last, "time_test_tests_valid", 0)
    tests_time_invalid = _frag(last, "time_test_tests_time_invalid", 0)

    print(f"  Total TIMEBASE records:     {len(rows):,}")
    print(f"  Valid TIME_TEST samples:    {len(deltas):,}")
    print(f"  Skipped (no field):         {skipped_no_field:,}")
    print(f"  Skipped (invalid):          {skipped_invalid:,}")
    print()
    print(f"  Lifetime (from firmware):")
    print(f"    tests_run:                {tests_run:,}")
    print(f"    tests_valid:              {tests_valid:,}")
    print(f"    tests_time_invalid:       {tests_time_invalid:,}")
    print()

    if not deltas:
        print("  No valid TIME_TEST samples found — cannot analyze.")
        return

    # ══════════════════════════════════════════════════════════════════
    # Section 1: ISR DELTA DISTRIBUTION (CH3 entry latency)
    # ══════════════════════════════════════════════════════════════════

    print("─" * 72)
    print("1. CH3 ISR ENTRY LATENCY (isr_delta_cycles = isr_dwt - shadow_dwt)")
    print("─" * 72)
    print()
    print("  This is the number of DWT cycles between the last shadow-write")
    print("  loop iteration and the first DWT_CYCCNT read inside the CH3 ISR.")
    print("  The minimum is the true fixed hardware overhead.")
    print()

    w_delta = Welford()
    for d in deltas:
        w_delta.update(float(d))

    min_delta = int(w_delta.min_val)
    max_delta = int(w_delta.max_val)
    delta_range = max_delta - min_delta

    print(f"    n={w_delta.n:,}  mean={w_delta.mean:.2f}  stddev={w_delta.stddev:.2f}")
    print(f"    min={min_delta}  max={max_delta}  range={delta_range}")
    print(f"    min in ns: {min_delta * DWT_NS_PER_CYCLE:.1f} ns")
    print()

    # Delta histogram (offset from minimum)
    offsets = [d - min_delta for d in deltas]
    counter = Counter(offsets)
    all_vals = sorted(counter.keys())
    max_count = max(counter.values()) if counter else 1
    bar_width = 50

    display_vals = [v for v in all_vals if v <= 200]
    hidden = len(all_vals) - len(display_vals)

    last_val = None
    for val in display_vals:
        if last_val is not None and val - last_val > 3:
            print(f"       │ {'':.<{bar_width}s}  ···")
        cnt = counter[val]
        bar_len = int((cnt / max_count) * bar_width)
        bar = "█" * bar_len
        pct = (cnt / len(offsets)) * 100
        abs_cycles = val + min_delta
        ns_off = val * DWT_NS_PER_CYCLE
        print(f"  {val:4d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)  [abs={abs_cycles}, +{ns_off:.1f} ns]")
        last_val = val

    if hidden > 0:
        print(f"       │  ... {hidden} additional values beyond +200 cycles")
    print()

    # Cluster analysis
    min_hits = max(2, len(offsets) // 50)
    clusters = find_clusters(offsets, min_count=min_hits)

    if clusters:
        print("  Clusters:")
        total_in_clusters = 0
        cluster_vals = []
        for val, cnt in clusters:
            pct = (cnt / len(offsets)) * 100
            total_in_clusters += cnt
            abs_cycles = val + min_delta
            print(f"    offset {val:>3d} (abs {abs_cycles:>3d}): {cnt:>5d} ({pct:5.1f}%)")
            cluster_vals.append(val)

        pct_clustered = (total_in_clusters / len(offsets)) * 100
        print(f"    Total in clusters: {total_in_clusters:,} ({pct_clustered:.1f}%)")

        if len(cluster_vals) >= 2:
            spacings = [cluster_vals[i+1] - cluster_vals[i] for i in range(len(cluster_vals)-1)]
            most_common_spacing = Counter(spacings).most_common(1)[0][0] if spacings else 0
            print(f"    Most common spacing: {most_common_spacing} cycles ({most_common_spacing * DWT_NS_PER_CYCLE:.1f} ns)")
            print(f"      → Shadow-write loop iteration time at 1008 MHz")
        print()

    # ── Recommended constant ──
    print("  RECOMMENDED CONSTANT (for isr_dwt_compensate.h):")
    print()
    print(f"    QTIMER1_CH3_ISR_ENTRY_DWT_CYCLES = {min_delta}")
    print(f"    // minimum observed delta across {w_delta.n:,} samples")
    print(f"    // {min_delta * DWT_NS_PER_CYCLE:.1f} ns at 1008 MHz")
    print()

    if delta_range <= 20:
        print(f"  Assessment: GOOD — {delta_range + 1} discrete latency values, tight distribution")
    elif delta_range <= 50:
        print(f"  Assessment: ACCEPTABLE — range {delta_range} cycles, moderate NVIC jitter")
    else:
        print(f"  Assessment: WIDE — range {delta_range} cycles, investigate contention sources")
    print()

    # ══════════════════════════════════════════════════════════════════
    # Section 2: RESIDUAL DISTRIBUTION (time_dwt_to_gnss_ns accuracy)
    # ══════════════════════════════════════════════════════════════════

    print("─" * 72)
    print("2. VALIDATION RESIDUAL (computed_gnss_ns - vclock_gnss_ns)")
    print("─" * 72)
    print()
    print("  The residual measures how accurately time_dwt_to_gnss_ns()")
    print("  reproduces the QTimer-derived ground truth.  A stable bias")
    print("  indicates the ISR overhead constant needs calibration.")
    print("  Jitter indicates DWT interpolation noise.")
    print()

    w_resid = Welford()
    for r in residuals:
        w_resid.update(float(r))

    print(f"    n={w_resid.n:,}  mean={w_resid.mean:.2f} ns  stddev={w_resid.stddev:.2f} ns")
    print(f"    min={int(w_resid.min_val)} ns  max={int(w_resid.max_val)} ns  range={int(w_resid.range)} ns")
    print()

    # Residual histogram
    resid_counter = Counter(residuals)
    resid_vals = sorted(resid_counter.keys())
    max_resid_count = max(resid_counter.values()) if resid_counter else 1

    print("  Distribution (ns):")
    for val in resid_vals:
        cnt = resid_counter[val]
        bar_len = int((cnt / max_resid_count) * bar_width)
        bar = "█" * bar_len
        pct = (cnt / len(residuals)) * 100
        print(f"  {val:>5d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)")
    print()

    # ── Corrected residual (what it would be with empirical overhead) ──
    #
    # If we replaced the ISR overhead constant with min_delta, the
    # edge_dwt would shift earlier by (current_constant - min_delta) cycles.
    # Each cycle ≈ 1 ns, so the residual shifts by that amount.

    current_constant = 29  # QTIMER1_CH3_ISR_ENTRY_DWT_CYCLES
    correction_shift = (min_delta - current_constant)  # cycles
    correction_shift_ns = correction_shift * DWT_NS_PER_CYCLE

    print(f"  If QTIMER1_CH3_ISR_ENTRY_DWT_CYCLES were updated from {current_constant} to {min_delta}:")
    print(f"    Residual shift: {-correction_shift_ns:+.1f} ns")
    print(f"    Corrected mean: {w_resid.mean - correction_shift_ns:.2f} ns")
    print(f"    Corrected range: {int(w_resid.min_val) - int(correction_shift_ns)}"
          f" to {int(w_resid.max_val) - int(correction_shift_ns)} ns")
    print()

    # ══════════════════════════════════════════════════════════════════
    # Section 3: POSITION-IN-SECOND ANALYSIS (DWT rate error ramp)
    # ══════════════════════════════════════════════════════════════════

    if residual_position_pairs:
        print("─" * 72)
        print("3. POSITION-IN-SECOND ANALYSIS (residual vs phase within PPS second)")
        print("─" * 72)
        print()
        print("  time_dwt_to_gnss_ns() interpolates using the previous second's")
        print("  DWT rate.  The rate prediction error accumulates linearly.")
        print("  Early-in-second samples have less accumulated error.")
        print()

        # Sort by position
        pairs_sorted = sorted(residual_position_pairs, key=lambda p: p[0])

        # Bin into 10 equal bins (100 ms each)
        n_bins = 10
        bin_size = NS_PER_SECOND // n_bins
        bins: List[List[int]] = [[] for _ in range(n_bins)]

        for pos_ns, resid in pairs_sorted:
            bin_idx = min(pos_ns // bin_size, n_bins - 1)
            bins[bin_idx].append(resid)

        print(f"  {'Bin':>5s}  {'Range (ms)':>14s}  {'n':>6s}  {'mean':>8s}  {'stddev':>7s}  {'min':>5s}  {'max':>5s}")
        print(f"  {'─'*5}  {'─'*14}  {'─'*6}  {'─'*8}  {'─'*7}  {'─'*5}  {'─'*5}")

        bin_means: List[Tuple[float, float]] = []  # (bin_center_s, mean_resid)

        for i, bin_data in enumerate(bins):
            lo_ms = i * 100
            hi_ms = (i + 1) * 100
            range_str = f"{lo_ms:>4d}–{hi_ms:<4d} ms"

            if not bin_data:
                print(f"  {i:>5d}  {range_str:>14s}  {0:>6d}  {'---':>8s}  {'---':>7s}  {'---':>5s}  {'---':>5s}")
                continue

            w = Welford()
            for v in bin_data:
                w.update(float(v))

            bin_center_s = (lo_ms + hi_ms) / 2000.0
            bin_means.append((bin_center_s, w.mean))

            mean_str = f"{w.mean:>8.1f}"
            sd_str = f"{w.stddev:>7.1f}" if w.n >= 2 else f"{'---':>7s}"
            min_str = f"{int(w.min_val):>5d}"
            max_str = f"{int(w.max_val):>5d}"

            print(f"  {i:>5d}  {range_str:>14s}  {w.n:>6d}  {mean_str}  {sd_str}  {min_str}  {max_str}")

        print()

        # ── Linear regression: residual = a + b * position_in_second ──
        if len(bin_means) >= 3:
            xs = [p[0] for p in bin_means]
            ys = [p[1] for p in bin_means]

            n = len(xs)
            sum_x = sum(xs)
            sum_y = sum(ys)
            sum_xy = sum(x * y for x, y in zip(xs, ys))
            sum_x2 = sum(x * x for x in xs)

            denom = n * sum_x2 - sum_x * sum_x
            if abs(denom) > 1e-12:
                slope = (n * sum_xy - sum_x * sum_y) / denom
                intercept = (sum_y - slope * sum_x) / n

                # R² from bin means
                y_mean = sum_y / n
                ss_tot = sum((y - y_mean) ** 2 for y in ys)
                ss_res = sum((y - (intercept + slope * x)) ** 2 for x, y in zip(xs, ys))
                r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

                print("  Linear fit:  residual_ns = intercept + slope × position_s")
                print()
                print(f"    intercept = {intercept:.2f} ns  (residual at PPS edge = ISR overhead bias)")
                print(f"    slope     = {slope:.2f} ns/s  (= {slope:.2f} ppb DWT rate prediction error)")
                print(f"    R²        = {r_squared:.4f}")
                print()

                # What the intercept tells us about the ISR overhead
                # intercept ≈ (true_overhead - current_constant) * DWT_NS_PER_CYCLE
                implied_overhead = current_constant + intercept / DWT_NS_PER_CYCLE
                print(f"    Implied true ISR overhead: {implied_overhead:.1f} cycles")
                print(f"    Current constant:          {current_constant} cycles")
                print(f"    Suggested update:          {int(round(implied_overhead))} cycles")
                print()

                if abs(slope) < 1.0:
                    print(f"  Assessment: EXCELLENT — sub-ppb DWT rate consistency")
                elif abs(slope) < 5.0:
                    print(f"  Assessment: GOOD — {abs(slope):.1f} ppb DWT rate error")
                else:
                    print(f"  Assessment: NOTABLE — {abs(slope):.1f} ppb DWT rate error per second")
                    print(f"    This is expected and represents the DWT crystal's")
                    print(f"    instantaneous frequency offset from the GNSS reference.")
            print()

    # ══════════════════════════════════════════════════════════════════
    # Section 4: PER-SAMPLE DETAIL (first 20 + last 10)
    # ══════════════════════════════════════════════════════════════════

    print("─" * 72)
    print("4. PER-SAMPLE DETAIL")
    print("─" * 72)
    print()

    detail_header = (
        f"  {'pps':>6s}  {'delta':>6s}  {'resid':>6s}  {'pos_ms':>7s}"
        f"  {'computed_gnss_ns':>18s}  {'vclock_gnss_ns':>18s}"
    )
    detail_sep = (
        f"  {'─'*6}  {'─'*6}  {'─'*6}  {'─'*7}"
        f"  {'─'*18}  {'─'*18}"
    )

    print(detail_header)
    print(detail_sep)

    valid_rows = []
    for r in rows:
        if not _frag(r, "time_test_valid"):
            continue
        delta = _frag(r, "time_test_isr_delta_cycles")
        resid = _frag(r, "time_test_residual_ns")
        if delta is None or resid is None:
            continue
        valid_rows.append(r)

    show_head = min(20, len(valid_rows))
    show_tail = min(10, max(0, len(valid_rows) - show_head))

    def print_detail_row(r):
        pps = r.get("pps_count", r.get("teensy_pps_count", "?"))
        delta = int(_frag(r, "time_test_isr_delta_cycles"))
        resid = int(_frag(r, "time_test_residual_ns"))
        computed = _frag(r, "time_test_computed_gnss_ns", 0)
        vclock = _frag(r, "time_test_vclock_gnss_ns", 0)
        pos_ns = int(vclock) % NS_PER_SECOND if vclock else 0
        pos_ms = pos_ns / 1_000_000.0

        print(f"  {pps:>6}  {delta:>6d}  {resid:>+6d}  {pos_ms:>7.1f}"
              f"  {int(computed):>18,}  {int(vclock):>18,}")

    for r in valid_rows[:show_head]:
        print_detail_row(r)

    if len(valid_rows) > show_head + show_tail:
        omitted = len(valid_rows) - show_head - show_tail
        print(f"  ... {omitted:,} rows omitted ...")

    if show_tail > 0 and len(valid_rows) > show_head:
        for r in valid_rows[-show_tail:]:
            print_detail_row(r)

    print()

    # ══════════════════════════════════════════════════════════════════
    # Section 5: SUMMARY
    # ══════════════════════════════════════════════════════════════════

    print("=" * 72)
    print("SUMMARY")
    print("=" * 72)
    print()
    print(f"  Valid samples:     {len(deltas):,}/{len(deltas) + skipped_invalid:,}"
          f" ({100 * len(deltas) / max(1, len(deltas) + skipped_invalid):.1f}%)")
    print()
    print(f"  CH3 ISR latency:")
    print(f"    Fixed overhead:  {min_delta} cycles ({min_delta * DWT_NS_PER_CYCLE:.1f} ns)")
    print(f"    Jitter range:    {delta_range} cycles ({delta_range * DWT_NS_PER_CYCLE:.1f} ns)")
    print(f"    Mean:            {w_delta.mean:.1f} cycles ({w_delta.mean * DWT_NS_PER_CYCLE:.1f} ns)")
    print()
    print(f"  Validation residual (current constants):")
    print(f"    Mean:            {w_resid.mean:+.1f} ns")
    print(f"    Stddev:          {w_resid.stddev:.1f} ns")
    print(f"    Range:           {int(w_resid.min_val)} to {int(w_resid.max_val)} ns")
    print()
    print(f"  Recommended update:")
    print(f"    QTIMER1_CH3_ISR_ENTRY_DWT_CYCLES = {min_delta}  (currently 29)")
    print()
    print("=" * 72)


# ─────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: time_test_analyzer <campaign_name> [limit]")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, count(*) as cnt,
                           count(*) FILTER (
                               WHERE (payload->'fragment'->>'time_test_valid')::boolean = true
                           ) as tt_valid
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s} {'TT_VALID':>8s}")
                print(f"  {'─' * 20} {'─' * 8} {'─' * 8}")
                for r in rows:
                    print(f"  {r['campaign']:<20s} {r['cnt']:>8d} {r['tt_valid']:>8d}")
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    limit_val = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit_val)


if __name__ == "__main__":
    main()