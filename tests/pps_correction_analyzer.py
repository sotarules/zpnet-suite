"""
ZPNet PPS Correction Analyzer — pps_correct_10mhz Diagnostic Deep-Dive

Analyzes TIME_TEST and PPS correction diagnostics from TIMEBASE records to:

  1. Characterize pps_correct_10mhz behavior per peripheral
     — DWT bracket (cycles from snap_dwt to helper's DWT read)
     — Elapsed with NVIC overhead (bracket + PPS_ISR_FIXED_OVERHEAD)
     — Tick correction applied (elapsed / 101)

  2. QTimer anchor analysis
     — qtimer_at_pps values, correction jitter
     — Correlation between anchor tick parity and residual

  3. Residual distribution with correction context
     — Bimodal detection (50 ns vs 150 ns clusters)
     — Per-cluster statistics

  4. VCLOCK ground truth comparison
     — vclock_at_fire vs vclock_at_edge (COMP1) delta
     — Confirms CH3 ISR read latency in ticks

  5. Position-in-second dependence
     — Residual vs phase within PPS second

Usage:
    python -m zpnet.tests.pps_correction_analyzer <campaign_name> [limit]
    .zt pps_correction_analyzer Beta2
    .zt pps_correction_analyzer pps1 500
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db

DWT_FREQ_HZ = 1_008_000_000
DWT_NS_PER_CYCLE = 125.0 / 126.0  # ~0.992 ns
NS_PER_SECOND = 1_000_000_000
PPS_ISR_FIXED_OVERHEAD = 48
TICK_DIVISOR = 101  # DWT cycles per 100 ns VCLOCK tick (conservative)


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


# ─────────────────────────────────────────────────────────────────────
# Linear regression helper
# ─────────────────────────────────────────────────────────────────────

def linear_regression(pairs: List[Tuple[float, float]]) -> Optional[Tuple[float, float, float]]:
    """Returns (slope, intercept, r_squared) or None if insufficient data."""
    n = len(pairs)
    if n < 3:
        return None

    sx = sum(x for x, _ in pairs)
    sy = sum(y for _, y in pairs)
    sxx = sum(x * x for x, _ in pairs)
    sxy = sum(x * y for x, y in pairs)
    syy = sum(y * y for _, y in pairs)

    denom = n * sxx - sx * sx
    if abs(denom) < 1e-12:
        return None

    slope = (n * sxy - sx * sy) / denom
    intercept = (sy - slope * sx) / n

    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in pairs)
    ss_tot = syy - sy * sy / n
    r_squared = 1.0 - (ss_res / ss_tot) if abs(ss_tot) > 1e-12 else 0.0

    return slope, intercept, r_squared


# ─────────────────────────────────────────────────────────────────────
# Analysis
# ─────────────────────────────────────────────────────────────────────

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print("=" * 78)
    print("ZPNet PPS CORRECTION ANALYZER — pps_correct_10mhz Diagnostic")
    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE records)")
    print("=" * 78)
    print()

    # ── Extract data ──

    # PPS correction brackets
    brackets_ocxo1: List[int] = []
    brackets_ocxo2: List[int] = []
    brackets_gnss: List[int] = []

    # TIME_TEST data
    residuals: List[int] = []
    positions: List[int] = []
    residual_position_pairs: List[Tuple[float, float]] = []
    vclock_deltas: List[int] = []  # vclock_at_fire - vclock_at_edge
    anchor_qtimers: List[int] = []
    anchor_last_digits: List[int] = []

    # Correction forensics (when isr_snap_dwt available)
    gnss_elapsed_values: List[int] = []
    gnss_ticks_subtracted: List[int] = []

    # Bimodal analysis
    residual_with_anchor: List[Tuple[int, int]] = []  # (residual, qtimer_at_pps)

    skipped_no_field = 0
    skipped_invalid = 0
    count = 0

    for r in rows:
        if limit and count >= limit:
            break

        # ── PPS correction brackets ──
        b_ocxo1 = _frag(r, "diag_pps_correct_dwt_ocxo1")
        b_ocxo2 = _frag(r, "diag_pps_correct_dwt_ocxo2")
        b_gnss = _frag(r, "diag_pps_correct_dwt_gnss")
        snap_dwt = _frag(r, "isr_snap_dwt") or _frag(r, "dwt_cyccnt_at_pps")

        if b_ocxo1 is not None and b_ocxo2 is not None and b_gnss is not None and snap_dwt is not None:
            snap_dwt = int(snap_dwt)
            b_ocxo1 = int(b_ocxo1)
            b_ocxo2 = int(b_ocxo2)
            b_gnss = int(b_gnss)

            # Compute brackets from snap_dwt
            bracket_ocxo1 = (b_ocxo1 - snap_dwt) & 0xFFFFFFFF
            bracket_ocxo2 = (b_ocxo2 - snap_dwt) & 0xFFFFFFFF
            bracket_gnss = (b_gnss - snap_dwt) & 0xFFFFFFFF

            brackets_ocxo1.append(bracket_ocxo1)
            brackets_ocxo2.append(bracket_ocxo2)
            brackets_gnss.append(bracket_gnss)

            # Compute correction applied to GNSS
            elapsed_gnss = bracket_gnss + PPS_ISR_FIXED_OVERHEAD
            ticks_sub = elapsed_gnss // TICK_DIVISOR
            gnss_elapsed_values.append(elapsed_gnss)
            gnss_ticks_subtracted.append(ticks_sub)

        # ── TIME_TEST data ──
        valid = _frag(r, "time_test_valid")
        if valid is None:
            skipped_no_field += 1
            continue

        if not valid:
            skipped_invalid += 1
            continue

        residual = _frag(r, "time_test_residual_ns")
        if residual is None:
            skipped_no_field += 1
            continue

        residual = int(residual)
        residuals.append(residual)

        # Anchor qtimer
        anchor_qt = _frag(r, "time_test_diag_anchor_qtimer")
        if anchor_qt is not None:
            anchor_qt = int(anchor_qt)
            anchor_qtimers.append(anchor_qt)
            anchor_last_digits.append(anchor_qt % 10)
            residual_with_anchor.append((residual, anchor_qt))

        # VCLOCK delta
        vf = _frag(r, "time_test_vclock_at_fire")
        ve = _frag(r, "time_test_vclock_at_edge")
        if vf is not None and ve is not None:
            vclock_deltas.append(int(vf) - int(ve))

        # Position in second
        vclock_gnss = _frag(r, "time_test_vclock_gnss_ns")
        if vclock_gnss is not None:
            pos_ns = int(vclock_gnss) % NS_PER_SECOND
            positions.append(pos_ns)
            residual_position_pairs.append((pos_ns / 1e9, float(residual)))

        count += 1

    print(f"  Total TIMEBASE records:     {len(rows):,}")
    print(f"  Valid TIME_TEST samples:    {len(residuals):,}")
    print(f"  PPS bracket samples:        {len(brackets_gnss):,}")
    print(f"  Skipped (no field):         {skipped_no_field:,}")
    print(f"  Skipped (invalid):          {skipped_invalid:,}")
    print()

    if not residuals and not brackets_gnss:
        print("  No valid data found — cannot analyze.")
        return

    # ══════════════════════════════════════════════════════════════════
    # Section 1: PPS CORRECTION BRACKETS
    # ══════════════════════════════════════════════════════════════════

    if brackets_gnss:
        print("─" * 78)
        print("1. PPS ISR PERIPHERAL READ LATENCY (DWT cycles from snap_dwt)")
        print("─" * 78)
        print()
        print("  Each value is the DWT cycle count captured inside pps_correct_10mhz")
        print("  minus isr_snap_dwt.  This is the cost of reaching each peripheral.")
        print()

        for name, brackets in [("OCXO1 (GPT1)", brackets_ocxo1),
                                ("OCXO2 (GPT2)", brackets_ocxo2),
                                ("GNSS (QTimer)", brackets_gnss)]:
            w = Welford()
            for b in brackets:
                w.update(float(b))
            print(f"  {name}:")
            print(f"    n={w.n:,}  mean={w.mean:.1f}  stddev={w.stddev:.1f}"
                  f"  min={int(w.min_val)}  max={int(w.max_val)}  range={int(w.range)}")
            elapsed_mean = w.mean + PPS_ISR_FIXED_OVERHEAD
            ticks_mean = elapsed_mean / TICK_DIVISOR
            print(f"    With NVIC overhead (+{PPS_ISR_FIXED_OVERHEAD}):"
                  f"  elapsed={elapsed_mean:.1f}  ticks={ticks_mean:.2f}")
            print()

        # GNSS correction distribution
        print("  GNSS tick correction applied:")
        tick_counter = Counter(gnss_ticks_subtracted)
        for ticks in sorted(tick_counter.keys()):
            cnt = tick_counter[ticks]
            pct = (cnt / len(gnss_ticks_subtracted)) * 100
            print(f"    {ticks} ticks: {cnt:,} ({pct:.1f}%)")
        print()

        # GNSS elapsed histogram
        print("  GNSS elapsed (bracket + 48) distribution:")
        elapsed_counter = Counter(gnss_elapsed_values)
        for val in sorted(elapsed_counter.keys()):
            cnt = elapsed_counter[val]
            pct = (cnt / len(gnss_elapsed_values)) * 100
            tick_boundary = val // TICK_DIVISOR
            margin = val - (tick_boundary * TICK_DIVISOR)
            print(f"    {val:>4d} cycles → {tick_boundary} ticks"
                  f"  (margin={margin} to next boundary)  {cnt:,} ({pct:.1f}%)")
        print()

    # ══════════════════════════════════════════════════════════════════
    # Section 2: VCLOCK FIRE vs EDGE (COMP1 validation)
    # ══════════════════════════════════════════════════════════════════

    if vclock_deltas:
        print("─" * 78)
        print("2. VCLOCK AT FIRE vs AT EDGE (CH3 ISR read latency)")
        print("─" * 78)
        print()
        print("  vclock_at_fire  = free-running QTimer read in CH3 ISR")
        print("  vclock_at_edge  = COMP1 (exact compare-match tick)")
        print("  delta           = fire - edge (ISR read overshoot in ticks)")
        print()

        w_vd = Welford()
        for d in vclock_deltas:
            w_vd.update(float(d))

        print(f"    n={w_vd.n:,}  mean={w_vd.mean:.2f}  stddev={w_vd.stddev:.2f}")
        print(f"    min={int(w_vd.min_val)}  max={int(w_vd.max_val)}")
        print()

        delta_counter = Counter(vclock_deltas)
        for val in sorted(delta_counter.keys()):
            cnt = delta_counter[val]
            pct = (cnt / len(vclock_deltas)) * 100
            print(f"    {val} ticks: {cnt:,} ({pct:.1f}%)  = {val * 100} ns")
        print()

    # ══════════════════════════════════════════════════════════════════
    # Section 3: RESIDUAL DISTRIBUTION
    # ══════════════════════════════════════════════════════════════════

    if residuals:
        print("─" * 78)
        print("3. TIME_TEST RESIDUAL (computed_gnss_ns - vclock_gnss_ns)")
        print("─" * 78)
        print()

        w_resid = Welford()
        for r in residuals:
            w_resid.update(float(r))

        print(f"    n={w_resid.n:,}  mean={w_resid.mean:.2f} ns  stddev={w_resid.stddev:.2f} ns")
        print(f"    min={int(w_resid.min_val)} ns  max={int(w_resid.max_val)} ns"
              f"  range={int(w_resid.range)} ns")
        print()

        # Histogram
        bar_width = 50
        resid_counter = Counter(residuals)
        resid_vals = sorted(resid_counter.keys())
        max_count = max(resid_counter.values()) if resid_counter else 1

        print("  Distribution (ns):")
        for val in resid_vals:
            cnt = resid_counter[val]
            bar_len = int((cnt / max_count) * bar_width)
            bar = "█" * bar_len
            pct = (cnt / len(residuals)) * 100
            print(f"  {val:>5d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)")
        print()

        # ── Bimodal detection ──
        # Split residuals into clusters around 50 ns and 150 ns boundaries
        cluster_low = [r for r in residuals if r < 100]
        cluster_high = [r for r in residuals if r >= 100]

        if cluster_low and cluster_high:
            w_low = Welford()
            w_high = Welford()
            for r in cluster_low:
                w_low.update(float(r))
            for r in cluster_high:
                w_high.update(float(r))

            print("  Bimodal analysis (split at 100 ns):")
            print(f"    Low cluster  (<100 ns): n={w_low.n:,}"
                  f"  mean={w_low.mean:.1f} ns  stddev={w_low.stddev:.1f} ns"
                  f"  ({100 * w_low.n / len(residuals):.1f}%)")
            print(f"    High cluster (≥100 ns): n={w_high.n:,}"
                  f"  mean={w_high.mean:.1f} ns  stddev={w_high.stddev:.1f} ns"
                  f"  ({100 * w_high.n / len(residuals):.1f}%)")
            print(f"    Cluster separation:     {w_high.mean - w_low.mean:.1f} ns"
                  f"  (expect ~100 ns = 1 VCLOCK tick)")
            print()

    # ══════════════════════════════════════════════════════════════════
    # Section 4: ANCHOR QTIMER ANALYSIS
    # ══════════════════════════════════════════════════════════════════

    if residual_with_anchor:
        print("─" * 78)
        print("4. ANCHOR QTIMER ANALYSIS (correction jitter)")
        print("─" * 78)
        print()

        # Group by qtimer_at_pps last digit to detect tick oscillation
        last_digit_counter = Counter(anchor_last_digits)
        print("  qtimer_at_pps last digit distribution:")
        for digit in sorted(last_digit_counter.keys()):
            cnt = last_digit_counter[digit]
            pct = (cnt / len(anchor_last_digits)) * 100
            print(f"    ...{digit}: {cnt:,} ({pct:.1f}%)")
        print()

        # Correlate last digit with residual
        digit_to_residuals: Dict[int, List[int]] = {}
        for resid, qt in residual_with_anchor:
            d = qt % 10
            if d not in digit_to_residuals:
                digit_to_residuals[d] = []
            digit_to_residuals[d].append(resid)

        print("  Residual by anchor last digit:")
        for digit in sorted(digit_to_residuals.keys()):
            resids = digit_to_residuals[digit]
            w = Welford()
            for r in resids:
                w.update(float(r))
            print(f"    ...{digit}: n={w.n:,}  mean={w.mean:.1f} ns  stddev={w.stddev:.1f} ns")
        print()

    # ══════════════════════════════════════════════════════════════════
    # Section 5: POSITION-IN-SECOND DEPENDENCE
    # ══════════════════════════════════════════════════════════════════

    if residual_position_pairs:
        print("─" * 78)
        print("5. POSITION-IN-SECOND ANALYSIS (residual vs phase)")
        print("─" * 78)
        print()
        print("  time_dwt_to_gnss_ns() interpolates using the previous second's")
        print("  DWT rate.  Rate prediction error accumulates linearly.")
        print()

        reg = linear_regression(residual_position_pairs)
        if reg:
            slope, intercept, r_squared = reg
            print(f"    intercept = {intercept:.1f} ns"
                  f"  (residual at PPS edge = ISR overhead bias)")
            print(f"    slope     = {slope:.2f} ns/s"
                  f"  (= {slope:.2f} ppb DWT rate prediction error)")
            print(f"    R²        = {r_squared:.4f}")
            print()

            if abs(slope) < 1.0:
                print(f"  Assessment: EXCELLENT — sub-ppb DWT rate consistency")
            elif abs(slope) < 5.0:
                print(f"  Assessment: GOOD — {abs(slope):.1f} ppb DWT rate error")
            else:
                print(f"  Assessment: NOTABLE — {abs(slope):.1f} ppb DWT rate error per second")
        print()

    # ══════════════════════════════════════════════════════════════════
    # Section 6: PER-SAMPLE DETAIL (first 20 + last 10)
    # ══════════════════════════════════════════════════════════════════

    if residuals:
        print("─" * 78)
        print("6. PER-SAMPLE DETAIL")
        print("─" * 78)
        print()

        detail_header = (
            f"  {'pps':>6s}  {'resid':>6s}  {'pos_ms':>7s}"
            f"  {'vf-ve':>5s}  {'anchor_qt':>12s}  {'qt%10':>5s}"
            f"  {'gnss_el':>7s}  {'ticks':>5s}"
        )
        detail_sep = (
            f"  {'─' * 6}  {'─' * 6}  {'─' * 7}"
            f"  {'─' * 5}  {'─' * 12}  {'─' * 5}"
            f"  {'─' * 7}  {'─' * 5}"
        )
        print(detail_header)
        print(detail_sep)

        valid_rows = []
        for r in rows:
            if not _frag(r, "time_test_valid"):
                continue
            resid = _frag(r, "time_test_residual_ns")
            if resid is None:
                continue
            valid_rows.append(r)

        show_head = min(20, len(valid_rows))
        show_tail = min(10, max(0, len(valid_rows) - show_head))

        def print_detail_row(r):
            pps = r.get("pps_count", r.get("teensy_pps_count", "?"))
            resid = int(_frag(r, "time_test_residual_ns"))
            vclock_gnss = _frag(r, "time_test_vclock_gnss_ns")
            pos_ms = (int(vclock_gnss) % NS_PER_SECOND) / 1_000_000.0 if vclock_gnss else 0

            vf = _frag(r, "time_test_vclock_at_fire")
            ve = _frag(r, "time_test_vclock_at_edge")
            vf_ve = int(vf) - int(ve) if vf is not None and ve is not None else None

            anchor_qt = _frag(r, "time_test_diag_anchor_qtimer")
            qt_mod = int(anchor_qt) % 10 if anchor_qt is not None else None

            # Compute GNSS elapsed from diag fields
            snap = _frag(r, "isr_snap_dwt") or _frag(r, "dwt_cyccnt_at_pps")
            b_gnss = _frag(r, "diag_pps_correct_dwt_gnss")
            gnss_el = None
            ticks = None
            if snap is not None and b_gnss is not None:
                bracket = (int(b_gnss) - int(snap)) & 0xFFFFFFFF
                gnss_el = bracket + PPS_ISR_FIXED_OVERHEAD
                ticks = gnss_el // TICK_DIVISOR

            vf_ve_str = f"{vf_ve:>5d}" if vf_ve is not None else f"{'?':>5s}"
            qt_str = f"{int(anchor_qt):>12,}" if anchor_qt is not None else f"{'?':>12s}"
            qt_mod_str = f"{qt_mod:>5d}" if qt_mod is not None else f"{'?':>5s}"
            el_str = f"{gnss_el:>7d}" if gnss_el is not None else f"{'?':>7s}"
            tk_str = f"{ticks:>5d}" if ticks is not None else f"{'?':>5s}"

            print(f"  {pps:>6}  {resid:>+6d}  {pos_ms:>7.1f}"
                  f"  {vf_ve_str}  {qt_str}  {qt_mod_str}"
                  f"  {el_str}  {tk_str}")

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
    # Section 7: SUMMARY
    # ══════════════════════════════════════════════════════════════════

    print("=" * 78)
    print("SUMMARY")
    print("=" * 78)
    print()

    if residuals:
        w = Welford()
        for r in residuals:
            w.update(float(r))

        print(f"  TIME_TEST residual:")
        print(f"    Mean:     {w.mean:+.1f} ns")
        print(f"    Stddev:   {w.stddev:.1f} ns")
        print(f"    Range:    {int(w.min_val)} to {int(w.max_val)} ns")
        print()

        cluster_low = [r for r in residuals if r < 100]
        cluster_high = [r for r in residuals if r >= 100]
        if cluster_low and cluster_high:
            print(f"  Bimodal split:")
            print(f"    ~50 ns cluster:  {len(cluster_low):,}"
                  f" ({100 * len(cluster_low) / len(residuals):.1f}%)")
            print(f"    ~150 ns cluster: {len(cluster_high):,}"
                  f" ({100 * len(cluster_high) / len(residuals):.1f}%)")
            print(f"    Cause: pps_correct_10mhz tick quantization (±1 tick)")
        elif cluster_low:
            print(f"  Unimodal: all residuals < 100 ns")
            print(f"    Correction is consistently subtracting the right tick count.")
        print()

    if vclock_deltas:
        w_vd = Welford()
        for d in vclock_deltas:
            w_vd.update(float(d))
        print(f"  CH3 ISR VCLOCK overshoot:")
        print(f"    Mean: {w_vd.mean:.1f} ticks = {w_vd.mean * 100:.0f} ns")
        print(f"    COMP1 (vclock_at_edge) eliminates this entirely.")
        print()

    if gnss_ticks_subtracted:
        tick_counter = Counter(gnss_ticks_subtracted)
        print(f"  GNSS anchor correction:")
        for ticks in sorted(tick_counter.keys()):
            cnt = tick_counter[ticks]
            pct = (cnt / len(gnss_ticks_subtracted)) * 100
            print(f"    {ticks} ticks subtracted: {cnt:,} ({pct:.1f}%)")
        print()

    print(f"  Remaining ~50 ns baseline = PPS_ISR_FIXED_OVERHEAD ({PPS_ISR_FIXED_OVERHEAD} cycles)")
    print(f"  This will be addressed in Phase 2 (TDC correction at ISR time).")
    print()
    print("=" * 78)


# ─────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: pps_correction_analyzer <campaign_name> [limit]")
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