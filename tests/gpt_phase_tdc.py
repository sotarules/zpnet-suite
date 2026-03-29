"""
ZPNet GPT Phase TDC Analyzer — OCXO Edge ISR Latency Characterization

Analyzes the GPT output-compare ISR captures for OCXO1 and OCXO2
phase detection, analogous to what tdc_analyzer does for the PPS
spin capture.

Architecture:

  The PPS ISR arms GPT1/GPT2 output-compare interrupts to fire on
  the first OCXO edge after PPS.  Each GPT ISR captures DWT_CYCCNT
  as its first instruction, then subtracts a fixed ISR entry latency
  constant (GPT1_ISR_ENTRY_DWT_CYCLES = 49, GPT2 = 49) to estimate
  the DWT value at the actual hardware event.

  Unlike the PPS spin capture (which has a shadow-write loop for
  per-sample latency characterization), the GPT ISRs apply a single
  fixed correction.  This analyzer evaluates whether that correction
  is accurate by examining the phase offset stability and consistency
  between the two OCXOs.

Key fields from TIMEBASE records:

  ocxo1_phase_offset_ns   — phase offset [0, 100) ns
  ocxo2_phase_offset_ns   — phase offset [0, 100) ns
  ocxo1_residual_ns       — per-second edge-to-edge residual
  ocxo2_residual_ns       — per-second edge-to-edge residual
  ocxo1_edge_gnss_ns      — absolute GNSS ns at canonical edge
  ocxo2_edge_gnss_ns      — absolute GNSS ns at canonical edge
  phase_detector_valid     — both OCXOs captured
  phase_residual_valid     — consecutive edges available
  dwt_cycles_per_pps       — DWT calibration constant
  isr_residual_ocxo1       — ISR-level tick residual (10 MHz ground truth)
  isr_residual_ocxo2       — ISR-level tick residual

Analysis sections:

  1. Phase offset distribution — histogram and statistics
     If the ISR latency constant is correct, the phase offset should
     walk smoothly at a rate determined by the OCXO's ppb error.
     Random scatter indicates ISR latency jitter.

  2. Phase offset walk rate — does φ drift at the expected ppb rate?
     At N ppb, the phase walks through [0,100) in 100/N seconds.
     We measure the actual walk rate and compare.

  3. Phase-to-tick cross-check — do the DWT-interpolated residuals
     agree with the ISR-level tick residuals?

  4. Inter-OCXO correlation — do OCXO1 and OCXO2 phase offsets
     track each other?  If so, the jitter is in the shared DWT/PPS
     path, not the individual GPT ISRs.

  5. Consecutive phase differences — the second-to-second change
     in phase offset reveals whether the signal is smooth (real
     physics) or noisy (ISR jitter).

Usage:
    python -m zpnet.tests.gpt_phase_tdc <campaign_name> [limit]
    .zt gpt_phase_tdc Alpha1
    .zt gpt_phase_tdc Alpha1 500
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

DWT_FREQ_HZ = 1_008_000_000
DWT_NS_PER_CYCLE = 125.0 / 126.0
NS_PER_SECOND = 1_000_000_000


# ─────────────────────────────────────────────────────────────────────
# Welford
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

    def reset(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = None
        self.max_val = None


# ─────────────────────────────────────────────────────────────────────
# Field access — fragment-aware
# ─────────────────────────────────────────────────────────────────────

def _frag(row: Dict[str, Any], key: str, default=None):
    """Read from fragment sub-dict first, fall back to top-level."""
    frag = row.get("fragment")
    if frag is not None:
        val = frag.get(key)
        if val is not None:
            return val
    val = row.get(key)
    return val if val is not None else default


def _int(row: Dict[str, Any], key: str) -> Optional[int]:
    val = _frag(row, key)
    return int(val) if val is not None else None


def _float(row: Dict[str, Any], key: str) -> Optional[float]:
    val = _frag(row, key)
    return float(val) if val is not None else None


def _bool(row: Dict[str, Any], key: str) -> bool:
    val = _frag(row, key)
    return bool(val) if val is not None else False


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


# ─────────────────────────────────────────────────────────────────────
# Histogram helper
# ─────────────────────────────────────────────────────────────────────

def print_hist(title: str, values: Sequence[int], max_bar: int = 50) -> None:
    if not values:
        print(f"  {title}: no data")
        print()
        return

    print(f"  {title}")
    print()

    counts = Counter(values)
    total = len(values)
    max_count = max(counts.values())

    for val in sorted(counts.keys()):
        cnt = counts[val]
        pct = 100.0 * cnt / total
        bar_len = int((cnt / max_count) * max_bar) if max_count > 0 else 0
        bar = "█" * bar_len
        print(f"    {val:>4d}: {bar:<{max_bar}s} {cnt:>5d} ({pct:5.1f}%)")

    print()


# ─────────────────────────────────────────────────────────────────────
# Single-OCXO analysis
# ─────────────────────────────────────────────────────────────────────

def analyze_ocxo(
    rows: List[Dict[str, Any]],
    label: str,
    phase_key: str,
    residual_key: str,
    edge_gnss_key: str,
    isr_residual_key: str,
    ppb_key: str,
) -> Tuple[List[int], List[int]]:
    """
    Analyze one OCXO's phase capture data.

    Returns (phase_offsets, residuals) for inter-OCXO correlation.
    """

    print("=" * 70)
    print(f"{label} PHASE CAPTURE ANALYSIS")
    print("=" * 70)
    print()

    # ── Collect data ──

    phases: List[int] = []
    residuals: List[int] = []
    isr_residuals: List[int] = []
    edge_gnss_values: List[int] = []
    pps_values: List[int] = []

    for row in rows:
        if not _bool(row, "phase_detector_valid"):
            continue

        phase = _int(row, phase_key)
        resid = _int(row, residual_key)
        edge_gnss = _int(row, edge_gnss_key)
        isr_r = _int(row, isr_residual_key)
        pps = _int(row, "pps_count") or _int(row, "teensy_pps_count")

        if phase is not None:
            phases.append(phase)
            pps_values.append(pps or 0)

        if resid is not None and _bool(row, "phase_residual_valid"):
            residuals.append(resid)

        if isr_r is not None:
            isr_residuals.append(isr_r)

        if edge_gnss is not None:
            edge_gnss_values.append(edge_gnss)

    print(f"  Valid phase captures:  {len(phases):,}")
    print(f"  Valid residuals:       {len(residuals):,}")
    print(f"  ISR tick residuals:    {len(isr_residuals):,}")
    print()

    if not phases:
        print("  No valid data — skipping")
        print()
        return [], []

    # ── Section 1: Phase offset distribution ──

    print("-" * 70)
    print("PHASE OFFSET DISTRIBUTION (0–99 ns)")
    print("-" * 70)
    print()

    w_phase = Welford()
    for p in phases:
        w_phase.update(float(p))

    print(f"  n       = {w_phase.n:,}")
    print(f"  mean    = {w_phase.mean:.2f} ns")
    print(f"  stddev  = {w_phase.stddev:.2f} ns")
    if w_phase.min_val is not None:
        print(f"  min     = {int(w_phase.min_val)} ns")
        print(f"  max     = {int(w_phase.max_val)} ns")
        print(f"  range   = {int(w_phase.range)} ns")
    print()

    # Bin into 10-ns buckets for a coarse histogram
    buckets = [p // 10 for p in phases]
    bucket_counts = Counter(buckets)
    print(f"  10-ns bucket histogram:")
    for b in range(10):
        cnt = bucket_counts.get(b, 0)
        pct = 100.0 * cnt / len(phases)
        bar = "█" * int(pct / 2)
        print(f"    [{b*10:>2d}-{b*10+9:>2d}]: {cnt:>6d} ({pct:5.1f}%)  {bar}")
    print()

    # Assessment
    if w_phase.stddev < 5.0:
        print(f"  Assessment: TIGHT — phase offset is stable (σ < 5 ns)")
        print(f"  ISR latency constant appears well-calibrated.")
    elif w_phase.stddev < 15.0:
        print(f"  Assessment: MODERATE — phase has some spread (σ = {w_phase.stddev:.1f} ns)")
        print(f"  ISR latency constant may need refinement.")
    elif w_phase.stddev < 30.0:
        print(f"  Assessment: WIDE — significant phase spread (σ = {w_phase.stddev:.1f} ns)")
        print(f"  ISR latency jitter or calibration error suspected.")
    else:
        print(f"  Assessment: UNIFORM — phase appears random (σ = {w_phase.stddev:.1f} ns)")
        print(f"  If σ ≈ 29 ns (uniform [0,100)), the measurement has no")
        print(f"  sub-tick resolution.  ISR latency jitter dominates.")
    print()

    # ── Section 2: Phase walk rate ──

    print("-" * 70)
    print("PHASE WALK RATE")
    print("-" * 70)
    print()

    if len(phases) >= 10:
        # Compute consecutive phase differences (mod 100, signed)
        phase_diffs: List[int] = []
        for i in range(1, len(phases)):
            diff = phases[i] - phases[i - 1]
            # Unwrap: if diff > 50, it wrapped backward; if < -50, forward
            if diff > 50:
                diff -= 100
            elif diff < -50:
                diff += 100
            phase_diffs.append(diff)

        w_diff = Welford()
        for d in phase_diffs:
            w_diff.update(float(d))

        print(f"  Consecutive phase differences (mod 100, unwrapped):")
        print(f"    n       = {w_diff.n:,}")
        print(f"    mean    = {w_diff.mean:+.3f} ns/s")
        print(f"    stddev  = {w_diff.stddev:.3f} ns/s")
        if w_diff.min_val is not None:
            print(f"    range   = [{int(w_diff.min_val):+d}, {int(w_diff.max_val):+d}]")
        print()

        # Compare to expected walk rate from ppb
        # The phase should walk at ppb/10 ns/s through [0,100)
        # At 239 ppb (OCXO1 from sample), expect ~23.9 ns/s walk rate
        # But this is the RATE error — the phase walks because the OCXO
        # is not exactly 10 MHz.  The ISR-level tick residual gives us
        # the expected walk rate.
        if isr_residuals:
            mean_isr_r = sum(isr_residuals) / len(isr_residuals)
            # ISR residual is in 10 MHz ticks (2 or 3 means 200 or 300 ppb)
            expected_walk = mean_isr_r * 100.0  # ticks × 100 ns/tick = ns
            # But we need to subtract the integer part — the phase walk
            # is only the fractional 100-ns part
            expected_walk_frac = expected_walk % 100.0
            if expected_walk_frac > 50:
                expected_walk_frac -= 100.0

            print(f"  Expected walk rate from ISR tick residuals:")
            print(f"    Mean ISR residual: {mean_isr_r:+.2f} ticks/s")
            print(f"    Gross rate:        {expected_walk:+.1f} ns/s ({mean_isr_r * 100:.1f} ppb)")
            print(f"    Fractional walk:   {expected_walk_frac:+.1f} ns/s (mod 100)")
            print(f"    Measured walk:     {w_diff.mean:+.3f} ns/s")
            print()

            if abs(w_diff.mean - expected_walk_frac) < w_diff.stddev * 2:
                print(f"  ✓ Measured walk rate is consistent with expected rate")
            else:
                print(f"  ✗ Walk rate mismatch — DWT interpolation may have a bias")
                print(f"    Difference: {w_diff.mean - expected_walk_frac:+.3f} ns/s")
        print()

        # Histogram of phase differences
        print_hist(
            "CONSECUTIVE PHASE DIFFERENCE HISTOGRAM (ns/s)",
            phase_diffs,
        )
    else:
        print(f"  Insufficient data (n={len(phases)})")
        print()

    # ── Section 3: Residual analysis ──

    print("-" * 70)
    print("PHASE RESIDUAL (edge-to-edge − 1e9)")
    print("-" * 70)
    print()

    if residuals:
        w_resid = Welford()
        for r in residuals:
            w_resid.update(float(r))

        print(f"  n       = {w_resid.n:,}")
        print(f"  mean    = {w_resid.mean:+.2f} ns")
        print(f"  stddev  = {w_resid.stddev:.2f} ns")
        if w_resid.min_val is not None:
            print(f"  min     = {int(w_resid.min_val):+d} ns")
            print(f"  max     = {int(w_resid.max_val):+d} ns")
            print(f"  range   = {int(w_resid.range)} ns")
        print()

        # Cross-check: does the residual mean match the ISR tick residual?
        if isr_residuals:
            mean_isr = sum(isr_residuals) / len(isr_residuals)
            expected_resid = mean_isr * 100.0  # ticks × 100 ns/tick
            print(f"  Cross-check against ISR tick residual:")
            print(f"    Phase residual mean:  {w_resid.mean:+.2f} ns")
            print(f"    ISR tick residual:    {mean_isr:+.2f} ticks → {expected_resid:+.1f} ns expected")
            delta = w_resid.mean - expected_resid
            print(f"    Discrepancy:         {delta:+.2f} ns")
            print()

            if abs(delta) < 10:
                print(f"  ✓ Phase and tick residuals are consistent")
            else:
                print(f"  ✗ MISMATCH — phase residual does not track tick residual")
                print(f"    This is the core diagnostic: the DWT-interpolated edge")
                print(f"    timestamps are not faithfully tracking the OCXO rate.")
            print()

        print_hist(
            "RESIDUAL HISTOGRAM (ns)",
            residuals,
        )
    else:
        print(f"  No valid residuals")
        print()

    # ── Section 4: Per-second detail (first 30) ──

    print("-" * 70)
    print("PER-SECOND DETAIL (first 30 valid)")
    print("-" * 70)
    print()

    print(f"  {'pps':>6s}  {'φ':>4s}  {'Δφ':>5s}  {'resid':>6s}  {'isr_r':>5s}  {'edge_gnss':>18s}")
    print(f"  {'─'*6}  {'─'*4}  {'─'*5}  {'─'*6}  {'─'*5}  {'─'*18}")

    shown = 0
    prev_phase: Optional[int] = None

    for row in rows:
        if not _bool(row, "phase_detector_valid"):
            continue

        pps = _int(row, "pps_count") or _int(row, "teensy_pps_count")
        phase = _int(row, phase_key)
        resid = _int(row, residual_key)
        isr_r = _int(row, isr_residual_key)
        edge_g = _int(row, edge_gnss_key)
        res_valid = _bool(row, "phase_residual_valid")

        if phase is None:
            continue

        if prev_phase is not None:
            d = phase - prev_phase
            if d > 50:
                d -= 100
            elif d < -50:
                d += 100
            diff_str = f"{d:>+5d}"
        else:
            diff_str = f"{'---':>5s}"

        resid_str = f"{resid:>+6d}" if (resid is not None and res_valid) else f"{'---':>6s}"
        isr_r_str = f"{isr_r:>+5d}" if isr_r is not None else f"{'---':>5s}"
        edge_str = f"{edge_g:>18,}" if edge_g is not None else f"{'---':>18s}"

        print(f"  {pps:>6}  {phase:>4d}  {diff_str}  {resid_str}  {isr_r_str}  {edge_str}")

        prev_phase = phase
        shown += 1
        if shown >= 30:
            remaining = len(phases) - shown
            if remaining > 0:
                print(f"  ... {remaining} more rows ...")
            break

    print()

    return phases, residuals


# ─────────────────────────────────────────────────────────────────────
# Inter-OCXO correlation
# ─────────────────────────────────────────────────────────────────────

def analyze_correlation(
    rows: List[Dict[str, Any]],
    phases1: List[int],
    phases2: List[int],
) -> None:

    print("=" * 70)
    print("INTER-OCXO CORRELATION")
    print("=" * 70)
    print()

    if len(phases1) < 10 or len(phases2) < 10:
        print("  Insufficient data for correlation analysis")
        print()
        return

    # Collect paired samples (both valid on the same PPS)
    paired_p1: List[int] = []
    paired_p2: List[int] = []
    paired_diffs: List[int] = []

    for row in rows:
        if not _bool(row, "phase_detector_valid"):
            continue

        p1 = _int(row, "ocxo1_phase_offset_ns")
        p2 = _int(row, "ocxo2_phase_offset_ns")

        if p1 is not None and p2 is not None:
            paired_p1.append(p1)
            paired_p2.append(p2)

            diff = p1 - p2
            if diff > 50:
                diff -= 100
            elif diff < -50:
                diff += 100
            paired_diffs.append(diff)

    print(f"  Paired samples: {len(paired_p1):,}")
    print()

    if len(paired_diffs) < 2:
        return

    w_diff = Welford()
    for d in paired_diffs:
        w_diff.update(float(d))

    print(f"  Phase difference (φ1 − φ2):")
    print(f"    mean    = {w_diff.mean:+.2f} ns")
    print(f"    stddev  = {w_diff.stddev:.2f} ns")
    if w_diff.min_val is not None:
        print(f"    range   = [{int(w_diff.min_val):+d}, {int(w_diff.max_val):+d}] ns")
    print()

    if w_diff.stddev < 5.0:
        print(f"  Assessment: TIGHTLY CORRELATED")
        print(f"  Both OCXOs see the same DWT interpolation noise.")
        print(f"  The jitter source is in the shared PPS→DWT path,")
        print(f"  not in the individual GPT ISRs.")
    elif w_diff.stddev < 15.0:
        print(f"  Assessment: MODERATELY CORRELATED")
        print(f"  Some shared noise, some independent ISR jitter.")
    else:
        print(f"  Assessment: WEAKLY CORRELATED")
        print(f"  Individual GPT ISR latency variation dominates.")
    print()

    # Correlation coefficient
    if len(paired_p1) >= 3:
        mean1 = sum(paired_p1) / len(paired_p1)
        mean2 = sum(paired_p2) / len(paired_p2)

        cov = sum((a - mean1) * (b - mean2) for a, b in zip(paired_p1, paired_p2)) / (len(paired_p1) - 1)
        var1 = sum((a - mean1) ** 2 for a in paired_p1) / (len(paired_p1) - 1)
        var2 = sum((b - mean2) ** 2 for b in paired_p2) / (len(paired_p2) - 1)

        if var1 > 0 and var2 > 0:
            r = cov / math.sqrt(var1 * var2)
            print(f"  Pearson r(φ1, φ2) = {r:+.4f}")
            if abs(r) > 0.7:
                print(f"  Strong correlation — shared noise dominates")
            elif abs(r) > 0.3:
                print(f"  Moderate correlation — mixed noise sources")
            else:
                print(f"  Weak/no correlation — independent noise sources")
            print()


# ─────────────────────────────────────────────────────────────────────
# Main analysis
# ─────────────────────────────────────────────────────────────────────

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    if limit > 0:
        rows = rows[:limit]

    print("=" * 70)
    print(f"ZPNet GPT PHASE TDC ANALYZER")
    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE records)")
    print("=" * 70)
    print()

    # ── Capture rate ──
    valid = sum(1 for r in rows if _bool(r, "phase_detector_valid"))
    invalid = len(rows) - valid
    print(f"  Phase detector valid:   {valid:,}")
    print(f"  Phase detector invalid: {invalid:,}")
    if valid > 0:
        print(f"  Capture rate:           {100 * valid / len(rows):.1f}%")
    print()

    # ── ISR fire counts ──
    gpt1_fires = _int(rows[-1], "diag_gpt1_isr_fires")
    gpt2_fires = _int(rows[-1], "diag_gpt2_isr_fires")
    ox1_captures = _int(rows[-1], "diag_ocxo1_phase_captures")
    ox1_misses = _int(rows[-1], "diag_ocxo1_phase_misses")
    ox2_captures = _int(rows[-1], "diag_ocxo2_phase_captures")
    ox2_misses = _int(rows[-1], "diag_ocxo2_phase_misses")

    if gpt1_fires is not None:
        print(f"  GPT1 ISR fires: {gpt1_fires:,}    GPT2 ISR fires: {gpt2_fires:,}")
    if ox1_captures is not None:
        print(f"  OCXO1 captures: {ox1_captures:,}  misses: {ox1_misses:,}")
    if ox2_captures is not None:
        print(f"  OCXO2 captures: {ox2_captures:,}  misses: {ox2_misses:,}")
    print()

    # ── Per-OCXO analysis ──

    phases1, resids1 = analyze_ocxo(
        rows, "OCXO1",
        "ocxo1_phase_offset_ns", "ocxo1_residual_ns",
        "ocxo1_edge_gnss_ns", "isr_residual_ocxo1",
        "ocxo1_ppb",
    )

    phases2, resids2 = analyze_ocxo(
        rows, "OCXO2",
        "ocxo2_phase_offset_ns", "ocxo2_residual_ns",
        "ocxo2_edge_gnss_ns", "isr_residual_ocxo2",
        "ocxo2_ppb",
    )

    # ── Inter-OCXO correlation ──

    analyze_correlation(rows, phases1, phases2)

    # ── Summary ──

    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print()
    print(f"  Campaign:              {campaign}")
    print(f"  Records:               {len(rows):,}")
    print(f"  Detector valid:        {valid:,} ({100 * valid / len(rows):.1f}%)")
    print()
    print(f"  Current ISR latency constants (from isr_dwt_compensate.h):")
    print(f"    GPT1_ISR_ENTRY_DWT_CYCLES = 49")
    print(f"    GPT2_ISR_ENTRY_DWT_CYCLES = 49")
    print()
    print(f"  To calibrate: adjust GPTx_ISR_ENTRY_DWT_CYCLES until")
    print(f"  the phase offset mean is stable and the phase walk rate")
    print(f"  matches the expected rate from ISR tick residuals.")
    print()
    print(f"  NOTE: If phase stddev ≈ 29 ns (uniform distribution),")
    print(f"  the DWT interpolation has no sub-tick resolution and")
    print(f"  ISR latency jitter is the dominant error source.")
    print(f"  Consider adding a shadow-write loop (PPS TDC pattern)")
    print(f"  to the GPT ISRs for per-sample latency characterization.")
    print()
    print("=" * 70)


# ─────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: gpt_phase_tdc <campaign_name> [limit]")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, count(*) as cnt,
                           count(*) FILTER (
                               WHERE (payload->>'phase_detector_valid')::boolean = true
                                  OR (payload->'fragment'->>'phase_detector_valid')::boolean = true
                           ) as phase_valid
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s} {'PHASE OK':>9s}")
                print(f"  {'─' * 20} {'─' * 8} {'─' * 9}")
                for r in rows:
                    print(f"  {r['campaign']:<20s} {r['cnt']:>8d} {r['phase_valid']:>9d}")
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    limit_val = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit_val)


if __name__ == "__main__":
    main()