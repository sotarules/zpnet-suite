"""
ZPNet PPS Edge Analyzer — Diagnose pre-PPS spin loop and TDC edge detection

Usage:
    python -m zpnet.tests.pps_edge_analyzer <campaign_name>
    .zt pps_edge_analyzer Test6

Reads all TIMEBASE rows for the named campaign and analyzes the
PPS edge detection pipeline:

  1. Was the spin loop running when PPS arrived? (pps_edge_valid)
  2. What was the dispatch latency? (dispatch_delta_ns)
  3. How long did the spin loop actually run? (approach_ns)
  4. Was the fine timer late? (fine_was_late)
  5. What correction was applied? (pps_edge_correction_ns)
  6. Per-second timer event rates (from monotonic counter diffs)
  7. How does dispatch latency correlate with GNSS discipline state?
  8. Are there temporal patterns (clustered failures, degradation)?

The pre-PPS pipeline:
  • TIMEPOP coarse fires at ~500ms, arms the fine timer
  • TIMEPOP fine fires at (1000 - PRE_PPS_US) ms
  • Spin loop runs until PPS edge arrives (~PRE_PPS_US later)
  • PPS ISR captures shadow DWT, TDC corrects to true edge

If PRE_PPS_US is too small, the fine timer may fire AFTER the
PPS edge (interrupt latency, USB SOF, etc.), causing pps_edge_valid=false.
This tool quantifies the failure rate and helps size the margin.

Data sources (from TIMEBASE JSONB payload):

  Per-edge state:
    diag_teensy_pps_edge_valid          — did TDC produce a valid correction?
    diag_teensy_dispatch_delta_ns       — ISR-to-shadow latency (ns)
    diag_teensy_dispatch_shadow_ns      — last spin loop DWT ns before ISR
    diag_teensy_dispatch_isr_ns         — DWT ns captured at ISR entry
    diag_teensy_pps_edge_correction_ns  — TDC correction applied (ns)
    diag_teensy_tdc_needs_recal         — firmware flag: constants stale
    diag_teensy_approach_ns             — GNSS ns from spin entry to PPS edge
    diag_teensy_dispatch_timeout        — spin loop timed out (bool)
    diag_teensy_fine_was_late           — fine timer arrived after PPS (bool)
    diag_teensy_spin_iters              — iterations in spin loop (0 in tight)

  Monotonic counters (diff consecutive for per-second rates):
    diag_teensy_coarse_fires            — coarse timer fires
    diag_teensy_fine_fires              — fine timer fires
    diag_teensy_late_count              — fine timer arrived after PPS
    diag_teensy_spin_count              — spin loop entries
    diag_teensy_timeout_count           — spin loop timeouts

  Raw TDC data:
    diag_raw_isr_cyc / diag_raw_shadow_cyc — raw 32-bit DWT values

  Clock residuals:
    isr_residual_dwt                    — DWT residual (ns, uncorrected jitter)

  GNSS discipline (from gnss block):
    gnss.pps_timing_error_ns            — GF-8802 phase detector output
    gnss.freq_mode_name                 — discipline loop state
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional

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

    @property
    def stderr(self) -> float:
        return (self.stddev / math.sqrt(self.n)) if self.n >= 2 else 0.0


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def pct(num: int, den: int) -> str:
    if den == 0:
        return "—"
    return f"{num / den * 100:.1f}%"


def format_ns(ns: Optional[float]) -> str:
    if ns is None:
        return "—"
    if abs(ns) >= 1_000_000:
        return f"{ns / 1_000_000:.3f} ms"
    if abs(ns) >= 1000:
        return f"{ns / 1000:.3f} µs"
    return f"{ns:.1f} ns"


def safe_int(val: Any) -> Optional[int]:
    if val is None:
        return None
    try:
        return int(val)
    except (ValueError, TypeError):
        return None


# ---------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"❌ No TIMEBASE rows found for campaign '{campaign}'")
        return

    print("=" * 70)
    print(f"PPS EDGE ANALYSIS: {campaign}")
    print("=" * 70)
    print()
    print(f"  Total TIMEBASE records: {len(rows)}")
    print()

    # =================================================================
    # Extract all fields into parallel lists
    # =================================================================

    valid_count = 0
    invalid_count = 0
    null_count = 0

    delta_ns_values: List[int] = []
    delta_ns_valid: List[int] = []
    delta_ns_invalid: List[int] = []

    correction_ns_values: List[int] = []

    approach_ns_values: List[int] = []
    approach_ns_valid: List[int] = []
    approach_ns_invalid: List[int] = []

    raw_deltas: List[int] = []

    dwt_residuals: List[int] = []

    gnss_phase_errors: List[int] = []
    gnss_phase_when_valid: List[int] = []
    gnss_phase_when_invalid: List[int] = []

    # Per-edge booleans
    fine_late_count = 0
    timeout_count = 0
    tdc_needs_recal_count = 0

    # Timeline entries
    timeline: List[Dict[str, Any]] = []

    # Failure run tracking
    current_run_length = 0
    failure_runs: List[int] = []

    # Monotonic counter diffs (per-second event rates)
    prev_coarse: Optional[int] = None
    prev_fine: Optional[int] = None
    prev_late: Optional[int] = None
    prev_spins: Optional[int] = None
    prev_timeouts: Optional[int] = None

    coarse_diffs: List[int] = []
    fine_diffs: List[int] = []
    late_diffs: List[int] = []
    spin_diffs: List[int] = []
    timeout_diffs: List[int] = []

    for r in rows:
        pps = r.get("pps_count", 0)
        edge_valid = r.get("diag_teensy_pps_edge_valid")
        delta_ns = safe_int(r.get("diag_teensy_dispatch_delta_ns"))
        correction_ns = safe_int(r.get("diag_teensy_pps_edge_correction_ns"))
        approach_ns = safe_int(r.get("diag_teensy_approach_ns"))
        isr_cyc = safe_int(r.get("diag_raw_isr_cyc"))
        shadow_cyc = safe_int(r.get("diag_raw_shadow_cyc"))
        dwt_resid = safe_int(r.get("isr_residual_dwt"))
        fine_late = r.get("diag_teensy_fine_was_late")
        timeout = r.get("diag_teensy_dispatch_timeout")
        tdc_recal = r.get("diag_teensy_tdc_needs_recal")

        gnss = r.get("gnss") or {}
        gnss_phase = safe_int(gnss.get("pps_timing_error_ns"))

        # Monotonic counters
        cur_coarse = safe_int(r.get("diag_teensy_coarse_fires"))
        cur_fine = safe_int(r.get("diag_teensy_fine_fires"))
        cur_late = safe_int(r.get("diag_teensy_late_count"))
        cur_spins = safe_int(r.get("diag_teensy_spin_count"))
        cur_timeouts = safe_int(r.get("diag_teensy_timeout_count"))

        if prev_coarse is not None and cur_coarse is not None:
            coarse_diffs.append(cur_coarse - prev_coarse)
        if prev_fine is not None and cur_fine is not None:
            fine_diffs.append(cur_fine - prev_fine)
        if prev_late is not None and cur_late is not None:
            late_diffs.append(cur_late - prev_late)
        if prev_spins is not None and cur_spins is not None:
            spin_diffs.append(cur_spins - prev_spins)
        if prev_timeouts is not None and cur_timeouts is not None:
            timeout_diffs.append(cur_timeouts - prev_timeouts)

        prev_coarse = cur_coarse
        prev_fine = cur_fine
        prev_late = cur_late
        prev_spins = cur_spins
        prev_timeouts = cur_timeouts

        # Edge validity
        if edge_valid is None:
            null_count += 1
        elif edge_valid:
            valid_count += 1
        else:
            invalid_count += 1

        # Dispatch delta
        if delta_ns is not None and delta_ns >= 0:
            delta_ns_values.append(delta_ns)
            if edge_valid:
                delta_ns_valid.append(delta_ns)
            elif edge_valid is not None:
                delta_ns_invalid.append(delta_ns)

        # Approach time
        if approach_ns is not None and approach_ns >= 0:
            approach_ns_values.append(approach_ns)
            if edge_valid:
                approach_ns_valid.append(approach_ns)
            elif edge_valid is not None:
                approach_ns_invalid.append(approach_ns)

        # Correction
        if correction_ns is not None and correction_ns >= 0:
            correction_ns_values.append(correction_ns)

        # Raw cycles
        if isr_cyc is not None and shadow_cyc is not None:
            delta = (isr_cyc - shadow_cyc) & 0xFFFFFFFF
            if 0 < delta < 10000:
                raw_deltas.append(delta)

        # DWT residual
        if dwt_resid is not None:
            dwt_residuals.append(dwt_resid)

        # Per-edge booleans
        if fine_late is True:
            fine_late_count += 1
        if timeout is True:
            timeout_count += 1
        if tdc_recal is True:
            tdc_needs_recal_count += 1

        # GNSS phase error
        if gnss_phase is not None:
            gnss_phase_errors.append(gnss_phase)
            if edge_valid:
                gnss_phase_when_valid.append(gnss_phase)
            elif edge_valid is not None:
                gnss_phase_when_invalid.append(gnss_phase)

        # Timeline
        timeline.append({
            "pps": pps,
            "valid": edge_valid,
            "delta_ns": delta_ns if delta_ns is not None and delta_ns >= 0 else None,
            "approach_ns": approach_ns if approach_ns is not None and approach_ns >= 0 else None,
            "fine_late": fine_late,
            "timeout": timeout,
            "cold_start": (pps == 0 and edge_valid is False and
                           (delta_ns is None or delta_ns < 0) and
                           (approach_ns is None or approach_ns < 0)),
        })

        # Failure run tracking
        if edge_valid is False:
            current_run_length += 1
        else:
            if current_run_length > 0:
                failure_runs.append(current_run_length)
            current_run_length = 0

    if current_run_length > 0:
        failure_runs.append(current_run_length)

    # =================================================================
    # Section 1: Edge validity overview
    # =================================================================

    total_classified = valid_count + invalid_count

    # Cold-start failures: pps_count=0 with no spin loop data.
    # These are structural (the spin loop hasn't armed yet on the
    # very first PPS after START/RECOVER) and are not indicative
    # of any timing or TDC problem.
    cold_start_count = sum(1 for t in timeline if t.get("cold_start"))
    real_invalid = invalid_count - cold_start_count
    real_classified = valid_count + real_invalid

    print("-" * 70)
    print("EDGE VALIDITY")
    print("-" * 70)
    print(f"  Valid (TDC correction applied): {valid_count:6d}  ({pct(valid_count, total_classified)})")
    print(f"  Invalid (fallback to ISR):      {invalid_count:6d}  ({pct(invalid_count, total_classified)})")
    if cold_start_count:
        print(f"    of which cold start (pps=0):  {cold_start_count:6d}  (structural, not a fault)")
    if null_count:
        print(f"  Null (no data):                 {null_count:6d}")
    print()

    # Per-edge boolean summary
    print(f"  Fine timer was late:            {fine_late_count:6d}  ({pct(fine_late_count, len(rows))})")
    print(f"  Dispatch timeout:               {timeout_count:6d}  ({pct(timeout_count, len(rows))})")
    if tdc_needs_recal_count > 0:
        print(f"  TDC needs recalibration:        {tdc_needs_recal_count:6d}  ({pct(tdc_needs_recal_count, len(rows))})")
    print()

    if real_invalid == 0 and valid_count > 0:
        if cold_start_count:
            print(f"  ✅ 100% edge detection (excluding {cold_start_count} cold-start PPS)")
        else:
            print("  ✅ 100% edge detection — TDC correction active on every PPS")
    elif real_classified > 0:
        fail_rate = real_invalid / real_classified * 100
        if fail_rate > 50:
            print(f"  🚨 {fail_rate:.1f}% failure rate — spin loop is NOT reaching PPS edge")
            if fine_late_count > invalid_count * 0.8:
                print(f"     Root cause: fine timer arriving AFTER PPS ({fine_late_count} late)")
                print(f"     → PRE_PPS_US is too small")
            elif timeout_count > invalid_count * 0.5:
                print(f"     Root cause: spin loop timeouts ({timeout_count})")
        elif fail_rate > 5:
            print(f"  ⚠️  {fail_rate:.1f}% failure rate — intermittent spin loop misses")
        else:
            print(f"  ℹ️  {fail_rate:.1f}% failure rate — occasional misses (may be acceptable)")
    print()

    # =================================================================
    # Section 2: Monotonic counter rates (per-second event rates)
    # =================================================================

    if coarse_diffs or fine_diffs or late_diffs:
        print("-" * 70)
        print("TIMER EVENT RATES (per-second, from monotonic counter diffs)")
        print("-" * 70)

        def _rate_summary(name: str, diffs: List[int]) -> None:
            if not diffs:
                return
            total = sum(diffs)
            nonzero = sum(1 for d in diffs if d > 0)
            multi = sum(1 for d in diffs if d > 1)
            print(f"  {name}:")
            print(f"    total events: {total}  over {len(diffs)} seconds")
            print(f"    seconds with event: {nonzero}  ({pct(nonzero, len(diffs))})")
            if multi:
                print(f"    seconds with >1 event: {multi}  ({pct(multi, len(diffs))})")
            print()

        _rate_summary("Coarse timer fires", coarse_diffs)
        _rate_summary("Fine timer fires", fine_diffs)
        _rate_summary("Fine timer late (after PPS)", late_diffs)
        _rate_summary("Spin loop entries", spin_diffs)
        _rate_summary("Spin loop timeouts", timeout_diffs)

        # Cross-check: fine fires should equal spin entries + late arrivals
        if fine_diffs and spin_diffs and late_diffs:
            total_fine = sum(fine_diffs)
            total_spins = sum(spin_diffs)
            total_late = sum(late_diffs)
            if total_fine > 0:
                accounted = total_spins + total_late
                if accounted == total_fine:
                    print(f"  ✅ Accounting: fine_fires({total_fine}) = spins({total_spins}) + late({total_late})")
                else:
                    print(f"  ⚠️  Accounting mismatch: fine_fires({total_fine}) ≠ spins({total_spins}) + late({total_late}) = {accounted}")
                print()

    # =================================================================
    # Section 3: Approach time (how long the spin loop ran)
    # =================================================================

    if approach_ns_values:
        print("-" * 70)
        print("APPROACH TIME (GNSS ns from spin loop entry to PPS edge)")
        print("-" * 70)
        print()
        print("  This is the actual time the spin loop ran before PPS arrived.")
        print("  It directly measures the PRE_PPS_US safety margin.")
        print()

        stats = WelfordStats()
        for a in approach_ns_values:
            stats.update(float(a))

        print(f"  All samples (n={stats.n}):")
        print(f"    mean   = {format_ns(stats.mean)}")
        print(f"    stddev = {format_ns(stats.stddev)}")
        print(f"    min    = {format_ns(stats.min_val)}")
        print(f"    max    = {format_ns(stats.max_val)}")
        print()

        if approach_ns_valid:
            sv = WelfordStats()
            for a in approach_ns_valid:
                sv.update(float(a))
            print(f"  When edge valid (n={sv.n}):")
            print(f"    mean   = {format_ns(sv.mean)}")
            print(f"    min    = {format_ns(sv.min_val)}")
            print(f"    max    = {format_ns(sv.max_val)}")
            print()

        if approach_ns_invalid:
            si = WelfordStats()
            for a in approach_ns_invalid:
                si.update(float(a))
            print(f"  When edge INVALID (n={si.n}):")
            print(f"    mean   = {format_ns(si.mean)}")
            print(f"    min    = {format_ns(si.min_val)}")
            print(f"    max    = {format_ns(si.max_val)}")
            print()

        # Histogram in adaptive bins
        max_approach = max(approach_ns_values)
        if max_approach > 500_000:
            bin_ns = 100_000
            bin_label = "100µs"
        elif max_approach > 50_000:
            bin_ns = 10_000
            bin_label = "10µs"
        else:
            bin_ns = 1_000
            bin_label = "1µs"

        binned: Counter = Counter()
        for a in approach_ns_values:
            binned[a // bin_ns * bin_ns] += 1

        sorted_bins = sorted(binned.keys())
        max_count = max(binned.values())
        bar_width = 50

        print(f"  APPROACH TIME HISTOGRAM ({bin_label} bins):")
        for b in sorted_bins:
            cnt = binned[b]
            bar_len = int((cnt / max_count) * bar_width)
            bar = "█" * bar_len
            p = cnt / len(approach_ns_values) * 100
            label = f"{b / 1000:7.0f}µs"
            print(f"  {label} │ {bar:<{bar_width}s} {cnt:5d} ({p:5.1f}%)")

        print()

    # =================================================================
    # Section 4: Dispatch delta statistics
    # =================================================================

    if delta_ns_values:
        print("-" * 70)
        print("DISPATCH DELTA (ISR snapshot - spin shadow, nanoseconds)")
        print("-" * 70)

        stats_all = WelfordStats()
        for d in delta_ns_values:
            stats_all.update(float(d))

        print(f"  All samples (n={stats_all.n}):")
        print(f"    mean   = {format_ns(stats_all.mean)}")
        print(f"    stddev = {format_ns(stats_all.stddev)}")
        print(f"    min    = {format_ns(stats_all.min_val)}")
        print(f"    max    = {format_ns(stats_all.max_val)}")
        print()

        if delta_ns_valid:
            stats_v = WelfordStats()
            for d in delta_ns_valid:
                stats_v.update(float(d))
            print(f"  When edge valid (n={stats_v.n}):")
            print(f"    mean   = {format_ns(stats_v.mean)}")
            print(f"    stddev = {format_ns(stats_v.stddev)}")
            print(f"    min    = {format_ns(stats_v.min_val)}")
            print(f"    max    = {format_ns(stats_v.max_val)}")
            print()

        if delta_ns_invalid:
            stats_i = WelfordStats()
            for d in delta_ns_invalid:
                stats_i.update(float(d))
            print(f"  When edge INVALID (n={stats_i.n}):")
            print(f"    mean   = {format_ns(stats_i.mean)}")
            print(f"    stddev = {format_ns(stats_i.stddev)}")
            print(f"    min    = {format_ns(stats_i.min_val)}")
            print(f"    max    = {format_ns(stats_i.max_val)}")
            print()

        # Histogram in 10ns bins
        BIN_NS = 10
        binned_d: Counter = Counter()
        for d in delta_ns_values:
            binned_d[d // BIN_NS * BIN_NS] += 1

        sorted_bins = sorted(binned_d.keys())
        max_count = max(binned_d.values())
        bar_width = 50

        print("  DISPATCH DELTA HISTOGRAM (10ns bins):")
        for b in sorted_bins:
            cnt = binned_d[b]
            bar_len = int((cnt / max_count) * bar_width)
            bar = "█" * bar_len
            p = cnt / len(delta_ns_values) * 100
            label = f"{b:5d}-{b + BIN_NS - 1}"
            print(f"  {label} ns │ {bar:<{bar_width}s} {cnt:5d} ({p:5.1f}%)")

        print()

    # =================================================================
    # Section 5: Raw cycle delta analysis (exact TDC data)
    # =================================================================

    if raw_deltas:
        print("-" * 70)
        print("RAW CYCLE DELTAS (isr_cyc - shadow_cyc)")
        print("-" * 70)

        raw_stats = WelfordStats()
        for d in raw_deltas:
            raw_stats.update(float(d))

        print(f"  n      = {raw_stats.n}")
        print(f"  mean   = {raw_stats.mean:.2f} cycles ({raw_stats.mean * 125 / 126:.1f} ns)")
        print(f"  stddev = {raw_stats.stddev:.2f}")
        print(f"  min    = {int(raw_stats.min_val)} cycles ({int(raw_stats.min_val) * 125 / 126:.1f} ns)")
        print(f"  max    = {int(raw_stats.max_val)} cycles ({int(raw_stats.max_val) * 125 / 126:.1f} ns)")
        print()

        counter = Counter(raw_deltas)
        all_vals = sorted(counter.keys())
        max_count = max(counter.values())
        bar_width = 50

        print("  CYCLE HISTOGRAM:")
        last_val = None
        for val in all_vals:
            if last_val is not None and val - last_val > 10:
                print(f"       │ {'':.<{bar_width}s}  ···")
            cnt = counter[val]
            bar_len = int((cnt / max_count) * bar_width)
            bar = "█" * bar_len
            p = cnt / len(raw_deltas) * 100
            ns_approx = val * 125 / 126
            print(f"  {val:4d} │ {bar:<{bar_width}s} {cnt:5d} ({p:5.1f}%)  ~{ns_approx:.1f}ns")
            last_val = val

        print()

    # =================================================================
    # Section 6: TDC correction distribution
    # =================================================================

    if correction_ns_values:
        print("-" * 70)
        print("TDC CORRECTION (applied to reconstruct true PPS edge)")
        print("-" * 70)

        corr_stats = WelfordStats()
        for c in correction_ns_values:
            corr_stats.update(float(c))

        print(f"  n      = {corr_stats.n}")
        print(f"  mean   = {format_ns(corr_stats.mean)}")
        print(f"  stddev = {format_ns(corr_stats.stddev)}")
        print(f"  min    = {format_ns(corr_stats.min_val)}")
        print(f"  max    = {format_ns(corr_stats.max_val)}")
        print()

        counter = Counter(correction_ns_values)
        sorted_vals = sorted(counter.keys())
        max_count = max(counter.values())
        bar_width = 50

        for val in sorted_vals:
            cnt = counter[val]
            bar_len = int((cnt / max_count) * bar_width)
            bar = "█" * bar_len
            p = cnt / len(correction_ns_values) * 100
            print(f"  {val:4d} ns │ {bar:<{bar_width}s} {cnt:5d} ({p:5.1f}%)")

        print()

    # =================================================================
    # Section 7: Failure patterns
    # =================================================================

    if invalid_count > 0:
        print("-" * 70)
        print("FAILURE PATTERNS")
        print("-" * 70)

        # Classify failure modes using the new per-edge booleans
        failures_cold_start = sum(
            1 for t in timeline if t.get("cold_start")
        )
        failures_fine_late = sum(
            1 for t in timeline if t["valid"] is False and not t.get("cold_start") and t["fine_late"] is True
        )
        failures_timeout = sum(
            1 for t in timeline if t["valid"] is False and not t.get("cold_start") and t["timeout"] is True
        )
        failures_no_spin = sum(
            1 for t in timeline if t["valid"] is False and not t.get("cold_start") and t["delta_ns"] is None
        )
        failures_tdc_reject = sum(
            1 for t in timeline if t["valid"] is False and not t.get("cold_start") and t["delta_ns"] is not None
        )

        print(f"  Failure mode breakdown ({invalid_count} total failures):")
        print()
        if failures_cold_start:
            print(f"    Cold start (pps=0, structural):       {failures_cold_start:5d}  ({pct(failures_cold_start, invalid_count)})")
        if failures_fine_late:
            print(f"    Fine timer late (PPS already fired):  {failures_fine_late:5d}  ({pct(failures_fine_late, invalid_count)})")
        if failures_timeout:
            print(f"    Spin loop timeout:                    {failures_timeout:5d}  ({pct(failures_timeout, invalid_count)})")
        if failures_no_spin:
            print(f"    No spin loop data (no delta):         {failures_no_spin:5d}  ({pct(failures_no_spin, invalid_count)})")
        if failures_tdc_reject:
            print(f"    TDC rejected (delta out of range):    {failures_tdc_reject:5d}  ({pct(failures_tdc_reject, invalid_count)})")
        print()

        # Run length analysis
        print(f"  Total failure runs: {len(failure_runs)}")
        if failure_runs:
            print(f"  Longest run:        {max(failure_runs)} consecutive PPS edges")
            print(f"  Mean run length:    {sum(failure_runs) / len(failure_runs):.1f}")
            print()

            run_counter = Counter(failure_runs)
            print("  Run length distribution:")
            for length in sorted(run_counter.keys()):
                cnt = run_counter[length]
                print(f"    {length:4d} consecutive: {cnt:5d} runs")
            print()

        # Total blackout check (exclude cold starts)
        if valid_count == 0 and real_invalid > 0:
            print("  🚨 TOTAL BLACKOUT — spin loop never reached PPS edge")
            print("     The spin loop is starting AFTER PPS fires every time.")
            print("     PRE_PPS_US must be increased significantly.")
            print()

        # Temporal clustering (10 buckets)
        if len(timeline) >= 20:
            bucket_size = len(timeline) // 10
            print("  Failure rate over time (10 buckets):")
            print()
            for i in range(10):
                start = i * bucket_size
                end = start + bucket_size if i < 9 else len(timeline)
                bucket = timeline[start:end]

                bucket_valid = sum(1 for t in bucket if t["valid"] is True)
                bucket_invalid = sum(1 for t in bucket if t["valid"] is False)
                bucket_total = bucket_valid + bucket_invalid

                pps_start = bucket[0]["pps"]
                pps_end = bucket[-1]["pps"]
                rate = bucket_invalid / bucket_total * 100 if bucket_total > 0 else 0

                bar_len = int(rate / 2)
                bar = "█" * bar_len

                print(f"    pps {pps_start:6d}-{pps_end:6d}: {rate:5.1f}% fail  {bar}")

            print()

        # Individual failures with full context
        failures_with_context = [t for t in timeline if t["valid"] is False]

        if len(failures_with_context) <= 20:
            label = "All failures"
            show = failures_with_context
        else:
            label = "First 10 + last 10 failures"
            show = failures_with_context[:10] + [None] + failures_with_context[-10:]

        if show:
            print(f"  {label} (with context):")
            print(f"    {'PPS':>8s}  {'DELTA':>12s}  {'APPROACH':>12s}  {'LATE':>5s}  {'TMOUT':>5s}")
            print(f"    {'─' * 8}  {'─' * 12}  {'─' * 12}  {'─' * 5}  {'─' * 5}")

            for f in show:
                if f is None:
                    print(f"    {'···':>8s}")
                    continue
                delta_str = format_ns(f["delta_ns"]) if f["delta_ns"] is not None else "no spin"
                approach_str = format_ns(f["approach_ns"]) if f["approach_ns"] is not None else "—"
                late_str = "YES" if f["fine_late"] else "no"
                timeout_str = "YES" if f["timeout"] else "no"
                print(f"    {f['pps']:8d}  {delta_str:>12s}  {approach_str:>12s}  {late_str:>5s}  {timeout_str:>5s}")

            print()

    # =================================================================
    # Section 8: DWT residual impact
    # =================================================================

    if dwt_residuals:
        print("-" * 70)
        print("DWT RESIDUAL (uncorrected ISR jitter)")
        print("-" * 70)

        resid_stats = WelfordStats()
        for rv in dwt_residuals:
            resid_stats.update(float(rv))

        print(f"  n      = {resid_stats.n}")
        print(f"  mean   = {format_ns(resid_stats.mean)}")
        print(f"  stddev = {format_ns(resid_stats.stddev)}")
        print(f"  min    = {format_ns(resid_stats.min_val)}")
        print(f"  max    = {format_ns(resid_stats.max_val)}")
        print()

        if valid_count > 0 and invalid_count > 0:
            resid_valid: List[int] = []
            resid_invalid: List[int] = []
            for r in rows:
                rv = safe_int(r.get("isr_residual_dwt"))
                ev = r.get("diag_teensy_pps_edge_valid")
                if rv is not None:
                    if ev is True:
                        resid_valid.append(rv)
                    elif ev is False:
                        resid_invalid.append(rv)

            if resid_valid and resid_invalid:
                sv = WelfordStats()
                for x in resid_valid:
                    sv.update(float(x))
                si = WelfordStats()
                for x in resid_invalid:
                    si.update(float(x))

                print(f"  When edge valid   (n={sv.n}):  mean={format_ns(sv.mean)}  stddev={format_ns(sv.stddev)}")
                print(f"  When edge INVALID (n={si.n}):  mean={format_ns(si.mean)}  stddev={format_ns(si.stddev)}")
                print()

                if sv.stddev > 0 and si.stddev > sv.stddev * 2:
                    print("  ⚠️  DWT residual variance is significantly higher when edge detection fails")
                    print("     TDC correction is meaningfully reducing ISR jitter when active")
                elif sv.n > 0:
                    print("  ℹ️  DWT residual variance similar regardless of edge detection status")
                print()

    # =================================================================
    # Section 9: GNSS discipline correlation
    # =================================================================

    if gnss_phase_errors:
        print("-" * 70)
        print("GNSS DISCIPLINE CORRELATION")
        print("-" * 70)

        gps_stats = WelfordStats()
        for g in gnss_phase_errors:
            gps_stats.update(float(g))

        print(f"  GF-8802 PPS timing error (phase detector, all samples):")
        print(f"    n      = {gps_stats.n}")
        print(f"    mean   = {format_ns(gps_stats.mean)}")
        print(f"    stddev = {format_ns(gps_stats.stddev)}")
        print(f"    min    = {format_ns(gps_stats.min_val)}")
        print(f"    max    = {format_ns(gps_stats.max_val)}")
        print()

        if gnss_phase_when_valid and gnss_phase_when_invalid:
            sv = WelfordStats()
            for x in gnss_phase_when_valid:
                sv.update(float(x))
            si = WelfordStats()
            for x in gnss_phase_when_invalid:
                si.update(float(x))

            print(f"  When edge valid   (n={sv.n}):  mean={format_ns(sv.mean)}  stddev={format_ns(sv.stddev)}")
            print(f"  When edge INVALID (n={si.n}):  mean={format_ns(si.mean)}  stddev={format_ns(si.stddev)}")
            print()

        # GNSS mode distribution
        mode_counter: Counter = Counter()
        for r in rows:
            gnss = r.get("gnss") or {}
            mode = gnss.get("freq_mode_name")
            if mode:
                mode_counter[mode] += 1

        if mode_counter:
            print("  GNSS frequency mode distribution:")
            for mode, cnt in mode_counter.most_common():
                print(f"    {mode:20s}: {cnt:6d} ({pct(cnt, len(rows))})")
            print()

    # =================================================================
    # Section 10: Recommendations
    # =================================================================

    print("=" * 70)
    print("RECOMMENDATIONS")
    print("=" * 70)
    print()

    if total_classified == 0:
        print("  ❌ No edge detection data available in this campaign")
        print("     Verify TIMEBASE records include diag_teensy_pps_edge_valid")

    elif real_invalid == 0:
        # Only cold-start failures — system is healthy
        if cold_start_count:
            print(f"  ✅ TDC edge correction is working perfectly")
            print(f"     ({cold_start_count} cold-start miss at pps=0 is structural, not a fault)")
        else:
            print("  ✅ TDC edge correction is working perfectly")
        print()
        if approach_ns_values:
            min_approach = min(approach_ns_values)
            mean_approach = sum(approach_ns_values) / len(approach_ns_values)
            print(f"     Spin loop approach time: min={format_ns(min_approach)}, mean={format_ns(mean_approach)}")
            print(f"     This is your actual safety margin before PPS arrives.")
            print()
            if min_approach < 10_000:
                print(f"  ⚠️  Minimum approach time is only {format_ns(min_approach)} — cutting it close")
                print(f"     Consider increasing PRE_PPS_US for more headroom")
            elif min_approach < 50_000:
                print(f"  ℹ️  Minimum approach time {format_ns(min_approach)} — adequate but tight")
            else:
                print(f"  ✅ Minimum approach time {format_ns(min_approach)} — comfortable margin")
        elif delta_ns_values:
            max_delta = max(delta_ns_values)
            suggested_us = int(max_delta / 1000) + 10
            print(f"     Maximum dispatch delta: {format_ns(max_delta)}")
            print(f"     Suggested minimum PRE_PPS_US: {suggested_us} µs")

    else:
        fail_rate = real_invalid / real_classified * 100

        # Classify primary failure mode from per-edge fields (excluding cold starts)
        failures_fine_late = sum(
            1 for t in timeline if t["valid"] is False and not t.get("cold_start") and t["fine_late"] is True
        )
        failures_no_spin = sum(
            1 for t in timeline if t["valid"] is False and not t.get("cold_start") and t["delta_ns"] is None
        )
        failures_tdc_reject = sum(
            1 for t in timeline if t["valid"] is False and not t.get("cold_start") and t["delta_ns"] is not None
        )

        if failures_fine_late > real_invalid * 0.5 or failures_no_spin > real_invalid * 0.5:
            print(f"  🚨 Primary failure mode: SPIN LOOP TOO LATE")
            print(f"     Fine timer late: {failures_fine_late}/{real_invalid}")
            print(f"     No spin data:    {failures_no_spin}/{real_invalid}")
            print()
            print(f"     The fine timer (PRE_PPS) is firing after the PPS edge.")
            print(f"     PRE_PPS_US must be increased.")
            print()

            if approach_ns_valid:
                mean_approach = sum(approach_ns_valid) / len(approach_ns_valid)
                suggested_us = max(100, int(mean_approach / 1000) * 3)
                print(f"     When spin loop IS running, mean approach = {format_ns(mean_approach)}")
                print(f"     Suggested PRE_PPS_US: {suggested_us} µs")
            elif delta_ns_valid:
                max_valid = max(delta_ns_valid)
                suggested_us = max(100, int(max_valid / 1000) * 3)
                print(f"     When spin loop IS running, max delta = {format_ns(max_valid)}")
                print(f"     Suggested PRE_PPS_US: {suggested_us} µs")
            else:
                print(f"     Suggested PRE_PPS_US: 100 µs (conservative default)")

        elif failures_tdc_reject > real_invalid * 0.5:
            print(f"  ⚠️  Primary failure mode: TDC REJECTION (spin loop was running)")
            print(f"     {failures_tdc_reject}/{real_invalid} failures had a dispatch delta but TDC rejected it")
            print(f"     → TDC constants may need recalibration")
            print()
            if delta_ns_invalid:
                print(f"     Rejected deltas: min={format_ns(min(delta_ns_invalid))}, max={format_ns(max(delta_ns_invalid))}")
            print(f"     Run: .zt tdc_analyzer {campaign}")

        else:
            print(f"  ⚠️  Mixed failure modes — review FAILURE PATTERNS section above")

        print()
        print(f"  Current failure rate: {fail_rate:.1f}% (excluding {cold_start_count} cold-start)" if cold_start_count else f"  Current failure rate: {fail_rate:.1f}%")
        print(f"  Impact: {real_invalid} PPS edges used uncorrected ISR snapshot")

    print()
    print("=" * 70)
    print("DONE")
    print("=" * 70)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: pps_edge_analyzer <campaign_name>")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, count(*) as cnt,
                           count(*) FILTER (
                               WHERE (payload->>'diag_teensy_pps_edge_valid') IS NOT NULL
                           ) as has_edge,
                           count(*) FILTER (
                               WHERE (payload->>'diag_teensy_pps_edge_valid')::text = 'true'
                           ) as edge_valid,
                           count(*) FILTER (
                               WHERE (payload->'gnss'->>'pps_timing_error_ns') IS NOT NULL
                           ) as has_gnss,
                           count(*) FILTER (
                               WHERE (payload->>'diag_teensy_approach_ns') IS NOT NULL
                           ) as has_approach
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s} {'EDGE':>8s} {'VALID':>8s} {'GNSS':>8s} {'APPRCH':>8s}")
                print(f"  {'─' * 20} {'─' * 8} {'─' * 8} {'─' * 8} {'─' * 8} {'─' * 8}")
                for r in rows:
                    pct_valid = f"{r['edge_valid'] / r['has_edge'] * 100:.0f}%" if r['has_edge'] > 0 else "—"
                    print(
                        f"  {r['campaign']:<20s} {r['cnt']:>8d} {r['has_edge']:>8d} "
                        f"{pct_valid:>8s} {r['has_gnss']:>8d} {r['has_approach']:>8d}"
                    )
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    analyze(campaign)


if __name__ == "__main__":
    main()