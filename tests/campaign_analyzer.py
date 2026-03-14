"""
ZPNet Campaign Analyzer — TIMEBASE integrity and continuity audit (v9 Dual OCXO)

Usage:
    python -m zpnet.tests.campaign_analyzer <campaign_name>
    .zt campaign_analyzer Baseline2

Reads all TIMEBASE rows for the named campaign and produces:
  - Record count (expected vs actual, missing pps_count indices)
  - pps_count continuity (gaps, repeats, regressions)
  - Per-clock monotonicity checks (ns/cycles must always increase)
  - Per-clock zero/duplicate detection
  - Per-clock delta statistics (min, max, mean, stddev)
  - Recovery-aware: gaps in pps_count are classified as recovery gaps
    and cross-boundary anomalies are expected artifacts, not failures.
  - PPB sanity check: flags any record where a clock domain's
    cumulative PPB (relative to GNSS) exceeds an absolute threshold.
    No real clock in ZPNet drifts more than 10,000 ppb -- values
    beyond that indicate a projection error.
  - Recovery forensics (v5): at each recovery boundary, reconstructs
    the symmetric nanosecond projection that _recover_campaign()
    performed.  Four clock domains (GNSS, DWT, OCXO1, OCXO2) use:

      projected_gnss_ns = next_pps_count x NS_PER_SECOND
      projected_ns = projected_gnss_ns x last_clock_ns / last_gnss_ns

    Pure integer arithmetic, no DWT cycle counts, no Pi epochs.
  - v9: Dual OCXO support.  All singular OCXO references split into
    OCXO1 and OCXO2.  Recovery forensics, PPB sanity, raw delta
    analysis, prediction quality, and servo state all handle both
    OCXOs independently.
  - GNSS phase lock analysis: scans isr_residual_gnss for non-zero
    values, histograms the distribution, and flags any violations.
  - Prediction quality analysis (v8): examines Teensy prediction
    statistics (dwt_pred_stddev, ocxo1_pred_stddev, ocxo2_pred_stddev) from the stats
    block in TIMEBASE records.  These are the authoritative
    interpolation uncertainty metrics.
  - Raw delta analysis (v8): examines dwt_delta_raw, ocxo1_delta_raw, and ocxo2_delta_raw
    per-second deltas for anomalies and drift trends.
  - Summary verdict: CLEAN, CLEAN (with recovery), or anomalies

v8 changes:
  - Pi clock domains removed (PI_NS, PI_CORR, PI_RAW) — deprecated
  - Pi capture sequence analysis removed (diag_pi_seq)
  - Pi ISR residual removed (isr_residual_pi)
  - Pi recovery forensics removed (pi_ns, pi_corrected, tau_pi)
  - Pi PPB domain removed
  - Prediction quality analysis added (DWT, OCXO1, OCXO2 pred_stddev)
  - Raw delta analysis added (dwt_delta_raw, ocxo1_delta_raw, ocxo2_delta_raw)
  - OCXO1/OCXO2 servo state surfaced in campaign summary
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional, Set, Tuple

from zpnet.shared.db import open_db


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

PPB_ABSOLUTE_THRESHOLD = 10_000

NS_PER_SECOND = 1_000_000_000


# ---------------------------------------------------------------------
# Clock domain definitions
# ---------------------------------------------------------------------

CLOCK_DOMAINS = [
    ("GNSS",    "teensy_gnss_ns",    "GNSS nanoseconds"),
    ("DWT_NS",  "teensy_dwt_ns",     "DWT nanoseconds"),
    ("DWT_CYC", "teensy_dwt_cycles", "DWT cycles"),
    ("OCXO1",   "teensy_ocxo1_ns",   "OCXO1 nanoseconds"),
    ("OCXO2",   "teensy_ocxo2_ns",   "OCXO2 nanoseconds"),
]

PPB_DOMAINS = [
    ("DWT",   "teensy_dwt_ns",   "DWT vs GNSS"),
    ("OCXO1", "teensy_ocxo1_ns", "OCXO1 vs GNSS"),
    ("OCXO2", "teensy_ocxo2_ns", "OCXO2 vs GNSS"),
]


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

    def summary(self) -> str:
        if self.n == 0:
            return "no data"
        return (
            f"n={self.n}  "
            f"mean={self.mean:+.3f}  "
            f"sd={self.stddev:.3f}  "
            f"min={self.min_val:.3f}  "
            f"max={self.max_val:.3f}"
        )


# ---------------------------------------------------------------------
# Fetch campaign TIMEBASE rows
# ---------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
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
        p["_db_id"] = row["id"]
        p["_db_ts"] = str(row["ts"])
        result.append(p)

    return result


# ---------------------------------------------------------------------
# Recovery boundary detection
# ---------------------------------------------------------------------

def find_recovery_boundaries(rows: List[Dict[str, Any]]) -> Set[int]:
    boundaries: Set[int] = set()
    for i in range(1, len(rows)):
        prev = int(rows[i - 1]["pps_count"])
        curr = int(rows[i]["pps_count"])
        if curr > prev + 1:
            boundaries.add(curr)
    return boundaries


# ---------------------------------------------------------------------
# PPB sanity check
# ---------------------------------------------------------------------

def _compute_ppb(clock_ns: int, gnss_ns: int) -> Optional[float]:
    if gnss_ns <= 0:
        return None
    return ((clock_ns - gnss_ns) / gnss_ns) * 1e9


def analyze_ppb_sanity(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> Tuple[List[str], List[str]]:
    anomalies: List[str] = []
    lines: List[str] = []

    for label, ns_key, desc in PPB_DOMAINS:
        violations: List[Dict[str, Any]] = []
        ppb_stats = WelfordStats()
        valid_count = 0

        for row in rows:
            gnss_ns_raw  = row.get("teensy_gnss_ns")
            clock_ns_raw = row.get(ns_key)
            if gnss_ns_raw is None or clock_ns_raw is None:
                continue
            gnss_ns  = int(gnss_ns_raw)
            clock_ns = int(clock_ns_raw)
            if gnss_ns <= 0:
                continue
            ppb = _compute_ppb(clock_ns, gnss_ns)
            if ppb is None:
                continue
            valid_count += 1
            ppb_stats.update(ppb)
            if abs(ppb) > PPB_ABSOLUTE_THRESHOLD:
                pps = int(row.get("pps_count", -1))
                violations.append({
                    "pps_count":   pps,
                    "ppb":         ppb,
                    "clock_ns":    clock_ns,
                    "gnss_ns":     gnss_ns,
                    "at_recovery": pps in recovery_boundaries,
                })

        lines.append(f"\n  [{label}] {desc}")
        if ppb_stats.n > 0:
            lines.append(f"    PPB stats: {ppb_stats.summary()}")
        else:
            lines.append(f"    PPB stats: no valid data")

        if not violations:
            lines.append(f"    PPB sanity: OK all {valid_count} records "
                         f"within +-{PPB_ABSOLUTE_THRESHOLD:,} ppb")
        else:
            at_recovery  = [v for v in violations if v["at_recovery"]]
            steady_state = [v for v in violations if not v["at_recovery"]]
            first_v = violations[0]
            worst_v = max(violations, key=lambda v: abs(v["ppb"]))
            lines.append(f"    WARN {len(violations)} record(s) exceed +-{PPB_ABSOLUTE_THRESHOLD:,} ppb threshold")
            lines.append(f"    Affected range: pps_count {violations[0]['pps_count']} -> {violations[-1]['pps_count']}")
            if at_recovery:
                lines.append(f"    At recovery boundaries: {len(at_recovery)}")
            if steady_state:
                lines.append(f"    In steady-state: {len(steady_state)}")
            lines.append(f"    First violation: pps_count={first_v['pps_count']}  ppb={first_v['ppb']:+,.1f}")
            lines.append(f"    Worst violation: pps_count={worst_v['pps_count']}  ppb={worst_v['ppb']:+,.1f}")
            samples = violations[:5]
            if len(violations) > 10:
                samples += violations[-3:]
            lines.append(f"    Sample violations:")
            shown = set()
            for v in samples:
                if v["pps_count"] in shown:
                    continue
                shown.add(v["pps_count"])
                rtag = " (recovery)" if v["at_recovery"] else ""
                lines.append(
                    f"      pps={v['pps_count']:>7d}  ppb={v['ppb']:>+14,.1f}"
                    f"  dns={v['clock_ns'] - v['gnss_ns']:>+14,d}{rtag}"
                )
            if len(violations) > len(shown):
                lines.append(f"      ... and {len(violations) - len(shown)} more")
            anomalies.append(
                f"{label}: {len(violations)} records with |ppb| > {PPB_ABSOLUTE_THRESHOLD:,} "
                f"(worst: {worst_v['ppb']:+,.1f} ppb at pps_count={worst_v['pps_count']})"
            )

    return anomalies, lines


# ---------------------------------------------------------------------
# Recovery forensics — v6 nanosecond architecture (4 domains)
# ---------------------------------------------------------------------

def analyze_recovery_forensics(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> List[str]:
    lines: List[str] = []
    if not recovery_boundaries:
        return lines

    pps_to_idx: Dict[int, int] = {}
    for i, r in enumerate(rows):
        pps_to_idx[int(r["pps_count"])] = i

    for boundary_pps in sorted(recovery_boundaries):
        boundary_idx = pps_to_idx.get(boundary_pps)
        if boundary_idx is None or boundary_idx == 0:
            lines.append(f"\n  Recovery at pps_count={boundary_pps}: WARN no pre-gap row found")
            continue

        post_row = rows[boundary_idx]
        pre_row  = rows[boundary_idx - 1]

        pre_pps  = int(pre_row["pps_count"])
        post_pps = int(post_row["pps_count"])
        gap_seconds     = post_pps - pre_pps - 1
        elapsed_seconds = post_pps - pre_pps
        next_pps_count  = post_pps

        lines.append(f"\n  {'=' * 62}")
        lines.append(f"  RECOVERY FORENSICS: pps_count {pre_pps} -> {post_pps}")
        lines.append(f"  {'=' * 62}")
        lines.append(f"  Gap: {gap_seconds} missing seconds")
        lines.append(f"  Elapsed (pps delta): {elapsed_seconds} seconds")
        lines.append(f"  Inferred next_pps_count: {next_pps_count}")

        anchor_gnss_ns = int(pre_row.get("teensy_gnss_ns") or 0)
        anchor_dwt_ns  = int(pre_row.get("teensy_dwt_ns")  or 0)
        anchor_ocxo1_ns = int(pre_row.get("teensy_ocxo1_ns") or 0)
        anchor_ocxo2_ns = int(pre_row.get("teensy_ocxo2_ns") or 0)
        anchor_time = pre_row.get("gnss_time_utc") or pre_row.get("system_time_utc", "?")

        lines.append(f"\n  Anchor (pps_count={pre_pps}):")
        lines.append(f"    gnss_time_utc: {anchor_time}")
        lines.append(f"    gnss_ns:       {anchor_gnss_ns:,}")
        lines.append(f"    dwt_ns:        {anchor_dwt_ns:,}")
        lines.append(f"    ocxo1_ns:      {anchor_ocxo1_ns:,}")
        lines.append(f"    ocxo2_ns:      {anchor_ocxo2_ns:,}")

        tau_dwt  = anchor_dwt_ns  / anchor_gnss_ns if anchor_gnss_ns > 0 else 1.0
        tau_ocxo1 = anchor_ocxo1_ns / anchor_gnss_ns if (anchor_gnss_ns > 0 and anchor_ocxo1_ns > 0) else 1.0
        tau_ocxo2 = anchor_ocxo2_ns / anchor_gnss_ns if (anchor_gnss_ns > 0 and anchor_ocxo2_ns > 0) else 1.0

        lines.append(f"\n  Tau (from anchor ratios):")
        lines.append(f"    tau_dwt:  {tau_dwt:.12f}")
        lines.append(f"    tau_ocxo1: {tau_ocxo1:.12f}")
        lines.append(f"    tau_ocxo2: {tau_ocxo2:.12f}")

        proj_gnss_ns = next_pps_count * NS_PER_SECOND
        proj_dwt_ns  = proj_gnss_ns * anchor_dwt_ns  // anchor_gnss_ns if anchor_gnss_ns > 0 else proj_gnss_ns
        proj_ocxo1_ns = proj_gnss_ns * anchor_ocxo1_ns // anchor_gnss_ns if (anchor_gnss_ns > 0 and anchor_ocxo1_ns > 0) else 0
        proj_ocxo2_ns = proj_gnss_ns * anchor_ocxo2_ns // anchor_gnss_ns if (anchor_gnss_ns > 0 and anchor_ocxo2_ns > 0) else 0

        lines.append(f"\n  v5 Symmetric projection:")
        lines.append(f"    projected_gnss_ns = {next_pps_count} x {NS_PER_SECOND:,} = {proj_gnss_ns:,}")
        lines.append(f"    GNSS:  {proj_gnss_ns:,}  (tau = 1.000000000000)")
        lines.append(f"    DWT:   {proj_dwt_ns:,}  (tau = {tau_dwt:.12f})")
        lines.append(f"    OCXO1: {proj_ocxo1_ns:,}  (tau = {tau_ocxo1:.12f})")
        lines.append(f"    OCXO2: {proj_ocxo2_ns:,}  (tau = {tau_ocxo2:.12f})")

        actual_gnss_ns = int(post_row.get("teensy_gnss_ns") or 0)
        actual_dwt_ns  = int(post_row.get("teensy_dwt_ns")  or 0)
        actual_ocxo1_ns = int(post_row.get("teensy_ocxo1_ns") or 0)
        actual_ocxo2_ns = int(post_row.get("teensy_ocxo2_ns") or 0)
        actual_time = post_row.get("gnss_time_utc") or post_row.get("system_time_utc", "?")

        lines.append(f"\n  Actual (pps_count={post_pps}):")
        lines.append(f"    gnss_time_utc: {actual_time}")
        lines.append(f"    gnss_ns:       {actual_gnss_ns:,}")
        lines.append(f"    dwt_ns:        {actual_dwt_ns:,}")
        lines.append(f"    ocxo1_ns:      {actual_ocxo1_ns:,}")
        lines.append(f"    ocxo2_ns:      {actual_ocxo2_ns:,}")

        d_gnss = actual_gnss_ns - proj_gnss_ns
        d_dwt  = actual_dwt_ns  - proj_dwt_ns
        d_ocxo1 = actual_ocxo1_ns - proj_ocxo1_ns
        d_ocxo2 = actual_ocxo2_ns - proj_ocxo2_ns

        lines.append(f"\n  Projection errors (actual - projected):")
        lines.append(f"    GNSS:  {d_gnss:+,d} ns")
        lines.append(f"    DWT:   {d_dwt:+,d} ns")
        lines.append(f"    OCXO1: {d_ocxo1:+,d} ns")
        lines.append(f"    OCXO2: {d_ocxo2:+,d} ns")

        if actual_gnss_ns > 0:
            lines.append(f"\n  Projection error as PPB of campaign GNSS time:")
            for lbl, err in [("GNSS", d_gnss), ("DWT", d_dwt), ("OCXO1", d_ocxo1), ("OCXO2", d_ocxo2)]:
                lines.append(f"    {lbl:6s} {(err / actual_gnss_ns) * 1e9:+,.3f} ppb")

        if d_gnss != 0:
            lines.append(f"\n  GNSS ns discrepancy: {d_gnss:+,d} ns ({d_gnss / NS_PER_SECOND:+.6f} seconds)")
            if abs(d_gnss) <= 400:
                lines.append(f"    (within GNSS receiver quantization -- normal)")
            else:
                lines.append(f"    WARN larger than expected GNSS quantization")

        lines.append(f"\n  Post-recovery PPB (all domains vs GNSS):")
        if actual_gnss_ns > 0:
            for lbl, actual_ns in [("DWT", actual_dwt_ns), ("OCXO1", actual_ocxo1_ns), ("OCXO2", actual_ocxo2_ns)]:
                if actual_ns > 0:
                    ppb  = _compute_ppb(actual_ns, actual_gnss_ns)
                    flag = f"  WARN EXCEEDS +-{PPB_ABSOLUTE_THRESHOLD:,} ppb" if ppb is not None and abs(ppb) > PPB_ABSOLUTE_THRESHOLD else ""
                    lines.append(f"    {lbl:6s} {ppb:+,.1f} ppb{flag}")

    return lines


# ---------------------------------------------------------------------
# GNSS Phase Lock Analysis
# ---------------------------------------------------------------------

def analyze_gnss_phase_lock(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> Tuple[List[str], List[str]]:
    """
    Analyze isr_residual_gnss to reason about VCLOCK/PPS phase lock.

    The ISR-level residual is:
        isr_residual_gnss = (snap_gnss_now - snap_gnss_prev) - 10_000_000

    A perfectly phase-locked system produces exactly zero every cycle.

    Non-zero values:
      +1/-1 (oscillating): phase boundary artifact -- PPS edge lands
        on the cusp between two 10 MHz cycles.
      Other values: real hardware issue.
    """
    anomalies: List[str] = []
    lines: List[str] = []

    samples: List[Tuple[int, int]] = []
    nulls = 0
    invalid_count = 0

    for row in rows:
        pps   = int(row.get("pps_count", -1))
        v     = row.get("isr_residual_gnss")
        valid = row.get("isr_residual_valid")

        if v is None:
            nulls += 1
            continue
        if valid is False or valid == "false" or valid == 0:
            invalid_count += 1
            continue
        if pps in recovery_boundaries:
            invalid_count += 1
            continue

        samples.append((pps, int(v)))

    total_samples = len(samples)
    non_zero  = [(pps, r) for pps, r in samples if r != 0]
    zero_count = total_samples - len(non_zero)

    lines.append(f"\n  isr_residual_gnss -- ISR-level 10 MHz cycle count per PPS")
    lines.append(f"  (expected: exactly 10,000,000 ticks/second)")
    lines.append(f"  (expected residual: 0 every cycle for a phase-locked system)")
    lines.append(f"")
    lines.append(f"  Total valid samples:         {total_samples}")
    lines.append(f"  Excluded (invalid/boundary): {invalid_count + nulls}")

    if total_samples == 0:
        lines.append(f"\n  WARN No valid isr_residual_gnss data found.")
        return anomalies, lines

    lines.append(f"  Zero residuals:              {zero_count}  ({100*zero_count/total_samples:.1f}%)")
    lines.append(f"  Non-zero residuals:          {len(non_zero)}  ({100*len(non_zero)/total_samples:.1f}%)")

    if not non_zero:
        lines.append(f"\n  PERFECT PHASE LOCK: all {total_samples} samples show exactly")
        lines.append(f"  zero residual.  VCLOCK and PPS are phase-coherent.")
        return anomalies, lines

    residual_values = [r for _, r in non_zero]
    value_counts    = Counter(residual_values)
    min_r = min(residual_values)
    max_r = max(residual_values)

    lines.append(f"\n  Non-zero residual distribution:")
    for val in sorted(value_counts.keys()):
        count = value_counts[val]
        bar   = "#" * min(40, count)
        lines.append(f"    {val:>+6d}  {count:>6d}x  {bar}")

    total_sum = sum(residual_values)
    lines.append(f"\n  Range: [{min_r:+d}, {max_r:+d}]")
    lines.append(f"  Sum of all residuals: {total_sum:+d}")

    plus_one  = value_counts.get(+1, 0)
    minus_one = value_counts.get(-1, 0)
    other     = len(non_zero) - plus_one - minus_one

    if plus_one > 0 or minus_one > 0:
        lines.append(f"\n  +/-1 residual breakdown:")
        lines.append(f"    +1 count:  {plus_one}")
        lines.append(f"    -1 count:  {minus_one}")
        lines.append(f"    other:     {other}")
        lines.append(f"    imbalance: {abs(plus_one - minus_one)}")

    lines.append(f"\n  Phase lock assessment:")

    if other == 0 and abs(total_sum) <= 2:
        lines.append(f"  PHASE BOUNDARY ARTIFACT detected.")
        lines.append(f"  All non-zero residuals are +/-1 cycle (100 ns at 10 MHz).")
        lines.append(f"  Net sum = {total_sum:+d} -- residuals cancel across cycles.")
        lines.append(f"  This is NOT a missed pulse.  VCLOCK and PPS are phase-locked,")
        lines.append(f"  but the PPS edge lands on the cusp between two 10 MHz cycles.")
        anomalies.append(
            f"GNSS phase boundary artifact: {len(non_zero)} +/-1 residuals "
            f"(+1={plus_one}, -1={minus_one}, sum={total_sum:+d}) -- "
            f"PPS edge on 10 MHz cycle boundary, not a hardware fault"
        )
    elif other == 0 and abs(total_sum) > 2:
        lines.append(f"  SYSTEMATIC PHASE OFFSET: +/-1 residuals are imbalanced.")
        lines.append(f"  Net sum = {total_sum:+d} suggests a consistent phase offset")
        lines.append(f"  between PPS and VCLOCK edges.")
        anomalies.append(
            f"GNSS systematic phase offset: {len(non_zero)} +/-1 residuals, "
            f"net sum={total_sum:+d} (imbalanced)"
        )
    else:
        large = [(pps, r) for pps, r in non_zero if abs(r) > 1]
        lines.append(f"  ANOMALOUS RESIDUALS: {len(large)} values with |residual| > 1.")
        lines.append(f"  These cannot be explained by phase boundary artifacts.")
        lines.append(f"  Possible causes: missed 10 MHz edges, counter glitch,")
        lines.append(f"  signal integrity issue, or ISR priority conflict.")
        lines.append(f"")
        lines.append(f"  First occurrences:")
        for pps, r in large[:10]:
            lines.append(f"    pps_count={pps}  residual={r:+d}")
        if len(large) > 10:
            lines.append(f"    ... and {len(large) - 10} more")
        anomalies.append(
            f"GNSS anomalous residuals: {len(large)} values with |residual| > 1 "
            f"(range [{min_r:+d}, {max_r:+d}])"
        )

    if len(non_zero) >= 4 and other == 0:
        consecutive_pairs = 0
        for i in range(len(samples) - 1):
            r_curr = samples[i][1]
            r_next = samples[i + 1][1]
            if r_curr != 0 and r_next != 0 and r_curr != r_next:
                consecutive_pairs += 1
        if consecutive_pairs > 0:
            lines.append(f"\n  Consecutive +/-1 pairs (alternating): {consecutive_pairs}")
            lines.append(f"  (Alternating pattern confirms phase boundary artifact.)")

    return anomalies, lines


# ---------------------------------------------------------------------
# Prediction Quality Analysis (v8)
# ---------------------------------------------------------------------

def analyze_prediction_quality(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> Tuple[List[str], List[str]]:
    """
    Examine Teensy prediction statistics from TIMEBASE stats block.

    The prediction residual stddev is the authoritative interpolation
    uncertainty metric.  For DWT this is typically ~3-4 cycles (3-4 ns);
    for OCXO1/OCXO2 ~1 tick (100 ns) when servo-locked.

    This section surfaces:
      - Final prediction stddev and N for each domain
      - Trend: is stddev stable or growing?
      - Any records where prediction N is unexpectedly low
        (indicates repeated resets / short campaigns)
    """
    anomalies: List[str] = []
    lines: List[str] = []

    for label, pred_key_prefix in [("DWT", "dwt"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        lines.append(f"\n  [{label}] Prediction statistics (Teensy v10)")

        stddev_values: List[Tuple[int, float]] = []
        n_values: List[Tuple[int, int]] = []
        residual_stats = WelfordStats()
        nulls = 0

        for row in rows:
            pps = int(row.get("pps_count", -1))
            if pps in recovery_boundaries:
                continue

            stats = row.get("stats", {})
            domain_stats = stats.get(pred_key_prefix, {})

            pred_n = domain_stats.get("pred_n")
            pred_stddev = domain_stats.get("pred_stddev")
            pred_residual = domain_stats.get("pred_residual")

            if pred_n is None:
                nulls += 1
                continue

            n_values.append((pps, int(pred_n)))

            if pred_stddev is not None:
                stddev_values.append((pps, float(pred_stddev)))

            if pred_residual is not None and int(pred_n) > 0:
                residual_stats.update(float(pred_residual))

        if not n_values:
            lines.append(f"    No prediction data ({nulls} nulls)")
            continue

        # Final values (last record)
        final_pps, final_n = n_values[-1]
        final_stddev = stddev_values[-1][1] if stddev_values else None

        lines.append(f"    Records with data: {len(n_values)}")
        lines.append(f"    Final pred_n:      {final_n}")
        if final_stddev is not None:
            unit = "cycles" if label == "DWT" else "ticks"
            ns_equiv = final_stddev * (1.0 if label == "DWT" else 100.0)
            lines.append(f"    Final pred_stddev: {final_stddev:.3f} {unit} ({ns_equiv:.1f} ns)")

        if residual_stats.n > 0:
            lines.append(f"    Prediction residuals (Pi-side observed): {residual_stats.summary()}")

        # Check for stability: compare first-quarter vs last-quarter stddev
        if len(stddev_values) >= 20:
            q1_end = len(stddev_values) // 4
            q4_start = 3 * len(stddev_values) // 4
            q1_mean = sum(s for _, s in stddev_values[:q1_end]) / q1_end
            q4_mean = sum(s for _, s in stddev_values[q4_start:]) / (len(stddev_values) - q4_start)

            if q4_mean > q1_mean * 1.5 and q4_mean > 1.0:
                lines.append(f"    WARN stddev growing: Q1 mean={q1_mean:.3f} -> Q4 mean={q4_mean:.3f}")
                anomalies.append(f"{label} prediction stddev growing (Q1={q1_mean:.3f} Q4={q4_mean:.3f})")
            else:
                lines.append(f"    Stability: OK (Q1={q1_mean:.3f} Q4={q4_mean:.3f})")

        # Check for zero-N records in the middle of a campaign (not just startup).
        # Records immediately after a recovery boundary are expected to have
        # pred_n=0 (prediction trackers reset on recovery).  Only flag records
        # that are NOT adjacent to a recovery boundary.
        zero_n_mid = [(pps, n) for pps, n in n_values if n == 0 and pps > n_values[0][0] + 5]
        if zero_n_mid:
            # A pred_n=0 record is "expected" if it is within 3 seconds of a recovery boundary
            at_recovery = [pps for pps, _ in zero_n_mid
                           if any(abs(pps - rb) <= 3 for rb in recovery_boundaries)]
            unexpected  = [pps for pps, _ in zero_n_mid
                           if not any(abs(pps - rb) <= 3 for rb in recovery_boundaries)]

            if at_recovery and not unexpected:
                lines.append(f"    pred_n=0: {len(at_recovery)} records "
                             f"(all at recovery boundaries — expected)")
            elif unexpected:
                lines.append(f"    WARN {len(unexpected)} records with pred_n=0 after startup "
                             f"(not at recovery boundaries — possible mid-campaign reset)")
                if at_recovery:
                    lines.append(f"    pred_n=0 at recovery: {len(at_recovery)} (expected)")

        lines.append(f"    Verdict: {'OK' if not any(label in a for a in anomalies) else 'WARN'}")

    return anomalies, lines


# ---------------------------------------------------------------------
# Raw Delta Analysis (v8)
# ---------------------------------------------------------------------

def analyze_raw_deltas(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> Tuple[List[str], List[str]]:
    """
    Examine dwt_delta_raw, ocxo1_delta_raw, and ocxo2_delta_raw per-second deltas.

    These are the ground truth for what each oscillator actually did
    in each second.  The analysis looks for:
      - Basic statistics (mean, stddev, min, max)
      - Outliers (deltas far from nominal)
      - Drift trend (is the mean shifting over time?)
    """
    anomalies: List[str] = []
    lines: List[str] = []

    DWT_NOMINAL  = 1_008_000_000
    OCXO_NOMINAL = 10_000_000

    # v14: Both OCXOs on GPT single-edge counting.  All raw deltas are 10 MHz.
    OCXO2_RAW_NOMINAL = 10_000_000

    for label, key, nominal in [
        ("DWT",   "dwt_delta_raw",   DWT_NOMINAL),
        ("OCXO1", "ocxo1_delta_raw", OCXO_NOMINAL),
        ("OCXO2", "ocxo2_delta_raw", OCXO2_RAW_NOMINAL),
    ]:
        lines.append(f"\n  [{label}] Raw per-second deltas (key: {key})")

        delta_stats = WelfordStats()
        residual_stats = WelfordStats()
        nulls = 0
        values: List[Tuple[int, int]] = []

        for row in rows:
            pps = int(row.get("pps_count", -1))
            if pps in recovery_boundaries:
                continue

            v = row.get(key)
            if v is None:
                nulls += 1
                continue

            delta = int(v)
            values.append((pps, delta))
            delta_stats.update(float(delta))
            residual_stats.update(float(delta - nominal))

        if not values:
            lines.append(f"    No data ({nulls} nulls)")
            continue

        lines.append(f"    Samples: {delta_stats.n}")
        lines.append(f"    Delta stats:    {delta_stats.summary()}")
        lines.append(f"    Residual stats: {residual_stats.summary()}")
        lines.append(f"    Nominal: {nominal:,}")

        # Outlier check: deltas more than 50% away from nominal
        outlier_threshold = nominal // 2
        outliers = [(pps, d) for pps, d in values if abs(d - nominal) > outlier_threshold]
        if outliers:
            lines.append(f"    WARN {len(outliers)} outlier deltas (>50% from nominal):")
            for pps, d in outliers[:5]:
                lines.append(f"      pps_count={pps}  delta={d:,}  residual={d - nominal:+,d}")
            if len(outliers) > 5:
                lines.append(f"      ... and {len(outliers) - 5} more")
            anomalies.append(f"{label}: {len(outliers)} outlier raw deltas")

        # Drift trend: compare first vs last quarter mean
        if len(values) >= 20:
            q1_end = len(values) // 4
            q4_start = 3 * len(values) // 4
            q1_deltas = [d for _, d in values[:q1_end]]
            q4_deltas = [d for _, d in values[q4_start:]]
            q1_mean = sum(q1_deltas) / len(q1_deltas)
            q4_mean = sum(q4_deltas) / len(q4_deltas)
            drift = q4_mean - q1_mean

            lines.append(f"    Drift (Q4-Q1 mean): {drift:+.3f} counts "
                         f"({drift * (1.0 if label == 'DWT' else 100.0):+.1f} ns)")

        lines.append(f"    Verdict: {'OK' if not any(label in a for a in anomalies) else 'WARN'}")

    return anomalies, lines


# ---------------------------------------------------------------------
# OCXO Servo Summary
# ---------------------------------------------------------------------

def analyze_ocxo_servo(rows: List[Dict[str, Any]]) -> List[str]:
    """
    Surface OCXO1 and OCXO2 servo state from TIMEBASE records.
    """
    lines: List[str] = []

    calibrating = False
    for row in rows:
        if row.get("calibrate_ocxo"):
            calibrating = True
            break

    lines.append(f"  Calibration active: {'YES' if calibrating else 'NO'}")

    for label, dac_key, adj_key in [
        ("OCXO1", "ocxo1_dac", "ocxo1_servo_adjustments"),
        ("OCXO2", "ocxo2_dac", "ocxo2_servo_adjustments"),
    ]:
        first_dac = None
        last_dac = None
        first_adj = None
        last_adj = None

        for row in rows:
            dac = row.get(dac_key)
            adj = row.get(adj_key)

            if dac is not None:
                if first_dac is None:
                    first_dac = float(dac)
                last_dac = float(dac)
            if adj is not None:
                if first_adj is None:
                    first_adj = int(adj)
                last_adj = int(adj)

        lines.append(f"\n  [{label}]")
        if first_dac is None:
            lines.append(f"    No DAC data.")
            continue

        lines.append(f"    DAC range: {first_dac:.6f} -> {last_dac:.6f}")
        lines.append(f"    DAC drift: {last_dac - first_dac:+.6f}")

        if first_adj is not None and last_adj is not None:
            lines.append(f"    Servo adjustments: {first_adj} -> {last_adj} ({last_adj - first_adj} during campaign)")

    return lines


# ---------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    total      = len(rows)
    anomalies: List[str] = []

    recovery_boundaries = find_recovery_boundaries(rows)
    recovery_count      = len(recovery_boundaries)

    pps_counts     = [int(r["pps_count"]) for r in rows]
    pps_min        = min(pps_counts)
    pps_max        = max(pps_counts)
    expected_count = pps_max - pps_min + 1
    campaign_seconds = pps_max - pps_min

    first_ts = rows[0].get("system_time_utc")  or rows[0].get("gnss_time_utc", "?")
    last_ts  = rows[-1].get("system_time_utc") or rows[-1].get("gnss_time_utc", "?")

    print("=" * 70)
    print(f"CAMPAIGN ANALYSIS: {campaign}")
    print("=" * 70)
    print()
    print(f"  Time range:        {first_ts}")
    print(f"                  -> {last_ts}")
    print(f"  pps_count range:   {pps_min} -> {pps_max}")
    print(f"  Campaign seconds:  {campaign_seconds}")
    hrs  = campaign_seconds // 3600
    mins = (campaign_seconds % 3600) // 60
    secs = campaign_seconds % 60
    print(f"  Campaign duration: {hrs:02d}:{mins:02d}:{secs:02d}")
    print()
    print(f"  Expected records:  {expected_count}")
    print(f"  Actual records:    {total}")
    print(f"  Missing records:   {expected_count - total}")
    if recovery_count > 0:
        print(f"  Recovery events:   {recovery_count}")
    if total > expected_count:
        anomalies.append(f"{total - expected_count} EXTRA records (duplicates?)")

    pps_set     = set(pps_counts)
    missing_pps = sorted(set(range(pps_min, pps_max + 1)) - pps_set)

    recovery_missing   = 0
    unexpected_missing = 0

    if missing_pps:
        print()
        ranges = []
        start = end = missing_pps[0]
        for v in missing_pps[1:]:
            if v == end + 1:
                end = v
            else:
                ranges.append((start, end))
                start = end = v
        ranges.append((start, end))

        print(f"  Missing pps_count values ({len(missing_pps)}):")
        for s, e in ranges:
            count       = e - s + 1
            is_recovery = (e + 1) in recovery_boundaries
            label       = " (recovery)" if is_recovery else " WARN"
            if is_recovery:
                recovery_missing += count
            else:
                unexpected_missing += count
            if s == e:
                print(f"    {s}{label}")
            else:
                print(f"    {s}-{e} ({count} consecutive){label}")

    if unexpected_missing > 0:
        anomalies.append(f"{unexpected_missing} unexpected missing TIMEBASE records")
    if recovery_missing > 0:
        print(f"\n  Recovery gap records: {recovery_missing} (expected)")

    if len(pps_counts) != len(pps_set):
        dup_count = len(pps_counts) - len(pps_set)
        anomalies.append(f"{dup_count} duplicate pps_count values")
        print(f"\n  WARN {dup_count} duplicate pps_count values detected!")
        ctr  = Counter(pps_counts)
        dups = [(k, v) for k, v in ctr.items() if v > 1]
        for pps, count in sorted(dups)[:20]:
            print(f"    pps_count={pps} appears {count} times")
        if len(dups) > 20:
            print(f"    ... and {len(dups) - 20} more")

    # --- pps_count continuity ---
    print()
    print("-" * 70)
    print("PPS_COUNT CONTINUITY")
    print("-" * 70)

    pps_jumps = pps_recovery_jumps = pps_regressions = pps_repeats = 0

    for i in range(1, len(rows)):
        prev  = int(rows[i - 1]["pps_count"])
        curr  = int(rows[i]["pps_count"])
        delta = curr - prev

        if delta == 0:
            pps_repeats += 1
            if pps_repeats <= 5:
                print(f"  WARN REPEAT at row {i}: pps_count={curr}")
        elif delta < 0:
            pps_regressions += 1
            if pps_regressions <= 5:
                print(f"  WARN REGRESSION at row {i}: {prev} -> {curr} (delta={delta})")
        elif delta > 1:
            if curr in recovery_boundaries:
                pps_recovery_jumps += 1
                print(f"  INFO RECOVERY GAP at row {i}: {prev} -> {curr} (skipped {delta - 1})")
            else:
                pps_jumps += 1
                if pps_jumps <= 10:
                    print(f"  WARN GAP at row {i}: {prev} -> {curr} (skipped {delta - 1})")

    if pps_jumps > 10:
        print(f"  ... and {pps_jumps - 10} more gaps")
    if pps_repeats > 5:
        print(f"  ... and {pps_repeats - 5} more repeats")

    print(f"\n  Jumps (unexpected):  {pps_jumps}")
    print(f"  Jumps (recovery):    {pps_recovery_jumps}")
    print(f"  Repeats:             {pps_repeats}")
    print(f"  Regressions:         {pps_regressions}")

    if pps_jumps:       anomalies.append(f"{pps_jumps} unexpected pps_count gaps")
    if pps_repeats:     anomalies.append(f"{pps_repeats} pps_count repeats")
    if pps_regressions: anomalies.append(f"{pps_regressions} pps_count regressions")

    # --- Per-clock monotonicity ---
    print()
    print("-" * 70)
    print("CLOCK DOMAIN ANALYSIS")
    print("-" * 70)

    for label, key, desc in CLOCK_DOMAINS:
        print(f"\n  [{label}] {desc}  (key: {key})")

        values     = []
        null_count = 0
        null_pps_list: List[int] = []

        for r in rows:
            v = r.get(key)
            if v is None:
                null_count += 1
                null_pps_list.append(int(r["pps_count"]))
            else:
                values.append((int(r["pps_count"]), int(v)))

        if not values:
            print(f"    WARN ALL NULL ({null_count}/{total} records)")
            continue

        if null_count > 0:
            null_at_boundary = sum(1 for p in null_pps_list if p in recovery_boundaries or p == 0)
            if null_at_boundary == null_count:
                print(f"    Nulls: {null_count}/{total} (all at recovery boundaries -- expected)")
            else:
                print(f"    Nulls: {null_count}/{total} ({null_at_boundary} at boundaries, {null_count - null_at_boundary} unexpected)")

        non_monotonic = zero_deltas = negative_deltas = 0
        delta_stats   = WelfordStats()

        for i in range(1, len(values)):
            prev_pps, prev_val = values[i - 1]
            curr_pps, curr_val = values[i]
            at_boundary = curr_pps in recovery_boundaries

            if curr_pps == prev_pps + 1:
                delta = curr_val - prev_val
                delta_stats.update(float(delta))
                if delta == 0:
                    zero_deltas += 1
                    if zero_deltas <= 3:
                        print(f"    WARN ZERO DELTA at pps_count={curr_pps}: value={curr_val}")
                elif delta < 0:
                    negative_deltas += 1
                    if negative_deltas <= 3:
                        print(f"    WARN NEGATIVE DELTA at pps_count={curr_pps}: {prev_val} -> {curr_val} (delta={delta})")
            elif curr_val <= prev_val and not at_boundary:
                non_monotonic += 1
                if non_monotonic <= 3:
                    print(f"    WARN NON-MONOTONIC across gap: pps {prev_pps}->{curr_pps}  val {prev_val}->{curr_val}")

        if zero_deltas > 3:     print(f"    ... and {zero_deltas - 3} more zero deltas")
        if negative_deltas > 3: print(f"    ... and {negative_deltas - 3} more negative deltas")

        print(f"    Range: {values[0][1]} -> {values[-1][1]}")
        print(f"    Total delta: {values[-1][1] - values[0][1]:,}")
        if delta_stats.n > 0:
            print(f"    Per-second deltas: {delta_stats.summary()}")

        issues = []
        if zero_deltas:     issues.append(f"{zero_deltas} zero")
        if negative_deltas: issues.append(f"{negative_deltas} negative")
        if non_monotonic:   issues.append(f"{non_monotonic} non-monotonic")

        if issues:
            print(f"    Verdict: WARN " + ", ".join(issues))
            anomalies.append(f"{label}: {', '.join(issues)} deltas")
        else:
            print(f"    Verdict: OK monotonic, no zeros, no duplicates")

    # --- PPB sanity check ---
    print()
    print("-" * 70)
    print(f"PPB SANITY CHECK (threshold: +-{PPB_ABSOLUTE_THRESHOLD:,} ppb)")
    print("-" * 70)

    ppb_anomalies, ppb_lines = analyze_ppb_sanity(rows, recovery_boundaries)
    for line in ppb_lines:
        print(line)
    anomalies.extend(ppb_anomalies)

    # --- Raw delta analysis (v8) ---
    print()
    print("-" * 70)
    print("RAW DELTA ANALYSIS (per-second oscillator deltas)")
    print("-" * 70)

    delta_anomalies, delta_lines = analyze_raw_deltas(rows, recovery_boundaries)
    for line in delta_lines:
        print(line)
    anomalies.extend(delta_anomalies)

    # --- Prediction quality analysis (v8) ---
    print()
    print("-" * 70)
    print("PREDICTION QUALITY (Teensy v10 trend-aware statistics)")
    print("-" * 70)

    pred_anomalies, pred_lines = analyze_prediction_quality(rows, recovery_boundaries)
    for line in pred_lines:
        print(line)
    anomalies.extend(pred_anomalies)

    # --- OCXO servo summary ---
    print()
    print("-" * 70)
    print("OCXO SERVO STATE (OCXO1 + OCXO2)")
    print("-" * 70)

    servo_lines = analyze_ocxo_servo(rows)
    for line in servo_lines:
        print(line)

    # --- Recovery forensics ---
    if recovery_boundaries:
        print()
        print("-" * 70)
        print("RECOVERY FORENSICS (v6 nanosecond architecture, 4 domains)")
        print("-" * 70)
        for line in analyze_recovery_forensics(rows, recovery_boundaries):
            print(line)

    # --- ISR residual summary ---
    print()
    print("-" * 70)
    print("ISR RESIDUALS (per-second, raw counts)")
    print("-" * 70)

    for label, key in [
        ("GNSS",  "isr_residual_gnss"),
        ("DWT",   "isr_residual_dwt"),
        ("OCXO1", "isr_residual_ocxo1"),
        ("OCXO2", "isr_residual_ocxo2"),
    ]:
        stats = WelfordStats()
        nulls = 0
        for r in rows:
            v = r.get(key)
            if v is None:
                nulls += 1
            else:
                stats.update(float(v))
        if stats.n > 0:
            print(f"  {label:6s}  {stats.summary()}")
        else:
            print(f"  {label:6s}  no data ({nulls} nulls)")

    # --- GNSS Phase Lock Analysis ---
    print()
    print("-" * 70)
    print("GNSS PHASE LOCK ANALYSIS (isr_residual_gnss)")
    print("-" * 70)

    phase_lock_anomalies, phase_lock_lines = analyze_gnss_phase_lock(rows, recovery_boundaries)
    for line in phase_lock_lines:
        print(line)

    large_residual_anomalies = [a for a in phase_lock_anomalies if "anomalous" in a.lower()]
    informational_anomalies  = [a for a in phase_lock_anomalies if "anomalous" not in a.lower()]
    anomalies.extend(large_residual_anomalies)
    if informational_anomalies:
        print()
        print("  [informational -- not counted as failures]")
        for a in informational_anomalies:
            print(f"    INFO {a}")

    # --- Final verdict ---
    print()
    print("=" * 70)
    if anomalies:
        print(f"VERDICT: ANOMALIES FOUND ({len(anomalies)})")
        for a in anomalies:
            print(f"  * {a}")
    elif recovery_count > 0:
        print(f"VERDICT: CLEAN (with {recovery_count} recovery event{'s' if recovery_count != 1 else ''}, "
              f"{recovery_missing} gap records)")
    else:
        print("VERDICT: CLEAN -- all checks passed")
    print("=" * 70)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: campaign_analyzer <campaign_name>")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, active, count(*) as tb_count,
                           min((payload->>'pps_count')::int) as pps_min,
                           max((payload->>'pps_count')::int) as pps_max
                    FROM timebase t
                    JOIN campaigns c USING (campaign)
                    GROUP BY campaign, active
                    ORDER BY max(t.ts) DESC
                    LIMIT 20
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'ACTIVE':>7s} {'RECORDS':>8s} {'PPS RANGE':>20s}")
                print(f"  {'-'*20} {'-'*7} {'-'*8} {'-'*20}")
                for r in rows:
                    active    = ">" if r["active"] else " "
                    pps_range = f"{r['pps_min']}-{r['pps_max']}"
                    print(f"  {r['campaign']:<20s} {active:>7s} {r['tb_count']:>8d} {pps_range:>20s}")
        except Exception:
            pass
        sys.exit(1)

    analyze(sys.argv[1])


if __name__ == "__main__":
    main()