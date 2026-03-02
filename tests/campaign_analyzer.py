"""
ZPNet Campaign Analyzer — TIMEBASE integrity and continuity audit

Usage:
    python -m zpnet.tests.campaign_analyzer <campaign_name>
    .zt campaign_analyzer Baseline2

Reads all TIMEBASE rows for the named campaign and produces:
  • Record count (expected vs actual, missing pps_count indices)
  • pps_count continuity (gaps, repeats, regressions)
  • Per-clock monotonicity checks (ns/cycles must always increase)
  • Per-clock zero/duplicate detection
  • Per-clock delta statistics (min, max, mean, stddev)
  • Pi capture sequence integrity (seq gaps, repeats)
  • Recovery-aware: gaps in pps_count are classified as recovery gaps
    and cross-boundary anomalies (pi_ns reset, pi_seq regression)
    are expected artifacts, not failures.
  • PPB sanity check: flags any record where a clock domain's
    cumulative PPB (relative to GNSS) exceeds an absolute threshold.
    No real clock in ZPNet drifts more than 10,000 ppb — values
    beyond that indicate an epoch projection error (typically Pi
    after recovery).
  • Recovery forensics: at each recovery boundary, reconstructs
    the epoch computation that _recover_campaign() would have
    performed, showing tau values, projected ticks/ns, actual
    post-recovery values, and the resulting epoch error.
  • Summary verdict: CLEAN, CLEAN (with recovery), or anomalies
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Set, Tuple

from zpnet.shared.db import open_db


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

# Absolute PPB threshold.  Any clock domain showing |ppb| above this
# is flagged as an anomaly.  No real ZPNet clock drifts this much —
# values beyond 10K ppb indicate an epoch projection error, typically
# in the Pi domain after recovery.
PPB_ABSOLUTE_THRESHOLD = 10_000

# Constants matching the Pi-side CLOCKS process
NS_PER_SECOND = 1_000_000_000
PI_TIMER_FREQ = 54_000_000
PI_NS_PER_TICK = 1e9 / PI_TIMER_FREQ  # ~18.519 ns/tick
DWT_CYCLES_PER_NS_NUM = 126
DWT_CYCLES_PER_NS_DEN = 125


# ---------------------------------------------------------------------
# Clock domain definitions
# ---------------------------------------------------------------------

CLOCK_DOMAINS = [
    # (label, payload_key, description)
    ("GNSS",     "teensy_gnss_ns",    "GNSS nanoseconds"),
    ("DWT_NS",   "teensy_dwt_ns",     "DWT nanoseconds"),
    ("DWT_CYC",  "teensy_dwt_cycles", "DWT cycles"),
    ("OCXO",     "teensy_ocxo_ns",    "OCXO nanoseconds"),
    ("PI_NS",    "pi_ns",             "Pi nanoseconds"),
    ("PI_CORR",  "pi_corrected",      "Pi corrected ticks"),
    ("PI_RAW",   "pi_counter",        "Pi raw counter"),
]

# Clock domains eligible for PPB sanity check (ns-valued, meaningful
# relative to GNSS).  DWT_CYC, PI_CORR, PI_RAW are tick-valued and
# don't produce meaningful PPB — they're covered by monotonicity checks.
PPB_DOMAINS = [
    # (label, ns_key, description)
    ("DWT",  "teensy_dwt_ns",  "DWT vs GNSS"),
    ("OCXO", "teensy_ocxo_ns", "OCXO vs GNSS"),
    ("Pi",   "pi_ns",          "Pi vs GNSS"),
]


# ---------------------------------------------------------------------
# Welford's online stats (lightweight)
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
    """Fetch all TIMEBASE rows for a campaign, ordered by pps_count."""
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
    """
    Find pps_count values that immediately follow a recovery gap.
    These are the first record after each gap where cross-boundary
    comparisons (pi_ns delta, pi_seq continuity) are not meaningful.
    """
    boundaries: Set[int] = set()
    for i in range(1, len(rows)):
        prev = int(rows[i - 1]["pps_count"])
        curr = int(rows[i]["pps_count"])
        if curr > prev + 1:
            boundaries.add(curr)
    return boundaries


# ---------------------------------------------------------------------
# PPB sanity check — absolute threshold
# ---------------------------------------------------------------------

def _compute_ppb(clock_ns: int, gnss_ns: int) -> Optional[float]:
    """Compute parts-per-billion of clock relative to GNSS."""
    if gnss_ns <= 0:
        return None
    return ((clock_ns - gnss_ns) / gnss_ns) * 1e9


def analyze_ppb_sanity(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> Tuple[List[str], List[str]]:
    """
    Check every TIMEBASE record for PPB values exceeding the absolute
    threshold.  Any clock domain showing |ppb| > PPB_ABSOLUTE_THRESHOLD
    is flagged — no real ZPNet clock drifts that much.

    Returns:
        anomalies: list of anomaly description strings
        report_lines: list of formatted output lines for display
    """
    anomalies: List[str] = []
    lines: List[str] = []

    for label, ns_key, desc in PPB_DOMAINS:
        violations: List[Dict[str, Any]] = []
        ppb_stats = WelfordStats()
        valid_count = 0

        for row in rows:
            gnss_ns_raw = row.get("teensy_gnss_ns")
            clock_ns_raw = row.get(ns_key)

            if gnss_ns_raw is None or clock_ns_raw is None:
                continue

            gnss_ns = int(gnss_ns_raw)
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
                at_boundary = pps in recovery_boundaries
                violations.append({
                    "pps_count": pps,
                    "ppb": ppb,
                    "clock_ns": clock_ns,
                    "gnss_ns": gnss_ns,
                    "at_recovery": at_boundary,
                })

        # --- Report for this domain ---
        lines.append(f"\n  [{label}] {desc}")

        if ppb_stats.n > 0:
            lines.append(f"    PPB stats: {ppb_stats.summary()}")
        else:
            lines.append(f"    PPB stats: no valid data")

        if not violations:
            lines.append(f"    PPB sanity: ✅ all {valid_count} records "
                         f"within ±{PPB_ABSOLUTE_THRESHOLD:,} ppb")
        else:
            at_recovery = [v for v in violations if v["at_recovery"]]
            steady_state = [v for v in violations if not v["at_recovery"]]

            first_v = violations[0]
            worst_v = max(violations, key=lambda v: abs(v["ppb"]))

            affected_start = violations[0]["pps_count"]
            affected_end = violations[-1]["pps_count"]

            lines.append(
                f"    ⚠️  {len(violations)} record(s) exceed "
                f"±{PPB_ABSOLUTE_THRESHOLD:,} ppb threshold"
            )
            lines.append(
                f"    Affected range: pps_count {affected_start} -> {affected_end} "
                f"({len(violations)} of {valid_count} records)"
            )

            if at_recovery:
                lines.append(
                    f"    At recovery boundaries: {len(at_recovery)}"
                )
            if steady_state:
                lines.append(
                    f"    In steady-state: {len(steady_state)}"
                )

            lines.append(
                f"    First violation: pps_count={first_v['pps_count']}  "
                f"ppb={first_v['ppb']:+,.1f}  "
                f"(clock_ns={first_v['clock_ns']:,}  "
                f"gnss_ns={first_v['gnss_ns']:,})"
            )
            lines.append(
                f"    Worst violation: pps_count={worst_v['pps_count']}  "
                f"ppb={worst_v['ppb']:+,.1f}"
            )

            samples = violations[:5]
            if len(violations) > 10:
                samples += violations[-3:]
            lines.append(f"    Sample violations:")
            shown = set()
            for v in samples:
                if v["pps_count"] in shown:
                    continue
                shown.add(v["pps_count"])
                recovery_tag = " (recovery)" if v["at_recovery"] else ""
                lines.append(
                    f"      pps={v['pps_count']:>7d}  "
                    f"ppb={v['ppb']:>+14,.1f}  "
                    f"Δns={v['clock_ns'] - v['gnss_ns']:>+14,d}"
                    f"{recovery_tag}"
                )
            if len(violations) > len(shown):
                lines.append(f"      ... and {len(violations) - len(shown)} more")

            anomalies.append(
                f"{label}: {len(violations)} records with |ppb| > "
                f"{PPB_ABSOLUTE_THRESHOLD:,} "
                f"(worst: {worst_v['ppb']:+,.1f} ppb at pps_count="
                f"{worst_v['pps_count']})"
            )

    return anomalies, lines


# ---------------------------------------------------------------------
# Recovery forensics — reconstruct epoch computation at each boundary
# ---------------------------------------------------------------------

def analyze_recovery_forensics(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> List[str]:
    """
    At each recovery boundary, reconstruct the computation that
    _recover_campaign() would have performed:

      1. Read last pre-gap row (the "anchor")
      2. Compute tau_dwt, tau_ocxo, tau_pi from anchor
      3. Compute gap_seconds and projections:
         - Teensy clocks: padded project_seconds (elapsed + 2)
         - Pi: actual_elapsed (post_pps - pre_pps, the true sync edge)
      4. Compare projected values against actual post-recovery values
      5. Reconstruct the epoch and show the error

    This mirrors _recover_campaign() exactly:
      - Teensy projections use the padded project_seconds because they
        are instructions loaded into the Teensy's accumulators.
      - Pi projection uses actual_elapsed because the epoch is derived
        from a hardware reading minus projected ticks, so it must match
        where the sync edge actually landed.
    """
    lines: List[str] = []

    if not recovery_boundaries:
        return lines

    # Build a pps_count -> row index for fast lookup
    pps_to_idx: Dict[int, int] = {}
    for i, r in enumerate(rows):
        pps_to_idx[int(r["pps_count"])] = i

    for boundary_pps in sorted(recovery_boundaries):
        boundary_idx = pps_to_idx.get(boundary_pps)
        if boundary_idx is None or boundary_idx == 0:
            lines.append(f"\n  Recovery at pps_count={boundary_pps}: ⚠️  no pre-gap row found")
            continue

        post_row = rows[boundary_idx]
        pre_row = rows[boundary_idx - 1]

        pre_pps = int(pre_row["pps_count"])
        post_pps = int(post_row["pps_count"])
        gap_seconds = post_pps - pre_pps - 1
        elapsed_seconds = post_pps - pre_pps

        # Teensy projections use padded project_seconds (matches _recover_campaign)
        teensy_project_seconds = elapsed_seconds + 2

        # Pi projection uses actual elapsed from anchor to sync edge
        # (matches the fixed _recover_campaign — no padding)
        pi_actual_elapsed = elapsed_seconds

        lines.append(f"\n  {'=' * 62}")
        lines.append(f"  RECOVERY FORENSICS: pps_count {pre_pps} -> {post_pps}")
        lines.append(f"  {'=' * 62}")
        lines.append(f"  Gap: {gap_seconds} missing seconds")
        lines.append(f"  Elapsed (pps delta): {elapsed_seconds} seconds")
        lines.append(f"  Teensy project seconds: {teensy_project_seconds} (elapsed + 2, padded)")
        lines.append(f"  Pi actual elapsed: {pi_actual_elapsed} (no padding)")

        # --- Anchor values (pre-gap row) ---
        anchor_gnss_ns = int(pre_row.get("teensy_gnss_ns") or 0)
        anchor_dwt_ns = int(pre_row.get("teensy_dwt_ns") or 0)
        anchor_ocxo_ns = int(pre_row.get("teensy_ocxo_ns") or 0)
        anchor_pi_ns = int(pre_row.get("pi_ns") or 0)
        anchor_pi_corrected = int(pre_row.get("pi_corrected") or 0)
        anchor_dwt_cycles = int(pre_row.get("teensy_dwt_cycles") or 0)

        anchor_system_utc = pre_row.get("system_time_utc") or pre_row.get("gnss_time_utc", "?")

        lines.append(f"\n  Anchor (pps_count={pre_pps}):")
        lines.append(f"    system_time_utc:   {anchor_system_utc}")
        lines.append(f"    gnss_ns:           {anchor_gnss_ns:,}")
        lines.append(f"    dwt_ns:            {anchor_dwt_ns:,}")
        lines.append(f"    dwt_cycles:        {anchor_dwt_cycles:,}")
        lines.append(f"    ocxo_ns:           {anchor_ocxo_ns:,}")
        lines.append(f"    pi_ns:             {anchor_pi_ns:,}")
        lines.append(f"    pi_corrected:      {anchor_pi_corrected:,}")

        # --- Tau computation (mirrors _recover_campaign) ---
        tau_dwt = float(anchor_dwt_ns) / float(anchor_gnss_ns) if anchor_gnss_ns > 0 else 1.0
        tau_ocxo = float(anchor_ocxo_ns) / float(anchor_gnss_ns) if (anchor_gnss_ns > 0 and anchor_ocxo_ns > 0) else 1.0
        tau_pi = float(anchor_pi_ns) / float(anchor_gnss_ns) if (anchor_gnss_ns > 0 and anchor_pi_ns > 0) else 1.0

        lines.append(f"\n  Tau (computed from anchor):")
        lines.append(f"    tau_dwt:   {tau_dwt:.12f}")
        lines.append(f"    tau_ocxo:  {tau_ocxo:.12f}")
        lines.append(f"    tau_pi:    {tau_pi:.12f}")

        if anchor_pi_ns == 0:
            lines.append(f"    ⚠️  anchor pi_ns=0 — tau_pi defaulted to 1.0")

        # --- Teensy projections (padded, matches _recover_campaign) ---
        tps = teensy_project_seconds

        proj_gnss_ns = anchor_gnss_ns + (tps * NS_PER_SECOND)
        proj_dwt_ns = anchor_dwt_ns + int(tps * NS_PER_SECOND * tau_dwt)
        proj_ocxo_ns = anchor_ocxo_ns + int(tps * NS_PER_SECOND * tau_ocxo) if anchor_ocxo_ns > 0 else 0
        proj_dwt_cycles = (proj_dwt_ns * DWT_CYCLES_PER_NS_NUM) // DWT_CYCLES_PER_NS_DEN

        lines.append(f"\n  Teensy projections (padded, project_seconds={tps}):")
        lines.append(f"    proj_gnss_ns:      {proj_gnss_ns:,}")
        lines.append(f"    proj_dwt_ns:       {proj_dwt_ns:,}")
        lines.append(f"    proj_dwt_cycles:   {proj_dwt_cycles:,}")
        lines.append(f"    proj_ocxo_ns:      {proj_ocxo_ns:,}")

        # --- Pi projection (actual elapsed, matches fixed _recover_campaign) ---
        pae = pi_actual_elapsed

        proj_pi_ns = anchor_pi_ns + int(pae * NS_PER_SECOND * tau_pi)
        proj_pi_ticks = (proj_pi_ns * PI_TIMER_FREQ) // NS_PER_SECOND

        lines.append(f"\n  Pi projection (actual elapsed={pae}, no padding):")
        lines.append(f"    proj_pi_ns:        {proj_pi_ns:,}")
        lines.append(f"    proj_pi_ticks:     {proj_pi_ticks:,}")

        # --- Actual post-recovery values ---
        actual_gnss_ns = int(post_row.get("teensy_gnss_ns") or 0)
        actual_dwt_ns = int(post_row.get("teensy_dwt_ns") or 0)
        actual_ocxo_ns = int(post_row.get("teensy_ocxo_ns") or 0)
        actual_pi_ns = int(post_row.get("pi_ns") or 0)
        actual_pi_corrected = int(post_row.get("pi_corrected") or 0)
        actual_dwt_cycles = int(post_row.get("teensy_dwt_cycles") or 0)

        actual_system_utc = post_row.get("system_time_utc") or post_row.get("gnss_time_utc", "?")

        lines.append(f"\n  Actual (pps_count={post_pps}):")
        lines.append(f"    system_time_utc:   {actual_system_utc}")
        lines.append(f"    gnss_ns:           {actual_gnss_ns:,}")
        lines.append(f"    dwt_ns:            {actual_dwt_ns:,}")
        lines.append(f"    dwt_cycles:        {actual_dwt_cycles:,}")
        lines.append(f"    ocxo_ns:           {actual_ocxo_ns:,}")
        lines.append(f"    pi_ns:             {actual_pi_ns:,}")
        lines.append(f"    pi_corrected:      {actual_pi_corrected:,}")

        # --- Teensy projection errors ---
        lines.append(f"\n  Teensy projection errors (actual - projected):")

        d_gnss = actual_gnss_ns - proj_gnss_ns
        d_dwt = actual_dwt_ns - proj_dwt_ns
        d_ocxo = actual_ocxo_ns - proj_ocxo_ns

        lines.append(f"    gnss_ns error:     {d_gnss:+,d} ns")
        lines.append(f"    dwt_ns error:      {d_dwt:+,d} ns")
        lines.append(f"    ocxo_ns error:     {d_ocxo:+,d} ns")
        lines.append(f"    (Teensy errors are ~-2s from padding — expected and harmless)")

        # --- Pi projection error ---
        d_pi = actual_pi_ns - proj_pi_ns

        lines.append(f"\n  Pi projection error (actual - projected):")
        lines.append(f"    pi_ns error:       {d_pi:+,d} ns")

        if actual_gnss_ns > 0:
            d_pi_ppb = (d_pi / actual_gnss_ns) * 1e9
            lines.append(f"    pi_ns error as ppb of gnss: {d_pi_ppb:+,.1f}")

        # --- Epoch reconstruction ---
        if actual_pi_ns > 0 and actual_pi_corrected > 0:
            actual_pi_ticks_elapsed = (actual_pi_ns * PI_TIMER_FREQ) // NS_PER_SECOND
            actual_epoch = actual_pi_corrected - actual_pi_ticks_elapsed

            ideal_pi_ns = int(actual_gnss_ns * 1.0000077)
            ideal_pi_ticks_elapsed = (ideal_pi_ns * PI_TIMER_FREQ) // NS_PER_SECOND
            ideal_epoch = actual_pi_corrected - ideal_pi_ticks_elapsed

            epoch_error_ticks = actual_epoch - ideal_epoch
            epoch_error_ns = int(epoch_error_ticks * PI_NS_PER_TICK)

            lines.append(f"\n  Epoch reconstruction:")
            lines.append(f"    actual pi_ticks elapsed:  {actual_pi_ticks_elapsed:,}")
            lines.append(f"    actual epoch (reversed):  {actual_epoch:,}")
            lines.append(f"    ideal pi_ticks elapsed:   {ideal_pi_ticks_elapsed:,}")
            lines.append(f"    ideal epoch (for ~7650 ppb): {ideal_epoch:,}")
            lines.append(f"    epoch error:              {epoch_error_ticks:+,d} ticks")
            lines.append(f"    epoch error:              {epoch_error_ns:+,d} ns ({epoch_error_ns / 1e9:+.3f} seconds)")

            # Reverse-engineer what projected_pi_ticks was actually used
            inferred_proj_ticks = actual_pi_corrected - actual_epoch
            lines.append(f"\n  Inferred from actual epoch:")
            lines.append(f"    projected_pi_ticks used:  {inferred_proj_ticks:,}")
            lines.append(f"    our estimate of proj_pi_ticks: {proj_pi_ticks:,}")
            lines.append(f"    difference:               {inferred_proj_ticks - proj_pi_ticks:+,d} ticks")

            if pae > 0:
                inferred_proj_pi_ns = (inferred_proj_ticks * NS_PER_SECOND) // PI_TIMER_FREQ
                inferred_addition = inferred_proj_pi_ns - anchor_pi_ns
                inferred_tau_pi = inferred_addition / (pae * NS_PER_SECOND) if (pae * NS_PER_SECOND) > 0 else 0
                lines.append(f"    inferred proj_pi_ns:      {inferred_proj_pi_ns:,}")
                lines.append(f"    inferred pi_ns addition:  {inferred_addition:,}")
                lines.append(f"    inferred tau_pi used:     {inferred_tau_pi:.12f}")
                lines.append(f"    our computed tau_pi:      {tau_pi:.12f}")
                lines.append(f"    tau_pi discrepancy:       {inferred_tau_pi - tau_pi:+.12f}")

        # --- PPB check on post-recovery row ---
        if actual_gnss_ns > 0 and actual_pi_ns > 0:
            post_ppb = _compute_ppb(actual_pi_ns, actual_gnss_ns)
            lines.append(f"\n  Post-recovery Pi PPB: {post_ppb:+,.1f}")
            if post_ppb is not None and abs(post_ppb) > PPB_ABSOLUTE_THRESHOLD:
                lines.append(f"    ⚠️  EXCEEDS ±{PPB_ABSOLUTE_THRESHOLD:,} ppb threshold")

    return lines


# ---------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"❌ No TIMEBASE rows found for campaign '{campaign}'")
        return

    total = len(rows)
    anomalies: List[str] = []

    # --- Recovery boundary detection ---
    recovery_boundaries = find_recovery_boundaries(rows)
    recovery_count = len(recovery_boundaries)

    # --- pps_count range and continuity ---
    pps_counts = [int(r["pps_count"]) for r in rows]
    pps_min = min(pps_counts)
    pps_max = max(pps_counts)
    expected_count = pps_max - pps_min + 1
    campaign_seconds = pps_max - pps_min

    # Time range
    first_ts = rows[0].get("system_time_utc") or rows[0].get("gnss_time_utc", "?")
    last_ts = rows[-1].get("system_time_utc") or rows[-1].get("gnss_time_utc", "?")

    print("=" * 70)
    print(f"CAMPAIGN ANALYSIS: {campaign}")
    print("=" * 70)
    print()
    print(f"  Time range:       {first_ts}")
    print(f"                 -> {last_ts}")
    print(f"  pps_count range:  {pps_min} -> {pps_max}")
    print(f"  Campaign seconds: {campaign_seconds}")
    hrs = campaign_seconds // 3600
    mins = (campaign_seconds % 3600) // 60
    secs = campaign_seconds % 60
    print(f"  Campaign duration: {hrs:02d}:{mins:02d}:{secs:02d}")
    print()
    print(f"  Expected records: {expected_count}")
    print(f"  Actual records:   {total}")
    missing_count = expected_count - total
    print(f"  Missing records:  {missing_count}")
    if recovery_count > 0:
        print(f"  Recovery events:  {recovery_count}")
    if total > expected_count:
        anomalies.append(f"{total - expected_count} EXTRA records (duplicates?)")

    # Find specific missing pps_counts
    pps_set = set(pps_counts)
    full_range = set(range(pps_min, pps_max + 1))
    missing_pps = sorted(full_range - pps_set)

    # Classify missing records as recovery gaps vs unexpected gaps
    recovery_missing = 0
    unexpected_missing = 0

    if missing_pps:
        print()
        # Group into ranges
        ranges = []
        start = missing_pps[0]
        end = start
        for v in missing_pps[1:]:
            if v == end + 1:
                end = v
            else:
                ranges.append((start, end))
                start = v
                end = v
        ranges.append((start, end))

        print(f"  Missing pps_count values ({len(missing_pps)}):")
        for s, e in ranges:
            count = e - s + 1
            next_pps = e + 1
            is_recovery = next_pps in recovery_boundaries

            label = " (recovery)" if is_recovery else " ⚠️"
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

    # Check for duplicate pps_counts
    if len(pps_counts) != len(pps_set):
        dup_count = len(pps_counts) - len(pps_set)
        anomalies.append(f"{dup_count} duplicate pps_count values")
        print(f"\n  ⚠️  {dup_count} duplicate pps_count values detected!")

        from collections import Counter
        ctr = Counter(pps_counts)
        dups = [(k, v) for k, v in ctr.items() if v > 1]
        for pps, count in sorted(dups)[:20]:
            print(f"    pps_count={pps} appears {count} times")
        if len(dups) > 20:
            print(f"    ... and {len(dups) - 20} more")

    # --- pps_count continuity (sequential check) ---
    print()
    print("-" * 70)
    print("PPS_COUNT CONTINUITY")
    print("-" * 70)

    pps_jumps = 0
    pps_recovery_jumps = 0
    pps_regressions = 0
    pps_repeats = 0

    for i in range(1, len(rows)):
        prev = int(rows[i - 1]["pps_count"])
        curr = int(rows[i]["pps_count"])
        delta = curr - prev

        if delta == 0:
            pps_repeats += 1
            if pps_repeats <= 5:
                print(f"  ⚠️  REPEAT at row {i}: pps_count={curr}")
        elif delta < 0:
            pps_regressions += 1
            if pps_regressions <= 5:
                print(f"  ⚠️  REGRESSION at row {i}: {prev} -> {curr} (delta={delta})")
        elif delta > 1:
            is_recovery = curr in recovery_boundaries
            if is_recovery:
                pps_recovery_jumps += 1
                print(f"  ℹ️  RECOVERY GAP at row {i}: {prev} -> {curr} (skipped {delta - 1})")
            else:
                pps_jumps += 1
                if pps_jumps <= 10:
                    print(f"  ⚠️  GAP at row {i}: {prev} -> {curr} (skipped {delta - 1})")

    if pps_jumps > 10:
        print(f"  ... and {pps_jumps - 10} more gaps")
    if pps_repeats > 5:
        print(f"  ... and {pps_repeats - 5} more repeats")

    print(f"\n  Jumps (unexpected):  {pps_jumps}")
    print(f"  Jumps (recovery):    {pps_recovery_jumps}")
    print(f"  Repeats:             {pps_repeats}")
    print(f"  Regressions:         {pps_regressions}")

    if pps_jumps:
        anomalies.append(f"{pps_jumps} unexpected pps_count gaps")
    if pps_repeats:
        anomalies.append(f"{pps_repeats} pps_count repeats")
    if pps_regressions:
        anomalies.append(f"{pps_regressions} pps_count regressions")

    # --- Per-clock monotonicity and delta analysis ---
    print()
    print("-" * 70)
    print("CLOCK DOMAIN ANALYSIS")
    print("-" * 70)

    for label, key, desc in CLOCK_DOMAINS:
        print(f"\n  [{label}] {desc}  (key: {key})")

        values = []
        null_count = 0
        null_pps = []
        for r in rows:
            v = r.get(key)
            if v is None:
                null_count += 1
                null_pps.append(int(r["pps_count"]))
            else:
                values.append((int(r["pps_count"]), int(v)))

        if not values:
            print(f"    ⚠️  ALL NULL ({null_count}/{total} records)")
            continue

        if null_count > 0:
            null_at_boundary = sum(1 for p in null_pps if p in recovery_boundaries or p == 0)
            if null_at_boundary == null_count:
                print(f"    Nulls: {null_count}/{total} (all at recovery boundaries — expected)")
            else:
                print(f"    Nulls: {null_count}/{total} ({null_at_boundary} at boundaries, {null_count - null_at_boundary} unexpected)")

        non_monotonic = 0
        zero_deltas = 0
        negative_deltas = 0
        delta_stats = WelfordStats()

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
                        print(f"    ⚠️  ZERO DELTA at pps_count={curr_pps}: "
                              f"value={curr_val}")
                elif delta < 0:
                    negative_deltas += 1
                    if negative_deltas <= 3:
                        print(f"    ⚠️  NEGATIVE DELTA at pps_count={curr_pps}: "
                              f"{prev_val} -> {curr_val} (delta={delta})")
            else:
                if curr_val <= prev_val:
                    if at_boundary and key == "pi_ns":
                        pass
                    elif at_boundary:
                        pass
                    else:
                        non_monotonic += 1
                        if non_monotonic <= 3:
                            print(f"    ⚠️  NON-MONOTONIC across gap: "
                                  f"pps {prev_pps}->{curr_pps}  "
                                  f"val {prev_val}->{curr_val}")

        if zero_deltas > 3:
            print(f"    ... and {zero_deltas - 3} more zero deltas")
        if negative_deltas > 3:
            print(f"    ... and {negative_deltas - 3} more negative deltas")

        first_val = values[0][1]
        last_val = values[-1][1]
        print(f"    Range: {first_val} -> {last_val}")
        print(f"    Total delta: {last_val - first_val:,}")

        if delta_stats.n > 0:
            print(f"    Per-second deltas: {delta_stats.summary()}")

        issues = []
        if zero_deltas:
            issues.append(f"{zero_deltas} zero")
        if negative_deltas:
            issues.append(f"{negative_deltas} negative")
        if non_monotonic:
            issues.append(f"{non_monotonic} non-monotonic")

        if issues:
            verdict = "⚠️  " + ", ".join(issues)
            anomalies.append(f"{label}: {', '.join(issues)} deltas")
        else:
            verdict = "✅ monotonic, no zeros, no duplicates"
        print(f"    Verdict: {verdict}")

    # --- Pi capture sequence check ---
    print()
    print("-" * 70)
    print("PI CAPTURE SEQUENCE (diag_pi_seq)")
    print("-" * 70)

    seqs = []
    seq_nulls = 0
    for r in rows:
        s = r.get("diag_pi_seq")
        if s is None:
            seq_nulls += 1
        else:
            seqs.append((int(r["pps_count"]), int(s)))

    if not seqs:
        print("  ALL NULL")
    else:
        if seq_nulls:
            print(f"  Nulls: {seq_nulls}/{total}")

        seq_gaps = 0
        seq_repeats = 0
        seq_regressions = 0
        seq_recovery_regressions = 0

        for i in range(1, len(seqs)):
            prev_pps, prev_seq = seqs[i - 1]
            curr_pps, curr_seq = seqs[i]
            delta = curr_seq - prev_seq

            at_boundary = curr_pps in recovery_boundaries

            if delta == 0:
                seq_repeats += 1
                if seq_repeats <= 3:
                    print(f"  ⚠️  SEQ REPEAT at pps_count={curr_pps}: seq={curr_seq}")
            elif delta < 0:
                if at_boundary:
                    seq_recovery_regressions += 1
                    print(f"  ℹ️  SEQ REGRESSION at recovery boundary pps_count={curr_pps}: "
                          f"{prev_seq} -> {curr_seq} (PITIMER restarted — expected)")
                else:
                    seq_regressions += 1
                    if seq_regressions <= 3:
                        print(f"  ⚠️  SEQ REGRESSION at pps_count={curr_pps}: "
                              f"{prev_seq} -> {curr_seq}")
            elif delta > 1:
                if at_boundary:
                    pass
                else:
                    seq_gaps += 1
                    if seq_gaps <= 5:
                        print(f"  ⚠️  SEQ GAP at pps_count={curr_pps}: "
                              f"{prev_seq} -> {curr_seq} (missed {delta - 1})")

        print(f"\n  Range: {seqs[0][1]} -> {seqs[-1][1]}")
        print(f"  Gaps: {seq_gaps}  Repeats: {seq_repeats}  Regressions: {seq_regressions}")
        if seq_recovery_regressions:
            print(f"  Recovery regressions: {seq_recovery_regressions} (expected)")

        if not (seq_gaps or seq_repeats or seq_regressions):
            print("  Verdict: ✅ sequential, no gaps" +
                  (f" ({seq_recovery_regressions} recovery regression{'s' if seq_recovery_regressions != 1 else ''} — expected)"
                   if seq_recovery_regressions else ""))
        else:
            print("  Verdict: ⚠️  anomalies detected")

    # --- PPB sanity check ---
    print()
    print("-" * 70)
    print(f"PPB SANITY CHECK (threshold: ±{PPB_ABSOLUTE_THRESHOLD:,} ppb)")
    print("-" * 70)

    ppb_anomalies, ppb_lines = analyze_ppb_sanity(rows, recovery_boundaries)
    for line in ppb_lines:
        print(line)
    anomalies.extend(ppb_anomalies)

    # --- Recovery forensics ---
    if recovery_boundaries:
        print()
        print("-" * 70)
        print("RECOVERY FORENSICS")
        print("-" * 70)

        forensic_lines = analyze_recovery_forensics(rows, recovery_boundaries)
        for line in forensic_lines:
            print(line)

    # --- ISR residual summary ---
    print()
    print("-" * 70)
    print("ISR RESIDUALS (per-second, nanoseconds)")
    print("-" * 70)

    for label, key in [
        ("GNSS", "isr_residual_gnss"),
        ("DWT",  "isr_residual_dwt"),
        ("OCXO", "isr_residual_ocxo"),
        ("Pi",   "isr_residual_pi"),
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

    # --- Final verdict ---
    print()
    print("=" * 70)
    if anomalies:
        print(f"VERDICT: ⚠️  {len(anomalies)} ANOMALIES FOUND")
        for a in anomalies:
            print(f"  • {a}")
    elif recovery_count > 0:
        print(f"VERDICT: ✅ CLEAN (with {recovery_count} recovery event{'s' if recovery_count != 1 else ''}, "
              f"{recovery_missing} gap records)")
    else:
        print("VERDICT: ✅ CLEAN — all checks passed")
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
                print(f"  {'─'*20} {'─'*7} {'─'*8} {'─'*20}")
                for r in rows:
                    active = "▶" if r["active"] else " "
                    pps_range = f"{r['pps_min']}-{r['pps_max']}"
                    print(f"  {r['campaign']:<20s} {active:>7s} {r['tb_count']:>8d} {pps_range:>20s}")
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    analyze(campaign)


if __name__ == "__main__":
    main()