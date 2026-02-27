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
  • Summary verdict: CLEAN, CLEAN (with recovery), or anomalies
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Set, Tuple

from zpnet.shared.db import open_db


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
            # Check if the record after this gap is a recovery boundary
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
            # Check if nulls are at recovery boundaries (expected)
            null_at_boundary = sum(1 for p in null_pps if p in recovery_boundaries or p == 0)
            if null_at_boundary == null_count:
                print(f"    Nulls: {null_count}/{total} (all at recovery boundaries — expected)")
            else:
                print(f"    Nulls: {null_count}/{total} ({null_at_boundary} at boundaries, {null_count - null_at_boundary} unexpected)")

        # Monotonicity check
        non_monotonic = 0
        zero_deltas = 0
        negative_deltas = 0
        delta_stats = WelfordStats()

        for i in range(1, len(values)):
            prev_pps, prev_val = values[i - 1]
            curr_pps, curr_val = values[i]

            # Check if this is a recovery boundary
            at_boundary = curr_pps in recovery_boundaries

            # Only check adjacent pps_counts for delta stats
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
                # Across a gap — check monotonicity but classify boundary issues
                if curr_val <= prev_val:
                    if at_boundary and key == "pi_ns":
                        pass  # expected: pi_ns resets at recovery (epoch recomputed)
                    elif at_boundary:
                        pass  # other clocks: Teensy counter reset at recovery is OK
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

        # Range
        first_val = values[0][1]
        last_val = values[-1][1]
        print(f"    Range: {first_val} -> {last_val}")
        print(f"    Total delta: {last_val - first_val:,}")

        # Delta stats
        if delta_stats.n > 0:
            print(f"    Per-second deltas: {delta_stats.summary()}")

        # Verdict for this clock
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
                    pass  # PITIMER restarted, seq reset — expected
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
        # List available campaigns
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