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
  • Summary verdict: CLEAN or list of anomalies
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

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
# Analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"❌ No TIMEBASE rows found for campaign '{campaign}'")
        return

    total = len(rows)
    anomalies: List[str] = []

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
    if missing_count > 0:
        anomalies.append(f"{missing_count} missing TIMEBASE records")
    if total > expected_count:
        anomalies.append(f"{total - expected_count} EXTRA records (duplicates?)")

    # Find specific missing pps_counts
    pps_set = set(pps_counts)
    full_range = set(range(pps_min, pps_max + 1))
    missing_pps = sorted(full_range - pps_set)

    if missing_pps:
        print()
        print(f"  Missing pps_count values ({len(missing_pps)}):")
        # Show in compact ranges
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

        for s, e in ranges:
            if s == e:
                print(f"    {s}")
            else:
                print(f"    {s}-{e} ({e - s + 1} consecutive)")

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
            pps_jumps += 1
            if pps_jumps <= 10:
                print(f"  ⚠️  GAP at row {i}: {prev} -> {curr} (skipped {delta - 1})")

    if pps_jumps > 10:
        print(f"  ... and {pps_jumps - 10} more gaps")
    if pps_repeats > 5:
        print(f"  ... and {pps_repeats - 5} more repeats")

    print(f"\n  Jumps (gaps):   {pps_jumps}")
    print(f"  Repeats:        {pps_repeats}")
    print(f"  Regressions:    {pps_regressions}")

    if pps_jumps:
        anomalies.append(f"{pps_jumps} pps_count gaps")
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
        for r in rows:
            v = r.get(key)
            if v is None:
                null_count += 1
            else:
                values.append((int(r["pps_count"]), int(v)))

        if not values:
            print(f"    ⚠️  ALL NULL ({null_count}/{total} records)")
            continue

        if null_count > 0:
            print(f"    Nulls: {null_count}/{total}")

        # Monotonicity check
        non_monotonic = 0
        zero_deltas = 0
        negative_deltas = 0
        delta_stats = WelfordStats()

        for i in range(1, len(values)):
            prev_pps, prev_val = values[i - 1]
            curr_pps, curr_val = values[i]

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
            elif curr_val <= prev_val:
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

        for i in range(1, len(seqs)):
            prev_pps, prev_seq = seqs[i - 1]
            curr_pps, curr_seq = seqs[i]
            delta = curr_seq - prev_seq

            if delta == 0:
                seq_repeats += 1
                if seq_repeats <= 3:
                    print(f"  ⚠️  SEQ REPEAT at pps_count={curr_pps}: seq={curr_seq}")
            elif delta < 0:
                seq_regressions += 1
                if seq_regressions <= 3:
                    print(f"  ⚠️  SEQ REGRESSION at pps_count={curr_pps}: "
                          f"{prev_seq} -> {curr_seq}")
            elif delta > 1:
                seq_gaps += 1
                if seq_gaps <= 5:
                    print(f"  ⚠️  SEQ GAP at pps_count={curr_pps}: "
                          f"{prev_seq} -> {curr_seq} (missed {delta - 1})")

        print(f"\n  Range: {seqs[0][1]} -> {seqs[-1][1]}")
        print(f"  Gaps: {seq_gaps}  Repeats: {seq_repeats}  Regressions: {seq_regressions}")

        if seq_gaps:
            anomalies.append(f"Pi seq: {seq_gaps} gaps")
        if seq_repeats:
            anomalies.append(f"Pi seq: {seq_repeats} repeats")
        if seq_regressions:
            anomalies.append(f"Pi seq: {seq_regressions} regressions")

        if not (seq_gaps or seq_repeats or seq_regressions):
            print("  Verdict: ✅ sequential, no gaps")
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