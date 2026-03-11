"""
ZPNet Raw DWT Cycles — bare per-second cycle table with running stats

Recovery-aware: gaps in pps_count (from service restarts) are shown
as marker lines and excluded from the Welford accumulators.  Only
consecutive pps_count pairs (delta == 1) contribute to statistics.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    .zt raw_cycles Shakeout1
    .zt raw_cycles Shakeout1 50
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List

from zpnet.shared.db import open_db

DWT_FREQ_HZ = 1_008_000_000


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


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()
    print(f"  {'pps':>5s}  {'actual':>14s}  {'ideal':>14s}  {'residual':>10s}  {'mean':>10s}  {'stddev':>10s}")
    print(f"  {'─'*5}  {'─'*14}  {'─'*14}  {'─'*10}  {'─'*10}  {'─'*10}")

    # Welford accumulators
    w_n = 0
    w_mean = 0.0
    w_m2 = 0.0

    count = 0
    gaps = 0
    for i in range(1, len(rows)):
        prev = rows[i - 1]
        curr = rows[i]

        pps = curr.get("pps_count")
        prev_pps = prev.get("pps_count")
        start = prev.get("teensy_dwt_cycles")
        end = curr.get("teensy_dwt_cycles")

        if pps is None or prev_pps is None or start is None or end is None:
            continue

        pps = int(pps)
        prev_pps = int(prev_pps)
        start = int(start)
        end = int(end)

        # Recovery gap: pps_count jumped by more than 1.
        # Show a marker line and skip — the delta spans multiple
        # seconds and would poison the Welford accumulators.
        if pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  {'--- recovery':>14s}  gap {prev_pps} → {pps}  ({skipped} seconds lost)  ---")
            continue

        actual = end - start
        residual = actual - DWT_FREQ_HZ

        # Welford update
        w_n += 1
        x = float(residual)
        d1 = x - w_mean
        w_mean += d1 / w_n
        d2 = x - w_mean
        w_m2 += d1 * d2

        stddev = math.sqrt(w_m2 / (w_n - 1)) if w_n >= 2 else 0.0

        print(f"  {pps:>5d}  {actual:>14,}  {DWT_FREQ_HZ:>14,}  {residual:>+10,}  {w_mean:>+10.1f}  {stddev:>10.1f}")

        count += 1
        if limit and count >= limit:
            break

    print()
    print(f"  {count} rows, {gaps} recovery gap(s) skipped")


def main():
    if len(sys.argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit]")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, count(*) as cnt
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s}")
                print(f"  {'─'*20} {'─'*8}")
                for r in rows:
                    print(f"  {r['campaign']:<20s} {r['cnt']:>8d}")
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()