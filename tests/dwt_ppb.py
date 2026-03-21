"""
ZPNet DWT PPB Diagnostic — Investigate DWT total offset

Usage:
    python -m zpnet.tests.dwt_ppb_diag <campaign_name> [limit]
    .zt dwt_ppb_diag Baseline2
    .zt dwt_ppb_diag Baseline2 20

Shows the first N records with raw DWT cycle counts, DWT ns, GNSS ns,
and cumulative PPB to help diagnose systematic offsets.
"""

from __future__ import annotations

import json
import sys
from typing import Any, Dict, List

from zpnet.shared.db import open_db


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


def analyze(campaign: str, limit: int = 20) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()

    # Show first record raw values
    r0 = rows[0]
    print("  ── First record (pps_count=0) raw values ──")
    print(f"  teensy_gnss_ns:       {r0.get('teensy_gnss_ns')}")
    print(f"  teensy_dwt_ns:        {r0.get('teensy_dwt_ns')}")
    print(f"  teensy_dwt_cycles:    {r0.get('teensy_dwt_cycles')}")
    print(f"  teensy_ocxo_ns:       {r0.get('teensy_ocxo_ns')}")
    print(f"  dwt_cyccnt_at_pps:    {r0.get('dwt_cyccnt_at_pps')}")
    print(f"  dwt_cycles_per_pps:   {r0.get('dwt_cycles_per_pps')}")
    print()

    # Check if dwt_cycles at pps 0 is suspiciously large (uint32 wrap)
    dwt0 = r0.get("teensy_dwt_cycles")
    if dwt0 is not None:
        dwt0 = int(dwt0)
        if dwt0 > 4_000_000_000:
            wrapped = dwt0 - (1 << 32)
            print(f"  ⚠️  dwt_cycles at pps 0 = {dwt0:,}")
            print(f"      Interpreted as signed: {wrapped:,}")
            print(f"      This looks like uint32 underflow (0 - {-wrapped})")
            print()

    # Show per-record table
    print(f"  {'pps':>5s}  {'gnss_ns':>18s}  {'dwt_ns':>18s}  {'dwt_cycles':>18s}  {'dwt-gnss':>14s}  {'ppb':>14s}")
    print(f"  {'─'*5}  {'─'*18}  {'─'*18}  {'─'*18}  {'─'*14}  {'─'*14}")

    count = 0
    for row in rows:
        pps = int(row.get("pps_count", -1))
        gnss_ns = row.get("teensy_gnss_ns")
        dwt_ns = row.get("teensy_dwt_ns")
        dwt_cycles = row.get("teensy_dwt_cycles")

        if gnss_ns is None or dwt_ns is None:
            continue

        gnss_ns = int(gnss_ns)
        dwt_ns = int(dwt_ns)
        dwt_cycles = int(dwt_cycles) if dwt_cycles is not None else 0

        diff = dwt_ns - gnss_ns
        ppb = ((dwt_ns - gnss_ns) / gnss_ns * 1e9) if gnss_ns > 0 else 0.0

        print(f"  {pps:>5d}  {gnss_ns:>18,}  {dwt_ns:>18,}  {dwt_cycles:>18,}  {diff:>+14,}  {ppb:>+14,.1f}")

        count += 1
        if count >= limit:
            break

    # Show last few records too if we limited early
    if limit < len(rows) and len(rows) > limit + 3:
        print(f"  {'...':>5s}")
        for row in rows[-3:]:
            pps = int(row.get("pps_count", -1))
            gnss_ns = int(row.get("teensy_gnss_ns", 0))
            dwt_ns = int(row.get("teensy_dwt_ns", 0))
            dwt_cycles = int(row.get("teensy_dwt_cycles", 0))

            diff = dwt_ns - gnss_ns
            ppb = ((dwt_ns - gnss_ns) / gnss_ns * 1e9) if gnss_ns > 0 else 0.0

            print(f"  {pps:>5d}  {gnss_ns:>18,}  {dwt_ns:>18,}  {dwt_cycles:>18,}  {diff:>+14,}  {ppb:>+14,.1f}")

    print()
    print(f"  {count} rows shown (of {len(rows)} total)")


def main():
    if len(sys.argv) < 2:
        print("Usage: dwt_ppb_diag <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    analyze(campaign, limit)


if __name__ == "__main__":
    main()