"""
ZPNet TIMEBASE Field Dumper — shows all keys in TIMEBASE payloads.

Usage:
    .zt tb_keys Calibrate1           # first 3 records (default)
    .zt tb_keys Calibrate1 0         # record 0 only (by offset)
    .zt tb_keys Calibrate1 0 1       # records 0 and 1
    .zt tb_keys Calibrate1 5 8       # records 5 through 8
"""

from __future__ import annotations

import json
import sys
from typing import Any, Dict, List

from zpnet.shared.db import open_db


def analyze(campaign: str, start: int = 0, count: int = 3) -> None:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC
            LIMIT %s OFFSET %s
            """,
            (campaign, count, start),
        )
        rows = cur.fetchall()

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}' at offset {start}")
        return

    for i, row in enumerate(rows):
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)

        print(f"── Record {start + i} (pps_count={p.get('pps_count', '?')}) ──")
        print()
        for key in sorted(p.keys()):
            val = p[key]
            print(f"  {key:<40s}  {val!r}")
        print()


def main():
    if len(sys.argv) < 2:
        print("Usage: tb_keys <campaign> [start] [end]")
        print()
        print("  tb_keys Calibrate1         first 3 records")
        print("  tb_keys Calibrate1 0       record 0 only")
        print("  tb_keys Calibrate1 0 2     records 0 through 2")
        sys.exit(1)

    campaign = sys.argv[1]

    if len(sys.argv) == 2:
        # Default: first 3
        analyze(campaign, 0, 3)
    elif len(sys.argv) == 3:
        # Single record
        start = int(sys.argv[2])
        analyze(campaign, start, 1)
    else:
        # Range: start through end (inclusive)
        start = int(sys.argv[2])
        end = int(sys.argv[3])
        analyze(campaign, start, end - start + 1)


if __name__ == "__main__":
    main()