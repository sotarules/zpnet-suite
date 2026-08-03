#!/usr/bin/env python3
"""
Print TIMEBASE JSON records around a PPS identity to stdout.

Examples:
    python dump_timebase_window.py Servo3 8321
    python dump_timebase_window.py Servo3 8321 --radius 4
    python dump_timebase_window.py Servo3 --start 8318 --end 8325

Stdout contains one JSON document containing:
  - campaign
  - requested range
  - record count
  - the complete TIMEBASE payload for every matching row
  - database id/timestamp metadata

No fields are filtered or normalized.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


PPS_COUNT_SQL = """
COALESCE(
    NULLIF(payload->>'teensy_pps_vclock_count', '')::bigint,
    NULLIF(payload->>'pps_count', '')::bigint,
    NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
    NULLIF(payload->'fragment'->>'pps_count', '')::bigint,
    NULLIF(payload->'fragment'->>'campaign_seconds', '')::bigint
)
"""


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dump complete TIMEBASE JSON records around a PPS count."
    )
    parser.add_argument("campaign", help="Campaign name, for example Servo3")
    parser.add_argument(
        "center",
        nargs="?",
        type=int,
        help="Center PPS count. Defaults to 8321 when neither --start nor --end is supplied.",
    )
    parser.add_argument(
        "--radius",
        type=int,
        default=3,
        help="Rows of PPS identity to include on each side of center (default: 3).",
    )
    parser.add_argument("--start", type=int, help="First PPS count to include.")
    parser.add_argument("--end", type=int, help="Last PPS count to include.")
    parser.add_argument(
        "--indent",
        type=int,
        default=2,
        help="JSON indentation level (default: 2).",
    )
    return parser.parse_args(argv)


def resolve_range(args: argparse.Namespace) -> tuple[int, int]:
    if args.radius < 0:
        raise ValueError("--radius must be zero or greater")

    if args.start is not None or args.end is not None:
        if args.start is None or args.end is None:
            raise ValueError("--start and --end must be supplied together")
        start = args.start
        end = args.end
    else:
        center = 8321 if args.center is None else args.center
        start = center - args.radius
        end = center + args.radius

    if start < 0:
        raise ValueError("start PPS count must be zero or greater")
    if end < start:
        raise ValueError("end PPS count must be greater than or equal to start")

    return start, end


def fetch_timebase_window(campaign: str, start: int, end: int) -> List[Dict[str, Any]]:
    query = f"""
        SELECT
            id,
            ts,
            {PPS_COUNT_SQL} AS pps_count,
            payload
        FROM timebase
        WHERE campaign = %s
          AND {PPS_COUNT_SQL} BETWEEN %s AND %s
        ORDER BY {PPS_COUNT_SQL} ASC, id ASC
    """

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(query, (campaign, start, end))
        rows = cur.fetchall()

    records: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)

        records.append(
            {
                "_db_id": row["id"],
                "_db_ts": str(row["ts"]),
                "_pps_count": int(row["pps_count"]),
                "payload": payload,
            }
        )

    return records


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    try:
        start, end = resolve_range(args)
        records = fetch_timebase_window(args.campaign, start, end)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    document = {
        "campaign": args.campaign,
        "requested_start_pps": start,
        "requested_end_pps": end,
        "record_count": len(records),
        "records": records,
    }

    print(json.dumps(document, indent=args.indent, sort_keys=False, default=str))

    found = [record["_pps_count"] for record in records]
    print(
        f"Returned {len(records)} TIMEBASE record(s) on stdout.",
        file=sys.stderr,
    )
    if found:
        print(
            f"Found PPS range: {found[0]} -> {found[-1]}",
            file=sys.stderr,
        )
        found_set = set(found)
        missing = [pps for pps in range(start, end + 1) if pps not in found_set]
        if missing:
            print(
                "Missing PPS identities in requested range: "
                + ", ".join(map(str, missing)),
                file=sys.stderr,
            )
    else:
        print(
            "No TIMEBASE rows matched the requested range.",
            file=sys.stderr,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())