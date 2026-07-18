"""ZPNet Cycle Audit — TIMEBASE raw intervals versus endpoint subtraction.

For each public TIMEBASE row, display PPS, VCLOCK, OCXO1, and OCXO2.
Each rail shows only:

    *_tb      one-second raw cycle count carried by TIMEBASE
    *_calc    one-second cycle count computed here from consecutive raw DWT endpoints

The two values should match.

Usage:
    python -m zpnet.tests.cycle_audit <campaign_name> [limit]
    python cycle_audit.py <campaign_name> [limit]
    python cycle_audit.py <campaign_name> --limit 500
"""

from __future__ import annotations

import json
import sys
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


LANES: Tuple[str, ...] = ("PPS", "VCLOCK", "OCXO1", "OCXO2")
LANE_KEYS = {"VCLOCK": "vclock", "OCXO1": "ocxo1", "OCXO2": "ocxo2"}
PREFIXES = {"PPS": "p", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}
MICRO_PREFIXES = {"vclock": "v", "ocxo1": "o1", "ocxo2": "o2"}


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                NULLIF(payload->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_count', '')::bigint,
                NULLIF(payload->'payload'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_count', '')::bigint
            ) ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            out.append(payload)
    return out


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    forensic = root.get("forensics")
    return forensic if isinstance(forensic, dict) else {}


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = _as_int(value)
        if parsed is not None:
            return parsed
    return None


def _delta_u32(now: Optional[int], previous: Optional[int]) -> Optional[int]:
    if now is None or previous is None:
        return None
    return (now - previous) & 0xFFFFFFFF


def _fmt(value: Optional[int], width: int = 13, signed: bool = False) -> str:
    if value is None:
        text = "---"
    elif signed:
        text = f"{value:+,d}"
    else:
        text = f"{value:,d}"
    return f"{text:>{width}s}"


def pps_count(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def pps_raw_endpoint(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "pps", "dwt_at_edge"),
        frag.get("pps_dwt_at_edge"),
        forensic.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_normalized_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_raw_at_edge"),
        frag.get("pps_diag_pps_edge_dwt_isr_entry_raw"),
        frag.get("pps_edge_dwt_at_edge"),
    )


def pps_timebase_interval(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    prediction = frag.get("prediction") if isinstance(frag.get("prediction"), dict) else {}
    pps_prediction = prediction.get("pps") if isinstance(prediction.get("pps"), dict) else {}
    return _first_int(
        forensic.get("pps_obs"),
        frag.get("pps_obs"),
        root.get("pps_obs"),
        pps_prediction.get("actual_cycles"),
        frag.get("pps_actual_cycles"),
        prediction.get("pps_actual_cycles"),
    )


def lane_alpha_event(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Dict[str, Any]:
    candidates = (
        _nested_get(root, "clock_forensics", lane, "alpha_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event"),
        _nested_get(forensic, lane, "alpha_event"),
        _nested_get(frag, lane, "alpha_event"),
        _nested_get(root, lane, "alpha_event"),
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
    )
    for candidate in candidates:
        if isinstance(candidate, dict):
            return candidate
    return {}


def _micro_first_int(
    root: Dict[str, Any],
    frag: Dict[str, Any],
    forensic: Dict[str, Any],
    lane: str,
    *suffixes: str,
) -> Optional[int]:
    prefix = MICRO_PREFIXES[lane]
    values: List[Any] = []
    for suffix in suffixes:
        key = f"{prefix}_{suffix}"
        for source in (forensic, frag, root):
            values.append(source.get(key) if isinstance(source, dict) else None)
    return _first_int(*values)


def lane_raw_endpoint(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Optional[int]:
    alpha = lane_alpha_event(root, frag, forensic, lane)
    authority = alpha.get("dwt_authority") if isinstance(alpha.get("dwt_authority"), dict) else {}
    regression = alpha.get("regression") if isinstance(alpha.get("regression"), dict) else {}

    flat_prefixes = (f"{lane}_forensics_", f"{lane}_alpha_event_", f"{lane}_")
    flattened: List[Any] = []
    for prefix in flat_prefixes:
        flattened.extend(
            (
                frag.get(prefix + "dwt_original_at_event"),
                root.get(prefix + "dwt_original_at_event"),
            )
        )

    return _first_int(
        _micro_first_int(root, frag, forensic, lane, "raw", "orig"),
        authority.get("original_at_event"),
        alpha.get("dwt_original_at_event"),
        *flattened,
        regression.get("observed_dwt_at_event"),
        alpha.get("regression_observed_dwt_at_event"),
    )


def lane_timebase_interval(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Optional[int]:
    alpha = lane_alpha_event(root, frag, forensic, lane)
    gate = alpha.get("dwt_interval_gate") if isinstance(alpha.get("dwt_interval_gate"), dict) else {}
    science_obj = frag.get(lane) if isinstance(frag.get(lane), dict) else {}
    science = science_obj.get("science") if isinstance(science_obj.get("science"), dict) else {}

    return _first_int(
        _micro_first_int(root, frag, forensic, lane, "obs", "raw_cyc"),
        gate.get("observed_cycles"),
        alpha.get("dwt_interval_observed_cycles"),
        science.get("clock_observed_interval_cycles"),
    )


def collect(rows: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int]:
    output: List[Dict[str, Any]] = []
    gaps = 0

    previous_pps_count: Optional[int] = None
    previous_endpoint: Dict[str, Optional[int]] = {lane: None for lane in LANES}

    for record in rows:
        root = _root(record)
        frag = _frag(record)
        forensic = _forensics(record)
        count = pps_count(root, frag, forensic)
        if count is None:
            continue

        if previous_pps_count is not None and count != previous_pps_count + 1:
            gaps += 1
            previous_endpoint = {lane: None for lane in LANES}

        row: Dict[str, Any] = {"pps_count": count, "lanes": {}}

        for display_lane in LANES:
            if display_lane == "PPS":
                endpoint = pps_raw_endpoint(root, frag, forensic)
                tb_interval = pps_timebase_interval(root, frag, forensic)
            else:
                lane = LANE_KEYS[display_lane]
                endpoint = lane_raw_endpoint(root, frag, forensic, lane)
                tb_interval = lane_timebase_interval(root, frag, forensic, lane)

            calc_interval = _delta_u32(endpoint, previous_endpoint[display_lane])
            row["lanes"][display_lane] = {
                "tb": tb_interval,
                "calc": calc_interval,
            }

            if endpoint is not None:
                previous_endpoint[display_lane] = endpoint

        output.append(row)
        previous_pps_count = count

    return output, gaps


def print_report(campaign: str, rows: List[Dict[str, Any]], limit: int = 0) -> None:
    collected, gaps = collect(rows)
    if limit:
        collected = collected[:limit]

    print(f"Campaign: {campaign}  ({len(rows):,} TIMEBASE rows)")
    print()
    print("Cycle audit: TIMEBASE raw interval versus raw-endpoint subtraction")
    print("=================================================================")
    print("  *_tb    = raw one-second cycle count carried by TIMEBASE")
    print("  *_calc  = raw_dwt[n] - raw_dwt[n-1], computed by this report")
    print()

    header = [f"{'pps':>6s}"]
    separator = ["─" * 6]
    for lane in LANES:
        prefix = PREFIXES[lane]
        for label, width in ((f"{prefix}_tb", 13), (f"{prefix}_calc", 13)):
            header.append(f"{label:>{width}s}")
            separator.append("─" * width)
    print("  ".join(header))
    print("  ".join(separator))

    for row in collected:
        parts = [f"{row['pps_count']:>6d}"]
        for lane in LANES:
            data = row["lanes"][lane]
            parts.extend(
                (
                    _fmt(data["tb"], 13),
                    _fmt(data["calc"], 13),
                )
            )
        print("  ".join(parts))

    print()
    print(f"Rows shown:       {len(collected):,}")
    print(f"PPS-count gaps:   {gaps:,}")


def parse_args(argv: List[str]) -> Tuple[str, int]:
    if len(argv) < 2:
        raise SystemExit("Usage: cycle_audit <campaign_name> [limit] | --limit N")

    campaign = argv[1]
    limit = 0
    i = 2
    while i < len(argv):
        arg = argv[i]
        if arg == "--limit":
            if i + 1 >= len(argv):
                raise SystemExit("--limit requires an integer")
            try:
                limit = int(argv[i + 1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 2
        elif arg.startswith("--limit="):
            try:
                limit = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 1
        else:
            try:
                limit = int(arg)
            except ValueError as exc:
                raise SystemExit(f"unknown argument '{arg}'") from exc
            i += 1

    if limit < 0:
        raise SystemExit("limit must be zero or positive")
    return campaign, limit


def main() -> None:
    campaign, limit = parse_args(sys.argv)
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return
    print_report(campaign, rows, limit)


if __name__ == "__main__":
    main()
