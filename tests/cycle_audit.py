"""ZPNet Cycle Audit — TEMPEST raw-interval lineage reconstruction.

The generalized campaign_detail stream intentionally preserves compact completed
intervals rather than every raw DWT endpoint.  For each TEMPEST campaign row,
this report therefore audits the durable raw-cycle lineage that replaces the
retired endpoint-subtraction court:

    *_raw      producer-published observed one-second interval
    *_recon    previous_observed_cycles + residual_cycles

On contiguous rows, previous_observed_cycles must also equal the observed value
from the preceding campaign row.  The reconstructed and observed values should
match exactly for PPS, VCLOCK, OCXO1, and OCXO2.

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
RAIL_KEYS = {lane: lane.lower() for lane in LANES}
PREFIXES = {"PPS": "p", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}
CAMPAIGN_TYPE = "TEMPEST"
PPS_COUNT_SQL = """
NULLIF(
    payload #>> '{campaign,tempest,teensy_pps_vclock_count}',
    ''
)::bigint
"""


def _dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_int(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _root(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Return the persisted detail, unwrapping only a true outer envelope."""
    if any(
        key in payload
        for key in ("schema", "monitor_fragment", "fragment", "campaign_row", "campaign")
    ):
        return payload
    inner = _dict(payload.get("payload"))
    return inner or payload


def _tempest_decoration(payload: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(payload)
    return _dict(_dict(root.get("campaign")).get("tempest"))


def _fragment(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Return the merged immutable campaign row plus final TEMPEST decoration."""
    root = _root(payload)
    direct = _dict(root.get("fragment")) or _dict(root.get("campaign_row"))
    monitor = _dict(root.get("monitor_fragment"))
    embedded = _dict(monitor.get("campaign_row"))
    decoration = _tempest_decoration(payload)
    source = embedded or direct
    if source or decoration:
        merged = dict(source)
        merged.update(decoration)
        return merged
    return root


def _payload_pps(payload: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    root = _root(payload)
    for value in (
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        root.get("pps_count"),
    ):
        parsed = _as_int(value)
        if parsed is not None:
            return parsed
    return None


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch completed TEMPEST details in campaign-public PPS order."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            f"""
            SELECT id, {PPS_COUNT_SQL} AS pps_count, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND payload #> '{{campaign,tempest}}' IS NOT NULL
              AND {PPS_COUNT_SQL} IS NOT NULL
            ORDER BY {PPS_COUNT_SQL} ASC, id ASC
            """,
            (CAMPAIGN_TYPE, campaign),
        )
        rows = cur.fetchall()

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if not isinstance(payload, dict):
            continue

        db_pps = int(row["pps_count"])
        frag = _fragment(payload)
        payload_pps = _payload_pps(payload, frag)
        if payload_pps is not None and payload_pps != db_pps:
            raise ValueError(
                "campaign_detail campaign PPS mismatch: "
                f"id={row['id']} db={db_pps} payload={payload_pps}"
            )

        out.append({"_db_pps": db_pps, "payload": payload})
    return out


def _fmt(value: Optional[int], width: int = 13, signed: bool = False) -> str:
    if value is None:
        text = "---"
    elif signed:
        text = f"{value:+,d}"
    else:
        text = f"{value:,d}"
    return f"{text:>{width}s}"


def collect(rows: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int, int, int]:
    output: List[Dict[str, Any]] = []
    gaps = 0
    previous_mismatches = 0
    reconstruction_mismatches = 0

    previous_pps: Optional[int] = None
    previous_observed: Dict[str, Optional[int]] = {lane: None for lane in LANES}

    for record in rows:
        count = _as_int(record.get("_db_pps"))
        payload = _dict(record.get("payload"))
        if count is None or not payload:
            continue

        frag = _fragment(payload)
        raw = _dict(frag.get("raw_cycles"))
        gap = previous_pps is not None and count != previous_pps + 1
        if gap:
            gaps += 1
            previous_observed = {lane: None for lane in LANES}

        notes: List[str] = []
        row: Dict[str, Any] = {"pps_count": count, "lanes": {}, "notes": notes}

        for lane in LANES:
            obj = _dict(raw.get(RAIL_KEYS[lane]))
            observed = _as_int(obj.get("observed_cycles"))
            published_previous = _as_int(obj.get("previous_observed_cycles"))
            published_residual = _as_int(obj.get("residual_cycles"))
            adjacent_previous = None if gap else previous_observed[lane]
            reconstructed = (
                published_previous + published_residual
                if published_previous is not None and published_residual is not None
                else None
            )

            previous_match = (
                None
                if adjacent_previous is None or published_previous is None
                else published_previous == adjacent_previous
            )
            reconstruction_match = (
                None
                if observed is None or reconstructed is None
                else observed == reconstructed
            )

            if previous_match is False:
                previous_mismatches += 1
                notes.append(f"{lane}:PREV")
            if reconstruction_match is False:
                reconstruction_mismatches += 1
                notes.append(f"{lane}:RECON")
            if observed is None:
                notes.append(f"{lane}:MISSING")

            row["lanes"][lane] = {
                "raw": observed,
                "recon": reconstructed,
                "published_residual": published_residual,
                "previous_match": previous_match,
                "reconstruction_match": reconstruction_match,
            }

            previous_observed[lane] = observed

        output.append(row)
        previous_pps = count

    return output, gaps, previous_mismatches, reconstruction_mismatches


def print_report(campaign: str, rows: List[Dict[str, Any]], limit: int = 0) -> None:
    collected, gaps, previous_mismatches, reconstruction_mismatches = collect(rows)
    if limit:
        collected = collected[:limit]

    print(f"Campaign: {campaign}  ({len(rows):,} TEMPEST campaign_detail rows)")
    print()
    print("Cycle audit: raw interval versus producer-lineage reconstruction")
    print("================================================================")
    print("  *_raw    = raw_cycles.<rail>.observed_cycles")
    print("  *_recon  = previous_observed_cycles + residual_cycles")
    print()

    header = [f"{'pps':>6s}"]
    separator = ["─" * 6]
    for lane in LANES:
        prefix = PREFIXES[lane]
        for label, width in ((f"{prefix}_raw", 13), (f"{prefix}_recon", 13)):
            header.append(f"{label:>{width}s}")
            separator.append("─" * width)
    header.append("NOTE")
    separator.append("─" * 12)
    print("  ".join(header))
    print("  ".join(separator))

    for row in collected:
        parts = [f"{row['pps_count']:>6d}"]
        for lane in LANES:
            data = row["lanes"][lane]
            parts.extend((_fmt(data["raw"], 13), _fmt(data["recon"], 13)))
        parts.append("|".join(row["notes"]) if row["notes"] else "OK")
        print("  ".join(parts))

    print()
    print(f"Rows shown:                  {len(collected):,}")
    print(f"Campaign PPS gaps:           {gaps:,}")
    print(f"Previous-lineage mismatches: {previous_mismatches:,}")
    print(f"Reconstruction mismatches:   {reconstruction_mismatches:,}")


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
        print(f"No TEMPEST campaign_detail rows for campaign '{campaign}'")
        return
    print_report(campaign, rows, limit)


if __name__ == "__main__":
    main()
