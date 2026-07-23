"""ZPNet raw_cycles — four-rail observed one-second DWT audit.

Every rail is independent.  For PPS, VCLOCK, OCXO1, and OCXO2 the report
shows both the producer-published lineage and an independent recomputation:

    computed_residual = observed_cycles[n] - observed_cycles[n - 1]
    residual_difference = published_residual - computed_residual

No FloorLine, projection, EMA, inferred endpoint, or cross-rail subtraction is
used.  Current firmware publishes the preferred facts under
``fragment.raw_cycles``.  Narrow fallbacks keep older observed-edge rows
readable without treating compatibility zeros as authority.
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

RAILS: Tuple[str, ...] = ("PPS", "VCLOCK", "OCXO1", "OCXO2")
RAIL_KEYS = {name: name.lower() for name in RAILS}
DEFAULT_GATE_CYCLES = 500
PLAUSIBLE_MIN = 900_000_000
PLAUSIBLE_MAX = 1_100_000_000


@dataclass
class Rail:
    observed: Optional[int] = None

    # Values published by the firmware producer under fragment.raw_cycles.*.
    published_previous: Optional[int] = None
    published_residual: Optional[int] = None

    # Values recomputed independently from adjacent rows returned by this report.
    computed_previous: Optional[int] = None
    computed_residual: Optional[int] = None

    # Producer minus report.  Zero means the published lineage agrees exactly
    # with the adjacent rows shown by raw_cycles.py.
    previous_difference: Optional[int] = None
    residual_difference: Optional[int] = None

    valid: Optional[bool] = None
    source: str = "MISSING"


@dataclass
class Row:
    count: Optional[int]
    disposition: str
    timeline_valid: Optional[bool]
    rails: Dict[str, Rail]
    issues: List[str]


def fetch(campaign: str, skip: int = 0, limit: int = 0) -> List[Dict[str, Any]]:
    pps_key_sql = """
        COALESCE(
          NULLIF(payload->>'pps_count','')::bigint,
          NULLIF(payload->'fragment'->>'pps_count','')::bigint,
          NULLIF(payload->'fragment'->>'campaign_seconds','')::bigint
        )
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        if skip > 0 and limit > 0:
            cur.execute(
                f"""
                SELECT payload FROM timebase
                WHERE campaign = %s
                  AND ({pps_key_sql}) > %s
                ORDER BY {pps_key_sql}
                LIMIT %s
                """,
                (campaign, skip, limit),
            )
        elif skip > 0:
            cur.execute(
                f"""
                SELECT payload FROM timebase
                WHERE campaign = %s
                  AND ({pps_key_sql}) > %s
                ORDER BY {pps_key_sql}
                """,
                (campaign, skip),
            )
        elif limit > 0:
            cur.execute(
                f"""
                SELECT payload FROM timebase
                WHERE campaign = %s
                ORDER BY {pps_key_sql}
                LIMIT %s
                """,
                (campaign, limit),
            )
        else:
            cur.execute(
                f"""
                SELECT payload FROM timebase
                WHERE campaign = %s
                ORDER BY {pps_key_sql}
                """,
                (campaign,),
            )
        rows = cur.fetchall()
    out: List[Dict[str, Any]] = []
    for item in rows:
        value = item["payload"]
        if isinstance(value, str):
            value = json.loads(value)
        if isinstance(value, dict):
            out.append(value)
    return out


def d(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def i(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def b(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    return None


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = i(value)
        if parsed is not None:
            return parsed
    return None


def fragment(record: Dict[str, Any]) -> Dict[str, Any]:
    root = d(record.get("payload")) or record
    frag = d(root.get("fragment"))
    return frag or root


def count_of(record: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    root = d(record.get("payload")) or record
    return first_int(
        frag.get("teensy_pps_vclock_count"), frag.get("pps_count"),
        frag.get("campaign_seconds"), root.get("pps_count")
    )


def fallback_observed(frag: Dict[str, Any], rail: str) -> Optional[int]:
    if rail == "PPS":
        return first_int(d(frag.get("pps")).get("dwt_cycles_between_edges"))
    if rail == "VCLOCK":
        v = d(frag.get("vclock"))
        return first_int(
            v.get("observed_cycles"),
            d(v.get("science")).get("cycles_between_edges"),
            frag.get("pps_vclock_dwt_cycles_between_edges"),
        )
    lane = d(frag.get(RAIL_KEYS[rail]))
    science = d(lane.get("science"))
    return first_int(
        lane.get("observed_cycles"),
        science.get("delta_raw_clock_interval_cycles"),
    )


def collect(records: Sequence[Dict[str, Any]]) -> List[Row]:
    rows: List[Row] = []
    previous: Dict[str, Optional[int]] = {name: None for name in RAILS}

    for record in records:
        frag = fragment(record)
        raw = d(frag.get("raw_cycles"))
        rails: Dict[str, Rail] = {}
        for name in RAILS:
            obj = d(raw.get(RAIL_KEYS[name]))
            observed = first_int(
                obj.get("observed_cycles"),
                fallback_observed(frag, name),
            )

            published_previous = i(obj.get("previous_observed_cycles"))
            published_residual = i(obj.get("residual_cycles"))
            computed_previous = previous[name]
            computed_residual = (
                observed - computed_previous
                if observed is not None and computed_previous is not None
                else None
            )
            previous_difference = (
                published_previous - computed_previous
                if published_previous is not None and computed_previous is not None
                else None
            )
            residual_difference = (
                published_residual - computed_residual
                if published_residual is not None and computed_residual is not None
                else None
            )

            rails[name] = Rail(
                observed=observed,
                published_previous=published_previous,
                published_residual=published_residual,
                computed_previous=computed_previous,
                computed_residual=computed_residual,
                previous_difference=previous_difference,
                residual_difference=residual_difference,
                valid=b(obj.get("valid")),
                source="RAW_CYCLES_OBSERVED_V1" if obj else "OBSERVED_FALLBACK",
            )
            previous[name] = observed

        rows.append(Row(
            count=count_of(record, frag),
            disposition=str(frag.get("candidate_disposition") or "ACCEPT").upper(),
            timeline_valid=b(frag.get("timeline_valid")),
            rails=rails,
            issues=[],
        ))
    return rows


def classify(row: Row, selected: Sequence[str], gate: int) -> List[str]:
    issues: List[str] = []
    if row.count is None:
        issues.append("missing PPS identity")
    if row.disposition != "ACCEPT":
        issues.append(f"candidate disposition is {row.disposition}")
    if row.timeline_valid is False:
        issues.append("timeline_valid is false")
    for name in selected:
        rail = row.rails[name]
        if rail.observed is None:
            issues.append(f"{name}: observed interval missing")
            continue
        if not PLAUSIBLE_MIN <= rail.observed <= PLAUSIBLE_MAX:
            issues.append(f"{name}: implausible observed interval {rail.observed:,d}")
        if rail.valid is False:
            issues.append(f"{name}: producer marked raw-cycle sample invalid")
        if (rail.computed_residual is not None and
                abs(rail.computed_residual) > gate):
            issues.append(
                f"{name}: computed current-minus-previous residual "
                f"{rail.computed_residual:+,d} exceeds gate"
            )
        if (rail.previous_difference is not None and
                rail.previous_difference != 0):
            issues.append(
                f"{name}: published previous differs from displayed previous by "
                f"{rail.previous_difference:+,d} cycles"
            )
        if (rail.residual_difference is not None and
                rail.residual_difference != 0):
            issues.append(
                f"{name}: published residual differs from computed residual by "
                f"{rail.residual_difference:+,d} cycles"
            )
    return issues


def fmt(value: Optional[int], width: int, signed: bool = False) -> str:
    text = "---" if value is None else (f"{value:+,d}" if signed else f"{value:,d}")
    return f"{text:>{width}}"


def parse(argv: Sequence[str]) -> Tuple[str, int, int, Optional[str], bool, int]:
    if len(argv) < 2:
        raise SystemExit("Usage: raw_cycles CAMPAIGN [limit] [clock] [--skip N] [--pathology-only] [--pathology-gate N]")
    campaign = argv[1]
    limit = 0
    skip = 0
    clock: Optional[str] = None
    pathology_only = False
    gate = DEFAULT_GATE_CYCLES
    pos: List[str] = []
    idx = 2
    while idx < len(argv):
        arg = argv[idx]
        if arg == "--pathology-only":
            pathology_only = True
            idx += 1
        elif arg == "--pathology-gate":
            gate = int(argv[idx + 1]); idx += 2
        elif arg.startswith("--pathology-gate="):
            gate = int(arg.split("=", 1)[1]); idx += 1
        elif arg == "--skip":
            skip = int(argv[idx + 1]); idx += 2
        elif arg.startswith("--skip="):
            skip = int(arg.split("=", 1)[1]); idx += 1
        elif arg == "--clock":
            clock = argv[idx + 1].upper(); idx += 2
        elif arg.startswith("--clock="):
            clock = arg.split("=", 1)[1].upper(); idx += 1
        elif arg == "--limit":
            limit = int(argv[idx + 1]); idx += 2
        elif arg.startswith("--limit="):
            limit = int(arg.split("=", 1)[1]); idx += 1
        elif arg in {"--align-ocxo", "--delay-pps-vclock", "--slip"}:
            idx += 1
        else:
            pos.append(arg); idx += 1
    for arg in pos:
        if arg.upper() in RAILS:
            clock = arg.upper()
        else:
            limit = int(arg)
    if skip < 0:
        raise SystemExit("skip must be zero or greater")
    if limit < 0:
        raise SystemExit("limit must be zero or greater")
    if clock is not None and clock not in RAILS:
        raise SystemExit("clock must be PPS, VCLOCK, OCXO1, or OCXO2")
    return campaign, limit, skip, clock, pathology_only, gate


def main(argv: Sequence[str]) -> None:
    campaign, limit, skip, clock, pathology_only, gate = parse(argv)
    records = fetch(campaign, skip, limit)
    rows = collect(records)
    selected = (clock,) if clock else RAILS
    for row in rows:
        row.issues = classify(row, selected, gate)

    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={clock or 'ALL'})")
    print("\nFour-rail observed one-second cycle audit")
    print("═════════════════════════════════════")
    print("  Published values come from fragment.raw_cycles; computed values use adjacent displayed rows.")
    print("  Differences are published minus computed.  Zero proves exact agreement.")
    print("  No cross-rail residuals, FloorLine, projection, EMA, or inferred endpoints are used.")
    print(f"  Pathology gate: {gate:,d} cycles.\n")

    header = [f"{'pps':>7}"]
    for name in selected:
        prefix = {"PPS":"p", "VCLOCK":"v", "OCXO1":"o1", "OCXO2":"o2"}[name]
        header += [
            f"{prefix + '_cyc':>13}",
            f"{prefix + '_pprev':>13}",
            f"{prefix + '_cprev':>13}",
            f"{prefix + '_pdif':>9}",
            f"{prefix + '_pres':>9}",
            f"{prefix + '_cres':>9}",
            f"{prefix + '_rdif':>9}",
        ]
    print("  ".join(header))
    print("  ".join("─" * len(x) for x in header))

    pathologies = 0
    stats: Dict[str, List[int]] = {name: [] for name in selected}
    residual_stats: Dict[str, List[int]] = {name: [] for name in selected}
    producer_residual_stats: Dict[str, List[int]] = {name: [] for name in selected}
    previous_mismatch_counts: Dict[str, int] = {name: 0 for name in selected}
    residual_mismatch_counts: Dict[str, int] = {name: 0 for name in selected}
    source_counts: Dict[str, Dict[str, int]] = {name: {} for name in selected}
    for row in rows:
        if row.issues:
            pathologies += 1
        if pathology_only and not row.issues:
            continue
        fields = [fmt(row.count, 7)]
        for name in selected:
            rail = row.rails[name]
            fields += [
                fmt(rail.observed, 13),
                fmt(rail.published_previous, 13),
                fmt(rail.computed_previous, 13),
                fmt(rail.previous_difference, 9, True),
                fmt(rail.published_residual, 9, True),
                fmt(rail.computed_residual, 9, True),
                fmt(rail.residual_difference, 9, True),
            ]
            if rail.observed is not None:
                stats[name].append(rail.observed)
            if rail.computed_residual is not None:
                residual_stats[name].append(rail.computed_residual)
            if rail.published_residual is not None:
                producer_residual_stats[name].append(rail.published_residual)
            if rail.previous_difference not in (None, 0):
                previous_mismatch_counts[name] += 1
            if rail.residual_difference not in (None, 0):
                residual_mismatch_counts[name] += 1
            source_counts[name][rail.source] = source_counts[name].get(rail.source, 0) + 1
        print("  ".join(fields))
        for issue in row.issues:
            print(f"    └─ {issue}")

    print(f"\nPathological rows: {pathologies:,} / {len(rows):,}")
    print("\nSummary")
    print("═══════")
    for name in selected:
        vals = stats[name]
        computed = residual_stats[name]
        published = producer_residual_stats[name]
        mean = sum(vals) / len(vals) if vals else math.nan
        computed_mean = sum(computed) / len(computed) if computed else math.nan
        computed_max = max((abs(x) for x in computed), default=0)
        published_mean = sum(published) / len(published) if published else math.nan
        published_max = max((abs(x) for x in published), default=0)
        sources = ",".join(
            f"{source}:{count}"
            for source, count in sorted(source_counts[name].items())
        ) or "none"
        print(
            f"  {name:<6} intervals={len(vals):>5,d} "
            f"mean={mean:>15,.3f} cycles  "
            f"computed_res_mean={computed_mean:>9,.3f} "
            f"computed_max_abs={computed_max:>5,d}  "
            f"published_res_mean={published_mean:>9,.3f} "
            f"published_max_abs={published_max:>5,d}"
        )
        print(
            f"         previous_mismatches={previous_mismatch_counts[name]:>5,d}  "
            f"residual_mismatches={residual_mismatch_counts[name]:>5,d}  "
            f"sources={sources}"
        )


if __name__ == "__main__":
    main(sys.argv)
