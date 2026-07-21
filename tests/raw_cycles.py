"""ZPNet raw_cycles — four-rail observed one-second DWT audit.

Every rail is independent.  For PPS, VCLOCK, OCXO1, and OCXO2:

    residual_cycles = observed_cycles[n] - observed_cycles[n - 1]

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
    previous: Optional[int] = None
    residual: Optional[int] = None
    valid: Optional[bool] = None
    source: str = "MISSING"


@dataclass
class Row:
    count: Optional[int]
    disposition: str
    timeline_valid: Optional[bool]
    rails: Dict[str, Rail]
    issues: List[str]


def fetch(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
              NULLIF(payload->>'pps_count','')::bigint,
              NULLIF(payload->'fragment'->>'pps_count','')::bigint,
              NULLIF(payload->'fragment'->>'campaign_seconds','')::bigint
            )
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
            observed = first_int(obj.get("observed_cycles"), fallback_observed(frag, name))
            explicit_previous = i(obj.get("previous_observed_cycles"))
            prior = explicit_previous if explicit_previous is not None else previous[name]
            explicit_residual = i(obj.get("residual_cycles"))
            residual = (
                explicit_residual
                if explicit_residual is not None
                else observed - prior
                if observed is not None and prior is not None
                else None
            )
            rails[name] = Rail(
                observed=observed,
                previous=prior,
                residual=residual,
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
        if rail.residual is not None and abs(rail.residual) > gate:
            issues.append(f"{name}: current-minus-previous residual {rail.residual:+,d} exceeds gate")
    return issues


def fmt(value: Optional[int], width: int, signed: bool = False) -> str:
    text = "---" if value is None else (f"{value:+,d}" if signed else f"{value:,d}")
    return f"{text:>{width}}"


def parse(argv: Sequence[str]) -> Tuple[str, int, Optional[str], bool, int]:
    if len(argv) < 2:
        raise SystemExit("Usage: raw_cycles CAMPAIGN [limit] [clock] [--pathology-only] [--pathology-gate N]")
    campaign = argv[1]
    limit = 0
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
    if clock is not None and clock not in RAILS:
        raise SystemExit("clock must be PPS, VCLOCK, OCXO1, or OCXO2")
    return campaign, limit, clock, pathology_only, gate


def main(argv: Sequence[str]) -> None:
    campaign, limit, clock, pathology_only, gate = parse(argv)
    records = fetch(campaign)
    rows = collect(records)
    if limit > 0:
        rows = rows[-limit:]
    selected = (clock,) if clock else RAILS
    for row in rows:
        row.issues = classify(row, selected, gate)

    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={clock or 'ALL'})")
    print("\nFour-rail observed one-second cycle audit")
    print("═════════════════════════════════════")
    print("  Every residual is current observed interval minus previous observed interval for the same rail.")
    print("  No cross-rail residuals, FloorLine, projection, EMA, or inferred endpoints are used.")
    print(f"  Pathology gate: {gate:,d} cycles.\n")

    header = [f"{'pps':>7}"]
    for name in selected:
        prefix = {"PPS":"p", "VCLOCK":"v", "OCXO1":"o1", "OCXO2":"o2"}[name]
        header += [f"{prefix + '_cyc':>13}", f"{prefix + '_res':>9}"]
    print("  ".join(header))
    print("  ".join("─" * len(x) for x in header))

    pathologies = 0
    stats: Dict[str, List[int]] = {name: [] for name in selected}
    residual_stats: Dict[str, List[int]] = {name: [] for name in selected}
    for row in rows:
        if row.issues:
            pathologies += 1
        if pathology_only and not row.issues:
            continue
        fields = [fmt(row.count, 7)]
        for name in selected:
            rail = row.rails[name]
            fields += [fmt(rail.observed, 13), fmt(rail.residual, 9, True)]
            if rail.observed is not None:
                stats[name].append(rail.observed)
            if rail.residual is not None:
                residual_stats[name].append(rail.residual)
        print("  ".join(fields))
        for issue in row.issues:
            print(f"    └─ {issue}")

    print(f"\nPathological rows: {pathologies:,} / {len(rows):,}")
    print("\nSummary")
    print("═══════")
    for name in selected:
        vals = stats[name]
        res = residual_stats[name]
        mean = sum(vals) / len(vals) if vals else math.nan
        rmean = sum(res) / len(res) if res else math.nan
        rmax = max((abs(x) for x in res), default=0)
        print(f"  {name:<6} intervals={len(vals):>5,d} mean={mean:>15,.3f} cycles  residual_mean={rmean:>9,.3f} max_abs={rmax:>5,d}")


if __name__ == "__main__":
    main(sys.argv)
