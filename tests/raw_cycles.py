"""ZPNet raw_cycles — four-rail observed one-second DWT audit.

For PPS, VCLOCK, OCXO1, and OCXO2 the report intentionally shows only the
producer-published observed cycle count and producer-published last-second raw
residual.  A single NOTE column summarizes current endpoint-delay verdicts for
all displayed clocks.  Raw endpoints remain immutable; no projection, repair,
or cross-rail subtraction is performed.
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
EXPLANATION_GATE_CYCLES = 16
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

    delay_status: str = "UNKNOWN"
    delay_by: str = "UNKNOWN"
    delay_cycles_valid: Optional[bool] = None
    delay_cycles: Optional[int] = None
    delay_confidence: str = "NONE"
    delay_uncertainty_cycles: Optional[int] = None
    interval_delay_valid: Optional[bool] = None
    interval_delay_cycles: Optional[int] = None
    residual_delay_valid: Optional[bool] = None
    residual_delay_cycles: Optional[int] = None
    residual_delay_by: str = "UNKNOWN"
    residual_after_delay_cycles: Optional[int] = None
    delay_explains_residual: Optional[bool] = None
    residual_after_difference: Optional[int] = None


@dataclass
class Row:
    count: Optional[int]
    disposition: str
    timeline_valid: Optional[bool]
    rails: Dict[str, Rail]
    issues: List[str]
    explanations: List[str]


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


def record_root(record: Dict[str, Any]) -> Dict[str, Any]:
    return d(record.get("payload")) or record


def fragment(record: Dict[str, Any]) -> Dict[str, Any]:
    root = record_root(record)
    frag = d(root.get("fragment"))
    return frag or root


def forensics(record: Dict[str, Any], frag: Dict[str, Any]) -> Dict[str, Any]:
    root = record_root(record)
    return d(root.get("forensics")) or d(frag.get("forensics"))


def text(value: Any, default: str = "UNKNOWN") -> str:
    return str(value).upper() if value is not None else default


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
    delay_prefix = {"PPS": "pps", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}

    for record in records:
        frag = fragment(record)
        forensic = forensics(record, frag)
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

            prefix = delay_prefix[name]
            residual_delay_valid = b(forensic.get(f"{prefix}_residual_delay_valid"))
            residual_delay_cycles = i(forensic.get(f"{prefix}_residual_delay_cycles"))
            residual_after_delay = i(
                forensic.get(f"{prefix}_residual_after_delay_cycles")
            )
            independently_normalized = (
                computed_residual - residual_delay_cycles
                if computed_residual is not None
                and residual_delay_valid is True
                and residual_delay_cycles is not None
                else None
            )
            residual_after_difference = (
                residual_after_delay - independently_normalized
                if residual_after_delay is not None
                and independently_normalized is not None
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
                delay_status=text(forensic.get(f"{prefix}_delay_status")),
                delay_by=text(forensic.get(f"{prefix}_delay_by")),
                delay_cycles_valid=b(
                    forensic.get(f"{prefix}_delay_cycles_valid")
                ),
                delay_cycles=i(forensic.get(f"{prefix}_delay_cycles")),
                delay_confidence=text(
                    forensic.get(f"{prefix}_delay_confidence"), "NONE"
                ),
                delay_uncertainty_cycles=i(
                    forensic.get(f"{prefix}_delay_uncertainty_cycles")
                ),
                interval_delay_valid=b(
                    forensic.get(f"{prefix}_interval_delay_valid")
                ),
                interval_delay_cycles=i(
                    forensic.get(f"{prefix}_interval_delay_cycles")
                ),
                residual_delay_valid=residual_delay_valid,
                residual_delay_cycles=residual_delay_cycles,
                residual_delay_by=text(
                    forensic.get(f"{prefix}_residual_delay_by")
                ),
                residual_after_delay_cycles=(
                    independently_normalized
                    if independently_normalized is not None
                    else residual_after_delay
                ),
                delay_explains_residual=b(
                    forensic.get(f"{prefix}_delay_explains_residual")
                ),
                residual_after_difference=residual_after_difference,
            )
            previous[name] = observed

        rows.append(Row(
            count=count_of(record, frag),
            disposition=str(frag.get("candidate_disposition") or "ACCEPT").upper(),
            timeline_valid=b(frag.get("timeline_valid")),
            rails=rails,
            issues=[],
            explanations=[],
        ))
    return rows


def classify(row: Row, selected: Sequence[str], gate: int) -> List[str]:
    issues: List[str] = []
    explanations: List[str] = []
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

        if rail.computed_residual is not None and abs(rail.computed_residual) > gate:
            normalized = rail.residual_after_delay_cycles
            explained = (
                rail.residual_delay_valid is True
                and rail.residual_delay_cycles is not None
                and normalized is not None
                and (
                    rail.delay_explains_residual is True
                    or (
                        rail.delay_explains_residual is None
                        and abs(normalized) <= EXPLANATION_GATE_CYCLES
                    )
                )
            )
            if explained:
                explanations.append(
                    f"{name}: raw residual {rail.computed_residual:+,d} is explained "
                    f"by ISR contribution {rail.residual_delay_cycles:+,d} from "
                    f"{rail.residual_delay_by}; normalized residual {normalized:+,d}"
                )
            else:
                detail = ""
                if rail.residual_delay_valid is True and normalized is not None:
                    detail = (
                        f"; ISR contribution {rail.residual_delay_cycles:+,d} from "
                        f"{rail.residual_delay_by}, normalized {normalized:+,d}"
                    )
                issues.append(
                    f"{name}: computed current-minus-previous residual "
                    f"{rail.computed_residual:+,d} exceeds gate{detail}"
                )

        if rail.previous_difference not in (None, 0):
            issues.append(
                f"{name}: published previous differs from displayed previous by "
                f"{rail.previous_difference:+,d} cycles"
            )
        if rail.residual_difference not in (None, 0):
            issues.append(
                f"{name}: published residual differs from computed residual by "
                f"{rail.residual_difference:+,d} cycles"
            )
        if rail.residual_after_difference not in (None, 0):
            issues.append(
                f"{name}: firmware normalized residual differs from report math by "
                f"{rail.residual_after_difference:+,d} cycles"
            )

    row.explanations = explanations
    return issues


def fmt(value: Optional[int], width: int, signed: bool = False) -> str:
    text = "---" if value is None else (f"{value:+,d}" if signed else f"{value:,d}")
    return f"{text:>{width}}"


def verdict_note(row: Row, selected: Sequence[str]) -> str:
    """Return one compact, row-wide interrupt timing verdict.

    Quiet rows are deliberately reduced to ON_TIME.  Exceptional clocks are
    named explicitly so the report can be scanned without reading a separate
    forensic column for every rail.
    """
    aliases = {
        "VCLOCK_TIMEPOP": "VCLOCK",
        "QTIMER1": "VCLOCK",
        "MASKING_OR_UNKNOWN_CPU": "CPU/MASK",
        "MULTIPLE_ISR": "MULTIPLE ISR",
    }
    notes: List[str] = []
    for name in selected:
        rail = row.rails[name]
        if rail.delay_status == "ON_TIME":
            continue
        if rail.delay_status == "DELAYED":
            cause = aliases.get(rail.delay_by, rail.delay_by)
            notes.append(f"{name} DELAYED BY {cause}")
        elif rail.delay_status == "UNKNOWN":
            notes.append(f"{name} UNKNOWN")
        else:
            notes.append(f"{name} {rail.delay_status}")
    return " | ".join(notes) if notes else "ON_TIME"



def bool_text(value: Optional[bool]) -> str:
    if value is True:
        return "YES"
    if value is False:
        return "NO"
    return "UNKNOWN"


def print_extended_row(row: Row, selected: Sequence[str]) -> None:
    """Print focused forensic testimony beneath an interesting compact row.

    A row is expanded by the caller when any displayed published raw residual has
    magnitude greater than 12 cycles.  Within the expansion, only rails whose
    own residual exceeds that threshold or whose endpoint verdict is exceptional
    are shown.  This keeps the ordinary report narrow while preserving the
    evidence needed to reason about OCXO1 Disease and larger ISR events.
    """
    interesting = [
        name for name in selected
        if (
            row.rails[name].published_residual is not None
            and abs(row.rails[name].published_residual) > 12
        )
        or row.rails[name].delay_status != "ON_TIME"
    ]
    if not interesting:
        return

    print("      └─ Extended ISR-delay testimony")
    for name in interesting:
        rail = row.rails[name]
        cause = rail.delay_by
        residual_cause = rail.residual_delay_by
        print(f"         {name}")
        print(
            "           Verdict   "
            f"status={rail.delay_status}  cause={cause}  "
            f"confidence={rail.delay_confidence}"
        )
        print(
            "           Endpoint  "
            f"valid={bool_text(rail.delay_cycles_valid)}  "
            f"delay={fmt(rail.delay_cycles, 0, True).strip()} cycles  "
            f"uncertainty={fmt(rail.delay_uncertainty_cycles, 0).strip()} cycles"
        )
        print(
            "           Interval  "
            f"valid={bool_text(rail.interval_delay_valid)}  "
            f"contribution={fmt(rail.interval_delay_cycles, 0, True).strip()} cycles"
        )
        print(
            "           Residual  "
            f"raw={fmt(rail.published_residual, 0, True).strip()}  "
            f"valid={bool_text(rail.residual_delay_valid)}  "
            f"ISR={fmt(rail.residual_delay_cycles, 0, True).strip()}  "
            f"by={residual_cause}  "
            f"normalized={fmt(rail.residual_after_delay_cycles, 0, True).strip()}  "
            f"explains={bool_text(rail.delay_explains_residual)}"
        )
        print(
            "           Lineage   "
            f"source={rail.source}  "
            f"previous_diff={fmt(rail.previous_difference, 0, True).strip()}  "
            f"residual_diff={fmt(rail.residual_difference, 0, True).strip()}"
        )

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
    print("\nObserved one-second cycle counts and raw residuals")
    print("══════════════════════════════════════════════════")
    print("  cyc = published observed cycle count; raw = published last-second residual.")
    print("  NOTE reports current endpoint-delay verdicts for the displayed clocks.\n")

    header = [f"{'pps':>7}"]
    for name in selected:
        prefix = {"PPS": "p", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}[name]
        header += [f"{prefix + '_cyc':>13}", f"{prefix + '_raw':>9}"]
    header.append("NOTE")
    print("  ".join(header))
    print("  ".join("─" * len(field) for field in header))

    displayed = 0
    for row in rows:
        note = verdict_note(row, selected)
        has_raw_pathology = any(
            row.rails[name].published_residual is not None
            and abs(row.rails[name].published_residual) > gate
            for name in selected
        )
        if pathology_only and not has_raw_pathology and note == "ON_TIME":
            continue

        fields = [fmt(row.count, 7)]
        for name in selected:
            rail = row.rails[name]
            fields += [
                fmt(rail.observed, 13),
                fmt(rail.published_residual, 9, True),
            ]
        fields.append(note)
        print("  ".join(fields))
        if any(
            row.rails[name].published_residual is not None
            and abs(row.rails[name].published_residual) > 12
            for name in selected
        ):
            print_extended_row(row, selected)
        displayed += 1

    if pathology_only:
        print(f"\nDisplayed {displayed:,} exceptional rows of {len(rows):,}.")


if __name__ == "__main__":
    main(sys.argv)
