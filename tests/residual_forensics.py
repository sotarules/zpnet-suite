"""ZPNet residual_forensics — single-line explanations for raw-cycle excursions.

For PPS, VCLOCK, OCXO1, and OCXO2 the report intentionally shows only the
producer-published observed cycle count and producer-published last-second raw
residual.  Rows are emitted only when at least one displayed raw residual exceeds the configured
threshold.  A single VERDICT column summarizes the strongest causal explanation from
the embedded ISR-delay testimony; no multi-line expansion is printed.  Raw endpoints remain immutable; no projection, repair,
or cross-rail subtraction is performed.  The report treats a RECOVER gap as a
non-adjacent timeline seam: it preserves the producer's observed interval, but
suppresses current-minus-previous residual and lineage comparisons for that row.
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
DEFAULT_GATE_CYCLES = 20
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

    # First-instruction DWT captured at the current event's ISR entry.
    dwt_at_edge: Optional[int] = None

    # Arrival-context testimony captured at ISR entry.
    blocker: str = "NONE"
    blocker_wall_cycles: Optional[int] = None
    lower_context_active_mask: Optional[int] = None
    lower_continuation_active: Optional[bool] = None
    lower_qtimer1_active: Optional[bool] = None
    pending_mask_at_entry: Optional[int] = None
    preempted_after_entry: Optional[bool] = None
    spinidle_running: Optional[bool] = None
    spinidle_age_cycles: Optional[int] = None


@dataclass
class Row:
    count: Optional[int]
    disposition: str
    timeline_valid: Optional[bool]
    rails: Dict[str, Rail]
    issues: List[str]
    explanations: List[str]
    previous_count: Optional[int] = None
    count_delta: Optional[int] = None
    gap_from_previous: bool = False
    recovery_boundary: bool = False


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


def signed_dwt_delta(later: int, earlier: int) -> int:
    """Return later-earlier as an unambiguous signed modulo-2^32 DWT delta."""
    value = (later - earlier) & 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def record_root(record: Dict[str, Any]) -> Dict[str, Any]:
    return d(record.get("payload")) or record


def fragment(record: Dict[str, Any]) -> Dict[str, Any]:
    root = record_root(record)
    frag = d(root.get("fragment"))
    return frag or root


def forensics(record: Dict[str, Any], frag: Dict[str, Any]) -> Dict[str, Any]:
    root = record_root(record)
    return d(root.get("forensics")) or d(frag.get("forensics"))


def lane_forensics(forensic: Dict[str, Any], rail: str) -> Dict[str, Any]:
    """Return the per-lane lossless forensic object for arrival-context testimony."""
    if rail == "PPS":
        return d(forensic.get("pps_vclock_edge"))
    if rail == "VCLOCK":
        return d(d(forensic.get("vclock")).get("forensics"))
    return d(d(forensic.get(RAIL_KEYS[rail])).get("forensics"))


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



def recovery_status(frag: Dict[str, Any]) -> Dict[str, Optional[bool]]:
    """Extract explicit RECOVER testimony from the TIMEBASE fragment."""
    return {
        "degraded_active": b(frag.get("recover_degraded_active")),
        "degraded_science_hold": b(frag.get("recover_degraded_science_hold")),
        "science_quarantine_active": b(frag.get("recover_science_quarantine_active")),
        "transition_active": b(frag.get("recover_transition_active")),
        "timeline_ready": b(frag.get("recover_timeline_ready")),
        "clockface_ready": b(frag.get("recover_clockface_ready")),
        "science_ready": b(frag.get("recover_science_ready")),
    }


def explicit_recovery_row(frag: Dict[str, Any]) -> bool:
    """Return True only when the row itself carries RECOVER lifecycle evidence."""
    status = recovery_status(frag)
    reason = str(frag.get("recover_reattach_reason") or "").strip().lower()
    return bool(
        reason not in {"", "idle", "none"}
        or status["degraded_active"] is True
        or status["degraded_science_hold"] is True
        or status["science_quarantine_active"] is True
        or status["transition_active"] is True
        or status["timeline_ready"] is True
        or status["clockface_ready"] is True
        or status["science_ready"] is True
    )


def collect(records: Sequence[Dict[str, Any]]) -> List[Row]:
    rows: List[Row] = []
    previous: Dict[str, Optional[int]] = {name: None for name in RAILS}
    previous_count: Optional[int] = None
    delay_prefix = {"PPS": "pps", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}

    for record in records:
        frag = fragment(record)
        count = count_of(record, frag)
        count_delta = (count - previous_count) if count is not None and previous_count is not None else None
        gap_from_previous = count_delta is not None and count_delta != 1
        recovery_boundary = bool(gap_from_previous and count_delta is not None and count_delta > 1
                                 and explicit_recovery_row(frag))
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
            computed_previous = previous[name] if not gap_from_previous else None
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

            lane_f = lane_forensics(forensic, name)
            arrival = d(lane_f.get("arrival"))

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
                dwt_at_edge=first_int(
                    lane_f.get("dwt_event_from_isr_entry_raw"),
                    lane_f.get("dwt_original_at_event"),
                    forensic.get(f"{prefix}_raw"),
                ),
                blocker=text(arrival.get("blocker"), "NONE"),
                blocker_wall_cycles=i(arrival.get("blocker_wall_cycles")),
                lower_context_active_mask=i(arrival.get("lower_context_active_mask")),
                lower_continuation_active=b(arrival.get("lower_continuation_active")),
                lower_qtimer1_active=b(arrival.get("lower_qtimer1_active")),
                pending_mask_at_entry=i(arrival.get("pending_mask_at_entry")),
                preempted_after_entry=b(arrival.get("preempted_after_entry")),
                spinidle_running=b(arrival.get("spinidle_running")),
                spinidle_age_cycles=i(arrival.get("spinidle_age_cycles")),
            )
            previous[name] = observed

        rows.append(Row(
            count=count,
            disposition=str(frag.get("candidate_disposition") or "ACCEPT").upper(),
            timeline_valid=b(frag.get("timeline_valid")),
            rails=rails,
            issues=[],
            explanations=[],
            previous_count=previous_count,
            count_delta=count_delta,
            gap_from_previous=gap_from_previous,
            recovery_boundary=recovery_boundary,
        ))
        previous_count = count
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
    if row.gap_from_previous and not row.recovery_boundary:
        issues.append(
            f"non-adjacent PPS identity {row.previous_count} -> {row.count} "
            f"(delta={row.count_delta}); residual comparison suppressed"
        )
    for name in selected:
        rail = row.rails[name]
        if rail.observed is None:
            issues.append(f"{name}: observed interval missing")
            continue
        if not PLAUSIBLE_MIN <= rail.observed <= PLAUSIBLE_MAX:
            issues.append(f"{name}: implausible observed interval {rail.observed:,d}")
        if rail.valid is False:
            issues.append(f"{name}: producer marked raw-cycle sample invalid")

        if not row.gap_from_previous and rail.computed_residual is not None and abs(rail.computed_residual) > gate:
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

        if not row.gap_from_previous and rail.previous_difference not in (None, 0):
            issues.append(
                f"{name}: published previous differs from displayed previous by "
                f"{rail.previous_difference:+,d} cycles"
            )
        if not row.gap_from_previous and rail.residual_difference not in (None, 0):
            issues.append(
                f"{name}: published residual differs from computed residual by "
                f"{rail.residual_difference:+,d} cycles"
            )
        if not row.gap_from_previous and rail.residual_after_difference not in (None, 0):
            issues.append(
                f"{name}: firmware normalized residual differs from report math by "
                f"{rail.residual_after_difference:+,d} cycles"
            )

    row.explanations = explanations
    return issues


def fmt(value: Optional[int], width: int, signed: bool = False) -> str:
    text = "---" if value is None else (f"{value:+,d}" if signed else f"{value:,d}")
    return f"{text:>{width}}"


def delay_verdict_is_exceptional(rail: Rail) -> bool:
    """Return True only when endpoint testimony asserts an abnormal verdict.

    UNKNOWN with no valid delay evidence means the producer could not classify
    that endpoint.  It is unavailable testimony, not evidence of a pathology.
    """
    return rail.delay_status not in {"ON_TIME", "UNKNOWN"}


def _cause_label(value: str) -> str:
    aliases = {
        "VCLOCK_TIMEPOP": "VCLOCK ISR",
        "QTIMER1": "VCLOCK ISR",
        "PPS": "PPS ISR",
        "VCLOCK": "VCLOCK ISR",
        "OCXO1": "OCXO1 ISR",
        "OCXO2": "OCXO2 ISR",
        "CONTINUATION": "CONTINUATION ISR",
        "MASKING_OR_UNKNOWN_CPU": "CPU MASKING",
        "MULTIPLE_ISR": "MULTIPLE ISRS",
    }
    return aliases.get(value, value.replace("_", " "))


def rail_excursion_verdict(name: str, rail: Rail, gate: int) -> str:
    """Explain one exceptional raw residual using producer ISR-delay testimony."""
    residual = rail.published_residual
    if residual is None or abs(residual) <= gate:
        return ""

    normalized = rail.residual_after_delay_cycles
    explains = (
        rail.residual_delay_valid is True
        and rail.residual_delay_cycles is not None
        and rail.residual_delay_by not in {"NONE", "UNKNOWN"}
        and normalized is not None
        and (
            rail.delay_explains_residual is True
            or (
                rail.delay_explains_residual is None
                and abs(normalized) <= EXPLANATION_GATE_CYCLES
            )
        )
    )
    if explains:
        cause = _cause_label(rail.residual_delay_by)
        # A negative ISR contribution generally means the previous endpoint was
        # delayed, lengthening the previous interval and shrinking this residual.
        if rail.residual_delay_cycles < 0:
            return f"{name}: PREVIOUS EDGE DELAYED BY {cause}"
        return f"{name}: DELAYED BY {cause}"

    if rail.delay_status == "DELAYED" and rail.delay_by not in {"NONE", "UNKNOWN"}:
        cause = _cause_label(rail.delay_by)
        return f"{name}: DELAYED BY {cause}"

    # Arrival evidence is an alibi, not automatically a cause.  Surface the most
    # useful context without pretending it explains the residual.
    if rail.spinidle_running is True:
        return f"{name}: NO REASONABLE EXPLANATION (CPU IN SPINIDLE)"
    if rail.lower_continuation_active is True:
        return f"{name}: NO REASONABLE EXPLANATION (PREEMPTED CONTINUATION)"
    if rail.lower_qtimer1_active is True:
        return f"{name}: NO REASONABLE EXPLANATION (PREEMPTED QTIMER1)"
    if rail.blocker not in {"NONE", "UNKNOWN"}:
        return f"{name}: POSSIBLY BLOCKED BY {_cause_label(rail.blocker)}"
    if rail.pending_mask_at_entry not in {None, 0}:
        return f"{name}: NO REASONABLE EXPLANATION (INTERRUPT BACKLOG PRESENT)"
    return f"{name}: NO REASONABLE EXPLANATION"


def verdict_note(row: Row, selected: Sequence[str], gate: int) -> str:
    """Return one compact, row-wide verdict for only the rails over the gate."""
    if row.recovery_boundary:
        skipped = (row.count_delta - 1) if row.count_delta is not None else None
        return f"RECOVER GAP ({skipped} skipped); residuals suppressed"
    if row.gap_from_previous:
        return f"NON-ADJACENT GAP delta={row.count_delta}; residuals suppressed"
    notes = [rail_excursion_verdict(name, row.rails[name], gate) for name in selected]
    notes = [note for note in notes if note]
    return " | ".join(notes) if notes else "NO REASONABLE EXPLANATION"


def collision_crosscheck(row: Row, victim_name: str) -> Optional[str]:
    """Crosscheck a claimed OCXO-on-OCXO delay against captured ISR-entry DWTs.

    The delayed endpoint's captured DWT already includes the service delay.  For
    a genuine same-epoch collision, the signed service-entry separation should
    therefore agree with the victim's positive endpoint delay within the
    producer's stated uncertainty envelope.
    """
    victim = row.rails[victim_name]
    cause_name = victim.delay_by
    if victim.delay_status != "DELAYED" or cause_name not in {"OCXO1", "OCXO2"}:
        return None

    cause = row.rails[cause_name]
    if victim.dwt_at_edge is None or cause.dwt_at_edge is None:
        return "DWT crosscheck unavailable"

    signed_sep = signed_dwt_delta(victim.dwt_at_edge, cause.dwt_at_edge)
    abs_sep = abs(signed_sep)

    expected = victim.delay_cycles if victim.delay_cycles_valid is True else None
    uncertainty = victim.delay_uncertainty_cycles

    plausible: Optional[bool] = None
    mismatch: Optional[int] = None
    if expected is not None:
        mismatch = signed_sep - expected
        if uncertainty is not None:
            plausible = signed_sep >= 0 and abs(mismatch) <= uncertainty

    parts = [
        f"victim_dwt={victim.dwt_at_edge:,d}",
        f"cause_dwt={cause.dwt_at_edge:,d}",
        f"service_sep={signed_sep:+,d}",
        f"abs={abs_sep:,d}",
    ]
    if expected is not None:
        parts.append(f"expected_delay={expected:+,d}")
    if mismatch is not None:
        parts.append(f"mismatch={mismatch:+,d}")
    if uncertainty is not None:
        parts.append(f"tolerance={uncertainty:,d}")
    parts.append(f"plausible={bool_text(plausible)}")
    return "  ".join(parts)


def bool_text(value: Optional[bool]) -> str:
    if value is True:
        return "YES"
    if value is False:
        return "NO"
    return "UNKNOWN"


def print_extended_row(row: Row, selected: Sequence[str], gate: int) -> None:
    """Print focused forensic testimony beneath a genuinely exceptional row.

    Residual testimony is expanded only when it exceeds the configured pathology
    gate.  Endpoint testimony is expanded only for an affirmative abnormal
    verdict.  UNKNOWN without evidence is intentionally quiet.
    """
    if row.gap_from_previous:
        return

    interesting = [
        name for name in selected
        if (
            row.rails[name].published_residual is not None
            and abs(row.rails[name].published_residual) > gate
        )
        or delay_verdict_is_exceptional(row.rails[name])
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
        crosscheck = collision_crosscheck(row, name)
        if crosscheck is not None:
            print(f"           Collision {crosscheck}")
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
        raise SystemExit("Usage: residual_forensics CAMPAIGN [limit] [clock] [--skip N] [--threshold N]")
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
        elif arg in {"--threshold", "--pathology-gate"}:
            gate = int(argv[idx + 1]); idx += 2
        elif arg.startswith("--threshold=") or arg.startswith("--pathology-gate="):
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
    if gate < 0:
        raise SystemExit("threshold must be zero or greater")
    return campaign, limit, skip, clock, pathology_only, gate


def main(argv: Sequence[str]) -> None:
    campaign, limit, skip, clock, pathology_only, gate = parse(argv)
    records = fetch(campaign, skip, limit)
    rows = collect(records)
    selected = (clock,) if clock else RAILS
    for row in rows:
        row.issues = classify(row, selected, gate)

    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={clock or 'ALL'})")
    print("\nObserved one-second cycle excursions with ISR verdicts")
    print("══════════════════════════════════════════════════")
    print("  cyc = published observed cycle count; raw = published last-second residual.")
    print(f"  Rows shown only when |raw residual| > {gate:,d} cycles.\n")

    header = [f"{'pps':>7}"]
    for name in selected:
        prefix = {"PPS": "p", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}[name]
        header += [f"{prefix + '_cyc':>13}", f"{prefix + '_raw':>9}"]
    header.append("VERDICT")
    print("  ".join(header))
    print("  ".join("─" * len(field) for field in header))

    displayed = 0
    for row in rows:
        note = verdict_note(row, selected, gate)
        has_raw_pathology = (not row.gap_from_previous) and any(
            row.rails[name].published_residual is not None
            and abs(row.rails[name].published_residual) > gate
            for name in selected
        )
        if not has_raw_pathology:
            continue

        fields = [fmt(row.count, 7)]
        for name in selected:
            rail = row.rails[name]
            fields += [
                fmt(rail.observed, 13),
                fmt(None if row.gap_from_previous else rail.published_residual, 9, True),
            ]
        fields.append(note)
        print("  ".join(fields))
        displayed += 1

    print(f"\nDisplayed {displayed:,} rows over {gate:,d} cycles of {len(rows):,} total.")


if __name__ == "__main__":
    main(sys.argv)