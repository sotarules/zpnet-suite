"""ZPNet raw_clock_candidates — streamed PhaseLedger/Delta clock court.

The report reads TIMEBASE in indexed ``timebase.pps_count`` order through a
named PostgreSQL cursor.  Each JSON payload is decoded, audited, printed, and
discarded, so memory use is independent of campaign length.

For each OCXO lane the report independently reconstructs both nanosecond clock
candidates:

* PhaseLedger: prior candidate + 1 second + reported PhaseLedger residual.
* Delta Cycles: prior candidate + exact raw-interval ratio, including the
  firmware's carried fractional-nanosecond remainder.

It then checks candidate-count conservation, highlights 100 ns tooth events,
and calls attention to phase suffix boundaries, discontinuities, malformed
candidate state, or disagreement that cannot be explained by the two residuals.
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


LANES: Tuple[str, ...] = ("OCXO1", "OCXO2")
LANE_KEYS = {"OCXO1": "ocxo1", "OCXO2": "ocxo2"}
NS_PER_SECOND = 1_000_000_000
DEFAULT_RESIDUAL_GATE_NS = 20.0
DEFAULT_DIFFERENCE_STEP_GATE_NS = 20
DEFAULT_CANDIDATE_GAP_GATE_NS = 100
DEFAULT_NEAR_TOOTH_NS = 8
DEFAULT_BATCH_SIZE = 16
DEFAULT_PAUSE_MS = 25
FLOAT_EPSILON_NS = 0.000_001


@dataclass
class Candidate:
    available: Optional[bool] = None
    continuity_valid: Optional[bool] = None
    status: str = "UNKNOWN"
    status_id: Optional[int] = None
    start_public_count: Optional[int] = None
    last_public_count: Optional[int] = None
    interval_count: Optional[int] = None
    ns: Optional[int] = None
    fractional_ns: Optional[float] = None
    residual_available: Optional[bool] = None
    residual_ns: Optional[int] = None
    residual_ns_exact: Optional[float] = None


@dataclass
class LaneState:
    previous_count: Optional[int] = None
    previous_phase_ns: Optional[int] = None
    previous_phase_suffix: Optional[int] = None
    previous_delta_ns: Optional[int] = None
    previous_delta_fractional_ns: Optional[float] = None
    previous_difference_ns: Optional[int] = None

    def reset(self) -> None:
        self.previous_count = None
        self.previous_phase_ns = None
        self.previous_phase_suffix = None
        self.previous_delta_ns = None
        self.previous_delta_fractional_ns = None
        self.previous_difference_ns = None


@dataclass
class LaneAudit:
    name: str
    published_source: str = "UNKNOWN"
    published_ns: Optional[int] = None
    clockface_valid: Optional[bool] = None
    phase: Candidate = field(default_factory=Candidate)
    delta: Candidate = field(default_factory=Candidate)
    reference_cycles: Optional[int] = None
    clock_cycles: Optional[int] = None
    reported_difference_ns: Optional[int] = None
    reported_residual_difference_ns_exact: Optional[float] = None

    phase_suffix_ns: Optional[int] = None
    phase_raw_suffix_step_ns: Optional[int] = None
    phase_wrap_direction: str = ""
    phase_deduced_ns: Optional[int] = None
    phase_reconstruction_error_ns: Optional[int] = None
    phase_residual_from_count_ns: Optional[int] = None
    phase_residual_error_ns: Optional[int] = None

    delta_formula_residual_ns_exact: Optional[float] = None
    delta_ratio_residual_ns_exact: Optional[float] = None
    delta_deduced_ns: Optional[int] = None
    delta_deduced_fractional_ns: Optional[float] = None
    delta_reconstruction_error_ns: Optional[int] = None
    delta_fractional_error_ns: Optional[float] = None
    delta_residual_formula_error_ns: Optional[float] = None

    computed_difference_ns: Optional[int] = None
    difference_field_error_ns: Optional[int] = None
    difference_step_ns: Optional[int] = None
    expected_difference_step_ns: Optional[int] = None
    difference_closure_error_ns: Optional[int] = None
    residual_difference_exact: Optional[float] = None
    residual_difference_field_error_ns: Optional[float] = None

    notes: list[str] = field(default_factory=list)


@dataclass
class AuditRow:
    count: int
    previous_count: Optional[int]
    count_delta: Optional[int]
    gap: bool
    recovery_boundary: bool
    disposition: str
    timeline_valid: Optional[bool]
    lanes: Dict[str, LaneAudit]


@dataclass
class Summary:
    rows_processed: int = 0
    lane_rows_displayed: int = 0
    phase_wraps: int = 0
    near_tooth_rows: int = 0
    tooth_alerts: int = 0
    reconstruction_failures: int = 0
    closure_failures: int = 0
    max_abs_residual_difference_ns: Dict[str, float] = field(
        default_factory=lambda: {name: 0.0 for name in LANES}
    )
    max_abs_difference_step_ns: Dict[str, int] = field(
        default_factory=lambda: {name: 0 for name in LANES}
    )


def d(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def i(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def f(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def b(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on", "nominal", "ready"}:
            return True
        if normalized in {"0", "false", "no", "off", "hold", "anomaly"}:
            return False
    return None


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = i(value)
        if parsed is not None:
            return parsed
    return None


def root(payload: Dict[str, Any]) -> Dict[str, Any]:
    inner = d(payload.get("payload"))
    return inner or payload


def fragment(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Return a persisted TIMEBASE fragment or an embedded MONITOR campaign row."""
    r = root(payload)
    direct = d(r.get("fragment")) or d(r.get("campaign_row"))
    if direct:
        return direct

    monitor_fragment = d(r.get("monitor_fragment"))
    embedded = d(monitor_fragment.get("campaign_row"))
    if embedded:
        return embedded

    clocks = d(r.get("clocks"))
    monitor_fragment = d(clocks.get("monitor_fragment"))
    embedded = d(monitor_fragment.get("campaign_row"))
    return embedded or r


def _payload_pps_count(payload: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    r = root(payload)
    return first_int(
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("campaign_seconds"),
        r.get("teensy_pps_vclock_count"),
        r.get("pps_count"),
    )


def _assert_campaign_indexed(cur: Any, campaign: str) -> None:
    cur.execute(
        """
        SELECT count(*) AS missing_count
        FROM timebase
        WHERE campaign = %s AND pps_count IS NULL
        """,
        (campaign,),
    )
    row = cur.fetchone()
    missing = int(row["missing_count"] if row else 0)
    if missing:
        raise RuntimeError(
            f"campaign {campaign!r} has {missing:,} rows with NULL timebase.pps_count; "
            "backfill the scalar identity before reporting"
        )


def iter_payloads(
    campaign: str,
    *,
    skip: int,
    limit: int,
    batch_size: int,
    pause_ms: int,
) -> Iterator[Tuple[int, int, Dict[str, Any]]]:
    """Yield ``(db_id, pps_count, payload)`` through a server-side cursor."""
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        check = conn.cursor()
        _assert_campaign_indexed(check, campaign)
        check.close()

        sql = """
            SELECT id, pps_count, payload
            FROM timebase
            WHERE campaign = %s
        """
        params: list[Any] = [campaign]
        if skip > 0:
            sql += " AND pps_count > %s"
            params.append(skip)
        sql += " ORDER BY pps_count ASC, id ASC"
        if limit > 0:
            sql += " LIMIT %s"
            params.append(limit)

        cur = conn.cursor(name="raw_clock_candidates_stream")
        cur.itersize = max(1, batch_size)
        cur.execute(sql, tuple(params))

        fetched_in_batch = 0
        for row in cur:
            payload = row["payload"]
            if isinstance(payload, str):
                payload = json.loads(payload)
            if not isinstance(payload, dict):
                continue

            db_count = int(row["pps_count"])
            frag = fragment(payload)
            payload_count = _payload_pps_count(payload, frag)
            if payload_count is not None and payload_count != db_count:
                raise ValueError(
                    "TIMEBASE relational/payload PPS mismatch: "
                    f"id={row['id']} db={db_count} payload={payload_count}"
                )

            yield int(row["id"]), db_count, payload

            fetched_in_batch += 1
            if fetched_in_batch >= max(1, batch_size):
                fetched_in_batch = 0
                if pause_ms > 0:
                    time.sleep(pause_ms / 1000.0)


def explicit_recovery_row(frag: Dict[str, Any]) -> bool:
    reason = str(frag.get("recover_reattach_reason") or "").strip().lower()
    return bool(
        reason not in {"", "idle", "none", "ok"}
        or b(frag.get("recover_degraded_active")) is True
        or b(frag.get("recover_degraded_science_hold")) is True
        or b(frag.get("recover_science_quarantine_active")) is True
        or b(frag.get("recover_transition_active")) is True
    )


def parse_candidate(obj: Dict[str, Any]) -> Candidate:
    return Candidate(
        available=b(obj.get("available")),
        continuity_valid=b(obj.get("continuity_valid")),
        status=str(obj.get("status") or "UNKNOWN").upper(),
        status_id=i(obj.get("status_id")),
        start_public_count=i(obj.get("start_public_count")),
        last_public_count=i(obj.get("last_public_count")),
        interval_count=i(obj.get("interval_count")),
        ns=i(obj.get("ns")),
        fractional_ns=f(obj.get("fractional_ns")),
        residual_available=b(obj.get("residual_available")),
        residual_ns=i(obj.get("residual_ns")),
        residual_ns_exact=f(obj.get("residual_ns_exact")),
    )


def round_positive_half_up(value: float) -> int:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"invalid positive clock increment {value!r}")
    return math.floor(value + 0.5)


def _append_candidate_health_notes(lane: LaneAudit, candidate: Candidate, label: str) -> None:
    if candidate.available is False:
        lane.notes.append(f"{label}_UNAVAILABLE")
    if candidate.continuity_valid is False:
        lane.notes.append(f"{label}_CONTINUITY_INVALID")
    if candidate.status not in {"ADVANCED", "SEEDED"}:
        lane.notes.append(f"{label}_STATUS={candidate.status}")
    if candidate.residual_available is False:
        lane.notes.append(f"{label}_RESIDUAL_UNAVAILABLE")


def build_lane(
    frag: Dict[str, Any],
    name: str,
    count: int,
    gap: bool,
    state: LaneState,
    *,
    residual_gate_ns: float,
    difference_step_gate_ns: int,
    candidate_gap_gate_ns: int,
    near_tooth_ns: int,
) -> LaneAudit:
    lane_obj = d(frag.get(LANE_KEYS[name]))
    candidates = d(lane_obj.get("clock_candidates"))
    phase = parse_candidate(d(candidates.get("phaseledger")))
    delta = parse_candidate(d(candidates.get("delta_cycles")))
    science = d(lane_obj.get("science"))

    lane = LaneAudit(
        name=name,
        published_source=str(candidates.get("published_source") or "UNKNOWN").upper(),
        published_ns=i(lane_obj.get("ns")),
        clockface_valid=b(lane_obj.get("clockface_valid")),
        phase=phase,
        delta=delta,
        reference_cycles=i(science.get("delta_raw_reference_interval_cycles")),
        clock_cycles=i(science.get("delta_raw_clock_interval_cycles")),
        reported_difference_ns=i(candidates.get("delta_cycles_minus_phaseledger_ns")),
        reported_residual_difference_ns_exact=f(
            candidates.get("delta_cycles_minus_phaseledger_residual_ns_exact")
        ),
    )

    if not candidates:
        lane.notes.append("NO_CLOCK_CANDIDATES")
    _append_candidate_health_notes(lane, phase, "PHASE")
    _append_candidate_health_notes(lane, delta, "DELTA")

    if lane.clockface_valid is False:
        lane.notes.append("CLOCKFACE_INVALID")
    if lane.published_source != "COUNTERLEDGER_PHASELEDGER":
        lane.notes.append(f"PUBLISHED_SOURCE={lane.published_source}")
    if lane.published_ns is not None and phase.ns is not None and lane.published_ns != phase.ns:
        lane.notes.append(f"PUBLISHED_PHASE_MISMATCH={lane.published_ns - phase.ns:+d}")

    if phase.last_public_count is not None and phase.last_public_count != count:
        lane.notes.append(f"PHASE_COUNT={phase.last_public_count}")
    if delta.last_public_count is not None and delta.last_public_count != count:
        lane.notes.append(f"DELTA_COUNT={delta.last_public_count}")

    if phase.start_public_count is not None and phase.interval_count is not None:
        expected = count - phase.start_public_count
        if phase.interval_count != expected:
            lane.notes.append(f"PHASE_POP={phase.interval_count}/{expected}")
    if delta.start_public_count is not None and delta.interval_count is not None:
        expected = count - delta.start_public_count
        if delta.interval_count != expected:
            lane.notes.append(f"DELTA_POP={delta.interval_count}/{expected}")

    if phase.ns is not None:
        lane.phase_suffix_ns = phase.ns % 100
        if lane.phase_suffix_ns <= near_tooth_ns or lane.phase_suffix_ns >= 100 - near_tooth_ns:
            lane.notes.append(f"NEAR_TOOTH={lane.phase_suffix_ns:02d}")

    if phase.ns is not None and delta.ns is not None:
        lane.computed_difference_ns = delta.ns - phase.ns
        if lane.reported_difference_ns is not None:
            lane.difference_field_error_ns = (
                lane.reported_difference_ns - lane.computed_difference_ns
            )
            if lane.difference_field_error_ns != 0:
                lane.notes.append(f"DIFF_FIELD_ERR={lane.difference_field_error_ns:+d}")
        if abs(lane.computed_difference_ns) >= candidate_gap_gate_ns:
            lane.notes.append(f"CANDIDATE_GAP={lane.computed_difference_ns:+d}")

    if phase.residual_ns_exact is not None and delta.residual_ns_exact is not None:
        lane.residual_difference_exact = delta.residual_ns_exact - phase.residual_ns_exact
        if abs(lane.residual_difference_exact) >= residual_gate_ns:
            lane.notes.append(f"RESIDUAL_SPLIT={lane.residual_difference_exact:+.3f}")
        if lane.reported_residual_difference_ns_exact is not None:
            lane.residual_difference_field_error_ns = (
                lane.reported_residual_difference_ns_exact
                - lane.residual_difference_exact
            )
            if abs(lane.residual_difference_field_error_ns) > FLOAT_EPSILON_NS:
                lane.notes.append(
                    "RES_DIFF_FIELD_ERR="
                    f"{lane.residual_difference_field_error_ns:+.6f}"
                )

    contiguous = (
        not gap
        and state.previous_count is not None
        and count == state.previous_count + 1
    )

    if contiguous and phase.ns is not None and state.previous_phase_ns is not None:
        phase_residual = phase.residual_ns
        if phase_residual is not None:
            lane.phase_deduced_ns = (
                state.previous_phase_ns + NS_PER_SECOND + phase_residual
            )
            lane.phase_reconstruction_error_ns = phase.ns - lane.phase_deduced_ns
            if lane.phase_reconstruction_error_ns != 0:
                lane.notes.append(
                    f"PHASE_RECON_ERR={lane.phase_reconstruction_error_ns:+d}"
                )
        lane.phase_residual_from_count_ns = (
            phase.ns - state.previous_phase_ns - NS_PER_SECOND
        )
        if phase_residual is not None:
            lane.phase_residual_error_ns = (
                phase_residual - lane.phase_residual_from_count_ns
            )
            if lane.phase_residual_error_ns != 0:
                lane.notes.append(
                    f"PHASE_RES_ERR={lane.phase_residual_error_ns:+d}"
                )

        if lane.phase_suffix_ns is not None and state.previous_phase_suffix is not None:
            lane.phase_raw_suffix_step_ns = (
                lane.phase_suffix_ns - state.previous_phase_suffix
            )
            if lane.phase_raw_suffix_step_ns <= -50:
                lane.phase_wrap_direction = "FWD"
                lane.notes.append(
                    f"PHASE_WRAP_FWD={state.previous_phase_suffix:02d}->{lane.phase_suffix_ns:02d}"
                )
            elif lane.phase_raw_suffix_step_ns >= 50:
                lane.phase_wrap_direction = "REV"
                lane.notes.append(
                    f"PHASE_WRAP_REV={state.previous_phase_suffix:02d}->{lane.phase_suffix_ns:02d}"
                )

    exact_delta_increment_ns: Optional[float] = None
    if (
        lane.reference_cycles is not None
        and lane.clock_cycles is not None
        and lane.reference_cycles > 0
        and lane.clock_cycles > 0
    ):
        exact_delta_increment_ns = (
            NS_PER_SECOND * lane.reference_cycles / lane.clock_cycles
        )
        lane.delta_ratio_residual_ns_exact = (
            exact_delta_increment_ns - NS_PER_SECOND
        )

    delta_increment_rounded: Optional[int] = None
    if (
        contiguous
        and delta.ns is not None
        and state.previous_delta_ns is not None
        and state.previous_delta_fractional_ns is not None
        and exact_delta_increment_ns is not None
    ):
        increment_with_carry = (
            state.previous_delta_fractional_ns + exact_delta_increment_ns
        )
        delta_increment_rounded = round_positive_half_up(increment_with_carry)
        lane.delta_deduced_ns = state.previous_delta_ns + delta_increment_rounded
        lane.delta_deduced_fractional_ns = (
            increment_with_carry - delta_increment_rounded
        )
        lane.delta_reconstruction_error_ns = delta.ns - lane.delta_deduced_ns
        if lane.delta_reconstruction_error_ns != 0:
            lane.notes.append(
                f"DELTA_RECON_ERR={lane.delta_reconstruction_error_ns:+d}"
            )
        if delta.fractional_ns is not None:
            lane.delta_fractional_error_ns = (
                delta.fractional_ns - lane.delta_deduced_fractional_ns
            )
            if abs(lane.delta_fractional_error_ns) > FLOAT_EPSILON_NS:
                lane.notes.append(
                    f"DELTA_FRAC_ERR={lane.delta_fractional_error_ns:+.6f}"
                )

    if (
        lane.reference_cycles is not None
        and lane.clock_cycles is not None
        and lane.reference_cycles > 0
        and lane.clock_cycles > 0
    ):
        lane.delta_formula_residual_ns_exact = (
            (lane.reference_cycles - lane.clock_cycles)
            * NS_PER_SECOND
            / lane.reference_cycles
        )
        if delta.residual_ns_exact is not None:
            lane.delta_residual_formula_error_ns = (
                delta.residual_ns_exact - lane.delta_formula_residual_ns_exact
            )
            if abs(lane.delta_residual_formula_error_ns) > FLOAT_EPSILON_NS:
                lane.notes.append(
                    "DELTA_RES_FORMULA_ERR="
                    f"{lane.delta_residual_formula_error_ns:+.6f}"
                )

    if (
        contiguous
        and lane.computed_difference_ns is not None
        and state.previous_difference_ns is not None
    ):
        lane.difference_step_ns = (
            lane.computed_difference_ns - state.previous_difference_ns
        )
        if abs(lane.difference_step_ns) >= difference_step_gate_ns:
            lane.notes.append(f"DIFF_STEP={lane.difference_step_ns:+d}")
        if 80 <= abs(lane.difference_step_ns) <= 120:
            lane.notes.append("ONE_TOOTH_STEP")

        if delta_increment_rounded is not None and phase.residual_ns is not None:
            phase_increment = NS_PER_SECOND + phase.residual_ns
            lane.expected_difference_step_ns = (
                delta_increment_rounded - phase_increment
            )
            lane.difference_closure_error_ns = (
                lane.difference_step_ns - lane.expected_difference_step_ns
            )
            if lane.difference_closure_error_ns != 0:
                lane.notes.append(
                    f"DIFF_CLOSURE_ERR={lane.difference_closure_error_ns:+d}"
                )

    if (
        lane.residual_difference_exact is not None
        and 80.0 <= abs(lane.residual_difference_exact) <= 120.0
    ):
        lane.notes.append("ONE_TOOTH_RESIDUAL")

    # Update state only after all deductions use the previous row.
    state.previous_count = count
    state.previous_phase_ns = phase.ns
    state.previous_phase_suffix = lane.phase_suffix_ns
    state.previous_delta_ns = delta.ns
    state.previous_delta_fractional_ns = delta.fractional_ns
    state.previous_difference_ns = lane.computed_difference_ns

    return lane


def build_row(
    payload: Dict[str, Any],
    db_count: int,
    previous_count: Optional[int],
    states: Dict[str, LaneState],
    *,
    residual_gate_ns: float,
    difference_step_gate_ns: int,
    candidate_gap_gate_ns: int,
    near_tooth_ns: int,
) -> AuditRow:
    frag = fragment(payload)
    delta = None if previous_count is None else db_count - previous_count
    gap = delta is not None and delta != 1
    recovery = bool(
        gap and delta is not None and delta > 1 and explicit_recovery_row(frag)
    )

    if gap:
        for state in states.values():
            state.reset()

    lanes = {
        name: build_lane(
            frag,
            name,
            db_count,
            gap,
            states[name],
            residual_gate_ns=residual_gate_ns,
            difference_step_gate_ns=difference_step_gate_ns,
            candidate_gap_gate_ns=candidate_gap_gate_ns,
            near_tooth_ns=near_tooth_ns,
        )
        for name in LANES
    }

    return AuditRow(
        count=db_count,
        previous_count=previous_count,
        count_delta=delta,
        gap=gap,
        recovery_boundary=recovery,
        disposition=str(frag.get("candidate_disposition") or "ACCEPT").upper(),
        timeline_valid=b(frag.get("timeline_valid")),
        lanes=lanes,
    )


def row_notes(row: AuditRow, lane: LaneAudit) -> list[str]:
    notes: list[str] = []
    if row.recovery_boundary:
        skipped = row.count_delta - 1 if row.count_delta is not None else 0
        notes.append(f"RECOVER_GAP={skipped}")
    elif row.gap:
        notes.append(f"NON_ADJACENT={row.count_delta}")
    if row.disposition != "ACCEPT":
        notes.append(row.disposition)
    if row.timeline_valid is False:
        notes.append("TIMELINE_INVALID")
    notes.extend(lane.notes)
    return notes


def fmt_int(value: Optional[int], width: int, *, signed: bool = False) -> str:
    if value is None:
        text = "---"
    else:
        text = f"{value:+,d}" if signed else f"{value:,d}"
    return f"{text:>{width}}"


def fmt_float(
    value: Optional[float], width: int, *, signed: bool = False, decimals: int = 3
) -> str:
    if value is None:
        text = "---"
    else:
        text = f"{value:+,.{decimals}f}" if signed else f"{value:,.{decimals}f}"
    return f"{text:>{width}}"


def fmt_suffix(value: Optional[int]) -> str:
    return " ---" if value is None else f" {value:02d}ns"


def parse(
    argv: Sequence[str],
) -> tuple[str, int, int, Optional[str], bool, float, int, int, int, int, int]:
    if len(argv) < 2:
        raise SystemExit(
            "Usage: raw_clock_candidates CAMPAIGN [limit] [clock] [--skip N] "
            "[--pathology-only] [--residual-gate-ns N] "
            "[--difference-step-gate-ns N] [--candidate-gap-gate-ns N] "
            "[--near-tooth-ns N] [--batch-size N] [--pause-ms N]"
        )

    campaign = argv[1]
    limit = 0
    skip = 0
    clock: Optional[str] = None
    pathology_only = False
    residual_gate_ns = DEFAULT_RESIDUAL_GATE_NS
    difference_step_gate_ns = DEFAULT_DIFFERENCE_STEP_GATE_NS
    candidate_gap_gate_ns = DEFAULT_CANDIDATE_GAP_GATE_NS
    near_tooth_ns = DEFAULT_NEAR_TOOTH_NS
    batch_size = DEFAULT_BATCH_SIZE
    pause_ms = DEFAULT_PAUSE_MS
    positional: list[str] = []

    value_options = {
        "--skip",
        "--clock",
        "--limit",
        "--residual-gate-ns",
        "--pathology-gate",  # compatibility alias
        "--difference-step-gate-ns",
        "--candidate-gap-gate-ns",
        "--near-tooth-ns",
        "--batch-size",
        "--pause-ms",
    }

    idx = 2
    while idx < len(argv):
        arg = argv[idx]
        if arg == "--pathology-only":
            pathology_only = True
            idx += 1
            continue

        if arg in value_options:
            if idx + 1 >= len(argv):
                raise SystemExit(f"{arg} requires a value")
            key, value = arg, argv[idx + 1]
            idx += 2
        elif arg.startswith("--") and "=" in arg:
            key, value = arg.split("=", 1)
            if key not in value_options:
                raise SystemExit(f"unknown option {key}")
            idx += 1
        elif arg in {"--align-ocxo", "--delay-pps-vclock", "--slip"}:
            # Retired raw_cycles compatibility switches: accepted and ignored.
            idx += 1
            continue
        else:
            positional.append(arg)
            idx += 1
            continue

        if key == "--skip":
            skip = int(value)
        elif key == "--clock":
            clock = value.upper()
        elif key == "--limit":
            limit = int(value)
        elif key in {"--residual-gate-ns", "--pathology-gate"}:
            residual_gate_ns = float(value)
        elif key == "--difference-step-gate-ns":
            difference_step_gate_ns = int(value)
        elif key == "--candidate-gap-gate-ns":
            candidate_gap_gate_ns = int(value)
        elif key == "--near-tooth-ns":
            near_tooth_ns = int(value)
        elif key == "--batch-size":
            batch_size = int(value)
        elif key == "--pause-ms":
            pause_ms = int(value)

    for arg in positional:
        if arg.upper() in LANES:
            clock = arg.upper()
        else:
            limit = int(arg)

    if min(skip, limit, pause_ms, difference_step_gate_ns, candidate_gap_gate_ns) < 0:
        raise SystemExit("skip/limit/pause/gates must be nonnegative")
    if residual_gate_ns < 0.0:
        raise SystemExit("residual gate must be nonnegative")
    if not 0 <= near_tooth_ns < 50:
        raise SystemExit("near-tooth-ns must be in 0..49")
    if batch_size <= 0:
        raise SystemExit("batch-size must be positive")
    if clock is not None and clock not in LANES:
        raise SystemExit("clock must be OCXO1 or OCXO2")

    return (
        campaign,
        limit,
        skip,
        clock,
        pathology_only,
        residual_gate_ns,
        difference_step_gate_ns,
        candidate_gap_gate_ns,
        near_tooth_ns,
        batch_size,
        pause_ms,
    )


def note_is_attention(note: str) -> bool:
    return note != "OK"


def update_summary(summary: Summary, lane: LaneAudit, displayed: bool) -> None:
    if displayed:
        summary.lane_rows_displayed += 1
    if lane.phase_wrap_direction:
        summary.phase_wraps += 1
    if any(note.startswith("NEAR_TOOTH=") for note in lane.notes):
        summary.near_tooth_rows += 1
    if "ONE_TOOTH_STEP" in lane.notes or "ONE_TOOTH_RESIDUAL" in lane.notes:
        summary.tooth_alerts += 1
    if any("RECON_ERR" in note or "FRAC_ERR" in note for note in lane.notes):
        summary.reconstruction_failures += 1
    if any("CLOSURE_ERR" in note for note in lane.notes):
        summary.closure_failures += 1
    if lane.residual_difference_exact is not None:
        summary.max_abs_residual_difference_ns[lane.name] = max(
            summary.max_abs_residual_difference_ns[lane.name],
            abs(lane.residual_difference_exact),
        )
    if lane.difference_step_ns is not None:
        summary.max_abs_difference_step_ns[lane.name] = max(
            summary.max_abs_difference_step_ns[lane.name],
            abs(lane.difference_step_ns),
        )


def main(argv: Sequence[str]) -> None:
    (
        campaign,
        limit,
        skip,
        clock,
        pathology_only,
        residual_gate_ns,
        difference_step_gate_ns,
        candidate_gap_gate_ns,
        near_tooth_ns,
        batch_size,
        pause_ms,
    ) = parse(argv)
    selected = (clock,) if clock else LANES

    print(
        f"Campaign: {campaign}  view={clock or 'ALL'}  "
        f"server_batch={batch_size} pause_ms={pause_ms}  "
        f"res_gate={residual_gate_ns:g}ns step_gate={difference_step_gate_ns}ns"
    )
    print(
        "Deduction: Phase=prior+1s+phase_residual; "
        "Delta=prior+round(prior_fraction+1s*reference_cycles/clock_cycles)."
    )

    header = [
        f"{'pps':>7}",
        f"{'clk':>3}",
        f"{'suf':>5}",
        f"{'p_res':>7}",
        f"{'d_res':>9}",
        f"{'ratio':>9}",
        f"{'phase_ns':>15}",
        f"{'phase_calc':>15}",
        f"{'delta_ns':>15}",
        f"{'delta_calc':>15}",
        f"{'D-P':>7}",
        f"{'dD':>6}",
        "NOTE",
    ]
    print("  ".join(header))
    print("  ".join("─" * len(field) for field in header))

    previous_count: Optional[int] = None
    states = {name: LaneState() for name in LANES}
    summary = Summary()

    for _db_id, db_count, payload in iter_payloads(
        campaign,
        skip=skip,
        limit=limit,
        batch_size=batch_size,
        pause_ms=pause_ms,
    ):
        row = build_row(
            payload,
            db_count,
            previous_count,
            states,
            residual_gate_ns=residual_gate_ns,
            difference_step_gate_ns=difference_step_gate_ns,
            candidate_gap_gate_ns=candidate_gap_gate_ns,
            near_tooth_ns=near_tooth_ns,
        )
        previous_count = db_count
        summary.rows_processed += 1

        for name in selected:
            lane = row.lanes[name]
            notes = row_notes(row, lane)
            note = " | ".join(notes) if notes else "OK"
            displayed = not pathology_only or note_is_attention(note)
            update_summary(summary, lane, displayed)
            if not displayed:
                continue

            fields = [
                fmt_int(row.count, 7),
                f"{('O1' if name == 'OCXO1' else 'O2'):>3}",
                fmt_suffix(lane.phase_suffix_ns),
                fmt_int(lane.phase.residual_ns, 7, signed=True),
                fmt_float(lane.delta.residual_ns_exact, 9, signed=True),
                fmt_float(lane.delta_ratio_residual_ns_exact, 9, signed=True),
                fmt_int(lane.phase.ns, 15),
                fmt_int(lane.phase_deduced_ns, 15),
                fmt_int(lane.delta.ns, 15),
                fmt_int(lane.delta_deduced_ns, 15),
                fmt_int(lane.computed_difference_ns, 7, signed=True),
                fmt_int(lane.difference_step_ns, 6, signed=True),
                note,
            ]
            print("  ".join(fields))

    print(
        f"\nProcessed {summary.rows_processed:,} TIMEBASE rows; "
        f"displayed {summary.lane_rows_displayed:,} lane rows."
    )
    print(
        "Attention totals: "
        f"wraps={summary.phase_wraps:,} "
        f"near_tooth={summary.near_tooth_rows:,} "
        f"one_tooth_alerts={summary.tooth_alerts:,} "
        f"reconstruction_failures={summary.reconstruction_failures:,} "
        f"closure_failures={summary.closure_failures:,}"
    )
    for name in selected:
        print(
            f"{name}: max|delta_res-phase_res|="
            f"{summary.max_abs_residual_difference_ns[name]:.6f} ns; "
            f"max|change in Delta-Phase count|="
            f"{summary.max_abs_difference_step_ns[name]} ns"
        )


if __name__ == "__main__":
    main(sys.argv)