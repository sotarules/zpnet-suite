"""
ZPNet Raw Cycles — observed-edge one-second cycle audit.

Reads TIMEBASE rows for a campaign and reports the one-second DWT-cycle facts
authored by the current observed-edge firmware.

The report deliberately ignores TIMEBASE_FORENSICS compatibility fields such as
*_raw, *_obs, *_fl, *_ema, *_used, and *_pub.  Those fields preserve an older
wire shape and may legally contain zero.  They are not current interval truth.

Current authority:

    v_cyc      VCLOCK/PPS reference interval from
               fragment.dwt.cycles_between_pps_vclock.

    o1_cyc     OCXO1 observed-edge interval from
               fragment.ocxo1.science.delta_raw_clock_interval_cycles.

    o2_cyc     OCXO2 observed-edge interval from
               fragment.ocxo2.science.delta_raw_clock_interval_cycles.

    *_d        first difference of that lane's observed interval:
               interval[n] - interval[n-1].

    o1-v/o2-v observed cycle residual against the reference interval carried
               by the same lane science record.  A positive cycle residual
               means the OCXO interval consumed more DWT cycles than VCLOCK.

    *_ns       science fast residual in nanoseconds.  TIMEBASE defines positive
               as clock fast, so its sign is intentionally opposite the cycle
               residual for the same interval.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> --clock OCXO2 [limit]
    python raw_cycles.py <campaign_name> --limit 300 --clock VCLOCK
    python raw_cycles.py <campaign_name> --pathology-only
    python raw_cycles.py <campaign_name> --pathology-gate 500

Compatibility:
    --align-ocxo, --delay-pps-vclock, and --slip are accepted but no longer
    transform the data.  Current TIMEBASE science carries the reference interval
    paired with each observed OCXO interval, so row shifting would now destroy
    the published relationship.
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


CLOCKS: Tuple[str, ...] = ("VCLOCK", "OCXO1", "OCXO2")
LANE_KEYS = {"OCXO1": "ocxo1", "OCXO2": "ocxo2"}

PATHOLOGY_DEFAULT_GATE_CYCLES = 500
EXPECTED_EDGE_SPECIES = "OBSERVED_DWT_EDGE"
EXPECTED_FREQUENCY_SOURCE = "DELTA_OBSERVED_DWT_EDGE"


class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, value: float) -> None:
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (value - self.mean)
        self.min_val = min(self.min_val, value)
        self.max_val = max(self.max_val, value)

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


@dataclass
class LaneRow:
    interval_cycles: Optional[int] = None
    interval_delta_cycles: Optional[int] = None
    reference_cycles: Optional[int] = None
    cycle_residual: Optional[int] = None
    fast_residual_ns: Optional[float] = None
    ppb_1s: Optional[float] = None
    science_valid: Optional[bool] = None
    delta_raw_valid: Optional[bool] = None
    edge_species: Optional[str] = None
    frequency_source: Optional[str] = None
    reported_minus_observed_ns: Optional[int] = None
    source: str = "MISSING"


@dataclass
class ReportRow:
    pps_count: Optional[int]
    campaign_seconds: Optional[int]
    reference_cycles: Optional[int]
    reference_delta_cycles: Optional[int]
    reference_nominal_residual_cycles: Optional[int]
    pps_vclock_phase_cycles: Optional[int]
    candidate_disposition: Optional[str]
    candidate_reason: Optional[str]
    timeline_valid: Optional[bool]
    lanes: Dict[str, LaneRow] = field(default_factory=dict)
    issues: List[str] = field(default_factory=list)


@dataclass
class Options:
    campaign: str
    limit: int = 0
    clock: Optional[str] = None
    pathology_only: bool = False
    pathology_gate: int = PATHOLOGY_DEFAULT_GATE_CYCLES
    compatibility_view_requested: bool = False


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
                NULLIF(payload->'fragment'->>'campaign_seconds', '')::bigint,
                NULLIF(payload->'payload'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'campaign_seconds', '')::bigint
            ) ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            result.append(payload)
    return result


def _root(record: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(record, dict):
        return {}
    payload = record.get("payload")
    return payload if isinstance(payload, dict) else record


def _fragment(record: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(record)
    fragment = root.get("fragment")
    if isinstance(fragment, dict):
        return fragment
    if root.get("schema") == "TIMEBASE_FRAGMENT_V4":
        return root
    return {}


def _dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _nested(obj: Dict[str, Any], *path: str) -> Any:
    value: Any = obj
    for key in path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _as_int(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _as_float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _as_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    if isinstance(value, str):
        folded = value.strip().lower()
        if folded in {"true", "yes", "1"}:
            return True
        if folded in {"false", "no", "0"}:
            return False
    return None


def _as_str(value: Any) -> Optional[str]:
    if value is None:
        return None
    result = str(value)
    return result if result else None


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = _as_int(value)
        if parsed is not None:
            return parsed
    return None


def _first_float(*values: Any) -> Optional[float]:
    for value in values:
        parsed = _as_float(value)
        if parsed is not None:
            return parsed
    return None


def _first_bool(*values: Any) -> Optional[bool]:
    for value in values:
        parsed = _as_bool(value)
        if parsed is not None:
            return parsed
    return None


def _first_str(*values: Any) -> Optional[str]:
    for value in values:
        parsed = _as_str(value)
        if parsed is not None:
            return parsed
    return None


def _fmt_int(value: Optional[int], width: int, signed: bool = False) -> str:
    if value is None:
        text = "---"
    else:
        text = f"{value:+,d}" if signed else f"{value:,d}"
    return f"{text:>{width}s}"


def _fmt_float(
    value: Optional[float],
    width: int,
    decimals: int,
    signed: bool = False,
) -> str:
    if value is None:
        text = "---"
    else:
        text = (
            f"{value:+,.{decimals}f}"
            if signed
            else f"{value:,.{decimals}f}"
        )
    return f"{text:>{width}s}"


def _update(stats: Welford, value: Optional[float]) -> None:
    if value is not None:
        stats.update(float(value))


def _print_stat(label: str, stats: Welford, unit: str) -> None:
    if stats.n == 0:
        print(f"  {label:<39s} no samples")
        return
    print(
        f"  {label:<39s} "
        f"n={stats.n:>6,d}  "
        f"mean={stats.mean:>12,.3f}  "
        f"sd={stats.stddev:>10,.3f}  "
        f"min={stats.min_val:>12,.3f}  "
        f"max={stats.max_val:>12,.3f}  {unit}"
    )


def normalize_clock(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    folded = value.strip().upper()
    aliases = {
        "V": "VCLOCK",
        "VC": "VCLOCK",
        "O1": "OCXO1",
        "O2": "OCXO2",
    }
    folded = aliases.get(folded, folded)
    if folded not in CLOCKS:
        raise SystemExit("clock must be VCLOCK, OCXO1, or OCXO2")
    return folded


def selected_lanes(clock: Optional[str]) -> Tuple[str, ...]:
    if clock in ("OCXO1", "OCXO2"):
        return (clock,)
    if clock == "VCLOCK":
        return ()
    return ("OCXO1", "OCXO2")


def _pps_count(root: Dict[str, Any], fragment: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        fragment.get("pps_count"),
        fragment.get("teensy_pps_vclock_count"),
        fragment.get("teensy_pps_count"),
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        root.get("teensy_pps_count"),
        fragment.get("campaign_seconds"),
    )


def _reference_cycles(fragment: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested(fragment, "dwt", "cycles_between_pps_vclock"),
        fragment.get("dwt_cycles_between_pps_vclock"),
        _nested(fragment, "pps", "dwt_cycles_between_edges"),
    )


def _phase_cycles(root: Dict[str, Any], fragment: Dict[str, Any]) -> Optional[int]:
    forensics = _dict(root.get("forensics"))
    return _first_int(
        fragment.get("pps_vclock_phase_cycles"),
        _nested(fragment, "pps", "vclock_phase_cycles"),
        forensics.get("pps_vclock_phase_cycles"),
    )


def _lane_row(
    fragment: Dict[str, Any],
    lane_name: str,
    reference_cycles: Optional[int],
    previous_interval: Optional[int],
) -> LaneRow:
    lane = _dict(fragment.get(LANE_KEYS[lane_name]))
    science = _dict(lane.get("science"))

    delta_raw_valid = _as_bool(science.get("delta_raw_valid"))
    interval = _as_int(science.get("delta_raw_clock_interval_cycles"))
    source = "SCIENCE_DELTA_OBSERVED"

    # This fallback remains an observed-edge fact, but a missing canonical science
    # interval is still reported as an issue so schema regressions stay visible.
    if interval is None:
        phaseledger = _dict(_nested(lane, "counterledger", "phaseledger"))
        if _as_bool(phaseledger.get("valid")) is True:
            interval = _as_int(phaseledger.get("ocxo_interval_cycles"))
            if interval is not None:
                source = "PHASELEDGER_FALLBACK"

    lane_reference = _first_int(
        science.get("delta_raw_reference_interval_cycles"),
        reference_cycles,
    )
    interval_delta = (
        interval - previous_interval
        if interval is not None and previous_interval is not None
        else None
    )
    cycle_residual = (
        interval - lane_reference
        if interval is not None and lane_reference is not None
        else None
    )

    return LaneRow(
        interval_cycles=interval,
        interval_delta_cycles=interval_delta,
        reference_cycles=lane_reference,
        cycle_residual=cycle_residual,
        fast_residual_ns=_first_float(
            science.get("delta_raw_fast_residual_ns_exact"),
            science.get("fast_residual_ns_exact"),
            science.get("fast_residual_ns"),
        ),
        ppb_1s=_as_float(science.get("ppb_1s")),
        science_valid=_as_bool(science.get("valid")),
        delta_raw_valid=delta_raw_valid,
        edge_species=_as_str(science.get("edge_species")),
        frequency_source=_as_str(science.get("frequency_source")),
        reported_minus_observed_ns=_as_int(
            science.get("reported_minus_observed_residual_ns")
        ),
        source=source,
    )


def collect_rows(records: Sequence[Dict[str, Any]]) -> List[ReportRow]:
    collected: List[ReportRow] = []
    previous_reference: Optional[int] = None
    previous_lane_intervals: Dict[str, Optional[int]] = {
        "OCXO1": None,
        "OCXO2": None,
    }

    for record in records:
        root = _root(record)
        fragment = _fragment(record)
        reference = _reference_cycles(fragment)
        reference_delta = (
            reference - previous_reference
            if reference is not None and previous_reference is not None
            else None
        )

        lanes: Dict[str, LaneRow] = {}
        for lane_name in ("OCXO1", "OCXO2"):
            lane_row = _lane_row(
                fragment,
                lane_name,
                reference,
                previous_lane_intervals[lane_name],
            )
            lanes[lane_name] = lane_row

        collected.append(
            ReportRow(
                pps_count=_pps_count(root, fragment),
                campaign_seconds=_as_int(fragment.get("campaign_seconds")),
                reference_cycles=reference,
                reference_delta_cycles=reference_delta,
                reference_nominal_residual_cycles=_first_int(
                    _nested(fragment, "dwt", "second_residual_cycles")
                ),
                pps_vclock_phase_cycles=_phase_cycles(root, fragment),
                candidate_disposition=_as_str(
                    fragment.get("candidate_disposition")
                ),
                candidate_reason=_as_str(fragment.get("candidate_reason")),
                timeline_valid=_as_bool(fragment.get("timeline_valid")),
                lanes=lanes,
            )
        )

        # A missing interval breaks adjacency.  Do not compute a later first
        # difference across an unobserved TIMEBASE row.
        previous_reference = reference
        for lane_name in ("OCXO1", "OCXO2"):
            previous_lane_intervals[lane_name] = (
                lanes[lane_name].interval_cycles
            )

    return collected


def classify_row(
    row: ReportRow,
    selected: Sequence[str],
    gate_cycles: int,
) -> List[str]:
    issues: List[str] = []

    if row.pps_count is None:
        issues.append("TIMEBASE row has no PPS/public-count identity")

    if row.reference_cycles is None:
        issues.append("VCLOCK/PPS reference interval is missing")
    elif row.reference_cycles <= 0:
        issues.append(
            f"VCLOCK/PPS reference interval is {row.reference_cycles:,d} cycles"
        )

    if (
        row.reference_delta_cycles is not None
        and abs(row.reference_delta_cycles) > gate_cycles
    ):
        issues.append(
            "VCLOCK/PPS interval first difference "
            f"{row.reference_delta_cycles:+,d} exceeds gate"
        )

    if row.timeline_valid is False:
        issues.append("TIMEBASE timeline_valid is false")

    if (
        row.candidate_disposition is not None
        and row.candidate_disposition.upper() != "ACCEPT"
    ):
        issues.append(
            "candidate disposition is "
            f"{row.candidate_disposition}"
            + (
                f" ({row.candidate_reason})"
                if row.candidate_reason
                else ""
            )
        )

    for lane_name in selected:
        lane = row.lanes[lane_name]

        if lane.interval_cycles is None:
            issues.append(f"{lane_name}: observed one-second interval is missing")
            continue
        if lane.interval_cycles <= 0:
            issues.append(
                f"{lane_name}: observed interval is "
                f"{lane.interval_cycles:,d} cycles"
            )

        if lane.source != "SCIENCE_DELTA_OBSERVED":
            issues.append(
                f"{lane_name}: canonical science interval missing; "
                "using Phase Ledger fallback"
            )

        if lane.delta_raw_valid is False:
            issues.append(f"{lane_name}: delta_raw_valid is false")
        if lane.science_valid is False:
            issues.append(f"{lane_name}: science.valid is false")

        if (
            lane.edge_species is not None
            and lane.edge_species != EXPECTED_EDGE_SPECIES
        ):
            issues.append(
                f"{lane_name}: edge species is {lane.edge_species}, "
                f"expected {EXPECTED_EDGE_SPECIES}"
            )

        if (
            lane.frequency_source is not None
            and lane.frequency_source != EXPECTED_FREQUENCY_SOURCE
        ):
            issues.append(
                f"{lane_name}: frequency source is {lane.frequency_source}, "
                f"expected {EXPECTED_FREQUENCY_SOURCE}"
            )

        if lane.reference_cycles is None:
            issues.append(f"{lane_name}: reference interval is missing")
        elif (
            row.reference_cycles is not None
            and lane.reference_cycles != row.reference_cycles
        ):
            issues.append(
                f"{lane_name}: lane reference differs from VCLOCK/PPS by "
                f"{lane.reference_cycles - row.reference_cycles:+,d} cycles"
            )

        if (
            lane.cycle_residual is not None
            and abs(lane.cycle_residual) > gate_cycles
        ):
            issues.append(
                f"{lane_name}: observed cycle residual "
                f"{lane.cycle_residual:+,d} exceeds gate"
            )

        if (
            lane.interval_delta_cycles is not None
            and abs(lane.interval_delta_cycles) > gate_cycles
        ):
            issues.append(
                f"{lane_name}: interval first difference "
                f"{lane.interval_delta_cycles:+,d} exceeds gate"
            )

        if (
            lane.reported_minus_observed_ns is not None
            and lane.reported_minus_observed_ns != 0
        ):
            issues.append(
                f"{lane_name}: reported residual differs from observed "
                f"residual by {lane.reported_minus_observed_ns:+,d} ns"
            )

    return issues


def _header(selected: Sequence[str]) -> Tuple[str, str]:
    columns = [
        ("pps", 7),
        ("v_cyc", 13),
        ("v_d", 8),
        ("v_nom", 8),
        ("p_voff", 8),
    ]
    for lane in selected:
        prefix = "o1" if lane == "OCXO1" else "o2"
        columns.extend(
            [
                (f"{prefix}_cyc", 13),
                (f"{prefix}_d", 8),
                (f"{prefix}-v", 8),
                (f"{prefix}_ns", 10),
                (f"{prefix}_ppb", 10),
            ]
        )
    header = "  ".join(f"{name:>{width}s}" for name, width in columns)
    rule = "  ".join("─" * width for _, width in columns)
    return header, rule


def _row_text(row: ReportRow, selected: Sequence[str]) -> str:
    fields = [
        _fmt_int(row.pps_count, 7),
        _fmt_int(row.reference_cycles, 13),
        _fmt_int(row.reference_delta_cycles, 8, signed=True),
        _fmt_int(row.reference_nominal_residual_cycles, 8, signed=True),
        _fmt_int(row.pps_vclock_phase_cycles, 8, signed=True),
    ]
    for lane_name in selected:
        lane = row.lanes[lane_name]
        fields.extend(
            [
                _fmt_int(lane.interval_cycles, 13),
                _fmt_int(lane.interval_delta_cycles, 8, signed=True),
                _fmt_int(lane.cycle_residual, 8, signed=True),
                _fmt_float(lane.fast_residual_ns, 10, 3, signed=True),
                _fmt_float(lane.ppb_1s, 10, 3, signed=True),
            ]
        )
    return "  ".join(fields)


def _detail_lines(
    row: ReportRow,
    selected: Sequence[str],
    gate_cycles: int,
) -> List[str]:
    lines = [
        "    └─ OBSERVED-EDGE PATHOLOGY "
        f"@ PPS {_fmt_int(row.pps_count, 0).strip()} "
        f"({len(row.issues)} issues, gate={gate_cycles:,d} cycles)"
    ]
    for issue in row.issues:
        lines.append(f"       • {issue}")

    lines.append(
        "       VCLOCK/PPS: "
        f"interval={_fmt_int(row.reference_cycles, 0).strip()} "
        f"first_delta={_fmt_int(row.reference_delta_cycles, 0, True).strip()} "
        f"nominal_residual={_fmt_int(row.reference_nominal_residual_cycles, 0, True).strip()} "
        f"phase={_fmt_int(row.pps_vclock_phase_cycles, 0, True).strip()}"
    )

    for lane_name in selected:
        lane = row.lanes[lane_name]
        lines.append(
            f"       {lane_name}: "
            f"interval={_fmt_int(lane.interval_cycles, 0).strip()} "
            f"reference={_fmt_int(lane.reference_cycles, 0).strip()} "
            f"cycle_residual={_fmt_int(lane.cycle_residual, 0, True).strip()} "
            f"first_delta={_fmt_int(lane.interval_delta_cycles, 0, True).strip()} "
            f"fast_ns={_fmt_float(lane.fast_residual_ns, 0, 6, True).strip()} "
            f"ppb={_fmt_float(lane.ppb_1s, 0, 6, True).strip()}"
        )
        lines.append(
            f"       {lane_name} authority: "
            f"source={lane.source} "
            f"science_valid={lane.science_valid} "
            f"delta_raw_valid={lane.delta_raw_valid} "
            f"edge={lane.edge_species or '---'} "
            f"frequency_source={lane.frequency_source or '---'}"
        )

    return lines


def analyze(options: Options) -> None:
    records = fetch_timebase(options.campaign)
    if not records:
        print(f"No TIMEBASE rows found for campaign '{options.campaign}'.")
        return

    collected = collect_rows(records)
    if options.limit > 0:
        collected = collected[-options.limit :]

    selected = selected_lanes(options.clock)
    for row in collected:
        row.issues = classify_row(
            row,
            selected,
            options.pathology_gate,
        )

    view = options.clock or "ALL"
    print(
        f"Campaign: {options.campaign}  "
        f"({len(collected):,} rows, view={view})"
    )
    print()
    print("Observed-edge one-second cycle audit")
    print("════════════════════════════════════")
    print("  VCLOCK/PPS is the reference rail.")
    print("  *_cyc is the observed DWT interval for one authored clock second.")
    print("  *_d is interval[n] - interval[n-1] for the same rail.")
    print("  o*-v is observed OCXO cycles - its paired VCLOCK/PPS reference.")
    print("  *_ns and *_ppb are TIMEBASE science residuals; positive means clock fast.")
    print("  Compatibility *_raw/*_fl/*_pub fields are intentionally ignored.")
    print(
        f"  Pathology gate: {options.pathology_gate:,d} cycles; "
        "only observed-edge or schema failures are considered."
    )
    if options.compatibility_view_requested:
        print(
            "  Note: legacy alignment/slip option accepted but ignored; "
            "current lane science is already paired to its reference interval."
        )
    print()

    header, rule = _header(selected)
    print(header)
    print(rule)

    pathology_count = 0
    for row in collected:
        if row.issues:
            pathology_count += 1
        if options.pathology_only and not row.issues:
            continue
        print(_row_text(row, selected))
        if row.issues:
            for line in _detail_lines(
                row,
                selected,
                options.pathology_gate,
            ):
                print(line)

    stats: Dict[str, Welford] = {
        "reference": Welford(),
        "reference_delta": Welford(),
        "reference_nominal": Welford(),
        "phase": Welford(),
    }
    for lane_name in selected:
        stats[f"{lane_name}.interval"] = Welford()
        stats[f"{lane_name}.delta"] = Welford()
        stats[f"{lane_name}.residual"] = Welford()
        stats[f"{lane_name}.fast_ns"] = Welford()
        stats[f"{lane_name}.ppb"] = Welford()

    coverage = {
        lane_name: {
            "interval": 0,
            "science_valid": 0,
            "observed_authority": 0,
        }
        for lane_name in selected
    }

    for row in collected:
        _update(stats["reference"], row.reference_cycles)
        _update(stats["reference_delta"], row.reference_delta_cycles)
        _update(
            stats["reference_nominal"],
            row.reference_nominal_residual_cycles,
        )
        _update(stats["phase"], row.pps_vclock_phase_cycles)
        for lane_name in selected:
            lane = row.lanes[lane_name]
            _update(stats[f"{lane_name}.interval"], lane.interval_cycles)
            _update(stats[f"{lane_name}.delta"], lane.interval_delta_cycles)
            _update(stats[f"{lane_name}.residual"], lane.cycle_residual)
            _update(stats[f"{lane_name}.fast_ns"], lane.fast_residual_ns)
            _update(stats[f"{lane_name}.ppb"], lane.ppb_1s)
            if lane.interval_cycles is not None:
                coverage[lane_name]["interval"] += 1
            if lane.science_valid is True and lane.delta_raw_valid is True:
                coverage[lane_name]["science_valid"] += 1
            if (
                lane.edge_species == EXPECTED_EDGE_SPECIES
                and lane.frequency_source == EXPECTED_FREQUENCY_SOURCE
            ):
                coverage[lane_name]["observed_authority"] += 1

    print()
    print("Schema coverage")
    print("═══════════════")
    print(
        f"  rows={len(collected):,}  "
        f"reference_intervals={stats['reference'].n:,}  "
        f"pathological_rows={pathology_count:,}"
    )
    for lane_name in selected:
        lane_coverage = coverage[lane_name]
        print(
            f"  {lane_name:<6s} "
            f"interval={lane_coverage['interval']:,}  "
            f"science_valid={lane_coverage['science_valid']:,}  "
            f"observed_authority={lane_coverage['observed_authority']:,}"
        )

    print()
    print("Summary")
    print("═══════")
    _print_stat("VCLOCK/PPS interval", stats["reference"], "cycles")
    _print_stat(
        "VCLOCK/PPS first difference",
        stats["reference_delta"],
        "cycles",
    )
    _print_stat(
        "DWT nominal-second residual",
        stats["reference_nominal"],
        "cycles",
    )
    _print_stat("PPS→VCLOCK phase", stats["phase"], "cycles")

    for lane_name in selected:
        print()
        _print_stat(
            f"{lane_name} observed interval",
            stats[f"{lane_name}.interval"],
            "cycles",
        )
        _print_stat(
            f"{lane_name} interval first difference",
            stats[f"{lane_name}.delta"],
            "cycles",
        )
        _print_stat(
            f"{lane_name} cycle residual vs VCLOCK",
            stats[f"{lane_name}.residual"],
            "cycles",
        )
        _print_stat(
            f"{lane_name} fast residual",
            stats[f"{lane_name}.fast_ns"],
            "ns",
        )
        _print_stat(
            f"{lane_name} one-second frequency",
            stats[f"{lane_name}.ppb"],
            "ppb",
        )

    print()
    print("Notes")
    print("═════")
    print("  • This report consumes fragment.dwt and fragment.<lane>.science only.")
    print("  • TIMEBASE_FORENSICS compatibility endpoints are not interval authority.")
    print("  • An OCXO cycle residual is measured_interval - paired_reference_interval.")
    print("  • Positive fast_ns/ppb means the clock is fast; that sign is opposite")
    print("    the DWT cycle residual because a faster clock reaches its authored")
    print("    second in fewer reference-DWT cycles.")
    print("  • A large *_d often appears as a positive ISR-delay spike followed by")
    print("    a compensating negative rebound.  The report shows the fact; it does")
    print("    not smooth, infer, repair, or replace the observed edge.")
    print()


def parse_args(argv: Sequence[str]) -> Options:
    if len(argv) < 2:
        raise SystemExit(
            "Usage: raw_cycles <campaign_name> [limit] [clock] "
            "[--pathology-only] [--pathology-gate N]"
        )

    options = Options(campaign=argv[1])
    index = 2
    while index < len(argv):
        arg = argv[index]

        if arg in (
            "--align-ocxo",
            "--align-ocxos",
            "--delay-pps-vclock",
            "--slip",
        ):
            options.compatibility_view_requested = True
            index += 1
            continue

        if arg == "--pathology-only":
            options.pathology_only = True
            index += 1
            continue

        if arg == "--pathology-gate":
            if index + 1 >= len(argv):
                raise SystemExit("--pathology-gate requires an integer")
            try:
                options.pathology_gate = int(argv[index + 1])
            except ValueError as exc:
                raise SystemExit(
                    "--pathology-gate requires an integer"
                ) from exc
            index += 2
            continue

        if arg.startswith("--pathology-gate="):
            try:
                options.pathology_gate = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit(
                    "--pathology-gate requires an integer"
                ) from exc
            index += 1
            continue

        if arg == "--clock":
            if index + 1 >= len(argv):
                raise SystemExit("--clock requires VCLOCK, OCXO1, or OCXO2")
            options.clock = normalize_clock(argv[index + 1])
            index += 2
            continue

        if arg.startswith("--clock="):
            options.clock = normalize_clock(arg.split("=", 1)[1])
            index += 1
            continue

        if arg == "--limit":
            if index + 1 >= len(argv):
                raise SystemExit("--limit requires an integer")
            try:
                options.limit = int(argv[index + 1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            index += 2
            continue

        if arg.startswith("--limit="):
            try:
                options.limit = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            index += 1
            continue

        folded = arg.upper()
        if folded in CLOCKS or folded in {"V", "VC", "O1", "O2"}:
            options.clock = normalize_clock(arg)
            index += 1
            continue

        try:
            options.limit = int(arg)
        except ValueError as exc:
            raise SystemExit(f"unknown argument '{arg}'") from exc
        index += 1

    if options.limit < 0:
        raise SystemExit("--limit must be zero or positive")
    if options.pathology_gate < 0:
        raise SystemExit("--pathology-gate must be zero or positive")
    return options


def main() -> None:
    analyze(parse_args(sys.argv))


if __name__ == "__main__":
    main()
