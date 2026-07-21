"""ZPNet OCXO lineage audit.

Tests the specific hypothesis that process_interrupt continues delivering lawful
one-second OCXO events while process_clocks later loses endpoint freshness or
pairs non-adjacent endpoints, producing zero/two-second science intervals.

The audit deliberately separates:

  1. raw observed one-second facts (fragment.raw_cycles)
  2. lane-local science products (fragment.ocxoN.science)
  3. optional lineage / generation / counter fields when present

Usage:
    python ocxo_lineage_audit.py CAMPAIGN
    python ocxo_lineage_audit.py CAMPAIGN --clock OCXO2 --window 8
    python ocxo_lineage_audit.py CAMPAIGN --all-pathologies
    python ocxo_lineage_audit.py CAMPAIGN --json-out forensic1_audit.json

The script is tolerant of schema evolution.  Missing optional fields are shown
as absent rather than treated as proof of failure.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

CLOCKS: Tuple[str, ...] = ("OCXO1", "OCXO2")
CLOCK_KEYS = {"OCXO1": "ocxo1", "OCXO2": "ocxo2"}

RAW_MIN = 900_000_000
RAW_MAX = 1_100_000_000
TWO_SECOND_MIN = 1_800_000_000
TWO_SECOND_MAX = 2_200_000_000
ONE_SECOND_TICKS = 10_000_000
TWO_SECOND_TICKS = 20_000_000


@dataclass
class Sample:
    pps: Optional[int]
    clock: str

    disposition: str = ""
    gate_mode: str = ""
    science_res: str = ""

    raw_observed: Optional[int] = None
    raw_previous: Optional[int] = None
    raw_residual: Optional[int] = None
    raw_valid: Optional[bool] = None

    science_clock_interval: Optional[int] = None
    science_reference_interval: Optional[int] = None
    science_valid: Optional[bool] = None
    delta_raw_valid: Optional[bool] = None
    antecedents_complete: Optional[bool] = None
    science_worthy: Optional[bool] = None

    event_counter32: Optional[int] = None
    previous_event_counter32: Optional[int] = None
    counter_delta_ticks: Optional[int] = None

    event_generation: Optional[int] = None
    previous_generation: Optional[int] = None
    consumed_generation: Optional[int] = None
    update_count: Optional[int] = None
    completed_interval_count: Optional[int] = None

    issues: List[str] = field(default_factory=list)
    evidence: List[str] = field(default_factory=list)


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
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        value = value.strip().lower()
        if value in {"true", "yes", "1"}:
            return True
        if value in {"false", "no", "0"}:
            return False
    return None


def s(value: Any) -> str:
    return "" if value is None else str(value)


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = i(value)
        if parsed is not None:
            return parsed
    return None


def first_bool(*values: Any) -> Optional[bool]:
    for value in values:
        parsed = b(value)
        if parsed is not None:
            return parsed
    return None


def dig(obj: Dict[str, Any], *paths: str) -> Any:
    """Return the first present dotted path."""
    for path in paths:
        cur: Any = obj
        found = True
        for part in path.split("."):
            if not isinstance(cur, dict) or part not in cur:
                found = False
                break
            cur = cur[part]
        if found:
            return cur
    return None


def fragment(record: Dict[str, Any]) -> Dict[str, Any]:
    root = d(record.get("payload")) or record
    return d(root.get("fragment")) or root


def pps_count(record: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    root = d(record.get("payload")) or record
    return first_int(
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("campaign_seconds"),
        root.get("pps_count"),
    )


def fetch(campaign: str) -> List[Dict[str, Any]]:
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
                NULLIF(payload->'fragment'->>'campaign_seconds', '')::bigint
            )
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    records: List[Dict[str, Any]] = []
    for row in rows:
        value = row["payload"]
        if isinstance(value, str):
            value = json.loads(value)
        if isinstance(value, dict):
            records.append(value)
    return records


def raw_lane(frag: Dict[str, Any], key: str) -> Dict[str, Any]:
    return d(d(frag.get("raw_cycles")).get(key))


def lane_science(frag: Dict[str, Any], key: str) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    lane = d(frag.get(key))
    return lane, d(lane.get("science"))


def collect(records: Sequence[Dict[str, Any]]) -> Dict[str, List[Sample]]:
    result: Dict[str, List[Sample]] = {clock: [] for clock in CLOCKS}
    prior_raw: Dict[str, Optional[int]] = {clock: None for clock in CLOCKS}

    for record in records:
        frag = fragment(record)
        count = pps_count(record, frag)

        disposition = s(
            dig(frag, "candidate_disposition", "candidate.disposition")
            or "ACCEPT"
        ).upper()
        gate_mode = s(dig(frag, "gate_mode", "acceptance_mode")).upper()
        science_res = s(
            dig(
                frag,
                "science_res",
                "science_residuals",
                "feature_status.SCI_RES",
                "features.SCI_RES",
            )
        ).upper()

        for clock in CLOCKS:
            key = CLOCK_KEYS[clock]
            raw = raw_lane(frag, key)
            lane, science = lane_science(frag, key)

            raw_observed = first_int(
                raw.get("observed_cycles"),
                lane.get("observed_cycles"),
                science.get("delta_raw_clock_interval_cycles"),
            )
            raw_previous = first_int(
                raw.get("previous_observed_cycles"),
                prior_raw[clock],
            )
            raw_residual = first_int(raw.get("residual_cycles"))
            if (
                raw_residual is None
                and raw_observed is not None
                and raw_previous is not None
            ):
                raw_residual = raw_observed - raw_previous

            sample = Sample(
                pps=count,
                clock=clock,
                disposition=disposition,
                gate_mode=gate_mode,
                science_res=science_res,
                raw_observed=raw_observed,
                raw_previous=raw_previous,
                raw_residual=raw_residual,
                raw_valid=first_bool(raw.get("valid")),
                science_clock_interval=first_int(
                    science.get("delta_raw_clock_interval_cycles"),
                    science.get("clock_interval_cycles"),
                    science.get("cycles_between_edges"),
                ),
                science_reference_interval=first_int(
                    science.get("delta_raw_reference_interval_cycles"),
                    science.get("reference_interval_cycles"),
                ),
                science_valid=first_bool(science.get("valid")),
                delta_raw_valid=first_bool(science.get("delta_raw_valid")),
                antecedents_complete=first_bool(
                    science.get("antecedents_complete")
                ),
                science_worthy=first_bool(
                    science.get("science_worthy"),
                    lane.get("science_worthy"),
                ),
                event_counter32=first_int(
                    dig(
                        lane,
                        "counter32_at_event",
                        "event.counter32_at_event",
                        "last_event_counter32",
                        "clockface.counter32",
                    ),
                    dig(
                        science,
                        "current_counter32",
                        "event_counter32",
                        "clock_counter32",
                    ),
                ),
                previous_event_counter32=first_int(
                    dig(
                        science,
                        "previous_counter32",
                        "previous_event_counter32",
                        "left_counter32",
                    )
                ),
                counter_delta_ticks=first_int(
                    dig(
                        science,
                        "counter_delta_ticks",
                        "delta_raw_clock_counter_delta_ticks",
                        "clock_counter_delta_ticks",
                    ),
                    dig(
                        lane,
                        "counter_delta_ticks",
                        "last_counter_delta_ticks",
                    ),
                ),
                event_generation=first_int(
                    dig(
                        science,
                        "event_generation",
                        "current_generation",
                        "generation",
                    ),
                    dig(lane, "event_generation", "generation"),
                ),
                previous_generation=first_int(
                    dig(
                        science,
                        "previous_generation",
                        "left_generation",
                        "prior_generation",
                    )
                ),
                consumed_generation=first_int(
                    dig(
                        science,
                        "consumed_generation",
                        "last_consumed_generation",
                    )
                ),
                update_count=first_int(
                    dig(
                        science,
                        "update_count",
                        "event_update_count",
                        "science_update_count",
                    ),
                    dig(lane, "update_count", "event_count"),
                ),
                completed_interval_count=first_int(
                    dig(
                        science,
                        "completed_interval_count",
                        "interval_count",
                        "science_interval_count",
                    ),
                    dig(lane, "completed_interval_count"),
                ),
            )

            classify(sample)
            result[clock].append(sample)
            prior_raw[clock] = raw_observed

    return result


def plausible_one_second(value: Optional[int]) -> bool:
    return value is not None and RAW_MIN <= value <= RAW_MAX


def plausible_two_seconds(value: Optional[int]) -> bool:
    return value is not None and TWO_SECOND_MIN <= value <= TWO_SECOND_MAX


def classify(sample: Sample) -> None:
    if sample.pps is None:
        sample.issues.append("missing PPS identity")

    raw_good = plausible_one_second(sample.raw_observed)
    science_two = plausible_two_seconds(sample.science_clock_interval)
    science_zero = sample.science_clock_interval == 0

    if sample.raw_observed is None:
        sample.issues.append("raw observed interval missing")
    elif not raw_good:
        sample.issues.append(
            f"raw observed interval is not one second: {sample.raw_observed:,d}"
        )

    if sample.raw_valid is False:
        sample.issues.append("raw producer marked sample invalid")

    if sample.science_valid is False:
        sample.issues.append("science.valid is false")
    if sample.delta_raw_valid is False:
        sample.issues.append("delta_raw_valid is false")
    if sample.antecedents_complete is False:
        sample.issues.append("antecedents_complete is false")
    if sample.science_worthy is False:
        sample.issues.append("science_worthy is false")

    if science_two:
        sample.issues.append(
            f"science interval spans approximately two seconds: "
            f"{sample.science_clock_interval:,d}"
        )
    elif science_zero:
        sample.issues.append("science interval is zero")
    elif (
        sample.science_clock_interval is not None
        and not plausible_one_second(sample.science_clock_interval)
    ):
        sample.issues.append(
            f"science interval is implausible: "
            f"{sample.science_clock_interval:,d}"
        )

    if (
        sample.counter_delta_ticks is not None
        and sample.counter_delta_ticks not in (ONE_SECOND_TICKS, TWO_SECOND_TICKS)
    ):
        sample.issues.append(
            f"unexpected counter delta: {sample.counter_delta_ticks:,d} ticks"
        )

    # Positive evidence for the downstream bookkeeping hypothesis.
    if raw_good and sample.raw_valid is not False:
        sample.evidence.append("raw one-second observation remains lawful")
    if raw_good and (
        science_two
        or science_zero
        or sample.science_valid is False
        or sample.delta_raw_valid is False
        or sample.antecedents_complete is False
    ):
        sample.evidence.append(
            "raw/science contradiction localizes failure downstream of capture"
        )
    if (
        sample.counter_delta_ticks == ONE_SECOND_TICKS
        and science_two
    ):
        sample.evidence.append(
            "one-second counter lineage coexists with two-second science pairing"
        )


def first_pathology(samples: Sequence[Sample]) -> Optional[int]:
    for idx, sample in enumerate(samples):
        raw_good = plausible_one_second(sample.raw_observed)
        downstream_bad = (
            plausible_two_seconds(sample.science_clock_interval)
            or sample.science_clock_interval == 0
            or sample.science_valid is False
            or sample.delta_raw_valid is False
            or sample.antecedents_complete is False
            or sample.science_worthy is False
        )
        if raw_good and downstream_bad:
            return idx
    return None


def fnum(value: Optional[int], width: int = 13) -> str:
    return ("---" if value is None else f"{value:,d}").rjust(width)


def fbool(value: Optional[bool], width: int = 5) -> str:
    text = "?" if value is None else ("T" if value else "F")
    return text.rjust(width)


def print_window(samples: Sequence[Sample], center: int, radius: int) -> None:
    start = max(0, center - radius)
    stop = min(len(samples), center + radius + 1)

    print(
        "    PPS     RAW_OBS       RAW_RES    SCI_CLOCK   "
        "RAW  SCI  DRV  ANT   CNT_DELTA   INT_N"
    )
    print(
        "  ─────  ─────────────  ─────────  ─────────────  "
        "───  ───  ───  ───  ───────────  ─────"
    )
    for idx in range(start, stop):
        row = samples[idx]
        marker = ">>" if idx == center else "  "
        print(
            f"{marker}{fnum(row.pps, 5)}  "
            f"{fnum(row.raw_observed)}  "
            f"{fnum(row.raw_residual, 9)}  "
            f"{fnum(row.science_clock_interval)}  "
            f"{fbool(row.raw_valid, 3)}  "
            f"{fbool(row.science_valid, 3)}  "
            f"{fbool(row.delta_raw_valid, 3)}  "
            f"{fbool(row.antecedents_complete, 3)}  "
            f"{fnum(row.counter_delta_ticks, 11)}  "
            f"{fnum(row.completed_interval_count, 5)}"
        )
        for issue in row.issues:
            print(f"      └─ {issue}")
        for evidence in row.evidence:
            print(f"         ✓ {evidence}")


def summarize_clock(clock: str, samples: Sequence[Sample], window: int) -> Dict[str, Any]:
    idx = first_pathology(samples)
    raw_bad = sum(
        1 for row in samples
        if row.raw_observed is not None and not plausible_one_second(row.raw_observed)
    )
    two_second_science = sum(
        1 for row in samples if plausible_two_seconds(row.science_clock_interval)
    )
    contradictions = sum(
        1 for row in samples
        if plausible_one_second(row.raw_observed)
        and (
            plausible_two_seconds(row.science_clock_interval)
            or row.science_clock_interval == 0
            or row.science_valid is False
            or row.delta_raw_valid is False
            or row.antecedents_complete is False
        )
    )

    summary = {
        "clock": clock,
        "rows": len(samples),
        "first_pathology_index": idx,
        "first_pathology_pps": samples[idx].pps if idx is not None else None,
        "raw_bad_rows": raw_bad,
        "two_second_science_rows": two_second_science,
        "raw_science_contradictions": contradictions,
    }

    print(f"\n{clock}")
    print("═" * len(clock))
    print(f"Rows examined:                  {len(samples):,}")
    print(f"Raw non-one-second rows:        {raw_bad:,}")
    print(f"Two-second science rows:        {two_second_science:,}")
    print(f"Raw/science contradictions:     {contradictions:,}")

    if idx is None:
        print("First downstream pathology:      not found")
    else:
        print(
            "First downstream pathology:      "
            f"PPS {samples[idx].pps if samples[idx].pps is not None else '?'}"
        )
        print_window(samples, idx, window)

    return summary


def infer(result: Dict[str, List[Sample]]) -> str:
    o1_idx = first_pathology(result["OCXO1"])
    o2_idx = first_pathology(result["OCXO2"])

    if o2_idx is not None:
        row = result["OCXO2"][o2_idx]
        if plausible_one_second(row.raw_observed):
            if o1_idx is None:
                return (
                    "SUPPORTED: OCXO2 retains a lawful one-second raw observation "
                    "at the first failure while its science product becomes invalid "
                    "or spans two seconds, and OCXO1 remains clean.  This strongly "
                    "supports a lane-local downstream endpoint freshness/pairing bug "
                    "rather than a process_interrupt capture, handoff, or dispatch loss."
                )
            return (
                "PARTIALLY SUPPORTED: OCXO2 shows a raw/science contradiction, but "
                "OCXO1 also develops a downstream pathology.  Inspect shared Alpha/Beta "
                "freshness and generation logic."
            )

    return (
        "NOT YET PROVEN: no row was found where lawful one-second raw OCXO2 evidence "
        "coexists with invalid, zero, or two-second OCXO2 science."
    )


def parse(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test whether OCXO science loses or mispairs delivered endpoints."
    )
    parser.add_argument("campaign")
    parser.add_argument(
        "--clock",
        choices=CLOCKS,
        help="Show only one OCXO lane.",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=6,
        help="Rows before and after the first pathology (default: 6).",
    )
    parser.add_argument(
        "--all-pathologies",
        action="store_true",
        help="Print every contradictory row after the main onset window.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        help="Write normalized samples and summary to JSON.",
    )
    return parser.parse_args(argv[1:])


def main(argv: Sequence[str]) -> None:
    args = parse(argv)
    records = fetch(args.campaign)
    result = collect(records)

    selected = (args.clock,) if args.clock else CLOCKS
    print(f"Campaign: {args.campaign}")
    print(f"TIMEBASE rows fetched: {len(records):,}")
    print(
        "\nTest: does raw one-second interrupt evidence remain lawful while "
        "downstream science loses freshness or pairs non-adjacent endpoints?"
    )

    summaries: List[Dict[str, Any]] = []
    for clock in selected:
        summaries.append(summarize_clock(clock, result[clock], args.window))

    verdict = infer(result)
    print("\nHypothesis verdict")
    print("══════════════════")
    print(verdict)

    if args.all_pathologies:
        for clock in selected:
            print(f"\nAll contradictory {clock} rows")
            print("─" * (23 + len(clock)))
            found = False
            for idx, row in enumerate(result[clock]):
                if (
                    plausible_one_second(row.raw_observed)
                    and (
                        plausible_two_seconds(row.science_clock_interval)
                        or row.science_clock_interval == 0
                        or row.science_valid is False
                        or row.delta_raw_valid is False
                        or row.antecedents_complete is False
                        or row.science_worthy is False
                    )
                ):
                    found = True
                    print_window(result[clock], idx, 0)
            if not found:
                print("  none")

    if args.json_out:
        payload = {
            "campaign": args.campaign,
            "verdict": verdict,
            "summary": summaries,
            "samples": {
                clock: [asdict(sample) for sample in result[clock]]
                for clock in selected
            },
        }
        args.json_out.write_text(
            json.dumps(payload, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        print(f"\nWrote {args.json_out}")


if __name__ == "__main__":
    main(sys.argv)
