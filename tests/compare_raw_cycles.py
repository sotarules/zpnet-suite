"""Compare literal one-second DWT cycle counts between two ZPNet campaigns.

The report has two purposes:

1. Show the observed cycle count published for PPS, VCLOCK, OCXO1, and OCXO2
   in each aligned campaign second, together with NEW - OLD.
2. Remove common-mode DWT-rate movement by comparing each rail with PPS and
   VCLOCK inside the same row before comparing campaigns.

No FloorLine, projection, EMA, inferred endpoint, or residual repair is used.
Current firmware is read from ``fragment.raw_cycles``.  Narrow observed-edge
fallbacks keep older campaigns readable.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


RAILS: Tuple[str, ...] = ("PPS", "VCLOCK", "OCXO1", "OCXO2")
RAIL_KEYS = {name: name.lower() for name in RAILS}
PLAUSIBLE_MIN = 900_000_000
PLAUSIBLE_MAX = 1_100_000_000


@dataclass(frozen=True)
class RailSample:
    observed: Optional[int]
    valid: Optional[bool]
    source: str


@dataclass(frozen=True)
class CampaignRow:
    ordinal: int
    count: Optional[int]
    disposition: str
    timeline_valid: Optional[bool]
    rails: Dict[str, RailSample]


@dataclass(frozen=True)
class Pair:
    key: int
    old: CampaignRow
    new: CampaignRow


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
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("campaign_seconds"),
        root.get("pps_count"),
    )


def fallback_observed(frag: Dict[str, Any], rail: str) -> Optional[int]:
    if rail == "PPS":
        return first_int(d(frag.get("pps")).get("dwt_cycles_between_edges"))

    if rail == "VCLOCK":
        vclock = d(frag.get("vclock"))
        return first_int(
            vclock.get("observed_cycles"),
            d(vclock.get("science")).get("cycles_between_edges"),
            frag.get("pps_vclock_dwt_cycles_between_edges"),
        )

    lane = d(frag.get(RAIL_KEYS[rail]))
    science = d(lane.get("science"))
    return first_int(
        lane.get("observed_cycles"),
        science.get("delta_raw_clock_interval_cycles"),
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
                        NULLIF(payload->'fragment'->>'campaign_seconds', '')::bigint
                      ),
                      payload::text
            """,
            (campaign,),
        )
        database_rows = cur.fetchall()

    records: List[Dict[str, Any]] = []
    for item in database_rows:
        value = item["payload"]
        if isinstance(value, str):
            value = json.loads(value)
        if isinstance(value, dict):
            records.append(value)
    return records


def collect(campaign: str) -> List[CampaignRow]:
    rows: List[CampaignRow] = []
    for ordinal, record in enumerate(fetch(campaign), start=1):
        frag = fragment(record)
        raw = d(frag.get("raw_cycles"))
        rails: Dict[str, RailSample] = {}

        for name in RAILS:
            obj = d(raw.get(RAIL_KEYS[name]))
            observed = first_int(
                obj.get("observed_cycles"),
                fallback_observed(frag, name),
            )
            rails[name] = RailSample(
                observed=observed,
                valid=b(obj.get("valid")),
                source=(
                    "RAW_CYCLES_OBSERVED_V1"
                    if obj
                    else "OBSERVED_FALLBACK"
                ),
            )

        rows.append(
            CampaignRow(
                ordinal=ordinal,
                count=count_of(record, frag),
                disposition=str(
                    frag.get("candidate_disposition") or "ACCEPT"
                ).upper(),
                timeline_valid=b(frag.get("timeline_valid")),
                rails=rails,
            )
        )
    return rows


def duplicate_counts(rows: Sequence[CampaignRow]) -> List[int]:
    counts = Counter(row.count for row in rows if row.count is not None)
    return sorted(count for count, occurrences in counts.items() if occurrences > 1)


def align_rows(
    old_rows: Sequence[CampaignRow],
    new_rows: Sequence[CampaignRow],
    mode: str,
) -> Tuple[List[Pair], List[str]]:
    notes: List[str] = []

    if mode == "count":
        old_duplicates = duplicate_counts(old_rows)
        new_duplicates = duplicate_counts(new_rows)
        if old_duplicates or new_duplicates:
            details: List[str] = []
            if old_duplicates:
                details.append(f"OLD duplicates={old_duplicates}")
            if new_duplicates:
                details.append(f"NEW duplicates={new_duplicates}")
            raise SystemExit(
                "Cannot align by campaign count because duplicate counts exist: "
                + "; ".join(details)
                + ". Re-run with --align ordinal to inspect raw row order."
            )

        old_index = {row.count: row for row in old_rows if row.count is not None}
        new_index = {row.count: row for row in new_rows if row.count is not None}
        common = sorted(set(old_index) & set(new_index))
        pairs = [Pair(key=count, old=old_index[count], new=new_index[count]) for count in common]

        old_only = sorted(set(old_index) - set(new_index))
        new_only = sorted(set(new_index) - set(old_index))
        missing_old = sum(row.count is None for row in old_rows)
        missing_new = sum(row.count is None for row in new_rows)
        if old_only:
            notes.append(f"OLD-only campaign counts: {compact_ints(old_only)}")
        if new_only:
            notes.append(f"NEW-only campaign counts: {compact_ints(new_only)}")
        if missing_old or missing_new:
            notes.append(
                f"Rows missing campaign identity: OLD={missing_old}, NEW={missing_new}"
            )
        return pairs, notes

    length = min(len(old_rows), len(new_rows))
    pairs = [
        Pair(key=index + 1, old=old_rows[index], new=new_rows[index])
        for index in range(length)
    ]
    if len(old_rows) != len(new_rows):
        notes.append(
            f"Ordinal alignment truncated to {length} rows "
            f"(OLD={len(old_rows)}, NEW={len(new_rows)})."
        )
    return pairs, notes


def compact_ints(values: Sequence[int], maximum: int = 20) -> str:
    if len(values) <= maximum:
        return ",".join(str(value) for value in values)
    head = ",".join(str(value) for value in values[:maximum])
    return f"{head},... ({len(values)} total)"


def observed(row: CampaignRow, rail: str) -> Optional[int]:
    return row.rails[rail].observed


def subtract(left: Optional[int], right: Optional[int]) -> Optional[int]:
    if left is None or right is None:
        return None
    return left - right


def relative(row: CampaignRow, rail: str, reference: str) -> Optional[int]:
    return subtract(observed(row, rail), observed(row, reference))


def pair_delta(pair: Pair, rail: str) -> Optional[int]:
    return subtract(observed(pair.new, rail), observed(pair.old, rail))


def relative_shift(pair: Pair, rail: str, reference: str) -> Optional[int]:
    return subtract(
        relative(pair.new, rail, reference),
        relative(pair.old, rail, reference),
    )


def fmt_int(value: Optional[int], width: int, signed: bool = False) -> str:
    if value is None:
        text = "---"
    elif signed:
        text = f"{value:+,d}"
    else:
        text = f"{value:,d}"
    return f"{text:>{width}}"


def numeric(values: Iterable[Optional[int]]) -> List[int]:
    return [value for value in values if value is not None]


def mean_or_nan(values: Sequence[int]) -> float:
    return statistics.fmean(values) if values else math.nan


def median_or_nan(values: Sequence[int]) -> float:
    return float(statistics.median(values)) if values else math.nan


def pstdev_or_nan(values: Sequence[int]) -> float:
    return statistics.pstdev(values) if values else math.nan


def mad_or_nan(values: Sequence[int]) -> float:
    if not values:
        return math.nan
    center = statistics.median(values)
    return float(statistics.median(abs(value - center) for value in values))


def print_campaign_health(name: str, rows: Sequence[CampaignRow]) -> None:
    non_accept = sum(row.disposition != "ACCEPT" for row in rows)
    timeline_false = sum(row.timeline_valid is False for row in rows)
    missing_counts = sum(row.count is None for row in rows)
    duplicates = duplicate_counts(rows)

    print(
        f"  {name:<8} rows={len(rows):,} non_accept={non_accept:,} "
        f"timeline_false={timeline_false:,} missing_count={missing_counts:,} "
        f"duplicate_counts={len(duplicates):,}"
    )
    for rail in RAILS:
        missing = sum(observed(row, rail) is None for row in rows)
        invalid = sum(row.rails[rail].valid is False for row in rows)
        implausible = sum(
            value is not None and not PLAUSIBLE_MIN <= value <= PLAUSIBLE_MAX
            for value in (observed(row, rail) for row in rows)
        )
        sources = Counter(row.rails[rail].source for row in rows)
        source_text = ",".join(
            f"{source}:{count}" for source, count in sorted(sources.items())
        )
        print(
            f"           {rail:<6} missing={missing:,} invalid={invalid:,} "
            f"implausible={implausible:,} sources={source_text}"
        )


def print_absolute_table(pairs: Sequence[Pair], rails: Sequence[str], align: str) -> None:
    identity = "count" if align == "count" else "ordinal"
    header = [f"{identity:>7}"]
    for rail in rails:
        prefix = {"PPS": "p", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}[rail]
        header.extend(
            [
                f"{'old_' + prefix:>13}",
                f"{'new_' + prefix:>13}",
                f"{'d_' + prefix:>10}",
            ]
        )

    print("\nLiteral observed one-second cycle counts")
    print("════════════════════════════════════════")
    print("  d_* = NEW observed_cycles - OLD observed_cycles")
    print("  Campaign seconds are aligned by " + identity + "; they are not time-synchronous.\n")
    print("  ".join(header))
    print("  ".join("─" * len(field) for field in header))

    for pair in pairs:
        fields = [fmt_int(pair.key, 7)]
        for rail in rails:
            fields.extend(
                [
                    fmt_int(observed(pair.old, rail), 13),
                    fmt_int(observed(pair.new, rail), 13),
                    fmt_int(pair_delta(pair, rail), 10, signed=True),
                ]
            )
        print("  ".join(fields))


def print_relative_table(
    pairs: Sequence[Pair],
    rails: Sequence[str],
    reference: str,
    align: str,
) -> None:
    compared = [rail for rail in rails if rail != reference]
    if not compared:
        return

    identity = "count" if align == "count" else "ordinal"
    ref_prefix = {"PPS": "p", "VCLOCK": "v"}[reference]
    header = [f"{identity:>7}"]
    for rail in compared:
        prefix = {"PPS": "p", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}[rail]
        suffix = f"{prefix}-{ref_prefix}"
        header.extend(
            [
                f"{'old_' + suffix:>13}",
                f"{'new_' + suffix:>13}",
                f"{'d_' + suffix:>11}",
            ]
        )

    print(f"\nWithin-row normalization to {reference}")
    print("════════════════════════════════════")
    print(
        f"  d_(rail-{reference}) = "
        f"(NEW rail - NEW {reference}) - (OLD rail - OLD {reference})\n"
    )
    print("  ".join(header))
    print("  ".join("─" * len(field) for field in header))

    for pair in pairs:
        fields = [fmt_int(pair.key, 7)]
        for rail in compared:
            fields.extend(
                [
                    fmt_int(relative(pair.old, rail, reference), 13, signed=True),
                    fmt_int(relative(pair.new, rail, reference), 13, signed=True),
                    fmt_int(relative_shift(pair, rail, reference), 11, signed=True),
                ]
            )
        print("  ".join(fields))


def print_absolute_summary(pairs: Sequence[Pair], rails: Sequence[str]) -> None:
    print("\nPaired absolute comparison")
    print("══════════════════════════")
    print(
        "  rail       n          old_mean     old_sd          new_mean     new_sd  "
        "mean_delta  delta_sd   delta_min   delta_max"
    )
    print(
        "  ──────  ─────  ─────────────────  ─────────  ─────────────────  ─────────  "
        "──────────  ─────────  ──────────  ──────────"
    )

    for rail in rails:
        old_values: List[int] = []
        new_values: List[int] = []
        deltas: List[int] = []
        for pair in pairs:
            old_value = observed(pair.old, rail)
            new_value = observed(pair.new, rail)
            delta = subtract(new_value, old_value)
            if old_value is None or new_value is None or delta is None:
                continue
            old_values.append(old_value)
            new_values.append(new_value)
            deltas.append(delta)

        print(
            f"  {rail:<6}  {len(deltas):>5,d}  "
            f"{mean_or_nan(old_values):>17,.3f}  "
            f"{pstdev_or_nan(old_values):>9,.3f}  "
            f"{mean_or_nan(new_values):>17,.3f}  "
            f"{pstdev_or_nan(new_values):>9,.3f}  "
            f"{mean_or_nan(deltas):>+10,.3f}  "
            f"{pstdev_or_nan(deltas):>9,.3f}  "
            f"{min(deltas) if deltas else math.nan:>+10,.0f}  "
            f"{max(deltas) if deltas else math.nan:>+10,.0f}"
        )


def print_relative_summary(
    pairs: Sequence[Pair],
    rails: Sequence[str],
    reference: str,
) -> None:
    compared = [rail for rail in rails if rail != reference]
    if not compared:
        return

    print(f"\nPaired comparison normalized to {reference}")
    print("══════════════════════════════════════")
    print("  ppb_equiv is the DWT-cycle shift scaled by one second; it is not an oscillator-sign claim.")
    print(
        "  rail       n     old_rel_mean     new_rel_mean      mean_shift  "
        "shift_sd  shift_median   shift_MAD   ppb_equiv"
    )
    print(
        "  ──────  ─────  ───────────────  ───────────────  ─────────────  "
        "────────  ────────────  ──────────  ──────────"
    )

    pps_values = numeric(observed(pair.new, "PPS") for pair in pairs)
    dwt_reference = mean_or_nan(pps_values)

    for rail in compared:
        old_relative: List[int] = []
        new_relative: List[int] = []
        shifts: List[int] = []
        for pair in pairs:
            old_value = relative(pair.old, rail, reference)
            new_value = relative(pair.new, rail, reference)
            shift = subtract(new_value, old_value)
            if old_value is None or new_value is None or shift is None:
                continue
            old_relative.append(old_value)
            new_relative.append(new_value)
            shifts.append(shift)

        mean_shift = mean_or_nan(shifts)
        approx_ppb = (
            mean_shift * 1_000_000_000.0 / dwt_reference
            if shifts and math.isfinite(dwt_reference) and dwt_reference != 0.0
            else math.nan
        )
        print(
            f"  {rail:<6}  {len(shifts):>5,d}  "
            f"{mean_or_nan(old_relative):>+15,.3f}  "
            f"{mean_or_nan(new_relative):>+15,.3f}  "
            f"{mean_shift:>+13,.3f}  "
            f"{pstdev_or_nan(shifts):>8,.3f}  "
            f"{median_or_nan(shifts):>+12,.3f}  "
            f"{mad_or_nan(shifts):>10,.3f}  "
            f"{approx_ppb:>+10,.3f}"
        )


def write_csv(path: Path, pairs: Sequence[Pair], align: str) -> None:
    fieldnames = ["alignment_key", "old_count", "new_count"]
    for rail in RAILS:
        key = RAIL_KEYS[rail]
        fieldnames.extend(
            [
                f"old_{key}_cycles",
                f"new_{key}_cycles",
                f"delta_{key}_cycles",
                f"old_{key}_source",
                f"new_{key}_source",
                f"old_{key}_valid",
                f"new_{key}_valid",
            ]
        )
    for reference in ("PPS", "VCLOCK"):
        for rail in RAILS:
            if rail == reference:
                continue
            rail_key = RAIL_KEYS[rail]
            ref_key = RAIL_KEYS[reference]
            fieldnames.extend(
                [
                    f"old_{rail_key}_minus_{ref_key}",
                    f"new_{rail_key}_minus_{ref_key}",
                    f"shift_{rail_key}_minus_{ref_key}",
                ]
            )

    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for pair in pairs:
            record: Dict[str, Any] = {
                "alignment_key": pair.key,
                "old_count": pair.old.count,
                "new_count": pair.new.count,
            }
            for rail in RAILS:
                key = RAIL_KEYS[rail]
                record[f"old_{key}_cycles"] = observed(pair.old, rail)
                record[f"new_{key}_cycles"] = observed(pair.new, rail)
                record[f"delta_{key}_cycles"] = pair_delta(pair, rail)
                record[f"old_{key}_source"] = pair.old.rails[rail].source
                record[f"new_{key}_source"] = pair.new.rails[rail].source
                record[f"old_{key}_valid"] = pair.old.rails[rail].valid
                record[f"new_{key}_valid"] = pair.new.rails[rail].valid

            for reference in ("PPS", "VCLOCK"):
                for rail in RAILS:
                    if rail == reference:
                        continue
                    rail_key = RAIL_KEYS[rail]
                    ref_key = RAIL_KEYS[reference]
                    record[f"old_{rail_key}_minus_{ref_key}"] = relative(
                        pair.old, rail, reference
                    )
                    record[f"new_{rail_key}_minus_{ref_key}"] = relative(
                        pair.new, rail, reference
                    )
                    record[f"shift_{rail_key}_minus_{ref_key}"] = relative_shift(
                        pair, rail, reference
                    )
            writer.writerow(record)

    print(f"\nCSV written: {path}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare literal observed one-second cycle counts between two "
            "ZPNet campaigns."
        )
    )
    parser.add_argument("old_campaign")
    parser.add_argument("new_campaign")
    parser.add_argument(
        "--align",
        choices=("count", "ordinal"),
        default="count",
        help="Align rows by campaign count (default) or database ordinal.",
    )
    parser.add_argument(
        "--clock",
        action="append",
        type=str.upper,
        choices=RAILS,
        help="Restrict output to one or more rails; may be repeated.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Keep only the last N aligned seconds; zero prints all.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Also write the selected aligned comparison to this CSV file.",
    )
    parser.add_argument(
        "--no-rows",
        action="store_true",
        help="Print summaries only; still writes all rows when --csv is used.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    rails: Tuple[str, ...] = tuple(dict.fromkeys(args.clock)) if args.clock else RAILS

    old_rows = collect(args.old_campaign)
    new_rows = collect(args.new_campaign)
    pairs, notes = align_rows(old_rows, new_rows, args.align)

    if args.limit < 0:
        raise SystemExit("--limit must be zero or positive")
    if args.limit > 0:
        pairs = pairs[-args.limit:]

    print(
        f"Campaign comparison: OLD={args.old_campaign} ({len(old_rows):,} rows)  "
        f"NEW={args.new_campaign} ({len(new_rows):,} rows)"
    )
    print(f"Aligned rows used: {len(pairs):,}  alignment={args.align.upper()}")
    for note in notes:
        print(f"  NOTE: {note}")

    print("\nCampaign input audit")
    print("════════════════════")
    print_campaign_health("OLD", old_rows)
    print_campaign_health("NEW", new_rows)

    if not pairs:
        raise SystemExit("No aligned campaign rows were available for comparison.")

    if not args.no_rows:
        print_absolute_table(pairs, rails, args.align)
        print_relative_table(pairs, rails, "PPS", args.align)
        if "VCLOCK" in rails:
            print_relative_table(pairs, rails, "VCLOCK", args.align)

    print_absolute_summary(pairs, rails)
    print_relative_summary(pairs, rails, "PPS")
    print_relative_summary(pairs, rails, "VCLOCK")

    if args.csv is not None:
        write_csv(args.csv, pairs, args.align)


if __name__ == "__main__":
    main(sys.argv[1:])
