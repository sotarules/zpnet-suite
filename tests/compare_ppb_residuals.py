"""Compare firmware-published PPB/residual surfaces between two ZPNet campaigns.

This report aligns campaign rows by campaign PPS identity and prints, for DWT,
OCXO1, and OCXO2:

* one-second residual sample (ns == ppb over one second), when published;
* firmware campaign PPB, when published;
* NEW - OLD for each surface;
* the exact JSON source path selected for each campaign/schema;
* OCXO Delta Cycles, CounterLedger, PhaseLedger, refined, and published
  residual surfaces on the same aligned campaign row;
* arithmetic checks showing where the residuals agree or diverge.

For OCXO lanes the preferred instantaneous source is the PPS-founded residual
surface, followed by current Delta Raw science fields and narrow legacy
fallbacks.  The evidence section is deliberately non-authoritative: it exposes
all available residual antecedents without choosing a winner.  For DWT the
preferred instantaneous source is a published DWT residual/offset sample; when
absent, the report independently computes the one-second DWT frequency sample
from the observed PPS interval against the nominal 1.008 GHz CPU clock.

The program does not smooth, repair, normalize, or infer campaign state.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

CLOCKS: Tuple[str, ...] = ("DWT", "OCXO1", "OCXO2")
LANE_KEYS = {"DWT": "dwt", "OCXO1": "ocxo1", "OCXO2": "ocxo2"}
DWT_NOMINAL_CYCLES_PER_SECOND = 1_008_000_000.0
OCXO_NOMINAL_TICKS_PER_SECOND = 10_000_000.0
OCXO_NS_PER_TICK = 100.0


@dataclass(frozen=True)
class Value:
    value: Optional[float]
    source: str


@dataclass(frozen=True)
class ClockSample:
    residual: Value
    ppb: Value


@dataclass(frozen=True)
class OcxoEvidence:
    delta_residual: Value
    counter_ticks: Value
    counter_residual: Value
    phase_raw_delta: Value
    phase_unwrapped_delta: Value
    phase_carry_ticks: Value
    phase_after_ns: Value
    phase_wrap_event: Value
    refined_interval: Value
    refined_residual: Value


@dataclass(frozen=True)
class CampaignRow:
    ordinal: int
    count: Optional[int]
    samples: Dict[str, ClockSample]
    ocxo_evidence: Dict[str, OcxoEvidence]


@dataclass(frozen=True)
class Pair:
    key: int
    old: CampaignRow
    new: CampaignRow


def d(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def nested(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def as_float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return out if math.isfinite(out) else None


def as_int(value: Any) -> Optional[int]:
    parsed = as_float(value)
    if parsed is None:
        return None
    return int(parsed)


def first_value(candidates: Iterable[Tuple[str, Any]]) -> Value:
    for source, raw in candidates:
        parsed = as_float(raw)
        if parsed is not None:
            return Value(parsed, source)
    return Value(None, "MISSING")


def first_flag(candidates: Iterable[Tuple[str, Any]]) -> Value:
    for source, raw in candidates:
        if isinstance(raw, bool):
            return Value(1.0 if raw else 0.0, source)
        parsed = as_float(raw)
        if parsed is not None:
            return Value(1.0 if parsed != 0.0 else 0.0, source)
    return Value(None, "MISSING")


def computed_value(value: Optional[float], source: str) -> Value:
    return Value(value, source) if value is not None else Value(None, "MISSING")


def root_of(record: Dict[str, Any]) -> Dict[str, Any]:
    inner = record.get("payload") if isinstance(record, dict) else None
    return d(inner) or d(record)


def fragment_of(record: Dict[str, Any]) -> Dict[str, Any]:
    root = root_of(record)
    return d(root.get("fragment")) or root


def count_of(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    for raw in (
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
        frag.get("campaign_seconds"),
    ):
        parsed = as_int(raw)
        if parsed is not None:
            return parsed
    return None


def fetch(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
              FROM timebase
             WHERE campaign = %s
             ORDER BY COALESCE(
                 NULLIF(jsonb_extract_path_text(payload::jsonb, 'pps_count'), '')::bigint,
                 NULLIF(jsonb_extract_path_text(payload::jsonb, 'teensy_pps_vclock_count'), '')::bigint,
                 NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'pps_count'), '')::bigint,
                 NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'teensy_pps_vclock_count'), '')::bigint,
                 NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'teensy_pps_count'), '')::bigint,
                 NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'campaign_seconds'), '')::bigint
             ), payload::text
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


def observed_pps_cycles(root: Dict[str, Any], frag: Dict[str, Any]) -> Value:
    return first_value((
        ("fragment.raw_cycles.pps.observed_cycles", nested(frag, "raw_cycles", "pps", "observed_cycles")),
        ("fragment.pps.dwt_cycles_between_edges", nested(frag, "pps", "dwt_cycles_between_edges")),
        ("fragment.dwt.cycles_between_pps_vclock", nested(frag, "dwt", "cycles_between_pps_vclock")),
        ("fragment.dwt_cycles_between_pps_vclock", frag.get("dwt_cycles_between_pps_vclock")),
        ("root.dwt_cycles_between_pps_vclock", root.get("dwt_cycles_between_pps_vclock")),
    ))


def dwt_residual(root: Dict[str, Any], frag: Dict[str, Any]) -> Value:
    published = first_value((
        ("fragment.dwt.residual_ppb", nested(frag, "dwt", "residual_ppb")),
        ("fragment.dwt.residual_ns", nested(frag, "dwt", "residual_ns")),
        ("fragment.dwt.pps_residual.fast_residual_ns", nested(frag, "dwt", "pps_residual", "fast_residual_ns")),
        ("fragment.dwt.fast_residual_ns", nested(frag, "dwt", "fast_residual_ns")),
        ("fragment.stats.dwt.last", nested(frag, "stats", "dwt", "last")),
        ("fragment.stats.dwt.residual", nested(frag, "stats", "dwt", "residual")),
        ("fragment.dwt_welford_last", frag.get("dwt_welford_last")),
    ))
    if published.value is not None:
        return published

    cyc = observed_pps_cycles(root, frag)
    if cyc.value is None:
        return Value(None, "MISSING")
    sample_ppb = ((cyc.value / DWT_NOMINAL_CYCLES_PER_SECOND) - 1.0) * 1_000_000_000.0
    return Value(sample_ppb, f"COMPUTED_FROM:{cyc.source}")


def ocxo_residual(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Value:
    return first_value((
        (f"fragment.{lane}.pps_residual.fast_residual_ns", nested(frag, lane, "pps_residual", "fast_residual_ns")),
        (f"fragment.{lane}.science.delta_raw_fast_residual_ns", nested(frag, lane, "science", "delta_raw_fast_residual_ns")),
        (f"fragment.{lane}.science.fast_residual_ns", nested(frag, lane, "science", "fast_residual_ns")),
        (f"fragment.{lane}.counterledger.refined_fast_residual_ns", nested(frag, lane, "counterledger", "refined_fast_residual_ns")),
        (f"fragment.{lane}.counterledger.fast_residual_ns", nested(frag, lane, "counterledger", "fast_residual_ns")),
        (f"fragment.{lane}.fast_residual_ns", nested(frag, lane, "fast_residual_ns")),
        (f"root.fragment.{lane}.pps_residual.fast_residual_ns", nested(root, "fragment", lane, "pps_residual", "fast_residual_ns")),
    ))


def published_ppb(root: Dict[str, Any], frag: Dict[str, Any], clock: str) -> Value:
    lane = LANE_KEYS[clock]
    return first_value((
        (f"fragment.stats.{lane}.ppb", nested(frag, "stats", lane, "ppb")),
        (f"fragment.{lane}.ppb", nested(frag, lane, "ppb")),
        (f"root.stats.{lane}.ppb", nested(root, "stats", lane, "ppb")),
        (f"root.fragment.stats.{lane}.ppb", nested(root, "fragment", "stats", lane, "ppb")),
        (f"fragment.{lane}_ppb", frag.get(f"{lane}_ppb")),
        (f"fragment.{lane}_welford_mean", frag.get(f"{lane}_welford_mean")),
    ))


def ocxo_evidence(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> OcxoEvidence:
    counter_ticks = first_value((
        (f"fragment.{lane}.counterledger.interval_ticks",
         nested(frag, lane, "counterledger", "interval_ticks")),
        (f"fragment.{lane}.counterledger.last_delta_ticks",
         nested(frag, lane, "counterledger", "last_delta_ticks")),
    ))

    counter_residual = first_value((
        (f"fragment.{lane}.counterledger.fast_residual_ns",
         nested(frag, lane, "counterledger", "fast_residual_ns")),
    ))
    if counter_residual.value is None and counter_ticks.value is not None:
        counter_residual = computed_value(
            (counter_ticks.value - OCXO_NOMINAL_TICKS_PER_SECOND) * OCXO_NS_PER_TICK,
            f"COMPUTED_FROM:{counter_ticks.source}",
        )

    return OcxoEvidence(
        delta_residual=first_value((
            (f"fragment.{lane}.science.delta_raw_fast_residual_ns_exact",
             nested(frag, lane, "science", "delta_raw_fast_residual_ns_exact")),
            (f"fragment.{lane}.science.delta_raw_fast_residual_ns",
             nested(frag, lane, "science", "delta_raw_fast_residual_ns")),
            (f"fragment.{lane}.science.fast_residual_ns_exact",
             nested(frag, lane, "science", "fast_residual_ns_exact")),
            (f"fragment.{lane}.science.fast_residual_ns",
             nested(frag, lane, "science", "fast_residual_ns")),
        )),
        counter_ticks=counter_ticks,
        counter_residual=counter_residual,
        phase_raw_delta=first_value((
            (f"fragment.{lane}.counterledger.phaseledger.raw_delta_ns",
             nested(frag, lane, "counterledger", "phaseledger", "raw_delta_ns")),
        )),
        phase_unwrapped_delta=first_value((
            (f"fragment.{lane}.counterledger.phaseledger.unwrapped_delta_ns",
             nested(frag, lane, "counterledger", "phaseledger", "unwrapped_delta_ns")),
        )),
        phase_carry_ticks=first_value((
            (f"fragment.{lane}.counterledger.phaseledger.unwrapped_carry_ticks",
             nested(frag, lane, "counterledger", "phaseledger", "unwrapped_carry_ticks")),
        )),
        phase_after_ns=first_value((
            (f"fragment.{lane}.counterledger.phaseledger.phase_after_last_00_ns",
             nested(frag, lane, "counterledger", "phaseledger", "phase_after_last_00_ns")),
        )),
        phase_wrap_event=first_flag((
            (f"fragment.{lane}.counterledger.phaseledger.wrap_event",
             nested(frag, lane, "counterledger", "phaseledger", "wrap_event")),
        )),
        refined_interval=first_value((
            (f"fragment.{lane}.counterledger.refined_interval_ns",
             nested(frag, lane, "counterledger", "refined_interval_ns")),
        )),
        refined_residual=first_value((
            (f"fragment.{lane}.counterledger.refined_fast_residual_ns",
             nested(frag, lane, "counterledger", "refined_fast_residual_ns")),
        )),
    )


def collect(campaign: str) -> List[CampaignRow]:
    rows: List[CampaignRow] = []
    for ordinal, record in enumerate(fetch(campaign), start=1):
        root = root_of(record)
        frag = fragment_of(record)
        samples: Dict[str, ClockSample] = {}
        evidence: Dict[str, OcxoEvidence] = {}
        for clock in CLOCKS:
            residual = dwt_residual(root, frag) if clock == "DWT" else ocxo_residual(root, frag, LANE_KEYS[clock])
            samples[clock] = ClockSample(
                residual=residual,
                ppb=published_ppb(root, frag, clock),
            )
            if clock != "DWT":
                evidence[clock] = ocxo_evidence(root, frag, LANE_KEYS[clock])
        rows.append(CampaignRow(ordinal, count_of(root, frag), samples, evidence))
    return rows


def duplicate_counts(rows: Sequence[CampaignRow]) -> List[int]:
    counts = Counter(row.count for row in rows if row.count is not None)
    return sorted(key for key, n in counts.items() if n > 1)


def align(old_rows: Sequence[CampaignRow], new_rows: Sequence[CampaignRow], mode: str) -> List[Pair]:
    if mode == "ordinal":
        return [Pair(index + 1, old_rows[index], new_rows[index]) for index in range(min(len(old_rows), len(new_rows)))]

    old_dups = duplicate_counts(old_rows)
    new_dups = duplicate_counts(new_rows)
    if old_dups or new_dups:
        raise SystemExit(f"duplicate campaign counts prevent count alignment: OLD={old_dups} NEW={new_dups}")
    old_index = {row.count: row for row in old_rows if row.count is not None}
    new_index = {row.count: row for row in new_rows if row.count is not None}
    return [Pair(key, old_index[key], new_index[key]) for key in sorted(set(old_index) & set(new_index))]


def delta(a: Optional[float], b: Optional[float]) -> Optional[float]:
    return None if a is None or b is None else b - a


def fmt(value: Optional[float], width: int = 11, decimals: int = 3) -> str:
    return f"{'---':>{width}}" if value is None else f"{value:+{width},.{decimals}f}"


def describe(values: Sequence[float]) -> Tuple[int, float, float, float, float, float]:
    if not values:
        return 0, math.nan, math.nan, math.nan, math.nan, math.nan
    return (
        len(values),
        statistics.fmean(values),
        statistics.stdev(values) if len(values) > 1 else 0.0,
        statistics.median(values),
        min(values),
        max(values),
    )


def source_audit(name: str, rows: Sequence[CampaignRow]) -> None:
    print(f"  {name}")
    for clock in CLOCKS:
        res = Counter(row.samples[clock].residual.source for row in rows)
        ppb = Counter(row.samples[clock].ppb.source for row in rows)
        print(f"    {clock:<6} residual_sources={dict(res)}")
        print(f"           ppb_sources={dict(ppb)}")


EVIDENCE_FIELDS: Tuple[Tuple[str, str], ...] = (
    ("delta", "delta_residual"),
    ("ticks", "counter_ticks"),
    ("integer", "counter_residual"),
    ("phase_raw", "phase_raw_delta"),
    ("phase_unwrapped", "phase_unwrapped_delta"),
    ("carry", "phase_carry_ticks"),
    ("phase_after", "phase_after_ns"),
    ("wrap", "phase_wrap_event"),
    ("refined_interval", "refined_interval"),
    ("refined", "refined_residual"),
)


def evidence_coverage(name: str, rows: Sequence[CampaignRow]) -> None:
    print(f"  {name}")
    for clock in ("OCXO1", "OCXO2"):
        parts: List[str] = []
        for label, attr in EVIDENCE_FIELDS:
            present = sum(
                1 for row in rows
                if getattr(row.ocxo_evidence[clock], attr).value is not None
            )
            parts.append(f"{label}={present:,}/{len(rows):,}")
        print(f"    {clock:<6} " + "  ".join(parts))


def value_of(row: CampaignRow, clock: str, attr: str) -> Optional[float]:
    if attr == "published":
        return row.samples[clock].residual.value
    if attr == "ppb":
        return row.samples[clock].ppb.value
    return getattr(row.ocxo_evidence[clock], attr).value


def subtract_values(lhs: Optional[float], rhs: Optional[float]) -> Optional[float]:
    return None if lhs is None or rhs is None else lhs - rhs


def refined_component_error(row: CampaignRow, clock: str) -> Optional[float]:
    e = row.ocxo_evidence[clock]
    if e.refined_residual.value is None or e.counter_residual.value is None or e.phase_raw_delta.value is None:
        return None
    return e.refined_residual.value - (e.counter_residual.value + e.phase_raw_delta.value)


def evidence_check(row: CampaignRow, clock: str, check: str) -> Optional[float]:
    e = row.ocxo_evidence[clock]
    if check == "refined_minus_components":
        return refined_component_error(row, clock)
    if check == "delta_minus_refined":
        return subtract_values(e.delta_residual.value, e.refined_residual.value)
    if check == "published_minus_delta":
        return subtract_values(row.samples[clock].residual.value, e.delta_residual.value)
    raise ValueError(f"unknown evidence check: {check}")


def fmt_flag(value: Optional[float], width: int = 5) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{('Y' if value != 0.0 else 'N'):>{width}}"


def print_ocxo_evidence_table(clock: str, pairs: Sequence[Pair]) -> None:
    short = "O1" if clock == "OCXO1" else "O2"
    print(f"\n{clock} aligned evidence decomposition")
    print("═" * (len(clock) + 31))
    print("  pub=selected pps_residual; delta=Delta Cycles exact; int=CounterLedger whole-tick residual")
    print("  phr/phu=raw/unwrapped PhaseLedger delta; ref=refined residual")
    print("  rΣ=ref-(int+phr); d-r=delta-ref; p-d=pub-delta; carry is cumulative; wrap is this row\n")

    columns = [
        "count",
        "old_pub", "old_del", "old_int", "old_phr", "old_phu", "old_ref",
        "new_pub", "new_del", "new_int", "new_phr", "new_phu", "new_ref",
        "rΣ", "d-r", "p-d", "carry", "wrap",
    ]
    print("  ".join(f"{column:>9}" for column in columns))
    print("  ".join("─" * 9 for _ in columns))

    for pair in pairs:
        old = pair.old.ocxo_evidence[clock]
        new = pair.new.ocxo_evidence[clock]
        fields = [f"{pair.key:>9,d}"]
        for row, evidence in ((pair.old, old), (pair.new, new)):
            fields.extend([
                fmt(row.samples[clock].residual.value, 9, 3),
                fmt(evidence.delta_residual.value, 9, 3),
                fmt(evidence.counter_residual.value, 9, 3),
                fmt(evidence.phase_raw_delta.value, 9, 3),
                fmt(evidence.phase_unwrapped_delta.value, 9, 3),
                fmt(evidence.refined_residual.value, 9, 3),
            ])
        fields.extend([
            fmt(evidence_check(pair.new, clock, "refined_minus_components"), 9, 3),
            fmt(evidence_check(pair.new, clock, "delta_minus_refined"), 9, 3),
            fmt(evidence_check(pair.new, clock, "published_minus_delta"), 9, 3),
            fmt(new.phase_carry_ticks.value, 9, 0),
            fmt_flag(new.phase_wrap_event.value, 9),
        ])
        print("  ".join(fields))


def finite_values(values: Iterable[Optional[float]]) -> List[float]:
    return [value for value in values if value is not None and math.isfinite(value)]


def summary_line(label: str, values: Sequence[float]) -> str:
    if not values:
        return f"{label:<24} n=    0  mean=         ---  sd=      ---  range=[---,---]"
    n, mean, sd, _, minimum, maximum = describe(values)
    return (
        f"{label:<24} n={n:>5,d}  mean={mean:+12.3f}  "
        f"sd={sd:9.3f}  range=[{minimum:+.3f},{maximum:+.3f}]"
    )


def print_ocxo_evidence_summary(clock: str, pairs: Sequence[Pair]) -> None:
    print(f"  {clock}")
    surfaces = (
        ("published", "published"),
        ("Delta Cycles", "delta_residual"),
        ("integer CounterLedger", "counter_residual"),
        ("PhaseLedger raw", "phase_raw_delta"),
        ("PhaseLedger unwrapped", "phase_unwrapped_delta"),
        ("refined", "refined_residual"),
    )
    for label, attr in surfaces:
        old_values = finite_values(value_of(pair.old, clock, attr) for pair in pairs)
        new_values = finite_values(value_of(pair.new, clock, attr) for pair in pairs)
        print(summary_line(f"OLD {label}", old_values))
        print(summary_line(f"NEW {label}", new_values))

    checks = (
        ("NEW ref-(int+phase)", "refined_minus_components"),
        ("NEW delta-refined", "delta_minus_refined"),
        ("NEW published-delta", "published_minus_delta"),
    )
    for label, check in checks:
        values = finite_values(evidence_check(pair.new, clock, check) for pair in pairs)
        print(summary_line(label, values))


def write_csv(path: Path, pairs: Sequence[Pair]) -> None:
    fields = ["count"]
    for clock in CLOCKS:
        p = clock.lower()
        fields.extend([
            f"old_{p}_residual", f"new_{p}_residual", f"delta_{p}_residual",
            f"old_{p}_ppb", f"new_{p}_ppb", f"delta_{p}_ppb",
            f"old_{p}_residual_source", f"new_{p}_residual_source",
            f"old_{p}_ppb_source", f"new_{p}_ppb_source",
        ])
        if clock != "DWT":
            for label, _ in EVIDENCE_FIELDS:
                fields.extend([
                    f"old_{p}_{label}", f"new_{p}_{label}",
                    f"old_{p}_{label}_source", f"new_{p}_{label}_source",
                ])
            fields.extend([
                f"new_{p}_refined_minus_components",
                f"new_{p}_delta_minus_refined",
                f"new_{p}_published_minus_delta",
            ])

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for pair in pairs:
            row: Dict[str, Any] = {"count": pair.key}
            for clock in CLOCKS:
                p = clock.lower()
                old = pair.old.samples[clock]
                new = pair.new.samples[clock]
                row.update({
                    f"old_{p}_residual": old.residual.value,
                    f"new_{p}_residual": new.residual.value,
                    f"delta_{p}_residual": delta(old.residual.value, new.residual.value),
                    f"old_{p}_ppb": old.ppb.value,
                    f"new_{p}_ppb": new.ppb.value,
                    f"delta_{p}_ppb": delta(old.ppb.value, new.ppb.value),
                    f"old_{p}_residual_source": old.residual.source,
                    f"new_{p}_residual_source": new.residual.source,
                    f"old_{p}_ppb_source": old.ppb.source,
                    f"new_{p}_ppb_source": new.ppb.source,
                })
                if clock != "DWT":
                    old_e = pair.old.ocxo_evidence[clock]
                    new_e = pair.new.ocxo_evidence[clock]
                    for label, attr in EVIDENCE_FIELDS:
                        old_value = getattr(old_e, attr)
                        new_value = getattr(new_e, attr)
                        row.update({
                            f"old_{p}_{label}": old_value.value,
                            f"new_{p}_{label}": new_value.value,
                            f"old_{p}_{label}_source": old_value.source,
                            f"new_{p}_{label}_source": new_value.source,
                        })
                    row.update({
                        f"new_{p}_refined_minus_components":
                            evidence_check(pair.new, clock, "refined_minus_components"),
                        f"new_{p}_delta_minus_refined":
                            evidence_check(pair.new, clock, "delta_minus_refined"),
                        f"new_{p}_published_minus_delta":
                            evidence_check(pair.new, clock, "published_minus_delta"),
                    })
            writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("old_campaign")
    parser.add_argument("new_campaign")
    parser.add_argument("--align", choices=("count", "ordinal"), default="count")
    parser.add_argument("--limit", type=int, default=0, help="show only the last N aligned rows; summary still uses all rows")
    parser.add_argument("--csv", type=Path)
    parser.add_argument(
        "--evidence-only",
        action="store_true",
        help="suppress the legacy residual/PPB table and print only OCXO decomposition evidence",
    )
    parser.add_argument(
        "--no-evidence",
        action="store_true",
        help="suppress the OCXO decomposition sections and retain the legacy report shape",
    )
    args = parser.parse_args()
    if args.evidence_only and args.no_evidence:
        parser.error("--evidence-only and --no-evidence are mutually exclusive")

    old_rows = collect(args.old_campaign)
    new_rows = collect(args.new_campaign)
    pairs = align(old_rows, new_rows, args.align)

    print(f"Campaign PPB/residual comparison: OLD={args.old_campaign} ({len(old_rows):,})  NEW={args.new_campaign} ({len(new_rows):,})")
    print(f"Aligned rows: {len(pairs):,}  alignment={args.align.upper()}\n")
    print("Source audit")
    print("════════════")
    source_audit("OLD", old_rows)
    source_audit("NEW", new_rows)

    if not args.no_evidence:
        print("\nOCXO evidence coverage")
        print("══════════════════════")
        evidence_coverage("OLD", old_rows)
        evidence_coverage("NEW", new_rows)

    display_pairs = pairs[-args.limit:] if args.limit > 0 else pairs

    if not args.evidence_only:
        print("\nPer-row residual and firmware PPB")
        print("═════════════════════════════════")
        print("  residual: one-second sample, ns == ppb over that second")
        print("  ppb: firmware-published campaign PPB")
        print("  every delta is NEW - OLD\n")
        header = [f"{'count':>7}"]
        for clock in CLOCKS:
            p = {"DWT": "d", "OCXO1": "o1", "OCXO2": "o2"}[clock]
            header += [f"old_{p}r", f"new_{p}r", f"d_{p}r", f"old_{p}p", f"new_{p}p", f"d_{p}p"]
        print("  ".join(f"{x:>11}" for x in header))
        print("  ".join("─" * 11 for _ in header))
        for pair in display_pairs:
            fields = [f"{pair.key:>11,d}"]
            for clock in CLOCKS:
                old = pair.old.samples[clock]
                new = pair.new.samples[clock]
                fields.extend([
                    fmt(old.residual.value), fmt(new.residual.value), fmt(delta(old.residual.value, new.residual.value)),
                    fmt(old.ppb.value), fmt(new.ppb.value), fmt(delta(old.ppb.value, new.ppb.value)),
                ])
            print("  ".join(fields))

        print("\nSummary of paired values")
        print("════════════════════════")
        for clock in CLOCKS:
            print(f"  {clock}")
            for surface in ("residual", "ppb"):
                old_vals: List[float] = []
                new_vals: List[float] = []
                diffs: List[float] = []
                for pair in pairs:
                    old_v = getattr(pair.old.samples[clock], surface).value
                    new_v = getattr(pair.new.samples[clock], surface).value
                    if old_v is not None and new_v is not None:
                        old_vals.append(old_v)
                        new_vals.append(new_v)
                        diffs.append(new_v - old_v)
                n, old_mean, old_sd, _, old_min, old_max = describe(old_vals)
                _, new_mean, new_sd, _, new_min, new_max = describe(new_vals)
                _, diff_mean, diff_sd, diff_med, diff_min, diff_max = describe(diffs)
                print(
                    f"    {surface:<8} n={n:>5,d}  "
                    f"OLD mean={old_mean:+12.3f} sd={old_sd:9.3f} range=[{old_min:+.3f},{old_max:+.3f}]  "
                    f"NEW mean={new_mean:+12.3f} sd={new_sd:9.3f} range=[{new_min:+.3f},{new_max:+.3f}]"
                )
                print(
                    f"             NEW-OLD mean={diff_mean:+12.3f} sd={diff_sd:9.3f} "
                    f"median={diff_med:+9.3f} range=[{diff_min:+.3f},{diff_max:+.3f}]"
                )

    if not args.no_evidence:
        for clock in ("OCXO1", "OCXO2"):
            print_ocxo_evidence_table(clock, display_pairs)
        print("\nOCXO evidence summary")
        print("═════════════════════")
        for clock in ("OCXO1", "OCXO2"):
            print_ocxo_evidence_summary(clock, pairs)

    if args.csv:
        write_csv(args.csv, pairs)
        print(f"\nCSV written: {args.csv}")


if __name__ == "__main__":
    main()
