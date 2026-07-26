"""ZPNet campaign-inception published-residual audit.

This program asks one tightly scoped scientific question:

    Does the first authoritative OCXO PPS residual, in nanoseconds, create a
    lasting bias in the Welford population that is published by the firmware?

Cycle counts are intentionally not used as the primary measurement surface.
They are antecedent forensics only.  The population audited here is exactly the
published per-second residual admitted by the firmware:

    fragment.ocxoN.pps_residual.fast_residual_ns

and the sovereign admission gate is:

    fragment.ocxoN.pps_residual.valid

The program independently replays Welford from those admitted nanosecond
samples, checks the replay against the firmware-published Welford trajectory,
and computes the counterfactual trajectory with the first admitted residual
removed.

Usage:
    campaign_residual_inception_bias CAMPAIGN
    campaign_residual_inception_bias CAMPAIGN --clock OCXO1
    campaign_residual_inception_bias CAMPAIGN --window 20
    campaign_residual_inception_bias CAMPAIGN --counterpart-gate-ns 8

The database is opened read-only by convention; this program performs SELECTs
only.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

LANES: Tuple[str, ...] = ("ocxo1", "ocxo2")


@dataclass
class ReplayWelford:
    n: int = 0
    mean: float = 0.0
    m2: float = 0.0

    def update(self, value: float) -> None:
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        delta2 = value - self.mean
        self.m2 += delta * delta2

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0


@dataclass(frozen=True)
class ResidualRow:
    pps_count: int
    db_id: int
    db_ts: str
    valid: Optional[bool]
    residual_ns: Optional[int]
    clock_interval_ns: Optional[int]
    gnss_interval_ns: Optional[int]
    published_n: Optional[int]
    published_mean: Optional[float]
    published_stddev: Optional[float]


@dataclass(frozen=True)
class LaneResult:
    lane: str
    rows: Tuple[ResidualRow, ...]
    accepted: Tuple[ResidualRow, ...]
    first: Optional[ResidualRow]
    replay: ReplayWelford
    counterfactual: ReplayWelford
    final_mean_shift_ns: Optional[float]
    final_ppb_shift: Optional[float]
    single_counterpart: Optional[Tuple[int, int, int]]
    cumulative_neutralization: Optional[Tuple[int, int]]
    trajectory_mismatches: Tuple[str, ...]


def _dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _path(obj: Dict[str, Any], dotted: str, default: Any = None) -> Any:
    current: Any = obj
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return default if current is None else current


def _int(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in ("true", "1", "yes", "on"):
        return True
    if text in ("false", "0", "no", "off"):
        return False
    return None


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = _int(value)
        if parsed is not None:
            return parsed
    return None


def _first_float(*values: Any) -> Optional[float]:
    for value in values:
        parsed = _float(value)
        if parsed is not None:
            return parsed
    return None


def _fragment(row: Dict[str, Any]) -> Dict[str, Any]:
    return _dict(row.get("fragment"))


def _pps_count(row: Dict[str, Any]) -> int:
    frag = _fragment(row)
    value = _first_int(
        row.get("teensy_pps_vclock_count"),
        row.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
    )
    if value is None:
        raise ValueError(f"TIMEBASE row missing PPS identity: db_id={row.get('_db_id')}")
    return value


def _residual(row: Dict[str, Any], lane: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path(frag, f"{lane}.pps_residual.fast_residual_ns"),
        _path(frag, f"{lane}.measurement.second_residual_ns"),
        frag.get(f"{lane}_second_residual_ns"),
        row.get(f"{lane}_second_residual_ns"),
    )


def _valid(row: Dict[str, Any], lane: str) -> Optional[bool]:
    frag = _fragment(row)
    explicit = _bool(_path(frag, f"{lane}.pps_residual.valid"))
    if explicit is not None:
        return explicit
    science = _dict(_path(frag, f"{lane}.science"))
    science_valid = _bool(science.get("valid"))
    if science_valid is not None:
        return science_valid
    return None


def _clock_interval(row: Dict[str, Any], lane: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path(frag, f"{lane}.pps_residual.clock_interval_ns"),
        _path(frag, f"{lane}.measurement.clock_interval_ns"),
        _path(frag, f"{lane}.interval.clock_interval_ns"),
        frag.get(f"{lane}_clock_interval_ns"),
    )


def _gnss_interval(row: Dict[str, Any], lane: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path(frag, f"{lane}.pps_residual.gnss_interval_ns"),
        _path(frag, f"{lane}.measurement.gnss_ns_between_edges"),
        _path(frag, f"{lane}.interval.gnss_ns_between_edges"),
        frag.get(f"{lane}_gnss_ns_between_edges"),
    )


def _welford_int(row: Dict[str, Any], lane: str, field: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path(frag, f"stats.{lane}.welford.{field}"),
        _path(frag, f"stats.{lane}.{field}"),
        frag.get(f"{lane}_welford_{field}"),
        row.get(f"{lane}_welford_{field}"),
    )


def _welford_float(row: Dict[str, Any], lane: str, field: str) -> Optional[float]:
    frag = _fragment(row)
    return _first_float(
        _path(frag, f"stats.{lane}.welford.{field}"),
        _path(frag, f"stats.{lane}.{field}"),
        frag.get(f"{lane}_welford_{field}"),
        row.get(f"{lane}_welford_{field}"),
    )


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                       NULLIF(payload->>'teensy_pps_vclock_count', '')::bigint,
                       NULLIF(payload->>'pps_count', '')::bigint,
                       NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                       NULLIF(payload->'fragment'->>'pps_count', '')::bigint,
                       id::bigint
                     ) ASC,
                     id ASC
            """,
            (campaign,),
        )
        records = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for record in records:
        payload = record["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if not isinstance(payload, dict):
            continue
        payload["_db_id"] = int(record["id"])
        payload["_db_ts"] = str(record["ts"])
        result.append(payload)
    return result


def extract_rows(records: Sequence[Dict[str, Any]], lane: str) -> Tuple[ResidualRow, ...]:
    rows: List[ResidualRow] = []
    for record in records:
        rows.append(
            ResidualRow(
                pps_count=_pps_count(record),
                db_id=int(record.get("_db_id", 0)),
                db_ts=str(record.get("_db_ts", "")),
                valid=_valid(record, lane),
                residual_ns=_residual(record, lane),
                clock_interval_ns=_clock_interval(record, lane),
                gnss_interval_ns=_gnss_interval(record, lane),
                published_n=_welford_int(record, lane, "n"),
                published_mean=_welford_float(record, lane, "mean"),
                published_stddev=_welford_float(record, lane, "stddev"),
            )
        )
    return tuple(rows)


def admitted(row: ResidualRow) -> bool:
    return row.valid is True and row.residual_ns is not None


def find_single_counterpart(
    first: int,
    later: Sequence[ResidualRow],
    gate_ns: int,
) -> Optional[Tuple[int, int, int]]:
    candidates = [
        row for row in later
        if row.residual_ns is not None
        and row.residual_ns != 0
        and (row.residual_ns > 0) != (first > 0)
    ]
    if not candidates:
        return None
    best = min(candidates, key=lambda row: abs(first + int(row.residual_ns or 0)))
    pair_sum = first + int(best.residual_ns or 0)
    if abs(pair_sum) <= gate_ns:
        return best.pps_count, int(best.residual_ns or 0), pair_sum
    return None


def find_cumulative_neutralization(
    accepted: Sequence[ResidualRow],
    gate_ns: int,
) -> Optional[Tuple[int, int]]:
    running = 0
    for row in accepted:
        running += int(row.residual_ns or 0)
        if abs(running) <= gate_ns:
            return row.pps_count, running
    return None


def audit_lane(
    records: Sequence[Dict[str, Any]],
    lane: str,
    counterpart_gate_ns: int,
    mean_tolerance_ns: float,
    sd_tolerance_ns: float,
) -> LaneResult:
    rows = extract_rows(records, lane)
    accepted = tuple(row for row in rows if admitted(row))
    first = accepted[0] if accepted else None

    replay = ReplayWelford()
    counterfactual = ReplayWelford()
    mismatches: List[str] = []
    accepted_index = 0

    for row in rows:
        if admitted(row):
            accepted_index += 1
            value = float(int(row.residual_ns or 0))
            replay.update(value)
            if accepted_index > 1:
                counterfactual.update(value)

        # Check only rows that publish a Welford snapshot.  A held/invalid row may
        # legitimately carry the unchanged population, which replay also reflects.
        if row.published_n is not None:
            if row.published_n != replay.n:
                mismatches.append(
                    f"pps={row.pps_count}: published n={row.published_n}, replay n={replay.n}"
                )
            if row.published_mean is not None and abs(row.published_mean - replay.mean) > mean_tolerance_ns:
                mismatches.append(
                    f"pps={row.pps_count}: published mean={row.published_mean:+.9f}, "
                    f"replay mean={replay.mean:+.9f} ns"
                )
            if row.published_stddev is not None and abs(row.published_stddev - replay.stddev) > sd_tolerance_ns:
                mismatches.append(
                    f"pps={row.pps_count}: published sd={row.published_stddev:.9f}, "
                    f"replay sd={replay.stddev:.9f} ns"
                )

    shift = replay.mean - counterfactual.mean if replay.n and counterfactual.n else None
    ppb_shift = shift if shift is not None else None  # 1 ns per nominal second == 1 ppb.

    single = None
    cumulative = None
    if first is not None and first.residual_ns is not None:
        single = find_single_counterpart(
            int(first.residual_ns),
            accepted[1:],
            counterpart_gate_ns,
        )
        cumulative = find_cumulative_neutralization(accepted, counterpart_gate_ns)

    return LaneResult(
        lane=lane,
        rows=rows,
        accepted=accepted,
        first=first,
        replay=replay,
        counterfactual=counterfactual,
        final_mean_shift_ns=shift,
        final_ppb_shift=ppb_shift,
        single_counterpart=single,
        cumulative_neutralization=cumulative,
        trajectory_mismatches=tuple(mismatches),
    )


def fmt(value: Optional[float], digits: int = 6) -> str:
    return "---" if value is None else f"{value:+.{digits}f}"


def print_lane(result: LaneResult, window: int, gate_ns: int) -> None:
    label = result.lane.upper()
    print()
    print("=" * 86)
    print(f"{label} PUBLISHED-NANOSECOND-RESIDUAL INCEPTION AUDIT")
    print("=" * 86)
    print()
    print("Population under audit:")
    print(f"  fragment.{result.lane}.pps_residual.fast_residual_ns")
    print(f"Admission gate:")
    print(f"  fragment.{result.lane}.pps_residual.valid == true")
    print()

    print(f"Rows examined:                {len(result.rows):,}")
    print(f"Authoritative residuals:      {len(result.accepted):,}")

    if result.first is None:
        print("First admitted residual:      none")
        print("Verdict: INDETERMINATE — no authoritative residual population")
        return

    first = result.first
    print(f"First admitted PPS:           {first.pps_count}")
    print(f"First admitted residual:      {first.residual_ns:+,d} ns")
    print(f"First row published Welford:  n={first.published_n} "
          f"mean={fmt(first.published_mean)} ns sd={fmt(first.published_stddev)} ns")

    print()
    print(f"First {min(window, len(result.rows))} persisted rows:")
    print(f"  {'PPS':>8s} {'VALID':>7s} {'RESIDUAL_NS':>14s} {'PUB_N':>8s} "
          f"{'PUB_MEAN_NS':>15s} {'CLOCK_NS':>13s} {'GNSS_NS':>13s}")
    print(f"  {'-'*8} {'-'*7} {'-'*14} {'-'*8} {'-'*15} {'-'*13} {'-'*13}")
    for row in result.rows[:window]:
        residual = "---" if row.residual_ns is None else f"{row.residual_ns:+,d}"
        valid = "---" if row.valid is None else str(row.valid)
        pub_n = "---" if row.published_n is None else f"{row.published_n:,d}"
        pub_mean = "---" if row.published_mean is None else f"{row.published_mean:+.6f}"
        clock_ns = "---" if row.clock_interval_ns is None else f"{row.clock_interval_ns:,d}"
        gnss_ns = "---" if row.gnss_interval_ns is None else f"{row.gnss_interval_ns:,d}"
        print(f"  {row.pps_count:8d} {valid:>7s} {residual:>14s} {pub_n:>8s} "
              f"{pub_mean:>15s} {clock_ns:>13s} {gnss_ns:>13s}")

    print()
    print("Independent Welford replay from admitted published residuals:")
    print(f"  With first residual:        n={result.replay.n:,} "
          f"mean={result.replay.mean:+.9f} ns  sd={result.replay.stddev:.9f} ns")
    print(f"  Without first residual:     n={result.counterfactual.n:,} "
          f"mean={result.counterfactual.mean:+.9f} ns  sd={result.counterfactual.stddev:.9f} ns")
    print(f"  Final mean contribution:    {fmt(result.final_mean_shift_ns, 9)} ns")
    print(f"  Equivalent frequency shift: {fmt(result.final_ppb_shift, 9)} ppb")

    # Algebraic cross-check: exact contribution of first sample relative to the
    # later population mean is (r1 - later_mean) / N.
    if result.counterfactual.n and first.residual_ns is not None:
        exact = (float(first.residual_ns) - result.counterfactual.mean) / float(result.replay.n)
        print(f"  Algebraic cross-check:      {exact:+.9f} ns")

    print()
    if result.single_counterpart:
        pps, residual, pair_sum = result.single_counterpart
        print(f"Single offsetting residual:   pps={pps} residual={residual:+d} ns "
              f"pair_sum={pair_sum:+d} ns")
    else:
        print(f"Single offsetting residual:   none within ±{gate_ns} ns")

    if result.cumulative_neutralization:
        pps, running_sum = result.cumulative_neutralization
        print(f"Cumulative return to zero:    pps={pps} running_sum={running_sum:+d} ns")
    else:
        print(f"Cumulative return to zero:    none within ±{gate_ns} ns")

    print()
    if result.trajectory_mismatches:
        print(f"Welford trajectory mismatch:  {len(result.trajectory_mismatches)} issue(s)")
        for mismatch in result.trajectory_mismatches[:12]:
            print(f"  - {mismatch}")
        if len(result.trajectory_mismatches) > 12:
            print(f"  ... and {len(result.trajectory_mismatches) - 12} more")
    else:
        print("Welford trajectory:           EXACTLY REPLAYED within tolerance")

    shift = abs(result.final_mean_shift_ns or 0.0)
    if result.trajectory_mismatches:
        verdict = "INDETERMINATE — published Welford does not replay from declared residuals"
    elif shift < 0.001:
        verdict = "CLEAN — first residual has negligible final mean influence"
    elif result.single_counterpart or result.cumulative_neutralization:
        verdict = "NEUTRALIZED — inception residual is offset in the admitted ns population"
    else:
        verdict = "UNCOMPENSATED — first admitted residual leaves a one-sided Welford mean shift"
    print(f"Verdict: {verdict}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit first published OCXO residual_ns contribution to firmware Welford."
    )
    parser.add_argument("campaign", help="campaign name in the timebase table")
    parser.add_argument(
        "--clock",
        choices=("OCXO1", "OCXO2", "BOTH"),
        default="BOTH",
        help="lane to audit (default: BOTH)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=20,
        help="number of inception rows to print (default: 20)",
    )
    parser.add_argument(
        "--counterpart-gate-ns",
        type=int,
        default=8,
        help="absolute ns gate for offsetting-pair/zero-sum detection (default: 8)",
    )
    parser.add_argument(
        "--mean-tolerance-ns",
        type=float,
        default=1.0e-6,
        help="published-vs-replayed Welford mean tolerance (default: 1e-6 ns)",
    )
    parser.add_argument(
        "--sd-tolerance-ns",
        type=float,
        default=1.0e-6,
        help="published-vs-replayed Welford stddev tolerance (default: 1e-6 ns)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.window < 1:
        raise SystemExit("--window must be at least 1")
    if args.counterpart_gate_ns < 0:
        raise SystemExit("--counterpart-gate-ns must be nonnegative")

    records = fetch_timebase(args.campaign)
    if not records:
        raise SystemExit(f"No TIMEBASE rows found for campaign '{args.campaign}'")

    lanes = LANES if args.clock == "BOTH" else (args.clock.lower(),)
    print(f"Campaign: {args.campaign}")
    print(f"Persisted TIMEBASE rows: {len(records):,}")
    print("Primary evidence: published, valid PPS residuals in nanoseconds")

    mismatch_total = 0
    for lane in lanes:
        result = audit_lane(
            records,
            lane,
            counterpart_gate_ns=args.counterpart_gate_ns,
            mean_tolerance_ns=args.mean_tolerance_ns,
            sd_tolerance_ns=args.sd_tolerance_ns,
        )
        mismatch_total += len(result.trajectory_mismatches)
        print_lane(result, args.window, args.counterpart_gate_ns)

    if mismatch_total:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
