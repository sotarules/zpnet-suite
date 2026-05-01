"""
ZPNet Cycle Prediction Report

Analyze DWT cycle prediction fields published in TIMEBASE_FRAGMENT.

This report focuses on the promoted top-level prediction fields:

  dwt_static_prediction_cycle_count
      The static/random-walk prediction for the *upcoming* second.
      To evaluate it honestly on row N, compare actual[N] against
      static_prediction[N-1].

  dwt_dynamic_prediction_cycle_count
      The final gated-servo dynamic prediction for the just-finished
      second represented by this row.  Compare actual[N] against this
      field directly.

  dwt_dynamic_prediction_adjust_count
      Number of accepted servo updates that changed the prediction in
      the just-finished second.

  dwt_dynamic_prediction_invalid_count
      Number of gated/impossible cadence captures in the just-finished
      second.

  dwt_dynamic_prediction_valid_count
      Number of accepted servo captures in the just-finished second.

  dwt_dynamic_prediction_adjust_cycles
      Net signed adjustment cycles applied by the dynamic servo during
      the just-finished second.

The script prints a compact row table plus summary statistics and event rows.

Usage:
    python cycle_prediction.py <campaign> [limit]
    python -m zpnet.tests.cycle_prediction <campaign> [limit]

Examples:
    python cycle_prediction.py Overnight1
    python cycle_prediction.py Overnight1 200
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

from zpnet.shared.db import open_db

DEFAULT_EVENT_THRESHOLD_CYCLES = 100
DWT_EXPECTED_PER_PPS = 1_008_000_000


@dataclass
class Row:
    pps: int
    actual_cycles: Optional[int]
    static_next: Optional[int]
    static_for_this_second: Optional[int]
    static_residual: Optional[int]
    dynamic_final: Optional[int]
    dynamic_residual: Optional[int]
    dynamic_adjust_count: Optional[int]
    dynamic_invalid_count: Optional[int]
    dynamic_valid_count: Optional[int]
    dynamic_adjust_cycles: Optional[int]
    fw_static_residual: Optional[int]
    dwt_second_residual: Optional[int]
    old_prediction_residual: Optional[int]
    flags: str


class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        self.min_val = min(self.min_val, x)
        self.max_val = max(self.max_val, x)

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                NULLIF(payload->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'fragment'->>'pps_count', '')::bigint
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
        result.append(payload)
    return result


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = _as_int(v)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_str(v: str, width: int = 0) -> str:
    s = v[:width] if width else v
    return f"{s:<{width}s}" if width else s


def _print_stats(name: str, w: Welford, unit: str = "cycles") -> None:
    if w.n == 0:
        print(f"  {name:<28s} no samples")
        return
    print(
        f"  {name:<28s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+10.3f} {unit:<6s}  "
        f"sd={w.stddev:9.3f}  "
        f"min={w.min_val:+8.0f}  "
        f"max={w.max_val:+8.0f}"
    )


def classify_flags(
    *,
    actual_cycles: Optional[int],
    static_residual: Optional[int],
    dynamic_residual: Optional[int],
    dynamic_invalid_count: Optional[int],
    dynamic_valid_count: Optional[int],
    dynamic_adjust_count: Optional[int],
    dynamic_adjust_cycles: Optional[int],
    threshold: int,
) -> str:
    flags: List[str] = []

    if actual_cycles is None:
        flags.append("MISSING_ACTUAL")

    if static_residual is not None and abs(static_residual) > threshold:
        flags.append("STATIC_SPIKE")
    if dynamic_residual is not None and abs(dynamic_residual) > threshold:
        flags.append("DYN_SPIKE")

    if dynamic_invalid_count is not None and dynamic_invalid_count > 0:
        flags.append(f"GATED:{dynamic_invalid_count}")

    if dynamic_valid_count is not None and dynamic_valid_count == 0:
        flags.append("NO_VALID_SERVO")

    if dynamic_adjust_count is not None and dynamic_adjust_count > 0:
        flags.append(f"ADJ:{dynamic_adjust_count}")

    if dynamic_adjust_cycles is not None and abs(dynamic_adjust_cycles) > threshold:
        flags.append("BIG_NET_ADJ")

    if static_residual is not None and dynamic_residual is not None:
        if abs(dynamic_residual) < abs(static_residual):
            flags.append("DYN_BETTER")
        elif abs(dynamic_residual) > abs(static_residual):
            flags.append("DYN_WORSE")
        else:
            flags.append("TIE")

    return ",".join(flags)


def build_rows(raw_rows: Iterable[Dict[str, Any]], threshold: int) -> List[Row]:
    out: List[Row] = []
    prev_static_next: Optional[int] = None

    for rec in raw_rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _first_int(
            root.get("teensy_pps_vclock_count"),
            root.get("pps_count"),
            frag.get("teensy_pps_vclock_count"),
            frag.get("pps_count"),
        )
        if pps is None:
            continue

        actual_cycles = _first_int(
            frag.get("dwt_cycles_between_pps_vclock"),
            frag.get("vclock_dwt_cycles_between_edges"),
            frag.get("dwt_cycle_count_between_pps"),  # legacy/script alias
        )

        static_next = _first_int(frag.get("dwt_static_prediction_cycle_count"))
        dynamic_final = _first_int(frag.get("dwt_dynamic_prediction_cycle_count"))
        dynamic_adjust_count = _first_int(frag.get("dwt_dynamic_prediction_adjust_count"))
        dynamic_invalid_count = _first_int(frag.get("dwt_dynamic_prediction_invalid_count"))
        dynamic_valid_count = _first_int(frag.get("dwt_dynamic_prediction_valid_count"))
        dynamic_adjust_cycles = _first_int(frag.get("dwt_dynamic_prediction_adjust_cycles"))

        # Existing/legacy residual surfaces, useful while schemas are in motion.
        old_prediction_residual = _first_int(
            frag.get("vclock_dwt_repair_last_prediction_residual_cycles"),
            frag.get("dwt_prediction_residual_cycles"),
        )
        dwt_second_residual = _first_int(frag.get("dwt_second_residual_cycles"))

        static_for_this_second = prev_static_next
        static_residual = (
            actual_cycles - static_for_this_second
            if actual_cycles is not None and static_for_this_second is not None
            else None
        )
        dynamic_residual = (
            actual_cycles - dynamic_final
            if actual_cycles is not None and dynamic_final is not None
            else None
        )

        # If firmware still publishes the old one-step residual, compare it to
        # the shifted static residual. This is a cheap schema sanity check.
        fw_static_residual = old_prediction_residual

        flags = classify_flags(
            actual_cycles=actual_cycles,
            static_residual=static_residual,
            dynamic_residual=dynamic_residual,
            dynamic_invalid_count=dynamic_invalid_count,
            dynamic_valid_count=dynamic_valid_count,
            dynamic_adjust_count=dynamic_adjust_count,
            dynamic_adjust_cycles=dynamic_adjust_cycles,
            threshold=threshold,
        )

        out.append(Row(
            pps=int(pps),
            actual_cycles=actual_cycles,
            static_next=static_next,
            static_for_this_second=static_for_this_second,
            static_residual=static_residual,
            dynamic_final=dynamic_final,
            dynamic_residual=dynamic_residual,
            dynamic_adjust_count=dynamic_adjust_count,
            dynamic_invalid_count=dynamic_invalid_count,
            dynamic_valid_count=dynamic_valid_count,
            dynamic_adjust_cycles=dynamic_adjust_cycles,
            fw_static_residual=fw_static_residual,
            dwt_second_residual=dwt_second_residual,
            old_prediction_residual=old_prediction_residual,
            flags=flags,
        ))

        prev_static_next = static_next

    return out


def print_table(rows: List[Row], limit: int) -> None:
    header = (
        f"{'pps':>7s}  "
        f"{'actual':>12s}  "
        f"{'static_pred':>12s}  "
        f"{'static_res':>10s}  "
        f"{'dyn_final':>12s}  "
        f"{'dyn_res':>9s}  "
        f"{'valid':>5s}  "
        f"{'invalid':>7s}  "
        f"{'adj_ct':>6s}  "
        f"{'adj_cyc':>8s}  "
        f"{'static_next':>12s}  "
        f"{'flags':<34s}"
    )
    print(header)
    print(
        f"{'─'*7}  {'─'*12}  {'─'*12}  {'─'*10}  {'─'*12}  {'─'*9}  "
        f"{'─'*5}  {'─'*7}  {'─'*6}  {'─'*8}  {'─'*12}  {'─'*34}"
    )

    shown = 0
    for r in rows:
        print(
            f"{r.pps:>7d}  "
            f"{_fmt_int(r.actual_cycles, 12)}  "
            f"{_fmt_int(r.static_for_this_second, 12)}  "
            f"{_fmt_int(r.static_residual, 10, signed=True)}  "
            f"{_fmt_int(r.dynamic_final, 12)}  "
            f"{_fmt_int(r.dynamic_residual, 9, signed=True)}  "
            f"{_fmt_int(r.dynamic_valid_count, 5)}  "
            f"{_fmt_int(r.dynamic_invalid_count, 7)}  "
            f"{_fmt_int(r.dynamic_adjust_count, 6)}  "
            f"{_fmt_int(r.dynamic_adjust_cycles, 8, signed=True)}  "
            f"{_fmt_int(r.static_next, 12)}  "
            f"{_fmt_str(r.flags, 34)}"
        )
        shown += 1
        if limit and shown >= limit:
            break


def analyze(campaign: str, limit: int = 0, threshold: int = DEFAULT_EVENT_THRESHOLD_CYCLES) -> None:
    raw = fetch_timebase(campaign)
    if not raw:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    rows = build_rows(raw, threshold)

    print(f"Campaign: {campaign}")
    print(f"Rows loaded: {len(rows):,}")
    print(f"Spike/gate threshold: {threshold:,} cycles")
    print()
    print("Column semantics:")
    print("  actual       = dwt_cycles_between_pps_vclock for this TIMEBASE row")
    print("  static_pred  = prior row's dwt_static_prediction_cycle_count")
    print("                 (the static prediction that applied to this row)")
    print("  static_next  = this row's static prediction for the upcoming second")
    print("  dyn_final    = final dynamic servo prediction for the just-finished second")
    print("  dyn_res      = actual - dyn_final")
    print("  adj_cyc      = net signed dynamic servo adjustment during the second")
    print()

    print_table(rows, limit)
    print()

    # Summary stats.
    w_static = Welford()
    w_dynamic = Welford()
    w_adjust_cycles = Welford()

    count_static = 0
    count_dynamic = 0
    dyn_better = 0
    dyn_worse = 0
    dyn_tie = 0
    static_spikes = 0
    dynamic_spikes = 0
    rows_with_gates = 0
    total_invalid = 0
    total_valid = 0
    total_adjust_count = 0
    missing_new_fields = 0
    fw_mismatch = 0

    events: List[Row] = []

    for r in rows:
        if r.static_residual is not None:
            w_static.update(float(r.static_residual))
            count_static += 1
            if abs(r.static_residual) > threshold:
                static_spikes += 1

        if r.dynamic_residual is not None:
            w_dynamic.update(float(r.dynamic_residual))
            count_dynamic += 1
            if abs(r.dynamic_residual) > threshold:
                dynamic_spikes += 1

        if r.static_residual is not None and r.dynamic_residual is not None:
            a = abs(r.static_residual)
            b = abs(r.dynamic_residual)
            if b < a:
                dyn_better += 1
            elif b > a:
                dyn_worse += 1
            else:
                dyn_tie += 1

        if r.dynamic_adjust_cycles is not None:
            w_adjust_cycles.update(float(r.dynamic_adjust_cycles))
        else:
            missing_new_fields += 1

        if r.dynamic_invalid_count:
            rows_with_gates += 1
            total_invalid += r.dynamic_invalid_count
        if r.dynamic_valid_count:
            total_valid += r.dynamic_valid_count
        if r.dynamic_adjust_count:
            total_adjust_count += r.dynamic_adjust_count

        if r.fw_static_residual is not None and r.static_residual is not None:
            if r.fw_static_residual != r.static_residual:
                fw_mismatch += 1

        # Save interesting rows for an event listing.
        if (
            (r.static_residual is not None and abs(r.static_residual) > threshold)
            or (r.dynamic_residual is not None and abs(r.dynamic_residual) > threshold)
            or (r.dynamic_invalid_count is not None and r.dynamic_invalid_count > 0)
            or "MISSING" in r.flags
        ):
            events.append(r)

    print("Summary")
    print("═══════")
    _print_stats("static residual", w_static)
    _print_stats("dynamic residual", w_dynamic)
    _print_stats("dynamic net adjustment", w_adjust_cycles)
    print()
    print(f"  static residual samples      = {count_static:,}")
    print(f"  dynamic residual samples     = {count_dynamic:,}")
    print(f"  dynamic better / worse / tie = {dyn_better:,} / {dyn_worse:,} / {dyn_tie:,}")
    print(f"  static spikes > threshold    = {static_spikes:,}")
    print(f"  dynamic spikes > threshold   = {dynamic_spikes:,}")
    print(f"  rows with gated captures     = {rows_with_gates:,}")
    print(f"  total gated captures         = {total_invalid:,}")
    print(f"  total valid servo captures   = {total_valid:,}")
    print(f"  total servo adjustments      = {total_adjust_count:,}")
    print(f"  rows missing dynamic fields  = {missing_new_fields:,}")
    print(f"  old pred residual mismatches = {fw_mismatch:,}")
    print()

    if events:
        print("Events / suspicious rows")
        print("════════════════════════")
        print_table(events, limit=50)
        if len(events) > 50:
            print(f"... and {len(events) - 50:,} more event rows")
        print()
    else:
        print("Events / suspicious rows")
        print("════════════════════════")
        print("  None detected above threshold.")
        print()

    print("Notes")
    print("═════")
    print("  • dwt_static_prediction_cycle_count is a prediction for the NEXT row.")
    print("    This report shifts it by one row to evaluate the static predictor.")
    print("  • dwt_dynamic_prediction_cycle_count is treated as the finalized")
    print("    dynamic prediction for the just-finished interval in the same row.")
    print("  • A row can have a good dynamic residual even when invalid_count > 0:")
    print("    that means the servo saw an impossible capture and refused to learn")
    print("    from it, which is the intended gate behavior.")
    print()


def main(argv: List[str]) -> None:
    if len(argv) < 2:
        print("Usage: cycle_prediction.py <campaign> [limit] [threshold]")
        print("  limit=0 means all rows. threshold defaults to 100 cycles.")
        raise SystemExit(1)

    campaign = argv[1]
    limit = int(argv[2]) if len(argv) >= 3 else 0
    threshold = int(argv[3]) if len(argv) >= 4 else DEFAULT_EVENT_THRESHOLD_CYCLES
    analyze(campaign, limit=limit, threshold=threshold)


if __name__ == "__main__":
    main(sys.argv)
