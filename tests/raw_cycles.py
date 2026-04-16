"""
ZPNet Raw Cycles — current TIMEBASE analyzer

Focused on:
  1. Raw PPS DWT counts and prediction residuals
  2. Raw OCXO1 / OCXO2 cycle counts between edges
  3. Raw OCXO1 / OCXO2 GNSS ns between edges
  4. Simple residuals and lane/common-mode comparisons

Expected TIMEBASE/TIMEBASE_FRAGMENT fields:
  dwt_cycle_count_at_pps
  dwt_cycle_count_between_pps
  dwt_cycle_count_last_second_prediction
  dwt_cycle_count_next_second_prediction
  dwt_cycle_count_next_second_adjustment
  dwt_effective_cycles_per_second
  dwt_expected_per_pps

  ocxo1_dwt_cycles_between_edges
  ocxo2_dwt_cycles_between_edges
  ocxo1_gnss_ns_between_edges
  ocxo2_gnss_ns_between_edges

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    .zt raw_cycles Calibrate4
    .zt raw_cycles Calibrate4 200
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000


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
        if x < self.min_val:
            self.min_val = x
        if x > self.max_val:
            self.max_val = x

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
            ORDER BY (payload->>'pps_count')::bigint ASC
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


def normalize_payload_object(rec: Dict[str, Any]) -> Dict[str, Any]:
    """
    Support both shapes:
      1) rec == actual TIMEBASE payload object
      2) rec == wrapper containing {"payload": actual_payload}
    """
    if not isinstance(rec, dict):
        return {}

    inner = rec.get("payload")
    if isinstance(inner, dict) and (
        "pps_count" in inner or "fragment" in inner or "campaign" in inner
    ):
        return inner

    return rec


def root_field(rec: Dict[str, Any], key: str) -> Any:
    return normalize_payload_object(rec).get(key)


def frag_field(rec: Dict[str, Any], key: str) -> Any:
    frag = normalize_payload_object(rec).get("fragment")
    if isinstance(frag, dict):
        return frag.get(key)
    return None


def as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def print_series(name: str, w: Welford, unit: str = "") -> None:
    if w.n < 2:
        return
    suffix = f" {unit}" if unit else ""
    print(f"{name}:")
    print(f"  n       = {w.n:,}")
    print(f"  mean    = {w.mean:,.3f}{suffix}")
    print(f"  stddev  = {w.stddev:,.3f}{suffix}")
    print(f"  stderr  = {w.stderr:,.3f}{suffix}")
    print(f"  min/max = {w.min_val:,.0f} .. {w.max_val:,.0f}{suffix}")
    print()


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} rows)")
    print()

    print(
        f"{'pps':>6s}  "
        f"{'dwt_act':>12s}  {'dwt_last_pred':>14s}  {'dwt_res':>8s}  "
        f"{'o1_cyc':>12s}  {'o1_ns':>11s}  {'o1_res':>8s}  "
        f"{'o2_cyc':>12s}  {'o2_ns':>11s}  {'o2_res':>8s}  "
        f"{'cycΔ':>8s}  {'nsΔ':>8s}"
    )
    print(
        f"{'─'*6}  "
        f"{'─'*12}  {'─'*14}  {'─'*8}  "
        f"{'─'*12}  {'─'*11}  {'─'*8}  "
        f"{'─'*12}  {'─'*11}  {'─'*8}  "
        f"{'─'*8}  {'─'*8}"
    )

    w_dwt_act = Welford()
    w_dwt_last_pred = Welford()
    w_dwt_next_pred = Welford()
    w_dwt_res = Welford()
    w_dwt_adj = Welford()
    w_dwt_effective = Welford()

    w_o1_cyc = Welford()
    w_o2_cyc = Welford()
    w_o1_ns = Welford()
    w_o2_ns = Welford()
    w_o1_res = Welford()
    w_o2_res = Welford()
    w_lane_cycle_delta = Welford()
    w_lane_ns_delta = Welford()
    w_joint_cycle_move_mismatch = Welford()
    w_joint_ns_move_mismatch = Welford()

    prev_row_pps: Optional[int] = None
    prev_o1_cyc: Optional[int] = None
    prev_o2_cyc: Optional[int] = None
    prev_o1_ns: Optional[int] = None
    prev_o2_ns: Optional[int] = None

    shown = 0
    gaps = 0

    for rec in rows:
        row_pps = as_int(frag_field(rec, "teensy_pps_count"))
        if row_pps is None:
            row_pps = as_int(root_field(rec, "pps_count"))
        if row_pps is None:
            continue

        if prev_row_pps is not None and row_pps != prev_row_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_row_pps} → {row_pps} ---")
        prev_row_pps = row_pps

        # DWT raw second truth surface.
        dwt_at_pps = as_int(frag_field(rec, "dwt_cycle_count_at_pps"))
        if dwt_at_pps is None:
            dwt_at_pps = as_int(root_field(rec, "dwt_cycle_count_at_pps"))

        dwt_actual = as_int(frag_field(rec, "dwt_cycle_count_between_pps"))
        if dwt_actual is None:
            dwt_actual = as_int(root_field(rec, "dwt_cycle_count_between_pps"))

        dwt_last_pred = as_int(frag_field(rec, "dwt_cycle_count_last_second_prediction"))
        if dwt_last_pred is None:
            dwt_last_pred = as_int(root_field(rec, "dwt_cycle_count_last_second_prediction"))

        dwt_next_pred = as_int(frag_field(rec, "dwt_cycle_count_next_second_prediction"))
        if dwt_next_pred is None:
            dwt_next_pred = as_int(root_field(rec, "dwt_cycle_count_next_second_prediction"))

        dwt_adj = as_int(frag_field(rec, "dwt_cycle_count_next_second_adjustment"))
        if dwt_adj is None:
            dwt_adj = as_int(root_field(rec, "dwt_cycle_count_next_second_adjustment"))

        dwt_effective = as_int(frag_field(rec, "dwt_effective_cycles_per_second"))
        if dwt_effective is None:
            dwt_effective = as_int(root_field(rec, "dwt_effective_cycles_per_second"))

        dwt_expected = as_int(frag_field(rec, "dwt_expected_per_pps"))
        if dwt_expected is None:
            dwt_expected = as_int(root_field(rec, "dwt_expected_per_pps"))

        dwt_res = None
        if dwt_actual is not None and dwt_last_pred is not None:
            dwt_res = dwt_actual - dwt_last_pred

        if dwt_actual is not None:
            w_dwt_act.update(float(dwt_actual))
        if dwt_last_pred is not None:
            w_dwt_last_pred.update(float(dwt_last_pred))
        if dwt_next_pred is not None:
            w_dwt_next_pred.update(float(dwt_next_pred))
        if dwt_res is not None:
            w_dwt_res.update(float(dwt_res))
        if dwt_adj is not None:
            w_dwt_adj.update(float(dwt_adj))
        if dwt_effective is not None:
            w_dwt_effective.update(float(dwt_effective))

        # OCXO raw second truth surface.
        o1_cyc = as_int(frag_field(rec, "ocxo1_dwt_cycles_between_edges"))
        o2_cyc = as_int(frag_field(rec, "ocxo2_dwt_cycles_between_edges"))
        o1_ns = as_int(frag_field(rec, "ocxo1_gnss_ns_between_edges"))
        o2_ns = as_int(frag_field(rec, "ocxo2_gnss_ns_between_edges"))

        o1_res = None if o1_ns is None else (NS_PER_SECOND - o1_ns)
        o2_res = None if o2_ns is None else (NS_PER_SECOND - o2_ns)

        cyc_delta = None
        if o1_cyc is not None and o2_cyc is not None:
            cyc_delta = o2_cyc - o1_cyc
            w_lane_cycle_delta.update(float(cyc_delta))

        ns_delta = None
        if o1_ns is not None and o2_ns is not None:
            ns_delta = o2_ns - o1_ns
            w_lane_ns_delta.update(float(ns_delta))

        if o1_cyc is not None:
            w_o1_cyc.update(float(o1_cyc))
        if o2_cyc is not None:
            w_o2_cyc.update(float(o2_cyc))
        if o1_ns is not None:
            w_o1_ns.update(float(o1_ns))
        if o2_ns is not None:
            w_o2_ns.update(float(o2_ns))
        if o1_res is not None:
            w_o1_res.update(float(o1_res))
        if o2_res is not None:
            w_o2_res.update(float(o2_res))

        if (
            prev_o1_cyc is not None and prev_o2_cyc is not None and
            o1_cyc is not None and o2_cyc is not None
        ):
            move1 = o1_cyc - prev_o1_cyc
            move2 = o2_cyc - prev_o2_cyc
            w_joint_cycle_move_mismatch.update(float(move1 - move2))

        if (
            prev_o1_ns is not None and prev_o2_ns is not None and
            o1_ns is not None and o2_ns is not None
        ):
            move1 = o1_ns - prev_o1_ns
            move2 = o2_ns - prev_o2_ns
            w_joint_ns_move_mismatch.update(float(move1 - move2))

        prev_o1_cyc = o1_cyc
        prev_o2_cyc = o2_cyc
        prev_o1_ns = o1_ns
        prev_o2_ns = o2_ns

        print(
            f"{row_pps:>6d}  "
            f"{fmt_int(dwt_actual,12)}  {fmt_int(dwt_last_pred,14)}  {fmt_int(dwt_res,8,True)}  "
            f"{fmt_int(o1_cyc,12)}  {fmt_int(o1_ns,11)}  {fmt_int(o1_res,8,True)}  "
            f"{fmt_int(o2_cyc,12)}  {fmt_int(o2_ns,11)}  {fmt_int(o2_res,8,True)}  "
            f"{fmt_int(cyc_delta,8,True)}  {fmt_int(ns_delta,8,True)}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()

    print_series("DWT cycle count between PPS", w_dwt_act, "cycles")
    print_series("DWT last-second prediction (untrammeled)", w_dwt_last_pred, "cycles")
    print_series("DWT next-second prediction (forward-looking)", w_dwt_next_pred, "cycles")
    print_series("DWT residual (actual - last-second prediction)", w_dwt_res, "cycles")
    print_series("DWT next-second adjustment", w_dwt_adj, "cycles")
    print_series("DWT effective cycles per second", w_dwt_effective, "cycles")

    print_series("OCXO1 raw cycles between edges", w_o1_cyc, "cycles")
    print_series("OCXO2 raw cycles between edges", w_o2_cyc, "cycles")
    print_series("OCXO1 raw GNSS ns between edges", w_o1_ns, "ns")
    print_series("OCXO2 raw GNSS ns between edges", w_o2_ns, "ns")
    print_series("OCXO1 simple residual (1e9 - ns)", w_o1_res, "ns")
    print_series("OCXO2 simple residual (1e9 - ns)", w_o2_res, "ns")
    print_series("Lane delta (OCXO2 cycles - OCXO1 cycles)", w_lane_cycle_delta, "cycles")
    print_series("Lane delta (OCXO2 ns - OCXO1 ns)", w_lane_ns_delta, "ns")
    print_series("Joint-motion mismatch (ΔOCXO1 vs ΔOCXO2 cycles)", w_joint_cycle_move_mismatch, "cycles")
    print_series("Joint-motion mismatch (ΔOCXO1 vs ΔOCXO2 ns)", w_joint_ns_move_mismatch, "ns")

    print("Notes:")
    print("  dwt_act       = dwt_cycle_count_between_pps")
    print("  dwt_last_pred = dwt_cycle_count_last_second_prediction")
    print("  dwt_next_pred = dwt_cycle_count_next_second_prediction")
    print("  dwt_res       = dwt_cycle_count_between_pps - dwt_cycle_count_last_second_prediction")
    print("  o1_res   = 1,000,000,000 - ocxo1_gnss_ns_between_edges")
    print("  o2_res   = 1,000,000,000 - ocxo2_gnss_ns_between_edges")
    print("  cycΔ/nsΔ = OCXO2 - OCXO1 for the same second")
    print("  Joint-motion mismatch near zero suggests common-mode movement.")
    print("  Large lane deltas with low joint-motion mismatch suggest shared excursions plus fixed bias.")
    if dwt_expected is not None:
        print(f"  dwt_expected_per_pps = {dwt_expected:,}")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()