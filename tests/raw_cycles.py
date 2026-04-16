"""
ZPNet Raw Cycles — legacy vs authoritative timing surfaces

Reads both:
  1. legacy top-level OCXO fields from TIMEBASE_FRAGMENT
  2. new authoritative interrupt-diag OCXO interval-ledger fields

This version is aligned to the actual flat fragment schema, where keys such as
"ocxo1_diag_ocxo_second_cycles_observed" live directly in fragment.
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
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload")
    if isinstance(inner, dict) and ("pps_count" in inner or "fragment" in inner or "campaign" in inner):
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
        f"{'dwt_fin':>12s}  {'dwt_raw':>12s}  {'dwt_Δ':>8s}  {'dwt_lp':>12s}  {'dwt_res':>8s}  "
        f"{'o1_old':>12s}  {'o1_fin':>12s}  {'o1_raw':>12s}  {'o1_ns_old':>11s}  {'o1_ns_fin':>11s}  {'o1_ns_raw':>11s}  "
        f"{'o2_old':>12s}  {'o2_fin':>12s}  {'o2_raw':>12s}  {'o2_ns_old':>11s}  {'o2_ns_fin':>11s}  {'o2_ns_raw':>11s}"
    )
    print(
        f"{'─'*6}  "
        f"{'─'*12}  {'─'*12}  {'─'*8}  {'─'*12}  {'─'*8}  "
        f"{'─'*12}  {'─'*12}  {'─'*12}  {'─'*11}  {'─'*11}  {'─'*11}  "
        f"{'─'*12}  {'─'*12}  {'─'*12}  {'─'*11}  {'─'*11}  {'─'*11}"
    )

    w_dwt_fin = Welford()
    w_dwt_raw = Welford()
    w_dwt_delta = Welford()
    w_dwt_last_pred = Welford()
    w_dwt_res = Welford()
    w_dwt_interval_count = Welford()

    w_o1_old = Welford()
    w_o1_fin = Welford()
    w_o1_raw = Welford()
    w_o1_ns_old = Welford()
    w_o1_ns_fin = Welford()
    w_o1_ns_raw = Welford()

    w_o2_old = Welford()
    w_o2_fin = Welford()
    w_o2_raw = Welford()
    w_o2_ns_old = Welford()
    w_o2_ns_fin = Welford()
    w_o2_ns_raw = Welford()

    shown = 0
    gaps = 0
    prev_pps: Optional[int] = None

    for rec in rows:
        pps = as_int(root_field(rec, "pps_count"))
        if pps is None:
            continue

        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps} → {pps} ---")
        prev_pps = pps

        dwt_fin = as_int(frag_field(rec, "dwt_cycle_count_between_pps"))
        dwt_raw = as_int(frag_field(rec, "dwt_cycle_count_between_pps_raw"))
        dwt_delta = as_int(frag_field(rec, "dwt_cycle_count_between_pps_raw_minus_final"))
        dwt_last_pred = as_int(frag_field(rec, "dwt_cycle_count_last_second_prediction"))
        dwt_interval_count = as_int(frag_field(rec, "dwt_interval_count_last_second"))

        dwt_res = None
        if dwt_fin is not None and dwt_last_pred is not None:
            dwt_res = dwt_fin - dwt_last_pred

        if dwt_fin is not None:
            w_dwt_fin.update(float(dwt_fin))
        if dwt_raw is not None:
            w_dwt_raw.update(float(dwt_raw))
        if dwt_delta is not None:
            w_dwt_delta.update(float(dwt_delta))
        if dwt_last_pred is not None:
            w_dwt_last_pred.update(float(dwt_last_pred))
        if dwt_res is not None:
            w_dwt_res.update(float(dwt_res))
        if dwt_interval_count is not None:
            w_dwt_interval_count.update(float(dwt_interval_count))

        # Legacy top-level OCXO summary surfaces.
        o1_old = as_int(frag_field(rec, "ocxo1_dwt_cycles_between_edges"))
        o2_old = as_int(frag_field(rec, "ocxo2_dwt_cycles_between_edges"))
        o1_ns_old = as_int(frag_field(rec, "ocxo1_gnss_ns_between_edges"))
        o2_ns_old = as_int(frag_field(rec, "ocxo2_gnss_ns_between_edges"))

        # New authoritative diag surfaces (final).
        o1_fin = as_int(frag_field(rec, "ocxo1_diag_ocxo_second_cycles_observed"))
        o2_fin = as_int(frag_field(rec, "ocxo2_diag_ocxo_second_cycles_observed"))
        o1_ns_fin = as_int(frag_field(rec, "ocxo1_diag_ocxo_second_gnss_ns_observed"))
        o2_ns_fin = as_int(frag_field(rec, "ocxo2_diag_ocxo_second_gnss_ns_observed"))

        # New diag raw surfaces.
        o1_raw = as_int(frag_field(rec, "ocxo1_diag_ocxo_second_cycles_observed_raw"))
        o2_raw = as_int(frag_field(rec, "ocxo2_diag_ocxo_second_cycles_observed_raw"))
        o1_ns_raw = as_int(frag_field(rec, "ocxo1_diag_ocxo_second_gnss_ns_observed_raw"))
        o2_ns_raw = as_int(frag_field(rec, "ocxo2_diag_ocxo_second_gnss_ns_observed_raw"))

        for v, w in (
            (o1_old, w_o1_old), (o1_fin, w_o1_fin), (o1_raw, w_o1_raw),
            (o1_ns_old, w_o1_ns_old), (o1_ns_fin, w_o1_ns_fin), (o1_ns_raw, w_o1_ns_raw),
            (o2_old, w_o2_old), (o2_fin, w_o2_fin), (o2_raw, w_o2_raw),
            (o2_ns_old, w_o2_ns_old), (o2_ns_fin, w_o2_ns_fin), (o2_ns_raw, w_o2_ns_raw),
        ):
            if v is not None:
                w.update(float(v))

        print(
            f"{pps:>6d}  "
            f"{fmt_int(dwt_fin,12)}  {fmt_int(dwt_raw,12)}  {fmt_int(dwt_delta,8,True)}  {fmt_int(dwt_last_pred,12)}  {fmt_int(dwt_res,8,True)}  "
            f"{fmt_int(o1_old,12)}  {fmt_int(o1_fin,12)}  {fmt_int(o1_raw,12)}  {fmt_int(o1_ns_old,11)}  {fmt_int(o1_ns_fin,11)}  {fmt_int(o1_ns_raw,11)}  "
            f"{fmt_int(o2_old,12)}  {fmt_int(o2_fin,12)}  {fmt_int(o2_raw,12)}  {fmt_int(o2_ns_old,11)}  {fmt_int(o2_ns_fin,11)}  {fmt_int(o2_ns_raw,11)}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()

    print_series("DWT final cycles between PPS", w_dwt_fin, "cycles")
    print_series("DWT raw cycles between PPS", w_dwt_raw, "cycles")
    print_series("DWT raw-minus-final", w_dwt_delta, "cycles")
    print_series("DWT last-second prediction", w_dwt_last_pred, "cycles")
    print_series("DWT residual (final - last prediction)", w_dwt_res, "cycles")
    print_series("DWT interval count last second", w_dwt_interval_count, "intervals")

    print_series("OCXO1 legacy cycles between edges", w_o1_old, "cycles")
    print_series("OCXO1 final observed cycles", w_o1_fin, "cycles")
    print_series("OCXO1 raw observed cycles", w_o1_raw, "cycles")
    print_series("OCXO1 legacy GNSS ns between edges", w_o1_ns_old, "ns")
    print_series("OCXO1 final observed GNSS ns", w_o1_ns_fin, "ns")
    print_series("OCXO1 raw observed GNSS ns", w_o1_ns_raw, "ns")

    print_series("OCXO2 legacy cycles between edges", w_o2_old, "cycles")
    print_series("OCXO2 final observed cycles", w_o2_fin, "cycles")
    print_series("OCXO2 raw observed cycles", w_o2_raw, "cycles")
    print_series("OCXO2 legacy GNSS ns between edges", w_o2_ns_old, "ns")
    print_series("OCXO2 final observed GNSS ns", w_o2_ns_fin, "ns")
    print_series("OCXO2 raw observed GNSS ns", w_o2_ns_raw, "ns")

    print("Notes:")
    print("  dwt_fin = dwt_cycle_count_between_pps")
    print("  dwt_raw = dwt_cycle_count_between_pps_raw")
    print("  dwt_Δ   = dwt_cycle_count_between_pps_raw_minus_final")
    print("  o*_old  = legacy top-level alpha surfaces")
    print("  o*_fin  = authoritative interrupt-diag final surfaces")
    print("  o*_raw  = interrupt-diag raw surfaces")
    print("  This report is intended to compare old vs new timing surfaces directly.")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
