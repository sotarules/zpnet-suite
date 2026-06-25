#!/usr/bin/env python3
"""
ZPNet Recovery Probe — TIMEBASE_V3 / TIMEBASE_FRAGMENT_V3

Read-only campaign forensic tool aimed specifically at recovery pathologies.

It is intentionally narrower than campaign_analyzer.py.  Instead of giving a
general campaign verdict, it tries to discover *facts around recovery
boundaries*:

  * What did the previous durable TIMEBASE row imply?
  * What did the first post-gap TIMEBASE row actually publish?
  * Did GNSS, DWT, OCXO1, and OCXO2 project cleanly?
  * Did OCXO public ledgers, PPS residuals, prediction cycles, Welfords,
    and phase offsets all describe the same surface?
  * Did any lane enter a plateau, alternating state, or contaminated Welford
    immediately after recovery?

Usage:
    python3 recovery_probe.py Memory8
    python3 recovery_probe.py Memory8 --before 8 --after 30
    python3 recovery_probe.py Memory8 --dump-json /tmp/memory8_recovery_probe.json

From inside zpnet-suite, you can also copy this to tests/recovery_probe.py
and run it however you normally run the other .zt tools.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_PPS = 1_008_000_000

OCXO_SECOND_ALARM_NS = 10_000
OCXO_LEDGER_MISMATCH_NS = 500
PHASE_STEP_ALARM_NS = 100_000
PPB_ABSOLUTE_ALARM = 10_000.0
WELFORD_STDDEV_ALARM_NS = 500.0
DWT_PROJECTION_ALARM_CYCLES = 100_000
OCXO_PROJECTION_ALARM_NS = 5_000


# -----------------------------------------------------------------------------
# Formatting
# -----------------------------------------------------------------------------

def fmt_int(v: Any, width: int = 0) -> str:
    if v is None:
        s = "---"
    else:
        try:
            s = f"{int(v):,}"
        except Exception:
            s = str(v)
    return f"{s:>{width}s}" if width else s


def fmt_signed(v: Any, width: int = 0) -> str:
    if v is None:
        s = "---"
    else:
        try:
            n = int(v)
            s = f"{n:+,d}"
        except Exception:
            s = str(v)
    return f"{s:>{width}s}" if width else s


def fmt_float(v: Any, digits: int = 3, width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        try:
            f = float(v)
            sign = "+" if signed else ""
            s = f"{f:{sign},.{digits}f}"
        except Exception:
            s = str(v)
    return f"{s:>{width}s}" if width else s


def hline(ch: str = "-", n: int = 96) -> None:
    print(ch * n)


# -----------------------------------------------------------------------------
# Schema helpers
# -----------------------------------------------------------------------------

def as_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def fragment(row: Dict[str, Any]) -> Dict[str, Any]:
    return as_dict(row.get("fragment"))


def forensics(row: Dict[str, Any]) -> Dict[str, Any]:
    return as_dict(row.get("forensics"))


def extra(row: Dict[str, Any]) -> Dict[str, Any]:
    return as_dict(row.get("extra_clocks"))


def path_get(obj: Dict[str, Any], path: str, default: Any = None) -> Any:
    cur: Any = obj
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur.get(part)
    return default if cur is None else cur


def to_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def to_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def to_bool(v: Any) -> Optional[bool]:
    if isinstance(v, bool):
        return v
    if v is None:
        return None
    s = str(v).strip().lower()
    if s in ("true", "1", "yes", "on"):
        return True
    if s in ("false", "0", "no", "off"):
        return False
    return None


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = to_int(value)
        if parsed is not None:
            return parsed
    return None


def first_float(*values: Any) -> Optional[float]:
    for value in values:
        parsed = to_float(value)
        if parsed is not None:
            return parsed
    return None


def first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def row_count(row: Dict[str, Any]) -> int:
    f = fragment(row)
    value = first_int(
        row.get("teensy_pps_vclock_count"),
        row.get("pps_count"),
        f.get("teensy_pps_vclock_count"),
        f.get("pps_count"),
        f.get("teensy_pps_count"),
    )
    if value is None:
        raise ValueError(f"TIMEBASE row missing PPS/VCLOCK count, db_id={row.get('_db_id')}")
    return value


def row_time(row: Dict[str, Any]) -> str:
    return str(row.get("gnss_time_utc") or row.get("system_time_utc") or row.get("_db_ts") or "?")


def gnss_ns(row: Dict[str, Any]) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, "gnss.ns"),
        f.get("gnss_ns"),
        row.get("teensy_gnss_ns"),
        row.get("gnss_ns"),
    )


def vclock_ns(row: Dict[str, Any]) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, "vclock.ns"),
        path_get(f, "gnss.ns"),
        f.get("vclock_ns"),
        row.get("vclock_ns"),
    )


def ocxo_ns(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, f"{lane}.ns"),
        path_get(f, f"gnss.{lane}_ns"),
        f.get(f"{lane}_ns"),
        f.get(f"{lane}_ns_at_pps_vclock"),
        row.get(f"teensy_{lane}_ns"),
        row.get(f"{lane}_ns"),
    )


def ocxo_gnss_alias_ns(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(path_get(f, f"gnss.{lane}_ns"), f.get(f"gnss_{lane}_ns"))


def ocxo_measured_gnss_ns(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, f"{lane}.measured_gnss_ns"),
        path_get(f, f"{lane}.measured.ns"),
    )


def ocxo_source_id(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(path_get(f, f"{lane}.ns_source_id"))


def ocxo_projection_source(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(path_get(f, f"{lane}.pps_projection_source"))


def ocxo_projected_valid(row: Dict[str, Any], lane: str) -> Optional[bool]:
    f = fragment(row)
    return to_bool(path_get(f, f"{lane}.pps_projected_valid"))


def dwt_cycles(row: Dict[str, Any]) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, "dwt.cycle_count_total"),
        path_get(f, "dwt.cycles"),
        path_get(f, "dwt.value"),
        f.get("dwt_cycle_count_total"),
        row.get("teensy_dwt_cycles"),
        row.get("dwt_cycle_count_total"),
        row.get("dwt_cycles"),
    )


def dwt_interval_cycles(row: Dict[str, Any]) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, "dwt.cycles_between_pps_vclock"),
        f.get("dwt_cycles_between_pps_vclock"),
        forensics(row).get("dwt_cycles_between_pps_vclock"),
        row.get("dwt_cycles_between_pps_vclock"),
    )


def pps_interval_cycles(row: Dict[str, Any]) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, "pps.dwt_cycles_between_edges"),
        f.get("pps_dwt_cycles_between_edges"),
        row.get("pps_dwt_cycles_between_edges"),
    )


def stat_float(row: Dict[str, Any], lane: str, field: str) -> Optional[float]:
    f = fragment(row)
    return first_float(
        path_get(f, f"stats.{lane}.{field}"),
        f.get(f"{lane}_{field}"),
        row.get(f"{lane}_{field}"),
    )


def welford_float(row: Dict[str, Any], lane: str, field: str) -> Optional[float]:
    f = fragment(row)
    return first_float(
        path_get(f, f"stats.{lane}.welford.{field}"),
        path_get(f, f"stats.{lane}.{field}"),
        f.get(f"{lane}_welford_{field}"),
        row.get(f"{lane}_welford_{field}"),
    )


def welford_int(row: Dict[str, Any], lane: str, field: str) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, f"stats.{lane}.welford.{field}"),
        path_get(f, f"stats.{lane}.{field}"),
        f.get(f"{lane}_welford_{field}"),
        row.get(f"{lane}_welford_{field}"),
    )


def dac_welford_float(row: Dict[str, Any], lane: str, field: str) -> Optional[float]:
    f = fragment(row)
    return first_float(
        path_get(f, f"stats.dac.{lane}.{field}"),
        f.get(f"{lane}_dac_welford_{field}"),
        row.get(f"{lane}_dac_welford_{field}"),
    )


def dac_welford_int(row: Dict[str, Any], lane: str, field: str) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, f"stats.dac.{lane}.{field}"),
        f.get(f"{lane}_dac_welford_{field}"),
        row.get(f"{lane}_dac_welford_{field}"),
    )


def pred_int(row: Dict[str, Any], lane: str, field: str) -> Optional[int]:
    f = fragment(row)
    return first_int(path_get(f, f"prediction.{lane}.{field}"))


def ocxo_clock_interval(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, f"{lane}.pps_residual.clock_interval_ns"),
        path_get(f, f"{lane}.measurement.clock_interval_ns"),
        path_get(f, f"{lane}.interval.clock_interval_ns"),
        f.get(f"{lane}_clock_interval_ns"),
    )


def ocxo_gnss_interval(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, f"{lane}.pps_residual.gnss_interval_ns"),
        path_get(f, f"{lane}.measurement.gnss_ns_between_edges"),
        path_get(f, f"{lane}.interval.gnss_ns_between_edges"),
        f.get(f"{lane}_gnss_ns_between_edges"),
    )


def ocxo_residual(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    return first_int(
        path_get(f, f"{lane}.pps_residual.fast_residual_ns"),
        path_get(f, f"{lane}.measurement.second_residual_ns"),
        f.get(f"{lane}_second_residual_ns"),
        row.get(f"{lane}_second_residual_ns"),
    )


def ocxo_residual_valid(row: Dict[str, Any], lane: str) -> Optional[bool]:
    f = fragment(row)
    return to_bool(path_get(f, f"{lane}.pps_residual.valid"))


def phase_offset(row: Dict[str, Any], lane: str) -> Optional[int]:
    f = fragment(row)
    explicit = first_int(
        path_get(f, f"{lane}.phase_offset_ns"),
        f.get(f"{lane}_phase_offset_ns"),
        row.get(f"{lane}_phase_offset_ns"),
    )
    if explicit is not None:
        return explicit
    v = vclock_ns(row)
    o = ocxo_ns(row, lane)
    return v - o if v is not None and o is not None else None


def total_ppb(row: Dict[str, Any], lane: str) -> Optional[float]:
    g = gnss_ns(row)
    if g is None or g <= 0:
        return None
    if lane == "vclock":
        v = vclock_ns(row)
        return ((float(v) / float(g)) - 1.0) * 1.0e9 if v is not None else None
    if lane == "dwt":
        c = dwt_cycles(row)
        if c is None:
            return None
        expected = (float(g) / NS_PER_SECOND) * DWT_EXPECTED_PER_PPS
        return ((float(c) / expected) - 1.0) * 1.0e9
    o = ocxo_ns(row, lane)
    return ((float(o) / float(g)) - 1.0) * 1.0e9 if o is not None else None


def recovery_project(prev: Dict[str, Any], curr: Dict[str, Any], lane: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    pg = gnss_ns(prev)
    cg = gnss_ns(curr)
    if pg is None or cg is None or pg <= 0:
        return None, None, None

    if lane == "gnss":
        actual = cg
        expected = row_count(curr) * NS_PER_SECOND
        return expected, actual, actual - expected

    if lane == "dwt":
        pv = dwt_cycles(prev)
        cv = dwt_cycles(curr)
    elif lane == "vclock":
        pv = vclock_ns(prev)
        cv = vclock_ns(curr)
    else:
        pv = ocxo_ns(prev, lane)
        cv = ocxo_ns(curr, lane)

    if pv is None or cv is None:
        return None, None, None
    projected = (cg * pv) // pg
    return projected, cv, cv - projected


def lane_value(row: Dict[str, Any], lane: str) -> Optional[int]:
    if lane == "gnss":
        return gnss_ns(row)
    if lane == "vclock":
        return vclock_ns(row)
    if lane == "dwt":
        return dwt_cycles(row)
    return ocxo_ns(row, lane)


# -----------------------------------------------------------------------------
# Fetch
# -----------------------------------------------------------------------------

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
        rows = cur.fetchall()

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        payload["_db_id"] = row["id"]
        payload["_db_ts"] = str(row["ts"])
        out.append(payload)
    return out


# -----------------------------------------------------------------------------
# Analysis model
# -----------------------------------------------------------------------------

def recovery_boundaries(rows: List[Dict[str, Any]]) -> List[Tuple[int, Dict[str, Any], Dict[str, Any], int]]:
    out: List[Tuple[int, Dict[str, Any], Dict[str, Any], int]] = []
    for i in range(1, len(rows)):
        prev = rows[i - 1]
        curr = rows[i]
        delta = row_count(curr) - row_count(prev)
        if delta > 1:
            out.append((i, prev, curr, delta))
    return out


def segment_ids(rows: List[Dict[str, Any]]) -> Dict[int, int]:
    seg = 0
    mapping: Dict[int, int] = {}
    last: Optional[int] = None
    for row in rows:
        p = row_count(row)
        if last is not None and p > last + 1:
            seg += 1
        mapping[p] = seg
        last = p
    return mapping


def adjacent_pairs(rows: List[Dict[str, Any]]) -> Iterable[Tuple[Dict[str, Any], Dict[str, Any]]]:
    for i in range(1, len(rows)):
        prev = rows[i - 1]
        curr = rows[i]
        if row_count(curr) == row_count(prev) + 1:
            yield prev, curr


def row_by_count(rows: List[Dict[str, Any]]) -> Dict[int, Dict[str, Any]]:
    return {row_count(r): r for r in rows}


# -----------------------------------------------------------------------------
# Printers
# -----------------------------------------------------------------------------

def print_overview(campaign: str, rows: List[Dict[str, Any]], bounds: List[Tuple[int, Dict[str, Any], Dict[str, Any], int]]) -> None:
    counts = [row_count(r) for r in rows]
    hline("=")
    print(f"ZPNET RECOVERY PROBE: {campaign}")
    hline("=")
    print(f"Rows:             {len(rows):,}")
    print(f"PPS/VCLOCK range: {min(counts):,} -> {max(counts):,}")
    print(f"Expected rows:    {max(counts) - min(counts) + 1:,}")
    print(f"Missing rows:     {max(counts) - min(counts) + 1 - len(rows):,}")
    print(f"First time:       {row_time(rows[0])}")
    print(f"Last time:        {row_time(rows[-1])}")
    print(f"Recovery gaps:    {len(bounds):,}")
    for _, prev, curr, delta in bounds:
        print(f"  gap: {row_count(prev):,} -> {row_count(curr):,}  delta={delta:,}  skipped={delta-1:,}")


def print_boundary_certificates(bounds: List[Tuple[int, Dict[str, Any], Dict[str, Any], int]]) -> List[Dict[str, Any]]:
    hline()
    print("RECOVERY BOUNDARY CERTIFICATES")
    hline()
    facts: List[Dict[str, Any]] = []

    if not bounds:
        print("No recovery gaps were found.")
        return facts

    for _, prev, curr, delta in bounds:
        prev_pps = row_count(prev)
        curr_pps = row_count(curr)
        print()
        print(f"Boundary {prev_pps:,} -> {curr_pps:,}  delta={delta:,}  skipped={delta-1:,}")
        print(f"  prev time: {row_time(prev)}")
        print(f"  curr time: {row_time(curr)}")
        print()
        print("  Projection from prior durable TIMEBASE row:")
        print(f"  {'LANE':<8s} {'PREV':>18s} {'PROJECTED':>18s} {'ACTUAL':>18s} {'ERROR':>16s} {'PUB_PPB(prev→curr)':>24s} {'WELFORD_N(prev→curr)':>22s} {'WELFORD_SD(prev→curr)':>26s}")
        print(f"  {'-'*8} {'-'*18} {'-'*18} {'-'*18} {'-'*16} {'-'*24} {'-'*22} {'-'*26}")

        for lane in ("gnss", "vclock", "dwt", "ocxo1", "ocxo2"):
            projected, actual, error = recovery_project(prev, curr, lane)
            prev_v = lane_value(prev, lane)
            prev_ppb = 0.0 if lane == "gnss" else total_ppb(prev, lane)
            curr_ppb = 0.0 if lane == "gnss" else total_ppb(curr, lane)
            wn0 = welford_int(prev, lane, "n") if lane != "gnss" else None
            wn1 = welford_int(curr, lane, "n") if lane != "gnss" else None
            sd0 = welford_float(prev, lane, "stddev") if lane != "gnss" else None
            sd1 = welford_float(curr, lane, "stddev") if lane != "gnss" else None
            ppb_s = f"{fmt_float(prev_ppb, 3, signed=True)} → {fmt_float(curr_ppb, 3, signed=True)}"
            wn_s = f"{fmt_int(wn0)} → {fmt_int(wn1)}"
            sd_s = f"{fmt_float(sd0, 3)} → {fmt_float(sd1, 3)}"
            print(f"  {lane.upper():<8s} {fmt_int(prev_v,18)} {fmt_int(projected,18)} {fmt_int(actual,18)} {fmt_signed(error,16)} {ppb_s:>24s} {wn_s:>22s} {sd_s:>26s}")

            facts.append({
                "boundary": curr_pps,
                "lane": lane,
                "prev": prev_v,
                "projected": projected,
                "actual": actual,
                "error": error,
                "prev_ppb": prev_ppb,
                "curr_ppb": curr_ppb,
                "prev_welford_n": wn0,
                "curr_welford_n": wn1,
                "prev_welford_sd": sd0,
                "curr_welford_sd": sd1,
            })

        print()
        print("  OCXO source surfaces at boundary:")
        print(f"  {'LANE':<7s} {'ROW':<5s} {'ns':>18s} {'gnss.alias':>18s} {'measured':>18s} {'phase':>16s} {'src':>5s} {'proj_src':>8s} {'proj_ok':>8s}")
        print(f"  {'-'*7} {'-'*5} {'-'*18} {'-'*18} {'-'*18} {'-'*16} {'-'*5} {'-'*8} {'-'*8}")
        for lane in ("ocxo1", "ocxo2"):
            for label, row in (("prev", prev), ("curr", curr)):
                print(f"  {lane.upper():<7s} {label:<5s} "
                      f"{fmt_int(ocxo_ns(row,lane),18)} "
                      f"{fmt_int(ocxo_gnss_alias_ns(row,lane),18)} "
                      f"{fmt_int(ocxo_measured_gnss_ns(row,lane),18)} "
                      f"{fmt_signed(phase_offset(row,lane),16)} "
                      f"{fmt_int(ocxo_source_id(row,lane),5)} "
                      f"{fmt_int(ocxo_projection_source(row,lane),8)} "
                      f"{str(ocxo_projected_valid(row,lane)):>8s}")

    return facts


def print_window(rows: List[Dict[str, Any]], bounds: List[Tuple[int, Dict[str, Any], Dict[str, Any], int]], before: int, after: int) -> None:
    hline()
    print(f"BOUNDARY WINDOWS  (--before {before}, --after {after})")
    hline()
    if not bounds:
        print("No recovery gaps were found.")
        return

    for idx, prev, curr, delta in bounds:
        start = max(0, idx - before)
        end = min(len(rows), idx + after + 1)
        print()
        print(f"Window around {row_count(prev):,} -> {row_count(curr):,}")
        print(f"  {'PPS':>7s} {'Δpps':>5s} {'DWT_PPB':>11s} "
              f"{'O1_OFF':>14s} {'O1_ΔNS':>14s} {'O1_INT':>14s} {'O1_RES':>13s} {'O1_PPB':>12s} {'O1_WSD':>12s} {'O1_ACTCYC':>13s} "
              f"{'O2_OFF':>14s} {'O2_ΔNS':>14s} {'O2_INT':>14s} {'O2_RES':>13s} {'O2_PPB':>12s} {'O2_WSD':>12s} {'O2_ACTCYC':>13s}")
        print("  " + "-" * 236)

        prev_row: Optional[Dict[str, Any]] = None
        for row in rows[start:end]:
            pps = row_count(row)
            dpps = None if prev_row is None else pps - row_count(prev_row)
            dwt_ppb = total_ppb(row, "dwt")

            values: List[str] = []
            for lane in ("ocxo1", "ocxo2"):
                off = phase_offset(row, lane)
                ns_delta = None
                if prev_row is not None and pps == row_count(prev_row) + 1:
                    pv = ocxo_ns(prev_row, lane)
                    cv = ocxo_ns(row, lane)
                    if pv is not None and cv is not None:
                        ns_delta = cv - pv
                values.extend([
                    fmt_signed(off, 14),
                    fmt_signed(ns_delta, 14),
                    fmt_int(ocxo_clock_interval(row, lane), 14),
                    fmt_signed(ocxo_residual(row, lane), 13),
                    fmt_float(total_ppb(row, lane), 3, 12, signed=True),
                    fmt_float(welford_float(row, lane, "stddev"), 3, 12),
                    fmt_int(pred_int(row, lane, "actual_cycles"), 13),
                ])

            gap_mark = ""
            if prev_row is not None and dpps is not None and dpps > 1:
                gap_mark = "<GAP>"
            print(f"  {pps:7d} {fmt_int(dpps,5)} {fmt_float(dwt_ppb,3,11,signed=True)} "
                  f"{values[0]} {values[1]} {values[2]} {values[3]} {values[4]} {values[5]} {values[6]} "
                  f"{values[7]} {values[8]} {values[9]} {values[10]} {values[11]} {values[12]} {values[13]} {gap_mark}")
            prev_row = row


def find_first_bad_after_boundaries(
    rows: List[Dict[str, Any]],
    bounds: List[Tuple[int, Dict[str, Any], Dict[str, Any], int]],
) -> List[Dict[str, Any]]:
    results: List[Dict[str, Any]] = []
    if not bounds:
        return results

    next_bound_indices = [b[0] for b in bounds] + [len(rows)]
    for b_i, (idx, prev, curr, delta) in enumerate(bounds):
        limit = next_bound_indices[b_i + 1] if b_i + 1 < len(next_bound_indices) else len(rows)
        start_pps = row_count(curr)
        for lane in ("ocxo1", "ocxo2"):
            first_res = None
            first_delta = None
            first_ppb = None
            first_wsd = None
            previous_row: Optional[Dict[str, Any]] = None
            for j in range(idx, limit):
                row = rows[j]
                pps = row_count(row)

                res = ocxo_residual(row, lane)
                if first_res is None and res is not None and abs(res) > OCXO_SECOND_ALARM_NS:
                    first_res = (pps, res)

                if previous_row is not None and pps == row_count(previous_row) + 1:
                    pv = ocxo_ns(previous_row, lane)
                    cv = ocxo_ns(row, lane)
                    ci = ocxo_clock_interval(row, lane)
                    if pv is not None and cv is not None and ci is not None:
                        err = (cv - pv) - ci
                        if first_delta is None and abs(err) > OCXO_LEDGER_MISMATCH_NS:
                            first_delta = (pps, err, cv - pv, ci)

                ppb = total_ppb(row, lane)
                if first_ppb is None and ppb is not None and abs(ppb) > PPB_ABSOLUTE_ALARM:
                    first_ppb = (pps, ppb)

                sd = welford_float(row, lane, "stddev")
                if first_wsd is None and sd is not None and sd > WELFORD_STDDEV_ALARM_NS:
                    first_wsd = (pps, sd)

                previous_row = row

            results.append({
                "boundary": start_pps,
                "lane": lane,
                "first_residual_alarm": first_res,
                "first_ledger_mismatch": first_delta,
                "first_ppb_alarm": first_ppb,
                "first_welford_sd_alarm": first_wsd,
            })
    return results


def print_onset(rows: List[Dict[str, Any]], bounds: List[Tuple[int, Dict[str, Any], Dict[str, Any], int]]) -> List[Dict[str, Any]]:
    hline()
    print("FIRST BAD FACTS AFTER EACH RECOVERY")
    hline()
    facts = find_first_bad_after_boundaries(rows, bounds)
    if not facts:
        print("No post-recovery bad facts found by the configured thresholds.")
        return facts

    for f in facts:
        print()
        print(f"Boundary starting at pps={f['boundary']:,}  lane={f['lane'].upper()}")
        print(f"  first residual alarm:       {f['first_residual_alarm']}")
        print(f"  first ledger mismatch:      {f['first_ledger_mismatch']}")
        print(f"  first absolute-PPB alarm:   {f['first_ppb_alarm']}")
        print(f"  first Welford-SD alarm:     {f['first_welford_sd_alarm']}")
    return facts


def print_ocxo_census(rows: List[Dict[str, Any]], segs: Dict[int, int], bad_limit: int) -> List[Dict[str, Any]]:
    hline()
    print("OCXO PATHOLOGY CENSUS")
    hline()
    bad_rows_json: List[Dict[str, Any]] = []

    for lane in ("ocxo1", "ocxo2"):
        print()
        print(f"[{lane.upper()}]")
        residual_alarms: List[Tuple[int, int]] = []
        ledger_mismatches: List[Tuple[int, int, int, int]] = []
        ppb_alarms: List[Tuple[int, float]] = []
        phase_jumps: List[Tuple[int, int]] = []
        phase_clusters = Counter()
        residual_clusters = Counter()
        bad_by_segment = Counter()

        prev_row: Optional[Dict[str, Any]] = None
        prev_phase: Optional[int] = None
        for row in rows:
            pps = row_count(row)
            seg = segs.get(pps, 0)

            off = phase_offset(row, lane)
            if off is not None:
                # 1 ms buckets: enough to expose large state switching without
                # drowning in cycle-level noise.
                phase_clusters[round(off / 1_000_000.0)] += 1

            res = ocxo_residual(row, lane)
            if res is not None:
                residual_clusters[round(res / 1_000_000.0)] += 1
                if abs(res) > OCXO_SECOND_ALARM_NS:
                    residual_alarms.append((pps, res))
                    bad_by_segment[seg] += 1

            ppb = total_ppb(row, lane)
            if ppb is not None and abs(ppb) > PPB_ABSOLUTE_ALARM:
                ppb_alarms.append((pps, ppb))

            if prev_row is not None and pps == row_count(prev_row) + 1:
                pv = ocxo_ns(prev_row, lane)
                cv = ocxo_ns(row, lane)
                ci = ocxo_clock_interval(row, lane)
                if pv is not None and cv is not None and ci is not None:
                    delta = cv - pv
                    err = delta - ci
                    if abs(err) > OCXO_LEDGER_MISMATCH_NS:
                        ledger_mismatches.append((pps, err, delta, ci))
                        bad_by_segment[seg] += 1

                if off is not None and prev_phase is not None:
                    step = off - prev_phase
                    if abs(step) > PHASE_STEP_ALARM_NS:
                        phase_jumps.append((pps, step))

            prev_phase = off
            prev_row = row

        print(f"  residual alarms:          {len(residual_alarms):,}")
        print(f"  ledger delta mismatches:  {len(ledger_mismatches):,}")
        print(f"  absolute PPB alarms:      {len(ppb_alarms):,}")
        print(f"  phase jumps:              {len(phase_jumps):,}")
        if bad_by_segment:
            print("  bad rows by segment:      " + ", ".join(f"seg{k}={v}" for k, v in sorted(bad_by_segment.items())))

        if residual_alarms:
            diffs = [residual_alarms[i][0] - residual_alarms[i - 1][0] for i in range(1, len(residual_alarms))]
            diff_counts = Counter(diffs)
            print("  residual alarm PPS-spacing top: " + ", ".join(f"{k}:{v}" for k, v in diff_counts.most_common(10)))
            print("  residual buckets, rounded ms:   " + ", ".join(f"{k:+d}ms:{v}" for k, v in residual_clusters.most_common(10)))

        if phase_clusters:
            print("  phase-offset buckets, rounded ms:")
            for bucket, count in phase_clusters.most_common(12):
                print(f"      {bucket:+8d} ms : {count:,}")

        print(f"  first {min(bad_limit, len(residual_alarms))} residual alarm rows:")
        for pps, res in residual_alarms[:bad_limit]:
            row = row_by_count(rows)[pps]
            record = {
                "lane": lane,
                "pps": pps,
                "segment": segs.get(pps, 0),
                "residual_ns": res,
                "clock_interval_ns": ocxo_clock_interval(row, lane),
                "gnss_interval_ns": ocxo_gnss_interval(row, lane),
                "phase_offset_ns": phase_offset(row, lane),
                "total_ppb": total_ppb(row, lane),
                "welford_n": welford_int(row, lane, "n"),
                "welford_stddev": welford_float(row, lane, "stddev"),
                "prediction_actual_cycles": pred_int(row, lane, "actual_cycles"),
                "prediction_residual_cycles": pred_int(row, lane, "residual_cycles"),
                "ns_source_id": ocxo_source_id(row, lane),
                "projection_source": ocxo_projection_source(row, lane),
            }
            bad_rows_json.append(record)
            print(f"    pps={pps:7d} seg={record['segment']} "
                  f"res={fmt_signed(record['residual_ns'])} "
                  f"clk={fmt_int(record['clock_interval_ns'])} "
                  f"gnss_int={fmt_int(record['gnss_interval_ns'])} "
                  f"phase={fmt_signed(record['phase_offset_ns'])} "
                  f"ppb={fmt_float(record['total_ppb'],3,signed=True)} "
                  f"wN={fmt_int(record['welford_n'])} "
                  f"wSD={fmt_float(record['welford_stddev'],3)} "
                  f"actcyc={fmt_int(record['prediction_actual_cycles'])} "
                  f"pred_res={fmt_signed(record['prediction_residual_cycles'])} "
                  f"src={fmt_int(record['ns_source_id'])}/{fmt_int(record['projection_source'])}")

        if ledger_mismatches:
            print(f"  first {min(bad_limit, len(ledger_mismatches))} ledger mismatch rows:")
            for pps, err, delta, ci in ledger_mismatches[:bad_limit]:
                print(f"    pps={pps:7d} ledger_delta={fmt_signed(delta)} clock_interval={fmt_int(ci)} error={fmt_signed(err)}")

    return bad_rows_json


def plateau_runs(rows: List[Dict[str, Any]], lane: str, field_getter) -> List[Tuple[int, int, int]]:
    runs: List[Tuple[int, int, int]] = []
    current_value = None
    start_pps = None
    last_pps = None
    length = 0

    def finish() -> None:
        if current_value is not None and start_pps is not None and length >= 3:
            runs.append((length, int(start_pps), int(current_value)))

    for row in rows:
        pps = row_count(row)
        value = field_getter(row, lane)
        contiguous = last_pps is not None and pps == last_pps + 1
        if value is None:
            finish()
            current_value = None
            start_pps = None
            length = 0
        elif value == current_value and contiguous:
            length += 1
        else:
            finish()
            current_value = value
            start_pps = pps
            length = 1
        last_pps = pps
    finish()
    runs.sort(reverse=True)
    return runs


def print_cycle_plateaus(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    hline()
    print("PREDICTION / INTERVAL CYCLE PLATEAUS")
    hline()
    facts: List[Dict[str, Any]] = []

    getters = [
        ("pps.actual_cycles", "pps", lambda r, lane: pred_int(r, "pps", "actual_cycles")),
        ("vclock.actual_cycles", "vclock", lambda r, lane: pred_int(r, "vclock", "actual_cycles")),
        ("ocxo1.actual_cycles", "ocxo1", lambda r, lane: pred_int(r, "ocxo1", "actual_cycles")),
        ("ocxo2.actual_cycles", "ocxo2", lambda r, lane: pred_int(r, "ocxo2", "actual_cycles")),
        ("pps.dwt_cycles_between_edges", "pps", lambda r, lane: pps_interval_cycles(r)),
        ("dwt.cycles_between_pps_vclock", "dwt", lambda r, lane: dwt_interval_cycles(r)),
    ]

    for label, lane, getter in getters:
        runs = plateau_runs(rows, lane, getter)
        print()
        print(f"[{label}]")
        if not runs:
            print("  no repeated-value plateaus of length >= 3")
            continue
        for length, start_pps, value in runs[:10]:
            end_pps = start_pps + length - 1
            print(f"  len={length:4d}  pps={start_pps:7d}..{end_pps:7d}  value={fmt_int(value)}")
            facts.append({"field": label, "length": length, "start_pps": start_pps, "end_pps": end_pps, "value": value})

    print()
    print("Cycle-difference sanity at adjacent rows:")
    for lane in ("ocxo1", "ocxo2"):
        stats = []
        for row in rows:
            a = pred_int(row, lane, "actual_cycles")
            p = pred_int(row, "pps", "actual_cycles")
            v = pred_int(row, "vclock", "actual_cycles")
            if a is not None and p is not None and v is not None:
                stats.append((row_count(row), a - p, a - v))
        if stats:
            ap = [x[1] for x in stats]
            av = [x[2] for x in stats]
            print(f"  {lane.upper()} actual - PPS actual:    n={len(ap):,} min={min(ap):+d} max={max(ap):+d} mean={sum(ap)/len(ap):+.3f}")
            print(f"  {lane.upper()} actual - VCLOCK actual: n={len(av):,} min={min(av):+d} max={max(av):+d} mean={sum(av)/len(av):+.3f}")

    return facts


def print_welford_boundary_report(bounds: List[Tuple[int, Dict[str, Any], Dict[str, Any], int]]) -> None:
    hline()
    print("WELFORD RESTORE / BOUNDARY REPORT")
    hline()
    if not bounds:
        print("No recovery gaps were found.")
        return

    for _, prev, curr, delta in bounds:
        print()
        print(f"Boundary {row_count(prev):,} -> {row_count(curr):,}")
        print(f"  {'LANE':<12s} {'N prev→curr':>18s} {'MEAN prev→curr':>28s} {'SD prev→curr':>26s} {'MIN prev→curr':>28s} {'MAX prev→curr':>28s}")
        print(f"  {'-'*12} {'-'*18} {'-'*28} {'-'*26} {'-'*28} {'-'*28}")
        for lane in ("dwt", "vclock", "ocxo1", "ocxo2"):
            n = f"{fmt_int(welford_int(prev,lane,'n'))} → {fmt_int(welford_int(curr,lane,'n'))}"
            mean = f"{fmt_float(welford_float(prev,lane,'mean'),3,signed=True)} → {fmt_float(welford_float(curr,lane,'mean'),3,signed=True)}"
            sd = f"{fmt_float(welford_float(prev,lane,'stddev'),3)} → {fmt_float(welford_float(curr,lane,'stddev'),3)}"
            mn = f"{fmt_float(welford_float(prev,lane,'min'),3,signed=True)} → {fmt_float(welford_float(curr,lane,'min'),3,signed=True)}"
            mx = f"{fmt_float(welford_float(prev,lane,'max'),3,signed=True)} → {fmt_float(welford_float(curr,lane,'max'),3,signed=True)}"
            print(f"  {lane.upper():<12s} {n:>18s} {mean:>28s} {sd:>26s} {mn:>28s} {mx:>28s}")

        for lane in ("ocxo1", "ocxo2"):
            n = f"{fmt_int(dac_welford_int(prev,lane,'n'))} → {fmt_int(dac_welford_int(curr,lane,'n'))}"
            mean = f"{fmt_float(dac_welford_float(prev,lane,'mean'),6)} → {fmt_float(dac_welford_float(curr,lane,'mean'),6)}"
            sd = f"{fmt_float(dac_welford_float(prev,lane,'stddev'),6)} → {fmt_float(dac_welford_float(curr,lane,'stddev'),6)}"
            print(f"  {lane.upper()+'_DAC':<12s} {n:>18s} {mean:>28s} {sd:>26s}")


def print_field_coverage(rows: List[Dict[str, Any]]) -> None:
    hline()
    print("SCHEMA / FIELD COVERAGE")
    hline()

    probes = [
        ("fragment", lambda r: bool(fragment(r))),
        ("forensics", lambda r: bool(forensics(r))),
        ("forensics.micro_raw_cycles", lambda r: bool(forensics(r).get("micro_raw_cycles"))),
        ("gnss.ns", lambda r: gnss_ns(r) is not None),
        ("vclock.ns", lambda r: vclock_ns(r) is not None),
        ("dwt.cycle_count_total", lambda r: dwt_cycles(r) is not None),
        ("dwt.cycles_between_pps_vclock", lambda r: dwt_interval_cycles(r) is not None),
    ]
    for lane in ("ocxo1", "ocxo2"):
        probes.extend([
            (f"{lane}.ns", lambda r, lane=lane: ocxo_ns(r, lane) is not None),
            (f"gnss.{lane}_ns alias", lambda r, lane=lane: ocxo_gnss_alias_ns(r, lane) is not None),
            (f"{lane}.measured_gnss_ns", lambda r, lane=lane: ocxo_measured_gnss_ns(r, lane) is not None),
            (f"{lane}.pps_residual.valid", lambda r, lane=lane: ocxo_residual_valid(r, lane) is not None),
            (f"{lane}.pps_residual.clock_interval_ns", lambda r, lane=lane: ocxo_clock_interval(r, lane) is not None),
            (f"{lane}.pps_residual.gnss_interval_ns", lambda r, lane=lane: ocxo_gnss_interval(r, lane) is not None),
            (f"{lane}.prediction.actual_cycles", lambda r, lane=lane: pred_int(r, lane, "actual_cycles") is not None),
            (f"{lane}.stats.welford.n", lambda r, lane=lane: welford_int(r, lane, "n") is not None),
        ])

    n = len(rows)
    for name, func in probes:
        count = sum(1 for r in rows if func(r))
        print(f"  {name:<45s} {count:>7,d}/{n:<7,d}  {count * 100.0 / n:6.2f}%")


def build_json_summary(
    campaign: str,
    rows: List[Dict[str, Any]],
    boundary_facts: List[Dict[str, Any]],
    onset_facts: List[Dict[str, Any]],
    bad_rows: List[Dict[str, Any]],
    plateau_facts: List[Dict[str, Any]],
) -> Dict[str, Any]:
    counts = [row_count(r) for r in rows]
    return {
        "campaign": campaign,
        "rows": len(rows),
        "pps_min": min(counts),
        "pps_max": max(counts),
        "missing_rows": max(counts) - min(counts) + 1 - len(rows),
        "boundary_facts": boundary_facts,
        "onset_facts": onset_facts,
        "bad_rows_sample": bad_rows,
        "plateaus": plateau_facts,
    }


def analyze(campaign: str, before: int, after: int, bad_limit: int, dump_json: Optional[str]) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign {campaign!r}")
        return

    bounds = recovery_boundaries(rows)
    segs = segment_ids(rows)

    print_overview(campaign, rows, bounds)
    boundary_facts = print_boundary_certificates(bounds)
    print_window(rows, bounds, before=before, after=after)
    onset_facts = print_onset(rows, bounds)
    bad_rows = print_ocxo_census(rows, segs, bad_limit=bad_limit)
    plateau_facts = print_cycle_plateaus(rows)
    print_welford_boundary_report(bounds)
    print_field_coverage(rows)

    hline("=")
    print("RECOVERY PROBE SUMMARY")
    hline("=")
    print("Facts this program is trying to isolate:")
    print("  1. whether recovery projection failed at the campaign-ledger surface,")
    print("  2. whether the OCXO public ns ledger and pps_residual interval disagree,")
    print("  3. whether prediction.actual_cycles entered a plateau after RECOVER,")
    print("  4. whether Welford restore carried a contaminated or wrong-population statistic,")
    print("  5. whether the issue is common-mode OCXO1+OCXO2 or isolated to one lane.")
    print()
    if dump_json:
        summary = build_json_summary(campaign, rows, boundary_facts, onset_facts, bad_rows, plateau_facts)
        with open(dump_json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
        print(f"Wrote JSON summary: {dump_json}")


def main() -> None:
    parser = argparse.ArgumentParser(description="ZPNet TIMEBASE recovery pathology probe")
    parser.add_argument("campaign", help="campaign name, e.g. Memory8")
    parser.add_argument("--before", type=int, default=6, help="rows before each recovery boundary to print")
    parser.add_argument("--after", type=int, default=24, help="rows after each recovery boundary to print")
    parser.add_argument("--bad-limit", type=int, default=20, help="number of bad rows to print per OCXO")
    parser.add_argument("--dump-json", default=None, help="optional path to write machine-readable summary")
    args = parser.parse_args()

    analyze(
        campaign=args.campaign,
        before=max(0, args.before),
        after=max(0, args.after),
        bad_limit=max(1, args.bad_limit),
        dump_json=args.dump_json,
    )


if __name__ == "__main__":
    main()
