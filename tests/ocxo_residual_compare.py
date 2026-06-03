"""
ZPNet OCXO Residual Compare — same-yardstick diagnostic report.

Reads TIMEBASE rows for a campaign and prints a one-row-per-second comparison
between the traditional PPS-founded nanosecond residual and the new
cycle-domain diagnostic residual published in TIMEBASE_FORENSICS.

Unlike raw_cycles.py, this report does not compare prediction vs actual inside
each lane.  It compares the GNSS/PPS next-second DWT-cycle prediction against
the OCXO next-second DWT-cycle prediction, then places that side-by-side with
the traditional GNSS-at-edge nanosecond interval math.

Usage:
    python -m zpnet.tests.ocxo_residual_compare <campaign_name> [limit] [clock]
    python tests/ocxo_residual_compare.py <campaign_name> [limit] [clock]
    python tests/ocxo_residual_compare.py <campaign_name> --clock OCXO1 [limit]

Clock filter:
    OCXO1, OCXO2, BOTH, ALL

Column doctrine:
    pps_pred       = GNSS/PPS next-second DWT-cycle prediction.
    ocxo*_pred     = OCXO next-second DWT-cycle prediction.
    ocxo*_pred_res = pps_pred - ocxo*_pred, sign-aligned so positive means
                     OCXO fast.

    gnss_ns        = GNSS interval nanoseconds from pps_residual.gnss_interval_ns.
    ocxo*_ns       = OCXO interval nanoseconds from pps_residual.clock_interval_ns.
    ocxo*_trad     = Traditional PPS-founded residual:
                       ocxo*_ns - gnss_ns
                     sign convention: positive means OCXO fast.

    ocxo*_diag     = Published diagnostic_fast_residual_cycles from
                     cycle_residual_diagnostic, based on actual DWT intervals.
    ocxo*_diff     = diagnostic_fast_residual_cycles - traditional_fast_residual_ns.
                     This is the normalization courtroom witness.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional

from zpnet.shared.db import open_db


VALID_CLOCKS = {"OCXO1", "OCXO2", "BOTH", "ALL"}


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
                NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_count', '')::bigint,
                NULLIF(payload->'payload'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_count', '')::bigint
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


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    f = root.get("forensics")
    return f if isinstance(f, dict) else {}


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _as_bool(v: Any) -> Optional[bool]:
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("true", "t", "yes", "y", "1"):
            return True
        if s in ("false", "f", "no", "n", "0"):
            return False
    return None


def _fmt_int(v: Optional[int], width: int, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}"


def _fmt_bool(v: Optional[bool], width: int = 5) -> str:
    if v is None:
        s = "---"
    else:
        s = "Y" if v else "N"
    return f"{s:>{width}s}"


def _prediction_cycles(frag: Dict[str, Any], lane: str) -> Optional[int]:
    pred = frag.get("prediction")
    if not isinstance(pred, dict):
        return None
    obj = pred.get(lane)
    if isinstance(obj, dict):
        return _as_int(obj.get("prediction_cycles") or obj.get("static_prediction_cycles"))
    return _as_int(pred.get(f"{lane}_prediction_cycles") or pred.get(f"{lane}_static_prediction_cycles"))


def _pps_count(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return (
        _as_int(frag.get("pps_count"))
        or _as_int(frag.get("teensy_pps_vclock_count"))
        or _as_int(forensic.get("pps_count"))
        or _as_int(root.get("pps_count"))
        or _as_int(root.get("teensy_pps_vclock_count"))
    )


def _cycle_diag(forensic: Dict[str, Any], lane: str) -> Dict[str, Any]:
    obj = _nested_get(forensic, lane, "cycle_residual_diagnostic")
    return obj if isinstance(obj, dict) else {}


def _pps_residual(frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Dict[str, Any]:
    # Prefer fragment because it is the canonical science row.  Fall back to
    # forensics for early paired-schema experiments.
    obj = _nested_get(frag, lane, "pps_residual")
    if isinstance(obj, dict):
        return obj
    obj = _nested_get(forensic, lane, "pps_residual")
    return obj if isinstance(obj, dict) else {}


def _lane_row_values(frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Dict[str, Optional[int] | Optional[bool]]:
    pps_pred = _prediction_cycles(frag, "pps")
    clock_pred = _prediction_cycles(frag, lane)

    pred_fast_res = None
    if pps_pred is not None and clock_pred is not None:
        pred_fast_res = pps_pred - clock_pred

    diag = _cycle_diag(forensic, lane)
    pps_res = _pps_residual(frag, forensic, lane)

    gnss_ns = _as_int(pps_res.get("gnss_interval_ns"))
    clock_ns = _as_int(pps_res.get("clock_interval_ns"))
    trad = _as_int(pps_res.get("fast_residual_ns"))
    if trad is None and gnss_ns is not None and clock_ns is not None:
        trad = clock_ns - gnss_ns

    diag_fast = _as_int(diag.get("diagnostic_fast_residual_cycles"))
    diag_diff = _as_int(diag.get("diagnostic_minus_traditional"))
    if diag_diff is None and diag_fast is not None and trad is not None:
        diag_diff = diag_fast - trad

    return {
        "pps_pred": pps_pred,
        "clock_pred": clock_pred,
        "pred_fast_res": _as_int(diag.get("prediction_fast_residual_cycles")) if diag else pred_fast_res,
        "gnss_ns": gnss_ns,
        "clock_ns": clock_ns,
        "trad": trad,
        "diag": diag_fast,
        "diff": diag_diff,
        "valid": _as_bool(diag.get("valid")),
        "trad_valid": _as_bool(diag.get("traditional_residual_valid")),
    }


def collect(rows: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics(rec)
        if not frag:
            continue

        pps_count = _pps_count(root, frag, forensic)
        row: Dict[str, Any] = {
            "pps_count": pps_count,
            "ocxo1": _lane_row_values(frag, forensic, "ocxo1"),
            "ocxo2": _lane_row_values(frag, forensic, "ocxo2"),
        }
        out.append(row)
    return out


def normalize_clock(clock: Optional[str]) -> str:
    if not clock:
        return "BOTH"
    c = clock.strip().upper()
    if c == "ALL":
        c = "BOTH"
    if c not in VALID_CLOCKS:
        raise SystemExit(f"Invalid clock filter '{clock}'. Use OCXO1, OCXO2, or BOTH.")
    return "BOTH" if c == "ALL" else c


def _header(clock: str) -> str:
    base = f"{'PPS':>8}"
    if clock in ("OCXO1", "BOTH"):
        base += (
            f"  {'G_PRED':>12} {'O1_PRED':>12} {'O1_PRES':>9}"
            f"  {'G_NS':>13} {'O1_NS':>13} {'O1_TRAD':>9}"
            f"  {'O1_DIAG':>9} {'O1_DIFF':>9} {'O1_V':>5}"
        )
    if clock in ("OCXO2", "BOTH"):
        base += (
            f"  {'G_PRED':>12} {'O2_PRED':>12} {'O2_PRES':>9}"
            f"  {'G_NS':>13} {'O2_NS':>13} {'O2_TRAD':>9}"
            f"  {'O2_DIAG':>9} {'O2_DIFF':>9} {'O2_V':>5}"
        )
    return base


def _line(row: Dict[str, Any], clock: str) -> str:
    s = f"{_fmt_int(row.get('pps_count'), 8)}"

    def add_lane(key: str) -> str:
        d = row[key]
        return (
            f"  {_fmt_int(d.get('pps_pred'), 12)}"
            f" {_fmt_int(d.get('clock_pred'), 12)}"
            f" {_fmt_int(d.get('pred_fast_res'), 9, signed=True)}"
            f"  {_fmt_int(d.get('gnss_ns'), 13)}"
            f" {_fmt_int(d.get('clock_ns'), 13)}"
            f" {_fmt_int(d.get('trad'), 9, signed=True)}"
            f"  {_fmt_int(d.get('diag'), 9, signed=True)}"
            f" {_fmt_int(d.get('diff'), 9, signed=True)}"
            f" {_fmt_bool(d.get('valid'), 5)}"
        )

    if clock in ("OCXO1", "BOTH"):
        s += add_lane("ocxo1")
    if clock in ("OCXO2", "BOTH"):
        s += add_lane("ocxo2")
    return s


def analyze(campaign: str, limit: int = 0, clock: str = "BOTH") -> None:
    clock = normalize_clock(clock)
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    collected = collect(rows)
    if limit:
        collected = collected[:limit]

    print(f"Campaign: {campaign}  ({len(rows):,} TIMEBASE rows, view={clock})")
    print()
    print("OCXO same-yardstick residual comparison")
    print("══════════════════════════════════════")
    print("  *_PRES = GNSS/PPS prediction cycles - OCXO prediction cycles.")
    print("  *_TRAD = traditional PPS-founded nanosecond residual.")
    print("  *_DIAG = diagnostic actual-cycle residual from TIMEBASE_FORENSICS.")
    print("  *_DIFF = *_DIAG - *_TRAD.")
    print("  *_V    = diagnostic object valid.")
    print()

    h = _header(clock)
    print(h)
    print("─" * len(h))
    for row in collected:
        print(_line(row, clock))

    print()
    print(f"Rows shown: {len(collected):,}")


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare traditional OCXO residuals against cycle-domain diagnostics."
    )
    parser.add_argument("campaign", help="Campaign name")
    parser.add_argument("rest", nargs="*", help="Optional [limit] [clock] in either order")
    parser.add_argument("--clock", choices=["OCXO1", "OCXO2", "BOTH", "ALL"], default=None)
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    limit = 0
    clock = args.clock

    for item in args.rest:
        try:
            limit = int(item)
            continue
        except ValueError:
            pass
        clock = item

    analyze(args.campaign, limit=limit, clock=normalize_clock(clock))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
