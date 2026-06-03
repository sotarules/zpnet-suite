
"""
ZPNet OCXO Raw-vs-PPS Cycle Report.

This report compares clean PPS actual one-second DWT cycles against raw OCXO
observed one-second DWT cycles from:

    TIMEBASE_FORENSICS.<lane>.forensics.dwt_interval_gate.observed_cycles

It intentionally avoids VCLOCK and avoids the OCXO published/static actual cycle
surface except as a context column.

Usage:
    python -m zpnet.tests.ocxo_raw_pps_cycles <campaign_name> [limit] [clock]
    python tests/ocxo_raw_pps_cycles.py <campaign_name> [limit] [clock]
    python tests/ocxo_raw_pps_cycles.py <campaign_name> --clock OCXO1 [limit]

Clock filter:
    OCXO1, OCXO2, BOTH, ALL
"""

from __future__ import annotations

import argparse
import json
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

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            out.append(payload)
    return out


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


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        out = _as_int(value)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}"


def _fmt_str(v: Optional[str], width: int) -> str:
    s = "---" if v is None else str(v)
    if len(s) > width:
        s = s[:width]
    return f"{s:>{width}s}"


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _interval_delta(now: Optional[int], prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    return now - prev


def _prediction_actual(frag: Dict[str, Any], lane: str) -> Optional[int]:
    pred = frag.get("prediction")
    if not isinstance(pred, dict):
        return None
    obj = pred.get(lane)
    if isinstance(obj, dict):
        return _as_int(obj.get("actual_cycles"))
    return _as_int(pred.get(f"{lane}_actual_cycles"))


def _physical_pps_dwt(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "pps", "dwt_at_edge"),
        frag.get("pps_dwt_at_edge"),
        forensic.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
    )


def _pps_count(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        forensic.get("pps_count"),
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
    )


def _lane_forensics(forensic: Dict[str, Any], frag: Dict[str, Any], root: Dict[str, Any], lane: str) -> Dict[str, Any]:
    for obj in (
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
    ):
        if isinstance(obj, dict):
            return obj
    return {}


def _gate(forensic: Dict[str, Any], frag: Dict[str, Any], root: Dict[str, Any], lane: str) -> Dict[str, Any]:
    f = _lane_forensics(forensic, frag, root, lane)
    g = f.get("dwt_interval_gate")
    return g if isinstance(g, dict) else {}


def _service(forensic: Dict[str, Any], frag: Dict[str, Any], root: Dict[str, Any], lane: str) -> Dict[str, Any]:
    for obj in (
        _nested_get(forensic, lane, "service"),
        _nested_get(frag, lane, "service"),
        _nested_get(root, lane, "service"),
    ):
        if isinstance(obj, dict):
            return obj
    return {}


def _adjacency(forensic: Dict[str, Any], frag: Dict[str, Any], root: Dict[str, Any], lane: str) -> Dict[str, Any]:
    f = _lane_forensics(forensic, frag, root, lane)
    adj = f.get("dwt_interval_adjacency")
    return adj if isinstance(adj, dict) else {}


def _adj_tag(adj: Dict[str, Any]) -> str:
    if not _as_bool(adj.get("valid")):
        return "---"
    if _as_bool(adj.get("rejected")):
        return "REJ"
    if _as_bool(adj.get("ok")):
        return "OK"
    return "BAD"


def _lane_values(root: Dict[str, Any],
                 frag: Dict[str, Any],
                 forensic: Dict[str, Any],
                 lane: str,
                 pps_actual: Optional[int],
                 prev_raw: Optional[int],
                 prev_res: Optional[int]) -> tuple[Dict[str, Any], Optional[int], Optional[int]]:
    gate = _gate(forensic, frag, root, lane)
    service = _service(forensic, frag, root, lane)
    adj = _adjacency(forensic, frag, root, lane)

    raw = _as_int(gate.get("observed_cycles"))
    gate_pred = _as_int(gate.get("prediction_cycles"))
    gate_res = _as_int(gate.get("residual_cycles"))
    if gate_res is None and raw is not None and gate_pred is not None:
        gate_res = raw - gate_pred

    effective = _as_int(gate.get("effective_cycles"))
    published = _prediction_actual(frag, lane)

    raw_res = None
    if pps_actual is not None and raw is not None:
        # Positive means OCXO fast: a fast OCXO has a shorter OCXO-second DWT interval.
        raw_res = pps_actual - raw

    out = {
        "raw": raw,
        "raw_delta": _interval_delta(raw, prev_raw),
        "raw_res": raw_res,
        "res_delta": _interval_delta(raw_res, prev_res),
        "gate_res": gate_res,
        "effective": effective,
        "published": published,
        "raw_minus_pub": (raw - published) if raw is not None and published is not None else None,
        "adj": _adj_tag(adj),
        "svc": _as_int(service.get("offset_ticks")),
    }
    return out, raw, raw_res


def collect(rows: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    prev_pps_count: Optional[int] = None
    prev_physical_pps_dwt: Optional[int] = None
    prev_pps_actual: Optional[int] = None
    prev_raw: Dict[str, Optional[int]] = {"ocxo1": None, "ocxo2": None}
    prev_res: Dict[str, Optional[int]] = {"ocxo1": None, "ocxo2": None}

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics(rec)
        if not frag:
            continue

        pps_count = _pps_count(root, frag, forensic)
        if pps_count is None:
            continue

        if prev_pps_count is not None and pps_count != prev_pps_count + 1:
            prev_physical_pps_dwt = None
            prev_pps_actual = None
            prev_raw = {"ocxo1": None, "ocxo2": None}
            prev_res = {"ocxo1": None, "ocxo2": None}

        pps_actual = _prediction_actual(frag, "pps")
        physical = _physical_pps_dwt(root, frag, forensic)
        if pps_actual is None and physical is not None and prev_physical_pps_dwt is not None:
            pps_actual = _delta_u32(physical, prev_physical_pps_dwt)

        o1, next_o1_raw, next_o1_res = _lane_values(
            root, frag, forensic, "ocxo1", pps_actual, prev_raw["ocxo1"], prev_res["ocxo1"]
        )
        o2, next_o2_raw, next_o2_res = _lane_values(
            root, frag, forensic, "ocxo2", pps_actual, prev_raw["ocxo2"], prev_res["ocxo2"]
        )

        out.append({
            "pps": pps_count,
            "pps_actual": pps_actual,
            "pps_delta": _interval_delta(pps_actual, prev_pps_actual),
            "ocxo1": o1,
            "ocxo2": o2,
        })

        prev_pps_count = pps_count
        if physical is not None:
            prev_physical_pps_dwt = physical
        if pps_actual is not None:
            prev_pps_actual = pps_actual
        prev_raw["ocxo1"] = next_o1_raw
        prev_raw["ocxo2"] = next_o2_raw
        prev_res["ocxo1"] = next_o1_res
        prev_res["ocxo2"] = next_o2_res

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
    h = f"{'PPS':>8} {'PPS_ACT':>13} {'PPS_Δ':>8}"
    if clock in ("OCXO1", "BOTH"):
        h += f"  {'O1_RAW':>13} {'O1_RES':>8} {'O1_RΔ':>8} {'O1_GATE':>8} {'O1_EFF':>13} {'O1_PUB':>13} {'O1_R-P':>8} {'O1_ADJ':>5} {'O1_SVC':>7}"
    if clock in ("OCXO2", "BOTH"):
        h += f"  {'O2_RAW':>13} {'O2_RES':>8} {'O2_RΔ':>8} {'O2_GATE':>8} {'O2_EFF':>13} {'O2_PUB':>13} {'O2_R-P':>8} {'O2_ADJ':>5} {'O2_SVC':>7}"
    return h


def _lane_cols(d: Dict[str, Any]) -> str:
    return (
        f"  {_fmt_int(d['raw'], 13)}"
        f" {_fmt_int(d['raw_res'], 8, signed=True)}"
        f" {_fmt_int(d['res_delta'], 8, signed=True)}"
        f" {_fmt_int(d['gate_res'], 8, signed=True)}"
        f" {_fmt_int(d['effective'], 13)}"
        f" {_fmt_int(d['published'], 13)}"
        f" {_fmt_int(d['raw_minus_pub'], 8, signed=True)}"
        f" {_fmt_str(d['adj'], 5)}"
        f" {_fmt_int(d['svc'], 7, signed=True)}"
    )


def _line(row: Dict[str, Any], clock: str) -> str:
    s = f"{_fmt_int(row['pps'], 8)} {_fmt_int(row['pps_actual'], 13)} {_fmt_int(row['pps_delta'], 8, signed=True)}"
    if clock in ("OCXO1", "BOTH"):
        s += _lane_cols(row["ocxo1"])
    if clock in ("OCXO2", "BOTH"):
        s += _lane_cols(row["ocxo2"])
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
    print("OCXO raw observed cycles vs PPS actual cycles")
    print("════════════════════════════════════════════")
    print("  PPS_ACT = firmware PPS actual one-second DWT cycles.")
    print("  O*_RAW  = raw OCXO observed_cycles from dwt_interval_gate.")
    print("  O*_RES  = PPS_ACT - O*_RAW; positive means OCXO fast.")
    print("  O*_RΔ   = O*_RES[n] - O*_RES[n-1].")
    print("  O*_GATE = raw - gate prediction; ISR/gate disagreement.")
    print("  O*_EFF  = gate effective cycles, context only.")
    print("  O*_PUB  = published/static OCXO actual cycles, context only.")
    print("  O*_R-P  = raw observed cycles - published actual cycles.")
    print()

    h = _header(clock)
    print(h)
    print("─" * len(h))
    for row in collected:
        print(_line(row, clock))

    print()
    print(f"Rows shown: {len(collected):,}")


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare raw OCXO observed cycles against PPS actual cycles.")
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
