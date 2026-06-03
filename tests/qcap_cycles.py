"""
ZPNet QCAP — QTimer capture packet dump report.

Reads TIMEBASE rows for a campaign and prints the compact qcap object from
TIMEBASE_FORENSICS once per second.

This is a narrow companion to raw_cycles.py.  Instead of showing prediction,
gate, adjacency, and published cycle surfaces, it focuses only on the compact
QTimer capture packet now published at:

    payload.forensics.<lane>.forensics.qcap

Default view:
    VCLOCK + OCXO1 + OCXO2

Focused view:
    one selected clock

Usage:
    python -m zpnet.tests.qcap_cycles <campaign_name> [limit] [clock]
    python tests/qcap_cycles.py <campaign_name> [limit] [clock]
    python qcap_cycles.py <campaign_name> --clock OCXO1 [limit]

Clock filter:
    VCLOCK, OCXO1, OCXO2, ALL

Column doctrine:
    *_v     qcap.valid
    *_svc   service offset in 10 MHz ticks
    *_cc    target correction in DWT cycles
    *_used  whether target-corrected DWT was used as the observed endpoint
    *_fix   fixed-latency event DWT
    *_cor   target-corrected event DWT
    *_obs   DWT endpoint fed to interval/gate machinery
    *_sub   final subscriber-facing DWT
    *_oΔ    obs[n] - obs[n-1], modulo DWT32
    *_cΔ    cor[n] - cor[n-1], modulo DWT32
    *_sΔ    sub[n] - sub[n-1], modulo DWT32
    *_o-c   obs - cor, signed; non-zero means service-time endpoint differs
            from the target-corrected endpoint
    *_s-c   sub - cor, signed; subscriber/predicted endpoint displacement
            relative to the target-corrected endpoint

The intended courtroom question:
    Do OCXO service-time intervals carry ISR-latency excursions that disappear
    or shrink when using the target-corrected DWT interval?
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


CLOCKS = ("VCLOCK", "OCXO1", "OCXO2")


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


def _forensics_root(rec: Dict[str, Any]) -> Dict[str, Any]:
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


def _first_bool(*values: Any) -> Optional[bool]:
    for value in values:
        out = _as_bool(value)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}"


def _fmt_bool(v: Optional[bool], width: int) -> str:
    if v is None:
        s = "---"
    else:
        s = "Y" if v else "N"
    return f"{s:>{width}s}"


def _fmt_str(v: Optional[str], width: int) -> str:
    s = "---" if v is None else str(v)
    if len(s) > width:
        s = s[:width]
    return f"{s:>{width}s}"


def _delta_u32(now: Optional[int], prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    return (now - prev) & 0xFFFFFFFF


def _signed_delta_u32(now: Optional[int], prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def normalize_clock_filter(clock: Optional[str]) -> Optional[str]:
    if clock is None:
        return None
    c = clock.strip().upper()
    if c in ("ALL", "BOTH"):
        return None
    if c in ("V", "VC", "VCLOCK"):
        return "VCLOCK"
    if c in ("O1", "OCXO1"):
        return "OCXO1"
    if c in ("O2", "OCXO2"):
        return "OCXO2"
    raise ValueError(f"unknown clock '{clock}', expected VCLOCK, OCXO1, OCXO2, or ALL")


def selected_clocks(clock_filter: Optional[str]) -> Tuple[str, ...]:
    return CLOCKS if clock_filter is None else (clock_filter,)


def _pps_count(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def _qcap(root: Dict[str, Any],
          frag: Dict[str, Any],
          forensic: Dict[str, Any],
          lane: str) -> Dict[str, Any]:
    key = lane.lower()
    q = _nested_get(forensic, key, "forensics", "qcap")
    if isinstance(q, dict):
        return q

    # Compatibility fallback for transitional flat rows, if present.
    f = _nested_get(forensic, key, "forensics")
    if isinstance(f, dict):
        return {
            "valid": _first_bool(f.get("qcap_valid"), f.get("qtimer_capture_valid")),
            "svc": _first_int(f.get("qcap_svc"), f.get("qtimer_service_offset_signed_ticks")),
            "cc": _first_int(f.get("qcap_cc"), f.get("qtimer_target_correction_cycles")),
            "fix": _first_int(f.get("qcap_fix"), f.get("qtimer_fixed_latency_event_dwt")),
            "cor": _first_int(f.get("qcap_cor"), f.get("qtimer_target_corrected_dwt_at_event")),
            "obs": _first_int(f.get("qcap_obs"), f.get("qtimer_observed_dwt_for_interval")),
            "sub": _first_int(f.get("qcap_sub"), f.get("qtimer_subscriber_dwt_at_event")),
            "used": _first_bool(f.get("qcap_used"), f.get("qtimer_target_correction_used_as_observed_endpoint")),
        }

    # Last chance: fragment should not normally carry qcap, but allow it for
    # debugging rows.
    q = _nested_get(frag, key, "forensics", "qcap")
    return q if isinstance(q, dict) else {}


def _qcap_values(q: Dict[str, Any]) -> Dict[str, Any]:
    fix = _as_int(q.get("fix"))
    cor = _as_int(q.get("cor"))
    obs = _as_int(q.get("obs"))
    sub = _as_int(q.get("sub"))

    return {
        "valid": _as_bool(q.get("valid")),
        "svc": _as_int(q.get("svc")),
        "cc": _as_int(q.get("cc")),
        "fix": fix,
        "cor": cor,
        "obs": obs,
        "sub": sub,
        "used": _as_bool(q.get("used")),
        "obs_minus_cor": _signed_delta_u32(obs, cor),
        "sub_minus_cor": _signed_delta_u32(sub, cor),
    }


def collect_rows(rows: Iterable[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int]:
    out: List[Dict[str, Any]] = []
    gaps = 0
    prev_pps: Optional[int] = None
    prev: Dict[str, Dict[str, Optional[int]]] = {
        lane: {"fix": None, "cor": None, "obs": None, "sub": None}
        for lane in CLOCKS
    }

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics_root(rec)

        pps = _pps_count(root, frag, forensic)
        if pps is None:
            continue

        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            prev = {lane: {"fix": None, "cor": None, "obs": None, "sub": None} for lane in CLOCKS}

        row: Dict[str, Any] = {"pps": pps, "lanes": {}}

        for lane in CLOCKS:
            q = _qcap_values(_qcap(root, frag, forensic, lane))
            p = prev[lane]
            q["fix_delta"] = _delta_u32(q["fix"], p["fix"])
            q["cor_delta"] = _delta_u32(q["cor"], p["cor"])
            q["obs_delta"] = _delta_u32(q["obs"], p["obs"])
            q["sub_delta"] = _delta_u32(q["sub"], p["sub"])
            row["lanes"][lane] = q

        out.append(row)

        prev_pps = pps
        for lane in CLOCKS:
            q = row["lanes"][lane]
            for field in ("fix", "cor", "obs", "sub"):
                if q[field] is not None:
                    prev[lane][field] = q[field]

    return out, gaps


def _headers(clock_filter: Optional[str]) -> Tuple[str, str]:
    parts = [f"{'pps':>6s}"]
    for lane in selected_clocks(clock_filter):
        prefix = "v" if lane == "VCLOCK" else ("o1" if lane == "OCXO1" else "o2")
        parts.extend([
            f"{prefix+'_v':>3s}",
            f"{prefix+'_svc':>6s}",
            f"{prefix+'_cc':>6s}",
            f"{prefix+'_used':>5s}",
            f"{prefix+'_fix':>13s}",
            f"{prefix+'_cor':>13s}",
            f"{prefix+'_obs':>13s}",
            f"{prefix+'_sub':>13s}",
            f"{prefix+'_oΔ':>13s}",
            f"{prefix+'_cΔ':>13s}",
            f"{prefix+'_sΔ':>13s}",
            f"{prefix+'_o-c':>8s}",
            f"{prefix+'_s-c':>8s}",
        ])
    header = "  ".join(parts)
    sep = "  ".join("─" * len(p) for p in parts)
    return header, sep


def _row_line(row: Dict[str, Any], clock_filter: Optional[str]) -> str:
    parts = [f"{row['pps']:>6d}"]
    for lane in selected_clocks(clock_filter):
        q = row["lanes"][lane]
        parts.extend([
            _fmt_bool(q["valid"], 3),
            _fmt_int(q["svc"], 6, signed=True),
            _fmt_int(q["cc"], 6, signed=True),
            _fmt_bool(q["used"], 5),
            _fmt_int(q["fix"], 13),
            _fmt_int(q["cor"], 13),
            _fmt_int(q["obs"], 13),
            _fmt_int(q["sub"], 13),
            _fmt_int(q["obs_delta"], 13),
            _fmt_int(q["cor_delta"], 13),
            _fmt_int(q["sub_delta"], 13),
            _fmt_int(q["obs_minus_cor"], 8, signed=True),
            _fmt_int(q["sub_minus_cor"], 8, signed=True),
        ])
    return "  ".join(parts)


def _summary(rows: List[Dict[str, Any]], clock_filter: Optional[str]) -> None:
    print()
    print("Summary")
    print("═══════")
    for lane in selected_clocks(clock_filter):
        valid = 0
        used = 0
        svc_counts: Dict[int, int] = {}
        cc_counts: Dict[int, int] = {}
        obs_minus_cor_counts: Dict[int, int] = {}

        for row in rows:
            q = row["lanes"][lane]
            if q["valid"]:
                valid += 1
            if q["used"]:
                used += 1
            if q["svc"] is not None:
                svc_counts[q["svc"]] = svc_counts.get(q["svc"], 0) + 1
            if q["cc"] is not None:
                cc_counts[q["cc"]] = cc_counts.get(q["cc"], 0) + 1
            if q["obs_minus_cor"] is not None:
                obs_minus_cor_counts[q["obs_minus_cor"]] = obs_minus_cor_counts.get(q["obs_minus_cor"], 0) + 1

        print(f"  {lane:<6s} valid={valid:,}/{len(rows):,}  used={used:,}/{len(rows):,}")
        print(f"         svc counts:     {dict(sorted(svc_counts.items()))}")
        print(f"         cc counts:      {dict(sorted(cc_counts.items()))}")
        print(f"         obs-cor counts: {dict(sorted(obs_minus_cor_counts.items()))}")


def analyze(campaign: str, limit: int = 0, clock_filter: Optional[str] = None) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    clock_filter = normalize_clock_filter(clock_filter) if clock_filter else None
    collected, gaps = collect_rows(rows)
    if limit:
        collected = collected[:limit]

    view = "ALL" if clock_filter is None else clock_filter
    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={view})")
    print()
    print("QCAP compact QTimer capture dump")
    print("════════════════════════════════")
    print("  *_svc  = service offset in 10 MHz ticks.")
    print("  *_cc   = target correction in DWT cycles.")
    print("  *_used = target-corrected DWT used as observed endpoint.")
    print("  *_fix  = fixed-latency event DWT.")
    print("  *_cor  = target-corrected event DWT.")
    print("  *_obs  = endpoint fed to interval/gate machinery.")
    print("  *_sub  = subscriber-facing DWT.")
    print("  *_oΔ/*_cΔ/*_sΔ are obs/cor/sub one-second intervals.")
    print("  *_o-c  = obs - cor.  *_s-c = sub - cor.")
    print()

    h, sep = _headers(clock_filter)
    print(h)
    print(sep)
    for row in collected:
        print(_row_line(row, clock_filter))

    print()
    print(f"Rows shown: {len(collected):,}")
    print(f"Gaps:       {gaps:,}")

    _summary(collected, clock_filter)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dump compact qcap TIMEBASE_FORENSICS rows.")
    parser.add_argument("campaign", help="Campaign name")
    parser.add_argument("rest", nargs="*", help="Optional [limit] [clock] in either order")
    parser.add_argument("--clock", default=None, help="VCLOCK, OCXO1, OCXO2, or ALL")
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
            clock = item

    analyze(args.campaign, limit=limit, clock_filter=clock)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
