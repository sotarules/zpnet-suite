"""
ZPNet OCXO Edge Forensics — row-oriented analyzer for quantized DWT regimes

Purpose:
  Inspect one TIMEBASE row per second and surface the direct observables around
  OCXO edge timing so quantized regimes are easy to spot.

Shows, per row:
  - pps_count
  - DWT cycles between PPS
  - OCXO1/2 dwt_at_edge
  - OCXO1/2 dwt_cycles_between_edges
  - OCXO1/2 gnss_ns_between_edges
  - OCXO1/2 second_residual_ns
  - OCXO1/2 ISR correction cycles
  - OCXO1/2 shadow→ISR delta (raw - shadow)
  - OCXO1/2 raw→adjusted correction check
  - OCXO1/2 edge phase vs PPS (gnss_ns_at_edge - gnss_ns_at_pps)

Usage:
    python -m zpnet.tests.ocxo_edge_forensics <campaign> [limit]
    .zt ocxo_edge_forensics Phase1
    .zt ocxo_edge_forensics Phase1 100
"""

from __future__ import annotations

import json
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


def _safe_int(v: Any) -> Optional[int]:
    try:
        if v is None:
            return None
        return int(v)
    except Exception:
        return None


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        result.append(p)
    return result


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()
    print(
        f"{'pps':>5s}  "
        f"{'dwt_pps':>10s}  "
        f"{'o1_dwtΔ':>10s}  {'o2_dwtΔ':>10s}  "
        f"{'o1_gnssΔ':>10s}  {'o2_gnssΔ':>10s}  "
        f"{'o1_res':>7s}  {'o2_res':>7s}  "
        f"{'o1_sh→isr':>10s}  {'o2_sh→isr':>10s}  "
        f"{'o1_corr':>7s}  {'o2_corr':>7s}  "
        f"{'o1_phase':>10s}  {'o2_phase':>10s}"
    )
    print(
        f"{'─'*5}  "
        f"{'─'*10}  "
        f"{'─'*10}  {'─'*10}  "
        f"{'─'*10}  {'─'*10}  "
        f"{'─'*7}  {'─'*7}  "
        f"{'─'*10}  {'─'*10}  "
        f"{'─'*7}  {'─'*7}  "
        f"{'─'*10}  {'─'*10}"
    )

    shown = 0
    for rec in rows:
        frag = rec.get("fragment", {}) if isinstance(rec.get("fragment"), dict) else {}
        if not frag:
            continue

        pps = _safe_int(rec.get("pps_count"))
        gnss_pps = _safe_int(rec.get("teensy_gnss_ns"))
        dwt_between_pps = _safe_int(frag.get("dwt_cycle_count_between_pps"))

        o1_dwt_delta = _safe_int(frag.get("ocxo1_dwt_cycles_between_edges"))
        o2_dwt_delta = _safe_int(frag.get("ocxo2_dwt_cycles_between_edges"))

        o1_gnss_delta = _safe_int(frag.get("ocxo1_gnss_ns_between_edges"))
        o2_gnss_delta = _safe_int(frag.get("ocxo2_gnss_ns_between_edges"))

        o1_res = _safe_int(frag.get("ocxo1_second_residual_ns"))
        o2_res = _safe_int(frag.get("ocxo2_second_residual_ns"))

        o1_shadow = _safe_int(frag.get("ocxo1_diag_shadow_dwt"))
        o2_shadow = _safe_int(frag.get("ocxo2_diag_shadow_dwt"))

        o1_raw = _safe_int(frag.get("ocxo1_diag_dwt_isr_entry_raw"))
        o2_raw = _safe_int(frag.get("ocxo2_diag_dwt_isr_entry_raw"))

        o1_adj = _safe_int(frag.get("ocxo1_diag_dwt_at_event_adjusted"))
        o2_adj = _safe_int(frag.get("ocxo2_diag_dwt_at_event_adjusted"))

        o1_corr = _safe_int(frag.get("ocxo1_diag_dwt_event_correction_cycles"))
        o2_corr = _safe_int(frag.get("ocxo2_diag_dwt_event_correction_cycles"))

        o1_edge_gnss = _safe_int(frag.get("ocxo1_gnss_ns_at_edge"))
        o2_edge_gnss = _safe_int(frag.get("ocxo2_gnss_ns_at_edge"))

        o1_shadow_to_isr = (o1_raw - o1_shadow) if (o1_raw is not None and o1_shadow is not None) else None
        o2_shadow_to_isr = (o2_raw - o2_shadow) if (o2_raw is not None and o2_shadow is not None) else None

        # Sanity check: raw - adjusted should equal correction cycles
        o1_corr_check = (o1_raw - o1_adj) if (o1_raw is not None and o1_adj is not None) else None
        o2_corr_check = (o2_raw - o2_adj) if (o2_raw is not None and o2_adj is not None) else None

        o1_phase = (o1_edge_gnss - gnss_pps) if (o1_edge_gnss is not None and gnss_pps is not None) else None
        o2_phase = (o2_edge_gnss - gnss_pps) if (o2_edge_gnss is not None and gnss_pps is not None) else None

        def fmt(v: Optional[int], width: int) -> str:
            return f"{v:>{width}d}" if v is not None else f"{'—':>{width}s}"

        print(
            f"{fmt(pps, 5)}  "
            f"{fmt(dwt_between_pps, 10)}  "
            f"{fmt(o1_dwt_delta, 10)}  {fmt(o2_dwt_delta, 10)}  "
            f"{fmt(o1_gnss_delta, 10)}  {fmt(o2_gnss_delta, 10)}  "
            f"{fmt(o1_res, 7)}  {fmt(o2_res, 7)}  "
            f"{fmt(o1_shadow_to_isr, 10)}  {fmt(o2_shadow_to_isr, 10)}  "
            f"{fmt(o1_corr_check, 7)}  {fmt(o2_corr_check, 7)}  "
            f"{fmt(o1_phase, 10)}  {fmt(o2_phase, 10)}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print("Notes:")
    print("  o*_dwtΔ     = ocxo*_dwt_cycles_between_edges")
    print("  o*_gnssΔ    = ocxo*_gnss_ns_between_edges")
    print("  o*_sh→isr   = diag_dwt_isr_entry_raw - diag_shadow_dwt")
    print("  o*_corr     = diag_dwt_isr_entry_raw - diag_dwt_at_event_adjusted")
    print("  o*_phase    = ocxo*_gnss_ns_at_edge - teensy_gnss_ns")
    print()
    print("Look for:")
    print("  • exact plateau switching in o*_dwtΔ")
    print("  • whether plateau switches correlate with o*_sh→isr")
    print("  • whether correction check stays constant (it should)")
    print("  • whether phase changes track the same regime changes")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: ocxo_edge_forensics <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
