
from __future__ import annotations

"""
ZPNet Phase Analyzer — two-phase validation view

Usage:
    python -m zpnet.tests.phase_analyzer <campaign_name> [limit]
    .zt phase_analyzer <campaign_name>
    .zt phase_analyzer <campaign_name> 120

Purpose:
    Print one TIMEBASE row per PPS and surface the two-phase OCXO capture
    facts needed to validate the modulo-100 idea directly.

For each OCXO we show:
    - phase #1 offset
    - phase #2 offset
    - wrapped phase delta (phase2 - phase1)
    - phase #1 / #2 elapsed ns
    - phase #1 / #2 bracket cycles
    - phase #1 / #2 GPT deltas
    - correction cycles
    - legacy residual fields for context only
"""

import json
import sys
from typing import Any, Dict, List

from zpnet.shared.db import open_db

DWT_NS_PER_CYCLE = 125.0 / 126.0


def _fmt_int(value: Any) -> str:
    if value is None:
        return "---"
    try:
        return f"{int(value)}"
    except Exception:
        return str(value)


def _fmt_signed(value: Any) -> str:
    if value is None:
        return "---"
    try:
        return f"{int(value):+d}"
    except Exception:
        return str(value)


def _fmt_float(value: Any, places: int = 1) -> str:
    if value is None:
        return "---"
    try:
        return f"{float(value):.{places}f}"
    except Exception:
        return str(value)


def _fmt_pair_valid(value: Any) -> str:
    if value is None:
        return "---"
    return "YES" if bool(value) else "NO"


def _fetch_rows(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC, ts ASC
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


def _gpt_delta(row: Dict[str, Any], base: str) -> Any:
    before = row.get(f"{base}_gpt_before")
    after = row.get(f"{base}_gpt_after")
    if before is None or after is None:
        return None
    return int(after) - int(before)


def _header() -> str:
    return (
        f"{'PPS':>6}  {'GNSS_NS':>14}  "
        f"{'O1_R1':>5} {'O1_R2':>5} {'O1_ΔR':>6}  "
        f"{'O1_A1':>5} {'O1_A2':>5} {'O1_ΔA':>6}  "
        f"{'O1_E1':>6} {'O1_E2':>6}  "
        f"{'O1_B1':>5} {'O1_B2':>5}  "
        f"{'O1_G1':>5} {'O1_G2':>5}  "
        f"{'O1_X1':>5} {'O1_X2':>5}  "
        f"{'O1_RES':>7}  "
        f"{'O2_R1':>5} {'O2_R2':>5} {'O2_ΔR':>6}  "
        f"{'O2_A1':>5} {'O2_A2':>5} {'O2_ΔA':>6}  "
        f"{'O2_E1':>6} {'O2_E2':>6}  "
        f"{'O2_B1':>5} {'O2_B2':>5}  "
        f"{'O2_G1':>5} {'O2_G2':>5}  "
        f"{'O2_X1':>5} {'O2_X2':>5}  "
        f"{'O2_RES':>7}  "
        f"{'PAIR':>4}"
    )


def _row(row: Dict[str, Any]) -> str:
    return (
        f"{_fmt_int(row.get('pps_count')):>6}  "
        f"{_fmt_int(row.get('teensy_gnss_ns')):>14}  "
        f"{_fmt_int(row.get('ocxo1_raw_phase_offset_ns')):>5} "
        f"{_fmt_int(row.get('ocxo1_phase2_raw_phase_offset_ns')):>5} "
        f"{_fmt_signed(row.get('ocxo1_phase_pair_delta_ns')):>6}  "
        f"{_fmt_int(row.get('ocxo1_phase_offset_ns')):>5} "
        f"{_fmt_int(row.get('ocxo1_phase2_phase_offset_ns')):>5} "
        f"{_fmt_signed(row.get('ocxo1_adjusted_phase_pair_delta_ns')):>6}  "
        f"{_fmt_int(row.get('ocxo1_raw_elapsed_ns')):>6} "
        f"{_fmt_int(row.get('ocxo1_phase2_raw_elapsed_ns')):>6}  "
        f"{_fmt_int(row.get('ocxo1_dwt_bracket_cycles')):>5} "
        f"{_fmt_int(row.get('ocxo1_phase2_dwt_bracket_cycles')):>5}  "
        f"{_fmt_signed(_gpt_delta(row, 'ocxo1')):>5} "
        f"{_fmt_signed(_gpt_delta(row, 'ocxo1_phase2')):>5}  "
        f"{_fmt_signed(row.get('ocxo1_phase_bias_ns')):>5} "
        f"{_fmt_signed(row.get('ocxo1_phase2_phase_bias_ns')):>5}  "
        f"{_fmt_signed(row.get('ocxo1_residual_ns')):>7}  "
        f"{_fmt_int(row.get('ocxo2_raw_phase_offset_ns')):>5} "
        f"{_fmt_int(row.get('ocxo2_phase2_raw_phase_offset_ns')):>5} "
        f"{_fmt_signed(row.get('ocxo2_phase_pair_delta_ns')):>6}  "
        f"{_fmt_int(row.get('ocxo2_phase_offset_ns')):>5} "
        f"{_fmt_int(row.get('ocxo2_phase2_phase_offset_ns')):>5} "
        f"{_fmt_signed(row.get('ocxo2_adjusted_phase_pair_delta_ns')):>6}  "
        f"{_fmt_int(row.get('ocxo2_raw_elapsed_ns')):>6} "
        f"{_fmt_int(row.get('ocxo2_phase2_raw_elapsed_ns')):>6}  "
        f"{_fmt_int(row.get('ocxo2_dwt_bracket_cycles')):>5} "
        f"{_fmt_int(row.get('ocxo2_phase2_dwt_bracket_cycles')):>5}  "
        f"{_fmt_signed(_gpt_delta(row, 'ocxo2')):>5} "
        f"{_fmt_signed(_gpt_delta(row, 'ocxo2_phase2')):>5}  "
        f"{_fmt_signed(row.get('ocxo2_phase_bias_ns')):>5} "
        f"{_fmt_signed(row.get('ocxo2_phase2_phase_bias_ns')):>5}  "
        f"{_fmt_signed(row.get('ocxo2_residual_ns')):>7}  "
        f"{_fmt_pair_valid(row.get('phase_pair_valid')):>4}"
    )


def analyze(campaign: str, limit: int = 0) -> None:
    rows = _fetch_rows(campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return
    if limit > 0:
        rows = rows[:limit]

    first = rows[0]
    last = rows[-1]
    print("=" * 220)
    print("PHASE ANALYZER — RAW/ADJUSTED TWO-PHASE VALIDATION")
    print("=" * 220)
    print(f"records:    {len(rows):,}")
    print(f"pps range:  {first.get('pps_count')} -> {last.get('pps_count')}")
    print(f"time range: {first.get('gnss_time_utc', '?')} -> {last.get('gnss_time_utc', '?')}")
    print()
    print(_header())
    print("-" * 220)
    for row in rows:
        print(_row(row))


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: phase_analyzer <campaign_name> [limit]")
        sys.exit(1)
    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) >= 3 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
