from __future__ import annotations

import json
import sys
from typing import Any, Dict, List

from zpnet.shared.db import open_db


def _fmt(value: Any, width: int = 0, signed: bool = False) -> str:
    if value is None:
        s = '---'
    else:
        try:
            iv = int(value)
            s = f'{iv:+d}' if signed else f'{iv}'
        except Exception:
            s = str(value)
    return f'{s:>{width}}' if width else s


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
    out = []
    for row in rows:
        payload = row['payload']
        if isinstance(payload, str):
            payload = json.loads(payload)
        out.append(payload)
    return out


def _header() -> str:
    return (
        f"{'PPS':>6}  "
        f"{'O1_PREV':>7} {'O1_WIN':>6} {'O1_IT':>5} {'O1_Δ':>6} {'O1_B1':>6} {'O1_B2':>6} {'O1_RES':>8}  "
        f"{'O2_PREV':>7} {'O2_WIN':>6} {'O2_IT':>5} {'O2_Δ':>6} {'O2_B1':>6} {'O2_B2':>6} {'O2_RES':>8}  "
        f"{'DET':>4}"
    )


def _row(row: Dict[str, Any]) -> str:
    return (
        f"{_fmt(row.get('pps_count') or row.get('teensy_pps_count'), 6)}  "
        f"{_fmt(row.get('ocxo1_match_prev_phase_offset_ns'), 7)} "
        f"{_fmt(row.get('ocxo1_winner_phase_offset_ns'), 6)} "
        f"{_fmt(row.get('ocxo1_match_iteration_count'), 5)} "
        f"{_fmt(row.get('ocxo1_match_delta_ns'), 6, signed=True)} "
        f"{_fmt(row.get('ocxo1_match_prev_dwt_bracket_cycles'), 6)} "
        f"{_fmt(row.get('ocxo1_winner_dwt_bracket_cycles'), 6)} "
        f"{_fmt(row.get('ocxo1_residual_ns'), 8, signed=True)}  "
        f"{_fmt(row.get('ocxo2_match_prev_phase_offset_ns'), 7)} "
        f"{_fmt(row.get('ocxo2_winner_phase_offset_ns'), 6)} "
        f"{_fmt(row.get('ocxo2_match_iteration_count'), 5)} "
        f"{_fmt(row.get('ocxo2_match_delta_ns'), 6, signed=True)} "
        f"{_fmt(row.get('ocxo2_match_prev_dwt_bracket_cycles'), 6)} "
        f"{_fmt(row.get('ocxo2_winner_dwt_bracket_cycles'), 6)} "
        f"{_fmt(row.get('ocxo2_residual_ns'), 8, signed=True)}  "
        f"{('YES' if row.get('phase_detector_valid') else 'NO'):>4}"
    )


def analyze(campaign: str, limit: int = 0) -> None:
    rows = _fetch_rows(campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return
    if limit > 0:
        rows = rows[:limit]

    print('=' * 150)
    print('PHASE ANALYZER — GENERALIZED CONSECUTIVE-MATCH DETECTOR')
    print('=' * 150)
    print(f"records: {len(rows):,}")
    print(_header())
    print('-' * 150)
    for row in rows:
        print(_row(row))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: phase_analyzer <campaign_name> [limit]')
        sys.exit(1)
    analyze(sys.argv[1], int(sys.argv[2]) if len(sys.argv) >= 3 else 0)
