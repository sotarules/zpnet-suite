from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Sequence

from zpnet.shared.db import open_db

DWT_NS_PER_CYCLE = 125.0 / 126.0


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
    out = []
    for row in rows:
        payload = row['payload']
        if isinstance(payload, str):
            payload = json.loads(payload)
        out.append(payload)
    return out


def stats(values: Sequence[int]) -> tuple[float, float, int, int]:
    if not values:
        return 0.0, 0.0, 0, 0
    mean = sum(values) / len(values)
    if len(values) >= 2:
        var = sum((v - mean) ** 2 for v in values) / (len(values) - 1)
        sd = math.sqrt(var)
    else:
        sd = 0.0
    return mean, sd, min(values), max(values)


def print_hist(title: str, values: Sequence[int]) -> None:
    print('-' * 78)
    print(title)
    print('-' * 78)
    if not values:
        print('  no samples\n')
        return
    counts = Counter(values)
    total = len(values)
    for v, c in sorted(counts.items()):
        print(f'  {v:>4d}: {c:>6d} ({100.0*c/total:5.1f}%)')
    print()


def analyze_ocxo(rows: List[Dict[str, Any]], key: str) -> None:
    prev_phase = []
    win_phase = []
    prev_br = []
    win_br = []
    iters = []
    residuals = []
    deltas = []

    for row in rows:
        if not row.get('phase_detector_valid'):
            continue
        a = row.get(f'{key}_match_prev_phase_offset_ns')
        b = row.get(f'{key}_winner_phase_offset_ns')
        c = row.get(f'{key}_match_prev_dwt_bracket_cycles')
        d = row.get(f'{key}_winner_dwt_bracket_cycles')
        i = row.get(f'{key}_match_iteration_count')
        r = row.get(f'{key}_residual_ns')
        x = row.get(f'{key}_match_delta_ns')
        if None in (a, b, c, d, i, x):
            continue
        prev_phase.append(int(a))
        win_phase.append(int(b))
        prev_br.append(int(c))
        win_br.append(int(d))
        iters.append(int(i))
        deltas.append(int(x))
        if r is not None:
            residuals.append(int(r))

    print('=' * 78)
    print(f'{key.upper()} DETECTOR ANALYSIS')
    print('=' * 78)
    mean_i, sd_i, min_i, max_i = stats(iters)
    print(f'  matches:              {len(iters):,}')
    print(f'  iterations mean/sd:   {mean_i:,.2f} / {sd_i:,.2f}')
    print(f'  iterations range:     {min_i} -> {max_i}')
    if residuals:
        mean_r, sd_r, min_r, max_r = stats(residuals)
        print(f'  residual mean/sd ns:  {mean_r:,.2f} / {sd_r:,.2f}')
        print(f'  residual range ns:    {min_r:+d} -> {max_r:+d}')
    print()
    print_hist('MATCH ITERATION HISTOGRAM', iters)
    print_hist('MATCH DELTA HISTOGRAM (should be all 0)', deltas)
    print_hist('PREV BRACKET HISTOGRAM', prev_br)
    print_hist('WINNER BRACKET HISTOGRAM', win_br)


def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return
    print('=' * 78)
    print('ZPNet EDGE TDC ANALYZER — GENERALIZED CONSECUTIVE-MATCH DETECTOR')
    print(f'Campaign: {campaign} ({len(rows)} TIMEBASE records)')
    print('=' * 78)
    print_hist('GLOBAL DETECTOR VALID HISTOGRAM', [1 if r.get('phase_detector_valid') else 0 for r in rows])
    analyze_ocxo(rows, 'ocxo1')
    analyze_ocxo(rows, 'ocxo2')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: edge_tdc_analyzer <campaign_name>')
        sys.exit(1)
    analyze(sys.argv[1])
