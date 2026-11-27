"""Investigate a LANTERN START/FLASH_CUT LAP bookend using persisted PHOTONS.

Install in zpnet-suite/tests/ and run:
    zt photons_boundary_witness HeatTest2
    zt photons_boundary_witness HeatTest2 --before 120 --after 120

Uses ZPNet's existing psycopg/open_db configuration through the zt runner.
Read-only PostgreSQL queries; all results go to STDOUT (PuTTY).
No files, device commands, changes to firmware, or campaign mutations.
"""

from __future__ import annotations

import argparse
import math
import sys
from datetime import timezone

from zpnet.shared.db import open_db


# Each field comes from the canonical PHOTONS_V1 campaign_detail.payload.
DETAIL_SQL = """
    SELECT id, ts, campaign, sequence,
           payload #>> '{{photons,stats,lap_buckets,60_sec,mean_lap_ns}}' AS lap60,
           payload #>> '{{photons,stats,lap_buckets,60_sec,sample_count}}' AS lap60_n,
           payload #>> '{{photons,race,flight_ns,mean}}' AS lap1,
           payload #>> '{{measurement,raw_flight_ns,mean}}' AS raw1,
           payload #>> '{{measurement,normalized_flight_ns,mean}}' AS published1,
           payload #>> '{{photons,pd_out,high1_mv}}' AS high1,
           payload #>> '{{photons,pd_out,high5_window,high_mean_mv}}' AS high5,
           payload #>> '{{laser_dac,commanded_dac_v}}' AS dac,
           payload #>> '{{photons,stats,rolling_ppb_current_sequence}}' AS rolling_sequence,
           payload #>> '{{photons,stats,rolling_ppb_checkpoint,60_sec,anchor,sequence}}' AS anchor_sequence
      FROM campaign_detail
     WHERE campaign_type = 'LANTERN' AND id {comparator} %s
     ORDER BY id {direction} LIMIT %s
"""


def number(value):
    return None if value is None else float(value)


def display(value, digits=3):
    return '---' if value is None else f'{number(value):,.{digits}f}'


def stamp(value):
    return value.astimezone(timezone.utc).strftime('%H:%M:%S')


def run(campaign, before, after):
    with open_db(row_dict=True) as db:
        db.execute('SET TRANSACTION READ ONLY')
        master = db.execute("""
            SELECT id, ts, campaign, active, payload
              FROM campaign_master
             WHERE campaign_type = 'LANTERN' AND campaign = %s
             ORDER BY id DESC LIMIT 1
        """, (campaign,)).fetchone()
        if master is None:
            raise ValueError(f'No LANTERN campaign named {campaign!r}')

        payload = master['payload']
        boundary_sequence = int(payload['start_after_sequence'])
        saved_start = number(payload['start_lap_ns'])
        first = db.execute("""
            SELECT id, sequence, ts FROM campaign_detail
             WHERE campaign_type = 'LANTERN' AND campaign = %s
             ORDER BY id ASC LIMIT 1
        """, (campaign,)).fetchone()
        if first is None:
            raise ValueError(f'{campaign!r} has no persisted PHOTONS detail yet')

        earlier = db.execute(DETAIL_SQL.format(comparator='<=', direction='DESC'),
                             (first['id'], before + 1)).fetchall()
        later = db.execute(DETAIL_SQL.format(comparator='>', direction='ASC'),
                           (first['id'], after)).fetchall()
        rows = sorted(earlier + later, key=lambda row: row['id'])

        previous_name = payload.get('flash_cut_from')
        previous = (db.execute("""
            SELECT payload FROM campaign_master
             WHERE campaign_type = 'LANTERN' AND campaign = %s
             ORDER BY id DESC LIMIT 1
        """, (previous_name,)).fetchone()
                    if previous_name else None)

    predecessor = next((r for r in reversed(rows)
                        if r['id'] < first['id'] and int(r['sequence']) == boundary_sequence), None)
    print('PHOTONS CAMPAIGN BOUNDARY WITNESS (READ ONLY)')
    print('Campaign:', campaign, '| master id:', master['id'],
          '| command time UTC:', master['ts'].astimezone(timezone.utc).isoformat())
    print('First public row: id', first['id'], '| sequence', first['sequence'],
          '| UTC', first['ts'].astimezone(timezone.utc).isoformat())
    print('Firmware START-after sequence:', boundary_sequence)
    print('Stored START LAP:', display(saved_start, 6), 'ns')
    if previous_name:
        print('Flash-cut from:', previous_name,
              '| stored prior END LAP:', display(previous['payload'].get('end_lap_ns'), 6)
              if previous else '---')
    if predecessor is None:
        print('BOUNDARY ROW NOT IN SELECTED HISTORY. Rerun with --before 500.')
    else:
        actual = number(predecessor['lap60'])
        print('Physical boundary row:', predecessor['id'],
              '| previous campaign:', predecessor['campaign'],
              '| UTC:', stamp(predecessor['ts']))
        print('Boundary producer 60-SEC LAP:', display(actual, 6), 'ns',
              '| difference (stored - boundary):',
              display(saved_start - actual, 6) if saved_start is not None and actual is not None else '---', 'ns')
        print('60-SEC window sample count:', predecessor['lap60_n'],
              '| rolling sequence:', predecessor['rolling_sequence'],
              '| anchor sequence:', predecessor['anchor_sequence'])
    print()
    print('REL    UTC       CAMPAIGN             LAP60(ns)  LAP1(ns)  RAW1(ns)  HIGH1(mV)  HIGH5(mV)  DAC(V)  60s span')
    for row in rows:
        rel = int(row['sequence']) - boundary_sequence
        lap1 = number(row['lap1'])
        high5 = number(row['high5'])
        # High-signal rows and a regular 10-second grid keep PuTTY output concise.
        show = (rel % 10 == 0 or -5 <= rel <= 5
                or (lap1 is not None and lap1 > 5155.0)
                or (high5 is not None and high5 < 435.0))
        if not show:
            continue
        span = (int(row['rolling_sequence']) - int(row['anchor_sequence'])
                if row['rolling_sequence'] is not None and row['anchor_sequence'] is not None
                else None)
        print(f'{rel:+5d}  {stamp(row["ts"])}  {str(row["campaign"] or "-")[:19]:19s}'
              f' {display(row["lap60"]):>10s} {display(lap1):>9s}'
              f' {display(row["raw1"]):>9s} {display(row["high1"]):>10s}'
              f' {display(high5):>10s} {display(row["dac"], 4):>7s}'
              f' {str(span) if span is not None else "---":>9s}')
    print('\nREADOUT: Boundary 60-SEC should match stored START LAP. A high LAP1 and low HIGH5')
    print('near REL 0 indicate an acquisition/servo transient rather than a bookend-copy error.')
    print('All data are immutable recorded observations; the script performs no writes.')


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('campaign', nargs='?', default='HeatTest2')
    parser.add_argument('--before', type=int, default=100)
    parser.add_argument('--after', type=int, default=100)
    args = parser.parse_args(argv[1:])
    if not 10 <= args.before <= 1200 or not 10 <= args.after <= 1200:
        parser.error('--before and --after must be between 10 and 1200')
    run(args.campaign, args.before, args.after)
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
