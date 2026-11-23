"""Read-only HIGH5 headroom diagnostic; all times are UTC."""
import argparse
import math
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from itertools import groupby
from statistics import median

from zpnet.shared.report_db import add_read_arguments, validate_read_arguments, campaign_rows


SQL = """SELECT id, schema #>> '{}' AS report_schema, sequence #>> '{}' AS seq,
 published_at_utc #>> '{}' AS utc, photons #>> '{stats,reset_count}' AS reset,
 photons #>> '{snapshot_ok}' AS ok,
 photons #> '{pd_out,high5_window}' AS win,
 environment #>> '{pd,temperature_c}' AS pd,
 temperature #>> '{components,rtd,temperature_c}' AS rtd,
 laser_dac #>> '{commanded_code}' AS code, laser_dac #>> '{max_code}' AS cap,
 laser_dac #>> '{commanded_dac_v}' AS v, laser_dac #>> '{servo,mode}' AS mode,
 laser_dac #>> '{servo,status}' AS status,
 laser_dac #>> '{servo,sweep,phase}' AS sweep, laser_dac #>> '{servo,target_mv}' AS target,
 laser_dac #>> '{servo,deadband_mv}' AS band"""


def analyze(rows):
    import json
    skipped = Counter()
    data = []
    segment = 0
    for r in rows:
        w = r['win'] or {}
        if isinstance(w, str):
            w = json.loads(w)
        why = None
        if r['ok'] != 'true' or r['mode'] != 'AUTO' or r['sweep'] != 'IDLE':
            why = 'not normal automatic acquisition'
        elif r['status'] not in ('IN_BAND', 'SETTLING', 'LIMIT_HIGH', 'LIMIT_LOW'):
            why = 'other servo status'
        elif not (w.get('target_ms') == 5000 and w.get('elapsed_ms', 0) >= 5000
                  and w.get('fragments', 0) >= 5 and w.get('sample_count', 0) >= 1000):
            why = 'incomplete HIGH5'
        elif any(r[k] is None for k in ('utc','reset','seq','code','cap','target','band','v')) or w.get('high_mean_mv') is None:
            why = 'missing witness'
        elif float(r['target']) != 440:
            why = 'other target'
        if why:
            skipped[why] += 1
            segment += 1
            continue
        ts = datetime.fromisoformat(r['utc'].replace('Z','+00:00'))
        if ts.utcoffset() is None:
            raise ValueError('publication timestamp lacks timezone')
        ts = ts.astimezone(timezone.utc)
        seq = int(r['seq'])
        if data:
            prev = data[-1]
            if (seq != prev['seq'] + 1 or r['reset'] != prev['reset'] or r['cap'] != prev['cap']
                    or not 0 < (ts-prev['ts']).total_seconds() <= 2.5):
                segment += 1
        high = float(w['high_mean_mv'])
        at_cap = int(r['code']) == int(r['cap'])
        data.append(dict(ts=ts, seq=seq, reset=r['reset'], cap=r['cap'], segment=segment,
                         high=high, v=float(r['v']), at_cap=at_cap,
                         limited=at_cap and 440-high > float(r['band']),
                         tracking=not at_cap and abs(440-high) <= float(r['band']),
                         pd=None if r['pd'] is None else float(r['pd']),
                         rtd=None if r['rtd'] is None else float(r['rtd'])))
    print('Usable one-second observations:',len(data),' omitted:',dict(skipped))
    if not data:
        print('No eligible 440 mV AUTO observations; no conclusion.')
        return 1
    def med(group,k):
        a = [x[k] for x in group if x[k] is not None]
        return f'{median(a):.4f}' if a else '-'
    def line(g):
        return (f"{g[0]['ts'].isoformat(timespec='seconds')}  n={len(g)}"
                f"  PD_C={med(g,'pd')} RTD_C={med(g,'rtd')} DAC_V={med(g,'v')}"
                f" HIGH5_mV={med(g,'high')} cap/short={sum(x['at_cap'] for x in g)}/{sum(x['limited'] for x in g)}")
    print('Chronological 30-minute bins (medians; cap/short = counts at cap / at cap below deadband):')
    for _,g in groupby(data,lambda x:int(x['ts'].timestamp())//1800):
        print(line(list(g)))
    episodes = []
    for (seg,limited),g in groupby(enumerate(data),lambda pair:(pair[1]['segment'],pair[1]['limited'])):
        indexes = [i for i,_ in g]
        if limited and len(indexes)>=30:
            episodes.append((indexes[0],indexes[-1]))
    print('Sustained shortages (>=30 consecutive observations):',len(episodes))
    for a,b in episodes[:10]:
        print(' start',line(data[a:b+1]),' through',data[b]['ts'].isoformat(timespec='seconds'))
    if episodes:
        a,b = episodes[0]
        print('First sustained shortage: five-minute lead-in and follow-up bins within continuous segment:')
        for off in range(-3,4):
            subset=[x for x in data[max(0,a+off*300):max(0,a+(off+1)*300)]
                    if x['segment']==data[a]['segment']]
            if subset:
                print(line(subset))
        regained=next((x for x in data[b+1:] if x['segment']==data[b]['segment'] and x['tracking']),None)
        print('First subsequent in-band sample below cap in same segment:',
              line([regained]) if regained else 'not observed')
    split = (data[-1]['ts']-data[0]['ts'])/2 + data[0]['ts']
    for sensor in ('pd','rtd'):
        bins=defaultdict(lambda:[[],[]])
        for x in data:
            if x['tracking'] and x[sensor] is not None:
                bins[math.floor(x[sensor]*2)][int(x['ts']>=split)].append(x['v'])
        print(sensor,'matched 0.5 C bins, in-band below cap only. Early/late half median DAC V, counts, delta V:')
        matched=0
        for k,(early,late) in sorted(bins.items()):
            if min(len(early),len(late)) >= 60:
                matched+=1
                print(f' {k/2:.1f}..{(k+1)/2:.1f}: {median(early):.4f}/{median(late):.4f}'
                      f' n={len(early)}/{len(late)} delta={median(late)-median(early):+.4f}')
        if not matched:
            print(' No bins with 60 observations in each half; revisit comparison inconclusive.')
    print('DAC is Pi command at receipt, not measured laser power. Counts include overlapping windows.')
    return 0


def main(argv):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign')
    add_read_arguments(p)
    args=p.parse_args(argv[1:])
    validate_read_arguments(p,args)
    print('PHOTONS HIGH5 HEADROOM:',args.campaign,flush=True)
    return analyze(campaign_rows(args.campaign,SQL,args,include_laser_dac=True))


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
