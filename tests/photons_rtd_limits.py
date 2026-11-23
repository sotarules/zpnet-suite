"""Report observed MAX31865 temperatures at HIGH5 rail and photon-race transitions."""
import argparse
import math
import sys
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
from statistics import median

from zpnet.shared.report_db import add_read_arguments, validate_read_arguments, campaign_rows

SQL = """SELECT id, schema #>> '{}' AS report_schema, sequence #>> '{}' AS seq,
 published_at_utc #>> '{}' AS utc, photons #>> '{stats,reset_count}' AS reset,
 photons #>> '{snapshot_ok}' AS ok, photons #>> '{fragment_period_ns}' AS period,
 temperature #>> '{components,rtd,status}' AS rstatus,
 temperature #>> '{components,rtd,source}' AS source,
 temperature #>> '{components,rtd,temperature_c}' AS temp,
 temperature #>> '{components,rtd,smoothing,window_samples}' AS smoothing,
 photons #>> '{race,active}' AS active,
 photons #>> '{race,attempts_this_fragment}' AS attempts,
 measurement #>> '{raw_flight_ns,n}' AS n,
 photons #> '{pd_out,high5_window}' AS win,
 laser_dac #>> '{commanded_code}' AS code, laser_dac #>> '{max_code}' AS cap,
 laser_dac #>> '{servo,mode}' AS mode, laser_dac #>> '{servo,target_mv}' AS target,
 laser_dac #>> '{servo,status}' AS status, laser_dac #>> '{servo,sweep,phase}' AS sweep,
 laser_dac #>> '{servo,deadband_mv}' AS band"""


class Detector:
    def __init__(self):
        self.reset()
        self.events = []

    def reset(self):
        self.confirmed = self.pending = self.last = None
        self.count = 0

    def observe(self, bad, point, length):
        if bad is None:
            self.reset()
            return
        if self.pending != bad:
            self.pending, self.count, self.first = bad, 0, point
        self.count += 1
        if self.count == length and self.confirmed != bad:
            if self.last is not None:
                self.events.append((bad, self.last, self.first))
            self.confirmed = bad
        if self.confirmed == bad:
            self.last = point


def analyze(rows, length):
    import json
    rail, race = Detector(), Detector()
    bins = defaultdict(Counter)
    stats = Counter()
    prev = None
    history = deque(maxlen=61)
    for r in rows:
        stats['read'] += 1
        if not (r['ok']=='true' and r['active']=='true' and r['mode']=='AUTO'
                and r['sweep']=='IDLE' and r['target'] is not None and float(r['target'])==440
                and r['period']=='1000000000' and r['rstatus']=='OK'
                and r['source']=='TEENSY.SPI1.MAX31865' and r['temp'] is not None
                and all(r[k] is not None for k in ('utc','seq','reset'))):
            prev = None
            stats['excluded context'] += 1
            continue
        t = datetime.fromisoformat(r['utc'].replace('Z','+00:00'))
        if t.utcoffset() is None:
            raise ValueError('timestamp lacks timezone')
        t = t.astimezone(timezone.utc)
        temp = float(r['temp'])
        if not math.isfinite(temp):
            raise ValueError('nonfinite RTD')
        seq = int(r['seq'])
        config = (r['reset'],r['smoothing'],r['cap'],r['band'])
        if prev is None or not (prev[1]+1==seq and prev[2]==config and
                                0 < (t-prev[0]).total_seconds() <= 2.5):
            rail.reset()
            race.reset()
            history.clear()
            stats['segments'] += 1
        prev = (t,seq,config)
        history.append((t,temp))
        slope = ((temp-history[0][1])*60/(t-history[0][0]).total_seconds()
                 if len(history)==61 else None)
        point = (t,temp,slope,seq)
        b = bins[math.floor(temp*2)]
        b['rows'] += 1
        w = r['win'] or {}
        if isinstance(w,str):
            w=json.loads(w)
        limited = None
        if (r['status'] in ('LIMIT_HIGH','LIMIT_LOW','IN_BAND','SETTLING')
                and all(r[k] is not None for k in ('cap','code','band'))
                and w.get('target_ms')==5000 and w.get('elapsed_ms',0)>=5000
                and w.get('fragments',0)>=5 and w.get('sample_count',0)>=1000
                and w.get('high_mean_mv') is not None):
            limited = int(r['code'])==int(r['cap']) and 440-float(w['high_mean_mv'])>float(r['band'])
            b['rail_n']+=1
            b['rail']+=limited
        rail.observe(limited,point,length)
        zero = None
        if r['attempts'] is not None and int(r['attempts'])>0 and r['n'] is not None:
            zero=int(r['n'])==0
            b['race_n']+=1
            b['zero']+=zero
        race.observe(zero,point,length)
    print('Counts:',dict(stats))
    print('RTD bin C     rows    cap+short/known    zero accepted/known (attempted fragments)')
    for k,b in sorted(bins.items()):
        print(f"{k/2:5.1f}..{(k+1)/2:4.1f} {b['rows']:7}    {b['rail']}/{b['rail_n']}                 {b['zero']}/{b['race_n']}")
    for name,det in [('DAC at upper cap AND HIGH5 below deadband',rail),
                     ('zero accepted races despite attempts',race)]:
        print(f'\n{name}: {len(det.events)} confirmed transitions; persistence={length} fragments')
        groups=defaultdict(list)
        for bad,before,after in det.events:
            t,c,s,seq=after
            direction='unknown' if s is None else 'warming' if s>.02 else 'cooling' if s<-.02 else 'flat'
            label='onset' if bad else 'recovery'
            groups[(label,direction)].append(c)
            print(f' {label:8} {t.isoformat(timespec="seconds")} seq={seq} {direction}:'
                  f' last previous state {before[1]:.2f} C -> first sustained new state {c:.2f} C')
        for (label,direction),v in sorted(groups.items()):
            print(f' observed {direction} {label}: median {median(v):.2f} C, range {min(v):.2f}..{max(v):.2f} C, events={len(v)}')
        if not det.events:
            print(' No witnessed change between sustained states; no crossing temperature estimated.')
    print('Initial states after gaps are not crossings. Counts are observations, not independent trials.')
    return 0 if bins else 1


def main(argv):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign')
    p.add_argument('--consecutive',type=int,default=30)
    add_read_arguments(p)
    a=p.parse_args(argv[1:])
    validate_read_arguments(p,a)
    if a.consecutive<1:
        p.error('--consecutive must be positive')
    print('MAX31865 operating limits |',a.campaign,'| UTC | AUTO 440 mV; no writes',flush=True)
    return analyze(campaign_rows(a.campaign,SQL,a,include_laser_dac=True),a.consecutive)


if __name__=='__main__':
    raise SystemExit(main(sys.argv))
