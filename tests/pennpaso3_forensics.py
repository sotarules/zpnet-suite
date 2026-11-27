"""Read-only PennPaso3 cold-soak investigation, using ZPNet's paced report reader.

Install in zpnet-suite/tests and run: zt pennpaso3_forensics PennPaso3
Optional: --bin-minutes 5 --output /tmp/pennpaso3.csv
No database writes, no commands to hardware, no changes to calibration.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import statistics as st
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from zpnet.shared.report_db import add_read_arguments, validate_read_arguments, campaign_rows


def dig(obj, *keys):
    for k in keys:
        if not isinstance(obj, dict):
            return None
        obj = obj.get(k)
    return obj


def num(v):
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def timestamp(s):
    return datetime.fromisoformat(s.replace('Z', '+00:00')).timestamp()


def stamp(t):
    return datetime.fromtimestamp(t, timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')


def mean(values):
    v = [x for x in values if x is not None]
    return st.mean(v) if v else None


def fmt(v, decimals=3):
    return 'n/a' if v is None else f'{v:.{decimals}f}'


def pearson(a, b):
    pairs = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
    if len(pairs) < 3:
        return None
    xs, ys = zip(*pairs)
    mx, my = st.mean(xs), st.mean(ys)
    sx, sy = st.pstdev(xs), st.pstdev(ys)
    return sum((x - mx) * (y - my) for x, y in pairs) / len(pairs) / sx / sy if sx and sy else None


def read(args):
    projection = '''SELECT id, schema #>> '{}' AS report_schema, campaign,
      jsonb_build_object('published_at_utc', published_at_utc,
        'temperature', temperature, 'environment', environment,
        'measurement', measurement, 'normalization', normalization,
        'photons', jsonb_build_object(
          'snapshot_ok', photons #> '{snapshot_ok}',
          'fragment_elapsed_ms', photons #> '{fragment_elapsed_ms}',
          'race', photons #> '{race}',
          'pd_out', photons #> '{pd_out}',
          'voltage_normalization', photons #> '{voltage_normalization}',
          'stats', photons #> '{stats}',
          'core', photons #> '{core}',
          'science', photons #> '{science}',
          'projection', photons #> '{projection}'
        )) AS forensic_payload'''
    for row in campaign_rows(args.campaign, projection, args):
        p = row['forensic_payload']
        p = json.loads(p) if isinstance(p, str) else p
        ph = p['photons']
        raw = dig(p, 'measurement', 'raw_flight_ns', 'mean')
        if raw is None:
            raw = dig(ph, 'voltage_normalization', 'raw_flight_ns', 'mean')
        published = dig(ph, 'race', 'flight_ns', 'mean')
        if published is None:
            published = dig(p, 'measurement', 'flight_ns', 'mean')
        if published is None:
            published = dig(ph, 'voltage_normalization', 'published_flight_ns', 'mean')
        correction = num(dig(ph, 'voltage_normalization', 'adjustment_ns'))
        if correction is None:
            correction = num(dig(ph, 'voltage_normalization', 'correction_ns'))
        if correction is None and raw is not None and published is not None:
            correction = num(published) - num(raw)
        t = timestamp(p['published_at_utc'])
        rtd = dig(p, 'temperature', 'components', 'rtd', 'temperature_c')
        if rtd is None:
            rtd = dig(p, 'temperature', 'components', 'rtd', 'mean_temperature_c')
        high5 = dig(ph, 'pd_out', 'high5_window', 'high_mean_mv')
        high60 = dig(ph, 'pd_out', 'high_window', 'high_mean_mv')
        fw = dig(ph, 'voltage_normalization') or {}
        race = dig(ph, 'race') or {}
        n = dig(race, 'flight_ns', 'n')
        if n is None:
            n = dig(p, 'measurement', 'raw_flight_ns', 'n')
        record = dict(id=int(row['id']), t=t, utc=stamp(t), raw=num(raw), lap=num(published),
                      correction=correction, rtd=num(rtd), high5=num(high5), high60=num(high60),
                      applied_rtd=num(fw.get('rtd_temperature_c') or fw.get('applied_rtd_temperature_c')),
                      synthetic=num(dig(p, 'temperature', 'temperature_c')),
                      laser_dac=num(dig(ph, 'race', 'laser_dac')),
                      n=int(n or 0), snapshot_ok=ph.get('snapshot_ok'),
                      model=fw.get('model_id'), acquisition=dig(ph, 'stats', 'acquisition_id'),
                      reset=dig(ph, 'stats', 'reset_count'),
                      sequence=dig(p, 'photons', 'core', 'sequence'),
                      status=dig(p, 'measurement', 'status'))
        yield record


def report(rows, args):
    rows.sort(key=lambda r: (r['t'], r['id']))
    print(f'PENNPASO3 FORENSICS | {args.campaign} | {len(rows):,} persisted PHOTONS rows')
    if not rows:
        return
    print('Range:', rows[0]['utc'], 'to', rows[-1]['utc'])
    print('Model/acquisition counts:', dict(Counter((r['model'], str(r['acquisition'])) for r in rows)))
    print('Measurement statuses:', dict(Counter(str(r['status']) for r in rows)))
    print('Snapshot status:', dict(Counter(str(r['snapshot_ok']) for r in rows)))
    print('Valid LAP:', sum(r['lap'] is not None and r['n'] > 0 for r in rows),
          '| zero-accepted:', sum(r['n'] == 0 for r in rows))
    # A continuous cold-soak may have no accepted laps. Preserve those rows.
    bins = defaultdict(list)
    width = args.bin_minutes * 60
    for r in rows:
        bins[int(r['t'] // width)].append(r)
    print('\nTIME BINS (UTC; mean of one-second rows; n=accepted races summed)')
    print('start UTC                 rows valid races    LAP ns   RAW ns  CORR ns  RTD C   HIGH5 mV')
    for b, v in sorted(bins.items()):
        valid = [r for r in v if r['lap'] is not None and r['n'] > 0]
        print(f'{stamp(b*width):23s} {len(v):5d} {len(valid):5d} {sum(r["n"] for r in v):7d} '
              f'{fmt(mean([r["lap"] for r in valid])):>9s} '
              f'{fmt(mean([r["raw"] for r in valid])):>9s} '
              f'{fmt(mean([r["correction"] for r in valid])):>9s} '
              f'{fmt(mean([r["rtd"] for r in v]),2):>7s} '
              f'{fmt(mean([r["high5"] for r in v]),2):>10s}')
    valid = [r for r in rows if r['lap'] is not None and r['n'] > 0]
    if not valid:
        return
    print('\nCORRELATIONS (descriptive, one-second means; not causal inference)')
    for key in ('rtd', 'high5', 'high60', 'synthetic', 'correction', 'laser_dac'):
        print(f'  LAP vs {key:12s}: {fmt(pearson([r[key] for r in valid], [r["lap"] for r in valid]))}')
    print('\nRAW vs NORMALIZED SPAN')
    for key in ('raw', 'lap', 'correction', 'rtd', 'high5'):
        vals = [r[key] for r in valid if r[key] is not None]
        print(f'  {key:12s}: min={fmt(min(vals) if vals else None)} max={fmt(max(vals) if vals else None)}')
    print('\nLARGEST ADJACENT VALID LAP CHANGES (time gap <= 10 s)')
    pairs = [(abs(b['lap']-a['lap']), a, b) for a,b in zip(valid,valid[1:]) if 0 < b['t']-a['t'] <= 10]
    for delta,a,b in sorted(pairs, key=lambda p:p[0], reverse=True)[:12]:
        print(f'  {b["utc"]}: {fmt(a["lap"])} -> {fmt(b["lap"])} (delta {fmt(b["lap"]-a["lap"])}) '
              f'RTD {fmt(a["rtd"],2)} -> {fmt(b["rtd"],2)}; HIGH5 {fmt(a["high5"],2)} -> {fmt(b["high5"],2)}')
    print('\nGAPS IN VALID LAPS (>60 seconds):')
    gaps = [(b['t']-a['t'],a,b) for a,b in zip(valid,valid[1:]) if b['t']-a['t'] > 60]
    for duration,a,b in sorted(gaps,reverse=True)[:20]:
        print(f'  {stamp(a["t"])} -> {stamp(b["t"])} ({duration/60:.1f} min), '
              f'LAP {fmt(a["lap"])} -> {fmt(b["lap"])}; RTD {fmt(a["rtd"],2)} -> {fmt(b["rtd"],2)}')
    print('\nMATCHED SENSOR CONDITIONS (RTD 32.7..33.1 C, HIGH5 438..442 mV):')
    matched = [r for r in valid if r['rtd'] is not None and 32.7 <= r['rtd'] <= 33.1
               and r['high5'] is not None and 438 <= r['high5'] <= 442]
    for label, group in [('first third',matched[:len(matched)//3]),
                         ('middle third',matched[len(matched)//3:2*len(matched)//3]),
                         ('last third',matched[2*len(matched)//3:])]:
        print(f'  {label:13s}: n={len(group):6d} LAP={fmt(mean([r["lap"] for r in group]))} '
              f'RAW={fmt(mean([r["raw"] for r in group]))} '
              f'CORR={fmt(mean([r["correction"] for r in group]))}')
    if args.output:
        with args.output.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader(); w.writerows(rows)
        print('Saved per-second CSV:', args.output)
    print('\nInterpretation: compare RAW and normalized trajectories, then inspect RTD/HIGH5/laser drive '
          'and any changes in model/acquisition/reset. Correlation alone cannot identify physical cause.')


def main(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign', nargs='?', default='PennPaso3')
    p.add_argument('--bin-minutes', type=int, default=5)
    p.add_argument('--output', type=Path)
    add_read_arguments(p)
    args = p.parse_args(argv[1:]); validate_read_arguments(p, args)
    if args.bin_minutes < 1:
        p.error('--bin-minutes must be >= 1')
    report(list(read(args)), args)
    return 0

if __name__ == '__main__':
    import sys
    raise SystemExit(main(sys.argv))
