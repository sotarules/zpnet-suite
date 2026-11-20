"""Compare frozen decay slopes against no correction on later campaign data.

Read-only operator report. Raw admitted DAC targets supply exact integer OLS
statistics; cumulative means and already-normalized PPB never train a fit.
Each forecast shares a past-only one-hour reference with its no-correction
comparison. Scoring uses minute means, not independent-sample uncertainty.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from fractions import Fraction
import json
import math
from statistics import median
import sys
from time import monotonic

from zpnet.shared.clocks_decay import CODE_SCALE, LANES, NS, PPB_PER_CODE, VOLTS_PER_CODE

HOUR = 3600 * NS
MINUTE = 60 * NS


def history_rows(campaign, batch_size, metadata):
    from psycopg.errors import QueryCanceled
    from zpnet.shared.db import open_db

    with open_db(row_dict=True) as db:
        db.execute('SET TRANSACTION READ ONLY')
        db.execute("SET LOCAL statement_timeout = '30s'")
        master = db.execute(
            "SELECT id FROM campaign_master WHERE campaign_type='TEMPEST' AND campaign=%s",
            (campaign,),
        ).fetchone()
        if master is None:
            raise LookupError(f'TEMPEST campaign not found: {campaign}')
        bounds = db.execute(
            "SELECT MIN(id) AS first_id, MAX(id) AS last_id FROM campaign_detail "
            "WHERE campaign_type='TEMPEST' AND campaign=%s", (campaign,),
        ).fetchone()
    first, tail = bounds['first_id'], bounds['last_id']
    metadata['through_detail_id'] = tail
    reader = dict(pages=0, indexed_rows=0, payload_rows=0, timeouts=0,
                  initial_batch_size=batch_size, final_batch_size=batch_size)
    metadata['reader'] = reader
    if tail is None:
        return
    after = first - 1
    progress_at = monotonic()
    print(f'Scanning detail IDs {first}..{tail}, at most {batch_size} IDs per page.',
          file=sys.stderr, flush=True)
    # Page the primary key before touching JSON. LIMIT on the old query bounded
    # matching results, not the payload work needed to find those results.
    # Scanning all types by id avoids requiring a new campaign-specific index.
    # Empty/noncanonical pages still advance the scanned watermark.
    while after < tail:
        try:
            with open_db(row_dict=True) as db:
                db.execute('SET TRANSACTION READ ONLY')
                db.execute("SET LOCAL statement_timeout = '30s'")
                page = db.execute(
                    "SELECT id, campaign_type, campaign FROM campaign_detail "
                    "WHERE id > %s AND id <= %s ORDER BY id LIMIT %s",
                    (after, tail, batch_size),
                ).fetchall()
                ids = [r['id'] for r in page
                       if r['campaign_type'] == 'TEMPEST' and r['campaign'] == campaign]
                rows = []
                if ids:
                    rows = db.execute(
                        """
                        WITH clocks_rows AS MATERIALIZED (
                            SELECT id, ts, viable, payload -> 'clocks' AS clocks
                            FROM campaign_detail
                            WHERE id = ANY(%s)
                              AND payload ->> 'schema' = 'CLOCKS_V4'
                              AND payload -> 'holistic_restore_superseded'
                                  IS DISTINCT FROM 'true'::jsonb
                        ), fields AS MATERIALIZED (
                            SELECT id, ts, viable, clocks -> 'decay' AS decay,
                                   clocks #>> '{clockfaces,gnss_ns}' AS gnss_ns,
                                   clocks #>> '{stats,reset_count}' AS reset_count,
                                   clocks -> 'dac_observation' AS observation
                            FROM clocks_rows
                        )
                        SELECT id, ts, viable, decay ->> 'session_id' AS session,
                               decay ->> 'time_ns' AS time_ns,
                               gnss_ns, reset_count, observation
                        FROM fields ORDER BY id
                        """, (ids,),
                    ).fetchall()
        except QueryCanceled as exc:
            # These are read-only transactions. open_db has rolled back before
            # retrying this same watermark, so no sample can be counted twice.
            reader['timeouts'] += 1
            if batch_size == 1:
                raise RuntimeError(
                    f'Read timed out at detail watermark {after} even with one ID per page. '
                    'No settings changed; database load/storage needs inspection.'
                ) from exc
            batch_size = max(1, batch_size // 2)
            reader['final_batch_size'] = batch_size
            print(f'Read timed out after detail {after}; retrying with {batch_size} IDs per page.',
                  file=sys.stderr, flush=True)
            continue
        if not page:
            break
        after = page[-1]['id']
        reader['pages'] += 1
        reader['indexed_rows'] += len(page)
        reader['payload_rows'] += len(ids)
        if monotonic() - progress_at >= 5:
            print(f"Scanned through detail {after}/{tail}; "
                  f"{reader['indexed_rows']:,} indexed rows, {reader['payload_rows']:,} campaign payloads.",
                  file=sys.stderr, flush=True)
            progress_at = monotonic()
        yield from rows


@dataclass
class Minute:
    key: int
    first: int
    last: int
    first_id: int
    last_id: int
    stamp: str
    n: int = 0
    x: int = 0
    xx: int = 0
    y: list = field(default_factory=lambda: [0, 0])
    xy: list = field(default_factory=lambda: [0, 0])

    def add(self, t, codes, detail_id):
        self.last = t
        self.last_id = detail_id
        self.n += 1
        self.x += t
        self.xx += t*t
        for i, code in enumerate(codes):
            self.y[i] += code
            self.xy[i] += t*code

    def values(self):
        return (self.n, self.x, self.xx, self.y[0], self.xy[0], self.y[1], self.xy[1])


def collect(rows, metadata):
    segments, blocks = [], []
    counts = Counter()
    previous = None
    progress_at = monotonic()
    for row in rows:
        counts['rows_read'] += 1
        if monotonic() - progress_at >= 5:
            print(f"Read {counts['rows_read']:,} rows; accepted {counts['accepted']:,}; "
                  f"detail {row['id']}", file=sys.stderr, flush=True)
            progress_at = monotonic()
        observation = row['observation']
        if not row['session'] or row['time_ns'] is None or row['gnss_ns'] is None:
            counts['before_profile_metadata'] += 1
            if blocks:
                segments.append(blocks)
                blocks = []
            previous = None
            continue
        if observation and observation['servo_mode'] != '10-MIN':
            counts['outside_10_min'] += 1
            if blocks:
                segments.append(blocks)
                blocks = []
            previous = None
            continue
        if not row['viable'] or observation is None:
            counts['excluded_or_unsampled'] += 1
            continue
        if observation['schema'] != 'DAC_OBSERVATION_V1':
            raise ValueError(f"Unexpected DAC observation schema at detail {row['id']}")
        t, gnss = int(row['time_ns']), int(row['gnss_ns'])
        codes = tuple(round(Fraction(str(observation[name])) * CODE_SCALE) for name in LANES)
        identity = (row['session'], row['reset_count'], observation['dither_enabled'])
        if previous is not None:
            old_t, old_gnss, old_identity, old_codes = previous
            if identity == old_identity and t == old_t and gnss == old_gnss:
                if codes != old_codes:
                    raise ValueError(f"Conflicting duplicate DAC timestamp at detail {row['id']}")
                counts['duplicates'] += 1
                continue
            if (identity != old_identity or t <= old_t or gnss <= old_gnss
                    or t-old_t > 120*NS or gnss-old_gnss != t-old_t):
                counts['boundaries'] += 1
                if blocks:
                    segments.append(blocks)
                    blocks = []
        if not blocks or blocks[-1].key != t // MINUTE:
            blocks.append(Minute(t // MINUTE, t, t, row['id'], row['id'], str(row['ts'])))
        blocks[-1].add(t, codes, row['id'])
        previous = (t, gnss, identity, codes)
        counts['accepted'] += 1
    if blocks:
        segments.append(blocks)
    metadata['counts'] = dict(counts)
    metadata['segments'] = [dict(number=i+1, first_id=b[0].first_id, last_id=b[-1].last_id,
                                 first_timestamp=b[0].stamp, last_minute_timestamp=b[-1].stamp,
                                 span_hours=(b[-1].last-b[0].first)/HOUR,
                                 observations=sum(m.n for m in b)) for i, b in enumerate(segments)]
    return segments


class Population:
    def __init__(self, blocks):
        self.blocks = blocks
        self.keys = [b.key*MINUTE for b in blocks]
        self.prefix = [(0,)*7]
        for b in blocks:
            self.prefix.append(tuple(a+c for a, c in zip(self.prefix[-1], b.values())))

    def span(self, start, end, coverage):
        lo, hi = bisect_left(self.keys, start), bisect_left(self.keys, end)
        if lo == hi:
            return None
        a, b = self.blocks[lo], self.blocks[hi-1]
        totals = tuple(y-x for x, y in zip(self.prefix[lo], self.prefix[hi]))
        if (a.first > start+120*NS or b.last < end-120*NS
                or totals[0]*NS < coverage*(end-start)):
            return None
        return lo, hi, totals


def slope(totals, lane_index):
    n, x, xx = totals[:3]
    y, xy = totals[3+2*lane_index:5+2*lane_index]
    return Fraction((n*xy-x*y)*NS, (n*xx-x*x)*CODE_SCALE)


def mean(totals, lane_index):
    return Fraction(totals[1], totals[0]), Fraction(totals[3+2*lane_index], totals[0]*CODE_SCALE)


def score(blocks, future, reference, endpoint, rate, lane_index):
    lane = LANES[lane_index]
    k = PPB_PER_CODE[lane]
    ref_t, ref_y = mean(reference, lane_index)
    raw_errors, corrected_errors = [], []
    for b in blocks[future[0]:future[1]]:
        dt = (Fraction(b.x, b.n)-ref_t)/NS
        dy = Fraction(b.y[lane_index], b.n*CODE_SCALE)-ref_y
        raw_errors.append(float(-k*dy))
        corrected_errors.append(float(-k*(dy-rate*dt)))
    end_t, end_y = mean(endpoint, lane_index)
    all_t, all_y = mean(future[2], lane_index)
    return dict(
        raw_rms_ppb=math.sqrt(sum(e*e for e in raw_errors)/len(raw_errors)),
        corrected_rms_ppb=math.sqrt(sum(e*e for e in corrected_errors)/len(corrected_errors)),
        raw_end_ppb=float(-k*(end_y-ref_y)),
        corrected_end_ppb=float(-k*(end_y-ref_y-rate*(end_t-ref_t)/NS)),
        raw_campaign_mean_ppb=float(-k*(all_y-ref_y)),
        corrected_campaign_mean_ppb=float(-k*(all_y-ref_y-rate*(all_t-ref_t)/NS)),
    )


def backtest(segments, windows, horizons, stride, coverage):
    results, rejected = [], Counter()
    for segment_no, blocks in enumerate(segments, 1):
        population = Population(blocks)
        # All training lengths share origins and forecast observations. Starting
        # at the longest window avoids ranking different windows on easier days.
        start = ((blocks[0].first+HOUR-1)//HOUR + max(windows))*HOUR
        stop = blocks[-1].last-min(horizons)*HOUR
        for cut in range(start, stop+1, stride*HOUR):
            training = {w: population.span(cut-w*HOUR, cut, coverage) for w in windows}
            reference = population.span(cut-HOUR, cut, coverage)
            if reference is None or any(v is None for v in training.values()):
                rejected['training_coverage'] += 1
                continue
            for horizon in horizons:
                end = cut+horizon*HOUR
                if end > blocks[-1].last+NS:
                    continue
                future = population.span(cut, end, coverage)
                endpoint = population.span(end-HOUR, end, coverage)
                if future is None or endpoint is None:
                    rejected['forecast_coverage'] += 1
                    continue
                for window, train in training.items():
                    for i, lane in enumerate(LANES):
                        rate = slope(train[2], i)
                        results.append(dict(
                            segment=segment_no, lane=lane, window_hours=window,
                            horizon_hours=horizon, cut_ns=cut,
                            first_future_id=blocks[future[0]].first_id,
                            first_future_timestamp=blocks[future[0]].stamp,
                            last_future_id=blocks[future[1]-1].last_id,
                            train_n=train[2][0], future_n=future[2][0],
                            slope_codes_per_second=dict(numerator=rate.numerator, denominator=rate.denominator),
                            slope_nv_per_second=float(rate*VOLTS_PER_CODE*NS),
                            slope_ppb_per_day=float(-rate*PPB_PER_CODE[lane]*86400),
                            **score(blocks, future, reference[2], endpoint[2], rate, i)))
    return results, dict(rejected)


def summarize(results):
    groups = defaultdict(list)
    for result in results:
        groups[result['lane'], result['horizon_hours'], result['window_hours']].append(result)
    summary = []
    for (lane, horizon, window), group in sorted(groups.items()):
        raw = median(r['raw_rms_ppb'] for r in group)
        corrected = median(r['corrected_rms_ppb'] for r in group)
        summary.append(dict(lane=lane, horizon_hours=horizon, window_hours=window,
                            folds=len(group), raw_rms_ppb=raw, corrected_rms_ppb=corrected,
                            improvement_percent=100*(1-corrected/raw) if raw > 1e-12 else None,
                            wins=sum(r['corrected_rms_ppb'] < r['raw_rms_ppb']-1e-12 for r in group),
                            raw_end_abs_ppb=median(abs(r['raw_end_ppb']) for r in group),
                            corrected_end_abs_ppb=median(abs(r['corrected_end_ppb']) for r in group),
                            worst_raw_end_abs_ppb=max(abs(r['raw_end_ppb']) for r in group),
                            worst_corrected_end_abs_ppb=max(abs(r['corrected_end_ppb']) for r in group)))
    return summary


def show(report, details):
    print(f"TEMPEST decay backtest  {report['campaign']}  through detail {report['through_detail_id']}")
    if report.get('reader'):
        print('Reader: '+', '.join(f'{k}={v:,}' for k, v in report['reader'].items()))
    print('Counts: '+', '.join(f'{k}={v:,}' for k, v in report['counts'].items()))
    for segment in report['segments']:
        print(f"Segment {segment['number']}: {segment['span_hours']:.3f} h, "
              f"{segment['observations']:,} samples, details {segment['first_id']}..{segment['last_id']}")
    print(f"Origin stride {report['stride_hours']} h; minimum sample coverage {100*report['coverage']:.1f}%")
    print('Fit per-second targets; score later minute means. Reference = last training-hour mean.')
    print('Each horizon compares identical origins for all fitting windows; no future fitting or recentering.')
    print('RMS and END are ppb (numerically ns over one second), not accumulated clock phase.')
    print('END uses the final forecast-hour mean. Summary RMS and absolute END are medians across origins.')
    print('\nLANE   TRAIN TEST FOLDS   RAW_RMS  CORR_RMS   GAIN%   WINS   RAW_END CORR_END WORST_RAW WORST_CORR')
    for r in report['summary']:
        gain = '---' if r['improvement_percent'] is None else f"{r['improvement_percent']:.1f}"
        print(f"{r['lane'].upper():<6} {r['window_hours']:>4}h {r['horizon_hours']:>3}h {r['folds']:>5} "
              f"{r['raw_rms_ppb']:>9.6f} {r['corrected_rms_ppb']:>9.6f} {gain:>7} "
              f"{str(r['wins'])+'/'+str(r['folds']):>6} {r['raw_end_abs_ppb']:>9.6f} "
              f"{r['corrected_end_abs_ppb']:>8.6f} {r['worst_raw_end_abs_ppb']:>9.6f} "
              f"{r['worst_corrected_end_abs_ppb']:>10.6f}")
    if not report['summary']:
        print('No complete comparisons. A continuous segment needs the longest training window plus a forecast horizon.')
    if report['rejected_origins']:
        print('Coverage rejections: '+str(report['rejected_origins']))
    if details:
        print('\nPer-origin results: signed END and campaign-mean residuals; RMS is unsigned.')
        print('LANE TRAIN TEST START(detail/time) SLOPE(nV/s) RAW_RMS CORR_RMS RAW_END CORR_END RAW_MEAN CORR_MEAN')
        for r in report['folds']:
            print(f"{r['lane'].upper()} {r['window_hours']}h {r['horizon_hours']}h "
                  f"{r['first_future_id']}/{r['first_future_timestamp']} "
                  f"{r['slope_nv_per_second']:+.6f} {r['raw_rms_ppb']:.6f} "
                  f"{r['corrected_rms_ppb']:.6f} {r['raw_end_ppb']:+.6f} "
                  f"{r['corrected_end_ppb']:+.6f} {r['raw_campaign_mean_ppb']:+.6f} "
                  f"{r['corrected_campaign_mean_ppb']:+.6f}")
    print('\nPositive GAIN means lower median prediction RMS; WINS counts origins improved.')
    print('Overlapping forecasts are correlated; FOLDS is not an independent sample size.')
    print('Short-history windows are not padded. No fit crosses session/reset/control changes or gaps over 120 s.')
    print('Rows before decay-session metadata are counted and excluded; old calibration steps are not silently fitted.')
    print('These comparisons use nominal DAC target voltages and fixed approximate tuning coefficients.')
    print('The analysis does not identify intrinsic aging separately from environmental or reference drift.')
    print('A promising historical window still needs a subsequent frozen stationary trial. Nothing has been changed.')


def hours(value):
    result = int(value)
    if result < 1 or result > 168:
        raise argparse.ArgumentTypeError('hours must be an integer from 1 to 168')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign')
    parser.add_argument('--windows', nargs='+', type=hours, default=[6,12,24,48], metavar='HOURS')
    parser.add_argument('--horizons', nargs='+', type=hours, default=[12,24,48], metavar='HOURS')
    parser.add_argument('--stride-hours', type=hours, default=12)
    parser.add_argument('--min-coverage', type=float, default=.95, help='minimum admitted samples / GNSS seconds (default .95)')
    parser.add_argument('--batch-size', type=int, default=128, help='maximum primary-key IDs per read page; halves on timeout (default 128)')
    parser.add_argument('--details', action='store_true', help='also print every historical forecast')
    parser.add_argument('--json', action='store_true', help='emit summary and all forecasts as JSON')
    args = parser.parse_args()
    if not 0 < args.min_coverage <= 1:
        parser.error('--min-coverage must be greater than zero and at most one')
    if not 1 <= args.batch_size <= 5000:
        parser.error('--batch-size must be between 1 and 5000')
    windows, horizons = sorted(set(args.windows)), sorted(set(args.horizons))
    report = dict(schema='CLOCKS_DECAY_BACKTEST_V1', campaign=args.campaign,
                  windows_hours=windows, horizons_hours=horizons, stride_hours=args.stride_hours,
                  coverage=args.min_coverage)
    print(f'Reading recorded DAC targets for {args.campaign}; CLOCKS controls remain unchanged.', file=sys.stderr, flush=True)
    try:
        segments = collect(history_rows(args.campaign, args.batch_size, report), report)
        print(f'Evaluating {len(segments)} continuous segments...', file=sys.stderr, flush=True)
        results, rejected = backtest(segments, windows, horizons, args.stride_hours, Fraction(str(args.min_coverage)))
    except (LookupError, ValueError, RuntimeError) as exc:
        parser.exit(1, f'{exc}\n')
    except KeyboardInterrupt:
        parser.exit(130, 'Report interrupted; no instrument settings were changed.\n')
    report.update(folds=results, rejected_origins=rejected, summary=summarize(results))
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        show(report, args.details)


if __name__ == '__main__':
    main()
