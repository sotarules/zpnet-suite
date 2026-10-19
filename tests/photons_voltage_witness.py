"""Read-only database witness for Teensy PHOTONS voltage LAP normalization.

    zt photons_voltage_witness
    zt photons_voltage_witness --rows 300 --show 30
    zt photons_voltage_witness --campaign HeatCool2 --rows 600
    zt photons_voltage_witness --rows 120 --jsonl

Reads latest persisted LANTERN/PHOTONS_V1 campaign_detail rows, including
campaign-less observations. Does not listen to PUBSUB, invoke firmware RPC,
change database records, or change the model. The displayed HIGH predictor is
this fragment's ADC population, NOT the lagged HIGH60 metric.

Reconstructs the linear-model correction from the persisted firmware identity,
reference and slope; corroborates raw, correction and published Welford
populations against both the Teensy and Pi witnesses. Reports descriptive
between-fragment variation and within-fragment ADC sample SE as separate
quantities. ADC samples and accepted races are not assumed to be paired.

main(argv) includes the program name, as required by the zt runner.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass
from datetime import datetime, timezone

VERSION = '2026-10-19.1'
SCHEMA = 'PHOTONS_V1'
FIRMWARE_SCHEMA = 'PHOTONS_VOLTAGE_NORMALIZATION_V1'
MODEL = 'PD200T_HIGH_LINEAR_V1'
APPLICATION = 'UNIFORM_OFFSET_AT_FRAGMENT_DRAIN'
ACQUISITION_ID = 0x4E4F5231  # NOR1
ADC_SCALE_MV = 3300.0 / 4095.0
# Overnight HeatCool1 development fit domain; observation outside it is an
# extrapolation, not a violation of the firmware normalization contract.
V1_FIT_LOW_MV = 428.284913
V1_FIT_HIGH_MV = 494.716587
# The Teensy carries sixteen significant decimal digits; these tolerances
# cover representation rounding, not unmodeled physical measurement noise.
CLOSURE_NS = 1e-6
CLOSURE_MV = 1e-6


@dataclass(frozen=True)
class Observation:
    id: int
    sequence: int
    reset: int
    campaign: str | None
    model: str
    voltage_mv: float
    slope_ns_per_mv: float
    raw_ns: float
    adjustment_ns: float
    published_ns: float
    adc_n: int
    races: int
    adc_sd_mv: float
    adc_se_mv: float
    seconds: float
    stamp: str


def arguments(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--rows', type=int, default=120,
                   help='latest database observations to inspect, including non-normalized rows (default 120)')
    p.add_argument('--show', type=int, default=25,
                   help='number of most recent normalized rows to print (default 25)')
    p.add_argument('--campaign', type=str,
                   help='select exact persisted campaign name; default includes campaign-less observations')
    p.add_argument('--jsonl', action='store_true',
                   help='print structured row and summary results as JSON lines')
    args = p.parse_args(argv[1:])
    if args.rows < 1 or args.show < 0:
        p.error('--rows must be positive and --show must be nonnegative')
    return args


def database_rows(args):
    from zpnet.shared.db import open_db

    # Select only the measurement witnesses, not the large recovery/checkpoint
    # state. Search backward through the indexed detail identity, then restore
    # chronological order in Python. No second-scale feed is needed.
    sql = """
        SELECT id, campaign,
               jsonb_build_object(
                   'schema', payload -> 'schema',
                   'sequence', payload -> 'sequence',
                   'published_at_utc', payload -> 'published_at_utc',
                   'normalization', payload -> 'normalization',
                   'measurement', payload -> 'measurement',
                   'photons', jsonb_build_object(
                       'fragment_elapsed_ms', payload #> '{photons,fragment_elapsed_ms}',
                       'pd_out', payload #> '{photons,pd_out}',
                       'voltage_normalization', payload #> '{photons,voltage_normalization}',
                       'race', jsonb_build_object(
                           'flight_ns', payload #> '{photons,race,flight_ns}'
                       ),
                       'stats', jsonb_build_object(
                           'acquisition_id', payload #> '{photons,stats,acquisition_id}',
                           'reset_count', payload #> '{photons,stats,reset_count}'
                       )
                   )
               ) AS witness
        FROM campaign_detail
        WHERE campaign_type = %s
          AND payload ->> 'schema' = %s
    """
    params = ['LANTERN', SCHEMA]
    if args.campaign is not None:
        sql += ' AND campaign = %s'
        params.append(args.campaign)
    sql += ' ORDER BY id DESC LIMIT %s'
    params.append(args.rows)
    with open_db(row_dict=True) as conn:
        conn.execute('SET TRANSACTION READ ONLY')
        cur = conn.cursor()
        cur.execute(sql, params)
        found = cur.fetchall()
    for row in reversed(found):
        value = row['witness']
        yield int(row['id']), row['campaign'], json.loads(value) if isinstance(value, str) else value


def finite(value, label):
    f = float(value)
    if not math.isfinite(f):
        raise ValueError(f'{label} must be finite: {value!r}')
    return f


def expect_close(observed, expected, label, tolerance=CLOSURE_NS):
    if abs(finite(observed, label) - finite(expected, label + ' expected')) > tolerance:
        raise ValueError(f'{label}: observed={observed} expected={expected} tolerance={tolerance:g}')


def check_welford(raw, published, delta):
    n = int(raw['n'])
    if n <= 0 or int(published['n']) != n:
        raise ValueError('raw/published accepted-race N does not close')
    for key in ('mean', 'min', 'max'):
        expect_close(published[key], finite(raw[key], 'raw.' + key) + delta,
                     'Welford ' + key)
    expect_close(published['m2'], raw['m2'], 'Welford M2')
    # A uniform offset preserves the within-fragment SD/SE.
    for key in ('stddev', 'stderr'):
        expect_close(published[key], raw[key], 'Welford ' + key)
    return n


def parse_row(row_id, campaign, payload):
    photons = payload['photons']
    fw = photons['voltage_normalization']
    if fw is None:
        return None
    if fw['schema'] != FIRMWARE_SCHEMA or fw['model_id'] != MODEL:
        raise ValueError('unexpected Teensy normalization model identity')
    if fw['application'] != APPLICATION:
        raise ValueError('unexpected Teensy normalization application policy')
    stats = photons['stats']
    measurement = payload['measurement']
    policy = payload['normalization']
    if int(stats['acquisition_id']) != ACQUISITION_ID or int(measurement['acquisition_id']) != ACQUISITION_ID:
        raise ValueError('Teensy/Pi acquisition epoch mismatch')
    if measurement['authority'] != 'TEENSY.PHOTONS':
        raise ValueError('Pi did not preserve Teensy normalization authority')
    if measurement['status'] not in ('FIRMWARE_CORRECTED', 'NO_ACCEPTED_RACES'):
        raise ValueError('unexpected Pi normalized-measurement status')
    if policy['schema'] != FIRMWARE_SCHEMA or policy['normalization_id'] != fw['model_id']:
        raise ValueError('Pi normalization policy identity mismatch')
    for key in ('application', 'reference_mv', 'slope_ns_per_mv'):
        if str(policy[key]) != str(fw[key]):
            raise ValueError('Pi/Teensy normalization policy disagrees: ' + key)

    pd = photons['pd_out']
    adc_n = int(pd['sample_count'])
    if adc_n != int(fw['adc_sample_count']):
        raise ValueError('voltage witness ADC N differs from PD OUT sample N')
    ref = finite(fw['reference_mv'], 'reference_mv')
    slope = finite(fw['slope_ns_per_mv'], 'slope_ns_per_mv')
    voltage = finite(fw['high_mv'], 'high_mv')
    adjustment = finite(fw['adjustment_ns'], 'adjustment_ns')
    if adc_n:
        total = int(pd['high_sum_counts'])
        squares = int(pd['high_sum_squares_counts'])
        if total < 0 or total > 4095 * adc_n or squares < 0 or adc_n * squares < total * total:
            raise ValueError('ADC integer moments are inconsistent')
        expect_close(voltage, total * ADC_SCALE_MV / adc_n,
                     'ADC HIGH mean voltage', CLOSURE_MV)
        adc_sd = math.sqrt((adc_n * squares - total * total) / (adc_n * (adc_n - 1))) * ADC_SCALE_MV if adc_n > 1 else 0.0
        adc_se = adc_sd / math.sqrt(adc_n) if adc_n > 1 else 0.0
        expect_close(adjustment, slope * (voltage - ref), 'linear formula')
    else:
        if voltage != 0 or adjustment != 0:
            raise ValueError('zero ADC sample population published nonzero correction')
        adc_sd = adc_se = 0.0

    raw = fw['raw_flight_ns']
    published = fw['published_flight_ns']
    race = photons['race']['flight_ns']
    if published != race:
        raise ValueError('Teensy published flight differs from canonical race flight')
    if raw != measurement['raw_flight_ns']:
        raise ValueError('Pi raw witness differs from Teensy raw flight')

    n = int(raw['n'])
    if n == 0:
        if int(published['n']) != 0 or int(race['n']) != 0:
            raise ValueError('empty accepted flight population mismatch')
        if measurement['status'] != 'NO_ACCEPTED_RACES':
            raise ValueError('zero-race row should not claim corrected science')
        return None
    if adc_n == 0:
        raise ValueError('accepted race population has no ADC voltage evidence')
    if measurement['status'] != 'FIRMWARE_CORRECTED':
        raise ValueError('nonempty accepted flight population is not marked corrected')
    check_welford(raw, published, adjustment)
    if published != measurement['normalized_flight_ns']:
        raise ValueError('Pi normalized witness differs from Teensy published flight')
    expect_close(measurement['correction_ns'], -adjustment, 'Pi inverse-correction sign')
    if measurement['normalization_id'] != fw['model_id']:
        raise ValueError('Pi normalized model ID mismatch')

    return Observation(
        id=row_id, sequence=int(payload['sequence']), reset=int(stats['reset_count']),
        campaign=campaign, model=str(fw['model_id']), voltage_mv=voltage,
        slope_ns_per_mv=slope,
        raw_ns=finite(raw['mean'], 'raw mean'), adjustment_ns=adjustment,
        published_ns=finite(published['mean'], 'published mean'), adc_n=adc_n,
        races=n, adc_sd_mv=adc_sd, adc_se_mv=adc_se,
        seconds=int(photons['fragment_elapsed_ms']) / 1000.0,
        stamp=str(payload['published_at_utc']),
    )


def sd(values):
    return statistics.stdev(values) if len(values) > 1 else 0.0


def pearson(x, y):
    if len(x) < 3:
        return None
    mx, my = statistics.fmean(x), statistics.fmean(y)
    xx = sum((v - mx) ** 2 for v in x)
    yy = sum((v - my) ** 2 for v in y)
    return sum((a - mx) * (b - my) for a, b in zip(x, y)) / math.sqrt(xx * yy) if xx > 0 and yy > 0 else None


def summary(rows):
    volts = [r.voltage_mv for r in rows]
    raw = [r.raw_ns for r in rows]
    published = [r.published_ns for r in rows]
    adj = [r.adjustment_ns for r in rows]
    raw_sd, pub_sd = sd(raw), sd(published)
    return {
        'first_id': rows[0].id, 'last_id': rows[-1].id,
        'first_sequence': rows[0].sequence, 'last_sequence': rows[-1].sequence,
        'fragments': len(rows), 'accepted_races': sum(r.races for r in rows),
        'adc_samples': sum(r.adc_n for r in rows),
        'high_range_mv': [min(volts), max(volts)],
        'outside_original_fit': sum(not V1_FIT_LOW_MV <= v <= V1_FIT_HIGH_MV for v in volts),
        'raw_range_ns': [min(raw), max(raw)],
        'adjustment_range_ns': [min(adj), max(adj)],
        'published_range_ns': [min(published), max(published)],
        'raw_between_fragment_sd_ps': raw_sd * 1000,
        'published_between_fragment_sd_ps': pub_sd * 1000,
        'between_fragment_sd_change_pct': ((1 - pub_sd / raw_sd) * 100 if raw_sd else None),
        'high_raw_correlation': pearson(volts, raw),
        'high_published_correlation': pearson(volts, published),
        'adc_one_second_se_mv_median': statistics.median(r.adc_se_mv for r in rows),
        'adjustment_one_second_se_ps_median': statistics.median(
            abs(r.slope_ns_per_mv) * r.adc_se_mv * 1000 for r in rows),
    }


def split_epochs(rows):
    groups = []
    for row in rows:
        if (not groups or row.sequence != groups[-1][-1].sequence + 1
                or row.reset != groups[-1][-1].reset
                or row.model != groups[-1][-1].model):
            groups.append([])
        groups[-1].append(row)
    return groups


def print_report(args, rows, older, failures):
    print(f'PHOTONS VOLTAGE NORMALIZATION WITNESS {VERSION}')
    print(f'Generated UTC: {datetime.now(timezone.utc).isoformat(timespec="seconds")}')
    print(f'Source: read-only campaign_detail; latest {args.rows} LANTERN rows; campaign={args.campaign or "ALL (including STOPPED)"}')
    print('Model: PD200T_HIGH_LINEAR_V1; predictor=per-fragment HIGH, NOT HIGH60; correction performed by Teensy')
    print(f'Original development-voltage domain: {V1_FIT_LOW_MV:.3f} .. {V1_FIT_HIGH_MV:.3f} mV (outside = extrapolation)')
    print(f'Rows: normalized={len(rows)}; older/unmodeled={older}; closure_errors={len(failures)}')
    for ident, why in failures[:20]:
        print(f'  FAIL id={ident}: {why}')
    if len(failures) > 20:
        print(f'  ... plus {len(failures) - 20} additional closure failures')
    if not rows:
        print('No usable normalized rows in the selected database tail.')
        return
    epochs = split_epochs(rows)
    print(f'Continuous segments: {len(epochs)} (split on model/reset/sequence discontinuity)')
    for i, group in enumerate(epochs, 1):
        info = summary(group)
        print(f'\nSEGMENT {i}: rows={info["fragments"]} ids={info["first_id"]}..{info["last_id"]} '
              f'sequence={info["first_sequence"]}..{info["last_sequence"]} '
              f'accepted={info["accepted_races"]:,} ADC={info["adc_samples"]:,}')
        print(f'  HIGH mV:          {info["high_range_mv"][0]:.6f} .. {info["high_range_mv"][1]:.6f}'
              f'  outside_original_fit={info["outside_original_fit"]}/{info["fragments"]}')
        print(f'  RAW LAP ns:       {info["raw_range_ns"][0]:.6f} .. {info["raw_range_ns"][1]:.6f}')
        print(f'  ADJUSTMENT ns:    {info["adjustment_range_ns"][0]:+.6f} .. {info["adjustment_range_ns"][1]:+.6f}')
        print(f'  PUBLISHED ns:     {info["published_range_ns"][0]:.6f} .. {info["published_range_ns"][1]:.6f}')
        sd_change = info['between_fragment_sd_change_pct']
        change_text = 'n/a' if sd_change is None else f'{sd_change:+.1f}%'
        raw_corr = info['high_raw_correlation']
        pub_corr = info['high_published_correlation']
        raw_corr_text = 'n/a' if raw_corr is None else f'{raw_corr:+.4f}'
        pub_corr_text = 'n/a' if pub_corr is None else f'{pub_corr:+.4f}'
        print(f'  Between-second SD: raw={info["raw_between_fragment_sd_ps"]:.2f} ps '
              f'published={info["published_between_fragment_sd_ps"]:.2f} ps '
              f'change={change_text}')
        print(f'  Corr(HIGH,raw/published): {raw_corr_text} / {pub_corr_text}')
        print(f'  Median ADC SE: {info["adc_one_second_se_mv_median"]:.4f} mV; '
              f'approx correction SE: {info["adjustment_one_second_se_ps_median"]:.1f} ps*')
    print('\nMOST RECENT PER-FRAGMENT WITNESSES')
    print(' ID       SEQ     HIGH_mV    RAW_ns    ADJUST_ns   PUB_ns     N_RACE   N_ADC   ADC_SE_mV  FIT')
    for r in rows[-args.show:] if args.show else []:
        print(f'{r.id:8d} {r.sequence:7d} {r.voltage_mv:11.4f} {r.raw_ns:10.4f} '
              f'{r.adjustment_ns:+11.4f} {r.published_ns:10.4f} {r.races:9d} '
              f'{r.adc_n:7d} {r.adc_se_mv:11.4f}  '
              f'{"IN" if V1_FIT_LOW_MV <= r.voltage_mv <= V1_FIT_HIGH_MV else "OUT"}')
    print('\nCLOSURE: raw + adjustment = published; N/M2/SD/SE preserved; Pi/Teensy/model identities agree.')
    print('* ADC SE assumes independent samples, which has not been established. It is NOT calibration accuracy.')
    print('  Between-fragment SD is descriptive and can increase even when the arithmetic is correct.')
    print('  Identical correction for every accepted race in a fragment cannot reduce within-fragment SD.')
    print('  No database records, producer state, or normalization coefficients were modified.')


def main(argv):
    args = arguments(argv)
    records = []
    failures = []
    older = 0
    try:
        for row_id, campaign, payload in database_rows(args):
            try:
                observed = parse_row(row_id, campaign, payload)
            except (ValueError, KeyError, TypeError, ZeroDivisionError) as error:
                failures.append((row_id, str(error)))
            else:
                if observed is None:
                    older += 1
                else:
                    records.append(observed)
    except Exception as error:
        print(f'PHOTONS VOLTAGE WITNESS DATABASE ERROR: {error}', file=sys.stderr)
        return 2

    if args.jsonl:
        for r in records:
            print(json.dumps({'kind': 'row', **r.__dict__}, separators=(',', ':')))
        for i, segment in enumerate(split_epochs(records), 1):
            print(json.dumps({'kind': 'segment_summary', 'segment': i, **summary(segment)}, separators=(',', ':')))
        print(json.dumps({'kind': 'summary', 'checked': len(records), 'older': older,
                          'errors': [{'id': rid, 'reason': reason} for rid, reason in failures]}, separators=(',', ':')))
    else:
        print_report(args, records, older, failures)
    return 2 if failures or not records else 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
