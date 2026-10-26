"""Export paired PHOTONS calibration observations as copyable, lossless text.

zt photons_extract HighLow2 --end 2026-10-25T14:40:00-07:00
Requires the paced zpnet.shared.report_db reader and sibling report programs.
No fitting, rounding, database writes, or firmware commands. main receives argv[0].
The temporary XZ spool bounds memory and permits a line-budget check before output.
"""
import argparse
import base64
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import lzma
from pathlib import Path
import sys
import tempfile

from zpnet.shared.report_db import add_read_arguments, validate_read_arguments

VERSION = '2026-10-26.1'
FIELDS = [
    'detail_id', 'published_at_utc', 'sequence', 'reset_count', 'update_count',
    'recovery_generation', 'anchor_pps_count', 'context_id', 'recovery_receipt_present',
    'raw_mean_ns', 'published_mean_ns', 'adjustment_ns', 'raw_m2_ns2', 'race_n',
    'fragment_elapsed_ms', 'adc_n', 'high_sum_counts', 'high_sum_squares_counts',
    'high60_mv', 'high60_n', 'high60_elapsed_ms', 'high60_target_ms',
    'high60_fragments', 'high60_sum_counts', 'high60_sum_squares_counts',
    'sensors',
]
SENSORS = ['synthetic', 'bme280', 'bme280_receiver', 'bme280_out', 'max31865']


def sibling(name):
    key = '_photons_extract_' + name
    spec = importlib.util.spec_from_file_location(key, Path(__file__).with_name(name + '.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[key] = module
    spec.loader.exec_module(module)
    return module


def timestamp(value):
    parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if parsed.utcoffset() is None:
        raise ValueError('timestamp must include Z or a UTC offset')
    return parsed.timestamp()


def arguments(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign', nargs='?')
    p.add_argument('--start', type=timestamp, help='inclusive publication time')
    p.add_argument('--end', type=timestamp, help='required exclusive publication time')
    p.add_argument('--replay', type=Path, help='print a retained XZ spool without querying PostgreSQL')
    p.add_argument('--line-width', type=int, default=160, help='terminal columns (default 160)')
    p.add_argument('--max-lines', type=int, default=80000, help='data-envelope line budget (default 80000)')
    add_read_arguments(p)
    a = p.parse_args(argv[1:])
    validate_read_arguments(p, a)
    if not 80 <= a.line_width <= 512 or not 100 <= a.max_lines <= 1000000:
        p.error('line width must be 80..512; max lines must be 100..1000000')
    if a.replay is None and (not a.campaign or a.end is None):
        p.error('campaign and --end are required for database extraction')
    if a.replay is not None and (a.campaign or a.start is not None or a.end is not None):
        p.error('--replay cannot be combined with campaign/time selection')
    if a.start is not None and a.start >= a.end:
        p.error('--start must precede --end')
    return a


def compact(value):
    return json.dumps(value, ensure_ascii=True, allow_nan=False, separators=(',', ':'))


def sensor_context(payload, thermal):
    readings, settings = [], []
    temperatures = payload.get('temperature') or {}
    for name in SENSORS:
        item, path = thermal.sensor_reading(payload, temperatures, name)
        if not item:
            readings.append(['NOT_RECORDED', None, None])
            settings.append({'path': path})
            continue
        status = item['status']
        value = thermal.number(item['temperature_c']) if status == 'OK' else None
        age = item.get('age_ms')
        if status == 'OK':
            thermal.integer(age)
            expected = 'PI.SYSTEM.TEMPERATURE' if name == 'synthetic' else thermal.SENSOR_COMPONENTS[name][1]
            if item['source'] != expected:
                raise ValueError('unexpected sensor source: ' + name)
        readings.append([status, value, age])
        settings.append({k: item.get(k) for k in ('schema', 'source', 'model_id')} |
                        {'path': path, 'window_samples': (item.get('smoothing') or {}).get('window_samples')})
    return readings, settings


def extract(args, rows, thermal, witness, output):
    """Write lossless JSONL to a binary compressor, retaining bounded row chunks.

    Publication-time selection is checked on every row, including late/reordered
    records. Do not stop at the first timestamp beyond --end.
    """
    counts, contexts, pending = Counter(), {}, []
    def write(value):
        output.write((compact(value) + '\n').encode('ascii'))
    def flush():
        if pending:
            # Column-major chunks compress repeated identity/sensor fields well.
            write({'type': 'columns', 'n': len(pending), 'data': list(map(list, zip(*pending)))})
            pending.clear()
    write({'type': 'header', 'schema': 'PHOTONS_CALIBRATION_EXTRACT_V1', 'version': VERSION,
           'campaign': args.campaign, 'created_utc': datetime.now(timezone.utc).isoformat(),
           'start_epoch_s_inclusive': args.start, 'end_epoch_s_exclusive': args.end,
           'fields': FIELDS, 'sensors_order': SENSORS,
           'sensor_tuple': ['recorded_status_or_NOT_RECORDED', 'temperature_c_or_null', 'age_ms_or_null'],
           'precision': 'JSON numbers preserved; no rounding or aggregation',
           'selection': 'publication time; separate read-only transactions; fixed reader ID ceiling',
           'adc_scale_mv': 3300 / 4095,
           'units': 'LAP/adjustment ns; raw M2 ns^2; voltage mV; temperature C; ages/durations ms'})
    for row_id, payload in rows:
        counts['scanned'] += 1
        try:
            time = timestamp(payload['published_at_utc'])
            if time >= args.end or (args.start is not None and time < args.start):
                counts['outside_time'] += 1
                continue
            counts['selected'] += 1
            sample, reason = thermal.parse_sample(row_id, payload, 'high')
            if sample is None:
                flush()
                write({'type': 'excluded', 'id': row_id, 'time': payload['published_at_utc'],
                       'sequence': payload['sequence'], 'reason': reason,
                       'recovery_receipt': payload.get('recovery_receipt')})
                counts['excluded_' + reason] += 1
                continue
            observed = witness.parse_row(row_id, args.campaign, payload)
            if observed is None or observed.voltage_source != 'PD_OUT_HIGH60_MV':
                raise ValueError('accepted row requires a validated persisted HIGH60 normalization witness')
            ph = payload['photons']
            window = ph['pd_out']['high_window']
            readings, sensor_settings = sensor_context(payload, thermal)
            settings = {'acquisition': json.loads(sample.signature), 'sensors': sensor_settings}
            key = compact(settings)
            if key not in contexts:
                contexts[key] = len(contexts) + 1
                write({'type': 'context', 'id': contexts[key], 'settings': settings})
            receipt = payload.get('recovery_receipt')
            if receipt is not None:
                write({'type': 'recovery', 'detail_id': row_id, 'receipt': receipt})
            pending.append([
                row_id, payload['published_at_utc'], sample.sequence, sample.reset, sample.update,
                sample.generation, sample.pps, contexts[key], receipt is not None,
                observed.raw_ns, observed.published_ns, observed.adjustment_ns,
                ph['voltage_normalization']['raw_flight_ns']['m2'], observed.races,
                sample.fragment_elapsed_ms, sample.adc_count, sample.adc_sum, sample.adc_squares,
                window['high_mean_mv'], window['sample_count'], window['elapsed_ms'], window['target_ms'],
                window['fragments'], window['high_sum_counts'], window['high_sum_squares_counts'], readings,
            ])
            counts['exported'] += 1
            counts['high60_complete'] += int(observed.window_complete)
            if len(pending) == 512:
                flush()
        except Exception as exc:
            raise ValueError(f'export row {row_id}: {exc}') from exc
    flush()
    if not counts['exported']:
        raise ValueError('no usable HIGH60 observations in selected interval')
    write({'type': 'summary', 'counts': dict(counts), 'contexts': len(contexts)})
    return counts


def emit(path, args):
    # 12-character prefix: ZPX1 + space + six-digit sequence + space.
    chars = ((args.line_width - 12) // 4) * 4
    chunk = chars // 4 * 3
    size = path.stat().st_size
    lines = (size + chunk - 1) // chunk
    if lines + 4 > args.max_lines:
        raise ValueError(f'export needs {lines + 4} lines at width {args.line_width}; budget={args.max_lines}. '
                         f'Complete spool retained at {path}. Increase terminal scrollback and replay with '
                         f'zt photons_extract --replay {path} --max-lines {lines + 1000}; no database reread needed.')
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(65536), b''):
            digest.update(block)
    print(f'ZPX1 BEGIN codec=xz+base64 bytes={size} lines={lines}')
    print('ZPX1 SHA256 ' + digest.hexdigest())
    with path.open('rb') as source:
        for i in range(1, lines + 1):
            print(f'ZPX1 {i:06d} ' + base64.b64encode(source.read(chunk)).decode('ascii'))
    print('ZPX1 END ' + digest.hexdigest(), flush=True)


def main(argv):
    args = arguments(argv)
    if args.replay is not None:
        emit(args.replay, args)
        return 0
    thermal, witness = sibling('photons_thermal'), sibling('photons_voltage_witness')
    with tempfile.NamedTemporaryFile(prefix='zpnet-calibration-', suffix='.xz', delete=False) as tmp:
        path = Path(tmp.name)
    print(f'[extract] building temporary spool {path}; data prints only after a complete read', file=sys.stderr, flush=True)
    try:
        with lzma.open(path, 'wb', preset=1) as output:
            counts = extract(args, thermal.database_rows(args), thermal, witness, output)
    except BaseException:
        path.unlink()
        raise
    print(f'[extract] complete: {dict(counts)}; printing export', file=sys.stderr, flush=True)
    # Retain the completed spool if line budget, cancellation or terminal fails.
    emit(path, args)
    path.unlink()
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
