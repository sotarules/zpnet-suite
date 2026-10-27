"""Report live PHOTONS publication cadence, processing delays and queue growth.

    zt photons_cadence_witness --seconds 60
    zt photons_cadence_witness --seconds 60 --jsonl
    python3 photons_cadence_witness.py --seconds 60
    zt photons_cadence_witness --adc-high
    zt photons_cadence_witness --adc-capture --index 0 --count 20
    zt photons_cadence_witness --db
    zt photons_cadence_witness --adc-high --fragments 120 --window-seconds 60 --jsonl

--adc-high reports HIGH ADC mean, sample SD and SE from the next completed
fragment (normally one second), or --fragments N consecutive fragments. This
mode only reads PHOTONS_FRAGMENT; it does not request cadence or health reports.
Also reports a rolling population over --window-seconds (default 60), combining
integer sums, squared sums and counts, never rounded per-second means. Whole
fragments cover at least the requested span once warmed up; actual span is shown.
Sequence gaps or acquisition changes restart this diagnostic window. Separately
shows the firmware's published 60-second window, which runs independently of this
report. Neither window changes lap normalization. Requires the published HIGH
window firmware patch. mV uses a nominal reference; SE assumes independent
samples and does not establish accuracy.

--adc-capture reads one page of the latest completed RAM2 single-sample capture through
the established Teensy RPC broker. Each ADC reading is displayed separately in mV;
there is no averaging, filtering or voting. Capture refreshes every PHOTONS fragment,
so separate requests can have different capture IDs. --jsonl retains the full payload.

Prints the complete report to STDOUT; no files or database values are changed.
Observes PHOTONS_FRAGMENT and PHOTONS through the read-only PUBSUB tap, requests
PHOTONS REPORT approximately every five seconds and reads process memory.
Timing at the tap includes observer/broker scheduling. Existing PHOTONS context
timestamps provide separate evidence of ingress, SYSTEM RPC and construction.
Instrumented PHOTONS REPORT adds bounded per-sequence monotonic stage timings;
these stay outside canonical measurements and recovery state. Only publish calls
inside this capture window are included; repeated REPORT samples are deduplicated.
--db captures host memory/swap, process RSS, disk space, PHOTONS backlog and
bounded read-only PostgreSQL diagnostics. It prints host evidence before trying
the database. No acquisition/service changes. PostgreSQL mode uses the installed
psycopg driver and shared connection configuration; other modes use only the
standard library, with --adc-capture using the shared RPC client. main(argv) includes the program name for the zt runner.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import statistics
import subprocess
import sys
import threading
import time

VERSION = '2026-10-18.1'
TAP = '/tmp/zpnet_pubsub_tap.sock'
COMMAND = '/tmp/zpnet-photons-command.sock'
TOPICS = ('PHOTONS_FRAGMENT', 'PHOTONS')
COUNTERS = (
    'fragments_received', 'fragments_processed', 'fragments_rejected',
    'photons_published', 'rows_persisted', 'ingress_queue_depth',
    'ingress_queue_depth_max', 'persist_queue_depth', 'persist_queue_depth_max',
    'system_report_retry_count', 'persistence_retry_count',
    'last_system_report_failure', 'last_persistence_failure',
)


def stamp():
    return datetime.now(timezone.utc).isoformat()


def epoch(value):
    return datetime.fromisoformat(value.replace('Z', '+00:00')).timestamp()


def report():
    request = {'machine':'PI', 'subsystem':'PHOTONS', 'command':'REPORT',
               '_client':{'process':'photons_cadence_witness', 'thread':'report'}}
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(2.0)
        sock.connect(COMMAND)
        sock.sendall(json.dumps(request).encode())
        sock.shutdown(socket.SHUT_WR)
        chunks = []
        while chunk := sock.recv(65536):
            chunks.append(chunk)
    return json.loads(b''.join(chunks))


def process_memory():
    result = subprocess.run(
        ['systemctl', 'show', 'zpnet-photons.service', '-p', 'MainPID', '--value'],
        capture_output=True, text=True, check=True, timeout=2)
    pid = int(result.stdout.strip())
    if not pid:
        return {'pid':0}
    values = {}
    for line in Path(f'/proc/{pid}/status').read_text().splitlines():
        key, _, value = line.partition(':')
        if key in ('VmRSS', 'VmSwap', 'VmSize', 'Threads'):
            values[key] = value.strip()
    return {'pid':pid, **values}


class Witness:
    def __init__(self, emit):
        self.emit = emit
        self.previous = {}
        self.fragments = {}
        self.fragment_order = deque()
        self.series = {}
        self.counts = {topic:0 for topic in TOPICS}
        # REPORT sampling is the sole writer of these collections. Tap sampling
        # continues to own series/counts above; summarize after sampler shutdown.
        self.started_ns = time.monotonic_ns()
        self.ended_ns = None
        self.processing_last = {}
        self.processing_previous = {}
        self.processing_series = {}
        self.processing_count = 0
        self.gc_samples = 0
        self.gc_collections = [0, 0, 0]
        self.gc_collected = [0, 0, 0]

    def measure(self, name, value):
        self.series.setdefault(name, []).append(value)
        return round(value, 3)

    def accept(self, message, mono, wall):
        if message.get('type') != 'publish':
            self.emit({'kind':'tap_control', 'message':message})
            return
        topic = message['topic']
        if topic not in TOPICS:
            return
        payload = message['payload']
        sequence = payload['sequence']
        row = {'kind':'publication', 'topic':topic, 'sequence':sequence,
               'observed_utc':datetime.fromtimestamp(wall, timezone.utc).isoformat()}
        previous = self.previous.get(topic)
        if previous is not None:
            row['interval_ms'] = self.measure(topic+'.interval_ms', (mono-previous[0])*1000)
            row['sequence_step'] = sequence-previous[1]
        self.previous[topic] = (mono, sequence)
        self.counts[topic] += 1
        if topic == 'PHOTONS_FRAGMENT':
            self.fragments[sequence] = mono
            self.fragment_order.append(sequence)
            if len(self.fragment_order) > 512:
                self.fragments.pop(self.fragment_order.popleft(), None)
        else:
            fragment_time = self.fragments.pop(sequence, None)
            if fragment_time is not None:
                row['fragment_to_canonical_tap_ms'] = self.measure(
                    'fragment_to_canonical_tap_ms', (mono-fragment_time)*1000)
            timing = payload['context_timing']
            row['context_timing'] = timing
            row['published_at_utc'] = payload['published_at_utc']
            rpc = timing['round_trip_ms']
            row['system_rpc_ms'] = self.measure('system_rpc_ms', rpc)
            row['ingress_to_rpc_start_est_ms'] = self.measure(
                'ingress_to_rpc_start_est_ms', timing['context_after_ingress_ms']-rpc/2)
            # published_at_utc is assigned inside _make_photons, AFTER the
            # rollback savepoint, but BEFORE ring/stats work and publish().
            created = epoch(payload['published_at_utc'])
            row['rpc_end_to_created_est_ms'] = self.measure(
                'rpc_end_to_created_est_ms',
                (created-epoch(timing['context_at_utc']))*1000-rpc/2)
            row['created_to_tap_observed_ms'] = self.measure(
                'created_to_tap_observed_ms', (wall-created)*1000)
        self.emit(row)

    def accept_processing(self, timing):
        run = timing['started_monotonic_ns']
        last = self.processing_last.get(run)
        samples = timing['samples']
        if last is not None and samples and samples[0]['sample_id'] > last + 1:
            self.emit({'kind':'processing_history_gap', 'started_monotonic_ns':run,
                       'missing_samples':samples[0]['sample_id']-last-1})
        for sample in samples:
            number = sample['sample_id']
            if last is not None and number <= last:
                continue
            self.processing_last[run] = number
            started = sample['publish_started_monotonic_ns']
            if started < self.started_ns or (self.ended_ns is not None and started > self.ended_ns):
                continue
            row = {'kind':'processing', 'started_monotonic_ns':run, **sample}
            previous = self.processing_previous.get(run)
            if previous is not None:
                row['publish_call_interval_ms'] = (started-previous[0])/1_000_000
                row['sequence_step'] = sample['sequence']-previous[1]
                self.processing_series.setdefault('publish_call_interval_ms', []).append(
                    row['publish_call_interval_ms'])
            self.processing_previous[run] = (started, sample['sequence'])
            for name in ('ingress_to_publish_ms', 'state_total_ms'):
                self.processing_series.setdefault(name, []).append(sample[name])
            for name, value in sample['stages_ms'].items():
                self.processing_series.setdefault('stage.'+name, []).append(value)
            if 'stage_cpu_ms' in sample:
                for name, value in sample['stage_cpu_ms'].items():
                    self.processing_series.setdefault('cpu.'+name, []).append(value)
            if 'stage_gc' in sample:
                self.gc_samples += 1
                for name, value in sample['stage_gc'].items():
                    self.processing_series.setdefault('gc.'+name, []).append(sum(value['elapsed_ms']))
                    for generation in range(3):
                        self.gc_collections[generation] += value['collections'][generation]
                        self.gc_collected[generation] += value['collected'][generation]
            self.processing_count += 1
            self.emit(row)

    def summarize(self):
        result = {'kind':'summary', 'counts':self.counts, 'timings_ms':{}}
        for name, values in self.series.items():
            ordered = sorted(values)
            result['timings_ms'][name] = {
                'n':len(values), 'min':round(ordered[0], 3),
                'median':round(statistics.median(values), 3),
                'max':round(ordered[-1], 3)}
        result['processing_samples'] = self.processing_count
        result['gc_samples'] = self.gc_samples
        result['gc_collections'] = self.gc_collections
        result['gc_collected'] = self.gc_collected
        result['processing_timings_ms'] = {}
        for name, values in self.processing_series.items():
            result['processing_timings_ms'][name] = {
                'n':len(values), 'min':round(min(values), 3),
                'median':round(statistics.median(values), 3),
                'max':round(max(values), 3)}
        self.emit(result)


def print_record(row):
    """Human-readable report output; --jsonl preserves the complete records."""
    kind = row['kind']
    if kind == 'start':
        print(f'PHOTONS CADENCE WITNESS REPORT {VERSION}')
        print(f'Generated UTC: {row["at_utc"]}; duration: {row["duration_s"]:g} seconds')
        print('Source: live PUBSUB tap, read-only PHOTONS REPORT and process memory.')
        print(row['notes'])
        print('All durations below are milliseconds; STEP is the sequence increment.')
        print('PAIR = matching fragment to canonical at tap; WAIT = ingress to RPC start estimate.')
        print('SAVE = RPC end to canonical construction estimate; AFTER = construction to tap.')
        print('published_at_utc marks construction, not the actual publish call.')
        print('PROCESSING rows are direct monotonic timings from REPORT, keyed by sequence.')
        print('Stages: elapsed/CPU/GC milliseconds. CPU is state-thread execution time.')
        print('GC is completed cyclic collection on that thread; it overlaps elapsed/CPU, not additive.')
        print('Collections on other threads and reference-count disposal are not measured as GC.')
        print('Elapsed minus CPU includes waits/scheduling; it does not identify their cause.')
        print('Publish call timing ends at local handoff, not broker delivery or database commit.')
        print('State total runs from state-lock request through persistence enqueue; stages partition it.')
        print('savepoint_complete/canonical_complete include return and replaced-object cleanup.')
        print('\nUTC             TOPIC        SEQUENCE STEP INTERVAL      PAIR      WAIT       RPC      SAVE     AFTER')
    elif kind == 'publication':
        topic = 'FRAGMENT' if row['topic'] == 'PHOTONS_FRAGMENT' else 'PHOTONS'
        fields = ('interval_ms', 'fragment_to_canonical_tap_ms',
                  'ingress_to_rpc_start_est_ms', 'system_rpc_ms',
                  'rpc_end_to_created_est_ms', 'created_to_tap_observed_ms')
        values = ' '.join(f'{row[key]:9.3f}' if key in row else f'{"-":>9}' for key in fields)
        print(f'{row["observed_utc"][11:26]:15} {topic:9} {row["sequence"]:10} '
              f'{row.get("sequence_step", "-"):>4} {values}')
    elif kind == 'processing':
        interval = f'{row["publish_call_interval_ms"]:.3f}' if 'publish_call_interval_ms' in row else '-'
        print(f'PROCESSING sequence={row["sequence"]} reset={row["reset_count"]} '
              f'update={row["update_count"]} step={row.get("sequence_step", "-")} '
              f'call_interval={interval} ingress_to_publish={row["ingress_to_publish_ms"]:.3f} '
              f'state_total={row["state_total_ms"]:.3f} ms')
        if 'stage_cpu_ms' in row and 'stage_gc' in row:
            stages = [f'{name}={value:.3f}/{row["stage_cpu_ms"][name]:.3f}/'
                      f'{sum(row["stage_gc"][name]["elapsed_ms"]):.3f}'
                      for name, value in row['stages_ms'].items()]
            counts = [sum(value['collections'][g] for value in row['stage_gc'].values()) for g in range(3)]
            collected = [sum(value['collected'][g] for value in row['stage_gc'].values()) for g in range(3)]
            print(f'  Cyclic GC generations 0/1/2: collections={counts}; objects collected={collected}')
        else:
            stages = [f'{name}={value:.3f}' for name, value in row['stages_ms'].items()]
            print('  CPU/GC instrumentation unavailable; stages show elapsed time only.')
        for offset in range(0, len(stages), 3):
            print('  ' + '  '.join(stages[offset:offset+3]))
    elif kind == 'health':
        print(f'HEALTH {row["at_utc"]}')
        memory = row.get('memory')
        if memory is not None:
            print('  Memory: ' + ', '.join(f'{key}={value}' for key, value in memory.items()))
        counters = row.get('counters')
        if counters is not None:
            print('  Counters: ' + ', '.join(f'{key}={value}' for key, value in counters.items()
                                          if not key.startswith('last_')))
            for key in ('last_system_report_failure', 'last_persistence_failure'):
                if counters.get(key) is not None:
                    print(f'  {key}: {json.dumps(counters[key])}')
        for key in ('report_round_trip_ms', 'memory_error', 'report_error', 'processing_timing', 'runtime'):
            if key in row:
                print(f'  {key}: {row[key]}')
    elif kind == 'summary':
        print('\nCADENCE SUMMARY')
        print('  Publications observed: ' + ', '.join(f'{key}={value}' for key, value in row['counts'].items()))
        print(f'{"Timing (ms)":37} {"N":>6} {"Minimum":>12} {"Median":>12} {"Maximum":>12}')
        for name, stats in row['timings_ms'].items():
            print(f'{name:37} {stats["n"]:6} {stats["min"]:12.3f} '
                  f'{stats["median"]:12.3f} {stats["max"]:12.3f}')
        print(f'\nDIRECT MONOTONIC PROCESSING: {row["processing_samples"]} completed samples')
        print(f'Cyclic GC diagnostics: {row["gc_samples"]} samples; generations 0/1/2: '
              f'collections={row["gc_collections"]}; objects collected={row["gc_collected"]}')
        print('stage.* = elapsed; cpu.* = state-thread CPU; gc.* = cyclic GC elapsed (overlapping).')
        print(f'{"Timing (ms)":42} {"N":>6} {"Minimum":>12} {"Median":>12} {"Maximum":>12}')
        for name, stats in row['processing_timings_ms'].items():
            print(f'{name:42} {stats["n"]:6} {stats["min"]:12.3f} '
                  f'{stats["median"]:12.3f} {stats["max"]:12.3f}')
        print('Direct timings exclude pre-capture history and duplicate REPORT samples.')
        print('Only completed state-worker rows are retained; final in-flight work may be absent.')
        print('The bounded history can omit rows after a long REPORT outage; gaps are reported.')
        print('Report duration may leave unmatched publications at the start/end.')
        print('Stage estimates include scheduling; wall-clock adjustments can affect them.')
        print('No service changes, database writes or automatic output files.')
        print('END PHOTONS CADENCE WITNESS REPORT')
    else:
        print(f'{kind.upper()}: {json.dumps({key:value for key, value in row.items() if key != "kind"})}')


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=int, default=60, help='cadence mode duration in seconds (default 60)')
    parser.add_argument('--jsonl', action='store_true', help='print complete JSONL records instead of the text report')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--db', action='store_true', help='capture host resources and read-only database diagnostics')
    modes.add_argument('--adc-high', action='store_true', help='report per-fragment ADC HIGH statistics')
    modes.add_argument('--adc-capture', action='store_true', help='dump individual ADC samples from the latest completed capture')
    parser.add_argument('--index', type=int, default=0,
                        help='ADC capture mode: starting row index (default 0)')
    parser.add_argument('--count', type=int, default=20,
                        help='ADC capture mode: requested rows, 1 to 64 (default 20)')
    parser.add_argument('--fragments', type=int, default=1,
                        help='ADC HIGH mode: number of fragments (default 1)')
    parser.add_argument('--window-seconds', type=int, default=60,
                        help='ADC HIGH mode: rolling population span, at least 2 seconds (default 60)')
    parser.add_argument('--timeout', type=float, default=10,
                        help='ADC HIGH mode: seconds waiting for each fragment (default 10)')
    parser.add_argument('--reference-mv', type=float, default=3300,
                        help='ADC HIGH mode: nominal reference, not calibration (default 3300)')
    args = parser.parse_args(argv[1:])
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    if args.window_seconds < 2:
        parser.error('--window-seconds must be at least 2')
    if args.fragments < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--fragments and --timeout must be positive')
    if not math.isfinite(args.reference_mv) or args.reference_mv <= 0:
        parser.error('--reference-mv must be positive and finite')
    if not 0 <= args.index <= 1000 or not 1 <= args.count <= 64:
        parser.error('--index must be 0 to 1000; --count must be 1 to 64')
    return args


def high_statistics(n, total, squares):
    """Exact integer variance numerator avoids cancellation near a constant HIGH."""
    if n == 0:
        return None, None, None
    mean = total / n
    if n == 1:
        return mean, None, None
    sd = math.sqrt((n * squares - total * total) / (n * (n - 1)))
    return mean, sd, sd / math.sqrt(n)


def observation(payload, reference_mv):
    instrument = payload['photons']
    pd = instrument['pd_out']
    n = pd['sample_count']
    # Missing instrumentation is an error, even for a fragment with no samples.
    total = pd['high_sum_counts']
    squares = pd['high_sum_squares_counts']
    target = pd['high_delay_target_ns']
    cps = pd['dwt_cycles_per_second']
    mean, sd, se = high_statistics(n, total, squares)
    scale = reference_mv / ((1 << pd['adc_bits']) - 1)
    return {
        'kind': 'fragment', 'sequence': payload['sequence'],
        'observed_utc': datetime.now(timezone.utc).isoformat(),
        'fragment_elapsed_ms': instrument['fragment_elapsed_ms'],
        'n': n, 'high_mean_counts': mean, 'high_sd_counts': sd,
        'high_se_counts': se,
        'high_mean_mv': None if mean is None else mean * scale,
        'high_sd_mv': None if sd is None else sd * scale,
        'high_se_mv': None if se is None else se * scale,
        'high_delay_target_ns': target,
        'high_start_after_entry_mean_ns': None if n == 0 else
            float(pd['high_start_after_entry_mean_cycles']) * 1e9 / cps,
        'busy_edges': pd['busy_edges'],
        'pd_out': pd,
    }


class HighWindow:
    """One population from exact fragment moments; retain whole fragments only."""
    def __init__(self, seconds, reference_mv):
        self.target_ms = seconds * 1000
        self.reference_mv = reference_mv
        self.rows = deque()
        self.elapsed_ms = self.n = self.total = self.squares = 0
        self.previous = None
        self.acquisition = None

    def add(self, row):
        pd = row['pd_out']
        acquisition = (pd['adc_bits'], pd.get('acquisition'),
                       pd['high_delay_target_ns'], pd['dwt_cycles_per_second'])
        reason = None
        if self.previous is not None:
            if row['sequence'] != self.previous + 1:
                reason = 'fragment sequence discontinuity'
            elif acquisition != self.acquisition:
                reason = 'ADC acquisition changed'
        if reason is not None:
            self.rows.clear()
            self.elapsed_ms = self.n = self.total = self.squares = 0
        elapsed = row['fragment_elapsed_ms']
        if elapsed <= 0:
            raise ValueError('ADC fragment duration must be positive')
        entry = (row['sequence'], elapsed, row['n'],
                 pd['high_sum_counts'], pd['high_sum_squares_counts'])
        self.rows.append(entry)
        self.elapsed_ms += elapsed
        self.n += entry[2]
        self.total += entry[3]
        self.squares += entry[4]
        # Keep the shortest suffix covering the target span; no invented
        # fractional-fragment samples at the oldest edge of the window.
        while len(self.rows) > 1 and self.elapsed_ms - self.rows[0][1] >= self.target_ms:
            _, duration, n, total, squares = self.rows.popleft()
            self.elapsed_ms -= duration
            self.n -= n
            self.total -= total
            self.squares -= squares
        self.previous = row['sequence']
        self.acquisition = acquisition
        mean, sd, se = high_statistics(self.n, self.total, self.squares)
        scale = self.reference_mv / ((1 << pd['adc_bits']) - 1)
        return {
            'target_seconds': self.target_ms / 1000,
            'elapsed_seconds': self.elapsed_ms / 1000,
            'fragments': len(self.rows), 'first_sequence': self.rows[0][0],
            'last_sequence': row['sequence'], 'n': self.n,
            'high_sum_counts': self.total, 'high_sum_squares_counts': self.squares,
            'high_mean_mv': None if mean is None else mean * scale,
            'high_sd_mv': None if sd is None else sd * scale,
            'high_se_mv': None if se is None else se * scale,
            'reset_reason': reason,
        }


def number(value):
    return 'n/a' if value is None else f'{value:.3f}'


def print_fragment(row):
    print(f"\nFragment {row['sequence']}  window={row['fragment_elapsed_ms']} ms"
          f"  N={row['n']}  sequence_step={row['sequence_step']}")
    print('                   MEAN           SD           SE')
    for unit in ('counts', 'mv'):
        values = [number(row[f'high_{stat}_{unit}']) for stat in ('mean', 'sd', 'se')]
        label = 'HIGH counts' if unit == 'counts' else 'HIGH mV'
        print(f'{label:12s}' + ''.join(f'{value:>13s}' for value in values))
    print(f"HIGH target={row['high_delay_target_ns']} ns; "
          f"entry->HIGH trigger mean={number(row['high_start_after_entry_mean_ns'])} ns")
    print(f"busy_edges={row['busy_edges']}")
    if row['n'] == 0:
        print('NO COMPLETED ADC HIGH SAMPLES in this fragment.')
    for key, label in (('high_window', 'HIGH60'), ('high5_window', 'HIGH5')):
        if key not in row['pd_out']:
            continue  # Historical firmware did not publish HIGH5.
        published = row['pd_out'][key]
        progress = 'warming up' if published['elapsed_ms'] < published['target_ms'] else 'full span'
        published_mean = published['high_mean_mv'] if published['sample_count'] > 0 else None
        print(f"PUBLISHED {label}: MEAN={number(published_mean)} mV; "
              f"{published['elapsed_ms'] / 1000:.3f} s / "
              f"{published['target_ms'] / 1000:g} s target ({progress}); "
              f"fragments={published['fragments']} N={published['sample_count']}; "
              f"nominal reference={published['reference_mv']} mV")
    window = row['rolling']
    if window['reset_reason']:
        print(f"ROLLING WINDOW RESTARTED: {window['reset_reason']}")
    progress = 'warming up' if window['elapsed_seconds'] < window['target_seconds'] else 'full span'
    print(f"LOCAL ROLLING HIGH: {window['elapsed_seconds']:.3f} s / "
          f"{window['target_seconds']:g} s target ({progress}); "
          f"fragments={window['fragments']} N={window['n']}")
    print(f"  MEAN={number(window['high_mean_mv'])} mV; "
          f"SD={number(window['high_sd_mv'])} mV; "
          f"SE={number(window['high_se_mv'])} mV (independence assumption)")


def adc_capture_report(args):
    """One read-only Teensy RPC; display each raw capture without reduction."""
    try:
        from zpnet.processes.processes import send_command

        response = send_command(
            machine='TEENSY', subsystem='INTERRUPT', command='REPORT_ADC_CAPTURE',
            args={'index': args.index, 'count': args.count}, retries=1)
        payload = response['payload']
        if not response['success'] or 'error' in payload:
            if args.jsonl:
                print(json.dumps({'kind': 'error', 'response': response},
                                 separators=(',', ':'), allow_nan=False))
            else:
                message = payload['error'] if 'error' in payload else response['message']
                print(f'ERROR: {message}')
            return 1
        if args.jsonl:
            print(json.dumps(payload, separators=(',', ':'), allow_nan=False))
            return 0

        if payload['samples_per_row'] != 1:
            raise ValueError('single-sample firmware required for this report version')

        print(f'PHOTONS ADC SINGLE CAPTURE REPORT {VERSION}')
        print(f"Capture {payload['capture_id']}  window={payload['elapsed_ms']} ms  "
              f"rows seen={payload['rows_seen']}  stored={payload['stored_rows']}  "
              f"omitted={payload['omitted_rows']}")
        print(f"Boot milliseconds: {payload['started_at_ms']} -> {payload['completed_at_ms']}; "
              f"requested index={payload['requested_index']} count={payload['requested_count']}; "
              f"returned={payload['returned_count']} next_index={payload['next_index']}")
        print(f"One {payload['adc_bits']}-bit ADC2 reading per edge on pin {payload['adc_pin']}; "
              f"nominal reference={payload['reference_mv']} mV.")
        print('Each value is one reading expressed in mV; no averaging, filtering or voting.')
        print('The latest completed capture refreshes automatically; separate calls may show different capture IDs.')
        if payload['stored_rows']:
            cps = payload['dwt_cycles_per_second']
            timing = payload['first_row_timing']
            print(f"First stored row only: entry DWT={payload['first_entry_dwt']}; CPU={cps} cycles/s; "
                  f"HIGH target={payload['high_delay_target_ns']} ns")
            print('             START cycles       START ns        READ cycles        READ ns')
            for sample in range(1, payload['samples_per_row'] + 1):
                values = timing[f'sample_{sample}']
                start = values['start_cycles']
                duration = values['read_cycles']
                print(f'{sample:>6d} {start:>18d} {start * 1e9 / cps:>14.3f} '
                      f'{duration:>18d} {duration * 1e9 / cps:>14.3f}')
            print('START is from ISR entry; READ includes handoff and preemption through result read, not ADC aperture.')
        else:
            print('No ADC samples in this completed window; first-row timing is unavailable.')
        print('   INDEX       SAMPLE 1 mV')
        first = payload['requested_index']
        for index in range(first, first + payload['returned_count']):
            row = payload[f'row_{index:04d}']
            print(f'{index:8d}' + ''.join(f"{row[f'sample_{sample}_mv']:18.3f}"
                                         for sample in range(1, payload['samples_per_row'] + 1)))
        if not payload['returned_count']:
            print('No rows in this requested page.')
        return 0
    except KeyboardInterrupt:
        if args.jsonl:
            print(json.dumps({'kind': 'interrupted', 'message': 'ADC capture report interrupted'}))
        else:
            print('ADC capture report interrupted.')
        return 130
    except (ImportError, OSError, KeyError, TypeError, ValueError, RuntimeError, ZeroDivisionError) as exc:
        message = f'ERROR: {exc}. Check the Teensy RPC broker and patched firmware.'
        if args.jsonl:
            print(json.dumps({'kind': 'error', 'message': message}))
        else:
            print(message)
        return 1


def adc_high_report(args):
    def emit(row):
        if args.jsonl:
            print(json.dumps(row, separators=(',', ':'), allow_nan=False), flush=True)
        elif row['kind'] == 'fragment':
            print_fragment(row)
            sys.stdout.flush()
        else:
            print(row['message'], flush=True)

    emit({'kind': 'start', 'version': VERSION, 'reference_mv': args.reference_mv,
          'message': f'PHOTONS ADC HIGH witness {VERSION}\n'
          'Each row covers completed physical ADC HIGH samples in one firmware fragment, not accepted laps.\n'
          'SD is sample SD; SE = SD/sqrt(N), assuming independent samples. Neither establishes accuracy.\n'
          f'mV uses nominal reference {args.reference_mv:g} mV. Trigger timing is relative to ISR entry,\n'
          'not the physical edge or ADC aperture. n/a means the statistic is undefined.\n'
          f'Rolling window target={args.window_seconds} s; integer moments pooled across whole fragments.\n'
          'PUBLISHED HIGH5/HIGH60 come from firmware; LOCAL ROLLING HIGH starts with this report.\n'
          'Both means lag changes. One-second HIGH and lap normalization remain unchanged.'})
    received = 0
    previous = None
    samples = 0
    window = HighWindow(args.window_seconds, args.reference_mv)
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(args.timeout)
            sock.connect(TAP)
            sock.sendall(b'{"type":"set_topics","topics":["PHOTONS_FRAGMENT"]}\n')
            buffer = b''
            deadline = time.monotonic() + args.timeout
            while received < args.fragments:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('Timed out waiting for PHOTONS_FRAGMENT')
                sock.settimeout(remaining)
                chunk = sock.recv(65536)
                if not chunk:
                    raise ConnectionError('PUBSUB tap disconnected')
                buffer += chunk
                while b'\n' in buffer and received < args.fragments:
                    line, buffer = buffer.split(b'\n', 1)
                    if not line.strip():
                        continue
                    message = json.loads(line)
                    if message.get('type') != 'publish':
                        emit({'kind': 'tap_control', 'message': json.dumps(message)})
                        continue
                    if message['topic'] != 'PHOTONS_FRAGMENT':
                        continue
                    row = observation(message['payload'], args.reference_mv)
                    row['sequence_step'] = None if previous is None else row['sequence'] - previous
                    row['rolling'] = window.add(row)
                    emit(row)
                    previous = row['sequence']
                    received += 1
                    samples += row['n']
                    deadline = time.monotonic() + args.timeout
    except KeyboardInterrupt:
        emit({'kind': 'interrupted', 'message': f'Interrupted after {received} fragments.'})
        return 130
    except (OSError, KeyError, ValueError) as exc:
        emit({'kind': 'error', 'message': f'ERROR: {exc}. Check acquisition and patched firmware.'})
        return 1
    emit({'kind': 'end', 'fragments': received, 'samples': samples,
          'message': f'Captured {received} fragments / {samples} completed ADC HIGH samples.'})
    return 0 if samples else 1


DB_PROBE = r'''
import json
import time
import psycopg
from psycopg.rows import dict_row
from zpnet.shared.db import PG_DSN

def emit(value):
    print(json.dumps(value, default=str), flush=True)

started = time.monotonic()
emit({'stage': 'connect_started'})
with psycopg.connect(
    PG_DSN, row_factory=dict_row, autocommit=True, connect_timeout=3,
    application_name='ZPNET:photons_cadence_witness',
    options='-c default_transaction_read_only=on -c statement_timeout=3000 -c lock_timeout=1000',
) as conn:
    emit({'stage': 'connected', 'elapsed_ms': round((time.monotonic()-started)*1000, 3)})
    queries = (
        ('server', """SELECT current_database() AS database,
            current_setting('server_version') AS version,
            current_setting('max_connections') AS max_connections,
            pg_postmaster_start_time() AS started_at"""),
        ('database', """SELECT numbackends, xact_commit, xact_rollback,
            blks_read, blks_hit, temp_files, temp_bytes, deadlocks, stats_reset
            FROM pg_stat_database WHERE datname = current_database()"""),
        ('session_groups', """SELECT datname, backend_type, state,
            wait_event_type, wait_event, count(*) AS sessions
            FROM pg_stat_activity GROUP BY 1,2,3,4,5 ORDER BY sessions DESC LIMIT 30"""),
        ('busy_sessions', """SELECT pid, application_name, backend_type, state,
            wait_event_type, wait_event,
            EXTRACT(EPOCH FROM clock_timestamp()-xact_start) AS transaction_age_s,
            EXTRACT(EPOCH FROM clock_timestamp()-query_start) AS query_age_s,
            pg_blocking_pids(pid) AS blocking_pids
            FROM pg_stat_activity WHERE pid <> pg_backend_pid()
              AND (state IS DISTINCT FROM 'idle')
            ORDER BY xact_start NULLS LAST, query_start NULLS LAST LIMIT 20"""),
        ('tables', """SELECT schemaname, relname, n_live_tup, n_dead_tup,
            n_tup_ins, n_tup_upd, n_tup_del, last_autovacuum, last_autoanalyze,
            pg_total_relation_size(relid) AS total_bytes
            FROM pg_stat_user_tables WHERE relname IN
              ('config', 'campaign_detail', 'campaign_master')"""),
        ('checkpoint_sizes', """SELECT config_key,
            pg_column_size(payload) AS stored_bytes,
            octet_length(payload::text) AS json_text_bytes
            FROM config WHERE config_key IN ('PHOTONS_RECOVERY', 'CLOCKS_RECOVERY')"""),
    )
    for name, sql in queries:
        emit({'stage': name+'_started'})
        started = time.monotonic()
        with conn.cursor() as cur:
            cur.execute(sql)
            emit({'stage': name, 'elapsed_ms': round((time.monotonic()-started)*1000, 3),
                  'rows': cur.fetchall()})
'''


def database_report(args):
    """One bounded diagnostic capture; no table scans of measurement history."""
    exit_code = 0

    def emit(section, value):
        if args.jsonl:
            print(json.dumps({'kind': 'db_diagnostic', 'at_utc': stamp(),
                              'section': section, 'value': value}, default=str), flush=True)
        elif isinstance(value, dict) and 'stdout' in value:
            print(f'\n{section}', flush=True)
            print(json.dumps({key: item for key, item in value.items()
                              if key not in ('stdout', 'stderr')}), flush=True)
            print(value['stdout'], flush=True)
            if value['stderr']:
                print('STDERR: ' + value['stderr'], flush=True)
        else:
            print(f'\n{section}\n' + (value if isinstance(value, str) else
                                      json.dumps(value, indent=2, default=str)), flush=True)

    def run(section, command, timeout, *, cwd=None, lines=None):
        nonlocal exit_code
        started = time.monotonic()
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=timeout, cwd=cwd)
            output = result.stdout
            if lines is not None:
                output = '\n'.join(output.splitlines()[:lines])
            emit(section, {'elapsed_ms': round((time.monotonic()-started)*1000, 3),
                           'returncode': result.returncode, 'stdout': output,
                           'stderr': result.stderr})
            if result.returncode:
                exit_code = 1
        except subprocess.TimeoutExpired as exc:
            # Timeout output can be bytes even with text=True; preserve evidence
            # produced before the connection or statement stalled.
            out = exc.stdout or b''
            err = exc.stderr or b''
            emit(section, {'error': f'Exceeded {timeout}s wall-clock limit',
                           'stdout': out.decode(errors='replace') if isinstance(out, bytes) else out,
                           'stderr': err.decode(errors='replace') if isinstance(err, bytes) else err})
            exit_code = 1
        except OSError as exc:
            emit(section, {'error': str(exc)})
            exit_code = 1

    emit('PHOTONS DATABASE / RESOURCE WITNESS',
         f'{VERSION}; read-only snapshot. No service restart or acquisition changes.\n'
         'PostgreSQL activity visibility depends on the configured DB role.\n'
         'vmstat: first row is since boot; subsequent rows cover one second.\n'
         'No full checkpoint payloads, query text or connection credentials are printed.')
    try:
        run('HOST MEMORY', ['free', '-m'], 3)
        run('LARGEST PROCESSES (RSS KiB)',
            ['ps', '-eo', 'pid,comm,rss,pcpu,stat', '--sort=-rss'], 3, lines=21)
        run('FILESYSTEM SPACE', ['df', '-h', '-l'], 3)
        run('SWAP / CPU / IO', ['vmstat', '1', '5'], 8)
        try:
            emit('PHOTONS PROCESS', process_memory())
            response = report()
            if not response['success']:
                raise RuntimeError(str(response))
            payload = response['payload']
            emit('PHOTONS BACKLOG', {key: payload[key] for key in COUNTERS})
            emit('PHOTONS STATE', {'operational_state': payload['operational_state'],
                                   'recovery': payload['recovery']})
        except (OSError, KeyError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            emit('PHOTONS REPORT ERROR', str(exc))
            exit_code = 1
        # A separate process puts a wall-clock bound on all DB work, even if
        # connection startup, scheduling or socket IO ignores server deadlines.
        run('POSTGRESQL', [sys.executable, '-c', DB_PROBE], 15,
            cwd=Path(__file__).resolve().parents[1])
    except KeyboardInterrupt:
        emit('INTERRUPTED', 'Diagnostic capture interrupted.')
        return 130
    emit('END DATABASE / RESOURCE WITNESS', {'returncode': exit_code})
    return exit_code


def main(argv):
    args = arguments(argv)
    if args.db:
        return database_report(args)
    if args.adc_high:
        return adc_high_report(args)
    if args.adc_capture:
        return adc_capture_report(args)
    output_lock = threading.Lock()
    def emit(value):
        with output_lock:
            if args.jsonl:
                print(json.dumps(value, separators=(',', ':')), flush=True)
            else:
                print_record(value)
                sys.stdout.flush()
    emit({'kind':'start', 'version':VERSION, 'at_utc':stamp(), 'duration_s':args.seconds,
          'notes':'Read-only tap + REPORT every 5 s. Tap times include delivery/scheduling; '
                  'wall-clock stage estimates can be affected by clock adjustments.'})
    stopped = threading.Event()
    def sample_once():
        row = {'kind':'health', 'at_utc':stamp()}
        try:
            row['memory'] = process_memory()
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            row['memory_error'] = str(exc)
        started = time.monotonic()
        try:
            response = report()
            row['report_round_trip_ms'] = round((time.monotonic()-started)*1000, 3)
            if response.get('success'):
                payload = response['payload']
                row['counters'] = {key:payload.get(key) for key in COUNTERS}
                timing = payload.get('cadence_timing')
                if timing is None:
                    row['processing_timing'] = 'Instrumentation not installed in this PHOTONS process.'
                else:
                    witness.accept_processing(timing)
                    if 'runtime' in timing:
                        row['runtime'] = timing['runtime']
                    row['processing_timing'] = (
                        f'{timing["sample_count"]} completed since process start; '
                        f'{len(timing["samples"])}/{timing["capacity"]} retained')
            else:
                row['report_error'] = response
        except (OSError, ValueError) as exc:
            row['report_error'] = str(exc)
        emit(row)
    def sample():
        while not stopped.is_set():
            sample_once()
            stopped.wait(5)
    sampler = threading.Thread(target=sample, daemon=True)
    witness = Witness(emit)
    exit_code = 0
    sampler.start()
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(2)
            sock.connect(TAP)
            sock.sendall((json.dumps({'type':'set_topics', 'topics':TOPICS})+'\n').encode())
            deadline = time.monotonic()+args.seconds
            buffer = b''
            while time.monotonic() < deadline:
                sock.settimeout(min(1, max(0.001, deadline-time.monotonic())))
                try:
                    chunk = sock.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    emit({'kind':'tap_disconnected', 'at_utc':stamp()})
                    exit_code = 1
                    break
                buffer += chunk
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    if line.strip():
                        mono, wall = time.monotonic(), time.time()
                        witness.accept(json.loads(line), mono, wall)
            witness.ended_ns = time.monotonic_ns()
            sock.sendall(b'{"type":"ping"}\n')
            sock.settimeout(1)
            # Read the broker's per-client drop counter without extending the
            # timing sample with publications from this final exchange.
            deadline = time.monotonic()+1
            pong = False
            while not pong and time.monotonic() < deadline:
                try:
                    chunk = sock.recv(65536)
                except socket.timeout:
                    break
                if not chunk:
                    break
                buffer += chunk
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    if line.strip():
                        message = json.loads(line)
                        if message.get('type') == 'pong':
                            emit({'kind':'tap_counters', 'message':message})
                            pong = True
            if not pong:
                emit({'kind':'tap_counters_unavailable'})
    except OSError as exc:
        emit({'kind':'tap_error', 'error':str(exc)})
        exit_code = 1
    except KeyboardInterrupt:
        emit({'kind':'interrupted', 'at_utc':stamp()})
        exit_code = 130
    finally:
        if witness.ended_ns is None:
            witness.ended_ns = time.monotonic_ns()
        stopped.set()
        sampler.join()
        # Pick up the last partial five-second REPORT interval without counting
        # subsequent publications. The tap's summary remains unchanged.
        sample_once()
        witness.summarize()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
