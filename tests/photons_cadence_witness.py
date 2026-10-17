"""Report live PHOTONS publication cadence, processing delays and queue growth.

    zt photons_cadence_witness --seconds 60
    zt photons_cadence_witness --seconds 60 --jsonl
    python3 photons_cadence_witness.py --seconds 60

Prints the complete report to STDOUT; no files or database values are changed.
Observes PHOTONS_FRAGMENT and PHOTONS through the read-only PUBSUB tap, requests
PHOTONS REPORT approximately every five seconds and reads process memory.
Timing at the tap includes observer/broker scheduling. Existing PHOTONS context
timestamps provide separate evidence of ingress, SYSTEM RPC and construction.
Instrumented PHOTONS REPORT adds bounded per-sequence monotonic stage timings;
these stay outside canonical measurements and recovery state. Only publish calls
inside this capture window are included; repeated REPORT samples are deduplicated.
Standard library only. main(argv) includes the program name for the zt runner.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import statistics
import subprocess
import sys
import threading
import time

VERSION = '2026-10-17.2'
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
        print('Stages are elapsed time, including scheduling/GC; they do not isolate CPU time.')
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
        stages = [f'{name}={value:.3f}' for name, value in row['stages_ms'].items()]
        for offset in range(0, len(stages), 4):
            print('  ' + '  '.join(stages[offset:offset+4]))
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
        for key in ('report_round_trip_ms', 'memory_error', 'report_error', 'processing_timing'):
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
    parser.add_argument('--seconds', type=int, default=60, help='capture duration in seconds (default 60)')
    parser.add_argument('--jsonl', action='store_true', help='print complete JSONL records instead of the text report')
    args = parser.parse_args(argv[1:])
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    return args


def main(argv):
    args = arguments(argv)
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
