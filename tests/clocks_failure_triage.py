"""Read-only evidence capture for CLOCKS silence and Teensy Payload fatalities.

    zt clocks_failure_triage NewOcxos2
    zt clocks_failure_triage NewOcxos2 --retained-traces --rows 96

Place in the repository's top-level tests/ directory. Run via the zt Python
runner; main receives argv[0]. This program never issues STOP, REPAIR, RESET,
CLEAR, DAC or servo commands, and never writes PostgreSQL state. It reads the
firmware's latest retained fatal evidence, Pi CLOCKS diagnostics, and bounded
campaign/Alpha chronology. Report is copyable JSONL on STDOUT. Do this before
any firmware flash or SYSTEM.CRASH_CLEAR that could retire retained evidence.

The potentially oversized firmware CLOCKS.REPORT_RECOVERY is deliberately NOT
invoked, because its construction is under investigation as a crash trigger.
Queries are separate paced read-only transactions with bounded statement/lock
limits. A failed RPC or missing required DB row is an explicit run failure;
earlier JSONL output remains available for diagnosis.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import sys
import time
from typing import Any, Sequence

from zpnet.processes.processes import send_command
from zpnet.shared.db import open_db
from zpnet.shared.report_db import add_read_arguments, validate_read_arguments

VERSION = '2026-10-26.1'
CAMPAIGN_TYPE = 'TEMPEST'


def emit(kind: str, **fields: Any) -> None:
    print(json.dumps({'type': kind, **fields}, ensure_ascii=True,
                     allow_nan=False, separators=(',', ':'), default=str), flush=True)


def rpc(machine: str, subsystem: str, command: str,
        args: dict[str, Any] | None = None) -> dict[str, Any]:
    response = send_command(machine=machine, subsystem=subsystem,
                            command=command, args=args, retries=1,
                            retry_delay_s=0.0)
    if response.get('success') is not True:
        raise RuntimeError(f'{machine}.{subsystem}.{command}: {response!r}')
    return response


class ReadOnly:
    def __init__(self, args: argparse.Namespace):
        self.pause = args.batch_pause
        self.batch_size = args.batch_size
        self.last_finished: float | None = None

    def query(self, sql: str, params: tuple[Any, ...] = ()) -> list[dict[str, Any]]:
        if self.last_finished is not None:
            time.sleep(max(0.0, self.pause - (time.monotonic() - self.last_finished)))
        try:
            with open_db(row_dict=True) as conn:
                conn.execute('SET TRANSACTION READ ONLY')
                conn.execute("SET LOCAL statement_timeout = '5s'")
                conn.execute("SET LOCAL lock_timeout = '500ms'")
                conn.execute("SET LOCAL work_mem = '4MB'")
                conn.execute('SET LOCAL max_parallel_workers_per_gather = 0')
                conn.execute('SET LOCAL jit = off')
                rows = conn.execute(sql, params).fetchall()
        finally:
            self.last_finished = time.monotonic()
        return rows

    def around(self, campaign: str, count: int) -> list[dict[str, Any]]:
        result = []
        before = None
        while len(result) < count:
            n = min(self.batch_size, count - len(result))
            predicate = ' AND id < %s' if before is not None else ''
            params = ((CAMPAIGN_TYPE, campaign, before, n) if before is not None
                      else (CAMPAIGN_TYPE, campaign, n))
            page = self.query(
                'WITH page AS MATERIALIZED ('
                'SELECT id,campaign,sequence,viable,payload FROM campaign_detail '
                f'WHERE campaign_type = %s AND campaign = %s{predicate} '
                'ORDER BY id DESC LIMIT %s) '
                'SELECT id,campaign,sequence,viable,'
                "payload #>> '{published_at_utc}' AS published_at_utc,"
                "payload #>> '{campaign,public_count}' AS public_count,"
                "payload #>> '{clocks,stats,reset_count}' AS reset_count,"
                "payload #>> '{clocks,stats,update_count}' AS update_count,"
                "payload #>> '{clocks,raw_cycles,ocxo1,observed_cycles}' AS ocxo1_cycles,"
                "payload #>> '{clocks,raw_cycles,ocxo2,observed_cycles}' AS ocxo2_cycles,"
                "payload #>> '{campaign,ocxo1,science,science_worthy}' AS ocxo1_science,"
                "payload #>> '{campaign,ocxo2,science,science_worthy}' AS ocxo2_science,"
                "payload #>> '{holistic_restore_superseded}' AS superseded,"
                "payload #>> '{recovery_custody,reason}' AS custody_reason,"
                "(payload #> '{campaign,adjudication}' IS NOT NULL) AS adjudicated "
                'FROM page ORDER BY id DESC', params)
            if not page:
                break
            result.extend(page)
            before = int(page[-1]['id'])
            if len(page) < n:
                break
        return result


def main(argv: Sequence[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign', help='TEMPEST campaign to inspect, e.g. NewOcxos2')
    p.add_argument('--rows', type=int, default=96,
                   help='most recent campaign rows (default 96, max 256)')
    p.add_argument('--after', type=int, default=64,
                   help='ambient CLOCKS rows after last campaign detail (default 64, max 128)')
    p.add_argument('--retained-traces', action='store_true',
                   help='also request bounded retained Payload traces and crash summary')
    add_read_arguments(p)
    a = p.parse_args(argv[1:])
    validate_read_arguments(p, a)
    if not 1 <= a.rows <= 256 or not 0 <= a.after <= 128:
        p.error('--rows must be 1..256 and --after must be 0..128')
    a.campaign = a.campaign.strip()
    if not a.campaign:
        p.error('campaign is required')

    emit('header', schema='CLOCKS_FAILURE_TRIAGE_V1', version=VERSION,
         campaign=a.campaign, created_utc=datetime.now(timezone.utc).isoformat(),
         read_only=True, retained_traces=a.retained_traces)

    # Read the smallest retained lifetime evidence first. The live counters below
    # belong to the current boot; the retained fatal trace may belong to an earlier one.
    for machine, subsystem, command, args in (
        ('TEENSY', 'SYSTEM', 'PAYLOAD_FATAL_INFO', None),
    ):
        emit('rpc', owner=f'{machine}.{subsystem}', command=command,
             response=rpc(machine, subsystem, command, args))

    if a.retained_traces:
        for command in ('CRASH_INFO', 'PAYLOAD_MEMORY_INFO'):
            emit('rpc', owner='TEENSY.SYSTEM', command=command,
                 response=rpc('TEENSY', 'SYSTEM', command))
        for command in ('PAYLOAD_APPEND_TRACE', 'PAYLOAD_HEAP_RESIZE_TRACE',
                        'PAYLOAD_STAMP_TRACE'):
            emit('rpc', owner='TEENSY.SYSTEM', command=command,
                 response=rpc('TEENSY', 'SYSTEM', command,
                              {'bank': 'retained', 'count': 8}))
        emit('rpc', owner='TEENSY.SYSTEM', command='PAYLOAD_CONTRACT_INFO',
             response=rpc('TEENSY', 'SYSTEM', 'PAYLOAD_CONTRACT_INFO'))

    clocks = rpc('PI', 'CLOCKS', 'CLOCKS_INFO').get('payload') or {}
    diag = clocks.get('diag') or {}
    relevant = (
        'last_watchdog_anomaly', 'last_timebase_silence',
        'last_timebase_silence_producer_progress', 'last_hard_fault',
        'last_auto_recovery_exhausted', 'last_recovery',
        'last_recovery_custody', 'last_hard_failure_hold',
        'clocks_fragments_with_campaign', 'clocks_fragments_observation_only',
        'timebase_candidates_received', 'timebase_candidates_processed',
        'last_timebase_activity', 'last_recovery_custody',
    )
    emit('pi_clocks', operational_state=clocks.get('operational_state'),
         campaign_active=clocks.get('campaign_active'),
         timebase_silence_monitor=clocks.get('timebase_silence_monitor'),
         diag={key: diag.get(key) for key in dict.fromkeys(relevant) if key in diag})
    emit('pi_dac', response=rpc('PI', 'CLOCKS', 'DAC_INFO'))

    db = ReadOnly(a)
    master_rows = db.query(
        'SELECT id,active,ts,payload FROM campaign_master '
        'WHERE campaign_type = %s AND campaign = %s',
        (CAMPAIGN_TYPE, a.campaign))
    if len(master_rows) != 1:
        raise RuntimeError(f'expected one master for {a.campaign!r}, got {len(master_rows)}')
    master = master_rows[0]
    payload = master['payload']
    if isinstance(payload, str):
        payload = json.loads(payload)
    dac = payload.get('campaign_dac') or {}
    last_id = dac.get('last_detail_id')
    emit('campaign_master', id=master['id'], active=master['active'],
         ts=master['ts'], started_at=payload.get('started_at'),
         stopped_at=payload.get('stopped_at'), campaign_dac=dac)

    configs = db.query(
        'SELECT config_key,payload FROM config WHERE config_key IN '
        "('CLOCKS_RECOVERY','CLOCKS_ALPHA_LINEAGE_CUTOFF')")
    for entry in configs:
        doc = entry['payload']
        if isinstance(doc, str):
            doc = json.loads(doc)
        if entry['config_key'] == 'CLOCKS_RECOVERY':
            doc = {key: doc.get(key) for key in (
                'schema','durable_source_detail_id','recoverable','status',
                'reset_count','update_count','current_sequence','rolling_sequence')}
        emit('config', key=entry['config_key'], data=doc)

    recent = db.around(a.campaign, a.rows)
    for row in reversed(recent):
        emit('campaign_row', **row)
    if last_id is not None and a.after:
        rows = db.query(
            'WITH page AS MATERIALIZED ('
            'SELECT id,campaign,sequence,payload FROM campaign_detail '
            'WHERE campaign_type=%s AND id > %s ORDER BY id ASC LIMIT %s) '
            'SELECT id,campaign,sequence,'
            "payload #>> '{published_at_utc}' AS published_at_utc,"
            "payload #>> '{clocks,stats,reset_count}' AS reset_count,"
            "payload #>> '{clocks,stats,update_count}' AS update_count,"
            "payload #>> '{campaign,public_count}' AS public_count,"
            "payload #>> '{recovery_custody,reason}' AS custody_reason,"
            "payload #>> '{holistic_restore_superseded}' AS superseded "
            'FROM page ORDER BY id ASC',
            (CAMPAIGN_TYPE, int(last_id), a.after))
        for row in rows:
            emit('after_campaign_boundary', **row)
    emit('end', campaign_rows=len(recent), last_campaign_detail_id=last_id)
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
