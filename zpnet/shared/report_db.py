"""Paced, read-only campaign scans for offline PHOTONS reports.

Pages are separate transactions, not a campaign-wide database snapshot.
The ID ceiling excludes subsequent inserts; campaign_detail must be append-only.
No timeout retries and no partial-success fallback.
"""
import math
import sys
import time



def add_read_arguments(parser):
    parser.add_argument('--batch-size', type=int, default=32,
                        help='database rows per transaction, 1..256 (default 32)')
    parser.add_argument('--batch-pause', type=float, default=0.5,
                        help='minimum idle seconds between pages, 0.1..30 (default 0.5)')


def validate_read_arguments(parser, args):
    if not 1 <= args.batch_size <= 256:
        parser.error('--batch-size must be 1..256')
    if not math.isfinite(args.batch_pause) or not 0.1 <= args.batch_pause <= 30:
        parser.error('--batch-pause must be finite and 0.1..30 seconds')


def _read(sql, params):
    from zpnet.shared.db import open_db

    with open_db(row_dict=True) as db:
        db.execute('SET TRANSACTION READ ONLY')
        # Local to this short transaction; acquisition retains its own policy.
        db.execute("SET LOCAL statement_timeout = '5s'")
        db.execute("SET LOCAL lock_timeout = '500ms'")
        db.execute("SET LOCAL work_mem = '4MB'")
        db.execute('SET LOCAL max_parallel_workers_per_gather = 0')
        db.execute('SET LOCAL jit = off')
        return db.execute(sql, params).fetchall()


def campaign_rows(campaign, projection, args, *, descending=False, limit=None):
    """Yield projected rows; projection is trusted report SQL, never user input.

    Schema filtering follows the bounded page, so LIMIT bounds payload reads
    even when a campaign contains other schemas. A supporting relational index
    is required for efficient page selection (see supplied index DDL).
    """
    where, params = 'campaign_type = %s', ['LANTERN']
    if campaign is not None:
        where += ' AND campaign = %s'
        params.append(campaign)
    started = time.monotonic()
    ceiling = _read('SELECT id FROM campaign_detail WHERE ' + where +
                    ' ORDER BY id DESC LIMIT 1', params)
    if not ceiling:
        return
    upper = int(ceiling[0]['id'])
    print(f'[report read] campaign={campaign!r} through_id={upper} '
          f'batch={args.batch_size} pause>={args.batch_pause:g}s', file=sys.stderr, flush=True)
    delay = max(args.batch_pause, 4 * (time.monotonic() - started))
    after, scanned, matched = None, 0, 0
    progress = time.monotonic()
    direction, operator = ('DESC', '<') if descending else ('ASC', '>')
    while limit is None or matched < limit:
        # Connection and transaction are closed before this idle interval.
        time.sleep(min(delay, 30))
        predicates = where + ' AND id <= %s'
        values = params + [upper]
        if after is not None:
            predicates += f' AND id {operator} %s'
            values.append(after)
        size = args.batch_size if limit is None else min(args.batch_size, limit - matched)
        values.append(size)
        # Extract top-level members once from the large TOASTed payload, then
        # project only the small report surface. Both materialization barriers
        # are intentional: LIMIT before extraction; extraction before reuse.
        sql = f'''WITH page AS MATERIALIZED (
            SELECT id, campaign, payload FROM campaign_detail
            WHERE {predicates} ORDER BY id {direction} LIMIT %s
        ), unpacked AS MATERIALIZED (
            SELECT page.id, page.campaign, doc.* FROM page
            CROSS JOIN LATERAL jsonb_to_record(page.payload) AS doc(
                schema jsonb, sequence jsonb, published_at_utc jsonb,
                temperature jsonb, environment jsonb, receiver jsonb,
                normalization jsonb, measurement jsonb, recovery_receipt jsonb,
                photons jsonb)
        ) ''' + projection + f' FROM unpacked ORDER BY id {direction}'
        started = time.monotonic()
        batch = _read(sql, values)
        delay = max(args.batch_pause, 4 * (time.monotonic() - started))
        if not batch:
            break
        after = int(batch[-1]['id'])
        scanned += len(batch)
        for row in batch:
            if row['report_schema'] != 'PHOTONS_V1':
                continue
            matched += 1
            yield row
        if time.monotonic() - progress >= 10:
            print(f'[report read] scanned={scanned} PHOTONS={matched} last_id={after}',
                  file=sys.stderr, flush=True)
            progress = time.monotonic()
        if len(batch) < size:
            break
    print(f'[report read] complete: scanned={scanned} PHOTONS={matched} '
          f'through_id={upper}', file=sys.stderr, flush=True)
