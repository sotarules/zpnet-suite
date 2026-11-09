"""Read-only campaign scans with paced and explicit full-speed profiles.

Pages are separate transactions, not a campaign-wide database snapshot.
The ID ceiling excludes subsequent inserts; campaign_detail must be append-only.
No timeout retries and no partial-success fallback.
--fast removes deliberate throttling and defaults to 1024 rows per page.
It can compete with live persistence for CPU/I/O; it changes no global DB
settings. A report-owned connection commits each page before yielding rows.
"""
import math
import sys
import time



def add_read_arguments(parser):
    parser.add_argument('--fast', action='store_true',
                        help='full-speed reads: 1024-row pages, no sleep/backoff, 30s query timeout')
    parser.add_argument('--batch-size', type=int, default=None,
                        help='rows per transaction, 1..4096 (default 32; --fast 1024)')
    parser.add_argument('--batch-pause', type=float, default=None,
                        help='minimum idle seconds, 0..30 (default 0.5; --fast 0)')
    parser.add_argument('--read-backoff', type=float, default=None,
                        help='idle/read-time multiplier, 0..30 (default 4; --fast 0)')


def validate_read_arguments(parser, args):
    # Explicit settings override the profile, independent of argument order.
    if args.batch_size is None:
        args.batch_size = 1024 if args.fast else 32
    if args.batch_pause is None:
        args.batch_pause = 0.0 if args.fast else 0.5
    if args.read_backoff is None:
        args.read_backoff = 0.0 if args.fast else 4.0
    if not 1 <= args.batch_size <= 4096:
        parser.error('--batch-size must be 1..4096')
    if not math.isfinite(args.batch_pause) or not 0 <= args.batch_pause <= 30:
        parser.error('--batch-pause must be finite and 0..30 seconds')
    if not math.isfinite(args.read_backoff) or not 0 <= args.read_backoff <= 30:
        parser.error('--read-backoff must be finite and 0..30')


def _read(sql, params, *, session=None, fast=False):
    from zpnet.shared.db import open_db

    with (session.open() if session is not None else open_db(row_dict=True)) as db:
        db.execute('SET TRANSACTION READ ONLY')
        # Local to this short transaction; acquisition retains its own policy.
        db.execute("SET LOCAL statement_timeout = '30s'" if fast
                   else "SET LOCAL statement_timeout = '5s'")
        db.execute("SET LOCAL lock_timeout = '500ms'")
        db.execute("SET LOCAL work_mem = '16MB'" if fast
                   else "SET LOCAL work_mem = '4MB'")
        db.execute('SET LOCAL max_parallel_workers_per_gather = 0')
        db.execute('SET LOCAL jit = off')
        return db.execute(sql, params).fetchall()


def campaign_rows(campaign, projection, args, *, descending=False, limit=None):
    """Yield projected rows; projection is trusted report SQL, never user input.

    Schema filtering follows the bounded page, so LIMIT bounds payload reads
    even when a campaign contains other schemas. A supporting relational index
    is required for efficient page selection (see supplied index DDL).
    """
    from zpnet.shared.db import DatabaseWorkerSession

    scan_started = time.monotonic()
    read_seconds = sleep_seconds = last_read_seconds = 0.0
    queries = scanned = matched = 0
    upper = None
    complete = False

    def timing(message):
        wall = time.monotonic() - scan_started
        # Read includes connect/reconnect, SQL execution, transfer, driver
        # decoding, and page commit. Other includes consumer time at yield
        # (parsing/validation/block building) plus reader bookkeeping. Fitting
        # after iterator exhaustion is outside this scan timing entirely.
        other = max(0.0, wall - read_seconds - sleep_seconds)
        rate = scanned / wall if wall else 0.0
        print(f'[report read] {message}; wall={wall:.2f}s read={read_seconds:.2f}s '
              f'sleep={sleep_seconds:.2f}s consumer+reader={other:.2f}s '
              f'queries={queries} rows/s={rate:.1f}', file=sys.stderr, flush=True)

    # Session is local to this iterator, never shared with acquisition workers.
    # Reuse its connection, not its transaction/snapshot. Existing ID ceiling
    # and keyset paging preserve the same append-only scan contract in FAST.
    # The shared session rotates connections after five minutes. Exceptions,
    # generator close, and Ctrl-C all leave this context and close the session.
    try:
        with DatabaseWorkerSession(row_dict=True) as session:
            def read(sql, params):
                nonlocal read_seconds, last_read_seconds, queries
                started = time.monotonic()
                queries += 1
                try:
                    return _read(sql, params, session=session, fast=args.fast)
                finally:
                    last_read_seconds = time.monotonic() - started
                    read_seconds += last_read_seconds

            where, params = 'campaign_type = %s', ['LANTERN']
            if campaign is not None:
                where += ' AND campaign = %s'
                params.append(campaign)
            ceiling = read('SELECT id FROM campaign_detail WHERE ' + where +
                            ' ORDER BY id DESC LIMIT 1', params)
            if not ceiling:
                complete = True
                return
            upper = int(ceiling[0]['id'])
            print(f'[report read] campaign={campaign!r} through_id={upper} '
                  f'profile={"FAST" if args.fast else "PACED"} batch={args.batch_size} '
                  f'pause>={args.batch_pause:g}s backoff={args.read_backoff:g}x', file=sys.stderr, flush=True)
            delay = max(args.batch_pause, args.read_backoff * last_read_seconds)
            after = None
            progress = time.monotonic()
            direction, operator = ('DESC', '<') if descending else ('ASC', '>')
            while limit is None or matched < limit:
                # No open transaction while sleeping or while the caller processes
                # a yielded row. FAST defaults to no sleep calls at all.
                if delay > 0:
                    sleep_started = time.monotonic()
                    time.sleep(min(delay, 30))
                    sleep_seconds += time.monotonic() - sleep_started
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
                batch = read(sql, values)
                delay = max(args.batch_pause, args.read_backoff * last_read_seconds)
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
                    timing(f'scanned={scanned} PHOTONS={matched} last_id={after}')
                    progress = time.monotonic()
                if len(batch) < size:
                    break
            complete = True

    finally:
        status = 'complete' if complete else 'interrupted'
        timing(f'{status}: scanned={scanned} PHOTONS={matched} through_id={upper}')
