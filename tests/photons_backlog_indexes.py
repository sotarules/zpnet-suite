"""Inspect/build campaign lookup indexes without restarting acquisition.

    zt photons_backlog_indexes          # catalog only, no table scan or DDL
    zt photons_backlog_indexes --apply  # explicit concurrent index maintenance

Run one maintenance instance at a time. Concurrent construction permits
writers, but still consumes CPU/I/O: close redundant Metrics/report sessions
first. No payloads, campaign state, planner defaults or durability settings
are changed. A failed build can leave an INVALID index; inspect the printed
definition and remove that failed index explicitly before retrying. Never
drop an existing index automatically or mistake its name for usability.
"""

import argparse

import psycopg
from psycopg import sql
from psycopg.rows import dict_row

from zpnet.shared.db import PG_DSN, PG_CONNECT_TIMEOUT_S


# B-tree prefixes match equality filters followed by the ORDER BY id endpoint.
# The first serves campaign-list/read-report queries and foreign-key lookups;
# the second serves instrument-wide recovery/report endpoints.
INDEXES = (
    ("campaign_detail_campaign_id_lookup", ("campaign_type", "campaign", "id")),
    ("campaign_detail_type_id_lookup", ("campaign_type", "id")),
)


def catalog(conn):
    return conn.execute(
        """
        SELECT c.relname AS name, i.indisvalid AND i.indisready AS usable,
               am.amname AS method,
               i.indpred IS NULL AND i.indexprs IS NULL AS plain,
               ARRAY(
                   SELECT a.attname::text
                   FROM unnest(i.indkey::smallint[]) WITH ORDINALITY AS k(attnum, n)
                   JOIN pg_attribute AS a ON a.attrelid = i.indrelid AND a.attnum = k.attnum
                   WHERE k.n <= i.indnkeyatts ORDER BY k.n
               ) AS keys,
               pg_get_indexdef(i.indexrelid) AS definition
        FROM pg_index AS i
        JOIN pg_class AS c ON c.oid = i.indexrelid
        JOIN pg_am AS am ON am.oid = c.relam
        WHERE i.indrelid = 'campaign_detail'::regclass
        ORDER BY c.relname
        """
    ).fetchall()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='build missing indexes concurrently')
    args = parser.parse_args(argv)
    # CONCURRENTLY must run outside a transaction. A dedicated connection
    # also keeps its maintenance timeouts independent of acquisition workers.
    with psycopg.connect(PG_DSN, autocommit=True, row_factory=dict_row,
                         connect_timeout=PG_CONNECT_TIMEOUT_S,
                         application_name='ZPNET:photons_backlog_indexes') as conn:
        if args.apply and not conn.execute(
            "SELECT pg_try_advisory_lock(1346916178, 1229215827) AS acquired"
        ).fetchone()['acquired']:
            raise RuntimeError('Another PHOTONS index-maintenance session is running')
        conn.execute("SET lock_timeout = '2s'")
        conn.execute("SET statement_timeout = '0'")
        conn.execute("SET maintenance_work_mem = '16MB'")
        conn.execute("SET max_parallel_maintenance_workers = 0")
        existing = catalog(conn)
        for row in existing:
            print(('READY' if row['usable'] else 'INVALID/NOT READY') + ': ' + row['definition'], flush=True)
        for name, keys in INDEXES:
            matches = [row for row in existing if row['usable'] and row['plain']
                       and row['method'] == 'btree' and tuple(row['keys'][:len(keys)]) == keys]
            if matches:
                print(f"Covered {keys}: {matches[0]['name']}", flush=True)
                continue
            if any(row['name'] == name for row in existing):
                raise RuntimeError(f'{name} exists but does not supply the required usable index; inspect it before retrying')
            statement = sql.SQL('CREATE INDEX CONCURRENTLY {} ON campaign_detail ({})').format(
                sql.Identifier(name), sql.SQL(', ').join(map(sql.Identifier, keys)))
            print(statement.as_string(conn), flush=True)
            if args.apply:
                conn.execute(statement)
                existing = catalog(conn)
                if not any(row['name'] == name and row['usable'] for row in existing):
                    raise RuntimeError(f'{name} did not become usable')
                print(f'Ready: {name}', flush=True)
        if not args.apply:
            print('Inspection only. Run with --apply to build missing indexes.', flush=True)


if __name__ == '__main__':
    main()
