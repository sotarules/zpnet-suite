"""
ZPNet Event Despooler  —  Stellar-Compliant + Timeout-Hardened Revision
(v2025-12-29-gzip)

Fetches unsent events from PostgreSQL and POSTs them to the remote ZPNet endpoint
using gzip-compressed JSON payloads. Marks events as despooled upon success.

Author: The Mule + GPT
"""

import logging
from datetime import datetime, timezone

import requests

from zpnet.shared.constants import (
    ZPNET_REMOTE_HOST,
    HTTP_TIMEOUT,
)
from zpnet.shared.db import open_db
from zpnet.shared.http import gzip_json


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
BATCH_SIZE = 50  # max events per batch (count-based; bytes handled by gzip)


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def serialize_event(row: dict) -> dict:
    """
    Convert a DB row into JSON-safe wire format.
    """
    return {
        "id": row["id"],
        "ts": (
            row["ts"].isoformat().replace("+00:00", "Z")
            if isinstance(row.get("ts"), datetime)
            else row.get("ts")
        ),
        "event_type": row["event_type"],
        "payload": row["payload"],
    }


# ---------------------------------------------------------------------
# Database Operations
# ---------------------------------------------------------------------
def fetch_unsent_events(limit: int) -> list[dict]:
    """Return up to `limit` unsent events from zpnet_events."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, event_type, payload
            FROM zpnet_events
            WHERE despooled IS NULL
            ORDER BY ts ASC
            LIMIT %s
            """,
            (limit,),
        )
        rows = cur.fetchall()

    # Explicitly serialize rows for JSON transport
    return [serialize_event(row) for row in rows]


def mark_despooled(ids: list[int]) -> None:
    """Mark the given event IDs as successfully despooled."""
    if not ids:
        return

    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    with open_db() as conn:
        cur = conn.cursor()
        cur.executemany(
            """
            UPDATE zpnet_events
            SET despooled = %s
            WHERE id = %s
            """,
            [(ts, event_id) for event_id in ids],
        )


# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    POST a batch of unsent events to the remote endpoint using gzip-compressed JSON.
    """
    try:
        endpoint = f"http://{ZPNET_REMOTE_HOST}/api"
        events = fetch_unsent_events(BATCH_SIZE)

        if not events:
            return

        body, headers = gzip_json(events)

        response = requests.post(
            endpoint,
            data=body,
            headers=headers,
            timeout=HTTP_TIMEOUT,
        )

        if response.status_code != 200:
            raise RuntimeError(
                f"[event_despooler] HTTP {response.status_code}: {response.text}"
            )

        ids = [e["id"] for e in events]
        mark_despooled(ids)

    except requests.RequestException as e:
        logging.warning(
            f"📡 [event_despooler] network issue: {e} — will retry later"
        )

    except Exception as e:
        logging.exception(
            f"💥 [event_despooler] unexpected failure: {e}"
        )


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    from zpnet.shared.logger import setup_logging
    setup_logging()
    run()
