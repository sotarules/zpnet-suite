"""
ZPNet Event Despooler  —  Stellar-Compliant + Timeout-Hardened Revision (v2025-10-28b)

Fetches unsent events from the local SQLite database and POSTs them to
the remote ZPNet endpoint.  Marks events as despooled upon success.
This revision imports HTTP timeout policy and host configuration from
zpnet.shared.constants to ensure consistent, bounded network behavior.

Author: The Mule
"""

import json
import logging
import sqlite3
from datetime import datetime, timezone

import requests

from zpnet.shared.logger import setup_logging
from zpnet.shared.constants import (
    ZPNET_REMOTE_HOST,
    DB_PATH,
    HTTP_TIMEOUT,
)

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
BATCH_SIZE = 50  # max events per batch

# ---------------------------------------------------------------------
# Database Operations
# ---------------------------------------------------------------------
def fetch_unsent_events(limit: int) -> list[dict]:
    """Return up to `limit` unsent events from zpnet_events."""
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, timestamp, event_type, payload
            FROM zpnet_events
            WHERE despooled IS NULL
            ORDER BY timestamp ASC
            LIMIT ?
            """,
            (limit,),
        )
        return [dict(row) for row in cur.fetchall()]


def mark_despooled(ids: list[int]) -> None:
    """Mark given event IDs as successfully despooled."""
    if not ids:
        return

    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    with sqlite3.connect(DB_PATH) as conn:
        cur = conn.cursor()
        cur.executemany(
            """
            UPDATE zpnet_events
            SET despooled = ?
            WHERE id = ?
            """,
            [(ts, event_id) for event_id in ids],
        )
        conn.commit()


# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Attempt to POST a batch of unsent events to the remote endpoint.
    Each batch POST operation has a strict network timeout
    (connect, read) from shared constants.
    """
    try:
        endpoint = f"http://{ZPNET_REMOTE_HOST}/api"
        events = fetch_unsent_events(BATCH_SIZE)

        if not events:
            return

        response = requests.post(
            endpoint,
            headers={
                "Content-Type": "application/json",
                "Connection": "close",
            },
            data=json.dumps(events, separators=(",", ":")),
            timeout=HTTP_TIMEOUT,
        )

        if response.status_code != 200:
            raise RuntimeError(
                f"[event_despooler] HTTP {response.status_code}: {response.text}"
            )

        ids = [e["id"] for e in events]
        mark_despooled(ids)

    except requests.RequestException as e:
        logging.warning(f"📡 [event_despooler] network issue: {e} — will retry later")

    except Exception as e:
        logging.exception(f"💥 [event_despooler] unexpected failure: {e}")


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
