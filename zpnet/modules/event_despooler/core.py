"""
ZPNet Event Despooler  —  Stellar-Compliant Revision

Fetches unsent events from the local SQLite database and POSTs them to
the remote ZPNet endpoint.  Marks events as despooled upon success.
Triggers network recovery if the host is unreachable.

Author: The Mule
"""

import json
import logging
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

import requests

from zpnet.shared.logger import setup_logging
from zpnet.shared.recovery import invoke_choosenet

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
DB_PATH = Path("/home/mule/zpnet/zpnet.db")
ENV_PATH = Path("/etc/zpnet.env")
BATCH_SIZE = 50           # max events per batch
HTTP_TIMEOUT_S = 5         # POST timeout (seconds)


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


def zpnet_host_from_env() -> str:
    """Read ZPNET_REMOTE_HOST from /etc/zpnet.env."""
    with ENV_PATH.open() as f:
        for line in f:
            if line.startswith("ZPNET_REMOTE_HOST="):
                return line.strip().split("=", 1)[1]
    raise RuntimeError("ZPNET_REMOTE_HOST not found in env")


# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run():
    """
    Attempt to POST a batch of unsent events to the remote endpoint.

    Emits:
        NETWORK recovery if remote host is unreachable.
    """
    try:
        host = zpnet_host_from_env()
        endpoint = f"http://{host}/api"
        events = fetch_unsent_events(BATCH_SIZE)

        if not events:
            return

        logging.info(f"📤 Despooling {len(events)} events → {endpoint}")
        response = requests.post(
            endpoint,
            headers={"Content-Type": "application/json"},
            data=json.dumps(events, separators=(",", ":")),
            timeout=HTTP_TIMEOUT_S,
        )

        if response.status_code != 200:
            raise RuntimeError(
                f"[event_despooler] HTTP {response.status_code}: {response.text}"
            )

        ids = [e["id"] for e in events]
        mark_despooled(ids)
        logging.info(f"✅ Successfully despooled {len(ids)} events")

    except requests.exceptions.RequestException:
        logging.warning("📡 [event_despooler] remote host unreachable; triggering recovery")
        invoke_choosenet()

    except Exception as e:
        logging.exception(f"💥 [event_despooler] unexpected: {e}")


def bootstrap():
    """Setup logging and execute run() once."""
    setup_logging()
    run()
