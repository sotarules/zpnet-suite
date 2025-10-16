"""
ZPNet Event Despooler

Fetches unsent events from SQLite and POSTs them to the remote endpoint.
Intended to be scheduled by the main scheduler loop.
"""

import json
import logging
import sqlite3
from datetime import datetime
from pathlib import Path

import requests

from zpnet.shared.logger import setup_logging
from zpnet.shared.recovery import invoke_choosenet

# --------------------------
# Configuration
# --------------------------
DB_PATH = Path("/home/mule/zpnet/zpnet.db")
ENV_PATH = Path("/etc/zpnet.env")
BATCH_SIZE = 50       # max events per batch
TIMEOUT = 5           # HTTP POST timeout (seconds)

# --------------------------
# Core database operations
# --------------------------

def fetch_unsent_events(limit: int):
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

def mark_despooled(ids):
    now = datetime.utcnow().isoformat()
    with sqlite3.connect(DB_PATH) as conn:
        cur = conn.cursor()
        cur.executemany(
            """
            UPDATE zpnet_events
            SET despooled = ?
            WHERE id = ?
            """,
            [(now, event_id) for event_id in ids],
        )
        conn.commit()

def zpnet_host_from_env():
    with open(ENV_PATH, "r") as f:
        for line in f:
            if line.startswith("ZPNET_REMOTE_HOST="):
                return line.strip().split("=", 1)[1]
    raise RuntimeError("ZPNET_REMOTE_HOST not found in env")

# --------------------------
# Main execution routine
# --------------------------

def run():
    """
    Attempt to POST a batch of unsent events to the remote endpoint.
    """
    try:

        host = zpnet_host_from_env()
        endpoint = f"http://{host}/api"

        events = fetch_unsent_events(BATCH_SIZE)

        if not events:
            return

        response = requests.post(
            endpoint,
            headers={"Content-Type": "application/json"},
            data=json.dumps(events),
            timeout=TIMEOUT,
        )

        if response.status_code != 200:
            raise RuntimeError(f"[event_despooler] unexpected HTTP response: {response.status_code} {response.text}")

        ids = [e["id"] for e in events]
        mark_despooled(ids)

    except requests.exceptions.RequestException as e:
        logging.warning("📡 [event_despooler] remote host unreachable; triggering network recovery")
        invoke_choosenet()

    except Exception as e:
        logging.warning(f"❌ [event_despooler] unexpected: {e}")

# --------------------------
# Optional entrypoint
# --------------------------

def bootstrap():
    setup_logging()
    run()
