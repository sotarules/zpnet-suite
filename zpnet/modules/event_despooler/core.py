"""
ZPNet Event Despooler

Fetches unsent events from SQLite and POSTs them to the remote endpoint.
Intended to be scheduled by the main scheduler loop.
"""

import sqlite3
import requests
import json
import logging
from datetime import datetime
from pathlib import Path

from zpnet.shared.logger import setup_logging


# --------------------------
# Configuration
# --------------------------
DB_PATH = Path("/home/mule/zpnet/zpnet.db")
ENV_PATH = Path("/etc/zpnet.env")
BATCH_SIZE = 50    # max events per batch
TIMEOUT = 5        # seconds for HTTP POST timeout

# Default host
ZPNET_REMOTE_HOST = "localhost"

# Attempt to load from env file
try:
    with open(ENV_PATH, "r") as f:
        for line in f:
            if line.startswith("ZPNET_REMOTE_HOST="):
                ZPNET_REMOTE_HOST = line.strip().split("=", 1)[1]
except Exception as e:
    logging.warning(f"Could not read {ENV_PATH}: {e}")

ENDPOINT = f"http://{ZPNET_REMOTE_HOST}/api"
logging.info(f"📡 Despooling to endpoint: {ENDPOINT}")


# --------------------------
# Database operations
# --------------------------
def fetch_unsent_events(limit: int):
    """Fetch up to `limit` unsent events (despooled IS NULL)."""
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
    """Mark events as successfully despooled with current UTC timestamp."""
    if not ids:
        return
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


# --------------------------
# Main task
# --------------------------
def run():
    """Fetch a batch of events and POST them to the API."""
    events = fetch_unsent_events(BATCH_SIZE)
    if not events:
        return

    try:
        response = requests.post(
            ENDPOINT,
            headers={"Content-Type": "application/json"},
            data=json.dumps(events),
            timeout=TIMEOUT,
        )
        if response.status_code == 200:
            ids = [e["id"] for e in events]
            mark_despooled(ids)
            logging.debug(f"✅ Despooled {len(ids)} events")
        else:
            logging.warning(f"⚠️ API returned {response.status_code}: {response.text}")

    except Exception:
        logging.exception("❌ Despooler POST failed")


# --------------------------
# Bootstrap for standalone use
# --------------------------
def bootstrap():
    """Setup logging and run once (for debugging)."""
    setup_logging()
    run()
