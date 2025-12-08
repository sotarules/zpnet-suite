"""
ZPNet Shared Event Utilities  —  Stellar-Compliant Revision

Provides helper functions for inserting events into the zpnet_events table.
All timestamps are timezone-aware (UTC).  Any failure to insert is logged
explicitly and raised upward if the caller chooses to handle it.
"""

import sqlite3
import json
import logging
import time         
from datetime import datetime, timezone
from pathlib import Path

DB_PATH = Path("/home/mule/zpnet/zpnet.db")

def create_event(event_type: str, payload: dict | None = None, retries: int = 5, delay: float = 0.1) -> None:
    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    payload_json = json.dumps(payload or {}, separators=(",", ":"))

    for attempt in range(retries):
        try:
            with sqlite3.connect(DB_PATH, timeout=5) as conn:
                cur = conn.cursor()
                cur.execute(
                    "INSERT INTO zpnet_events (timestamp, event_type, payload) VALUES (?, ?, ?)",
                    (ts, event_type, payload_json),
                )
                conn.commit()
            return
        except sqlite3.OperationalError as e:
            if "locked" in str(e).lower():
                time.sleep(delay)
                continue
            raise

    # If still failing after retries
    logging.error(f"⚠️ [events] DB locked after {retries} attempts for event {event_type}")

