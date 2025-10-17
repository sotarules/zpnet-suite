"""
ZPNet Shared Event Utilities  —  Stellar-Compliant Revision

Provides helper functions for inserting events into the zpnet_events table.
All timestamps are timezone-aware (UTC).  Any failure to insert is logged
explicitly and raised upward if the caller chooses to handle it.
"""

import sqlite3
import json
import logging
from datetime import datetime, timezone
from pathlib import Path

DB_PATH = Path("/home/mule/zpnet/zpnet.db")


def create_event(event_type: str, payload: dict | None = None) -> None:
    """
    Create a new event in the zpnet_events table.

    Args:
        event_type (str): e.g., "POWER_STATUS" or "TEENSY_STATUS".
        payload (dict | None): Additional info as a dict (stored as JSON).

    Emits:
        A single new row into the zpnet_events table.
    """
    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    payload_json = json.dumps(payload or {}, separators=(",", ":"))

    try:
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO zpnet_events (timestamp, event_type, payload)
                VALUES (?, ?, ?)
                """,
                (ts, event_type, payload_json),
            )
            conn.commit()
        logging.debug(f"🪶 Event created: {event_type}")
    except Exception as e:
        # Fail loudly — no silent data loss
        logging.exception(f"⚠️ Failed to create event {event_type}: {e}")
        raise
