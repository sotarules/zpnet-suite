"""
ZPNet Shared Event Utilities

Provides helper functions for inserting events into the zpnet_events table.
"""

import sqlite3
import json
import logging
from datetime import datetime
from pathlib import Path

DB_PATH = Path("/home/mule/zpnet/zpnet.db")


def create_event(event_type: str, payload: dict | None = None) -> None:
    """
    Create a new event in the zpnet_events table.

    Args:
        event_type (str): e.g., "POWER_STATUS"
        payload (dict): Additional info as a dict (will be stored as JSON)
    """
    ts = datetime.utcnow().isoformat()
    payload_json = json.dumps(payload or {})

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
    except Exception:
        logging.exception(f"Failed to create event {event_type}")
