"""
ZPNet Shared Event Utilities  —  Stellar-Compliant Revision

Provides helper functions for inserting events into the zpnet_events table.
All timestamps are timezone-aware (UTC).
Database access is centralized through zpnet.shared.db to ensure
consistent WAL usage, busy-timeout behavior, and concurrency policy.

Author: The Mule
"""

import json
import logging
import time
from datetime import datetime, timezone

import sqlite3

from zpnet.shared.db import open_db


# ---------------------------------------------------------------------
# Event insertion
# ---------------------------------------------------------------------
def create_event(
    event_type: str,
    payload: dict | None = None,
    retries: int = 5,
    delay: float = 0.1,
) -> None:
    """
    Insert a single event into the zpnet_events table.

    Args:
        event_type (str): Event name (uppercase identifier).
        payload (dict | None): JSON-serializable payload.
        retries (int): Number of retries on SQLITE_BUSY / locked.
        delay (float): Delay (seconds) between retries.

    Raises:
        sqlite3.OperationalError: on unrecoverable database errors.
    """
    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    payload_json = json.dumps(payload or {}, separators=(",", ":"))

    for attempt in range(1, retries + 1):
        try:
            with open_db(read_only=False) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    INSERT INTO zpnet_events (timestamp, event_type, payload)
                    VALUES (?, ?, ?)
                    """,
                    (ts, event_type, payload_json),
                )
                conn.commit()
            return

        except sqlite3.OperationalError as e:
            # Expected under contention — writers wait politely
            if "locked" in str(e).lower():
                time.sleep(delay)
                continue
            raise

    # If still failing after retries, log explicitly
    logging.error(
        f"⚠️ [events] DB locked after {retries} attempts for event {event_type}"
    )
