"""
ZPNet Shared Event Utilities — PostgreSQL Backend (psycopg v3)

Inserts immutable ZPNet events into the Postgres events table.
Payloads remain opaque JSON strings.

Author: The Mule + GPT
"""

import json
from datetime import datetime, timezone

from zpnet.shared.db import open_db


# ---------------------------------------------------------------------
# Event insertion
# ---------------------------------------------------------------------

def create_event(event_type: str, payload=None):
    """
    Insert a single event into the events table.

    Args:
        event_type (str): Event name (uppercase identifier).
        payload (dict | None): JSON-serializable payload.

    Semantics:
        • Append-only
        • One row per event
        • No retries needed (Postgres handles concurrency)
        • Caller does not receive a return value
    """
    ts = datetime.now(timezone.utc)
    payload_json = json.dumps(payload or {}, separators=(",", ":"))

    with open_db() as conn:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO zpnet_events (ts, event_type, payload)
                VALUES (%s, %s, %s)
                """,
                (ts, event_type, payload_json),
            )
