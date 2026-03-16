"""
ZPNet EVENTS Process — Durable Event Ingress + Outbound Despooler

This service is the single authoritative bridge between:

  • The Teensy-originated EVENTS topic (durable ingress)
  • The local PostgreSQL event store
  • The remote ZPNet server (durable egress)

Responsibilities:
  • Subscribe to EVENTS and persist them locally (append-only)
  • Periodically despool unsent events to the ZPNet backend
  • Mark events as despooled only upon confirmed delivery

Process model:
  • One systemd service
  • Two execution contexts:
      1) Pub/Sub handler thread (EVENTS topic)
      2) Despooler loop thread (HTTP egress)

Semantics:
  • No internal recovery or inference
  • One fault barrier per execution context
  • Failures are logged, never fabricated into success
  • Database is the source of truth for event durability

This file intentionally replaces the legacy event_despooler module
and paves the way for its deprecation.

Author: The Mule + GPT
"""

from __future__ import annotations

import logging
import threading
import time
from datetime import datetime, timezone
from typing import List

import requests

from zpnet.processes.processes import server_setup
from zpnet.shared.constants import (
    Payload,
    ZPNET_REMOTE_HOST,
    HTTP_TIMEOUT,
)
from zpnet.shared.db import open_db
from zpnet.shared.events import create_event
from zpnet.shared.http import gzip_json
from zpnet.shared.logger import setup_logging


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

DESPOOL_BATCH_SIZE = 50          # events per HTTP POST
DESPOOL_INTERVAL_S = 5.0         # polling interval for unsent events


# ---------------------------------------------------------------------
# EVENTS topic handler (INGRESS)
# ---------------------------------------------------------------------

def on_events(payload: Payload) -> None:
    """
    Handle EVENTS topic payload from Teensy.

    Payload shape (authoritative):
        {
            "events": [
                { "event_type": "...", "payload": {...} },
                ...
            ]
        }

    Semantics:
      • Each item is persisted verbatim
      • No retries
      • No inference
      • Any failure is fatal to this execution context
    """
    for item in payload["events"]:
        create_event(item["event_type"], item.get("payload"))


SUBSCRIPTIONS = {
    "EVENTS": on_events
}


# ---------------------------------------------------------------------
# Despooler helpers (EGRESS)
# ---------------------------------------------------------------------

def _serialize_event(row: dict) -> dict:
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


def _fetch_undispooled(limit: int) -> List[dict]:
    """
    Fetch undelivered events from PostgreSQL.

    Truth source:
      • despooled IS NULL
      • ordered oldest → newest
    """
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

    return [_serialize_event(row) for row in rows]


def _mark_despooled(ids: List[int]) -> None:
    """
    Mark events as successfully despooled.
    """
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
# Despooler execution context (OWN THREAD)
# ---------------------------------------------------------------------

def despooler_loop() -> None:
    """
    Periodically POST undelivered events to the ZPNet backend.

    Semantics:
      • Best-effort
      • No retries inside the loop
      • Network failure leaves events untouched
      • Success marks events permanently despooled
    """
    endpoint = f"http://{ZPNET_REMOTE_HOST}/api/events"

    while True:
        try:
            events = _fetch_undispooled(DESPOOL_BATCH_SIZE)
            if not events:
                time.sleep(DESPOOL_INTERVAL_S)
                continue

            body, headers = gzip_json(events)

            response = requests.post(
                endpoint,
                data=body,
                headers=headers,
                timeout=HTTP_TIMEOUT,
            )

            if response.status_code != 200:
                raise RuntimeError(
                    f"[events] despool HTTP {response.status_code}: {response.text}"
                )

            ids = [e["id"] for e in events]
            _mark_despooled(ids)

        except requests.RequestException as e:
            logging.warning(
                f"📡 [events] despool network issue: {e} — will retry"
            )

        except Exception:
            logging.exception(
                "💥 [events] unhandled exception in despooler thread"
            )

        time.sleep(DESPOOL_INTERVAL_S)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    """
    Start the EVENTS process.

    Execution contexts:
      • Pub/Sub handler via server_setup (EVENTS topic)
      • Despooler loop thread
    """
    setup_logging()

    # --------------------------------------------------------------
    # Start despooler thread (independent fault barrier)
    # --------------------------------------------------------------
    threading.Thread(
        target=despooler_loop,
        daemon=True,
        name="events-despooler",
    ).start()

    # --------------------------------------------------------------
    # Start pub/sub server (blocks forever)
    # --------------------------------------------------------------
    try:
        server_setup(
            subsystem="EVENTS",
            subscriptions=SUBSCRIPTIONS,
        )
    except Exception:
        logging.exception("💥 [events] unhandled exception in main thread")


if __name__ == "__main__":
    run()
