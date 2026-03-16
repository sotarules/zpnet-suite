"""
ZPNet EVENTS Process — Durable Ingress + Outbound Despooler

This service is the single authoritative egress bridge between:

  • Local PostgreSQL stores (zpnet_events, timebase)
  • The remote ZPNet server

It is also the ingress authority for Teensy-originated events.

Responsibilities:
  • Subscribe to EVENTS and persist them locally (append-only)
  • Periodically despool unsent events to the ZPNet backend
  • Periodically despool unsent timebase records to the ZPNet backend
  • Mark records as despooled only upon confirmed delivery

Note on TIMEBASE ingress:
  • TIMEBASE records are persisted by the CLOCKS process, not here.
  • CLOCKS owns persistence because the INSERT and the campaign
    report UPDATE share transactional context.
  • This process owns only TIMEBASE egress (despooling to server).

Process model:
  • One systemd service
  • Three execution contexts:
      1) Pub/Sub handler thread (EVENTS topic)
      2) Events despooler loop thread (HTTP egress)
      3) Timebase despooler loop thread (HTTP egress)

Semantics:
  • No internal recovery or inference
  • One fault barrier per execution context
  • Failures are logged, never fabricated into success
  • Database is the source of truth for durability
  • Server unavailability is a normal operating state, not an error

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

DESPOOL_BATCH_SIZE = 50          # records per HTTP POST
DESPOOL_INTERVAL_S = 5.0         # polling interval when server is reachable
DESPOOL_BACKOFF_S = 60.0         # polling interval when server is unreachable


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
    "EVENTS": on_events,
}


# ---------------------------------------------------------------------
# Events despooler helpers (EGRESS)
# ---------------------------------------------------------------------

def _serialize_event(row: dict) -> dict:
    """
    Convert an events DB row into JSON-safe wire format.
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


def _fetch_undespooled_events(limit: int) -> List[dict]:
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


def _mark_events_despooled(ids: List[int]) -> None:
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
# Timebase despooler helpers (EGRESS)
# ---------------------------------------------------------------------

def _serialize_timebase(row: dict) -> dict:
    """
    Convert a timebase DB row into JSON-safe wire format.
    """
    return {
        "id": row["id"],
        "ts": (
            row["ts"].isoformat().replace("+00:00", "Z")
            if isinstance(row.get("ts"), datetime)
            else row.get("ts")
        ),
        "campaign": row["campaign"],
        "payload": row["payload"],
    }


def _fetch_undespooled_timebase(limit: int) -> List[dict]:
    """
    Fetch undelivered timebase records from PostgreSQL.

    Truth source:
      • despooled IS NULL
      • ordered oldest → newest
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, campaign, payload
            FROM timebase
            WHERE despooled IS NULL
            ORDER BY ts ASC
            LIMIT %s
            """,
            (limit,),
        )
        rows = cur.fetchall()

    return [_serialize_timebase(row) for row in rows]


def _mark_timebase_despooled(ids: List[int]) -> None:
    """
    Mark timebase records as successfully despooled.
    """
    if not ids:
        return

    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    with open_db() as conn:
        cur = conn.cursor()
        cur.executemany(
            """
            UPDATE timebase
            SET despooled = %s
            WHERE id = %s
            """,
            [(ts, record_id) for record_id in ids],
        )


# ---------------------------------------------------------------------
# Events despooler execution context (OWN THREAD)
# ---------------------------------------------------------------------

def events_despooler_loop() -> None:
    """
    Periodically POST undelivered events to the ZPNet backend.

    Semantics:
      • Best-effort, patient
      • No retries inside the loop
      • Network failure leaves events untouched in the DB
      • Success marks events permanently despooled
      • Server unavailability is boring — back off and wait quietly
    """
    endpoint = f"http://{ZPNET_REMOTE_HOST}/api/events"
    server_down = False

    while True:
        try:
            events = _fetch_undespooled_events(DESPOOL_BATCH_SIZE)
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
            _mark_events_despooled(ids)

            if server_down:
                logging.info("📡 [events] server reachable — despool resumed")
                server_down = False

        except requests.RequestException:
            if not server_down:
                logging.info("📡 [events] server unreachable — backing off")
                server_down = True
            time.sleep(DESPOOL_BACKOFF_S)
            continue

        except Exception:
            logging.exception(
                "💥 [events] unhandled exception in events despooler thread"
            )

        time.sleep(DESPOOL_INTERVAL_S)


# ---------------------------------------------------------------------
# Timebase despooler execution context (OWN THREAD)
# ---------------------------------------------------------------------

def timebase_despooler_loop() -> None:
    """
    Periodically POST undelivered timebase records to the ZPNet backend.

    Semantics:
      • Symmetric with events despooler
      • Best-effort, patient
      • No retries inside the loop
      • Network failure leaves records untouched in the DB
      • Success marks records permanently despooled
      • Server unavailability is boring — back off and wait quietly
    """
    endpoint = f"http://{ZPNET_REMOTE_HOST}/api/timebase"
    server_down = False

    while True:
        try:
            records = _fetch_undespooled_timebase(DESPOOL_BATCH_SIZE)
            if not records:
                time.sleep(DESPOOL_INTERVAL_S)
                continue

            body, headers = gzip_json(records)

            response = requests.post(
                endpoint,
                data=body,
                headers=headers,
                timeout=HTTP_TIMEOUT,
            )

            if response.status_code != 200:
                raise RuntimeError(
                    f"[timebase] despool HTTP {response.status_code}: {response.text}"
                )

            ids = [r["id"] for r in records]
            _mark_timebase_despooled(ids)

            if server_down:
                logging.info("📡 [timebase] server reachable — despool resumed")
                server_down = False

        except requests.RequestException:
            if not server_down:
                logging.info("📡 [timebase] server unreachable — backing off")
                server_down = True
            time.sleep(DESPOOL_BACKOFF_S)
            continue

        except Exception:
            logging.exception(
                "💥 [timebase] unhandled exception in timebase despooler thread"
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
      • Events despooler loop thread
      • Timebase despooler loop thread
    """
    setup_logging()

    # --------------------------------------------------------------
    # Start events despooler thread (independent fault barrier)
    # --------------------------------------------------------------
    threading.Thread(
        target=events_despooler_loop,
        daemon=True,
        name="events-despooler",
    ).start()

    # --------------------------------------------------------------
    # Start timebase despooler thread (independent fault barrier)
    # --------------------------------------------------------------
    threading.Thread(
        target=timebase_despooler_loop,
        daemon=True,
        name="timebase-despooler",
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