"""
ZPNet EVENTS Process — Durable Ingress + Outbound Despooler

This service is the single authoritative egress bridge between:

  • Local PostgreSQL stores (zpnet_events, timebase, witness)
  • The remote ZPNet server

It is also the ingress authority for Teensy-originated event streams.

Responsibilities:
  • Subscribe to EVENTS and persist them locally (append-only)
  • Subscribe to CLOCK_WITNESS and persist it locally in witness (append-only)
  • Periodically despool unsent events to the ZPNet backend
  • Periodically despool unsent timebase records to the ZPNet backend
  • Mark records as despooled only upon confirmed delivery

Note on TIMEBASE ingress:
  • TIMEBASE records are persisted by the CLOCKS process, not here.
  • CLOCKS owns persistence because the INSERT and the campaign
    report UPDATE share transactional context.
  • This process owns only TIMEBASE egress (despooling to server).

Note on CLOCK_WITNESS ingress:
  • CLOCK_WITNESS is persisted here because it is a durable observation stream,
    not a temporary live-report subscription.
  • The payload is stored verbatim as JSONB in the witness table.
  • The witness table intentionally mirrors timebase shape:
      id, ts, campaign, payload, despooled.
  • Remote witness despooling is not enabled here yet; despooled remains NULL
    until a future remote witness endpoint is intentionally added.

Process model:
  • One systemd service
  • Three execution contexts:
      1) Pub/Sub handler thread (EVENTS and CLOCK_WITNESS topics)
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

import json
import logging
import threading
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

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

CLOCK_WITNESS_TOPIC = "CLOCK_WITNESS"
WITNESS_CAMPAIGN_FALLBACK = "witness"
WITNESS_CAMPAIGN_CACHE_TTL_S = 5.0
WITNESS_LOG_EVERY_N = 100


# ---------------------------------------------------------------------
# Local diagnostics
# ---------------------------------------------------------------------

_diag_lock = threading.Lock()
_witness_diag: Dict[str, Any] = {
    "witness_received": 0,
    "witness_inserted": 0,
    "witness_insert_failures": 0,
    "witness_campaign_from_payload": 0,
    "witness_campaign_from_active_campaign": 0,
    "witness_campaign_from_fallback": 0,
    "witness_last_campaign": None,
    "witness_last_insert_id": None,
    "witness_last_insert_ts_utc": None,
    "witness_last_payload_bytes": 0,
    "witness_last_error": None,
}

_active_campaign_cache: Optional[str] = None
_active_campaign_cache_expires_at = 0.0
_active_campaign_cache_error_logged = False


def _system_time_z() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _json_bytes_len(obj: Any) -> int:
    try:
        return len(json.dumps(obj, separators=(",", ":"), default=str).encode("utf-8"))
    except Exception:
        return 0


def _diag_update(**fields: Any) -> None:
    with _diag_lock:
        _witness_diag.update(fields)


def _diag_incr(key: str, amount: int = 1) -> int:
    with _diag_lock:
        value = int(_witness_diag.get(key, 0)) + amount
        _witness_diag[key] = value
        return value


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


# ---------------------------------------------------------------------
# CLOCK_WITNESS topic handler (INGRESS)
# ---------------------------------------------------------------------

def _campaign_from_payload(payload: Any) -> Optional[str]:
    """
    Return campaign identity embedded in a CLOCK_WITNESS payload, if present.

    The current firmware witness payload is mostly hardware state and may not
    carry campaign identity.  This helper is intentionally observational: it
    accepts a few obvious field names and otherwise returns None.
    """
    if not isinstance(payload, dict):
        return None

    for key in ("campaign", "campaign_name", "campaign_id"):
        value = payload.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()

    state = payload.get("state")
    if isinstance(state, dict):
        for key in ("campaign", "campaign_name", "campaign_id"):
            value = state.get(key)
            if isinstance(value, str) and value.strip():
                return value.strip()

    return None


def _active_campaign_name() -> Optional[str]:
    """
    Return the active CLOCKS campaign name, if one exists.

    EVENTS does not own campaign lifecycle.  This is only a best-effort label
    so witness rows keep the same structural shape as timebase rows.
    """
    global _active_campaign_cache, _active_campaign_cache_expires_at
    global _active_campaign_cache_error_logged

    now = time.monotonic()
    if now < _active_campaign_cache_expires_at:
        return _active_campaign_cache

    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT campaign
                FROM campaigns
                WHERE active = true
                ORDER BY ts DESC
                LIMIT 1
                """
            )
            row = cur.fetchone()

        _active_campaign_cache = (
            str(row["campaign"]).strip()
            if row and row.get("campaign") is not None and str(row["campaign"]).strip()
            else None
        )
        _active_campaign_cache_expires_at = now + WITNESS_CAMPAIGN_CACHE_TTL_S
        _active_campaign_cache_error_logged = False
        return _active_campaign_cache

    except Exception as e:
        _active_campaign_cache = None
        _active_campaign_cache_expires_at = now + WITNESS_CAMPAIGN_CACHE_TTL_S
        if not _active_campaign_cache_error_logged:
            logging.warning(
                "⚠️ [witness] active campaign lookup failed — using fallback campaign '%s': %s",
                WITNESS_CAMPAIGN_FALLBACK,
                e,
            )
            _active_campaign_cache_error_logged = True
        return None


def _resolve_witness_campaign(payload: Any) -> str:
    campaign = _campaign_from_payload(payload)
    if campaign:
        _diag_incr("witness_campaign_from_payload")
        return campaign

    campaign = _active_campaign_name()
    if campaign:
        _diag_incr("witness_campaign_from_active_campaign")
        return campaign

    _diag_incr("witness_campaign_from_fallback")
    return WITNESS_CAMPAIGN_FALLBACK


def _insert_witness_row(*, campaign: str, payload: Any) -> int:
    """
    Insert one CLOCK_WITNESS payload into the witness table.

    The payload is stored verbatim as JSONB.  We explicitly JSON-serialize to
    mirror create_event() and avoid driver-specific dict adaptation surprises.
    """
    ts = datetime.now(timezone.utc)
    payload_json = json.dumps(payload if payload is not None else {}, separators=(",", ":"), default=str)

    with open_db() as conn:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO witness (ts, campaign, payload)
                VALUES (%s, %s, %s::jsonb)
                RETURNING id
                """,
                (ts, campaign, payload_json),
            )
            row = cur.fetchone()

    if isinstance(row, dict):
        return int(row["id"])
    if isinstance(row, (tuple, list)):
        return int(row[0])
    return 0


def on_clock_witness(payload: Payload) -> None:
    """
    Handle CLOCK_WITNESS topic payload from Teensy.

    Semantics:
      • Persist the payload verbatim into witness.payload
      • Attach a best-effort campaign label for timebase-shaped querying
      • No routing inference
      • No report formatting
      • Any database failure is logged and allowed to fault this handler context
    """
    received = _diag_incr("witness_received")
    campaign = _resolve_witness_campaign(payload)
    payload_bytes = _json_bytes_len(payload)

    try:
        row_id = _insert_witness_row(campaign=campaign, payload=payload)
        inserted = _diag_incr("witness_inserted")
        _diag_update(
            witness_last_campaign=campaign,
            witness_last_insert_id=row_id,
            witness_last_insert_ts_utc=_system_time_z(),
            witness_last_payload_bytes=payload_bytes,
            witness_last_error=None,
        )

        if inserted == 1 or (WITNESS_LOG_EVERY_N > 0 and inserted % WITNESS_LOG_EVERY_N == 0):
            logging.info(
                "🧾 [witness] inserted CLOCK_WITNESS row id=%s campaign=%s received=%d inserted=%d payload_bytes=%d",
                row_id,
                campaign,
                received,
                inserted,
                payload_bytes,
            )

    except Exception as e:
        failures = _diag_incr("witness_insert_failures")
        _diag_update(
            witness_last_campaign=campaign,
            witness_last_payload_bytes=payload_bytes,
            witness_last_error=str(e),
        )
        logging.exception(
            "💥 [witness] failed to insert CLOCK_WITNESS payload campaign=%s received=%d failures=%d payload_bytes=%d",
            campaign,
            received,
            failures,
            payload_bytes,
        )
        raise


SUBSCRIPTIONS = {
    "EVENTS": on_events,
    CLOCK_WITNESS_TOPIC: on_clock_witness,
}


# ---------------------------------------------------------------------
# Command surface
# ---------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    with _diag_lock:
        witness_diag = dict(_witness_diag)

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "subscriptions": sorted(SUBSCRIPTIONS.keys()),
            "witness": witness_diag,
        },
    }


COMMANDS = {
    "REPORT": cmd_report,
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
      • Pub/Sub handler via server_setup (EVENTS and CLOCK_WITNESS topics)
      • Events despooler loop thread
      • Timebase despooler loop thread
    """
    setup_logging()

    logging.info(
        "🚀 [events] subscriptions=%s witness_table_enabled=True witness_remote_despool=False",
        sorted(SUBSCRIPTIONS.keys()),
    )

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
            commands=COMMANDS,
            subscriptions=SUBSCRIPTIONS,
        )
    except Exception:
        logging.exception("💥 [events] unhandled exception in main thread")


if __name__ == "__main__":
    run()
