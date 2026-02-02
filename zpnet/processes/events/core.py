"""
ZPNet Event Despooler (Pi-side, authoritative)

Responsibilities:
  • Periodically drain the Teensy durable event queue
  • Forward each event into the local ZPNet event store

Process model:
  • One systemd service
  • One polling thread (own execution context)
  • One blocking command socket (REPORT)

Semantics:
  • Uses unified send_command() interface
  • No internal defensiveness
  • One fault barrier per execution context
"""

from __future__ import annotations

import logging
import time
from typing import Dict, Optional

from zpnet.processes.processes import server_setup, publish
from zpnet.shared.constants import Payload
from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging

POLL_INTERVAL_S = 5


# ---------------------------------------------------------------------
# Internal state
# ---------------------------------------------------------------------

_last_drain_ts: float = 0.0
_last_count: int = 0

# ---------------------------------------------------------------------
# Polling thread (fault barrier)
# ---------------------------------------------------------------------

def event_despooler(payload: Payload) -> None:
    import logging
    global _last_drain_ts, _last_count

    try:
        for item in payload["events"]:
            create_event(item["event_type"], item.get("payload"))
        count = len(payload)
        _last_count = count
        _last_drain_ts = time.time()

    except Exception:
        logging.exception(
            "💥 [event_despooler] unhandled exception — despooler thread terminating"
        )

# ---------------------------------------------------------------------
# Publish surface
# ---------------------------------------------------------------------

def on_events(payload: Payload) -> None:
    logging.info("🚀 [event_despooler] received events: %s", payload)
    event_despooler(payload)

# ---------------------------------------------------------------------
# Command surface
# ---------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "poll_interval_s": POLL_INTERVAL_S,
            "last_drain_ts": _last_drain_ts,
            "last_count": _last_count,
        },
    }

def cmd_subscribe(_: Optional[dict]) -> Dict:
    payload: Payload = {}
    payload["subsystem"] = "EVENTS"
    payload["topics"] = ["GNSS_NEWS_FEED"]
    publish("SUBSCRIBE", payload)
    return {
        "success": True,
        "message": "OK",
        "payload": { "status": "NOMINAL" }
    }

def cmd_publish(_: Optional[dict]) -> Dict:
    payload: Payload = {}
    payload["alpha"] = "Events"
    payload["beta"] = "News"
    payload["gamma"] = "Feed"
    publish("EVENTS_NEWS_FEED", payload)
    return {
        "success": True,
        "message": "OK",
        "payload": { "status": "NOMINAL" }
    }


COMMANDS = {
    "REPORT": cmd_report,
    "PUBLISH": cmd_publish,
    "SUBSCRIBE": cmd_subscribe
}

SUBSCRIPTIONS = {
    "EVENTS": on_events
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    try:
        server_setup(
            subsystem="EVENTS",
            commands=COMMANDS,
            subscriptions=SUBSCRIPTIONS
        )
    except Exception:
        logging.exception("💥 [event_despooler] unhandled exception in main thread")

if __name__ == "__main__":
    run()
