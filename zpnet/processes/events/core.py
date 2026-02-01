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
import threading
import time
from typing import Dict, Optional

from zpnet.processes.processes import send_command, server_setup, publish
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

def setup_events() -> None:
    payload: Payload = {}
    payload["subsystem"] = "EVENTS"
    payload["topics"] = ["EVENTS"]
    publish("SUBSCRIBE", payload)

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

# ---------------------------------------------------------------------
# Publish surface
# ---------------------------------------------------------------------

def on_message(topic: str, payload: Payload) -> None:
    logging.info("🚀 [event_despooler] received message on topic %s: %s", topic, payload)
    event_despooler(payload)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    setup_events()
    try:

        #threading.Thread(
        #    target=despooler_loop,
        #    daemon=True,
        #).start()

        server_setup(
            subsystem="EVENTS",
            commands=COMMANDS,
            subscriptions=["EVENTS"],
            on_message=on_message,
        )

    except Exception:
        import logging
        logging.exception("💥 [event_despooler] unhandled exception in main thread")


if __name__ == "__main__":
    run()
