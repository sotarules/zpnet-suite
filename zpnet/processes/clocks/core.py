"""
ZPNet Clocks Handler — Monitor and persist clock stream updates

Responsibilities:
  • Respond to CLOCK, GNSS, and PPS topic messages
  • Capture GNSS-provided UTC date/time as local state
  • Annotate raw PPS events with most recent GNSS time
  • Republish annotated PPS as TIMELOCK

Process model:
  • One systemd service

Semantics:
  • No internal defensiveness
  • One fault barrier per execution context
  • Never fabricates temporal continuity
"""

from __future__ import annotations

import logging
from typing import Dict, Any, Optional

from zpnet.processes.processes import server_setup, publish
from zpnet.shared.constants import Payload
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_last_pps_payload: Optional[Dict[str, Any]] = None

# ---------------------------------------------------------------------
# Handlers
# ---------------------------------------------------------------------

def on_gnss(payload: Payload) -> None:
    global _last_pps_payload
    date = payload["date"]
    time = payload["time"]
    timelock = {
        "utc": f"{date}T{time}Z",
        "date": date,
        "time": time,
        **_last_pps_payload
    }
    publish("TIMELOCK", timelock)

def on_pps(payload: Payload) -> None:
    global _last_pps_payload
    _last_pps_payload = payload


# ---------------------------------------------------------------------
# Subscriptions
# ---------------------------------------------------------------------

SUBSCRIPTIONS = {
    "GNSS": on_gnss,
    "PPS": on_pps,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    try:
        server_setup(
            subsystem="CLOCKS",
            subscriptions=SUBSCRIPTIONS,
        )
    except Exception:
        logging.exception("💥 [clocks] unhandled exception in main thread")


if __name__ == "__main__":
    run()
