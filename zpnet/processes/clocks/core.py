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

from datetime import datetime
import logging
import time
from typing import Dict, Any, Optional

from zpnet.processes.processes import server_setup, publish, send_command
from zpnet.shared.constants import Payload
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_last_pps_payload: Optional[Dict[str, Any]] = None

# ---------------------------------------------------------------------
# Handlers
# ---------------------------------------------------------------------

def on_pps(payload: Payload) -> None:
    global _last_pps_payload

    _last_pps_payload = payload
    gnss_payload = send_command(machine="PI", subsystem="GNSS", command="REPORT")["payload"]
    system_time = datetime.now().isoformat(timespec='milliseconds')
    gnss_payload_date = gnss_payload["date"]
    gnss_payload_time = gnss_payload["time"]

    timelock = {
        "utc": f"{gnss_payload_date}T{gnss_payload_time}Z",
        "date": gnss_payload_date,
        "time": gnss_payload_time,
        "system_time": system_time,
        **_last_pps_payload
    }
    publish("TIMELOCK", timelock)


# ---------------------------------------------------------------------
# Subscriptions
# ---------------------------------------------------------------------

SUBSCRIPTIONS = {
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
