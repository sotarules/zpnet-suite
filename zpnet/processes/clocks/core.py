"""
ZPNet Clocks Handler — Monitor and persist clock stream updates

Responsibilities:
  • Respond to CLOCK topic messages
  • Store most recent clock updates

Process model:
  • One systemd service

Semantics:
  • No internal defensiveness
  • One fault barrier per execution context
"""

from __future__ import annotations

import logging

from zpnet.processes.processes import server_setup
from zpnet.shared.constants import Payload
from zpnet.shared.logger import setup_logging


# ---------------------------------------------------------------------
# Publish surface
# ---------------------------------------------------------------------

def on_clocks(payload: Payload) -> None:
    try:
        logging.info("🚀 [on_clocks] received message: %s", payload)
    except Exception:
        logging.exception("💥 [on_clocks] unhandled exception processing events: %s", payload)

# ---------------------------------------------------------------------
# Publish surface
# ---------------------------------------------------------------------

SUBSCRIPTIONS = {
    "CLOCKS/STATE": on_clocks
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    try:
        server_setup(
            subsystem="CLOCKS",
            subscriptions=SUBSCRIPTIONS
        )
    except Exception:
        logging.exception("💥 [clocks] unhandled exception in main thread")

if __name__ == "__main__":
    run()
