"""
ZPNet GNSS Status Monitor — Liveness Snapshot Publisher

Responsibilities:
  • Issue a GNSS.STATUS command to the Teensy
  • Cause a GNSS_STATUS event to be enqueued on the Teensy
  • Perform NO interpretation, inference, or cadence logic
  • Emit NOTHING directly
  • Remain stateless and scheduler-driven

GNSS_STATUS represents lightweight GNSS liveness and raw-sentence visibility.
It is not authoritative clock truth.

Delivery semantics:
  • Command returns ACK only
  • Actual GNSS_STATUS flows through the EVENT queue
  • Ingested via teensy_listener during normal despool
  • Aggregated downstream if desired

Author: The Mule + GPT
"""

import logging

from zpnet.shared.logger import setup_logging
from zpnet.shared.serial import (
    send_teensy_command,
    cmd_gnss_status,   # ← event-generating command (no ?)
)

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Request that the Teensy publish its current GNSS liveness status.

    Semantics:
      • Imperative command (GNSS.STATUS)
      • Causes GNSS_STATUS to be enqueued as an EVENT
      • No immediate response data is expected
      • Durable, framed delivery via EVENTS.GET

    Emits (indirectly, via Teensy event queue):
        GNSS_STATUS
    """
    logging.debug("[gnss_status_monitor] requesting GNSS_STATUS publication")
    send_teensy_command(cmd_gnss_status())


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
