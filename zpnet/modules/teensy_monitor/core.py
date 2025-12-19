"""
ZPNet Teensy Monitor — Health Snapshot Publisher

Responsibilities:
  • Issue a TEENSY.STATUS command to the Teensy
  • Cause a TEENSY_STATUS event to be enqueued on the Teensy
  • Perform NO interpretation, inference, or aggregation
  • Emit NOTHING directly
  • Remain stateless and scheduler-driven

TEENSY_STATUS represents Teensy-local health truth:
  • CPU temperature
  • Free heap
  • Firmware version
  • Laser enable flag
  • Monotonic millis

Delivery semantics:
  • Command returns ACK only
  • Actual status flows through the EVENT queue
  • Ingested via teensy_listener during normal despool

Author: The Mule + GPT
"""

import logging

from zpnet.shared.logger import setup_logging
from zpnet.shared.serial import (
    send_teensy_command,
    cmd_teensy_status,   # ← event-generating command (no ?)
)

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Request that the Teensy publish its current health status.

    Semantics:
      • Imperative command (TEENSY.STATUS)
      • Causes TEENSY_STATUS to be enqueued as an EVENT
      • No immediate response data is expected
      • Durable, framed delivery via EVENTS.GET

    Emits (indirectly, via Teensy event queue):
        TEENSY_STATUS
    """
    logging.debug("[teensy_monitor] requesting TEENSY_STATUS publication")
    send_teensy_command(cmd_teensy_status())


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
