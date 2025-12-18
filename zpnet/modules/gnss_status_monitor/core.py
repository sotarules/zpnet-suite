"""
ZPNet GNSS Status Monitor — Liveness Observer

Responsibilities:
  • Request GNSS_STATUS snapshots from the Teensy
  • Emit no interpretation or aggregation
  • Remain stateless and scheduler-driven

Author: The Mule + GPT
"""

import logging

from zpnet.shared.logger import setup_logging
from zpnet.shared.serial import send_teensy_command


def run() -> None:
    """
    Request a single GNSS_STATUS snapshot from the Teensy.

    Emits (via Teensy response):
        GNSS_STATUS
    """
    logging.debug("[gnss_status_monitor] requesting GNSS_STATUS_NOW")
    send_teensy_command({"cmd": "GNSS_STATUS_NOW"})


def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
