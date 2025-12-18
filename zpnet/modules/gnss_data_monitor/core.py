"""
ZPNet GNSS Data Monitor — Authoritative Clock Snapshotter

Responsibilities:
  • Request GNSS_DATA snapshots from the Teensy
  • Emit no interpretation or cadence logic
  • Serve as the sole source of authoritative clock state

Author: The Mule + GPT
"""

import logging

from zpnet.shared.logger import setup_logging
from zpnet.shared.serial import send_teensy_command


def run() -> None:
    """
    Request a single GNSS_DATA snapshot from the Teensy.

    Emits (via Teensy response):
        GNSS_DATA
    """
    logging.info("[gnss_data_monitor] requesting GNSS_DATA_NOW")
    send_teensy_command({"cmd": "GNSS_DATA_NOW"})


def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
