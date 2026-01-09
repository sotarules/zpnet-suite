"""
ZPNet Teensy Monitor — Health Snapshot Publisher

Author: The Mule + GPT
"""

import logging

from zpnet.shared.logger import setup_logging
from zpnet.shared.teensy import request_teensy_status

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    logging.debug("[teensy_monitor] requesting TEENSY_STATUS publication")
    request_teensy_status()


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    setup_logging()
    run()
