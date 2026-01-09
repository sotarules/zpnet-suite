"""
ZPNet GNSS Status Monitor — Liveness Snapshot Publisher

Author: The Mule + GPT
"""

import logging

from zpnet.shared.logger import setup_logging
from zpnet.shared.teensy import request_gnss_status

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    logging.debug("[gnss_status_monitor] requesting GNSS_STATUS publication")
    request_gnss_status()


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    setup_logging()
    run()
