"""
ZPNet GNSS Data Monitor — Authoritative Clock Snapshot Publisher

Author: The Mule + GPT
"""

import logging

from zpnet.shared.logger import setup_logging
from zpnet.shared.teensy import request_gnss_data

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    logging.debug("[gnss_data_monitor] requesting GNSS_DATA publication")
    request_gnss_data()


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
