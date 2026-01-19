"""
ZPNet Teensy Monitor — Health Snapshot Publisher

Author: The Mule + GPT
"""

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging
from zpnet.shared.teensy import send_command

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    reply = send_command("TEENSY.STATUS")
    payload = reply.get("payload")
    create_event("TEENSY_STATUS", payload)

# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    setup_logging()
    run()
