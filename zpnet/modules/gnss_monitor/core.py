"""
ZPNet GNSS Monitor — Unified GNSS Report Publisher

Author: The Mule + GPT
"""

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging
from zpnet.shared.teensy import send_command


# ---------------------------------------------------------------------
# Main routine
# ---------------------------------------------------------------------
def run() -> None:
    reply = send_command(
        "PROCESS.COMMAND",
        { "type": "GNSS", "proc_cmd": "REPORT" }
    )

    payload = reply.get("payload")

    create_event("GNSS_REPORT", payload)


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
