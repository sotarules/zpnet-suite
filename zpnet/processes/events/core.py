"""
ZPNet Event Despooler (Pi-side, authoritative)

Responsibilities:
  • Periodically drain the Teensy durable event queue
  • Forward each event into the local ZPNet event store

Process model:
  • One systemd service
  • One polling thread (own execution context)
  • One blocking command socket (REPORT)

Semantics:
  • Uses unified send_command() interface
  • No internal defensiveness
  • One fault barrier per execution context
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional

from zpnet.processes.processes import serve_commands, send_command
from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging

CMD_SOCKET_PATH = "/tmp/zpnet-events.sock"
POLL_INTERVAL_S = 5


# ---------------------------------------------------------------------
# Internal state
# ---------------------------------------------------------------------

_last_drain_ts: float = 0.0
_last_count: int = 0

# ---------------------------------------------------------------------
# Polling thread (fault barrier)
# ---------------------------------------------------------------------

def despooler_loop() -> None:
    import logging
    global _last_drain_ts, _last_count

    try:
        while True:
            payload = send_command(machine="TEENSY", subsystem="EVENTS", command="GET")["payload"]
            for item in payload["events"]:
                create_event(item["event_type"], item.get("payload"))
            count = len(payload)
            _last_count = count
            _last_drain_ts = time.time()
            time.sleep(POLL_INTERVAL_S)

    except Exception:
        logging.exception(
            "[event_despooler] unhandled exception — despooler thread terminating"
        )


# ---------------------------------------------------------------------
# Command surface
# ---------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "poll_interval_s": POLL_INTERVAL_S,
            "last_drain_ts": _last_drain_ts,
            "last_count": _last_count,
        },
    }


COMMANDS = {
    "REPORT": cmd_report,
}


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    try:
        threading.Thread(
            target=despooler_loop,
            daemon=True,
        ).start()

        serve_commands(
            socket_path=CMD_SOCKET_PATH,
            commands=COMMANDS,
        )

    except Exception:
        import logging
        logging.exception("[event_despooler] unhandled exception in main thread")


if __name__ == "__main__":
    run()
