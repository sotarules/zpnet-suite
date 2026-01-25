"""
ZPNet SYSTEM Process (Pi-side, authoritative aggregator)

This process replaces multiple Pi-side monitors with a single
system_monitor service exposing a unified SYSTEM.REPORT surface.

Responsibilities:
  • Query authoritative SYSTEM state from the Teensy
  • Maintain a last-known-good system snapshot
  • Expose system-wide facts via REPORT
  • Emit SYSTEM_STATUS events on a fixed cadence
  • Serve as the future consolidation point for system observability

Design invariants:
  • Read-only from the Pi perspective
  • Single-writer semantics (Teensy owns mutation)
  • Deterministic polling cadence
  • Snapshot replacement (last-known-good)

Process model:
  • One systemd service
  • One polling thread (Teensy PROCESS.COMMAND)
  • One blocking command socket (REPORT)
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time

from typing import Dict, Optional

from zpnet.shared.teensy import send_command
from zpnet.shared.events import create_event


# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

CMD_SOCKET_PATH = "/tmp/zpnet-system.sock"
POLL_INTERVAL_SEC = 30


# ------------------------------------------------------------------
# Authoritative SYSTEM snapshot
# ------------------------------------------------------------------

# Snapshot schema (phase 1):
#   {
#     "teensy": { ... payload from Teensy SYSTEM.REPORT ... }
#   }
SYSTEM: Dict[str, object] = {}


# ------------------------------------------------------------------
# Teensy polling thread
# ------------------------------------------------------------------

def system_poller() -> None:
    """
    Periodically request SYSTEM.REPORT from the Teensy.

    The Teensy is the sole authority for system mutation. This thread
    mirrors the latest successful snapshot without interpretation.

    Phase 1 schema:
      • SYSTEM["teensy"] contains the raw Teensy SYSTEM.REPORT payload.
    """
    global SYSTEM

    while True:
        try:
            resp = send_command(
                "PROCESS.COMMAND",
                {
                    "type": "SYSTEM",
                    "proc_cmd": "REPORT",
                },
            )
        except Exception:
            resp = None

        if resp and resp.get("success"):
            payload = resp.get("payload", {})
            SYSTEM = {
                "teensy": dict(payload),
            }

        snapshot = dict(SYSTEM)
        create_event("SYSTEM_STATUS", snapshot)

        time.sleep(POLL_INTERVAL_SEC)


# ------------------------------------------------------------------
# REPORT command
# ------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:
    """Return the most recent SYSTEM snapshot."""
    snapshot = dict(SYSTEM)

    return {
        "success": True,
        "message": "OK",
        "payload": snapshot,
    }


COMMANDS = {
    "REPORT": cmd_report,
}


# ------------------------------------------------------------------
# Command socket server
# ------------------------------------------------------------------

def command_server() -> None:
    try:
        os.unlink(CMD_SOCKET_PATH)
    except FileNotFoundError:
        pass

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as srv:
        srv.bind(CMD_SOCKET_PATH)
        srv.listen()

        while True:
            conn, _ = srv.accept()
            with conn:
                data = conn.recv(65536)
                if not data:
                    continue

                try:
                    req = json.loads(data.decode())
                except Exception:
                    resp = {
                        "success": False,
                        "message": "invalid json",
                    }
                    conn.sendall(json.dumps(resp).encode())
                    continue

                handler = COMMANDS.get(req.get("cmd"))
                resp = handler(req.get("args")) if handler else {
                    "success": False,
                    "message": "unrecognized command",
                }

                conn.sendall(json.dumps(resp).encode())


# ------------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------------

def run() -> None:
    threading.Thread(target=system_poller, daemon=True).start()
    command_server()
