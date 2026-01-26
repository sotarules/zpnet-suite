"""
ZPNet Unified Command Router (Pi-Side)

Single canonical send_command() for all machines.

Author: The Mule + GPT
"""

import json
import logging
import os
import socket
from typing import Any
from typing import Dict, Callable, Optional

# ---------------------------------------------------------------------
# Socket Configuration (single source of truth)
# ---------------------------------------------------------------------

# Pi-side subsystem sockets (systemd-managed)
PI_SUBSYSTEM_SOCKETS: Dict[str, str] = {
    "SYSTEM":     "/tmp/zpnet-system.sock",
    "GNSS":       "/tmp/zpnet-gnss.sock",
    "LASER":      "/tmp/zpnet-laser.sock",
    "PHOTODIODE": "/tmp/zpnet-photodiode.sock",
    "TEMPEST":    "/tmp/zpnet-tempest.sock",
    "LANTERN":    "/tmp/zpnet-lantern.sock",
}

# Teensy transport authority (always single endpoint)
TEENSY_RPC_SOCKET = "/tmp/zpnet_teensy_rt.sock"

# ---------------------------------------------------------------------
# Public API — single canonical entry point
# ---------------------------------------------------------------------

def send_command(
    *,
    machine: str,
    subsystem: str,
    command: str,
    args: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:

    sock_path = PI_SUBSYSTEM_SOCKETS.get(subsystem) if machine == "PI" else TEENSY_RPC_SOCKET

    req: Dict[str, Any] = {
        "machine": machine,
        "subsystem": subsystem,
        "command": command,
    }

    if args is not None:
        req["args"] = args

    raw = json.dumps(
        req,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")

    print(f"Sending command to {machine} {subsystem} {command} {sock_path}")

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.connect(sock_path)
        sock.sendall(raw)
        sock.shutdown(socket.SHUT_WR)
        buf = bytearray()
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf.extend(chunk)
        return json.loads(buf.decode("utf-8"))


import socket
import json
import logging
import os
from typing import Dict, Callable, Optional

def serve_commands(
    *,
    socket_path: str,
    commands: Dict[str, Callable[[Optional[dict]], dict]],
) -> None:
    logging.info("[serve_commands] Starting command server on socket: %s", socket_path)
    logging.info("[serve_commands] Registered command handlers: %s", list(commands.keys()))

    try:
        os.unlink(socket_path)
    except FileNotFoundError:
        pass
    except Exception as e:
        logging.error("[serve_commands] Failed to unlink socket: %s", e)

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as srv:
        srv.bind(socket_path)
        srv.listen()
        logging.info("[serve_commands] Listening...")

        while True:
            conn, _ = srv.accept()
            with conn:
                try:
                    raw = conn.recv(65536).decode()
                    logging.info("[serve_commands] Raw request: %s", raw)

                    req = json.loads(raw)

                    machine = req.get("machine")
                    subsystem = req.get("subsystem")
                    command = req.get("command")
                    args = req.get("args", {})

                    logging.info("[serve_commands] machine=%s subsystem=%s command=%s args=%s", machine, subsystem, command, args)

                    if command not in commands:
                        logging.warning("[serve_commands] Unknown command: %s", command)
                        conn.sendall(json.dumps({
                            "success": False,
                            "message": f"Unknown command: {command}"
                        }).encode())
                        continue

                    handler = commands[command]
                    resp = handler(args)

                    logging.info("[serve_commands] Response: %s", resp)
                    conn.sendall(json.dumps(resp).encode())

                except Exception as e:
                    logging.exception("[serve_commands] Exception while handling command")
                    conn.sendall(json.dumps({
                        "success": False,
                        "message": str(e)
                    }).encode())

