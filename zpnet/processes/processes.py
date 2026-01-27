"""
ZPNet Unified Command Router (Pi-Side)

Single canonical send_command() for all machines.

Semantics:
  • One request → one response
  • Message-based, not stream-based
  • No internal recovery
  • All failures are programmer-visible

Author: The Mule + GPT
"""

import json
import logging
import os
import socket
import time
from typing import Any, Dict, Callable, Optional

# ---------------------------------------------------------------------
# Socket Configuration (single source of truth)
# ---------------------------------------------------------------------

PI_SUBSYSTEM_SOCKETS: Dict[str, str] = {
    "SYSTEM":     "/tmp/zpnet-system.sock",
    "GNSS":       "/tmp/zpnet-gnss.sock",
    "EVENTS":     "/tmp/zpnet-events.sock",
    "LASER":      "/tmp/zpnet-laser.sock",
    "PHOTODIODE": "/tmp/zpnet-photodiode.sock",
    "TEMPEST":    "/tmp/zpnet-tempest.sock",
    "LANTERN":    "/tmp/zpnet-lantern.sock",
}

TEENSY_RPC_SOCKET = "/tmp/zpnet_teensy_rt.sock"

# ---------------------------------------------------------------------
# Public API — canonical RPC client
# ---------------------------------------------------------------------

def send_command(
    *,
    machine: str,
    subsystem: str,
    command: str,
    args: Optional[Dict[str, Any]] = None,
    retries: int = 5,
    retry_delay_s: float = 0.25,
) -> Dict[str, Any]:
    """
    Send a single command and return a single response.

    Semantics:
      • JSON-in / JSON-out
      • Bounded retries at unowned transport boundary
      • Any persistent failure propagates
    """

    if machine == "PI":
        sock_path = PI_SUBSYSTEM_SOCKETS[subsystem]
    else:
        sock_path = TEENSY_RPC_SOCKET

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

    last_exc: Exception | None = None

    for attempt in range(1, retries + 1):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)

                resp_raw = sock.recv(65536)

                if not resp_raw:
                    raise RuntimeError(
                        "[send_command] empty response (device reset or transport not ready)"
                    )

                return json.loads(resp_raw.decode("utf-8"))

        except (
            FileNotFoundError,
            ConnectionRefusedError,
            ConnectionResetError,
            RuntimeError,
        ) as e:
            last_exc = e

            if attempt >= retries:
                break

            time.sleep(retry_delay_s)

    # -----------------------------------------------------------------
    # Final failure — propagate loudly
    # -----------------------------------------------------------------
    raise RuntimeError(
        f"[send_command] failed after {retries} attempts "
        f"({machine} {subsystem} {command})"
    ) from last_exc


# ---------------------------------------------------------------------
# Command server (Pi-side process endpoint)
# ---------------------------------------------------------------------

def serve_commands(
    *,
    socket_path: str,
    commands: Dict[str, Callable[[Optional[dict]], dict]],
) -> None:
    """
    Serve blocking command requests forever.

    Semantics:
      • Exactly one server owns the socket
      • One connection = one request
      • One handler invocation
      • One response
      • Stale socket files are removed
      • Live servers are never preempted
    """

    logging.info("[serve_commands] binding socket: %s", socket_path)
    logging.info(
        "[serve_commands] commands: %s",
        sorted(commands.keys())
    )

    # ------------------------------------------------------------
    # Socket ownership check (unowned boundary)
    # ------------------------------------------------------------
    if os.path.exists(socket_path):
        # Is someone already listening?
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                probe.connect(socket_path)
            # If connect() succeeds, another server is alive.
            raise RuntimeError(
                f"command socket already active: {socket_path}"
            )
        except ConnectionRefusedError:
            # Stale socket file — safe to remove
            os.unlink(socket_path)

    # ------------------------------------------------------------
    # Bind and serve
    # ------------------------------------------------------------
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as srv:
        srv.bind(socket_path)
        srv.listen()

        while True:
            conn, _ = srv.accept()
            with conn:
                raw = conn.recv(65536).decode("utf-8")
                logging.info("[serve_commands] raw request: %s", raw)

                req = json.loads(raw)

                command = req["command"]
                args = req.get("args")

                handler = commands[command]
                resp = handler(args)

                conn.sendall(
                    json.dumps(resp, separators=(",", ":")).encode("utf-8")
                )

