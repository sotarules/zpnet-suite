"""
ZPNet Teensy Client Interface

Single-path command interface to the Teensy runtime via teensy_listener.

Design principles:
  • No direct serial access (ever)
  • All communication goes through teensy_listener via UNIX socket
  • Exactly one command semantic
  • Every command returns a response
  • Timeouts are transport policy, not user intent
  • RPC is a pure transport pipe
  • Thin, synchronous, deterministic
  • Safe to mock in tests

The Teensy is treated as a remote instrument, not a local device.

Author: The Mule + GPT
"""

import json
import socket
from typing import Any, Dict, Optional

# ---------------------------------------------------------------------
# Transport Configuration (internal policy)
# ---------------------------------------------------------------------
TEENSY_RPC_SOCKET = "/tmp/zpnet_teensy_rt.sock"
RPC_TIMEOUT_S = 10.0
MAX_REPLY_BYTES = 256 * 1024


# ---------------------------------------------------------------------
# Public API — single canonical entry point
# ---------------------------------------------------------------------
def send_command(
    command: str,
    args: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:

    cmd_obj: Dict[str, Any] = {"cmd": command}
    if args is not None:
        cmd_obj["args"] = args

    raw_req = json.dumps(
        cmd_obj,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")

    # --------------------------------------------------------------
    # RPC transport (UNIX socket)
    # --------------------------------------------------------------
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(RPC_TIMEOUT_S)
        s.connect(TEENSY_RPC_SOCKET)
        s.sendall(raw_req)
        s.shutdown(socket.SHUT_WR)

        buf = bytearray()
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf.extend(chunk)
            if len(buf) > MAX_REPLY_BYTES:
                raise RuntimeError("RPC reply too large")

    # --------------------------------------------------------------
    # Decode JSON reply (transport responsibility only)
    # --------------------------------------------------------------
    try:
        resp = json.loads(buf.decode("utf-8"))
    except Exception as e:
        raise RuntimeError(f"Invalid JSON reply from teensy_listener: {e}")

    if not isinstance(resp, dict):
        raise RuntimeError("RPC reply was not a JSON object")

    return resp


# ---------------------------------------------------------------------
# Convenience wrappers (thin, expressive, STILL SEMANTIC-FREE)
# ---------------------------------------------------------------------
def request_teensy_status() -> Dict[str, Any]:
    return send_command("TEENSY.STATUS")


def request_gnss_status() -> Dict[str, Any]:
    return send_command("GNSS.STATUS")


def request_gnss_data() -> Dict[str, Any]:
    return send_command("GNSS.DATA")


def dwt_count() -> Dict[str, Any]:
    return send_command("DWT.COUNT")


def photodiode_status() -> Dict[str, Any]:
    return send_command("PHOTODIODE.STATUS")


def system_shutdown() -> Dict[str, Any]:
    return send_command("SYSTEM.SHUTDOWN")
