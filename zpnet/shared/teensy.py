"""
ZPNet Teensy Client Interface

This module provides the *only* supported way for Python code to interact
with the Teensy runtime system.

Design principles:
  • No direct serial access (ever)
  • All communication goes through teensy_listener via UNIX socket
  • Intent-level API (commands, queries), not transport-level
  • Thin, synchronous, deterministic
  • Safe to mock in tests

The Teensy is treated as a remote instrument, not a local device.

Author: The Mule + GPT
"""

import json
import socket
from typing import Any, Dict

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
TEENSY_RPC_SOCKET = "/tmp/zpnet_teensy_rt.sock"
DEFAULT_TIMEOUT_S = 1.0
MAX_REPLY_BYTES = 256 * 1024


# ---------------------------------------------------------------------
# Low-level RPC helper
# ---------------------------------------------------------------------
def _rpc_call(
    payload: Dict[str, Any],
    *,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> Dict[str, Any]:
    """
    Perform a single RPC call to the teensy_listener socket.

    Args:
        payload (dict): JSON request object.
        timeout_s (float): socket timeout.

    Returns:
        dict: parsed JSON response.

    Raises:
        RuntimeError: on socket errors or protocol failure.
    """
    if not isinstance(payload, dict):
        raise TypeError("RPC payload must be a dict")

    raw_req = json.dumps(payload, separators=(",", ":")).encode("utf-8")

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(timeout_s)
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

    try:
        resp = json.loads(buf.decode("utf-8"))
    except Exception as e:
        raise RuntimeError(f"Invalid JSON reply from teensy_listener: {e}")

    if not isinstance(resp, dict):
        raise RuntimeError("RPC reply was not a JSON object")

    return resp


# ---------------------------------------------------------------------
# Public API — intent-level helpers
# ---------------------------------------------------------------------
def send_command(
    cmd: Dict[str, Any],
    *,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> None:
    """
    Send a command to the Teensy that is expected to enqueue durable events
    (ACK, status events, etc.) but does NOT return an immediate reply.

    Example:
        send_command({"cmd": "GNSS.DATA"})

    Raises:
        RuntimeError on failure.
    """
    resp = _rpc_call(
        {
            "op": "send",
            "cmd": cmd,
        },
        timeout_s=timeout_s,
    )

    if not resp.get("ok"):
        raise RuntimeError(f"Teensy send_command failed: {resp}")


def query(
    cmd: Dict[str, Any],
    *,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> Dict[str, Any]:
    """
    Send a command that expects an immediate reply (CMD?).

    Returns:
        dict with keys:
            {
              "type": <str>,
              "payload": <dict>
            }

    Example:
        reply = query({"cmd": "DWT.COUNT?"})
        cycles = reply["payload"]["cycles"]
    """
    resp = _rpc_call(
        {
            "op": "query",
            "cmd": cmd,
            "timeout_s": timeout_s,
        },
        timeout_s=timeout_s,
    )

    if not resp.get("ok"):
        raise RuntimeError(f"Teensy query failed: {resp}")

    reply = resp.get("reply")
    if not isinstance(reply, dict):
        raise RuntimeError("Malformed reply from teensy_listener")

    return reply

# ---------------------------------------------------------------------
# Convenience wrappers (optional but expressive)
# ---------------------------------------------------------------------
def request_teensy_status() -> None:
    send_command({"cmd": "TEENSY.STATUS"})


def request_gnss_status() -> None:
    send_command({"cmd": "GNSS.STATUS"})


def request_gnss_data() -> None:
    send_command({"cmd": "GNSS.DATA"})


def dwt_count(timeout_s: float = DEFAULT_TIMEOUT_S) -> int:
    reply = query({"cmd": "DWT.COUNT?"}, timeout_s=timeout_s)
    payload = reply.get("payload", {})
    if "cycles" not in payload:
        raise RuntimeError("DWT_COUNT reply missing 'cycles'")
    return int(payload["cycles"])


def photodiode_status(timeout_s: float = DEFAULT_TIMEOUT_S) -> Dict[str, Any]:
    reply = query({"cmd": "PHOTODIODE.STATUS?"}, timeout_s=timeout_s)
    return reply.get("payload", {})


def system_shutdown() -> None:
    send_command({"cmd": "SYSTEM.SHUTDOWN"})
