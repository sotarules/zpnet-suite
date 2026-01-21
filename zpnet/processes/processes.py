"""
ZPNet Pi-Side Process Command Client

Provides a Teensy-equivalent PROCESS.COMMAND interface
for Pi-side systemd-managed processes.

This module:
  • does NOT track lifecycle or state
  • does NOT start/stop processes
  • assumes systemd is authoritative
  • routes commands to process-owned sockets
  • preserves the canonical command contract

Author: The Mule + GPT
"""

from __future__ import annotations

import json
import socket
from enum import IntEnum
from typing import Any, Dict, Optional


# ---------------------------------------------------------------------
# Process Types (shared vocabulary with Teensy)
# ---------------------------------------------------------------------

class ProcessType(IntEnum):
    NONE        = 0
    GNSS        = 1
    LASER       = 2
    PHOTODIODE  = 3
    TEMPEST     = 4
    LANTERN     = 5


# ---------------------------------------------------------------------
# String → ProcessType mapping (Teensy-compatible)
# ---------------------------------------------------------------------

_PROCESS_TYPE_BY_NAME = {
    "GNSS":        ProcessType.GNSS,
    "LASER":       ProcessType.LASER,
    "PHOTODIODE":  ProcessType.PHOTODIODE,
    "TEMPEST":     ProcessType.TEMPEST,
    "LANTERN":     ProcessType.LANTERN,
}


# ---------------------------------------------------------------------
# Process → Socket Mapping (single source of truth)
# ---------------------------------------------------------------------

_PROCESS_SOCKETS = {
    ProcessType.GNSS:       "/tmp/zpnet-gnss.sock",
    ProcessType.LASER:      "/tmp/zpnet-laser.sock",
    ProcessType.PHOTODIODE: "/tmp/zpnet-photodiode.sock",
    ProcessType.TEMPEST:    "/tmp/zpnet-tempest.sock",
    ProcessType.LANTERN:    "/tmp/zpnet-lantern.sock",
}


# ---------------------------------------------------------------------
# Command Response Helpers (shape only, no interpretation)
# ---------------------------------------------------------------------

def _resp_err(message: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    out = {"success": False, "message": message}
    if payload is not None:
        out["payload"] = payload
    return out


# ---------------------------------------------------------------------
# Low-level process command (socket-based)
# ---------------------------------------------------------------------

def _process_command(
    ptype: ProcessType,
    proc_cmd: str,
    args: Optional[Dict[str, Any]] = None,
    timeout_s: float = 10.0,
) -> Dict[str, Any]:
    """
    Low-level PROCESS.COMMAND implementation.

    This is the Pi analogue of Teensy process_command().
    """

    sock_path = _PROCESS_SOCKETS.get(ptype)
    if not sock_path:
        return _resp_err("unknown process type")

    req = {
        "cmd": proc_cmd,
        "args": args,
    }

    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout_s)
            sock.connect(sock_path)
            sock.sendall(json.dumps(req).encode("utf-8"))

            data = sock.recv(256 * 1024)
            if not data:
                return _resp_err("empty response")

            resp = json.loads(data.decode("utf-8"))
            if not isinstance(resp, dict):
                return _resp_err("invalid response")

            return resp

    except FileNotFoundError:
        return _resp_err("process not running")
    except socket.timeout:
        return _resp_err("process timeout")
    except Exception as e:
        return _resp_err(
            "process communication failed",
            {"exception": str(e)}
        )


# ---------------------------------------------------------------------
# Public send_command() — Teensy-compatible entry point
# ---------------------------------------------------------------------

def send_command(op: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Canonical command entry point.

    This function intentionally mirrors the Teensy client interface.

    Example:
        send_command(
            "PROCESS.COMMAND",
            {
                "type": "GNSS",
                "proc_cmd": "REPORT",
            }
        )
    """

    # -----------------------------------------------------------------
    # PROCESS.COMMAND
    # -----------------------------------------------------------------

    if op == "PROCESS.COMMAND":
        type_name = payload.get("type")
        proc_cmd  = payload.get("proc_cmd")
        args      = payload.get("args")

        if not type_name or not proc_cmd:
            return _resp_err("missing type or proc_cmd")

        ptype = _PROCESS_TYPE_BY_NAME.get(type_name)
        if not ptype:
            return _resp_err("unknown process type")

        return _process_command(
            ptype=ptype,
            proc_cmd=proc_cmd,
            args=args,
        )

    # -----------------------------------------------------------------
    # Unknown operation
    # -----------------------------------------------------------------

    return _resp_err("unrecognized operation")
