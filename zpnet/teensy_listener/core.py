"""
ZPNet Teensy Listener — Serial Transport Authority + Socket RPC Server
(UART framed transport edition)

This daemon is the absolute sole owner of the Teensy runtime serial interface
(e.g. /dev/zpnet-teensy-serial). It provides two orthogonal services:

  1) Durable Event Despooling (canonical memory path)
     - Periodically issues EVENTS.GET
     - Drains Teensy event queue (durable facts only)
     - Persists events to PostgreSQL via create_event()

  2) Local Socket RPC (client command/query plane)
     - UNIX socket server used by all other Python modules
     - Allows:
         op="send"  : send a command (no immediate reply expected)
         op="query" : send a command and wait for one immediate reply (type)
         op="drain" : force an EVENTS.GET drain and return events (optional persist)

Design invariants:
  • Exactly one owner of the Teensy serial port (THIS daemon)
  • Framed UART transport (<STX=N> payload <ETX>)
  • Durable truth is persisted to PostgreSQL
  • Ephemeral truth is observable via socket RPC but not remembered by default
  • No other module opens the serial device

Author: The Mule + GPT
"""

import json
import logging
import os
import socket
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import serial

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging
from zpnet.shared.constants import (
    TEENSY_SERIAL_PORT,
    TEENSY_BAUDRATE,
    TEENSY_READ_TIMEOUT_S,
)

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
EVENT_DESPOOL_INTERVAL_S = 10.0

# Single well-known RPC socket
RPC_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"

# Socket behavior
RPC_LISTEN_BACKLOG = 8
RPC_RECV_MAX_BYTES = 256 * 1024  # defensive cap per request
RPC_DEFAULT_TIMEOUT_S = 1.0

# Drain behavior
DRAIN_READ_TIMEOUT_S = 1.0
DRAIN_MAX_EVENTS_RETURNED = 64

# Framing constants
STX_PREFIX = b"<STX="
ETX_SEQ = b"<ETX>"

# ---------------------------------------------------------------------
# Environment overrides
# ---------------------------------------------------------------------
SERIAL_SNOOP_PATH = os.environ.get("ZPNET_SERIAL_SNOOP")

# ---------------------------------------------------------------------
# Serial ownership primitives
# ---------------------------------------------------------------------
_serial_lock = threading.RLock()
_serial: serial.Serial | None = None

# ---------------------------------------------------------------------
# Serial open/close
# ---------------------------------------------------------------------
def open_teensy_serial() -> serial.Serial:
    """
    Open the Teensy runtime serial port.

    Note: TEENSY_SERIAL_PORT should point to the runtime UART identity,
    typically /dev/zpnet-teensy-serial.
    """
    ser = serial.Serial(
        TEENSY_SERIAL_PORT,
        TEENSY_BAUDRATE,
        timeout=TEENSY_READ_TIMEOUT_S,
        write_timeout=TEENSY_READ_TIMEOUT_S,
    )
    # Small settle is fine even for UART; harmless and stabilizing.
    time.sleep(0.05)
    return ser


# ---------------------------------------------------------------------
# Framed transport helpers
# ---------------------------------------------------------------------
def frame_encode(payload: bytes) -> bytes:
    """
    Encode payload bytes into a framed message:
      <STX=N><payload><ETX>
    """
    if not isinstance(payload, (bytes, bytearray)):
        raise TypeError("frame_encode expects bytes")

    n = len(payload)
    return b"<STX=" + str(n).encode("ascii") + b">" + payload + ETX_SEQ


def _read_exact(ser: serial.Serial, n: int, deadline: float) -> bytes:
    """
    Read exactly n bytes from serial or raise TimeoutError.
    """
    buf = bytearray()
    while len(buf) < n:
        if time.time() > deadline:
            raise TimeoutError("serial read_exact timeout")
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
            continue
        # No data: yield briefly
        time.sleep(0.001)
    return bytes(buf)


def frame_read_one(
    ser: serial.Serial,
    *,
    timeout_s: float,
) -> Optional[dict]:
    """
    Read one framed payload and return it parsed as JSON dict.

    Returns:
        dict on success, None on timeout/empty.
    Raises:
        ValueError on malformed framing or JSON.
    """
    deadline = time.time() + timeout_s

    # Step 1: hunt for '<'
    while True:
        if time.time() > deadline:
            return None
        b = ser.read(1)
        if not b:
            continue
        if b == b"<":
            break

    # Step 2: read until '>' to complete "<STX=...>"
    header = bytearray(b"<")
    while True:
        if time.time() > deadline:
            raise TimeoutError("timeout while reading STX header")
        b = ser.read(1)
        if not b:
            continue
        header.extend(b)
        if b == b">":
            break
        if len(header) > 32:
            raise ValueError("STX header too long")

    # Validate and parse length
    if not header.startswith(STX_PREFIX) or not header.endswith(b">"):
        raise ValueError(f"invalid STX header: {header!r}")

    # header like b"<STX=21>"
    inner = header[len(STX_PREFIX):-1]  # b"21"
    if not inner.isdigit():
        raise ValueError(f"invalid STX length: {inner!r}")

    length = int(inner.decode("ascii"))
    if length <= 0 or length > 4096:
        raise ValueError(f"unsupported payload length: {length}")

    # Step 3: read payload bytes
    payload = _read_exact(ser, length, deadline)

    # Step 4: read ETX sequence
    etx = _read_exact(ser, len(ETX_SEQ), deadline)
    if etx != ETX_SEQ:
        raise ValueError(f"invalid ETX terminator: {etx!r}")

    # Step 5: parse JSON payload
    try:
        msg = json.loads(payload.decode("utf-8", errors="strict"))
        _snoop("←", payload)
    except Exception as e:
        raise ValueError(f"invalid JSON payload: {e}")

    if not isinstance(msg, dict):
        raise ValueError("framed payload was not a JSON object")

    return msg


def send_cmd_framed(ser: serial.Serial, cmd: dict) -> None:
    """
    Send a command dict as framed JSON.

    This is the ONLY correct way to send commands over the new transport.
    """
    payload = json.dumps(cmd, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    framed = frame_encode(payload)
    _snoop("→", payload)
    logging.debug("[teensy_listener] → %s", payload.decode("utf-8", errors="ignore"))
    ser.write(framed)
    ser.flush()


# ---------------------------------------------------------------------
# Drain logic (durable event queue)
# ---------------------------------------------------------------------
def drain_events(
    ser: serial.Serial,
    *,
    persist: bool,
    collect: bool,
    stop_after: int | None = None,
    timeout_s: float = DRAIN_READ_TIMEOUT_S,
) -> List[Dict[str, Any]]:
    """
    Issue EVENTS.GET, then read framed messages until control EVENTS_END.

    Teensy emits:
      {"control":"EVENTS_BEGIN", "count":..., "dropped":...}
      {"event_type":"...", ...payload fields...}
      {"control":"EVENTS_END"}

    Returns collected durable events if collect=True.

    Notes:
      • control messages are not persisted
      • only messages with "event_type" are treated as durable facts
    """
    collected: List[Dict[str, Any]] = []
    in_drain = False

    send_cmd_framed(ser, {"cmd": "EVENTS.GET"})

    while True:
        msg = frame_read_one(ser, timeout_s=timeout_s)
        if msg is None:
            break

        # Control messages
        control = msg.get("control")
        if isinstance(control, str):
            if control == "EVENTS_BEGIN":
                in_drain = True
                continue
            if control == "EVENTS_END":
                break
            # Unknown control: ignore
            continue

        if not in_drain:
            # Ignore anything arriving outside a drain window
            continue

        event_type = msg.get("event_type")
        if not isinstance(event_type, str) or not event_type:
            continue

        payload = msg.copy()
        payload.pop("event_type", None)

        if persist:
            create_event(event_type, payload)

        if collect:
            collected.append({"event_type": event_type, "payload": payload})
            if stop_after and len(collected) >= stop_after:
                break

    return collected


def drain_events_once(ser: serial.Serial) -> Tuple[int, float]:
    """
    Durable, periodic despool. Persists to PostgreSQL.
    """
    start = time.time()
    acquired = _serial_lock.acquire(timeout=0.25)
    if not acquired:
        return 0, 0.0

    try:
        events = drain_events(ser, persist=True, collect=False)
        return len(events), time.time() - start
    finally:
        _serial_lock.release()


# ---------------------------------------------------------------------
# Realtime query plane (immediate replies only)
# ---------------------------------------------------------------------
def perform_realtime_query(
    cmd: dict,
    *,
    timeout_s: float = RPC_DEFAULT_TIMEOUT_S,
) -> Dict[str, Any]:
    """
    Send a command and wait for one immediate reply message:

      {"type":"DWT_COUNT", ...}
      {"type":"PHOTODIODE_STATUS", ...}

    Returns:
      {"type": <str>, "payload": <dict>} on success

    Raises:
      TimeoutError if no immediate reply arrives.
      ValueError on malformed reply.
    """
    if _serial is None:
        raise RuntimeError("serial not connected")

    deadline = time.time() + timeout_s

    with _serial_lock:
        send_cmd_framed(_serial, cmd)

        while time.time() < deadline:
            msg = frame_read_one(_serial, timeout_s=max(0.01, deadline - time.time()))
            if msg is None:
                continue

            msg_type = msg.get("type")
            if not isinstance(msg_type, str) or not msg_type:
                # Not an immediate reply; ignore it here.
                # (Durable events should only appear during drains.)
                continue

            payload = msg.copy()
            payload.pop("type", None)

            return {"type": msg_type, "payload": payload}

    raise TimeoutError("no immediate reply from Teensy")


def perform_send_only(cmd: dict) -> None:
    """
    Send a command that is expected to enqueue durable events (ACK, etc.)
    and does not produce an immediate reply.
    """
    if _serial is None:
        raise RuntimeError("serial not connected")

    with _serial_lock:
        send_cmd_framed(_serial, cmd)


def perform_drain_on_demand(
    *,
    persist: bool,
    max_events: int,
    timeout_s: float,
) -> List[Dict[str, Any]]:
    """
    Force an EVENTS.GET drain and return collected durable events.
    Optionally persist them.
    """
    if _serial is None:
        raise RuntimeError("serial not connected")

    with _serial_lock:
        events = drain_events(
            _serial,
            persist=persist,
            collect=True,
            stop_after=max_events,
            timeout_s=timeout_s,
        )
    return events


# ---------------------------------------------------------------------
# Socket RPC server
# ---------------------------------------------------------------------
def _read_json_request(conn: socket.socket) -> dict:
    """
    Read one JSON request object from a stream socket.

    Contract:
      • client sends exactly one JSON object
      • client may close immediately after sending
      • server reads until JSON parses or size cap reached

    Raises:
      ValueError on invalid JSON
    """
    buf = bytearray()
    conn.settimeout(2.0)

    while True:
        if len(buf) > RPC_RECV_MAX_BYTES:
            raise ValueError("request too large")

        chunk = conn.recv(4096)
        if not chunk:
            # EOF
            break
        buf.extend(chunk)

        # Try parsing at each step (cheap for small payloads)
        try:
            obj = json.loads(buf.decode("utf-8"))
            if not isinstance(obj, dict):
                raise ValueError("request must be a JSON object")
            return obj
        except json.JSONDecodeError:
            continue

    # Final parse attempt
    obj = json.loads(buf.decode("utf-8"))
    if not isinstance(obj, dict):
        raise ValueError("request must be a JSON object")
    return obj


def _send_json_response(conn: socket.socket, payload: dict) -> None:
    raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    conn.sendall(raw)


def _handle_client(conn: socket.socket) -> None:
    """
    Handle one RPC connection.
    """
    try:
        req = _read_json_request(conn)

        op = req.get("op", "query")
        cmd = req.get("cmd")
        timeout_s = float(req.get("timeout_s", RPC_DEFAULT_TIMEOUT_S))

        # Defensive normalization
        if op not in ("query", "send", "drain"):
            raise ValueError("invalid op (expected query|send|drain)")

        if op in ("query", "send"):
            if not isinstance(cmd, dict):
                raise ValueError("cmd must be a dict for query/send")

        if op == "query":
            reply = perform_realtime_query(cmd, timeout_s=timeout_s)
            _send_json_response(conn, {"ok": True, "reply": reply})
            return

        if op == "send":
            perform_send_only(cmd)
            _send_json_response(conn, {"ok": True})
            return

        # op == "drain"
        persist = bool(req.get("persist", False))
        max_events = int(req.get("max_events", DRAIN_MAX_EVENTS_RETURNED))
        max_events = max(1, min(max_events, DRAIN_MAX_EVENTS_RETURNED))
        events = perform_drain_on_demand(
            persist=persist,
            max_events=max_events,
            timeout_s=timeout_s,
        )
        _send_json_response(conn, {"ok": True, "events": events})
        return

    except BrokenPipeError:
        logging.debug("[teensy_listener][rpc] client disconnected (broken pipe)")
    except Exception as e:
        try:
            _send_json_response(conn, {"ok": False, "error": str(e)})
        except Exception:
            pass
    finally:
        try:
            conn.close()
        except Exception:
            pass


def rpc_socket_server() -> None:
    """
    Run the UNIX socket RPC server forever.
    """
    if os.path.exists(RPC_SOCKET_PATH):
        os.unlink(RPC_SOCKET_PATH)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(RPC_SOCKET_PATH)
    os.chmod(RPC_SOCKET_PATH, 0o660)  # group-readable/writable
    sock.listen(RPC_LISTEN_BACKLOG)

    logging.info("🧭 [teensy_listener] RPC socket ready at %s", RPC_SOCKET_PATH)

    while True:
        conn, _ = sock.accept()
        threading.Thread(target=_handle_client, args=(conn,), daemon=True).start()


def _snoop(direction: str, payload: bytes | str) -> None:
    if not SERIAL_SNOOP_PATH:
        return
    try:
        with open(SERIAL_SNOOP_PATH, "a") as f:
            ts = time.time()
            if isinstance(payload, bytes):
                payload = payload.decode("utf-8", errors="replace")
            f.write(f"{ts:.6f} {direction} {payload}\n")
    except Exception:
        pass  # absolutely never affect runtime

# ---------------------------------------------------------------------
# Main daemon loop with serial reconnect
# ---------------------------------------------------------------------
def run() -> None:
    """
    Main daemon loop. Owns the serial port forever.
    Starts RPC socket server once and keeps it alive.
    """
    logging.info("🚀 [teensy_listener] starting Teensy transport authority")
    global _serial

    # Start RPC server once (independent of serial reconnects)
    threading.Thread(target=rpc_socket_server, daemon=True).start()

    while True:
        try:
            _serial = open_teensy_serial()
            logging.info("🔌 [teensy_listener] serial connection established: %s", TEENSY_SERIAL_PORT)

            while True:
                try:
                    _count, _elapsed = drain_events_once(_serial)
                    time.sleep(EVENT_DESPOOL_INTERVAL_S)

                except serial.SerialException as e:
                    logging.error("💥 [teensy_listener] serial exception: %s — reconnecting", e)
                    try:
                        _serial.close()
                    except Exception:
                        pass
                    _serial = None
                    time.sleep(1.0)
                    break  # exit inner loop; reconnect

        except Exception as e:
            logging.exception("💥 [teensy_listener] daemon failure: %s", e)
            _serial = None
            time.sleep(2.0)


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    setup_logging()
    run()
