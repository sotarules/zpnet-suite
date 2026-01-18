"""
ZPNet Teensy Listener — Serial Transport Authority + Deterministic RPC

Single reader. Explicit correlation. No guessing.
This version is hardened for bring-up, forensic visibility,
and USB disconnect/reconnect resilience.

Author: The Mule + GPT
"""

import json
import logging
import os
import socket
import threading
import time
import itertools
import queue
from queue import Queue
from typing import Dict

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

RPC_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"
RPC_BACKLOG = 8

EVENT_DESPOOL_INTERVAL_S = 10.0

STX_PREFIX = b"<STX="
ETX_SEQ = b"<ETX>"

SERIAL_SNOOP_PATH = os.environ.get("ZPNET_SERIAL_SNOOP")

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

serial_port: serial.Serial | None = None

# req_id → Queue[dict]
pending_replies: Dict[int, Queue[dict]] = {}

req_id_counter = itertools.count(1)

state_lock = threading.Lock()

# ---------------------------------------------------------------------
# Serial open / reopen
# ---------------------------------------------------------------------

def open_serial_blocking() -> serial.Serial:
    """
    Open the Teensy serial port, blocking until it is available.
    Used on startup and after USB disconnect.
    """
    while True:
        try:
            ser = serial.Serial(
                TEENSY_SERIAL_PORT,
                TEENSY_BAUDRATE,
                timeout=TEENSY_READ_TIMEOUT_S,
                write_timeout=TEENSY_READ_TIMEOUT_S,
            )
            logging.info("[teensy_listener] serial connected")
            return ser
        except Exception as e:
            logging.warning(
                "[teensy_listener] waiting for Teensy serial (%s)",
                e,
            )
            time.sleep(1.0)

# ---------------------------------------------------------------------
# Framed transport helpers
# ---------------------------------------------------------------------

def send_frame(payload: dict) -> None:
    """
    Frame and send a JSON payload to the Teensy.
    This is the ONLY outbound serial path.
    """
    raw = json.dumps(
        payload,
        separators=(",", ":"),
        ensure_ascii=False
    ).encode("utf-8")

    frame = (
        b"<STX=" +
        str(len(raw)).encode("ascii") +
        b">" +
        raw +
        ETX_SEQ
    )

    logging.info(
        "[teensy_listener] SEND_FRAME cmd=%s bytes=%d",
        payload.get("cmd"),
        len(raw),
    )

    _snoop("→", raw)

    serial_port.write(frame)
    serial_port.flush()


def read_frame(timeout_s: float) -> dict | None:
    """
    Read exactly one framed payload from the serial port.
    JSON errors are logged and ignored.
    """
    deadline = time.time() + timeout_s

    # Seek '<'
    while time.time() < deadline:
        b = serial_port.read(1)
        if b == b"<":
            break
    else:
        return None

    # Read STX header
    header = b"<"
    while True:
        b = serial_port.read(1)
        header += b
        if b == b">":
            break

    try:
        length = int(header[len(STX_PREFIX):-1])
    except Exception:
        logging.error(
            "[teensy_listener] BAD STX HEADER: %r",
            header,
        )
        return None

    payload = serial_port.read(length)
    serial_port.read(len(ETX_SEQ))

    _snoop("←", payload)

    try:
        return json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError as e:
        logging.error(
            "[teensy_listener] INVALID JSON FROM TEENSY: %r (%s)",
            payload,
            e,
        )
        _snoop("✗", payload)
        return None

# ---------------------------------------------------------------------
# Serial reader loop (IMMORTAL + USB-RESILIENT)
# ---------------------------------------------------------------------

def serial_reader() -> None:
    global serial_port

    logging.info("[teensy_listener] serial_reader started")
    last_drain = 0.0

    while True:
        try:
            msg = read_frame(timeout_s=0.1)
            now = time.time()

            if msg:
                logging.debug(
                    "[teensy_listener] RX_FRAME %s",
                    msg,
                )

                # -----------------------------------------------------
                # Correlated RPC response
                # -----------------------------------------------------
                if "req_id" in msg:
                    req_id = msg["req_id"]
                    with state_lock:
                        q = pending_replies.pop(req_id, None)
                    if q:
                        q.put(msg)
                    else:
                        logging.warning(
                            "[teensy_listener] ORPHAN RESPONSE req_id=%s",
                            req_id,
                        )
                    continue

                # -----------------------------------------------------
                # Event drain
                # -----------------------------------------------------
                if msg.get("control") == "EVENTS_BEGIN":
                    logging.info("[teensy_listener] EVENTS_BEGIN")
                    while True:
                        ev = read_frame(timeout_s=1.0)
                        if ev is None:
                            break
                        if ev.get("control") == "EVENTS_END":
                            logging.info("[teensy_listener] EVENTS_END")
                            break
                        if "event_type" in ev:
                            payload = ev.copy()
                            event_type = payload.pop("event_type")
                            create_event(event_type, payload)
                    continue

            # ---------------------------------------------------------
            # Periodic drain request
            # ---------------------------------------------------------
            if now - last_drain >= EVENT_DESPOOL_INTERVAL_S:
                send_frame({"cmd": "EVENTS.GET"})
                last_drain = now

        except serial.SerialException as e:
            # USB disconnect / Teensy reset
            logging.error(
                "[teensy_listener] serial disconnected (%s) — reopening",
                e,
            )
            try:
                serial_port.close()
            except Exception:
                pass

            serial_port = open_serial_blocking()
            last_drain = 0.0

        except Exception as e:
            # Absolute last-resort containment: reader must never die
            logging.exception(
                "[teensy_listener] serial_reader exception (ignored): %s",
                e,
            )
            time.sleep(0.1)

# ---------------------------------------------------------------------
# RPC handler
# ---------------------------------------------------------------------

def handle_client(conn: socket.socket) -> None:
    req_id = 0
    q: Queue[dict] | None = None

    try:
        # Buffered read-until-JSON (stream-safe)
        buf = bytearray()
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf.extend(chunk)
            try:
                req = json.loads(buf.decode("utf-8"))
                break
            except json.JSONDecodeError:
                continue

        logging.info("[teensy_listener] RPC_REQUEST %s", req)

        req_id = next(req_id_counter)
        req["req_id"] = req_id

        q = Queue(maxsize=1)

        with state_lock:
            pending_replies[req_id] = q

        send_frame(req)

        reply = q.get(timeout=1.0)

        try:
            conn.sendall(
                json.dumps(reply, separators=(",", ":")).encode("utf-8")
            )
        except BrokenPipeError:
            logging.warning(
                "[teensy_listener] client disconnected before reply (req_id=%s)",
                req_id,
            )

    except queue.Empty:
        logging.warning(
            "[teensy_listener] RPC TIMEOUT req_id=%s",
            req_id,
        )
        with state_lock:
            pending_replies.pop(req_id, None)

        try:
            conn.sendall(
                json.dumps(
                    {"success": False, "message": "RPC timeout"},
                    separators=(",", ":")
                ).encode("utf-8")
            )
        except BrokenPipeError:
            pass

    except Exception as e:
        logging.exception(
            "[teensy_listener] RPC ERROR req_id=%s: %s",
            req_id,
            e,
        )
        with state_lock:
            if req_id:
                pending_replies.pop(req_id, None)

        try:
            conn.sendall(
                json.dumps(
                    {"success": False, "message": str(e)},
                    separators=(",", ":")
                ).encode("utf-8")
            )
        except BrokenPipeError:
            pass

    finally:
        try:
            conn.close()
        except Exception:
            pass

# ---------------------------------------------------------------------
# RPC server
# ---------------------------------------------------------------------

def rpc_server() -> None:
    if os.path.exists(RPC_SOCKET_PATH):
        os.unlink(RPC_SOCKET_PATH)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(RPC_SOCKET_PATH)
    os.chmod(RPC_SOCKET_PATH, 0o660)
    sock.listen(RPC_BACKLOG)

    logging.info(
        "🧭 [teensy_listener] RPC socket ready at %s",
        RPC_SOCKET_PATH,
    )

    while True:
        conn, _ = sock.accept()
        threading.Thread(
            target=handle_client,
            args=(conn,),
            daemon=True
        ).start()

# ---------------------------------------------------------------------
# Serial snoop
# ---------------------------------------------------------------------

def _snoop(direction: str, payload: bytes) -> None:
    if not SERIAL_SNOOP_PATH:
        return
    try:
        with open(SERIAL_SNOOP_PATH, "a") as f:
            f.write(
                f"{time.time():.6f} {direction} "
                f"{payload.decode('utf-8', errors='replace')}\n"
            )
    except Exception:
        pass

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def run() -> None:
    global serial_port

    setup_logging()
    logging.info("🚀 [teensy_listener] starting Teensy transport authority")

    serial_port = open_serial_blocking()

    threading.Thread(target=serial_reader, daemon=True).start()
    threading.Thread(target=rpc_server, daemon=True).start()

    while True:
        time.sleep(1)

# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------

def bootstrap() -> None:
    run()
