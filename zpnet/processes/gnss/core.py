"""
ZPNet GNSS Process (Pi-side, authoritative)

Responsibilities:
  • Own the GNSS UART
  • Parse NMEA into authoritative state
  • Expose derived facts via REPORT
  • Expose raw NMEA sentences via a live stream socket

Process model:
  • One systemd service
  • One acquisition thread (UART)
  • One blocking command socket (REPORT)
  • One blocking stream socket (fan-out)
"""

from __future__ import annotations

import logging
import math
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Set

import serial

from zpnet.processes.processes import server_setup, publish
from zpnet.shared.constants import Payload
from zpnet.shared.logger import setup_logging

GNSS_DEVICE = os.environ.get("ZPNET_GNSS_PORT", "/dev/zpnet-gnss-serial")
GNSS_BAUD   = int(os.environ.get("ZPNET_GNSS_BAUD", "38400"))

CMD_SOCKET_PATH    = "/tmp/zpnet-gnss.sock"
STREAM_SOCKET_PATH = "/tmp/zpnet-gnss-stream.sock"

@dataclass
class GnssState:
    has_time: bool = False
    has_date: bool = False
    hour: int = 0
    minute: int = 0
    second: int = 0
    year: int = 0
    month: int = 0
    day: int = 0

    has_fix: bool = False
    fix_quality: int = 0
    fix_type: int = 0
    satellites: int = 0

    latitude_deg: float = math.nan
    longitude_deg: float = math.nan
    altitude_m: float = math.nan

    speed_knots: float = math.nan
    course_deg: float = math.nan

    has_discipline: bool = False
    discipline_mode: str = ""

    last_sentence: str = ""
    last_rmc: str = ""
    last_gga: str = ""
    last_gsa: str = ""
    last_zda: str = ""
    last_gsv: str = ""
    last_crw: str = ""
    last_crx: str = ""
    last_crz: str = ""

    last_rx_ts: float = 0.0

GNSS = GnssState()

_stream_clients: Set[socket.socket] = set()
_stream_lock = threading.Lock()

def publish_sentence(line: str) -> None:
    payload = (line + "\n").encode("utf-8", errors="ignore")
    with _stream_lock:
        dead = []
        for conn in _stream_clients:
            try:
                conn.sendall(payload)
            except Exception:
                dead.append(conn)
        for conn in dead:
            _stream_clients.discard(conn)
            try:
                conn.close()
            except Exception:
                pass

def stream_server() -> None:
    import logging

    try:

        if os.path.exists(STREAM_SOCKET_PATH):
            os.unlink(STREAM_SOCKET_PATH)

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as srv:
            srv.bind(STREAM_SOCKET_PATH)
            srv.listen()

            while True:
                conn, _ = srv.accept()
                with _stream_lock:
                    _stream_clients.add(conn)

    except Exception:
        logging.exception("[stream_server] unhandled exception")

def nmea_latlon_to_deg(ddmm: str, hemi: str) -> float:
    v = float(ddmm)
    deg = math.floor(v / 100.0)
    minutes = v - deg * 100.0
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out

def parse_rmc(line: str) -> None:
    parts = line.split(",")
    time_s = parts[1]
    lat, lat_h = parts[3], parts[4]
    lon, lon_h = parts[5], parts[6]
    spd = parts[7]
    crs = parts[8]
    date_s = parts[9]

    GNSS.latitude_deg  = nmea_latlon_to_deg(lat, lat_h)
    GNSS.longitude_deg = nmea_latlon_to_deg(lon, lon_h)
    GNSS.speed_knots   = float(spd)
    GNSS.course_deg    = float(crs)
    GNSS.has_fix       = True

    GNSS.hour   = int(time_s[0:2])
    GNSS.minute = int(time_s[2:4])
    GNSS.second = int(time_s[4:6])
    GNSS.has_time = True

    GNSS.day   = int(date_s[0:2])
    GNSS.month = int(date_s[2:4])
    GNSS.year  = 2000 + int(date_s[4:6])
    GNSS.has_date = True

def parse_gga(line: str) -> None:
    parts = line.split(",")
    GNSS.fix_quality = int(parts[6])
    GNSS.satellites  = int(parts[7])
    GNSS.altitude_m  = float(parts[9])
    GNSS.has_fix     = GNSS.fix_quality > 0

def parse_gsa(line: str) -> None:
    parts = line.split(",")
    GNSS.fix_type = int(parts[2])

def extract_discipline(line: str) -> None:
    parts = line.split(",")
    GNSS.discipline_mode = parts[1]
    GNSS.has_discipline = True

def ingest_line(line: str) -> None:
    GNSS.last_sentence = line
    GNSS.last_rx_ts = time.time()

    publish_sentence(line)

    if line.startswith("$PERDCRZ,"):
        GNSS.last_crz = line
        extract_discipline(line)
        if "$GNRMC," in line:
            rmc = line[line.index("$GNRMC,"):]
            GNSS.last_rmc = rmc
            parse_rmc(rmc)
        return

    if line.startswith( ("$GNRMC,", "$GPRMC,") ):
        GNSS.last_rmc = line
        parse_rmc(line)
    elif line.startswith( ("$GNGGA,", "$GPGGA,") ):
        GNSS.last_gga = line
        parse_gga(line)
    elif line.startswith("$GNGSA,"):
        GNSS.last_gsa = line
        parse_gsa(line)
    elif line.startswith("$GPZDA,"):
        GNSS.last_zda = line
    elif line.startswith( ("$GPGSV,", "$GAGSV,") ):
        GNSS.last_gsv = line
    elif line.startswith("$PERDCRW,"):
        GNSS.last_crw = line
    elif line.startswith("$PERDCRX,"):
        GNSS.last_crx = line

def gnss_reader() -> None:
    try:
        with serial.Serial(GNSS_DEVICE, GNSS_BAUD, timeout=1) as ser:
            buf = ""
            while True:
                c = ser.read(1)
                if not c:
                    continue
                ch = c.decode(errors="ignore")
                if ch == "\n":
                    line = buf.strip()
                    buf = ""
                    if line:
                        ingest_line(line)
                elif ch != "\r":
                    buf += ch
    except Exception:
        import logging
        logging.exception("[gnss_reader] unhandled exception")

# ------------------------------------------------------------------
# Command handlers
# ------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:
    p: Dict[str, object] = {}

    if GNSS.has_time:
        p["time"] = f"{GNSS.hour:02d}:{GNSS.minute:02d}:{GNSS.second:02d}"
    if GNSS.has_date:
        p["date"] = f"{GNSS.year:04d}-{GNSS.month:02d}-{GNSS.day:02d}"
    if not math.isnan(GNSS.latitude_deg):
        p["latitude_deg"]  = GNSS.latitude_deg
        p["longitude_deg"] = GNSS.longitude_deg
    if not math.isnan(GNSS.altitude_m):
        p["altitude_m"] = GNSS.altitude_m
    if GNSS.has_discipline:
        p["discipline"] = GNSS.discipline_mode

    p["raw"] = {
        k: v for k, v in {
            "rmc": GNSS.last_rmc,
            "gga": GNSS.last_gga,
            "gsa": GNSS.last_gsa,
            "zda": GNSS.last_zda,
            "crz": GNSS.last_crz,
        }.items() if v
    }

    return {"success": True, "message": "OK", "payload": p}

def cmd_publish(_: Optional[dict]) -> Dict:
    payload: Payload = {}
    payload["alpha"] = "GNSS"
    payload["beta"] = "News"
    payload["gamma"] = "Feed"
    logging.info("🚀 [gnss] publishing %s: ", payload)
    publish("GNSS_NEWS_FEED", payload)
    return {
        "success": True,
        "message": "OK",
        "payload": { "implications": "Nothing short of astounding." },
    }

# ---------------------------------------------------------------------
# Publish surface
# ---------------------------------------------------------------------

def on_message(payload: Payload) -> None:
    logging.info("🚀 [gnss] received message on topic %s", payload)

COMMANDS = {
    "REPORT": cmd_report,
    "PUBLISH": cmd_publish
}

SUBSCRIPTIONS = {
    "EVENTS": on_message
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    try:
        threading.Thread(
            target=gnss_reader,
            daemon=True,
        ).start()

        threading.Thread(
            target=stream_server,
            daemon=True,
        ).start()

        server_setup(
            subsystem="GNSS",
            commands=COMMANDS,
            subscriptions=SUBSCRIPTIONS
        )

    except Exception:
        import logging
        logging.exception("💥 [gnss] unhandled exception in main thread")

if __name__ == "__main__":
    run()
