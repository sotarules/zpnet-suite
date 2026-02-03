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

import math
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Set

import serial

from zpnet.processes.processes import server_setup
from zpnet.shared.logger import setup_logging

GNSS_DEVICE = os.environ.get("ZPNET_GNSS_PORT", "/dev/zpnet-gnss-serial")
GNSS_BAUD   = int(os.environ.get("ZPNET_GNSS_BAUD", "38400"))

STREAM_SOCKET_PATH = "/tmp/zpnet-gnss-stream.sock"

# ------------------------------------------------------------------
# State
# ------------------------------------------------------------------

@dataclass
class GnssState:
    # Time / date
    has_time: bool = False
    has_date: bool = False
    hour: int = 0
    minute: int = 0
    second: int = 0
    year: int = 0
    month: int = 0
    day: int = 0

    # Fix / quality
    has_fix: bool = False
    fix_quality: int = 0
    fix_type: int = 0
    fix_mode: str = ""
    satellites: int = 0
    hdop: float = math.nan

    # Position
    latitude_deg: float = math.nan
    longitude_deg: float = math.nan

    altitude_m: float = math.nan
    geoid_sep_m: float = math.nan
    ellipsoid_height_m: float = math.nan

    # Motion
    speed_knots: float = math.nan
    course_deg: float = math.nan

    # Discipline
    has_discipline: bool = False
    discipline_mode: str = ""

    # Raw sentences
    last_rmc: str = ""
    last_gga: str = ""
    last_gns: str = ""
    last_gsa: str = ""
    last_zda: str = ""
    last_crz: str = ""

    last_rx_ts: float = 0.0

GNSS = GnssState()

_stream_clients: Set[socket.socket] = set()
_stream_lock = threading.Lock()

# ------------------------------------------------------------------
# Streaming
# ------------------------------------------------------------------

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

# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def nmea_latlon_to_deg(ddmm: str, hemi: str) -> float:
    v = float(ddmm)
    deg = math.floor(v / 100.0)
    minutes = v - deg * 100.0
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out

def recompute_ellipsoid_height() -> None:
    if not math.isnan(GNSS.altitude_m) and not math.isnan(GNSS.geoid_sep_m):
        GNSS.ellipsoid_height_m = GNSS.altitude_m + GNSS.geoid_sep_m

# ------------------------------------------------------------------
# Sentence parsers
# ------------------------------------------------------------------

def parse_rmc(line: str) -> None:
    parts = line.split(",")

    if parts[2] != "A":
        return

    time_s = parts[1]
    date_s = parts[9]

    GNSS.latitude_deg  = nmea_latlon_to_deg(parts[3], parts[4])
    GNSS.longitude_deg = nmea_latlon_to_deg(parts[5], parts[6])
    GNSS.speed_knots   = float(parts[7])
    GNSS.course_deg   = float(parts[8])

    GNSS.hour   = int(time_s[0:2])
    GNSS.minute = int(time_s[2:4])
    GNSS.second = int(time_s[4:6])
    GNSS.has_time = True

    GNSS.day   = int(date_s[0:2])
    GNSS.month = int(date_s[2:4])
    GNSS.year  = 2000 + int(date_s[4:6])
    GNSS.has_date = True

    GNSS.has_fix = True

def parse_gga(line: str) -> None:
    parts = line.split(",")
    GNSS.fix_quality = int(parts[6])
    GNSS.satellites  = int(parts[7])
    GNSS.altitude_m  = float(parts[9])
    GNSS.has_fix     = GNSS.fix_quality > 0
    recompute_ellipsoid_height()

def parse_gns(line: str) -> None:
    parts = line.split(",")

    GNSS.latitude_deg  = nmea_latlon_to_deg(parts[2], parts[3])
    GNSS.longitude_deg = nmea_latlon_to_deg(parts[4], parts[5])

    GNSS.fix_mode   = parts[6]
    GNSS.satellites = int(parts[7])
    GNSS.hdop       = float(parts[8])

    GNSS.altitude_m  = float(parts[9])
    GNSS.geoid_sep_m = float(parts[10])

    GNSS.has_fix = True
    recompute_ellipsoid_height()

def parse_gsa(line: str) -> None:
    parts = line.split(",")
    GNSS.fix_type = int(parts[2])

def extract_discipline(line: str) -> None:
    GNSS.discipline_mode = line.split(",")[1]
    GNSS.has_discipline = True

# ------------------------------------------------------------------
# Ingest
# ------------------------------------------------------------------

def ingest_line(line: str) -> None:
    GNSS.last_rx_ts = time.time()
    publish_sentence(line)

    if line.startswith("$PERDCRZ,"):
        GNSS.last_crz = line
        extract_discipline(line)
        return

    if line.startswith(("$GNRMC,", "$GPRMC,")):
        GNSS.last_rmc = line
        parse_rmc(line)
    elif line.startswith(("$GNGNS,", "$GPGNS,")):
        GNSS.last_gns = line
        parse_gns(line)
    elif line.startswith(("$GNGGA,", "$GPGGA,")):
        GNSS.last_gga = line
        parse_gga(line)
    elif line.startswith("$GNGSA,"):
        GNSS.last_gsa = line
        parse_gsa(line)
    elif line.startswith("$GPZDA,"):
        GNSS.last_zda = line

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
                    if buf:
                        ingest_line(buf.strip())
                        buf = ""
                elif ch != "\r":
                    buf += ch
    except Exception:
        import logging
        logging.exception("[gnss_reader] unhandled exception")

# ------------------------------------------------------------------
# Lock-quality derivation
# ------------------------------------------------------------------

def derive_lock_quality() -> str:
    if not (GNSS.has_time and GNSS.has_date and GNSS.last_zda):
        return "WEAK"

    score = 0

    if GNSS.satellites >= 18:
        score += 2
    elif GNSS.satellites >= 12:
        score += 1

    if not math.isnan(GNSS.hdop):
        if GNSS.hdop <= 0.6:
            score += 2
        elif GNSS.hdop <= 1.5:
            score += 1

    if GNSS.fix_mode.count("D") >= 2:
        score += 1

    if GNSS.has_discipline:
        try:
            if int(GNSS.discipline_mode[-1]) >= 4:
                score += 2
        except Exception:
            pass

    if score >= 6:
        return "STRONG"
    if score >= 3:
        return "MEDIUM"
    return "WEAK"

# ------------------------------------------------------------------
# Command handlers
# ------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:
    lock_quality = derive_lock_quality()

    p: Dict[str, object] = {
        "time_valid": GNSS.has_time and GNSS.has_date and bool(GNSS.last_zda),
        "pps_valid":  GNSS.has_discipline and GNSS.discipline_mode.startswith("TPS"),
        "lock_quality": lock_quality,
    }

    if GNSS.has_time:
        p["time"] = f"{GNSS.hour:02d}:{GNSS.minute:02d}:{GNSS.second:02d}"
    if GNSS.has_date:
        p["date"] = f"{GNSS.year:04d}-{GNSS.month:02d}-{GNSS.day:02d}"

    if not math.isnan(GNSS.latitude_deg):
        p["latitude_deg"]  = GNSS.latitude_deg
        p["longitude_deg"] = GNSS.longitude_deg

    if not math.isnan(GNSS.altitude_m):
        p["altitude_m"] = GNSS.altitude_m
    if not math.isnan(GNSS.geoid_sep_m):
        p["geoid_sep_m"] = GNSS.geoid_sep_m
    if not math.isnan(GNSS.ellipsoid_height_m):
        p["ellipsoid_height_m"] = GNSS.ellipsoid_height_m

    if GNSS.satellites:
        p["satellites"] = GNSS.satellites
    if not math.isnan(GNSS.hdop):
        p["hdop"] = GNSS.hdop

    if GNSS.fix_mode:
        p["fix_mode"] = GNSS.fix_mode

    if GNSS.has_discipline:
        p["discipline"] = GNSS.discipline_mode

    p["raw"] = {
        k: v for k, v in {
            "rmc": GNSS.last_rmc,
            "gns": GNSS.last_gns,
            "gga": GNSS.last_gga,
            "gsa": GNSS.last_gsa,
            "zda": GNSS.last_zda,
            "crz": GNSS.last_crz,
        }.items() if v
    }

    return {"success": True, "message": "OK", "payload": p}

COMMANDS = {
    "REPORT": cmd_report,
}

# ------------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------------

def run() -> None:
    setup_logging()
    try:
        threading.Thread(target=gnss_reader, daemon=True).start()
        threading.Thread(target=stream_server, daemon=True).start()
        server_setup(subsystem="GNSS", commands=COMMANDS)
    except Exception:
        import logging
        logging.exception("💥 [gnss] unhandled exception in main thread")

if __name__ == "__main__":
    run()