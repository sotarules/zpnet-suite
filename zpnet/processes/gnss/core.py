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
from dataclasses import dataclass, field
from typing import Dict, Optional, Set, TextIO

import serial

from zpnet.processes.processes import server_setup
from zpnet.shared.logger import setup_logging

GNSS_DEVICE = os.environ.get("ZPNET_GNSS_PORT", "/dev/zpnet-gnss-serial")
GNSS_BAUD   = int(os.environ.get("ZPNET_GNSS_BAUD", "38400"))

STREAM_SOCKET_PATH = "/tmp/zpnet-gnss-stream.sock"

# ------------------------------------------------------------------
# Logging
# ------------------------------------------------------------------

GNSS_LOG_PATH = "/home/mule/zpnet/logs/zpnet-gnss.log"

gnss_log_fh: Optional[TextIO] = None

def open_gnss_log() -> None:
    global gnss_log_fh
    if gnss_log_fh:
        gnss_log_fh.close()
    os.makedirs(os.path.dirname(GNSS_LOG_PATH), exist_ok=True)
    gnss_log_fh = open(GNSS_LOG_PATH, "w", buffering=1)
    logging.info("📝 [open_gnss_log] gnss log opened at %s", GNSS_LOG_PATH)

def log_gnss(line: str) -> None:
    if gnss_log_fh:
        gnss_log_fh.write(line + "\n")

# ------------------------------------------------------------------
# Frequency mode names (TPS4 field 2)
# ------------------------------------------------------------------

FREQ_MODE_NAMES = {
    0: "WARMUP",
    1: "PULLIN",
    2: "COARSE_LOCK",
    3: "FINE_LOCK",
    4: "HOLDOVER",
    5: "OUT_OF_HOLDOVER",
}

# PPS sync status names (TPS1 field 7)
PPS_STATUS_NAMES = {
    0: "RTC",
    1: "GPS",
    2: "UTC_USNO",
    3: "UTC_SU",
    4: "UTC_EU",
    5: "UTC_NICT",
}

# Time status names (TPS1 field 3)
TIME_STATUS_NAMES = {
    0: "BEFORE_FIX",
    1: "LS_UNKNOWN",
    2: "LS_FIX",
}

# Position mode names (TPS3 field 2)
POS_MODE_NAMES = {
    0: "NAV",
    1: "SS",
    2: "CSS",
    3: "TO",
}

# TRAIM solution names (TPS3 field 7)
TRAIM_NAMES = {
    0: "OK",
    1: "ALARM",
    2: "INSUFFICIENT",
}

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

    # Position acquisition (NOT a validated fix)
    has_position: bool = False
    position_quality: int = 0      # GGA quality indicator
    position_type: int = 0         # GSA fix type
    position_mode: str = ""        # GNS mode string
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

    # Discipline — basic flag (kept for backward compat)
    has_discipline: bool = False
    discipline_mode: str = ""

    # ---------------------------------------------------------------
    # TPS4 (CRZ) — VCLK Frequency and Control
    # ---------------------------------------------------------------
    tps4_freq_mode: int = -1               # 0-5
    tps4_freq_mode_name: str = ""
    tps4_phase_skip: int = -1              # 0=auto, 1=execute
    tps4_alarm: int = 0                    # bitmask
    tps4_status: int = 0                   # bitmask
    tps4_pps_timing_error_ns: int = 0      # nsec offset from sync target
    tps4_freq_error_ppb: int = 0           # VCLK deviation from nominal
    tps4_learning_time_s: int = 0          # seconds in fine-lock learning
    tps4_holdover_avail_s: int = 0         # holdover seconds remaining

    # ---------------------------------------------------------------
    # TPS1 (CRW) — Time and Leap Second
    # ---------------------------------------------------------------
    tps1_time_status: int = -1             # 0=before fix, 1=LS unknown, 2=LS fix
    tps1_time_status_name: str = ""
    tps1_pps_status: int = -1              # 0=RTC .. 5=UTC(NICT)
    tps1_pps_status_name: str = ""
    tps1_clock_drift_ppb: float = math.nan # 26MHz TCXO drift
    tps1_temperature_c: float = math.nan   # ambient temp * 100 → degrees C
    tps1_present_ls: int = 0               # current leap second
    tps1_future_ls: int = 0                # upcoming leap second

    # ---------------------------------------------------------------
    # TPS2 (CRX) — PPS Information
    # ---------------------------------------------------------------
    tps2_pps_on: bool = False
    tps2_pps_mode: int = -1                # 0-3
    tps2_pulse_width_ms: int = 0
    tps2_cable_delay_ns: int = 0
    tps2_polarity: int = 0                 # 0=rising, 1=falling
    tps2_estimated_accuracy_ns: int = 9999

    # ---------------------------------------------------------------
    # TPS3 (CRY) — Position Mode & TRAIM
    # ---------------------------------------------------------------
    tps3_pos_mode: int = -1                # 0=NAV,1=SS,2=CSS,3=TO
    tps3_pos_mode_name: str = ""
    tps3_pos_diff_m: int = 0               # fixed vs calculated position diff
    tps3_survey_count: int = 0             # self-survey update count
    tps3_survey_threshold: int = 0
    tps3_traim_solution: int = 2           # 0=OK,1=ALARM,2=insufficient
    tps3_traim_name: str = "INSUFFICIENT"
    tps3_traim_removed_svs: int = 0
    tps3_receiver_status: int = 0          # bitmask

    # Raw sentences
    last_rmc: str = ""
    last_gga: str = ""
    last_gns: str = ""
    last_gsa: str = ""
    last_zda: str = ""
    last_crz: str = ""
    last_crw: str = ""
    last_crx: str = ""
    last_cry: str = ""

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

def maybe_int(s: str) -> Optional[int]:
    return int(s) if s else None

def maybe_float(s: str) -> Optional[float]:
    return float(s) if s else None

def nmea_latlon_to_deg(ddmm: str, hemi: str) -> float:
    v = float(ddmm)
    deg = math.floor(v / 100.0)
    minutes = v - deg * 100.0
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out

def recompute_ellipsoid_height() -> None:
    if not math.isnan(GNSS.altitude_m) and not math.isnan(GNSS.geoid_sep_m):
        GNSS.ellipsoid_height_m = GNSS.altitude_m + GNSS.geoid_sep_m

def strip_checksum(field: str) -> str:
    """Remove *XX checksum suffix from the last field of an NMEA sentence."""
    idx = field.find("*")
    return field[:idx] if idx >= 0 else field

# ------------------------------------------------------------------
# Sentence parsers — standard NMEA
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
    GNSS.course_deg    = float(parts[8])

    GNSS.hour   = int(time_s[0:2])
    GNSS.minute = int(time_s[2:4])
    GNSS.second = int(time_s[4:6])
    GNSS.has_time = True

    GNSS.day   = int(date_s[0:2])
    GNSS.month = int(date_s[2:4])
    GNSS.year  = 2000 + int(date_s[4:6])
    GNSS.has_date = True

    GNSS.has_position = True

def parse_gga(line: str) -> None:
    parts = line.split(",")

    GNSS.position_quality = int(parts[6]) if parts[6] else 0
    GNSS.satellites       = int(parts[7]) if parts[7] else 0

    alt = maybe_float(parts[9])
    if alt is not None:
        GNSS.altitude_m = alt

    GNSS.has_position = GNSS.position_quality > 0
    recompute_ellipsoid_height()

def parse_gns(line: str) -> None:
    parts = line.split(",")

    GNSS.latitude_deg  = nmea_latlon_to_deg(parts[2], parts[3])
    GNSS.longitude_deg = nmea_latlon_to_deg(parts[4], parts[5])

    GNSS.position_mode = parts[6]

    sats = maybe_int(parts[7])
    if sats is not None:
        GNSS.satellites = sats

    hdop = maybe_float(parts[8])
    if hdop is not None:
        GNSS.hdop = hdop

    alt = maybe_float(parts[9])
    if alt is not None:
        GNSS.altitude_m = alt

    sep = maybe_float(parts[10])
    if sep is not None:
        GNSS.geoid_sep_m = sep

    GNSS.has_position = bool(GNSS.position_mode and GNSS.position_mode != "N")
    recompute_ellipsoid_height()

def parse_gsa(line: str) -> None:
    parts = line.split(",")
    if parts[2]:
        GNSS.position_type = int(parts[2])

# ------------------------------------------------------------------
# Sentence parsers — Furuno proprietary TPS sentences
# ------------------------------------------------------------------

def parse_crz(line: str) -> None:
    """TPS4 — VCLK Frequency and Control.

    $PERDCRZ,TPS4,<freq_mode>,<phase_skip>,<alarm>,<status>,
             <pps_timing_error>,<freq_error>,<reserve>,
             <learning_time>,<available_time>,<reserve>*XX
    """
    parts = line.split(",")
    GNSS.discipline_mode = parts[1]          # "TPS4"
    GNSS.has_discipline = True

    try:
        GNSS.tps4_freq_mode      = int(parts[2])
        GNSS.tps4_freq_mode_name = FREQ_MODE_NAMES.get(GNSS.tps4_freq_mode, f"UNKNOWN({GNSS.tps4_freq_mode})")
        GNSS.tps4_phase_skip     = int(parts[3])
        GNSS.tps4_alarm          = int(parts[4], 16)
        GNSS.tps4_status         = int(parts[5], 16)
        GNSS.tps4_pps_timing_error_ns = int(parts[6])
        GNSS.tps4_freq_error_ppb = int(parts[7])
        # parts[8] is reserved
        GNSS.tps4_learning_time_s = int(parts[9])
        GNSS.tps4_holdover_avail_s = int(strip_checksum(parts[10]))
    except (IndexError, ValueError) as e:
        logging.warning("[parse_crz] failed to parse TPS4: %s — %s", e, line)

def parse_crw(line: str) -> None:
    """TPS1 — Time and Leap Second.

    $PERDCRW,TPS1,<datetime>,<time_status>,<update_date>,<present_ls>,
             <future_ls>,<pps_status>,<drift>,<temperature>*XX
    """
    parts = line.split(",")
    try:
        GNSS.tps1_time_status      = int(parts[3])
        GNSS.tps1_time_status_name = TIME_STATUS_NAMES.get(GNSS.tps1_time_status, f"UNKNOWN({GNSS.tps1_time_status})")
        GNSS.tps1_present_ls       = int(parts[5])
        GNSS.tps1_future_ls        = int(parts[6])
        GNSS.tps1_pps_status       = int(parts[7])
        GNSS.tps1_pps_status_name  = PPS_STATUS_NAMES.get(GNSS.tps1_pps_status, f"UNKNOWN({GNSS.tps1_pps_status})")
        GNSS.tps1_clock_drift_ppb  = float(parts[8])
        temp_raw = strip_checksum(parts[9])
        GNSS.tps1_temperature_c    = int(temp_raw) / 100.0
    except (IndexError, ValueError) as e:
        logging.warning("[parse_crw] failed to parse TPS1: %s — %s", e, line)

def parse_crx(line: str) -> None:
    """TPS2 — PPS Information.

    $PERDCRX,TPS2,<pps_out>,<pps_mode>,<period>,<pulse_width>,<cable_delay>,
             <polarity>,<pps_type>,<est_accuracy>,<r1>,<r2>,<r3>,<r4>*XX
    """
    parts = line.split(",")
    try:
        GNSS.tps2_pps_on              = parts[2] == "1"
        GNSS.tps2_pps_mode            = int(parts[3])
        GNSS.tps2_pulse_width_ms      = int(parts[5])
        GNSS.tps2_cable_delay_ns      = int(parts[6])
        GNSS.tps2_polarity            = int(parts[7])
        GNSS.tps2_estimated_accuracy_ns = int(parts[9])
    except (IndexError, ValueError) as e:
        logging.warning("[parse_crx] failed to parse TPS2: %s — %s", e, line)

def parse_cry(line: str) -> None:
    """TPS3 — Position Mode & TRAIM.

    $PERDCRY,TPS3,<pos_mode>,<pos_diff>,<sigma_thresh>,<time>,<time_thresh>,
             <traim_solution>,<traim_status>,<removed_svs>,<rx_status>,<reserve>*XX
    """
    parts = line.split(",")
    try:
        GNSS.tps3_pos_mode      = int(parts[2])
        GNSS.tps3_pos_mode_name = POS_MODE_NAMES.get(GNSS.tps3_pos_mode, f"UNKNOWN({GNSS.tps3_pos_mode})")
        GNSS.tps3_pos_diff_m    = int(parts[3])
        GNSS.tps3_survey_count  = int(parts[5])
        GNSS.tps3_survey_threshold = int(parts[6])
        GNSS.tps3_traim_solution = int(parts[7])
        GNSS.tps3_traim_name    = TRAIM_NAMES.get(GNSS.tps3_traim_solution, f"UNKNOWN({GNSS.tps3_traim_solution})")
        GNSS.tps3_traim_removed_svs = int(parts[9])
        rx_str = strip_checksum(parts[10])
        if rx_str.startswith("0x"):
            GNSS.tps3_receiver_status = int(rx_str, 16)
        else:
            GNSS.tps3_receiver_status = int(rx_str)
    except (IndexError, ValueError) as e:
        logging.warning("[parse_cry] failed to parse TPS3: %s — %s", e, line)


# ------------------------------------------------------------------
# Ingest
# ------------------------------------------------------------------

def ingest_line(line: str) -> None:
    log_gnss(line)
    GNSS.last_rx_ts = time.time()
    publish_sentence(line)

    if line.startswith("$PERDCRZ,"):
        GNSS.last_crz = line
        parse_crz(line)
        return

    if line.startswith("$PERDCRW,"):
        GNSS.last_crw = line
        parse_crw(line)
        return

    if line.startswith("$PERDCRX,"):
        GNSS.last_crx = line
        parse_crx(line)
        return

    if line.startswith("$PERDCRY,"):
        GNSS.last_cry = line
        parse_cry(line)
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

    if GNSS.position_mode.count("D") >= 2:
        score += 1

    if GNSS.has_discipline:
        # FINE_LOCK (3) or HOLDOVER (4) are the good states
        if GNSS.tps4_freq_mode >= 3:
            score += 2
        elif GNSS.tps4_freq_mode >= 2:
            score += 1

    if score >= 6:
        return "STRONG"
    if score >= 3:
        return "MEDIUM"
    return "WEAK"

# ------------------------------------------------------------------
# Shared payload for REPORT and publish
# ------------------------------------------------------------------

def get_gnss_payload(_: Optional[dict]) -> Dict:

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

    if GNSS.position_mode:
        p["position_mode"] = GNSS.position_mode

    # -----------------------------------------------------------
    # Discipline block — full TPS4 state
    # -----------------------------------------------------------
    if GNSS.has_discipline:
        p["discipline"] = {
            "sentence":           GNSS.discipline_mode,       # "TPS4"
            "freq_mode":          GNSS.tps4_freq_mode,        # 0-5
            "freq_mode_name":     GNSS.tps4_freq_mode_name,   # "FINE_LOCK" etc
            "phase_skip":         GNSS.tps4_phase_skip,
            "alarm":              GNSS.tps4_alarm,
            "status":             GNSS.tps4_status,
            "pps_timing_error_ns": GNSS.tps4_pps_timing_error_ns,
            "freq_error_ppb":     GNSS.tps4_freq_error_ppb,
            "learning_time_s":    GNSS.tps4_learning_time_s,
            "holdover_avail_s":   GNSS.tps4_holdover_avail_s,
        }

    # -----------------------------------------------------------
    # Clock block — TPS1 time/drift/temperature
    # -----------------------------------------------------------
    clock: Dict[str, object] = {}
    if GNSS.tps1_time_status >= 0:
        clock["time_status"]      = GNSS.tps1_time_status
        clock["time_status_name"] = GNSS.tps1_time_status_name
    if GNSS.tps1_pps_status >= 0:
        clock["pps_sync"]         = GNSS.tps1_pps_status_name
    if not math.isnan(GNSS.tps1_clock_drift_ppb):
        clock["drift_ppb"]        = GNSS.tps1_clock_drift_ppb
    if not math.isnan(GNSS.tps1_temperature_c):
        clock["temperature_c"]    = GNSS.tps1_temperature_c
    if GNSS.tps1_present_ls:
        clock["leap_second"]      = GNSS.tps1_present_ls
    if clock:
        p["clock"] = clock

    # -----------------------------------------------------------
    # PPS block — TPS2 configuration and estimated accuracy
    # -----------------------------------------------------------
    if GNSS.tps2_pps_mode >= 0:
        p["pps"] = {
            "active":              GNSS.tps2_pps_on,
            "mode":                GNSS.tps2_pps_mode,
            "pulse_width_ms":      GNSS.tps2_pulse_width_ms,
            "cable_delay_ns":      GNSS.tps2_cable_delay_ns,
            "polarity":            "rising" if GNSS.tps2_polarity == 0 else "falling",
            "estimated_accuracy_ns": GNSS.tps2_estimated_accuracy_ns,
        }

    # -----------------------------------------------------------
    # Integrity block — TPS3 position mode, TRAIM, survey
    # -----------------------------------------------------------
    if GNSS.tps3_pos_mode >= 0:
        integrity: Dict[str, object] = {
            "pos_mode":          GNSS.tps3_pos_mode_name,
            "pos_diff_m":        GNSS.tps3_pos_diff_m,
            "traim":             GNSS.tps3_traim_name,
            "traim_removed_svs": GNSS.tps3_traim_removed_svs,
        }
        if GNSS.tps3_pos_mode in (1, 2):   # SS or CSS — survey in progress
            integrity["survey_count"]     = GNSS.tps3_survey_count
            integrity["survey_threshold"] = GNSS.tps3_survey_threshold
        if GNSS.tps3_receiver_status:
            integrity["receiver_status_hex"] = f"0x{GNSS.tps3_receiver_status:08X}"
        p["integrity"] = integrity

    # -----------------------------------------------------------
    # Raw sentences
    # -----------------------------------------------------------
    p["raw"] = {
        k: v for k, v in {
            "rmc": GNSS.last_rmc,
            "gns": GNSS.last_gns,
            "gga": GNSS.last_gga,
            "gsa": GNSS.last_gsa,
            "zda": GNSS.last_zda,
            "crz": GNSS.last_crz,
            "crw": GNSS.last_crw,
            "crx": GNSS.last_crx,
            "cry": GNSS.last_cry,
        }.items() if v
    }
    return p

# ------------------------------------------------------------------
# Command handlers
# ------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:

    payload = get_gnss_payload(None)
    return {"success": True, "message": "OK", "payload": payload}

COMMANDS = {
    "REPORT": cmd_report,
}

# ------------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------------

def run() -> None:
    setup_logging()
    open_gnss_log()
    try:
        threading.Thread(target=gnss_reader, daemon=True).start()
        threading.Thread(target=stream_server, daemon=True).start()
        server_setup(subsystem="GNSS", commands=COMMANDS)
    except Exception:
        logging.exception("💥 [gnss] unhandled exception in main thread")

if __name__ == "__main__":
    run()