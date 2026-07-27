"""
ZPNet SYSTEM Process (Pi-side, authoritative aggregator)

Responsibilities:
  • Query authoritative SYSTEM state from the Teensy
  • Collect Raspberry Pi host metrics
  • Maintain last-known-good SYSTEM snapshot
  • Expose SYSTEM.REPORT
  • Emit SYSTEM_STATUS events on fixed cadence
  • Publish FEATURE_STATUS whenever readiness state changes
  • Republish FEATURE_STATUS periodically even when quiescent

Process model:
  • One systemd service
  • One polling thread
  • One blocking command socket
  • One FEATURE_STATUS_FRAGMENT subscription from Teensy SYSTEM
"""

from __future__ import annotations

import datetime
import logging
import os
import platform
import random
import socket
import string
import subprocess
import threading
import time
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Optional, Tuple

import psutil
import requests
from smbus2 import SMBus

from zpnet.processes.processes import publish, send_command, server_setup
from zpnet.shared.constants import (
    ZPNET_REMOTE_HOST,
    HTTP_TIMEOUT,
    DEFINITIVE_TEST_TIMEOUT_S,
    DEFINITIVE_TEST_HOST,
)
from zpnet.shared.db import open_db
from zpnet.shared.events import create_event
from zpnet.shared.http import gzip_text
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import (
    feature_get,
    feature_status,
    normalize_feature_status,
    normalize_payload,
    normalize_ts,
)

# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

POLL_INTERVAL_SEC = 30
FEATURE_STATUS_PUBLISH_INTERVAL_SEC = 10
STARTUP_TEENSY_QUIET_DELAY_S = 10.0

# ------------------------------------------------------------------
# Raspberry Pi status configuration
# ------------------------------------------------------------------

CPU_TEMP_PATHS = [
    Path("/sys/class/thermal/thermal_zone0/temp"),
    Path("/sys/class/hwmon/hwmon0/temp1_input"),
]

DEVICE_NAME = platform.node() or "raspberrypi"

# ------------------------------------------------------------------
# Hardware constants
# ------------------------------------------------------------------

LASER_ADDR = 0x66

REGS = {
    "CTL0":      0x00,
    "CTL1":      0x01,
    "CTL2":      0x03,
    "ID1_MSB":   0x07,
    "ID1_LSB":   0x08,
    "STATUS1":   0x2E,
    "STATUS2":   0x2F,
    "STATUS3":   0x30,
    "STATUS4":   0x31,
    "INT":       0x32,
}

SYSEN_BIT   = 0x80           # CTL0.D7
ID_EN_BIT   = 0x80           # CTL1.D7
ID1_EN_BIT  = 0x08           # CTL1.D3

ID1_FLG_BIT = 0x02           # STATUS1
VIN2_UV_BIT = 0x02           # STATUS3
LD_ON_BIT   = 0x08           # STATUS3

ID_SCP_BIT  = 0x10           # INT
OT_WARN_BIT = 0x02           # INT
OT_SHDN_BIT = 0x01           # INT

# ------------------------------------------------------------------
# I2C BUS IDENTITY (authoritative)
# ------------------------------------------------------------------

I2C_BUS_LEGACY = 1          # SMBUS1 — hardware I2C (SDA/SCL)
I2C_BUS_EXPANDED = 2        # SMBUS2 — i2c-gpio on GPIO23/24

# ------------------------------------------------------------------
# I2C ADDRESSES THAT ARE REAL BUT NOT SENSORS (must be ignored)
# ------------------------------------------------------------------
# Many DS3231 breakout boards include an AT24Cxx EEPROM at 0x57
# alongside the RTC at 0x68. It will appear on any bus that hosts
# such an RTC module.
I2C_IGNORE_ADDRS = {
    0x57,  # AT24Cxx EEPROM (RTC companion)
}

# ------------------------------------------------------------------
# SENSOR INVENTORY BY BUS (explicit, no hidden assumptions)
# ------------------------------------------------------------------

I2C_SENSORS_BY_BUS = {
    I2C_BUS_LEGACY: {

        0x40: "INA260 0x40 (3.3v Rail)",
        0x41: "INA260 0x41 (5v Rail+",
        0x44: "INA260 0x44 (12v Rail)",
        0x66: "EV5491 0x66 (Laser Controller)",
        0x76: "BME280 0x76 (Environment)",
        0x4C: "AD5693R 0x4C (OCXO1 DAC)",
        0x4E: "AD5693R 0x4E (OCXO2 DAC)",
        # 0x57 will appear via scan but is ignored (EEPROM)
    },

    I2C_BUS_EXPANDED: {
        0x40: "INA260 0x40 (Pi Domain)",
        0x41: "INA260 0x41 (OCXO1 Domain)",
        0x44: "INA260 0x44 (OCXO1 Domain)",
        0x45: "INA260 0x45 (Teensy Domain)"
        # 0x57 will appear via scan but is ignored (EEPROM)
    },
}

# ------------------------------------------------------------------
# Power monitoring (legacy: power_monitor)
# ------------------------------------------------------------------

POWER_CONFIG_BY_BUS = {
    I2C_BUS_LEGACY: {
        0x44: {"label": "Battery", "ideal_voltage_v": 12.8},
        0x40: {"label": "3.3v Rail", "ideal_voltage_v": 3.3},
        0x41: {"label": "5v Rail", "ideal_voltage_v": 5.0},
    },
    I2C_BUS_EXPANDED: {
        0x40: {"label": "Pi Domain", "ideal_voltage_v": 5.0},
        0x45: {"label": "Teensy Domain", "ideal_voltage_v": 5.0},
        0x44: {"label": "OCXO1 Domain", "ideal_voltage_v": 5.0},
        0x41: {"label": "OCXO2 Domain", "ideal_voltage_v": 5.0},
    }
}

BME280_ID_REG = 0xD0
BME280_EXPECTED_ID = 0x60

BME280_ADDR   = 0x76

REG_ID        = 0xD0
REG_CTRL_HUM  = 0xF2
REG_CTRL_MEAS = 0xF4
REG_DATA      = 0xF7

EXPECTED_CHIP_ID = 0x60
SEA_LEVEL_PRESSURE_HPA = 1013.25

# BME280 is on the shared legacy SMBus with the OCXO DACs.  SyncDAC keeps DAC
# writes sparse, but the BME280 calibration/data read is still a multi-register
# transaction and can occasionally lose arbitration / NACK mid-sequence.  Treat
# environmental telemetry as best-effort context: retry a few times, then
# publish a stale/error payload instead of killing the SYSTEM poller.
BME280_RETRY_COUNT = 3
BME280_RETRY_DELAY_SEC = 0.05

_BME280_CAL_CACHE: Optional[dict] = None
_BME280_LAST_GOOD: Optional[dict] = None
_BME280_READ_FAIL_COUNT = 0
_BME280_CONSECUTIVE_FAIL_COUNT = 0
_BME280_RECOVERY_COUNT = 0

REG_CURRENT = 0x01
REG_VOLTAGE = 0x02
REG_POWER   = 0x03

# INA260s share SMBus surfaces with other active devices.  Individual register
# reads are short, but sparse SyncDAC writes or other bus activity can still
# collide with a read_word_data transaction.  Treat power telemetry as
# best-effort context: retry transient I/O errors and publish stale/error rail
# payloads instead of terminating the SYSTEM poller.
INA260_RETRY_COUNT = 3
INA260_RETRY_DELAY_SEC = 0.03

_INA260_LAST_GOOD: Dict[Tuple[int, int], dict] = {}
_INA260_READ_FAIL_COUNT: Dict[Tuple[int, int], int] = {}
_INA260_CONSECUTIVE_FAIL_COUNT: Dict[Tuple[int, int], int] = {}
_INA260_RECOVERY_COUNT: Dict[Tuple[int, int], int] = {}

# ------------------------------------------------------------------
# Battery monitoring (legacy: battery_monitor)
# ------------------------------------------------------------------

BATTERY_LABEL = "Battery"
BATTERY_CAPACITY_WH = 640.0          # example — use your real value
POWER_SAMPLE_STEP = 5                # same semantics as before

# ------------------------------------------------------------------
# Authoritative SYSTEM snapshot
# ------------------------------------------------------------------

# Snapshot schema (phase 2):
#   {
#     "teensy": { ... payload from Teensy SYSTEM.REPORT ... },
#     "pi":     { ... Raspberry Pi host metrics ... }
#   }
SYSTEM: Dict[str, object] = {}

_SYSTEM_LOCK = threading.Lock()

# ------------------------------------------------------------------
# Feature status clearing house
# ------------------------------------------------------------------

FEATURE_STATUSES = {"INITIALIZING", "NOMINAL", "HOLD", "ANOMALY"}

# Public FEATURE_STATUS is a mission-readiness surface.  The raw QTimer/DWT
# interval witness remains available through Teensy INTERRUPT diagnostics, but
# its ISR-displacement-sensitive state is intentionally not an annunciator.
_PUBLIC_FEATURE_EXCLUSIONS = {
    ("TEENSY", "INTERRUPT", "QTIMER_DWT_RULER"),
}


_FEATURE_LOCK = threading.Lock()
_PI_FEATURES: Dict[str, Dict[str, Dict[str, str]]] = {"PI": {}}
_TEENSY_FEATURES: Dict[str, Dict[str, Dict[str, str]]] = {"TEENSY": {}}
_FEATURE_PUBLISH_LOCK = threading.Lock()
_LAST_PUBLISHED_FEATURE_STATUS: Dict[str, Dict[str, Dict[str, str]]] = {}


def _health_to_feature_status(health_state: Any) -> str:
    return normalize_feature_status(health_state, default="INITIALIZING")


def set_pi_feature(subsystem: str,
                   feature: str,
                   status: Any,
                   detail: str = "",
                   **extra: Any) -> str:
    """Set one Pi-authored feature status in the local SYSTEM registry."""
    subsystem_key = str(subsystem or "").strip().upper()
    feature_key = str(feature or "").strip().upper()
    if not subsystem_key or not feature_key:
        raise ValueError("subsystem and feature are required")

    value = normalize_feature_status(status)
    with _FEATURE_LOCK:
        subsystem_map = _PI_FEATURES.setdefault("PI", {}).setdefault(subsystem_key, {})
        if subsystem_map.get(feature_key) == value:
            return value
        subsystem_map[feature_key] = value

    # detail/extra are accepted for compatibility with earlier callers, but
    # the feature-state substrate is deliberately scalar-only.
    _ = detail, extra
    return value


def _copy_feature_tree(tree: Any) -> Dict[str, Dict[str, Dict[str, str]]]:
    if not isinstance(tree, dict):
        return {}

    out: Dict[str, Dict[str, Dict[str, str]]] = {}
    for machine, subsystems in tree.items():
        if not isinstance(subsystems, dict):
            continue
        machine_key = str(machine).strip().upper()
        machine_out: Dict[str, Dict[str, str]] = {}
        for subsystem, features in subsystems.items():
            if not isinstance(features, dict):
                continue
            subsystem_key = str(subsystem).strip().upper()
            subsystem_out: Dict[str, str] = {}
            for feature, entry in features.items():
                feature_key = str(feature).strip().upper()
                if not feature_key:
                    continue
                if (machine_key, subsystem_key, feature_key) in _PUBLIC_FEATURE_EXCLUSIONS:
                    continue
                if isinstance(entry, dict):
                    status = entry.get("status")
                else:
                    status = entry
                subsystem_out[feature_key] = normalize_feature_status(status)
            if subsystem_out:
                machine_out[subsystem_key] = subsystem_out
        if machine_out:
            out[machine_key] = machine_out
    return out


def _pi_feature_tree_snapshot() -> Dict[str, Dict[str, Dict[str, str]]]:
    with _FEATURE_LOCK:
        return _copy_feature_tree(_PI_FEATURES)


def _teensy_feature_tree_snapshot() -> Dict[str, Dict[str, Dict[str, str]]]:
    with _FEATURE_LOCK:
        return _copy_feature_tree(_TEENSY_FEATURES)


def _teensy_feature_tree_from_report(teensy_payload: Dict[str, Any]) -> Dict[str, Dict[str, Dict[str, str]]]:
    features = teensy_payload.get("features") if isinstance(teensy_payload, dict) else None
    tree = _copy_feature_tree(features)
    return {"TEENSY": tree["TEENSY"]} if "TEENSY" in tree else {}


def _combine_feature_trees(*trees: Any) -> Dict[str, Dict[str, Dict[str, str]]]:
    combined: Dict[str, Dict[str, Dict[str, str]]] = {}
    for tree in trees:
        for machine, subsystems in _copy_feature_tree(tree).items():
            machine_out = combined.setdefault(machine, {})
            for subsystem, features in subsystems.items():
                subsystem_out = machine_out.setdefault(subsystem, {})
                for feature, status in features.items():
                    subsystem_out[feature] = normalize_feature_status(status)
    return combined


def _replace_teensy_feature_tree(tree: Any) -> bool:
    """Install a Teensy-authored feature tree.  Returns True on real change."""
    incoming = _copy_feature_tree(tree)
    normalized = {"TEENSY": incoming["TEENSY"]} if incoming.get("TEENSY") else {"TEENSY": {}}

    with _FEATURE_LOCK:
        global _TEENSY_FEATURES
        if normalized == _TEENSY_FEATURES:
            return False
        _TEENSY_FEATURES = normalized
        return True


def _feature_tree_snapshot() -> Dict[str, Dict[str, Dict[str, str]]]:
    with _FEATURE_LOCK:
        return _combine_feature_trees(_PI_FEATURES, _TEENSY_FEATURES)


def _publish_feature_status_if_changed(features: Any = None, *, force: bool = False) -> bool:
    """
    Publish FEATURE_STATUS when the scalar feature tree changed, or when forced.

    The forced path is the quiescent-system heartbeat: subscribers that start
    after the last feature transition must still receive the current readiness
    tree without waiting for a real state change.

    Wire shape:
      {"topic":"FEATURE_STATUS","payload":{"TEENSY":{...},"PI":{...}}}
    """
    global _LAST_PUBLISHED_FEATURE_STATUS

    snapshot = _copy_feature_tree(features if features is not None else _feature_tree_snapshot())
    if not snapshot:
        return False

    with _FEATURE_PUBLISH_LOCK:
        if not force and snapshot == _LAST_PUBLISHED_FEATURE_STATUS:
            return False
        _LAST_PUBLISHED_FEATURE_STATUS = _copy_feature_tree(snapshot)

    publish("FEATURE_STATUS", snapshot)
    return True


def feature_status_periodic_publisher() -> None:
    """
    Publish FEATURE_STATUS on a fixed cadence even when no feature changes.

    Change-triggered publication remains the fast path.  This heartbeat keeps
    late subscribers, campaign preflight, and metrics taps from hanging behind
    a quiescent feature tree after the system is already nominal.
    """
    while True:
        try:
            time.sleep(FEATURE_STATUS_PUBLISH_INTERVAL_SEC)
            _publish_feature_status_if_changed(force=True)
        except Exception:
            logging.exception("[feature_status_periodic_publisher] publish failed")


def _update_builtin_pi_features(*,
                                pi_payload: Dict[str, Any],
                                network_payload: Dict[str, Any],
                                sensor_payload: Dict[str, Any],
                                environment_payload: Dict[str, Any],
                                gnss_payload: Dict[str, Any],
                                power_payload: Dict[str, Any],
                                battery_payload: Dict[str, Any],
                                teensy_features_available: bool) -> None:
    set_pi_feature("SYSTEM", "FEATURE_STATUS", "NOMINAL")
    set_pi_feature("SYSTEM", "HOST", _health_to_feature_status(pi_payload.get("health_state")))
    set_pi_feature("SYSTEM", "NETWORK", _health_to_feature_status(network_payload.get("network_status")))
    set_pi_feature("SYSTEM", "SENSORS", _health_to_feature_status(sensor_payload.get("health_state")))
    set_pi_feature("SYSTEM", "ENVIRONMENT", _health_to_feature_status(environment_payload.get("health_state")))
    set_pi_feature("GNSS", "REPORT", _health_to_feature_status(gnss_payload.get("health_state")))
    set_pi_feature("SYSTEM", "POWER", _health_to_feature_status(power_payload.get("health_state")))
    set_pi_feature("SYSTEM", "BATTERY", _health_to_feature_status(battery_payload.get("health_state")))
    set_pi_feature("SYSTEM", "TEENSY_FEATURE_IMPORT", "NOMINAL" if teensy_features_available else "INITIALIZING")


set_pi_feature("SYSTEM", "FEATURE_STATUS", "NOMINAL")

# ------------------------------------------------------------------
# ZPNet Server reachability state (speed tests only)
# ------------------------------------------------------------------
# Speed tests hit sota.ddns.net which is an unowned boundary.
# Server downtime is normal (deploys, OOM, maintenance) and must
# not fill the log with traceback spam every poll cycle.

_server_reachable = True

# ------------------------------------------------------------------
# I2C BUS RESOLUTION (single source of truth)
# ------------------------------------------------------------------

def open_i2c(bus_id: int) -> SMBus:
    """
    Open an I2C bus by logical ID.
    All SMBus access must go through here.
    """
    return SMBus(bus_id)

# ------------------------------------------------------------------
# Helper for instumenting I2C adapter names
# ------------------------------------------------------------------

def i2c_adapter_name(bus_id: int) -> str:
    try:
        return Path(f"/sys/class/i2c-dev/i2c-{bus_id}/name").read_text().strip()
    except Exception:
        return "UNKNOWN"

# ------------------------------------------------------------------
# Raspberry Pi helpers (migrated from pi_monitor)
# ------------------------------------------------------------------

def find_i2c_bus_by_name(name_fragment: str) -> int:
    for d in Path("/sys/class/i2c-dev").iterdir():
        name = (d / "name").read_text().strip()
        if name_fragment in name:
            return int(d.name.split("-")[1])
    raise RuntimeError(f"I2C bus not found: {name_fragment}")


def get_cpu_temp_c() -> float | None:
    for path in CPU_TEMP_PATHS:
        if path.exists():
            return round(float(path.read_text().strip()) / 1000.0, 1)

    temps = psutil.sensors_temperatures()
    for entries in temps.values():
        for entry in entries:
            if "cpu" in entry.label.lower() or "core" in entry.label.lower():
                return round(entry.current, 1)

    return None


def get_uptime_seconds() -> float:
    return time.time() - psutil.boot_time()


def get_load_average() -> Tuple[float, float, float]:
    one, five, fifteen = os.getloadavg()
    return (round(one, 2), round(five, 2), round(fifteen, 2))


def get_memory_stats() -> dict:
    mem = psutil.virtual_memory()
    return {
        "total_mb": round(mem.total / 1e6, 1),
        "used_mb": round(mem.used / 1e6, 1),
        "available_mb": round(mem.available / 1e6, 1),
        "percent": mem.percent,
    }


def get_disk_stats() -> dict:
    disk = psutil.disk_usage("/")
    return {
        "total_gb": round(disk.total / 1e9, 2),
        "used_gb": round(disk.used / 1e9, 2),
        "free_gb": round(disk.free / 1e9, 2),
        "percent": disk.percent,
    }


def get_network_bytes() -> dict:
    counters = psutil.net_io_counters()
    return {
        "bytes_sent": counters.bytes_sent,
        "bytes_recv": counters.bytes_recv,
        "packets_sent": counters.packets_sent,
        "packets_recv": counters.packets_recv,
    }


def get_undervoltage_flags() -> dict:
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            check=True,
        )
        raw = result.stdout.strip().split("=")[-1]
        flags = int(raw, 16)
        return {
            "raw_hex": raw,
            "currently_undervolted": bool(flags & (1 << 0)),
            "previously_undervolted": bool(flags & (1 << 16)),
        }
    except Exception:
        logging.exception("[system] vcgencmd failed")
        return {
            "raw_hex": "unavailable",
            "currently_undervolted": None,
            "previously_undervolted": None,
        }


def build_pi_status() -> dict:
    payload = {
        "device_name": DEVICE_NAME,
        "platform": platform.platform(),
        "cpu_temp_c": get_cpu_temp_c(),
        "load_1m": get_load_average()[0],
        "load_5m": get_load_average()[1],
        "load_15m": get_load_average()[2],
        "uptime_s": round(get_uptime_seconds(), 1),
        "memory": get_memory_stats(),
        "disk": get_disk_stats(),
        "network": get_network_bytes(),
        "undervoltage_flags": get_undervoltage_flags(),
    }

    temp = payload["cpu_temp_c"] or 0.0
    load = payload["load_1m"]
    mem_pct = payload["memory"]["percent"]

    if temp < 70 and load < 4.0 and mem_pct < 85:
        payload["health_state"] = "NOMINAL"
    elif temp < 80 and mem_pct < 95:
        payload["health_state"] = "HOLD"
    else:
        payload["health_state"] = "DOWN"

    return payload


# ------------------------------------------------------------------
# Network helpers (migrated from network_monitor)
# ------------------------------------------------------------------

def get_ssid() -> str:
    result = subprocess.run(
        ["iwgetid", "-r"],
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()

def zpnet_definitive_test() -> bool:
    """Verify basic internet reachability via ICMP ping."""
    result = subprocess.run(
        ["ping", "-c", "1", "-W", str(DEFINITIVE_TEST_TIMEOUT_S), DEFINITIVE_TEST_HOST],
        capture_output=True,
        text=True,
    )
    return result.returncode == 0

def get_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip

def ping_latency_ms() -> float:
    host = "8.8.8.8"
    port = 53
    attempts = 3
    timeout_s = 2

    times = []

    for _ in range(attempts):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout_s)
        try:
            start = time.time()
            s.connect((host, port))
            end = time.time()
            times.append((end - start) * 1000.0)
        except Exception:
            pass
        finally:
            s.close()

    return round(mean(times), 2) if times else 0.0

def get_interface_stats() -> dict:
    stats = psutil.net_io_counters(pernic=True)
    return {
        iface: {
            "bytes_sent": s.bytes_sent,
            "bytes_recv": s.bytes_recv,
            "packets_sent": s.packets_sent,
            "packets_recv": s.packets_recv,
        }
        for iface, s in stats.items()
    }

def download_test_mbps() -> float:
    url = f"http://{ZPNET_REMOTE_HOST}/api/download_test"
    headers = {
        "Connection": "close",
        "Accept-Encoding": "identity",
    }

    start = time.time()
    r = requests.get(url, headers=headers, timeout=HTTP_TIMEOUT)
    elapsed = time.time() - start

    bits = len(r.text) * 8
    return round((bits / 1e6) / elapsed, 2) if elapsed > 0 else 0.0

def upload_test_mbps() -> float:
    url = f"http://{ZPNET_REMOTE_HOST}/api/upload_test"

    payload = "".join(
        random.choice(string.ascii_uppercase)
        for _ in range(1024 * 1024)
    )

    body, headers = gzip_text(payload)

    start = time.time()
    requests.post(url, data=body, headers=headers, timeout=HTTP_TIMEOUT)
    elapsed = time.time() - start

    bits = len(payload) * 8
    return round((bits / 1e6) / elapsed, 2) if elapsed > 0 else 0.0

def build_network_status() -> dict:
    """
    Build network status snapshot.

    Semantics:
      • Definitive test (ICMP ping) determines network reachability
      • Speed tests hit sota.ddns.net — an unowned boundary
      • Server downtime is normal and must not spam the log
      • Transition logging: one message when server goes away,
        one message when it comes back
    """
    global _server_reachable

    payload = {}

    try:
        payload["ssid"] = get_ssid()
    except Exception:
        logging.exception("[system] get_ssid failed")
        payload["ssid"] = ""

    try:
        if zpnet_definitive_test():
            payload["network_status"] = "NOMINAL"
            payload["local_ip"] = get_local_ip()
            payload["interfaces"] = get_interface_stats()
            payload["ping_ms"] = ping_latency_ms()

            try:
                payload["download_mbps"] = download_test_mbps()
                payload["upload_mbps"] = upload_test_mbps()

                if not _server_reachable:
                    logging.info("[system] ZPNet Server reachable — speed tests resumed")
                    _server_reachable = True

            except requests.RequestException:
                if _server_reachable:
                    logging.info("[system] ZPNet Server unreachable — speed tests skipped")
                    _server_reachable = False

        else:
            payload["network_status"] = "DOWN"

    except Exception:
        logging.exception("[system] definitive test block failed")
        payload["network_status"] = "DOWN"

    return payload


# ------------------------------------------------------------------
# Laser helpers (migrated from laser_monitor)
# ------------------------------------------------------------------

def read_mp5491_registers():
    with open_i2c(I2C_BUS_LEGACY) as bus:
        return {
            name: bus.read_byte_data(LASER_ADDR, addr)
            for name, addr in REGS.items()
        }


def decode_mp5491_state(vals: dict) -> dict:
    sysen  = bool(vals["CTL0"] & SYSEN_BIT)
    id_en  = bool(vals["CTL1"] & ID_EN_BIT)
    id1_en = bool(vals["CTL1"] & ID1_EN_BIT)

    id1_raw = ((vals["ID1_MSB"] << 2) | (vals["ID1_LSB"] & 0x03))
    id1_current_ma = id1_raw * 0.25

    id1_active = bool(vals["STATUS1"] & ID1_FLG_BIT)
    vin2_ok    = not bool(vals["STATUS3"] & VIN2_UV_BIT)
    ld_on_seen = bool(vals["STATUS3"] & LD_ON_BIT)

    id_scp  = bool(vals["INT"] & ID_SCP_BIT)
    ot_warn = bool(vals["INT"] & OT_WARN_BIT)
    ot_shdn = bool(vals["INT"] & OT_SHDN_BIT)

    return {
        "sys_enabled": sysen,
        "id_enabled": id_en,
        "id1_enabled": id1_en,
        "id1_current_ma": round(id1_current_ma, 2),

        "id1_active": id1_active,
        "vin2_ok": vin2_ok,
        "ld_on_seen": ld_on_seen,

        "scp_tripped": id_scp,
        "ot_warn": ot_warn,
        "ot_shutdown": ot_shdn,
    }


def build_laser_status() -> dict:
    payload: dict[str, str | float | bool | dict[str, str]] = {
        "i2c_address": "0x66",
        "device_present": True,
    }

    # ----------------------------------------------------------
    # I²C capability truth (owned hardware)
    # ----------------------------------------------------------
    vals = read_mp5491_registers()
    decoded = decode_mp5491_state(vals)

    payload.update(decoded)

    payload["raw_registers"] = {
        f"0x{addr:02X}": f"0x{vals[name]:02X}"
        for name, addr in REGS.items()
    }

    teensy = send_command(machine="TEENSY", subsystem="LASER", command="REPORT")["payload"]
    payload.update(teensy)

    payload["health_state"] = "NOMINAL"
    return payload

# ------------------------------------------------------------------
# GNSS helpers (migrated from gnss_monitor)
# ------------------------------------------------------------------

def build_gnss_status() -> dict:
    return {
        **send_command(machine="PI", subsystem="GNSS", command="REPORT")["payload"],
        "health_state": "NOMINAL",
    }

# ------------------------------------------------------------------
# Sensor scan
# ------------------------------------------------------------------

def build_sensor_scan_status() -> dict:
    """
    Build sensor scan snapshot.

    Semantics:
      • Reports electrical presence only
      • Two buses are first-class: SMBUS1 and SMBUS2
      • Scan never crashes the poller
      • Generic probes are best-effort; protocol reads belong elsewhere
      • EEPROM companions (0x57) are ignored explicitly
      • health_state is NOMINAL only if all sensors are NOMINAL
    """
    # Top-level snapshot is heterogeneous:
    #   - per-bus entries are dict[str, str]
    #   - health_state is a scalar
    snapshot: dict[str, object] = {}
    all_nominal = True

    for bus_id, devices in I2C_SENSORS_BY_BUS.items():
        results: dict[str, str] = {}
        snapshot[f"i2c-{bus_id}"] = results

        try:
            with open_i2c(bus_id) as bus:
                for addr, label in devices.items():

                    if addr in I2C_IGNORE_ADDRS:
                        continue

                    # MiniMongo-safe key: address only (no periods)
                    rail_key = f"0x{addr:02X}"

                    # Presence probe: best-effort.
                    # Some devices do not support SMBus quick ops; do not treat
                    # that as absence. We only require that the bus itself works.
                    try:
                        bus.write_quick(addr)
                        results[rail_key] = "NOMINAL"
                    except OSError:
                        results[rail_key] = "PRESENT"
                        all_nominal = False

        except Exception:
            logging.exception(f"[system] sensor scan failed on i2c-{bus_id}")
            snapshot[f"i2c-{bus_id}"] = {"error": "scan_failed"}
            all_nominal = False

    snapshot["health_state"] = "NOMINAL" if all_nominal else "ANOMALY"
    return snapshot


# ------------------------------------------------------------------
# Environment helpers (migrated from environment_monitor)
# ------------------------------------------------------------------

def read_u16_le(bus: SMBus, addr: int, reg: int) -> int:
    lsb = bus.read_byte_data(addr, reg)
    msb = bus.read_byte_data(addr, reg + 1)
    return (msb << 8) | lsb


def read_s16_le(bus: SMBus, addr: int, reg: int) -> int:
    val = read_u16_le(bus, addr, reg)
    if val & 0x8000:
        val -= 1 << 16
    return val


def read_bme280_calibration(bus: SMBus) -> dict:
    cal = {}

    # Temperature
    cal["dig_T1"] = read_u16_le(bus, BME280_ADDR, 0x88)
    cal["dig_T2"] = read_s16_le(bus, BME280_ADDR, 0x8A)
    cal["dig_T3"] = read_s16_le(bus, BME280_ADDR, 0x8C)

    # Pressure
    cal["dig_P1"] = read_u16_le(bus, BME280_ADDR, 0x8E)
    cal["dig_P2"] = read_s16_le(bus, BME280_ADDR, 0x90)
    cal["dig_P3"] = read_s16_le(bus, BME280_ADDR, 0x92)
    cal["dig_P4"] = read_s16_le(bus, BME280_ADDR, 0x94)
    cal["dig_P5"] = read_s16_le(bus, BME280_ADDR, 0x96)
    cal["dig_P6"] = read_s16_le(bus, BME280_ADDR, 0x98)
    cal["dig_P7"] = read_s16_le(bus, BME280_ADDR, 0x9A)
    cal["dig_P8"] = read_s16_le(bus, BME280_ADDR, 0x9C)
    cal["dig_P9"] = read_s16_le(bus, BME280_ADDR, 0x9E)

    # Humidity
    cal["dig_H1"] = bus.read_byte_data(BME280_ADDR, 0xA1)
    cal["dig_H2"] = read_s16_le(bus, BME280_ADDR, 0xE1)
    cal["dig_H3"] = bus.read_byte_data(BME280_ADDR, 0xE3)

    e4 = bus.read_byte_data(BME280_ADDR, 0xE4)
    e5 = bus.read_byte_data(BME280_ADDR, 0xE5)
    e6 = bus.read_byte_data(BME280_ADDR, 0xE6)

    cal["dig_H4"] = (e4 << 4) | (e5 & 0x0F)
    cal["dig_H5"] = (e6 << 4) | (e5 >> 4)

    cal["dig_H6"] = bus.read_byte_data(BME280_ADDR, 0xE7)
    if cal["dig_H6"] & 0x80:
        cal["dig_H6"] -= 256

    return cal


def compensate_temperature(adc_T: int, cal: dict) -> tuple[float, float]:
    var1 = (adc_T / 16384.0 - cal["dig_T1"] / 1024.0) * cal["dig_T2"]
    var2 = ((adc_T / 131072.0 - cal["dig_T1"] / 8192.0) ** 2) * cal["dig_T3"]
    t_fine = var1 + var2
    return t_fine / 5120.0, t_fine


def compensate_pressure(adc_P: int, t_fine: float, cal: dict) -> float:
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * cal["dig_P6"] / 32768.0
    var2 += var1 * cal["dig_P5"] * 2.0
    var2 = var2 / 4.0 + cal["dig_P4"] * 65536.0
    var1 = (cal["dig_P3"] * var1 * var1 / 524288.0 + cal["dig_P2"] * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * cal["dig_P1"]

    p = 1048576.0 - adc_P
    p = (p - var2 / 4096.0) * 6250.0 / var1
    var1 = cal["dig_P9"] * p * p / 2147483648.0
    var2 = p * cal["dig_P8"] / 32768.0

    return (p + (var1 + var2 + cal["dig_P7"]) / 16.0) / 100.0


def compensate_humidity(adc_H: int, t_fine: float, cal: dict) -> float:
    h = t_fine - 76800.0
    h = (adc_H - (cal["dig_H4"] * 64.0 + cal["dig_H5"] / 16384.0 * h)) * (
        cal["dig_H2"]
        / 65536.0
        * (1.0 + cal["dig_H6"] / 67108864.0 * h * (1.0 + cal["dig_H3"] / 67108864.0 * h))
    )
    h *= 1.0 - cal["dig_H1"] * h / 524288.0
    return max(0.0, min(100.0, h))


def _build_environment_status_once() -> dict:
    """Perform one complete BME280 transaction sequence.

    This helper is intentionally allowed to raise.  The public
    build_environment_status() wrapper owns retry/stale-payload behavior.
    """
    global _BME280_CAL_CACHE

    with open_i2c(I2C_BUS_LEGACY) as bus:
        chip_id = bus.read_byte_data(BME280_ADDR, REG_ID)
        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(f"unexpected BME280 chip ID 0x{chip_id:02X}")

        # Forced mode, oversampling x1.
        #
        # These writes are normal BME280 control-plane writes: ctrl_hum selects
        # humidity oversampling, and ctrl_meas selects pressure/temperature
        # oversampling plus measurement mode.  The calibration table is static
        # after boot, so cache it after the first successful read to reduce the
        # transaction footprint on the shared SMBus.
        bus.write_byte_data(BME280_ADDR, REG_CTRL_HUM, 0x01)
        bus.write_byte_data(BME280_ADDR, REG_CTRL_MEAS, 0x27)

        if _BME280_CAL_CACHE is None:
            _BME280_CAL_CACHE = read_bme280_calibration(bus)
        cal = _BME280_CAL_CACHE

        data = bus.read_i2c_block_data(BME280_ADDR, REG_DATA, 8)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_H = (data[6] << 8) | data[7]

        temp_c, t_fine = compensate_temperature(adc_T, cal)
        pressure_hpa = compensate_pressure(adc_P, t_fine, cal)
        humidity_pct = compensate_humidity(adc_H, t_fine, cal)

        altitude_m = 44330.0 * (1.0 - (pressure_hpa / SEA_LEVEL_PRESSURE_HPA) ** 0.1903)

        return {
            "sensor_address": "0x76",
            "sensor_present": True,
            "temperature_c": round(temp_c, 2),
            "pressure_hpa": round(pressure_hpa, 2),
            "humidity_pct": round(humidity_pct, 2),
            "altitude_m": round(altitude_m, 1),
            "read_ok": True,
            "stale": False,
            "retry_count": 0,
            "read_fail_count": _BME280_READ_FAIL_COUNT,
            "consecutive_fail_count": 0,
            "recovery_count": _BME280_RECOVERY_COUNT,
            "last_error": "",
            "health_state": "NOMINAL",
        }


def _bme280_failure_payload(error: Exception, attempts: int) -> dict:
    base = dict(_BME280_LAST_GOOD) if _BME280_LAST_GOOD else {
        "sensor_address": "0x76",
        "sensor_present": False,
        "temperature_c": None,
        "pressure_hpa": None,
        "humidity_pct": None,
        "altitude_m": None,
    }

    base.update({
        "read_ok": False,
        "stale": _BME280_LAST_GOOD is not None,
        "retry_count": max(0, attempts - 1),
        "read_fail_count": _BME280_READ_FAIL_COUNT,
        "consecutive_fail_count": _BME280_CONSECUTIVE_FAIL_COUNT,
        "recovery_count": _BME280_RECOVERY_COUNT,
        "last_error": f"{type(error).__name__}: {error}",
        "health_state": "HOLD",
    })
    return base


def build_environment_status() -> dict:
    """
    Read environmental data from BME280.

    Semantics:
      • Returns a complete fresh payload when the BME280 transaction succeeds
      • Retries transient I2C failures a small number of times
      • Returns last-known-good/stale telemetry after exhausted retries
      • Never terminates the SYSTEM poller because of BME280 bus contention
    """
    global _BME280_LAST_GOOD
    global _BME280_READ_FAIL_COUNT
    global _BME280_CONSECUTIVE_FAIL_COUNT
    global _BME280_RECOVERY_COUNT

    last_error: Exception | None = None

    for attempt in range(1, BME280_RETRY_COUNT + 1):
        try:
            payload = _build_environment_status_once()
            if _BME280_CONSECUTIVE_FAIL_COUNT:
                _BME280_RECOVERY_COUNT += 1
            _BME280_CONSECUTIVE_FAIL_COUNT = 0
            payload["read_fail_count"] = _BME280_READ_FAIL_COUNT
            payload["consecutive_fail_count"] = 0
            payload["recovery_count"] = _BME280_RECOVERY_COUNT
            payload["retry_count"] = attempt - 1
            _BME280_LAST_GOOD = dict(payload)
            return payload
        except OSError as e:
            last_error = e
            _BME280_READ_FAIL_COUNT += 1
            _BME280_CONSECUTIVE_FAIL_COUNT += 1
            if attempt < BME280_RETRY_COUNT:
                logging.warning(
                    "[system] BME280 transient I2C error on attempt %d/%d: %s",
                    attempt,
                    BME280_RETRY_COUNT,
                    e,
                )
                time.sleep(BME280_RETRY_DELAY_SEC)
        except Exception as e:
            last_error = e
            _BME280_READ_FAIL_COUNT += 1
            _BME280_CONSECUTIVE_FAIL_COUNT += 1
            if attempt < BME280_RETRY_COUNT:
                logging.warning(
                    "[system] BME280 read error on attempt %d/%d: %s",
                    attempt,
                    BME280_RETRY_COUNT,
                    e,
                )
                time.sleep(BME280_RETRY_DELAY_SEC)

    logging.exception("[system] BME280 read failed after retries")
    return _bme280_failure_payload(last_error or RuntimeError("unknown BME280 error"),
                                   BME280_RETRY_COUNT)

# ------------------------------------------------------------------
# Power monitoring helpers (legacy: power_monitor)
# ------------------------------------------------------------------

def read_word(bus: SMBus, addr: int, reg: int) -> int:
    """Read a 16-bit INA260 register with byte swap."""
    raw = bus.read_word_data(addr, reg)
    return ((raw & 0xFF) << 8) | (raw >> 8)


def read_ina260(bus_id: int, bus: SMBus, addr: int) -> dict:
    adapter = i2c_adapter_name(bus_id)

    current_raw = read_word(bus, addr, REG_CURRENT)
    voltage_raw = read_word(bus, addr, REG_VOLTAGE)
    power_raw   = read_word(bus, addr, REG_POWER)

    # Sign-extend current
    if current_raw & 0x8000:
        current_raw -= 1 << 16

    current_ma = current_raw * 1.0          # 1 mA/LSB
    voltage_v  = voltage_raw * 0.00125      # 1.25 mV/LSB
    power_w    = (power_raw * 10) / 1000.0  # 10 mW/LSB → W

    return {
        "volts": round(voltage_v, 3),
        "amps": round(current_ma, 2),
        "watts": round(power_w, 3),
    }


def _ina260_key(bus_id: int, addr: int) -> Tuple[int, int]:
    return (bus_id, addr)


def _ina260_counter_get(store: Dict[Tuple[int, int], int],
                        bus_id: int,
                        addr: int) -> int:
    return store.get(_ina260_key(bus_id, addr), 0)


def _ina260_counter_set(store: Dict[Tuple[int, int], int],
                        bus_id: int,
                        addr: int,
                        value: int) -> None:
    store[_ina260_key(bus_id, addr)] = value


def _ina260_failure_payload(bus_id: int,
                            addr: int,
                            err: BaseException,
                            retry_count: int) -> dict:
    key = _ina260_key(bus_id, addr)
    last_good = _INA260_LAST_GOOD.get(key)
    base = dict(last_good) if last_good else {
        "volts": None,
        "amps": None,
        "watts": None,
    }
    base.update({
        "read_ok": False,
        "stale": last_good is not None,
        "retry_count": retry_count,
        "read_fail_count": _ina260_counter_get(_INA260_READ_FAIL_COUNT, bus_id, addr),
        "consecutive_fail_count": _ina260_counter_get(_INA260_CONSECUTIVE_FAIL_COUNT, bus_id, addr),
        "recovery_count": _ina260_counter_get(_INA260_RECOVERY_COUNT, bus_id, addr),
        "last_error": str(err),
        "health_state": "HOLD",
    })
    return base


def read_ina260_defensive(bus_id: int, bus: SMBus, addr: int) -> dict:
    """Read one INA260 rail with retry/stale-payload behavior.

    INA260 readings are useful context and battery-accounting inputs, but an
    occasional SMBus I/O error should not terminate the SYSTEM poller.  A
    successful read becomes the last-known-good payload for that bus/address.
    """
    key = _ina260_key(bus_id, addr)
    last_error: BaseException | None = None

    for attempt in range(1, INA260_RETRY_COUNT + 1):
        try:
            payload = read_ina260(bus_id, bus, addr)
            if _ina260_counter_get(_INA260_CONSECUTIVE_FAIL_COUNT, bus_id, addr):
                _ina260_counter_set(
                    _INA260_RECOVERY_COUNT,
                    bus_id,
                    addr,
                    _ina260_counter_get(_INA260_RECOVERY_COUNT, bus_id, addr) + 1,
                )
            _ina260_counter_set(_INA260_CONSECUTIVE_FAIL_COUNT, bus_id, addr, 0)

            payload.update({
                "read_ok": True,
                "stale": False,
                "retry_count": attempt - 1,
                "read_fail_count": _ina260_counter_get(_INA260_READ_FAIL_COUNT, bus_id, addr),
                "consecutive_fail_count": 0,
                "recovery_count": _ina260_counter_get(_INA260_RECOVERY_COUNT, bus_id, addr),
                "last_error": "",
                "health_state": "NOMINAL",
            })
            _INA260_LAST_GOOD[key] = dict(payload)
            return payload

        except OSError as e:
            last_error = e
            _ina260_counter_set(
                _INA260_READ_FAIL_COUNT,
                bus_id,
                addr,
                _ina260_counter_get(_INA260_READ_FAIL_COUNT, bus_id, addr) + 1,
            )
            _ina260_counter_set(
                _INA260_CONSECUTIVE_FAIL_COUNT,
                bus_id,
                addr,
                _ina260_counter_get(_INA260_CONSECUTIVE_FAIL_COUNT, bus_id, addr) + 1,
            )
            if attempt < INA260_RETRY_COUNT:
                logging.warning(
                    "[system] INA260 i2c-%d/0x%02X transient I2C error on attempt %d/%d: %s",
                    bus_id,
                    addr,
                    attempt,
                    INA260_RETRY_COUNT,
                    e,
                )
                time.sleep(INA260_RETRY_DELAY_SEC)

    logging.warning(
        "[system] INA260 i2c-%d/0x%02X read failed after retries: %s",
        bus_id,
        addr,
        last_error,
    )
    return _ina260_failure_payload(
        bus_id,
        addr,
        last_error or RuntimeError("unknown INA260 error"),
        INA260_RETRY_COUNT,
    )


def _ina260_bus_open_failure_payload(bus_id: int,
                                     addr: int,
                                     err: BaseException) -> dict:
    _ina260_counter_set(
        _INA260_READ_FAIL_COUNT,
        bus_id,
        addr,
        _ina260_counter_get(_INA260_READ_FAIL_COUNT, bus_id, addr) + 1,
    )
    _ina260_counter_set(
        _INA260_CONSECUTIVE_FAIL_COUNT,
        bus_id,
        addr,
        _ina260_counter_get(_INA260_CONSECUTIVE_FAIL_COUNT, bus_id, addr) + 1,
    )
    return _ina260_failure_payload(bus_id, addr, err, 0)


def build_power_status() -> dict:
    """
    Build power rail snapshot from INA260 sensors.

    Semantics:
      • Observation only
      • Per-rail I2C failures are nonfatal telemetry faults
      • Last-known-good rail values are retained as stale context
      • Boundary at SMBus acquisition

    Mongo / MiniMongo constraint:
      • Dictionary keys must NOT contain '.'.
      • Therefore, per-rail keys are address-derived (e.g., "0x41").
      • Human-readable domain names live in the inner "label" field.
    """
    snapshot: dict[str, dict[str, dict]] = {}
    all_nominal = True

    for bus_id, devices in POWER_CONFIG_BY_BUS.items():
        bus_key = f"i2c-{bus_id}"
        results: dict[str, dict] = {}
        snapshot[bus_key] = results

        try:
            with open_i2c(bus_id) as bus:
                for addr, cfg in devices.items():
                    reading = read_ina260_defensive(bus_id, bus, addr)
                    if not reading.get("read_ok", False):
                        all_nominal = False

                    # MiniMongo-safe key: address only (no periods)
                    rail_key = f"0x{addr:02X}"

                    results[rail_key] = {
                        "label": cfg["label"],
                        "address": rail_key,
                        "ideal_voltage_v": cfg["ideal_voltage_v"],
                        **reading,
                    }

        except Exception as e:
            logging.exception("[system] INA260 power bus i2c-%d failed", bus_id)
            all_nominal = False
            for addr, cfg in devices.items():
                rail_key = f"0x{addr:02X}"
                results[rail_key] = {
                    "label": cfg["label"],
                    "address": rail_key,
                    "ideal_voltage_v": cfg["ideal_voltage_v"],
                    **_ina260_bus_open_failure_payload(bus_id, addr, e),
                }

    return {
        "health_state": "NOMINAL" if all_nominal else "HOLD",
        **snapshot,
    }


# ------------------------------------------------------------------
# Battery status helpers
# ------------------------------------------------------------------

def extract_battery_power_w(system_payload: dict) -> Optional[float]:
    power = system_payload.get("power", {})

    for key, bus in power.items():
        if not key.startswith("i2c-"):
            continue

        for rail in bus.values():
            if rail.get("label") == BATTERY_LABEL:
                return rail.get("watts")

    return None


def get_last_battery_swap_ts() -> Optional[datetime]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT ts
            FROM zpnet_events
            WHERE event_type = 'SWAP_BATTERY'
            ORDER BY ts DESC
            LIMIT 1
            """
        )
        row = cur.fetchone()

    if not row:
        return None

    return normalize_ts(row["ts"])


def build_battery_status() -> dict:
    """
    Build battery state-of-charge snapshot.

    Semantics:
      • Read-only
      • Derived from SYSTEM power rails
      • Integration window anchored at last SWAP_BATTERY
      • No aggregates
      • No events emitted
    """

    payload = {
        "remaining_pct": None,
        "tte_minutes": None,
        "wh_used_since_recharge": None,
        "wh_remaining_estimate": None,
        "samples_used": 0,
        "sample_step": POWER_SAMPLE_STEP,
        "battery_capacity_wh": BATTERY_CAPACITY_WH,
        "health_state": "UNKNOWN",
    }

    # --------------------------------------------------------------
    # Determine integration start
    # --------------------------------------------------------------
    swap_ts = get_last_battery_swap_ts()
    if not swap_ts:
        return payload

    # --------------------------------------------------------------
    # Fetch SYSTEM_STATUS samples since swap
    # --------------------------------------------------------------
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT ts, payload
            FROM zpnet_events
            WHERE event_type = 'SYSTEM_STATUS'
              AND ts >= %s
            ORDER BY ts ASC
            """,
            (swap_ts,),
        )
        rows = cur.fetchall()

    if len(rows) < 2:
        return payload

    total_wh = 0.0
    last_ts = None
    last_power = None
    samples_used = 0

    # --------------------------------------------------------------
    # Integrate battery power over time
    # --------------------------------------------------------------
    for i in range(0, len(rows), POWER_SAMPLE_STEP):
        row = rows[i]

        ts = normalize_ts(row["ts"])
        system = normalize_payload(row["payload"])

        power_w = extract_battery_power_w(system)
        if power_w is None:
            continue

        if last_ts is not None and last_power is not None:
            dt = (ts - last_ts).total_seconds()
            avg_power = (power_w + last_power) / 2.0
            total_wh += abs(avg_power * dt / 3600.0)
            samples_used += 1

        last_ts = ts
        last_power = power_w

    # --------------------------------------------------------------
    # Compute remaining capacity
    # --------------------------------------------------------------
    remaining_wh = max(0.0, BATTERY_CAPACITY_WH - total_wh)
    remaining_pct = round(
        100.0 * remaining_wh / BATTERY_CAPACITY_WH, 1
    )

    if last_power and last_power > 1.0:
        tte_minutes = round(remaining_wh / last_power * 60.0, 1)
    else:
        tte_minutes = float("inf")

    # --------------------------------------------------------------
    # Health classification
    # --------------------------------------------------------------
    if remaining_pct <= 3:
        health_state = "DOWN"
    elif remaining_pct <= 10:
        health_state = "HOLD"
    else:
        health_state = "NOMINAL"

    # --------------------------------------------------------------
    # Final payload
    # --------------------------------------------------------------
    payload.update(
        {
            "remaining_pct": remaining_pct,
            "tte_minutes": tte_minutes,
            "wh_used_since_recharge": round(total_wh, 2),
            "wh_remaining_estimate": round(remaining_wh, 2),
            "samples_used": samples_used,
            "health_state": health_state,
        }
    )

    return payload


# ------------------------------------------------------------------
# Teensy status helpers (migrated from teensy_monitor)
# ------------------------------------------------------------------

def build_teensy_status() -> dict:
    response = send_command(machine="TEENSY", subsystem="SYSTEM", command="REPORT")
    payload = response["payload"]
    payload["health_state"] = "NOMINAL"
    return payload


def build_clocks_status() -> dict:
    response = send_command(machine="TEENSY", subsystem="CLOCKS", command="REPORT")
    payload = response["payload"]
    payload["health_state"] = "NOMINAL"
    return payload


def build_transport_status() -> dict:
    response = send_command(machine="TEENSY", subsystem="SYSTEM", command="TRANSPORT_INFO")
    payload = response["payload"]
    payload["health_state"] = "NOMINAL"
    return payload


def build_payload_status() -> dict:
    response = send_command(machine="TEENSY", subsystem="SYSTEM", command="PAYLOAD_INFO")
    payload = response["payload"]
    payload["health_state"] = "NOMINAL"
    return payload


def build_memory_status() -> dict:
    response = send_command(machine="TEENSY", subsystem="SYSTEM", command="MEMORY_INFO")
    payload = response["payload"]
    payload["health_state"] = "NOMINAL"
    return payload


def build_process_status() -> dict:
    response = send_command(machine="TEENSY", subsystem="SYSTEM", command="PROCESS_INFO")
    payload = response["payload"]
    payload["health_state"] = "NOMINAL"
    return payload

# ------------------------------------------------------------------
# System poller thread
# ------------------------------------------------------------------

def system_poller() -> None:
    global SYSTEM

    try:
        while True:

            pi_payload = build_pi_status()
            teensy_payload = build_teensy_status()
            network_payload = build_network_status()
            # laser_payload = build_laser_status()
            sensor_payload = build_sensor_scan_status()
            environment_payload = build_environment_status()
            gnss_payload = build_gnss_status()
            power_payload = build_power_status()
            battery_payload = build_battery_status()
            clocks_payload = build_clocks_status()
            transport_payload = build_transport_status()
            payload_payload = build_payload_status()
            # MULE: do not ask for memory information too dangerous because of mallinfo
            #memory_payload = build_memory_status()
            process_payload = build_process_status()

            teensy_features = _teensy_feature_tree_from_report(teensy_payload)
            if teensy_features:
                _replace_teensy_feature_tree(teensy_features)

            _update_builtin_pi_features(
                pi_payload=pi_payload, network_payload=network_payload,
                sensor_payload=sensor_payload, environment_payload=environment_payload,
                gnss_payload=gnss_payload, power_payload=power_payload,
                battery_payload=battery_payload,
                teensy_features_available=bool(_teensy_feature_tree_snapshot()),
            )
            features = _feature_tree_snapshot()
            snapshot = {
                "pi": dict(pi_payload),
                "teensy": dict(teensy_payload),
                "network": dict(network_payload),
                # "laser": dict(laser_payload),
                "sensors": dict(sensor_payload),
                "environment": dict(environment_payload),
                "gnss": dict(gnss_payload),
                "power": dict(power_payload),
                "battery": dict(battery_payload),
                "clocks": dict(clocks_payload),
                "transport": dict(transport_payload),
                "payload": dict(payload_payload),
                #"memory": dict(memory_payload),
                "process": dict(process_payload),
                "features": features,
            }

            with _SYSTEM_LOCK:
                SYSTEM = snapshot

            _publish_feature_status_if_changed(features)
            # ----------------------------------------------------------
            # Emit consolidated system event
            # ----------------------------------------------------------
            create_event("SYSTEM_STATUS", dict(SYSTEM))

            time.sleep(POLL_INTERVAL_SEC)

    except Exception:
        logging.exception("[system_poller] unhandled exception - poller thread terminating")


# ------------------------------------------------------------------
# Pub/Sub handlers
# ------------------------------------------------------------------

def on_feature_status_fragment(payload: Optional[dict]) -> None:
    """Consume Teensy FEATURE_STATUS_FRAGMENT and relay the unified tree."""
    if not isinstance(payload, dict):
        logging.warning("[system] ignoring malformed FEATURE_STATUS_FRAGMENT: %r", payload)
        return

    if not _replace_teensy_feature_tree(payload):
        return

    features = _refresh_feature_payload_from_registry()
    _publish_feature_status_if_changed(features)


# ------------------------------------------------------------------
# Command handlers
# ------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:
    """Return the most recent SYSTEM snapshot."""
    with _SYSTEM_LOCK:
        snapshot = dict(SYSTEM)
    return {
        "success": True,
        "message": "OK",
        "payload": snapshot,
    }


def _current_feature_payload() -> dict:
    features = _feature_tree_snapshot()
    if features:
        return features



def _refresh_feature_payload_from_registry() -> dict:
    global SYSTEM
    features = _feature_tree_snapshot()

    with _SYSTEM_LOCK:
        current = dict(SYSTEM)
        current["features"] = features
        current.pop("feature_summary", None)
        SYSTEM = current


    with _SYSTEM_LOCK:
        system_features = SYSTEM.get("features")
    return _copy_feature_tree(system_features)

def cmd_features(_: Optional[dict]) -> Dict:
    return {
        "success": True,
        "message": "OK",
        "payload": _current_feature_payload(),
    }


def cmd_get_feature(args: Optional[dict]) -> Dict:
    args = args or {}
    name = str(args.get("name") or "").strip().upper()
    if not name:
        machine = str(args.get("machine") or "PI").strip().upper()
        subsystem = str(args.get("subsystem") or "").strip().upper()
        feature = str(args.get("feature") or "").strip().upper()
        name = f"{machine}.{subsystem}.{feature}" if subsystem and feature else ""
    if not name:
        return {"success": False, "message": "GET_FEATURE requires name or subsystem+feature"}

    features = _current_feature_payload()
    try:
        entry = feature_get(features, name, default=None)
        status = feature_status(features, name)
    except ValueError as e:
        return {"success": False, "message": str(e)}

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "name": name,
            "known": entry is not None,
            "status": status,
        },
    }


def cmd_set_feature(args: Optional[dict]) -> Dict:
    args = args or {}
    machine = str(args.get("machine") or "PI").strip().upper()
    if machine != "PI":
        return {"success": False, "message": "Pi SYSTEM may only set PI.* feature state"}

    raw_status = str(args.get("status") or "").strip().upper()
    if raw_status not in FEATURE_STATUSES and raw_status != "DOWN":
        return {
            "success": False,
            "message": "SET_FEATURE status must be INITIALIZING, NOMINAL, HOLD, or ANOMALY",
        }

    subsystem = str(args.get("subsystem") or "")
    feature = str(args.get("feature") or "")
    try:
        status = set_pi_feature(subsystem, feature, raw_status)
    except ValueError as e:
        return {"success": False, "message": str(e)}

    features = _refresh_feature_payload_from_registry()
    _publish_feature_status_if_changed(features)
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "subsystem": subsystem.strip().upper(),
            "feature": feature.strip().upper(),
            "status": status,
        },
    }

def cmd_swap_battery(_: Optional[dict]) -> Dict:
    create_event("SWAP_BATTERY", None)
    return {
        "success": True,
        "message": "OK",
    }

COMMANDS = {
    "REPORT": cmd_report,
    "FEATURES": cmd_features,
    "REPORT_FEATURES": cmd_features,
    "GET_FEATURE": cmd_get_feature,
    "SET_FEATURE": cmd_set_feature,
    "SWAP_BATTERY": cmd_swap_battery,
}

# ---------------------------------------------------------------------
# Startup quiet barrier
# ---------------------------------------------------------------------

def startup_teensy_quiet_delay() -> None:
    """
    Let pubsub discover this process before SYSTEM begins polling the cluster.
    """
    logging.info(
        "⏳ [system] waiting %.1fs for pubsub routing and Teensy initialization "
        "before active polling",
        STARTUP_TEENSY_QUIET_DELAY_S,
    )
    time.sleep(STARTUP_TEENSY_QUIET_DELAY_S)
    logging.info("✅ [system] startup quiet delay complete — polling may begin")


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    try:
        server_setup(
            subsystem="SYSTEM",
            commands=COMMANDS,
            subscriptions={
                "FEATURE_STATUS_FRAGMENT": on_feature_status_fragment,
            },
            blocking=False,
        )

        startup_teensy_quiet_delay()

        threading.Thread(
            target=system_poller,
            daemon=True,
        ).start()

        threading.Thread(
            target=feature_status_periodic_publisher,
            daemon=True,
        ).start()

        logging.info("🏁 [system] entering main loop")
        while True:
            time.sleep(3600)

    except Exception:
        logging.exception("💥 [system] unhandled exception in main thread")


if __name__ == "__main__":
    run()