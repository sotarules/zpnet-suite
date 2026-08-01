"""
ZPNet SYSTEM Process (Pi-side, authoritative aggregator)

Responsibilities:
  • Consume the PPS-aligned Teensy MONITOR_FRAGMENT feed
  • Collect Raspberry Pi host metrics on a slower local cadence
  • Decorate each Teensy fragment with last-known-good Pi-owned state
  • Publish one unified MONITOR snapshot per Teensy second
  • Preserve SYSTEM.REPORT as an explicit command surface

Process model:
  • One systemd service
  • One Pi-context polling thread
  • One blocking command socket
  • MONITOR_FRAGMENT, CLOCKS_MONITOR, and GNSS_ANNOUNCEMENT subscriptions
"""

from __future__ import annotations

import copy
import datetime
from collections import deque
import json
import logging
import math
import queue
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
STARTUP_TEENSY_QUIET_DELAY_S = 10.0
CLOCKS_MONITOR_TOPIC = "CLOCKS_MONITOR"
GNSS_ANNOUNCEMENT_TOPIC = "GNSS_ANNOUNCEMENT"
GNSS_MONITOR_FRESHNESS_MAX_AGE_S = 2.5
GNSS_ANNOUNCEMENT_HISTORY_MAX = 8
MONITOR_CHECKPOINT_CONFIG_KEY = "MONITOR"
MONITOR_RESTORE_TIMEOUT_S = 60.0
MONITOR_RESTORE_COMMAND_RETRY_S = 10.0

_TEENSY_MONITOR_RESTORE_ACCEPTED_STATUSES = {
    "monitor_restore_requested",
}

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

# Pi CLOCKS publishes its Pi-owned GNSS_RAW/baseline decoration independently
# of the PPS-aligned Teensy fragment.  SYSTEM retains only the latest value and
# folds it into the next ephemeral MONITOR snapshot.
_CLOCKS_MONITOR_LOCK = threading.Lock()
_LATEST_CLOCKS_MONITOR: Dict[str, Any] = {}
_LATEST_CLOCKS_MONITOR_RECEIVED_MONOTONIC: Optional[float] = None
_LATEST_CLOCKS_MONITOR_RECEIVED_UTC: Optional[str] = None

# GNSS publishes predictive TPS1 announcements.  Retain a short history so
# MONITOR can select the announcement naming its completed UTC second rather
# than accidentally displaying the following staged second.
_GNSS_MONITOR_LOCK = threading.Lock()
_GNSS_ANNOUNCEMENT_HISTORY: deque[Dict[str, Any]] = deque(maxlen=GNSS_ANNOUNCEMENT_HISTORY_MAX)
_GNSS_ANNOUNCEMENT_RECEIVED = 0
_GNSS_ANNOUNCEMENT_MALFORMED = 0
_GNSS_ANNOUNCEMENT_EXACT_MATCHES = 0
_GNSS_ANNOUNCEMENT_FALLBACK_MATCHES = 0

# SYSTEM owns the durable unified MONITOR checkpoint and the boot restore
# transaction.  Callbacks may stage fresh checkpoints immediately, but the
# writer starts only after the previous checkpoint has been consumed and its
# restore has either succeeded or been found unavailable.
_LATEST_MONITOR_RECEIVED_MONOTONIC: Optional[float] = None
_MONITOR_CHECKPOINT_QUEUE: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=1)
_MONITOR_CHECKPOINT_WRITER_STARTED = threading.Event()

# ------------------------------------------------------------------
# Feature status clearing house
# ------------------------------------------------------------------

FEATURE_STATUSES = {"INITIALIZING", "NOMINAL", "HOLD", "ANOMALY"}

# MONITOR.features is the mission-readiness surface.  The raw QTimer/DWT
# interval witness remains available through Teensy INTERRUPT diagnostics, but
# its ISR-displacement-sensitive state is intentionally not an annunciator.
_PUBLIC_FEATURE_EXCLUSIONS = {
    ("TEENSY", "INTERRUPT", "QTIMER_DWT_RULER"),
}


_FEATURE_LOCK = threading.Lock()
_PI_FEATURES: Dict[str, Dict[str, Dict[str, str]]] = {"PI": {}}
_TEENSY_FEATURES: Dict[str, Dict[str, Dict[str, str]]] = {"TEENSY": {}}


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

def _parse_gnss_utc(value: Any) -> Optional[datetime.datetime]:
    if not isinstance(value, str) or not value.strip():
        return None
    try:
        return datetime.datetime.fromisoformat(value.strip().replace("Z", "+00:00")).astimezone(datetime.timezone.utc)
    except (TypeError, ValueError):
        return None


def _announcement_to_gnss(announcement: Dict[str, Any]) -> Dict[str, Any]:
    receiver = announcement.get("receiver")
    gnss = dict(receiver) if isinstance(receiver, dict) else {}
    next_utc = str(announcement.get("next_utc") or "")
    parsed = _parse_gnss_utc(next_utc)
    if parsed is not None:
        gnss["date"] = parsed.strftime("%Y-%m-%d")
        gnss["time"] = parsed.strftime("%H:%M:%S")
    gnss.update({
        "next_utc": next_utc or None,
        "gnss_time_utc": next_utc or None,
        "announcement_schema": announcement.get("schema"),
        "semantics": announcement.get("semantics"),
        "source_sentence": announcement.get("source_sentence"),
        "announcement_sequence": announcement.get("announcement_sequence"),
        "received_at_utc": announcement.get("received_at_utc"),
        "time_valid": announcement.get("time_valid"),
        "time_status": announcement.get("time_status"),
        "pps_sync": announcement.get("pps_sync"),
        "leap_second": announcement.get("leap_second"),
        "clock_drift_ppb": announcement.get("clock_drift_ppb"),
        "temperature_c": announcement.get("temperature_c"),
    })
    return gnss


def _gnss_announcement_snapshot(target_utc: datetime.datetime) -> tuple[dict, dict]:
    global _GNSS_ANNOUNCEMENT_EXACT_MATCHES, _GNSS_ANNOUNCEMENT_FALLBACK_MATCHES
    target = target_utc.astimezone(datetime.timezone.utc).replace(microsecond=0)
    with _GNSS_MONITOR_LOCK:
        history = [dict(item) for item in _GNSS_ANNOUNCEMENT_HISTORY]
    selected = None
    for item in history:
        named = _parse_gnss_utc(item.get("next_utc"))
        if named is not None and named.replace(microsecond=0) == target:
            selected = item
            _GNSS_ANNOUNCEMENT_EXACT_MATCHES += 1
            break
    if selected is None:
        return {}, {
            "source": GNSS_ANNOUNCEMENT_TOPIC,
            "source_fresh": False,
            "source_age_s": None,
            "matched_target_utc": False,
            "received_count": _GNSS_ANNOUNCEMENT_RECEIVED,
            "malformed_count": _GNSS_ANNOUNCEMENT_MALFORMED,
        }
    local_received = selected.get("_system_received_monotonic")
    age_s = None if local_received is None else max(0.0, time.monotonic() - float(local_received))
    named = _parse_gnss_utc(selected.get("next_utc"))
    matched = bool(named is not None and named.replace(microsecond=0) == target)
    metadata = {
        "source": GNSS_ANNOUNCEMENT_TOPIC,
        "received_at_utc": selected.get("_system_received_at_utc"),
        "source_received_at_utc": selected.get("received_at_utc"),
        "source_age_s": None if age_s is None else round(age_s, 3),
        "source_fresh": bool(age_s is not None and age_s <= GNSS_MONITOR_FRESHNESS_MAX_AGE_S),
        "matched_target_utc": matched,
        "used_prior_announcement": False,
        "announcement_sequence": selected.get("announcement_sequence"),
        "next_utc": selected.get("next_utc"),
        "received_count": _GNSS_ANNOUNCEMENT_RECEIVED,
        "malformed_count": _GNSS_ANNOUNCEMENT_MALFORMED,
        "exact_match_count": _GNSS_ANNOUNCEMENT_EXACT_MATCHES,
        "fallback_match_count": _GNSS_ANNOUNCEMENT_FALLBACK_MATCHES,
    }
    return _announcement_to_gnss(selected), metadata


def build_gnss_status() -> dict:
    gnss, metadata = _gnss_announcement_snapshot(datetime.datetime.now(datetime.timezone.utc))
    health = "NOMINAL" if metadata.get("source_fresh") else "HOLD"
    return {**gnss, "announcement": metadata, "health_state": health}

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
# Durable MONITOR checkpoint and boot recovery
# ------------------------------------------------------------------

def _read_monitor_checkpoint() -> Optional[Dict[str, Any]]:
    """Read the last unified MONITOR checkpoint without changing it."""
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                "SELECT payload FROM config WHERE config_key = %s",
                (MONITOR_CHECKPOINT_CONFIG_KEY,),
            )
            row = cur.fetchone()
    except Exception:
        logging.exception("⚠️ [system/recovery] failed to read config.MONITOR")
        return None

    if row is None:
        return None

    payload = row["payload"]
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except Exception:
            payload = None
    if not isinstance(payload, dict):
        logging.error("💥 [system/recovery] config.MONITOR payload is not an object")
        return None
    return copy.deepcopy(payload)


def _persist_monitor_checkpoint(monitor: Dict[str, Any]) -> None:
    """Replace config.MONITOR with one unchanged unified MONITOR payload."""
    encoded = json.dumps(monitor, separators=(",", ":"), ensure_ascii=False)
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            WITH updated AS (
                UPDATE config
                SET "timestamp" = now(), payload = %s::jsonb
                WHERE config_key = %s
                RETURNING 1
            )
            INSERT INTO config (config_key, payload)
            SELECT %s, %s::jsonb
            WHERE NOT EXISTS (SELECT 1 FROM updated)
            """,
            (
                encoded,
                MONITOR_CHECKPOINT_CONFIG_KEY,
                MONITOR_CHECKPOINT_CONFIG_KEY,
                encoded,
            ),
        )


def _monitor_clocks_payload(monitor: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not isinstance(monitor, dict):
        return {}
    clocks = monitor.get("clocks")
    return clocks if isinstance(clocks, dict) else {}


def _monitor_restore_state(monitor: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Return firmware-authored structured sufficient state from MONITOR."""
    state = _monitor_clocks_payload(monitor).get("restore_state")
    if not isinstance(state, dict):
        return None
    try:
        version = int(state.get("version") or state.get("schema_version") or 0)
    except (TypeError, ValueError):
        return None
    if version != 2 or not state.get("present", True):
        return None
    return copy.deepcopy(state)


def _structured_restore_args(state: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten structured MONITOR state into ordinary CLOCKS command fields."""
    instrument = state.get("instrument_clockfaces")
    stats = state.get("stats")
    dac = state.get("dac")
    if not isinstance(instrument, dict) or not isinstance(stats, dict) or not isinstance(dac, dict):
        raise ValueError("structured restore missing instrument_clockfaces/stats/dac")

    def path(mapping: Dict[str, Any], dotted: str) -> Any:
        current: Any = mapping
        for part in dotted.split("."):
            if not isinstance(current, dict):
                return None
            current = current.get(part)
        return current

    out: Dict[str, Any] = {
        "restore_schema_version": 2,
        "restore_instrument_gnss_ns": instrument.get("gnss_ns"),
        "restore_instrument_dwt_cycles": instrument.get("dwt_cycles"),
        "restore_instrument_ocxo1_ns": instrument.get("ocxo1_ns"),
        "restore_instrument_ocxo2_ns": instrument.get("ocxo2_ns"),
        "restore_stats_valid": stats.get("valid"),
        "restore_stats_completed_row_coherent": stats.get("completed_row_coherent"),
        "restore_stats_reset_count": stats.get("reset_count"),
        "restore_stats_update_count": stats.get("update_count"),
        "restore_stats_vclock_interval_reject_count": path(stats, "interval_admission.vclock_reject_count"),
        "restore_stats_ocxo1_interval_reject_count": path(stats, "interval_admission.ocxo1_reject_count"),
        "restore_stats_ocxo2_interval_reject_count": path(stats, "interval_admission.ocxo2_reject_count"),
        "restore_servo_mode": dac.get("servo_mode") or "OFF",
        "restore_dither_operator_enabled": dac.get("dither_operator_enabled", False),
    }
    welfords = {
        "gnss": path(stats, "gnss.welford"),
        "dwt": path(stats, "dwt.welford"),
        "vclock": path(stats, "vclock.welford"),
        "ocxo1": path(stats, "ocxo1.welford"),
        "ocxo2": path(stats, "ocxo2.welford"),
        "pps_witness": path(stats, "pps_witness.welford"),
        "ocxo1_dac": path(stats, "dac.ocxo1"),
        "ocxo2_dac": path(stats, "dac.ocxo2"),
    }
    for name, wf in welfords.items():
        if not isinstance(wf, dict):
            raise ValueError(f"structured restore missing {name} Welford")
        for field in ("n", "mean", "m2", "min", "max"):
            out[f"restore_{name}_welford_{field}"] = wf.get(field)
    for lane in ("ocxo1", "ocxo2"):
        tau = path(stats, f"{lane}_tau_state")
        if not isinstance(tau, dict):
            raise ValueError(f"structured restore missing {lane} TAU state")
        for field in (
            "valid", "reset_count", "sample_count", "interval_count",
            "reject_count", "gap_reset_count", "last_pps_sequence",
            "last_interval_pps_sequence", "first_refined_ns", "last_refined_ns",
            "last_fast_residual_ns", "cumulative_reference_ns",
            "cumulative_clock_ns", "cumulative_clock_ns_exact", "mean_x",
            "mean_y", "sxx", "sxy", "syy", "interval_mean_ppb",
            "interval_m2_ppb",
        ):
            out[f"restore_{lane}_tau_{field}"] = tau.get(field)
        lane_state = dac.get(lane)
        servo = lane_state.get("servo") if isinstance(lane_state, dict) else None
        if not isinstance(lane_state, dict) or not isinstance(servo, dict):
            raise ValueError(f"structured restore missing {lane} DAC/servo state")
        out[f"restore_{lane}_dac_value"] = lane_state.get("value", lane_state.get("dac"))
        field_map = {
            "servo_last_step": "last_step", "servo_last_residual": "last_residual",
            "servo_settle_count": "settle_count", "servo_adjustments": "adjustments",
            "servo_predictor_initialized": "predictor_initialized",
            "servo_last_raw_residual": "last_raw_residual",
            "servo_filtered_residual": "filtered_residual",
            "servo_filtered_slope": "filtered_slope",
            "servo_predicted_residual": "predicted_residual",
            "servo_predictor_updates": "predictor_updates",
        }
        for command_field, json_field in field_map.items():
            out[f"restore_{lane}_dac_{command_field}"] = servo.get(json_field)
    missing = [key for key, value in out.items() if value is None]
    if missing:
        raise ValueError(f"structured restore missing fields: {missing}")
    return out

def _monitor_gnss_raw_payload(monitor: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Return the Pi CLOCKS-owned GNSS_RAW state carried by MONITOR."""
    gnss_raw = _monitor_clocks_payload(monitor).get("gnss_raw")
    return copy.deepcopy(gnss_raw) if isinstance(gnss_raw, dict) else None


def _queue_monitor_checkpoint(monitor: Dict[str, Any]) -> None:
    """Stage only the newest fully recoverable MONITOR; never block pub/sub."""
    if (
        _monitor_restore_state(monitor) is None
        or _monitor_gnss_raw_payload(monitor) is None
    ):
        return

    item = copy.deepcopy(monitor)
    try:
        _MONITOR_CHECKPOINT_QUEUE.put_nowait(item)
        return
    except queue.Full:
        pass

    try:
        _MONITOR_CHECKPOINT_QUEUE.get_nowait()
    except queue.Empty:
        pass

    try:
        _MONITOR_CHECKPOINT_QUEUE.put_nowait(item)
    except queue.Full:
        # Another callback won the latest-state replacement race.
        pass


def _monitor_checkpoint_writer_loop() -> None:
    """Persist latest-state MONITOR checkpoints after boot recovery completes."""
    _MONITOR_CHECKPOINT_WRITER_STARTED.set()
    while True:
        monitor = _MONITOR_CHECKPOINT_QUEUE.get()
        while True:
            try:
                monitor = _MONITOR_CHECKPOINT_QUEUE.get_nowait()
            except queue.Empty:
                break
        try:
            _persist_monitor_checkpoint(monitor)
        except Exception:
            logging.exception("⚠️ [system] failed to persist config.MONITOR")


def _start_monitor_checkpoint_writer() -> None:
    if _MONITOR_CHECKPOINT_WRITER_STARTED.is_set():
        return
    threading.Thread(
        target=_monitor_checkpoint_writer_loop,
        daemon=True,
        name="system-monitor-checkpoint-writer",
    ).start()


def _seed_system_from_checkpoint(checkpoint: Dict[str, Any]) -> None:
    """Publish the last unified state immediately and seed CLOCKS decoration."""
    global SYSTEM
    global _LATEST_MONITOR_RECEIVED_MONOTONIC
    global _LATEST_CLOCKS_MONITOR
    global _LATEST_CLOCKS_MONITOR_RECEIVED_MONOTONIC
    global _LATEST_CLOCKS_MONITOR_RECEIVED_UTC

    seeded_at_utc = datetime.datetime.now(datetime.timezone.utc) \
        .isoformat() \
        .replace("+00:00", "Z")
    clocks = _monitor_clocks_payload(checkpoint)
    pi_clocks = clocks.get("pi")
    if not isinstance(pi_clocks, dict):
        pi_clocks = {
            "schema": clocks.get("pi_schema"),
            "gnss_raw": _dict_copy(clocks.get("gnss_raw")),
            "extra_clocks": _dict_copy(clocks.get("extra_clocks")),
            "baseline": _dict_copy(clocks.get("baseline")),
            "stats_reset": _dict_copy(clocks.get("stats_reset")),
        }

    with _CLOCKS_MONITOR_LOCK:
        _LATEST_CLOCKS_MONITOR = copy.deepcopy(pi_clocks)
        _LATEST_CLOCKS_MONITOR_RECEIVED_MONOTONIC = time.monotonic()
        _LATEST_CLOCKS_MONITOR_RECEIVED_UTC = seeded_at_utc

    with _SYSTEM_LOCK:
        SYSTEM = copy.deepcopy(checkpoint)
        _LATEST_MONITOR_RECEIVED_MONOTONIC = time.monotonic()

    # The checkpoint writer is still closed, so this immediate display seed
    # cannot overwrite the durable recovery authority.
    publish("MONITOR", copy.deepcopy(checkpoint))


def _request_teensy_monitor_restore(state: Dict[str, Any]) -> Dict[str, Any]:
    """Stage one structured firmware MONITOR restore, retrying only busy."""
    deadline = time.monotonic() + MONITOR_RESTORE_COMMAND_RETRY_S
    last_response: Any = None
    restore_args = _structured_restore_args(state)
    while True:
        response = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="RESTORE_MONITOR",
            args=restore_args,
        )
        last_response = response
        outer_success = isinstance(response, dict) and bool(response.get("success"))
        payload = response.get("payload") if isinstance(response, dict) else None
        status = str(payload.get("status") or "") if isinstance(payload, dict) else ""
        if outer_success and status in _TEENSY_MONITOR_RESTORE_ACCEPTED_STATUSES:
            return response
        if outer_success and status == "monitor_restore_rejected_busy":
            campaign_state = str(
                payload.get("campaign_state") or ""
            ).strip().upper() if isinstance(payload, dict) else ""
            if campaign_state == "STARTED":
                return response
            if time.monotonic() < deadline:
                time.sleep(0.25)
                continue
        raise RuntimeError(
            "Teensy CLOCKS.RESTORE_MONITOR rejected: "
            f"status={status or 'missing_handler_status'} "
            f"outer_success={outer_success} response={last_response!r}"
        )

def _request_clocks_gnss_raw_restore(gnss_raw: Dict[str, Any]) -> Dict[str, Any]:
    """Hand Pi-owned GNSS_RAW state to its CLOCKS process owner."""
    deadline = time.monotonic() + MONITOR_RESTORE_COMMAND_RETRY_S
    last_error: Optional[BaseException] = None
    while True:
        try:
            response = send_command(
                machine="PI",
                subsystem="CLOCKS",
                command="RESTORE_GNSS_RAW",
                args={
                    "state": json.dumps(
                        gnss_raw,
                        separators=(",", ":"),
                        ensure_ascii=False,
                    )
                },
                retries=1,
                retry_delay_s=0.0,
            )
            if isinstance(response, dict) and response.get("success"):
                return response
            last_error = RuntimeError(f"CLOCKS.RESTORE_GNSS_RAW rejected: {response!r}")
        except Exception as exc:
            last_error = exc

        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"CLOCKS.RESTORE_GNSS_RAW unavailable: {last_error}"
            ) from last_error
        time.sleep(0.25)


def _monitor_restore_probe(monitor: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Extract the durable surfaces used to prove a MONITOR restore completed."""
    clocks = _monitor_clocks_payload(monitor)
    instrument = clocks.get("instrument_clockfaces")
    stats = clocks.get("stats")
    dac = clocks.get("dac")
    if not isinstance(instrument, dict):
        instrument = {}
    if not isinstance(stats, dict):
        stats = {}
    if not isinstance(dac, dict):
        dac = {}

    welford_n: Dict[str, int] = {}
    for lane in ("gnss", "dwt", "vclock", "ocxo1", "ocxo2", "pps_witness"):
        node = stats.get(lane)
        wf = node.get("welford") if isinstance(node, dict) else None
        try:
            welford_n[lane] = int(wf.get("n") or 0) if isinstance(wf, dict) else 0
        except (TypeError, ValueError):
            welford_n[lane] = 0

    dac_stats = stats.get("dac")
    for lane in ("ocxo1", "ocxo2"):
        wf = dac_stats.get(lane) if isinstance(dac_stats, dict) else None
        try:
            welford_n[f"dac.{lane}"] = int(wf.get("n") or 0) if isinstance(wf, dict) else 0
        except (TypeError, ValueError):
            welford_n[f"dac.{lane}"] = 0

    def _int_value(value: Any) -> int:
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return 0

    def _float_value(value: Any) -> Optional[float]:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    return {
        "instrument_gnss_ns": _int_value(instrument.get("gnss_ns")),
        "instrument_dwt_cycles": _int_value(instrument.get("dwt_cycles")),
        "instrument_ocxo1_ns": _int_value(instrument.get("ocxo1_ns")),
        "instrument_ocxo2_ns": _int_value(instrument.get("ocxo2_ns")),
        "welford_n": welford_n,
        "servo_mode": str(dac.get("servo_mode") or "OFF").upper(),
        "dither_operator_enabled": bool(dac.get("dither_operator_enabled")),
        "ocxo1_dac": _float_value(dac.get("ocxo1_dac")),
        "ocxo2_dac": _float_value(dac.get("ocxo2_dac")),
    }


def _monitor_restore_probe_satisfied(
    expected: Dict[str, Any],
    observed: Dict[str, Any],
) -> bool:
    for key in (
        "instrument_gnss_ns",
        "instrument_dwt_cycles",
        "instrument_ocxo1_ns",
        "instrument_ocxo2_ns",
    ):
        target = int(expected.get(key) or 0)
        if target > 0 and int(observed.get(key) or 0) < target:
            return False

    expected_n = expected.get("welford_n")
    observed_n = observed.get("welford_n")
    if isinstance(expected_n, dict) and isinstance(observed_n, dict):
        for key, target_value in expected_n.items():
            target = int(target_value or 0)
            if target > 0 and int(observed_n.get(key) or 0) < target:
                return False

    if str(observed.get("servo_mode") or "OFF").upper() != str(
        expected.get("servo_mode") or "OFF"
    ).upper():
        return False
    if bool(observed.get("dither_operator_enabled")) != bool(
        expected.get("dither_operator_enabled")
    ):
        return False
    for key in ("ocxo1_dac", "ocxo2_dac"):
        target = expected.get(key)
        value = observed.get(key)
        if target is not None and (
            value is None or abs(float(value) - float(target)) > 0.01
        ):
            return False
    return True


def _monitor_restore_pending_categories(
    expected: Dict[str, Any],
    observed: Dict[str, Any],
) -> list[str]:
    """Summarize restore proof gaps without logging payloads or numeric state."""
    pending: list[str] = []

    clockface_keys = {
        "instrument_gnss_ns": "GNSS_CLOCKFACE",
        "instrument_dwt_cycles": "DWT_CLOCKFACE",
        "instrument_ocxo1_ns": "OCXO_CLOCKFACES",
        "instrument_ocxo2_ns": "OCXO_CLOCKFACES",
    }
    for key, category in clockface_keys.items():
        target = int(expected.get(key) or 0)
        if target > 0 and int(observed.get(key) or 0) < target:
            if category not in pending:
                pending.append(category)

    expected_n = expected.get("welford_n")
    observed_n = observed.get("welford_n")
    if isinstance(expected_n, dict):
        observed_n = observed_n if isinstance(observed_n, dict) else {}
        if any(
            int(target or 0) > 0
            and int(observed_n.get(key) or 0) < int(target or 0)
            for key, target in expected_n.items()
        ):
            pending.append("STATISTICS")

    control_pending = (
        str(observed.get("servo_mode") or "OFF").upper()
        != str(expected.get("servo_mode") or "OFF").upper()
        or bool(observed.get("dither_operator_enabled"))
        != bool(expected.get("dither_operator_enabled"))
    )
    for key in ("ocxo1_dac", "ocxo2_dac"):
        target = expected.get(key)
        value = observed.get(key)
        if target is not None and (
            value is None or abs(float(value) - float(target)) > 0.01
        ):
            control_pending = True
    if control_pending:
        pending.append("DAC_CONTROL")

    return pending or ["FRESH_MONITOR_PROOF"]


def _wait_for_monitor_restore(
    checkpoint: Dict[str, Any],
    *,
    requested_monotonic: float,
    timeout_s: float = MONITOR_RESTORE_TIMEOUT_S,
) -> Dict[str, Any]:
    """Wait for a fresh unified MONITOR proving the staged restore committed."""
    expected = _monitor_restore_probe(checkpoint)
    deadline = time.monotonic() + float(timeout_s)
    next_progress_log = requested_monotonic + 10.0
    last_observed: Dict[str, Any] = {}
    while time.monotonic() < deadline:
        with _SYSTEM_LOCK:
            monitor = copy.deepcopy(SYSTEM)
            received = _LATEST_MONITOR_RECEIVED_MONOTONIC
        if received is not None and received > requested_monotonic and monitor:
            last_observed = _monitor_restore_probe(monitor)
            if _monitor_restore_probe_satisfied(expected, last_observed):
                return {
                    "proved": True,
                    "expected": expected,
                    "observed": last_observed,
                    "monitor_sequence": monitor.get("sequence"),
                    "waited_s": round(
                        max(0.0, time.monotonic() - requested_monotonic),
                        3,
                    ),
                }

        now = time.monotonic()
        if now >= next_progress_log:
            pending = _monitor_restore_pending_categories(expected, last_observed)
            logging.info(
                "⏳ [system/recovery] MONITOR restore still converging after %.1fs; "
                "waiting for %s",
                max(0.0, now - requested_monotonic),
                ", ".join(pending),
            )
            next_progress_log = now + 10.0
        time.sleep(0.1)

    raise TimeoutError(
        "timed out waiting for restored MONITOR state "
        f"after {timeout_s:.1f}s; expected={expected!r} observed={last_observed!r}"
    )


def _recover_monitor_checkpoint(checkpoint: Dict[str, Any]) -> Dict[str, Any]:
    """Restore always-on Teensy and Pi clock state without campaign policy."""
    restore_state = _monitor_restore_state(checkpoint)
    if restore_state is None:
        raise RuntimeError("config.MONITOR has no structured firmware restore state")
    gnss_raw = _monitor_gnss_raw_payload(checkpoint)
    if gnss_raw is None:
        raise RuntimeError("config.MONITOR has no clocks.gnss_raw state")

    logging.info(
        "♻️ [system/recovery] found recoverable MONITOR checkpoint sequence=%s; "
        "seeding last-known display state and restoring the always-on instrument",
        checkpoint.get("sequence"),
    )
    _seed_system_from_checkpoint(checkpoint)
    requested_monotonic = time.monotonic()
    logging.info(
        "⏳ [system/recovery] last-known MONITOR published; requesting Teensy "
        "clockface, statistics, DAC, servo, and dither restoration"
    )
    teensy_response = _request_teensy_monitor_restore(restore_state)
    teensy_payload = (
        teensy_response.get("payload", {})
        if isinstance(teensy_response, dict)
        and isinstance(teensy_response.get("payload"), dict)
        else {}
    )
    teensy_status = str(teensy_payload.get("status") or "")
    teensy_campaign_state = str(
        teensy_payload.get("campaign_state") or ""
    ).strip().upper()
    if (
        teensy_status == "monitor_restore_rejected_busy"
        and teensy_campaign_state == "STARTED"
    ):
        result = {
            "success": True,
            "mode": "LIVE_CAMPAIGN_SUPERSEDED_MONITOR_RESTORE",
            "checkpoint_sequence": checkpoint.get("sequence"),
            "restore_schema_version": 2,
            "teensy": teensy_payload,
            "clocks": None,
            "proof": {
                "proved": True,
                "basis": "TEENSY_LIVE_CAMPAIGN_CUSTODY",
                "campaign_state": teensy_campaign_state,
            },
        }
        logging.info(
            "♻️ [system/recovery] live Teensy campaign superseded MONITOR "
            "restore: sequence=%s campaign_state=%s; checkpoint writing may resume",
            checkpoint.get("sequence"),
            teensy_campaign_state,
        )
        return result

    clocks_response = _request_clocks_gnss_raw_restore(gnss_raw)
    logging.info(
        "⏳ [system/recovery] restore commands accepted; waiting for a fresh "
        "MONITOR to prove clockfaces, statistics, DACs, servo mode, and dithering"
    )
    proof = _wait_for_monitor_restore(
        checkpoint,
        requested_monotonic=requested_monotonic,
    )

    result = {
        "success": True,
        "mode": "SYSTEM_MONITOR",
        "checkpoint_sequence": checkpoint.get("sequence"),
        "restore_schema_version": 2,
        "teensy": teensy_response.get("payload")
            if isinstance(teensy_response, dict) else None,
        "clocks": clocks_response.get("payload")
            if isinstance(clocks_response, dict) else None,
        "proof": proof,
    }
    logging.info(
        "✅ [system/recovery] MONITOR restore proved by fresh sequence=%s: "
        "clockfaces and statistics resumed, servo=%s, waited=%.3fs",
        proof.get("monitor_sequence"),
        proof.get("observed", {}).get("servo_mode"),
        float(proof.get("waited_s") or 0.0),
    )
    return result


# ------------------------------------------------------------------
# Pi context poller and MONITOR aggregation
# ------------------------------------------------------------------

def _dict_copy(value: Any) -> dict:
    return dict(value) if isinstance(value, dict) else {}


def _seconds_to_hms(seconds: Any) -> str:
    try:
        total = max(0, int(seconds))
    except (TypeError, ValueError):
        total = 0
    return f"{total // 3600:02d}:{(total % 3600) // 60:02d}:{total % 60:02d}"


def _gnss_time_utc(gnss: Dict[str, Any]) -> Optional[str]:
    date = str(gnss.get("date") or "").strip()
    clock = str(gnss.get("time") or "").strip()
    if not date or not clock:
        return None
    return f"{date}T{clock}Z"


def _clocks_monitor_snapshot() -> tuple[Dict[str, Any], Optional[float], Optional[str]]:
    with _CLOCKS_MONITOR_LOCK:
        payload = dict(_LATEST_CLOCKS_MONITOR)
        received = _LATEST_CLOCKS_MONITOR_RECEIVED_MONOTONIC
        received_utc = _LATEST_CLOCKS_MONITOR_RECEIVED_UTC
    age_s = None if received is None else max(0.0, time.monotonic() - received)
    return payload, age_s, received_utc


def _combine_live_clocks(fragment: Dict[str, Any],
                         gnss: Dict[str, Any],
                         published_at_utc: str) -> Dict[str, Any]:
    """Combine Teensy physical clocks with Pi-owned GNSS_RAW/baseline state."""
    teensy_clocks = _dict_copy(fragment.get("clocks"))
    pi_clocks, pi_age_s, pi_received_utc = _clocks_monitor_snapshot()

    clocks = dict(teensy_clocks)
    clocks["schema"] = "CLOCKS_LIVE_V1"
    clocks["teensy_schema"] = teensy_clocks.get("schema")
    clocks["pi_schema"] = pi_clocks.get("schema")
    clocks["system_time_utc"] = published_at_utc
    clocks["gnss_time_utc"] = _gnss_time_utc(gnss)

    presentation_count = (
        clocks.get("teensy_pps_vclock_count")
        or clocks.get("campaign_seconds")
        or 0
    )
    clocks["display_elapsed"] = _seconds_to_hms(presentation_count)
    clocks["campaign_elapsed"] = (
        _seconds_to_hms(clocks.get("campaign_seconds"))
        if clocks.get("campaign_present")
        else None
    )
    clocks["instrument_elapsed"] = _seconds_to_hms(
        clocks.get("instrument_age_seconds")
    )
    completed_sequence = clocks.get("completed_pps_sequence")
    monitor_sequence = _monitor_fragment_count(fragment)
    try:
        clocks["teensy_clock_source_age_sequences"] = max(
            0, int(monitor_sequence) - int(completed_sequence)
        )
    except (TypeError, ValueError):
        clocks["teensy_clock_source_age_sequences"] = None

    clocks["pi"] = pi_clocks
    clocks["gnss_raw"] = _dict_copy(pi_clocks.get("gnss_raw"))
    clocks["extra_clocks"] = _dict_copy(pi_clocks.get("extra_clocks"))
    clocks["baseline"] = _dict_copy(pi_clocks.get("baseline"))
    clocks["stats_reset"] = _dict_copy(pi_clocks.get("stats_reset"))
    clocks["pi_clock_source_age_s"] = (
        None if pi_age_s is None else round(pi_age_s, 3)
    )
    clocks["pi_clock_source_received_at_utc"] = pi_received_utc
    clocks["pi_clock_source_fresh"] = bool(
        pi_age_s is not None and pi_age_s <= 2.5
    )
    clocks["complete_for_display"] = bool(
        teensy_clocks and pi_clocks.get("gnss_raw")
    )
    clocks["ephemeral"] = True
    clocks["persisted"] = False
    return clocks


def _monitor_fragment_count(fragment: Dict[str, Any]) -> Optional[int]:
    for key in ("sequence", "teensy_pps_vclock_count", "teensy_pps_count", "pps_count"):
        value = fragment.get(key)
        if value is None:
            continue
        try:
            return int(value)
        except (TypeError, ValueError):
            continue
    return None


def _monitor_fragment_teensy(fragment: Dict[str, Any]) -> dict:
    for key in ("teensy", "system"):
        value = fragment.get(key)
        if isinstance(value, dict):
            return dict(value)
    return {}


def _update_pi_context(snapshot: Dict[str, Any]) -> None:
    """Replace Pi-owned context while preserving the latest Teensy fragment."""
    global SYSTEM
    with _SYSTEM_LOCK:
        current = dict(SYSTEM)
        current.update(snapshot)
        current["features"] = _feature_tree_snapshot()
        SYSTEM = current


def system_poller() -> None:
    """Refresh only Pi-owned operational context.

    This thread deliberately issues no recurring Teensy commands. The current
    Teensy/process/transport/Payload surfaces arrive through MONITOR_FRAGMENT.
    """
    try:
        while True:
            pi_payload = build_pi_status()
            network_payload = build_network_status()
            sensor_payload = build_sensor_scan_status()
            environment_payload = build_environment_status()
            gnss_payload = build_gnss_status()
            power_payload = build_power_status()
            battery_payload = build_battery_status()

            _update_builtin_pi_features(
                pi_payload=pi_payload,
                network_payload=network_payload,
                sensor_payload=sensor_payload,
                environment_payload=environment_payload,
                gnss_payload=gnss_payload,
                power_payload=power_payload,
                battery_payload=battery_payload,
                teensy_features_available=bool(_teensy_feature_tree_snapshot()),
            )

            pi_context = {
                "pi": dict(pi_payload),
                "network": dict(network_payload),
                "sensors": dict(sensor_payload),
                "environment": dict(environment_payload),
                "gnss": dict(gnss_payload),
                "power": dict(power_payload),
                "battery": dict(battery_payload),
            }
            _update_pi_context(pi_context)

            # Persist only the slow Pi context.  The 1 Hz MONITOR publication,
            # its Teensy fragment, and live CLOCKS block are intentionally
            # ephemeral latest-state telemetry.
            create_event("SYSTEM_STATUS", pi_context)
            time.sleep(POLL_INTERVAL_SEC)

    except Exception:
        logging.exception("[system_poller] unhandled exception - poller thread terminating")


# ------------------------------------------------------------------
# Pub/Sub handlers
# ------------------------------------------------------------------

def on_gnss_announcement(payload: Optional[dict]) -> None:
    """Cache one predictive TPS1 announcement without command/response polling."""
    global _GNSS_ANNOUNCEMENT_RECEIVED, _GNSS_ANNOUNCEMENT_MALFORMED
    if not isinstance(payload, dict):
        _GNSS_ANNOUNCEMENT_MALFORMED += 1
        logging.warning("[system] ignoring malformed GNSS_ANNOUNCEMENT: %r", payload)
        return
    item = dict(payload)
    if (
        item.get("schema") != "GNSS_ANNOUNCEMENT_V1"
        or item.get("semantics") != "NEXT_PPS_UTC"
        or _parse_gnss_utc(item.get("next_utc")) is None
    ):
        _GNSS_ANNOUNCEMENT_MALFORMED += 1
        logging.warning("[system] ignoring malformed GNSS_ANNOUNCEMENT: %r", item)
        return
    item["_system_received_monotonic"] = time.monotonic()
    item["_system_received_at_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat().replace("+00:00", "Z")
    with _GNSS_MONITOR_LOCK:
        _GNSS_ANNOUNCEMENT_HISTORY.append(item)
    _GNSS_ANNOUNCEMENT_RECEIVED += 1


def on_clocks_monitor(payload: Optional[dict]) -> None:
    """Cache the latest Pi-owned CLOCKS decoration; never persist it."""
    global _LATEST_CLOCKS_MONITOR
    global _LATEST_CLOCKS_MONITOR_RECEIVED_MONOTONIC
    global _LATEST_CLOCKS_MONITOR_RECEIVED_UTC

    if not isinstance(payload, dict):
        logging.warning("[system] ignoring malformed CLOCKS_MONITOR: %r", payload)
        return

    received_utc = datetime.datetime.now(datetime.timezone.utc) \
        .isoformat() \
        .replace("+00:00", "Z")
    with _CLOCKS_MONITOR_LOCK:
        _LATEST_CLOCKS_MONITOR = dict(payload)
        _LATEST_CLOCKS_MONITOR_RECEIVED_MONOTONIC = time.monotonic()
        _LATEST_CLOCKS_MONITOR_RECEIVED_UTC = received_utc


def on_monitor_fragment(payload: Optional[dict]) -> None:
    """Decorate one Teensy MONITOR_FRAGMENT and rebroadcast it as MONITOR."""
    global SYSTEM
    global _LATEST_MONITOR_RECEIVED_MONOTONIC

    if not isinstance(payload, dict):
        logging.warning("[system] ignoring malformed MONITOR_FRAGMENT: %r", payload)
        return

    fragment = dict(payload)
    count = _monitor_fragment_count(fragment)

    teensy_features = _copy_feature_tree(fragment.get("features"))
    if teensy_features:
        _replace_teensy_feature_tree(teensy_features)

    features = _feature_tree_snapshot()
    with _SYSTEM_LOCK:
        current = dict(SYSTEM)

    published_at = datetime.datetime.now(datetime.timezone.utc)
    published_at_utc = published_at.isoformat().replace("+00:00", "Z")
    gnss, gnss_monitor = _gnss_announcement_snapshot(published_at)
    clocks = _combine_live_clocks(fragment, gnss, published_at_utc)
    gnss_time_utc = _gnss_time_utc(gnss)
    time_sanity = {
        "schema": "MONITOR_TIME_SANITY_V1",
        "gnss_utc": gnss_time_utc,
        "system_utc": published_at_utc,
        "gnss_source_fresh": bool(gnss_monitor.get("source_fresh")),
        "gnss_source_age_s": gnss_monitor.get("source_age_s"),
        "gnss_announcement_sequence": gnss_monitor.get("announcement_sequence"),
        "gnss_matched_target_utc": gnss_monitor.get("matched_target_utc"),
    }
    fragment_evidence = dict(fragment)
    # The normalized top-level clocks block is the display authority.  Avoid
    # duplicating that comparatively large object inside monitor_fragment.
    fragment_evidence.pop("clocks", None)

    monitor = {
        "schema": "MONITOR_V2",
        "source_schema": fragment.get("schema"),
        "sequence": count,
        "teensy_pps_vclock_count": count,
        "teensy_pps_count": count,
        "pps_count": count,
        "published_at_utc": published_at_utc,
        "pi": _dict_copy(current.get("pi")),
        "network": _dict_copy(current.get("network")),
        "sensors": _dict_copy(current.get("sensors")),
        "environment": _dict_copy(current.get("environment")),
        "gnss": gnss,
        "gnss_monitor": gnss_monitor,
        "time_sanity": time_sanity,
        "power": _dict_copy(current.get("power")),
        "battery": _dict_copy(current.get("battery")),
        "clocks": clocks,
        "teensy": _monitor_fragment_teensy(fragment),
        "process": _dict_copy(fragment.get("process")),
        "transport": _dict_copy(fragment.get("transport")),
        "payload": _dict_copy(fragment.get("payload")),
        "features": features,
        "monitor_fragment": fragment_evidence,
    }

    with _SYSTEM_LOCK:
        SYSTEM = dict(monitor)
        _LATEST_MONITOR_RECEIVED_MONOTONIC = time.monotonic()

    publish("MONITOR", monitor)
    _queue_monitor_checkpoint(monitor)


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

    _refresh_feature_payload_from_registry()
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
    Let pubsub discover SYSTEM before MONITOR recovery and active polling.
    """
    logging.info(
        "⏳ [system] waiting %.1fs for pubsub routing and Teensy initialization "
        "before MONITOR recovery or active polling",
        STARTUP_TEENSY_QUIET_DELAY_S,
    )
    time.sleep(STARTUP_TEENSY_QUIET_DELAY_S)
    logging.info(
        "✅ [system] startup quiet delay complete — MONITOR recovery may begin"
    )


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
                "MONITOR_FRAGMENT": on_monitor_fragment,
                GNSS_ANNOUNCEMENT_TOPIC: on_gnss_announcement,
                CLOCKS_MONITOR_TOPIC: on_clocks_monitor,
            },
            blocking=False,
        )

        startup_teensy_quiet_delay()

        # SYSTEM always restores the latest recoverable MONITOR, independent of
        # campaign state.  CLOCKS may later supersede this with TIMEBASE recovery
        # when an active campaign exists.  Keep the checkpoint writer closed
        # until this transaction has consumed the previous durable authority.
        checkpoint_writer_allowed = True
        checkpoint = _read_monitor_checkpoint()
        checkpoint_recoverable = bool(
            checkpoint is not None
            and _monitor_restore_state(checkpoint) is not None
            and _monitor_gnss_raw_payload(checkpoint) is not None
        )
        if checkpoint_recoverable:
            try:
                assert checkpoint is not None
                _recover_monitor_checkpoint(checkpoint)
            except Exception:
                checkpoint_writer_allowed = False
                logging.exception(
                    "💥 [system/recovery] MONITOR restore failed; preserving "
                    "config.MONITOR and continuing with the seeded last-known state"
                )
        else:
            logging.info(
                "ℹ️ [system/recovery] no structured-recoverable config.MONITOR; "
                "starting from live MONITOR_FRAGMENT state"
            )

        if checkpoint_writer_allowed:
            _start_monitor_checkpoint_writer()
        else:
            logging.warning(
                "🧤 [system] MONITOR checkpoint writer held closed to preserve "
                "the last recoverable checkpoint after restore failure"
            )

        threading.Thread(
            target=system_poller,
            daemon=True,
            name="system-poller",
        ).start()

        logging.info("🏁 [system] entering main loop")
        while True:
            time.sleep(3600)

    except Exception:
        logging.exception("💥 [system] unhandled exception in main thread")


if __name__ == "__main__":
    run()
