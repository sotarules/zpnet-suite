"""
ZPNet SYSTEM Process (Pi-side, authoritative aggregator)

This process replaces multiple Pi-side monitors with a single
system_monitor service exposing a unified SYSTEM.REPORT surface.

Responsibilities:
  • Query authoritative SYSTEM state from the Teensy
  • Collect Raspberry Pi host metrics (formerly pi_monitor)
  • Maintain a last-known-good system snapshot
  • Expose system-wide facts via REPORT
  • Emit SYSTEM_STATUS events on a fixed cadence
  • (Transitional) Emit legacy RASPBERRY_PI_STATUS for compatibility
  • Serve as the future consolidation point for system observability

Design invariants:
  • Read-only from the Pi perspective
  • Single-writer semantics (Teensy owns mutation)
  • Deterministic polling cadence
  • Snapshot replacement (last-known-good)

Process model:
  • One systemd service
  • One polling thread (Teensy PROCESS.COMMAND + Pi metrics)
  • One blocking command socket (REPORT)
"""

from __future__ import annotations

import json
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
from typing import Dict, Optional, Tuple

import psutil
import requests
from smbus2 import SMBus

from zpnet.shared.constants import ZPNET_REMOTE_HOST, ZPNET_TEST_PATH, HTTP_TIMEOUT, EXPECTED_TEST_STRING
from zpnet.shared.events import create_event
from zpnet.shared.http import gzip_text
from zpnet.shared.logger import setup_logging
from zpnet.shared.teensy import send_command

# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

CMD_SOCKET_PATH = "/tmp/zpnet-system.sock"
POLL_INTERVAL_SEC = 30

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

I2C_BUS  = 1

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
# Sensor Scan (legacy: sensor_monitor)
# ------------------------------------------------------------------

I2C_DEVICES = {
    0x40: "INA260 0x40 (Battery)",
    0x41: "INA260 0x41 (3v3 Rail)",
    0x44: "INA260 0x44 (5v0 Rail)",
    0x45: "INA260 0x45 (5v Teensy Rail)",
    0x76: "BME280 0x76 (Environment)",
    0x66: "EV5491 0x66 (Laser Controller)",
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

# ------------------------------------------------------------------
# Power monitoring (legacy: power_monitor)
# ------------------------------------------------------------------

DEVICE_CONFIG = {
    0x40: {"label": "Battery",          "ideal_voltage_v": 12.8},
    0x41: {"label": "3v3 Rail",         "ideal_voltage_v": 3.3},
    0x44: {"label": "5v0 Rail",         "ideal_voltage_v": 5.0},
    0x45: {"label": "5v0 Teensy Rail",  "ideal_voltage_v": 5.0},
}

REG_CURRENT = 0x01
REG_VOLTAGE = 0x02
REG_POWER   = 0x03

I2C_BUS = 1


# ------------------------------------------------------------------
# Authoritative SYSTEM snapshot
# ------------------------------------------------------------------

# Snapshot schema (phase 2):
#   {
#     "teensy": { ... payload from Teensy SYSTEM.REPORT ... },
#     "pi":     { ... Raspberry Pi host metrics ... }
#   }
SYSTEM: Dict[str, object] = {}


# ------------------------------------------------------------------
# Raspberry Pi helpers (migrated from pi_monitor)
# ------------------------------------------------------------------

def get_cpu_temp_c() -> float | None:
    """Read CPU temperature in °C from sysfs or psutil fallback."""
    try:
        for path in CPU_TEMP_PATHS:
            if path.exists():
                val = float(path.read_text().strip()) / 1000.0
                return round(val, 1)

        temps = psutil.sensors_temperatures()
        if temps:
            for entries in temps.values():
                for entry in entries:
                    if "cpu" in entry.label.lower() or "core" in entry.label.lower():
                        return round(entry.current, 1)

        return None

    except Exception:
        return None


def get_uptime_seconds() -> float:
    """Return system uptime in seconds."""
    return time.time() - psutil.boot_time()


def get_load_average() -> Tuple[float, float, float]:
    """Return (1 min, 5 min, 15 min) load averages."""
    try:
        one, five, fifteen = os.getloadavg()
        return (round(one, 2), round(five, 2), round(fifteen, 2))
    except Exception:
        return (0.0, 0.0, 0.0)


def get_memory_stats() -> dict:
    """Return memory usage metrics in MB."""
    mem = psutil.virtual_memory()
    return {
        "total_mb": round(mem.total / 1e6, 1),
        "used_mb": round(mem.used / 1e6, 1),
        "available_mb": round(mem.available / 1e6, 1),
        "percent": mem.percent,
    }


def get_disk_stats() -> dict:
    """Return disk usage for root filesystem in GB."""
    disk = psutil.disk_usage("/")
    return {
        "total_gb": round(disk.total / 1e9, 2),
        "used_gb": round(disk.used / 1e9, 2),
        "free_gb": round(disk.free / 1e9, 2),
        "percent": disk.percent,
    }


def get_network_bytes() -> dict:
    """Return aggregate RX/TX bytes across all interfaces."""
    counters = psutil.net_io_counters()
    return {
        "bytes_sent": counters.bytes_sent,
        "bytes_recv": counters.bytes_recv,
        "packets_sent": counters.packets_sent,
        "packets_recv": counters.packets_recv,
    }


def get_undervoltage_flags() -> dict:
    """
    Query `vcgencmd get_throttled` and decode undervoltage condition flags.
    Requires the vcgencmd binary to be available (normally present on RPi).
    """
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise RuntimeError("vcgencmd failed")

        line = (result.stdout or "").strip()
        if not line.startswith("throttled="):
            raise ValueError("unexpected vcgencmd output")

        raw_hex = line.split("=")[-1]
        flags = int(raw_hex, 16)

        return {
            "raw_hex": raw_hex,
            "currently_undervolted": bool(flags & (1 << 0)),
            "previously_undervolted": bool(flags & (1 << 16)),
        }

    except Exception:
        return {
            "raw_hex": "unavailable",
            "currently_undervolted": None,
            "previously_undervolted": None,
        }


def build_pi_status() -> dict:
    """Gather Raspberry Pi host metrics."""

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

    # Health classification (simple heuristic)
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
    """Return the currently associated Wi-Fi SSID, or empty string if none."""
    try:
        result = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True,
            check=True,
            text=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return ""

def zpnet_definitive_test() -> bool:
    """Definitive connectivity test against ZPNet API endpoint."""
    try:
        url = f"http://{ZPNET_REMOTE_HOST}{ZPNET_TEST_PATH}"
        resp = requests.get(url, timeout=HTTP_TIMEOUT)
        return resp.status_code == 200 and EXPECTED_TEST_STRING in resp.text
    except requests.RequestException:
        return False

def get_local_ip() -> str:
    """Best-effort local IP address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()

def ping_latency_ms() -> float:
    """Approximate latency (ms) via TCP connect timing."""
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
    """Return RX/TX bytes per interface."""
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
    """Estimate download throughput via a 1MB GET."""
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
    """Estimate upload throughput via gzip-compressed POST."""
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
    Gather network status snapshot.

    This is a direct refactor of network_monitor.run(),
    returning a payload instead of emitting events.
    """

    payload: dict = {}

    try:
        payload["ssid"] = get_ssid()

        test_ok = zpnet_definitive_test()
        payload["network_status"] = "NOMINAL" if test_ok else "DOWN"

        if not test_ok:
            return payload

        payload["local_ip"] = get_local_ip()
        payload["interfaces"] = get_interface_stats()
        payload["ping_ms"] = ping_latency_ms()

        try:
            payload["download_mbps"] = download_test_mbps()
            payload["upload_mbps"] = upload_test_mbps()
        except requests.RequestException:
            pass
        except Exception:
            pass

    except Exception:
        # Absolute containment — match legacy behavior
        pass

    return payload

# ------------------------------------------------------------------
# Laser helpers (migrated from laser_monitor)
# ------------------------------------------------------------------

def read_mp5491_registers() -> dict:
    """
    Read all MP5491 registers over I²C.

    Returns:
        dict[str, int]: register name → raw byte value
    """
    bus = SMBus(I2C_BUS)
    try:
        return {
            name: bus.read_byte_data(LASER_ADDR, addr)
            for name, addr in REGS.items()
        }
    finally:
        bus.close()

def decode_mp5491_state(vals: dict) -> dict:
    """
    Decode MP5491 configuration, status, and protection flags.
    """

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

def query_teensy_laser_status() -> dict:
    """
    Query Teensy LASER.REPORT for optical ground truth.
    """
    try:
        resp = send_command(
            "PROCESS.COMMAND",
            {
                "type": "LASER",
                "proc_cmd": "REPORT",
            }
        )
    except Exception:
        return {}

    if not resp or not resp.get("success"):
        return {}

    payload = resp.get("payload", {}) or {}

    return {
        "pd_voltage": payload.get("PD_voltage"),
        "laser_emitting": payload.get("laser_emitting"),
    }

def build_laser_status() -> dict:
    """
    Build consolidated laser status snapshot.

    Combines:
      • MP5491 I²C capability truth (Pi-side)
      • Optical emission truth (Teensy-side)
    """

    payload = {
        "i2c_address": "0x66",
        "device_present": False,
        "health_state": "UNKNOWN",
    }

    try:
        # ----------------------------------------------------------
        # I²C capability truth
        # ----------------------------------------------------------
        vals = read_mp5491_registers()
        decoded = decode_mp5491_state(vals)

        payload.update({
            "device_present": True,
            **decoded,

            "raw_registers": {
                f"0x{addr:02X}": f"0x{vals[name]:02X}"
                for name, addr in REGS.items()
            },
        })

        # ----------------------------------------------------------
        # Optical ground truth (Teensy)
        # ----------------------------------------------------------
        teensy = query_teensy_laser_status()
        payload.update(teensy)

        payload["health_state"] = "NOMINAL"

    except Exception:
        payload["health_state"] = "DOWN"

    return payload

# ------------------------------------------------------------------
# Sensor helpers (migrated from sensor_monitor)
# ------------------------------------------------------------------

def sensor_scan_probe_device(bus: SMBus, addr: int, label: str) -> str:
    """
    Probe a single I²C device.

    Returns:
        "NOMINAL" | "OFFLINE"
    """
    try:
        if "BME280" in label:
            chip_id = bus.read_byte_data(addr, BME280_ID_REG)
            if chip_id != BME280_EXPECTED_ID:
                raise RuntimeError(
                    f"unexpected BME280 chip ID 0x{chip_id:02X}"
                )
        else:
            # Generic safe probe
            bus.read_byte_data(addr, 0x00)

        return "NOMINAL"

    except Exception as e:
        logging.info(f"[system_poller] sensor_scan {label} offline: {e}")
        return "OFFLINE"


def build_sensor_scan_status() -> dict:
    """
    Build sensor scan snapshot.

    Legacy origin:
        zpnet.modules.sensor_monitor

    Semantics:
        • Best-effort
        • Read-only
        • No events
        • No interpretation
    """
    results: dict[str, str] = {}
    bus: SMBus | None = None

    try:
        bus = SMBus(I2C_BUS)

        for addr, label in I2C_DEVICES.items():
            results[label] = sensor_scan_probe_device(bus, addr, label)

    except Exception as e:
        logging.exception("[system_poller] sensor_scan I²C bus unavailable")
        for label in I2C_DEVICES.values():
            results[label] = "OFFLINE"

    finally:
        try:
            if bus:
                bus.close()
        except Exception:
            pass

    return results

# ------------------------------------------------------------------
# Enviornment helpers (migrated from environment_monitor)
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

    if var1 == 0:
        return 0.0

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

def build_environment_status() -> dict:
    payload = {
        "sensor_address": "0x76",
        "sensor_present": False,
        "health_state": "DOWN",
    }

    bus = None
    try:
        bus = SMBus(1)

        chip_id = bus.read_byte_data(BME280_ADDR, REG_ID)
        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(f"unexpected BME280 chip ID 0x{chip_id:02X}")

        payload["sensor_present"] = True

        # Forced mode, oversampling x1
        bus.write_byte_data(BME280_ADDR, REG_CTRL_HUM, 0x01)
        bus.write_byte_data(BME280_ADDR, REG_CTRL_MEAS, 0x27)

        cal = read_bme280_calibration(bus)

        data = bus.read_i2c_block_data(BME280_ADDR, REG_DATA, 8)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_H = (data[6] << 8) | data[7]

        temp_c, t_fine = compensate_temperature(adc_T, cal)
        pressure_hpa = compensate_pressure(adc_P, t_fine, cal)
        humidity_pct = compensate_humidity(adc_H, t_fine, cal)

        altitude_m = 44330.0 * (1.0 - (pressure_hpa / SEA_LEVEL_PRESSURE_HPA) ** 0.1903)

        payload.update(
            {
                "temperature_c": round(temp_c, 2),
                "pressure_hpa": round(pressure_hpa, 2),
                "humidity_pct": round(humidity_pct, 2),
                "altitude_m": round(altitude_m, 1),
                "health_state": "NOMINAL",
            }
        )

    except Exception:
        logging.exception("[system] environment_status collection failed")

    finally:
        try:
            if bus:
                bus.close()
        except Exception:
            pass

    return payload

# ------------------------------------------------------------------
# Power monitoring helpers (legacy: power_monitor)
# ------------------------------------------------------------------

def read_word(bus: SMBus, addr: int, reg: int) -> int:
    """Read a 16-bit INA260 register with byte swap."""
    raw = bus.read_word_data(addr, reg)
    return ((raw & 0xFF) << 8) | (raw >> 8)

def read_ina260(bus: SMBus, addr: int) -> dict:
    """
    Read current, voltage, and power from an INA260.

    Returns raw, ground-truth measurements only.
    """
    current_raw = read_word(bus, addr, REG_CURRENT)
    voltage_raw = read_word(bus, addr, REG_VOLTAGE)
    power_raw   = read_word(bus, addr, REG_POWER)

    # Sign-extend current
    if current_raw & 0x8000:
        current_raw -= 1 << 16

    current_ma = current_raw * 1.0          # 1 mA/LSB
    voltage_v  = voltage_raw * 0.00125      # 1.25 mV/LSB
    power_w    = (power_raw * 10) / 1000.0  # 10 mW/LSB → W

    cfg = DEVICE_CONFIG.get(addr, {})

    return {
        "address": f"0x{addr:02X}",
        "label": cfg.get("label", "Unknown"),
        "voltage_v": round(voltage_v, 3),
        "current_ma": round(current_ma, 2),
        "power_w": round(power_w, 3),
        "ideal_voltage_v": cfg.get("ideal_voltage_v"),
    }

def build_power_status() -> dict:
    """
    Build power rail snapshot from INA260 sensors.

    Legacy origin:
        zpnet.modules.power_monitor

    Semantics:
        • Observation only
        • No inference
        • No shutdown
        • No tolerance enforcement
        • No events
    """

    payload = {
        "bus": "i2c-1",
        "health_state": "UNKNOWN",
        "rails": [],
    }

    bus = None
    try:
        bus = SMBus(I2C_BUS)

        for addr in DEVICE_CONFIG:
            try:
                rail = read_ina260(bus, addr)
                payload["rails"].append(rail)
            except Exception as e:
                logging.exception(
                    f"[system_poller] power rail 0x{addr:02X} read failed"
                )
                payload["rails"].append({
                    "address": f"0x{addr:02X}",
                    "label": DEVICE_CONFIG[addr]["label"],
                    "health_state": "OFFLINE",
                })

        payload["health_state"] = "NOMINAL"

    except Exception:
        logging.exception("[system_poller] power_status I²C bus unavailable")
        payload["health_state"] = "DOWN"

    finally:
        try:
            if bus:
                bus.close()
        except Exception:
            pass

    return payload


# ------------------------------------------------------------------
# System poller thread
# ------------------------------------------------------------------

def system_poller() -> None:
    """
    Periodically request SYSTEM.REPORT from the Teensy and gather
    consolidated Pi-side system metrics.

    Semantics:
      • Teensy is authoritative but optional
      • Pi-side metrics are best-effort
      • No last-known-good preservation
      • Snapshot reflects current reality only
    """
    global SYSTEM

    while True:
        teensy_payload: dict | None = None

        try:
            resp = send_command(
                "PROCESS.COMMAND",
                {
                    "type": "SYSTEM",
                    "proc_cmd": "REPORT",
                },
            )
            if resp and resp.get("success"):
                teensy_payload = resp.get("payload", {})
        except Exception:
            teensy_payload = None

        try:
            pi_payload = build_pi_status()
            network_payload = build_network_status()
        except Exception:
            pi_payload = {
                "device_name": DEVICE_NAME,
                "health_state": "DOWN",
            }
            network_payload = {
                "health_state": "DOWN",
            }

        try:
            laser_payload = build_laser_status()
        except Exception:
            logging.exception("[system_poller] laser_status collection failed")
            laser_payload = {
                "device_present": False,
                "health_state": "DOWN",
                "i2c_address": "0x66",
            }

        try:
            sensor_payload = build_sensor_scan_status()
        except Exception:
            sensor_payload = {}

        try:
            environment_payload = build_environment_status()
        except Exception:
            logging.exception("[system_poller] environment_status collection failed")
            environment_payload = {
                "sensor_address": "0x76",
                "sensor_present": False,
                "health_state": "DOWN",
            }

        try:
            power_payload = build_power_status()
        except Exception:
            power_payload = {
                "health_state": "DOWN",
                "rails": [],
            }

        SYSTEM = {
            "teensy": teensy_payload,
            "pi": dict(pi_payload),
            "network": dict(network_payload),
            "laser": dict(laser_payload),
            "sensors": dict(sensor_payload),
            "environment": dict(environment_payload),
            "power": dict(power_payload),
        }

        # ----------------------------------------------------------
        # Emit consolidated system event
        # ----------------------------------------------------------
        create_event("SYSTEM_STATUS", dict(SYSTEM))

        time.sleep(POLL_INTERVAL_SEC)


# ------------------------------------------------------------------
# REPORT command
# ------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict:
    """Return the most recent SYSTEM snapshot."""
    snapshot = dict(SYSTEM)

    return {
        "success": True,
        "message": "OK",
        "payload": snapshot,
    }


COMMANDS = {
    "REPORT": cmd_report,
}


# ------------------------------------------------------------------
# Command socket server
# ------------------------------------------------------------------

def command_server() -> None:
    try:
        os.unlink(CMD_SOCKET_PATH)
    except FileNotFoundError:
        pass

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as srv:
        srv.bind(CMD_SOCKET_PATH)
        srv.listen()

        while True:
            conn, _ = srv.accept()
            with conn:
                data = conn.recv(65536)
                if not data:
                    continue

                try:
                    req = json.loads(data.decode())
                except Exception:
                    resp = {
                        "success": False,
                        "message": "invalid json",
                    }
                    conn.sendall(json.dumps(resp).encode())
                    continue

                handler = COMMANDS.get(req.get("cmd"))
                resp = handler(req.get("args")) if handler else {
                    "success": False,
                    "message": "unrecognized command",
                }

                conn.sendall(json.dumps(resp).encode())


# ------------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------------

def run() -> None:
    setup_logging()
    threading.Thread(target=system_poller, daemon=True).start()
    command_server()
