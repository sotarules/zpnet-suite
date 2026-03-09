"""
ZPNet SYSTEM Process (Pi-side, authoritative aggregator)

Responsibilities:
  • Query authoritative SYSTEM state from the Teensy
  • Collect Raspberry Pi host metrics
  • Maintain last-known-good SYSTEM snapshot
  • Expose SYSTEM.REPORT
  • Emit SYSTEM_STATUS events on fixed cadence

Process model:
  • One systemd service
  • One polling thread
  • One blocking command socket
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
from typing import Dict, Tuple, Optional

import psutil
import requests
from smbus2 import SMBus

from zpnet.processes.processes import send_command, server_setup
from zpnet.shared.constants import (
    ZPNET_REMOTE_HOST,
    ZPNET_TEST_PATH,
    HTTP_TIMEOUT,
    EXPECTED_TEST_STRING,
)
from zpnet.shared.db import open_db
from zpnet.shared.events import create_event
from zpnet.shared.http import gzip_text
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import normalize_payload, normalize_ts

# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

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
        0x76: "BME280 0x76 (Environment)"
        # 0x57 will appear via scan but is ignored (EEPROM)
    },

    I2C_BUS_EXPANDED: {
        0x40: "INA260 0x40 (Pi Domain)",
        0x41: "INA260 0x41 (24V Rail / Motors)",
        0x44: "INA260 0x44 (OCXO Domain)",
        0x45: "INA260 0x45 (Teensy Domain)"
        # 0x57 will appear via scan but is ignored (EEPROM)
    },
}

# ------------------------------------------------------------------
# Power monitoring (legacy: power_monitor)
# ------------------------------------------------------------------

POWER_CONFIG_BY_BUS = {
    I2C_BUS_LEGACY: {
        0x40: {"label": "3.3v Rail", "ideal_voltage_v": 3.3},
        0x41: {"label": "5v Rail", "ideal_voltage_v": 5.0},
        0x44: {"label": "Battery", "ideal_voltage_v": 12.8},
    },
    I2C_BUS_EXPANDED: {
        0x40: {"label": "Pi Domain", "ideal_voltage_v": 5.0},
        0x41: {"label": "24v Domain", "ideal_voltage_v": 24.0},
        0x44: {"label": "OCXO Domain", "ideal_voltage_v": 5.0},
        0x45: {"label": "Teensy Domain", "ideal_voltage_v": 5.0},
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

REG_CURRENT = 0x01
REG_VOLTAGE = 0x02
REG_POWER   = 0x03

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
    url = f"http://{ZPNET_REMOTE_HOST}{ZPNET_TEST_PATH}"
    resp = requests.get(url, timeout=HTTP_TIMEOUT)
    return resp.status_code == 200 and EXPECTED_TEST_STRING in resp.text

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
            except Exception:
                logging.exception("[system] speed tests failed")

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


def build_environment_status() -> dict:
    """
    Read environmental data from BME280.

    Semantics:
      • Either returns a complete, truthful payload
      • Or raises
    """
    with open_i2c(I2C_BUS_LEGACY) as bus:
        chip_id = bus.read_byte_data(BME280_ADDR, REG_ID)
        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(f"unexpected BME280 chip ID 0x{chip_id:02X}")

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

        return {
            "sensor_address": "0x76",
            "sensor_present": True,
            "temperature_c": round(temp_c, 2),
            "pressure_hpa": round(pressure_hpa, 2),
            "humidity_pct": round(humidity_pct, 2),
            "altitude_m": round(altitude_m, 1),
            "health_state": "NOMINAL",
        }

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


def build_power_status() -> dict:
    """
    Build power rail snapshot from INA260 sensors.

    Semantics:
      • Observation only
      • No inference
      • No per-rail fault masking
      • Boundary at SMBus acquisition

    Mongo / MiniMongo constraint:
      • Dictionary keys must NOT contain '.'.
      • Therefore, per-rail keys are address-derived (e.g., "0x41").
      • Human-readable domain names live in the inner "label" field.
    """
    snapshot: dict[str, dict[str, dict]] = {}

    for bus_id, devices in POWER_CONFIG_BY_BUS.items():
        bus_key = f"i2c-{bus_id}"
        results: dict[str, dict] = {}
        snapshot[bus_key] = results

        with open_i2c(bus_id) as bus:
            for addr, cfg in devices.items():
                reading = read_ina260(bus_id, bus, addr)
                label = cfg["label"]

                # MiniMongo-safe key: address only (no periods)
                rail_key = f"0x{addr:02X}"

                results[rail_key] = {
                    "label": label,
                    "address": rail_key,
                    "ideal_voltage_v": cfg["ideal_voltage_v"],
                    "volts": reading["volts"],
                    "amps": reading["amps"],
                    "watts": reading["watts"],
                }

    return {
        "health_state": "NOMINAL",
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
            laser_payload = build_laser_status()
            sensor_payload = build_sensor_scan_status()
            environment_payload = build_environment_status()
            gnss_payload = build_gnss_status()
            power_payload = build_power_status()
            battery_payload = build_battery_status()
            clocks_payload = build_clocks_status()
            transport_payload = build_transport_status()
            payload_payload = build_payload_status()
            memory_payload = build_memory_status()
            process_payload = build_process_status()

            SYSTEM = {
                "pi": dict(pi_payload),
                "teensy": dict(teensy_payload),
                "network": dict(network_payload),
                "laser": dict(laser_payload),
                "sensors": dict(sensor_payload),
                "environment": dict(environment_payload),
                "gnss": dict(gnss_payload),
                "power": dict(power_payload),
                "battery": dict(battery_payload),
                "clocks": dict(clocks_payload),
                "transport": dict(transport_payload),
                "payload": dict(payload_payload),
                "memory": dict(memory_payload),
                "process": dict(process_payload),
            }

            # ----------------------------------------------------------
            # Emit consolidated system event
            # ----------------------------------------------------------
            create_event("SYSTEM_STATUS", dict(SYSTEM))

            time.sleep(POLL_INTERVAL_SEC)

    except Exception:
        logging.exception("[system_poller] unhandled exception - poller thread terminating")


# ------------------------------------------------------------------
# Command handlers
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
    "REPORT": cmd_report
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    try:
        threading.Thread(
            target=system_poller,
            daemon=True,
        ).start()

        server_setup(
            subsystem="SYSTEM",
            commands=COMMANDS
        )

    except Exception:
        logging.exception("💥 [system] unhandled exception in main thread")


if __name__ == "__main__":
    run()
