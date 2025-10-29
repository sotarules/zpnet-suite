"""
ZPNet Raspberry Pi Monitor — Stellar-Compliant + Constants-Integrated Revision (v2025-10-28c)

Collects Raspberry Pi system metrics and emits a RASPBERRY_PI_STATUS event.
This subsystem parallels the Teensy telemetry stream, providing host-level
status such as CPU temperature, load averages, memory usage, disk usage,
uptime, and network I/O statistics.

Now imports shared constants (DB_PATH, etc.) from zpnet.shared.constants
to maintain symmetry with other ZPNet modules.

Author: The Mule
"""

import logging
import os
import platform
import subprocess
import time
from pathlib import Path
from typing import Tuple

import psutil

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging
from zpnet.shared.constants import DB_PATH  # ← NEW (for consistency / future use)

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
CPU_TEMP_PATHS = [
    Path("/sys/class/thermal/thermal_zone0/temp"),  # standard Pi path
    Path("/sys/class/hwmon/hwmon0/temp1_input"),    # alternate hwmon path
]
DEVICE_NAME = platform.node() or "raspberrypi"

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
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
        result = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError("vcgencmd failed")
        line = result.stdout.strip()
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


# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Gather Raspberry Pi host metrics and emit a RASPBERRY_PI_STATUS event.

    Emits:
        RASPBERRY_PI_STATUS: contains CPU, memory, disk, uptime, and network stats.
    """
    try:
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

        create_event("RASPBERRY_PI_STATUS", payload)

    except Exception as e:
        logging.exception(f"🔥 [pi_monitor] unexpected failure: {e}")
        raise


def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
