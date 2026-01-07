"""
ZPNet Power Monitor — Protective Shutdown Revision (v2026-01-07)

Polls INA260 sensors over I²C and emits POWER_STATUS events.

If any rail voltage drifts outside its expected tolerance, the system
is immediately shut down to protect hardware integrity.

POWER_EXCEPTION events are intentionally retired.
Out-of-tolerance is no longer a reportable condition; it is terminal.

Author: The Mule
"""

import logging
import subprocess
from smbus2 import SMBus

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.constants import DB_PATH  # retained for structural symmetry


# ---------------------------------------------------------------------
# INA260 device configuration
# ---------------------------------------------------------------------
DEVICE_CONFIG = {
    0x40: {"label": "Battery",   "ideal_voltage_v": 12.8, "max_dev_pct": 10},
    0x41: {"label": "3v3 Rail",  "ideal_voltage_v": 3.3,  "max_dev_pct": 5},
    0x44: {"label": "5v0 Rail",  "ideal_voltage_v": 5.0,  "max_dev_pct": 5},
    0x45: {"label": "24v Spur",  "ideal_voltage_v": 24.0, "max_dev_pct": 5},
}

# INA260 registers
REG_CURRENT = 0x01
REG_VOLTAGE = 0x02
REG_POWER   = 0x03


# ---------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------
def read_word(bus: SMBus, addr: int, reg: int) -> int:
    """Read a 16-bit register with byte swap for endianness."""
    raw = bus.read_word_data(addr, reg)
    return ((raw & 0xFF) << 8) | (raw >> 8)


def read_ina260(bus: SMBus, addr: int) -> dict:
    """
    Read current, voltage, and power from an INA260.

    Returns:
        dict: {address, label, voltage_v, current_ma, power_w}
    """
    current_raw = read_word(bus, addr, REG_CURRENT)
    voltage_raw = read_word(bus, addr, REG_VOLTAGE)
    power_raw   = read_word(bus, addr, REG_POWER)

    # Sign-extend current (per INA260 datasheet)
    if current_raw & 0x8000:
        current_raw -= 1 << 16

    current_ma = current_raw * 1.0         # 1 mA/LSB
    voltage_v  = voltage_raw * 0.00125     # 1.25 mV/LSB
    power_w    = (power_raw * 10) / 1000.0 # 10 mW/LSB → W

    return {
        "address": f"0x{addr:02X}",
        "label": DEVICE_CONFIG[addr]["label"],
        "voltage_v": round(voltage_v, 3),
        "current_ma": round(current_ma, 2),
        "power_w": round(power_w, 3),
    }


# ---------------------------------------------------------------------
# Protective shutdown
# ---------------------------------------------------------------------
def shutdown_system(reason: str) -> None:
    """
    Immediately shut down the system due to a power safety violation.

    This function does not return.
    """
    logging.critical(f"🛑 [power_monitor] SYSTEM SHUTDOWN — {reason}")

    try:
        subprocess.run(
            ["systemctl", "poweroff", "--no-block"],
            check=False,
        )
    finally:
        # Absolute last resort: ensure this process does not continue.
        raise SystemExit(1)


# ---------------------------------------------------------------------
# Main routine
# ---------------------------------------------------------------------
def run():
    """
    Poll INA260 rails and emit POWER_STATUS.

    If any rail voltage exceeds its allowed tolerance, immediately
    shut down the system.
    """
    sensors = []
    bus = None

    try:
        bus = SMBus(1)

        for addr, cfg in DEVICE_CONFIG.items():
            result = read_ina260(bus, addr)
            sensors.append(result)

            ideal = cfg["ideal_voltage_v"]
            tol = cfg["max_dev_pct"]
            v_min = ideal * (1 - tol / 100)
            v_max = ideal * (1 + tol / 100)
            actual = result["voltage_v"]

            if not (v_min <= actual <= v_max):
                shutdown_system(
                    f"{cfg['label']} out of tolerance: "
                    f"{actual:.3f} V (expected {v_min:.2f}–{v_max:.2f} V)"
                )

    except SystemExit:
        raise

    except Exception as e:
        logging.exception(f"[power_monitor] unexpected failure: {e}")
        raise

    finally:
        try:
            if bus:
                bus.close()
        except Exception:
            pass

    # Emit raw sensor snapshot (only reached if system is safe)
    if sensors:
        create_event("POWER_STATUS", {"sensors": sensors})


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap():
    """Setup logging and execute run() once."""
    setup_logging()
    run()
