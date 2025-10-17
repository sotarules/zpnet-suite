"""
ZPNet Power Monitor  —  Stellar-Compliant Revision

Polls INA260 sensors over I²C and emits POWER_STATUS events.
Also emits POWER_EXCEPTION events when any rail voltage drifts
outside its expected tolerance.

Author: The Mule
"""

import logging
from smbus2 import SMBus
from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event

# ---------------------------------------------------------------------
# INA260 device configuration
# ---------------------------------------------------------------------
DEVICE_CONFIG = {
    0x40: {"label": "Battery",   "ideal_voltage_v": 12.8, "max_dev_pct": 10},
    0x41: {"label": "3v3 Rail",  "ideal_voltage_v": 3.3,  "max_dev_pct": 5},
    0x44: {"label": "5v0 Rail",  "ideal_voltage_v": 5.0,  "max_dev_pct": 5},
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
    Raises:
        Any underlying I²C exceptions for upper-level handling.
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
# Main routine
# ---------------------------------------------------------------------
def run():
    """
    Poll INA260 rails and emit POWER_STATUS plus any POWER_EXCEPTION events.

    Emits:
        POWER_STATUS: full sensor snapshot.
        POWER_EXCEPTION: any rail voltage out of tolerance.
    """
    sensors = []
    exceptions = []
    bus = None
    try:
        bus = SMBus(1)
        for addr, cfg in DEVICE_CONFIG.items():
            try:
                result = read_ina260(bus, addr)
                sensors.append(result)

                ideal = cfg["ideal_voltage_v"]
                tol = cfg["max_dev_pct"]
                v_min = ideal * (1 - tol / 100)
                v_max = ideal * (1 + tol / 100)
                actual = result["voltage_v"]

                if not (v_min <= actual <= v_max):
                    exceptions.append({
                        "address": result["address"],
                        "label": result["label"],
                        "ideal_voltage_v": ideal,
                        "actual_voltage_v": actual,
                    })

            except Exception as e:
                logging.error(f"I²C read failure at 0x{addr:02X}: {e}")
                continue

    finally:
        try:
            bus.close()
        except Exception:
            pass

    # Emit raw sensor snapshot
    if sensors:
        create_event("POWER_STATUS", {"sensors": sensors})

    # Emit exceptions for out-of-range rails
    for ex in exceptions:
        create_event(
            "POWER_EXCEPTION",
            {
                "address": ex["address"],
                "label": ex["label"],
                "ideal_voltage_v": ex["ideal_voltage_v"],
                "actual_voltage_v": ex["actual_voltage_v"],
                "alert": True,
            },
        )


def bootstrap():
    """Setup logging and execute run() once."""
    setup_logging()
    run()
