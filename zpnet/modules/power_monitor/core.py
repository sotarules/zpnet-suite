"""
ZPNet Power Monitor

Poll INA260 sensors and emit raw events.
"""

import logging
from smbus2 import SMBus
from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event

# INA260 I2C device config
DEVICE_CONFIG = {
    0x40: {"label": "Battery",     "ideal_voltage": 12.8, "max_deviation_pct": 10},
    0x41: {"label": "3.3V Rail",   "ideal_voltage": 3.3,  "max_deviation_pct": 5},
    0x44: {"label": "5V Rail",     "ideal_voltage": 5.0,  "max_deviation_pct": 5},
}

# INA260 registers
REG_CURRENT = 0x01
REG_VOLTAGE = 0x02
REG_POWER   = 0x03


# ----------------------
# Low-level helpers
# ----------------------
def read_word(bus, addr, reg):
    raw = bus.read_word_data(addr, reg)
    return ((raw & 0xFF) << 8) | (raw >> 8)


def read_ina260(bus, addr):
    """Return a reading dict or raise if true I²C/hardware error occurs."""
    current_raw = read_word(bus, addr, REG_CURRENT)
    voltage_raw = read_word(bus, addr, REG_VOLTAGE)
    power_raw   = read_word(bus, addr, REG_POWER)

    # Convert raw values using INA260 datasheet scaling
    if current_raw & 0x8000:  # sign extend
        current_raw -= 1 << 16

    current_mA = current_raw * 1.0
    voltage_V  = voltage_raw * 0.00125
    power_mW   = power_raw * 10

    return {
        "address": f"0x{addr:02X}",
        "label": DEVICE_CONFIG[addr]["label"],
        "voltage": round(voltage_V, 3),
        "current": round(current_mA, 2),
        "power": round(power_mW / 1000, 3),  # in Watts
    }


# ----------------------
# Main routine
# ----------------------
def run():
    """Poll INA260 rails and emit events."""
    sensors = []
    exceptions = []

    try:
        bus = SMBus(1)
        for addr in DEVICE_CONFIG:
            try:
                result = read_ina260(bus, addr)
                sensors.append(result)

                # Voltage tolerance check
                ideal = DEVICE_CONFIG[addr]["ideal_voltage"]
                max_dev = DEVICE_CONFIG[addr]["max_deviation_pct"]
                v_min = ideal * (1 - max_dev / 100)
                v_max = ideal * (1 + max_dev / 100)
                actual = result["voltage"]

                if not (v_min <= actual <= v_max):
                    exceptions.append({
                        "address": result["address"],
                        "label": result["label"],
                        "ideal_voltage": ideal,
                        "actual_voltage": actual
                    })

            except Exception as e:
                # This is a *real* I²C error: device missing, bus fault, etc.
                logging.error("I2C read failure at 0x%02X: %s", addr, e)
                continue

    finally:
        try:
            bus.close()
        except Exception:
            pass

    # Emit raw sensor snapshot
    if sensors:
        create_event(event_type="POWER_STATUS", payload={"sensors": sensors})

    # Emit exceptions (logic-level: rails out of expected range)
    for ex in exceptions:
        create_event(
            event_type="POWER_EXCEPTION",
            payload={
                "address": ex["address"],
                "label": ex["label"],
                "ideal_voltage": ex["ideal_voltage"],
                "actual_voltage": ex["actual_voltage"],
                "alert": True,
            },
        )


def bootstrap():
    """Setup logging and run once (for debugging)."""
    setup_logging()
    run()
