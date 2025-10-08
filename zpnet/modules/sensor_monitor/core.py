"""
ZPNet Sensor Monitor

Scan critical devices and emit SENSOR_SCAN events.
"""

import logging
import json
from smbus2 import SMBus
from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event

# I2C addresses for INA260 sensors
INA260_ADDRS = {
    0x40: "INA260 0x40 (Battery)",
    0x41: "INA260 0x41 (3v Rail)",
    0x44: "INA260 0x44 (5v Rail)",
}

def check_raspberry_pi():
    """Always nominal if code is running."""
    return "NOMINAL"

def check_ina260_devices():
    """
    Attempt to read voltage register on each INA260.
    If success, mark NOMINAL. If IOError, mark OFFLINE.
    """
    results = {}
    try:
        bus = SMBus(1)
        for addr, label in INA260_ADDRS.items():
            try:
                # Read voltage register (0x02)
                raw = bus.read_word_data(addr, 0x02)
                results[label] = "NOMINAL"
            except Exception as e:
                logging.warning(f"{label} offline: {e}")
                results[label] = "OFFLINE"
    except Exception as e:
        logging.error(f"I2C bus unavailable: {e}")
        for addr, label in INA260_ADDRS.items():
            results[label] = "OFFLINE"
    finally:
        try:
            bus.close()
        except Exception:
            pass
    return results

def run():
    """Perform sensor scan and emit SENSOR_SCAN event."""
    payload = {
        "Raspberry Pi": check_raspberry_pi(),
    }
    payload.update(check_ina260_devices())

    create_event(event_type="SENSOR_SCAN", payload=payload)

def bootstrap():
    setup_logging()
    run()
