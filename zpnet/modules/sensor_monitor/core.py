"""
ZPNet Sensor Monitor.

Scans I²C devices (INA260 sensors) for presence and responsiveness.
Emits SENSOR_SCAN events summarizing which devices are nominal or offline.

Author: The Mule
"""

import logging
from smbus2 import SMBus
from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event

# ---------------------------------------------------------------------
# INA260 I²C addresses
# ---------------------------------------------------------------------
INA260_ADDRS = {
    0x40: "INA260 0x40 (Battery)",
    0x41: "INA260 0x41 (3v3 Rail)",
    0x44: "INA260 0x44 (5v0 Rail)",
    0x45: "INA260 0x45 (24v Spur)",
}

# ---------------------------------------------------------------------
# Individual device checks
# ---------------------------------------------------------------------
def check_ina260_devices() -> dict:
    """
    Attempt to read voltage register on each INA260.
    If success → NOMINAL, else → OFFLINE.

    Returns:
        dict[str, str]: device label → status string
    """
    results = {}
    bus = None
    try:
        bus = SMBus(1)
        for addr, label in INA260_ADDRS.items():
            try:
                # Read voltage register (0x02)
                _ = bus.read_word_data(addr, 0x02)
                results[label] = "NOMINAL"
            except Exception as e:
                logging.warning(f"{label} offline: {e}")
                results[label] = "OFFLINE"
    except Exception as e:
        logging.error(f"[sensor_monitor] I²C bus unavailable: {e}")
        for label in INA260_ADDRS.values():
            results[label] = "OFFLINE"
    finally:
        try:
            bus.close()
        except Exception:
            pass
    return results

# ---------------------------------------------------------------------
# Main routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Perform sensor scan and emit SENSOR_SCAN event.

    Emits:
        SENSOR_SCAN: contains status of all INA260 sensors.
    """
    payload = check_ina260_devices()
    create_event("SENSOR_SCAN", payload)

def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
