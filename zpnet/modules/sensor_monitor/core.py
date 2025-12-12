"""
ZPNet Sensor Monitor — Stellar-Compliant + Constants-Integrated Revision (v2025-12-12a)

Scans I²C devices for presence and responsiveness.
Includes INA260 power sensors and the BME280 environmental sensor.

Emits SENSOR_SCAN events summarizing which devices are nominal or offline.

Author: The Mule
"""

import logging

from smbus2 import SMBus

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# I²C Device Map
# ---------------------------------------------------------------------
I2C_DEVICES = {
    0x40: "INA260 0x40 (Battery)",
    0x41: "INA260 0x41 (3v3 Rail)",
    0x44: "INA260 0x44 (5v0 Rail)",
    0x45: "INA260 0x45 (24v Spur)",
    0x76: "BME280 0x76 (Environment)",
}

# BME280 specifics
BME280_ID_REG = 0xD0
BME280_EXPECTED_ID = 0x60

# ---------------------------------------------------------------------
# Individual device checks
# ---------------------------------------------------------------------
def check_i2c_devices() -> dict:
    """
    Attempt to verify presence of known I²C devices.

    Returns:
        dict[str, str]: device label → status string (NOMINAL | OFFLINE)
    """
    results = {}
    bus = None

    try:
        bus = SMBus(1)

        for addr, label in I2C_DEVICES.items():
            try:
                if "BME280" in label:
                    chip_id = bus.read_byte_data(addr, BME280_ID_REG)
                    if chip_id != BME280_EXPECTED_ID:
                        raise RuntimeError(
                            f"unexpected BME280 chip ID 0x{chip_id:02X}"
                        )
                else:
                    # Generic probe for INA260 (voltage register)
                    _ = bus.read_word_data(addr, 0x02)

                results[label] = "NOMINAL"

            except Exception as e:
                logging.warning(f"{label} offline: {e}")
                results[label] = "OFFLINE"

    except Exception as e:
        logging.error(f"[sensor_monitor] I²C bus unavailable: {e}")
        for label in I2C_DEVICES.values():
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
        SENSOR_SCAN: contains status of all known I²C sensors.
    """
    payload = check_i2c_devices()
    create_event("SENSOR_SCAN", payload)


def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
