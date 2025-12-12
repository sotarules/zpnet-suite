"""
ZPNet Environment Monitor — BME280 Raw I²C Implementation (v2025-12-11a)

Reads environmental data from a Bosch BME280 sensor over I²C (address 0x76)
and emits ENVIRONMENT_STATUS events containing raw, ground-truth measurements.

Measured quantities:
    • Temperature (°C)
    • Pressure (hPa)
    • Humidity (% RH)
    • Derived altitude (m) using standard atmosphere model

No filtering, smoothing, or inference is performed.

Author: The Mule + GPT
"""

import logging
import math
from smbus2 import SMBus

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event

# ---------------------------------------------------------------------
# BME280 Configuration
# ---------------------------------------------------------------------
I2C_ADDR = 0x76

REG_ID = 0xD0
REG_RESET = 0xE0
REG_CTRL_HUM = 0xF2
REG_STATUS = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_DATA = 0xF7  # pressure[3], temp[3], humidity[2]

EXPECTED_CHIP_ID = 0x60

SEA_LEVEL_PRESSURE_HPA = 1013.25


# ---------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------
def read_u16_le(bus: SMBus, addr: int, reg: int) -> int:
    lsb = bus.read_byte_data(addr, reg)
    msb = bus.read_byte_data(addr, reg + 1)
    return (msb << 8) | lsb


def read_s16_le(bus: SMBus, addr: int, reg: int) -> int:
    val = read_u16_le(bus, addr, reg)
    if val & 0x8000:
        val -= 1 << 16
    return val


# ---------------------------------------------------------------------
# Calibration loading
# ---------------------------------------------------------------------
def read_calibration(bus: SMBus) -> dict:
    cal = {}

    # Temperature / Pressure calibration
    cal["dig_T1"] = read_u16_le(bus, I2C_ADDR, 0x88)
    cal["dig_T2"] = read_s16_le(bus, I2C_ADDR, 0x8A)
    cal["dig_T3"] = read_s16_le(bus, I2C_ADDR, 0x8C)

    cal["dig_P1"] = read_u16_le(bus, I2C_ADDR, 0x8E)
    cal["dig_P2"] = read_s16_le(bus, I2C_ADDR, 0x90)
    cal["dig_P3"] = read_s16_le(bus, I2C_ADDR, 0x92)
    cal["dig_P4"] = read_s16_le(bus, I2C_ADDR, 0x94)
    cal["dig_P5"] = read_s16_le(bus, I2C_ADDR, 0x96)
    cal["dig_P6"] = read_s16_le(bus, I2C_ADDR, 0x98)
    cal["dig_P7"] = read_s16_le(bus, I2C_ADDR, 0x9A)
    cal["dig_P8"] = read_s16_le(bus, I2C_ADDR, 0x9C)
    cal["dig_P9"] = read_s16_le(bus, I2C_ADDR, 0x9E)

    # Humidity calibration
    cal["dig_H1"] = bus.read_byte_data(I2C_ADDR, 0xA1)
    cal["dig_H2"] = read_s16_le(bus, I2C_ADDR, 0xE1)
    cal["dig_H3"] = bus.read_byte_data(I2C_ADDR, 0xE3)
    e4 = bus.read_byte_data(I2C_ADDR, 0xE4)
    e5 = bus.read_byte_data(I2C_ADDR, 0xE5)
    e6 = bus.read_byte_data(I2C_ADDR, 0xE6)
    cal["dig_H4"] = (e4 << 4) | (e5 & 0x0F)
    cal["dig_H5"] = (e6 << 4) | (e5 >> 4)
    cal["dig_H6"] = bus.read_byte_data(I2C_ADDR, 0xE7)
    if cal["dig_H6"] & 0x80:
        cal["dig_H6"] -= 256

    return cal


# ---------------------------------------------------------------------
# Compensation formulas (Bosch datasheet)
# ---------------------------------------------------------------------
def compensate_temperature(adc_T: int, cal: dict) -> tuple[float, float]:
    var1 = (adc_T / 16384.0 - cal["dig_T1"] / 1024.0) * cal["dig_T2"]
    var2 = ((adc_T / 131072.0 - cal["dig_T1"] / 8192.0) ** 2) * cal["dig_T3"]
    t_fine = var1 + var2
    temp_c = t_fine / 5120.0
    return temp_c, t_fine


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


# ---------------------------------------------------------------------
# Main routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Read BME280 sensor and emit ENVIRONMENT_STATUS event.

    Emits:
        ENVIRONMENT_STATUS
    """
    payload = {
        "sensor_address": "0x76",
        "sensor_present": False,
    }

    bus = None
    try:
        bus = SMBus(1)

        chip_id = bus.read_byte_data(I2C_ADDR, REG_ID)
        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(f"Unexpected BME280 chip ID: 0x{chip_id:02X}")

        payload["sensor_present"] = True

        # Configure oversampling (x1) and forced mode
        bus.write_byte_data(I2C_ADDR, REG_CTRL_HUM, 0x01)
        bus.write_byte_data(I2C_ADDR, REG_CTRL_MEAS, 0x27)

        cal = read_calibration(bus)

        data = bus.read_i2c_block_data(I2C_ADDR, REG_DATA, 8)
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

    except Exception as e:
        logging.warning(f"[environment_monitor] BME280 read failed: {e}")
        payload["health_state"] = "DOWN"

    finally:
        try:
            bus.close()
        except Exception:
            pass

        create_event("ENVIRONMENT_STATUS", payload)


def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
