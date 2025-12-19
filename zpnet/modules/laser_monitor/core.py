"""
ZPNet Laser Monitor — EV5491 Ground-Truth Observer

Responsibilities:
  • Observe EV5491 laser controller register state over I²C
  • Emit only ground-truth hardware facts
  • Perform NO control operations
  • Perform NO inference about operator intent

Important:
  • Commanded laser state (LASER_STATE) is emitted by the Teensy
  • Observed driver state (LASER_STATUS) is emitted here
  • Aggregation merges the two truths explicitly

Author: The Mule + GPT
"""

import logging
from smbus2 import SMBus

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event

# ---------------------------------------------------------------------
# EV5491 Registers (observed subset only)
# ---------------------------------------------------------------------
I2C_ADDR = 0x66

REG_CTL0 = 0x00
REG_CTL1 = 0x01
REG_ID1_LSB = 0x07
REG_ID1_MSB = 0x08
REG_ID1_MODE = 0x0A

SYSEN_BIT = 0x80
ID1_EN_BIT = 0x80

MODE_MAP = {
    0x00: "OFF",
    0x01: "CC",
    0x02: "PWM",
}

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Read EV5491 registers and emit LASER_STATUS event.

    Semantics:
      • Observation only
      • No writes
      • No assumptions about commanded state
      • No health inference beyond presence/readability

    Emits:
        LASER_STATUS
    """
    payload = {
        "i2c_address": "0x66",
        "device_present": False,
    }

    bus = None
    try:
        bus = SMBus(1)

        ctl0 = bus.read_byte_data(I2C_ADDR, REG_CTL0)
        ctl1 = bus.read_byte_data(I2C_ADDR, REG_CTL1)
        id1_lsb = bus.read_byte_data(I2C_ADDR, REG_ID1_LSB)
        id1_msb = bus.read_byte_data(I2C_ADDR, REG_ID1_MSB)
        id1_mode_raw = bus.read_byte_data(I2C_ADDR, REG_ID1_MODE)

        id1_current_code = ((id1_msb & 0x03) << 8) | id1_lsb

        payload.update({
            "device_present": True,
            "sys_enabled": bool(ctl0 & SYSEN_BIT),
            "id1_enabled": bool(ctl1 & ID1_EN_BIT),
            "id1_mode": MODE_MAP.get(id1_mode_raw & 0x03, "UNKNOWN"),
            "id1_current_code": id1_current_code,
            "raw_registers": {
                "0x00": f"0x{ctl0:02X}",
                "0x01": f"0x{ctl1:02X}",
                "0x07": f"0x{id1_lsb:02X}",
                "0x08": f"0x{id1_msb:02X}",
                "0x0A": f"0x{id1_mode_raw:02X}",
            },
            "health_state": "NOMINAL",
        })

    except Exception as e:
        logging.warning(f"[laser_monitor] EV5491 read failed: {e}")
        payload["health_state"] = "DOWN"

    finally:
        try:
            bus.close()
        except Exception:
            pass

        create_event("LASER_STATUS", payload)


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and execute run() once."""
    setup_logging()
    run()
