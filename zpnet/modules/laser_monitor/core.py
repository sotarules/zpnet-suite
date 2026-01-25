"""
ZPNet Laser Monitor — MP5491 + Optical Ground Truth Observer

Responsibilities:
  • Observe MP5491 register state over I²C (read-only)
  • Decode enable, current, and protection state correctly
  • Query Teensy LASER process for optical ground truth
  • Emit authoritative LASER_STATUS event
  • Perform NO control operations
  • Perform NO inference about operator intent

Semantics:
  • "configured"  = what the driver is set up to do (I²C truth)
  • "emitting"    = what photons are actually doing (Teensy truth)
  • Both truths are emitted explicitly and separately

Author: The Mule + GPT
"""

import logging
from smbus2 import SMBus

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.teensy import send_command

# ---------------------------------------------------------------------
# Hardware constants
# ---------------------------------------------------------------------
I2C_ADDR = 0x66

# ---------------------------------------------------------------------
# MP5491 Registers
# ---------------------------------------------------------------------
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

# Bit masks
SYSEN_BIT   = 0x80           # CTL0.D7
ID_EN_BIT   = 0x80           # CTL1.D7
ID1_EN_BIT  = 0x08           # CTL1.D3

ID1_FLG_BIT = 0x02           # STATUS1
VIN2_UV_BIT = 0x02           # STATUS3
LD_ON_BIT   = 0x08           # STATUS3

ID_SCP_BIT  = 0x10           # INT
OT_WARN_BIT = 0x02           # INT
OT_SHDN_BIT = 0x01           # INT

# ---------------------------------------------------------------------
# Main monitor routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Read MP5491 registers and query Teensy laser process,
    then emit LASER_STATUS event.
    """
    payload = {
        "i2c_address": "0x66",
        "device_present": False,
        "health_state": "UNKNOWN",
    }

    bus = None
    try:
        # -------------------------------------------------------------
        # I²C: Read MP5491 configuration & status (capability truth)
        # -------------------------------------------------------------
        bus = SMBus(I2C_BUS)

        vals = {
            name: bus.read_byte_data(I2C_ADDR, addr)
            for name, addr in REGS.items()
        }

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

        # -------------------------------------------------------------
        # Teensy: Query LASER process for optical ground truth
        # -------------------------------------------------------------
        try:
            resp = send_command(
                "PROCESS.COMMAND",
                {
                    "type": "LASER",
                    "proc_cmd": "REPORT",
                }
            )
        except Exception:
            resp = {}

        pd_voltage = None
        laser_emitting = None

        if resp and resp.get("success"):
            tp = resp.get("payload", {})
            pd_voltage = tp.get("PD_voltage")
            laser_emitting = tp.get("laser_emitting")

        # -------------------------------------------------------------
        # Populate payload
        # -------------------------------------------------------------
        payload.update({
            "device_present": True,

            # Configuration / capability (I²C truth)
            "sys_enabled": sysen,
            "id_enabled": id_en,
            "id1_enabled": id1_en,
            "id1_current_ma": round(id1_current_ma, 2),

            # Driver state
            "id1_active": id1_active,
            "vin2_ok": vin2_ok,
            "ld_on_seen": ld_on_seen,

            # Optical ground truth (Teensy instrument)
            "pd_voltage": pd_voltage,
            "laser_emitting": laser_emitting,

            # Protection
            "scp_tripped": id_scp,
            "ot_warn": ot_warn,
            "ot_shutdown": ot_shdn,

            # Raw registers (for forensics)
            "raw_registers": {
                f"0x{addr:02X}": f"0x{vals[name]:02X}"
                for name, addr in REGS.items()
            },

            "health_state": "NOMINAL"
        })

    except Exception as e:
        logging.warning(f"[laser_monitor] MP5491 read failed: {e}")
        payload["health_state"] = "DOWN"

    finally:
        try:
            if bus:
                bus.close()
        except Exception:
            pass

        create_event("LASER_STATUS", payload)


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    setup_logging()
    run()
