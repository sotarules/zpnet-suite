"""
ZPNet Startup Ritual

One-shot initialization tasks executed exactly once at system boot,
before the scheduler begins.

Author: The Mule
"""

import logging

# ---------------------------------------------------------------------
# Startup tasks (ordered)
# ---------------------------------------------------------------------
def startup_tasks():
    """
    Return ordered list of startup callables.
    Add new tasks here.
    """
    return [
        # init_laser_driver,
        # future:
        # init_photodiode,
        # verify_gnss_lock,
        # sanity_check_power_rails,
    ]


# ---------------------------------------------------------------------
# Individual startup tasks
# ---------------------------------------------------------------------
def init_laser_driver():
    """
    Initialize EV5491 laser driver into a safe, armed configuration.

    This function:
      - Enables SYSEN
      - Enables ID1 channel
      - Sets ID1 to constant-current (CC) mode
      - Programs a safe default current
      - Does NOT enable laser emission (LD_ON remains controlled by Teensy)

    Runs exactly once at system startup.

    Emits:
        LASER_ARMED (on success)

    Raises:
        Exception on any unrecoverable I²C or configuration failure.
    """
    import logging
    from smbus2 import SMBus

    from zpnet.shared.events import create_event

    # ------------------------------------------------------------------
    # EV5491 constants
    # ------------------------------------------------------------------
    I2C_ADDR = 0x66

    REG_CTL0 = 0x00
    REG_CTL1 = 0x01
    REG_ID1_MODE = 0x0A
    REG_ID1_LSB = 0x07
    REG_ID1_MSB = 0x08

    SYSEN_BIT = 0x80
    ID1_EN_BIT = 0x80

    # Safe default current (same as existing test code)
    ID1_CURRENT_CODE = 0x0060

    logging.info("🔧 [startup] EV5491: beginning laser driver initialization")

    bus = None
    try:
        bus = SMBus(1)

        # --- Enable system (SYSEN) ---
        ctl0 = bus.read_byte_data(I2C_ADDR, REG_CTL0)
        bus.write_byte_data(I2C_ADDR, REG_CTL0, ctl0 | SYSEN_BIT)

        # --- Enable ID1 channel ---
        ctl1 = bus.read_byte_data(I2C_ADDR, REG_CTL1)
        bus.write_byte_data(I2C_ADDR, REG_CTL1, ctl1 | ID1_EN_BIT)

        # --- Set ID1 to constant-current (CC) mode ---
        # Datasheet-defined value: 0x01
        bus.write_byte_data(I2C_ADDR, REG_ID1_MODE, 0x01)

        # --- Program safe current ---
        bus.write_byte_data(I2C_ADDR, REG_ID1_LSB, ID1_CURRENT_CODE & 0xFF)
        bus.write_byte_data(I2C_ADDR, REG_ID1_MSB, (ID1_CURRENT_CODE >> 8) & 0x03)

        logging.info("✅ [startup] EV5491 armed (SYSEN + ID1 + CC mode)")

        # Emit explicit, auditable event
        create_event(
            "LASER_ARMED",
            {
                "i2c_address": "0x66",
                "mode": "CC",
                "current_code": ID1_CURRENT_CODE,
                "sys_enabled": True,
                "id1_enabled": True,
                "source": "startup",
            },
        )

    except Exception as e:
        logging.exception("💥 [startup] EV5491 initialization failed")
        raise

    finally:
        try:
            bus.close()
        except Exception:
            pass

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------
def run_startup():
    """
    Execute startup tasks sequentially.
    """
    logging.info("🚀 [startup] beginning startup ritual")

    for task in startup_tasks():
        name = task.__name__
        logging.info(f"▶️ [startup] {name}")
        task()

    logging.info("🌱 [startup] startup ritual complete")
