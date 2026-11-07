"""AD5693R transfers for the independent CLOCKS and PHOTONS DAC owners.

Linux serializes each adapter transfer. Each call here is a complete register
operation; it does not reserve the adapter across sensor conversions or sleeps.
Only repeatable register reads/writes belong here, never software-reset commands.
Opening a fresh descriptor on retry does not reset or electrically recover a bus.
"""

import errno
import logging
import time

from smbus2 import SMBus, i2c_msg


# Short bounded recovery: at most 4 ms of explicit delay, plus ioctl/open time.
# CLOCKS dither must not inherit the much longer environmental-sensor retries.
_ATTEMPT_DELAYS = (0.0, 0.001, 0.003)
_RETRY_ERRNOS = {errno.EIO, errno.EREMOTEIO, errno.ENXIO,
                 errno.EAGAIN, errno.ETIMEDOUT}


def _transfer(bus_id, address, operation, make_message):
    for attempt, delay in enumerate(_ATTEMPT_DELAYS, start=1):
        if delay:
            time.sleep(delay)
        try:
            with SMBus(bus_id) as bus:
                message = make_message()
                bus.i2c_rdwr(message)
        except OSError as exc:
            if exc.errno not in _RETRY_ERRNOS:
                raise
            if attempt == len(_ATTEMPT_DELAYS):
                raise OSError(
                    exc.errno,
                    f"AD5693R i2c-{bus_id}/0x{address:02X} {operation} "
                    f"failed after {attempt} attempts: {exc}",
                ) from exc
        else:
            if attempt > 1:
                logging.warning(
                    "[ad5693r] i2c-%d/0x%02X %s recovered on attempt %d/%d",
                    bus_id, address, operation, attempt, len(_ATTEMPT_DELAYS),
                )
            return message


def write_register(bus_id, address, command, value):
    """Send the complete command + two data bytes on every attempt."""
    data = [command, value >> 8, value & 0xFF]
    _transfer(bus_id, address, f"write 0x{command:02X}",
              lambda: i2c_msg.write(address, data))


def read_input(bus_id, address):
    """Read only the input register; this is not analog-output verification."""
    message = _transfer(bus_id, address, "read input",
                        lambda: i2c_msg.read(address, 2))
    high, low = list(message)
    return (high << 8) | low
