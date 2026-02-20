"""
zpnet.native.pps_raw — Raw PPS capture via /dev/pps_raw kernel module

Provides blocking reads of raw ARM arch timer (CNTVCT_EL0) values
captured at PPS GPIO interrupt edges by the pps_raw kernel module.

Unlike the standard /dev/pps0 interface, these values are NOT
processed through the kernel timekeeping layer and are therefore
free of chrony/NTP rate adjustments.

The captures are raw 54 MHz tick counts — the same counter that
systimer.read() returns, but captured in kernel interrupt context
with ~1µs latency from the physical edge (vs. tens-to-hundreds
of microseconds for userspace reads after ioctl return).

Usage:
    from zpnet.native.pps_raw import PpsRawReader

    reader = PpsRawReader()         # opens /dev/pps_raw
    counter, seq, missed = reader.read()   # blocks until next PPS
    reader.close()

    # Or as context manager:
    with PpsRawReader() as pps:
        counter, seq, missed = pps.read()

Struct layout (16 bytes, native byte order):
    uint64_t counter;    // raw CNTVCT_EL0 ticks
    uint32_t sequence;   // monotonic sequence number (starts at 1)
    uint32_t missed;     // captures missed since last read by this fd
"""

import os
import struct
from typing import NamedTuple

DEVICE = "/dev/pps_raw"
CAPTURE_FMT = "=QII"          # uint64 + uint32 + uint32
CAPTURE_SIZE = struct.calcsize(CAPTURE_FMT)  # 16 bytes


class PpsRawCapture(NamedTuple):
    """A single PPS capture from the kernel module."""
    counter: int     # raw CNTVCT_EL0 ticks at PPS edge
    sequence: int    # monotonic sequence number
    missed: int      # captures this reader missed (overruns)


class PpsRawReader:
    """
    Blocking reader for /dev/pps_raw.

    Each call to read() blocks until the next PPS edge and returns
    the raw CNTVCT_EL0 counter value captured in interrupt context.

    Multiple readers are supported — each gets every capture
    independently (the kernel module tracks per-fd state).
    """

    def __init__(self, device: str = DEVICE) -> None:
        self._fd = os.open(device, os.O_RDONLY)

    def read(self) -> PpsRawCapture:
        """
        Block until next PPS edge, return raw capture.

        Returns PpsRawCapture(counter, sequence, missed).
        Raises OSError on device error.
        """
        data = os.read(self._fd, CAPTURE_SIZE)
        if len(data) < CAPTURE_SIZE:
            raise OSError(f"short read from {DEVICE}: {len(data)}/{CAPTURE_SIZE}")
        counter, sequence, missed = struct.unpack(CAPTURE_FMT, data)
        return PpsRawCapture(counter, sequence, missed)

    def fileno(self) -> int:
        """Return fd for use with select/poll."""
        return self._fd

    def close(self) -> None:
        """Close the device."""
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        self.close()