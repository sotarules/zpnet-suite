"""
zpnet.native.systimer — ARM Generic Timer access from Python

Provides direct access to the ARM architected system timer
(CNTVCT_EL0) via a tiny shared library.  No syscall overhead,
no vDSO interpolation, no NTP steering — just the raw hardware
counter in a single MRS instruction.

On Raspberry Pi 4/5 the counter runs at 54 MHz (18.519 ns/tick).
The actual frequency is read from hardware at import time.

Usage:
    from zpnet.native import systimer

    ticks = systimer.read()           # raw 64-bit counter
    freq  = systimer.frequency()      # ticks per second (cached)
    ns    = systimer.ticks_to_ns(t)   # convert ticks to nanoseconds

For PPS capture, use read() directly and store the raw tick
value.  Convert to nanoseconds only when needed for display
or statistics — this avoids rounding at the capture point.

Build the shared library first:
    cd zpnet/native && make
"""

from __future__ import annotations

import ctypes
import os
import logging

# ---------------------------------------------------------------------
# Load shared library
# ---------------------------------------------------------------------

_LIB_NAME = "libsystimer.so"
_LIB_DIR = os.path.dirname(os.path.abspath(__file__))
_LIB_PATH = os.path.join(_LIB_DIR, _LIB_NAME)

try:
    _lib = ctypes.CDLL(_LIB_PATH)

    # Configure function signatures
    _lib.systimer_read.argtypes = []
    _lib.systimer_read.restype = ctypes.c_uint64

    _lib.systimer_frequency.argtypes = []
    _lib.systimer_frequency.restype = ctypes.c_uint64

    # Read frequency once at import time (set by firmware, never changes)
    _FREQUENCY: int = _lib.systimer_frequency()

    _available = True

    logging.getLogger(__name__).debug(
        "systimer loaded: freq=%d Hz (%.3f ns/tick)",
        _FREQUENCY,
        1e9 / _FREQUENCY if _FREQUENCY else 0,
    )

except OSError:
    _lib = None
    _FREQUENCY = 0
    _available = False

    logging.getLogger(__name__).warning(
        "systimer: %s not found — falling back to time.monotonic_ns(). "
        "Build with: cd zpnet/native && make",
        _LIB_PATH,
    )

# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------

def available() -> bool:
    """True if the native systimer library is loaded."""
    return _available


def read() -> int:
    """
    Read the ARM system timer (CNTVCT_EL0).

    Returns a raw 64-bit tick count.  At 54 MHz the resolution
    is ~18.5 ns per tick.

    If the native library is not available, falls back to
    time.monotonic_ns() (which reads the same counter but
    through the kernel vDSO with more overhead and jitter).
    """
    if _lib is not None:
        return _lib.systimer_read()

    import time
    return time.monotonic_ns()


def frequency() -> int:
    """
    Counter frequency in Hz (e.g. 54000000 on Pi 4/5).

    Read from CNTFRQ_EL0 once at import time and cached.
    Returns 1000000000 (1 GHz) when falling back to
    monotonic_ns.
    """
    if _available:
        return _FREQUENCY
    return 1_000_000_000  # monotonic_ns is nominally 1 GHz


def ticks_to_ns(ticks: int) -> int:
    """
    Convert a tick count (or tick delta) to nanoseconds.

    Uses integer arithmetic to avoid floating-point rounding:
        ns = ticks * 1_000_000_000 // frequency

    For PPS residual tracking, prefer storing raw ticks and
    converting only at display time.
    """
    freq = frequency()
    if freq == 0:
        return 0
    return (ticks * 1_000_000_000) // freq


def ns_per_tick() -> float:
    """Nanoseconds per tick (e.g. ~18.519 at 54 MHz)."""
    freq = frequency()
    if freq == 0:
        return 0.0
    return 1e9 / freq


def expected_ticks_per_pps() -> int:
    """
    Expected tick count between consecutive PPS pulses.

    This is simply the frequency — one second of ticks.
    Used as the 'expected' value in residual calculations.
    """
    return frequency()