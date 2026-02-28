"""timebase_watch — Exercise Timebase interpolation against chrony-disciplined system clock.

Tails the PUBSUB log file for TIMEBASE records, feeds them into the
Timebase singleton, then at jittery ~1s intervals reads:
  1. Pi cycle counter (CNTVCT_EL0 via libppscapture.so)
  2. timebase.now("GNSS", pi_cycles)  — interpolated GNSS time
  3. clock_gettime(CLOCK_REALTIME)     — chrony-disciplined system time

Displays them side-by-side with the delta.  The delta is the interesting
signal: it shows how well the TIMEBASE interpolation agrees with chrony's
independent GNSS-disciplined clock.

Usage:
    python -m zpnet.tests.timebase_watch [duration_s] [base_interval_s]

    duration_s       — how long to run (default: 120)
    base_interval_s  — base sleep between samples (default: 1.0)
                       actual sleep is jittered ±30% to avoid phase-lock
"""

from __future__ import annotations

import ctypes
import json
import os
import random
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone

from zpnet.shared.logger import setup_logging
from zpnet.shared.timebase import timebase

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

DEFAULT_DURATION_S = 120.0
DEFAULT_INTERVAL_S = 1.0
JITTER_FRACTION = 0.30  # ±30% of base interval

PUBSUB_LOG = "/home/mule/zpnet/logs/zpnet-pubsub.log"

# ---------------------------------------------------------------------
# Pi cycle counter access (CNTVCT_EL0 — raw ARM generic timer)
# ---------------------------------------------------------------------
#
# libppscapture.so exports ppscapture_next() which BLOCKS until the
# next second boundary — that's not what we want.  We need an
# instantaneous read of CNTVCT_EL0.
#
# Strategy: build a tiny shared object at runtime that exports
# read_cntvct() as a single MRS instruction.  Cached in /tmp.
# ---------------------------------------------------------------------

import subprocess
import tempfile

_lib = None
_CNTVCT_SO = "/tmp/zpnet_cntvct.so"

_CNTVCT_C = r"""
#include <stdint.h>
uint64_t read_cntvct(void) {
    uint64_t val;
    __asm__ __volatile__("isb\n\tmrs %0, cntvct_el0" : "=r"(val) :: "memory");
    return val;
}
"""


def _build_cntvct_lib() -> str:
    """Compile a minimal .so for CNTVCT_EL0 reads.  Cached in /tmp."""
    if os.path.exists(_CNTVCT_SO):
        return _CNTVCT_SO

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".c", dir="/tmp", delete=False
    ) as f:
        f.write(_CNTVCT_C)
        c_path = f.name

    try:
        subprocess.run(
            ["gcc", "-shared", "-fPIC", "-O2", "-o", _CNTVCT_SO, c_path],
            check=True, capture_output=True,
        )
    finally:
        os.unlink(c_path)

    return _CNTVCT_SO


def _load_cntvct_lib():
    global _lib
    if _lib is not None:
        return
    so_path = _build_cntvct_lib()
    _lib = ctypes.CDLL(so_path)
    _lib.read_cntvct.restype = ctypes.c_uint64
    _lib.read_cntvct.argtypes = []


def read_pi_cycles() -> int:
    """Read the Pi's CNTVCT_EL0 counter — instantaneous, non-blocking."""
    _load_cntvct_lib()
    return int(_lib.read_cntvct())


# ---------------------------------------------------------------------
# System clock (CLOCK_REALTIME) in nanoseconds
# ---------------------------------------------------------------------

# struct timespec { time_t tv_sec; long tv_nsec; }
_CLOCK_REALTIME = 0
_libc = ctypes.CDLL("libc.so.6", use_errno=True)


class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


def system_clock_ns() -> int:
    """Read CLOCK_REALTIME as nanoseconds since Unix epoch."""
    ts = _Timespec()
    rc = _libc.clock_gettime(_CLOCK_REALTIME, ctypes.byref(ts))
    if rc != 0:
        raise OSError("clock_gettime failed")
    return ts.tv_sec * 1_000_000_000 + ts.tv_nsec


# ---------------------------------------------------------------------
# TIMEBASE ingestion from log tail
# ---------------------------------------------------------------------

_records_lock = threading.Lock()
_first_system_ns: int | None = None
_first_gnss_ns: int | None = None

# GNSS absolute epoch anchor: maps campaign-relative gnss_ns to UTC
_gnss_epoch_offset_ns: int | None = None  # add to campaign gnss_ns to get Unix epoch ns


def _parse_gnss_utc_to_epoch_ns(gnss_time_utc: str) -> int | None:
    """Convert GNSS UTC string like '2026-02-28T19:22:43Z' to Unix epoch nanoseconds."""
    try:
        # Parse ISO8601 — these are always whole seconds at PPS edges
        dt = datetime.fromisoformat(gnss_time_utc.replace("Z", "+00:00"))
        return int(dt.timestamp()) * 1_000_000_000
    except (ValueError, AttributeError):
        return None


def _ingest_line(line: str) -> None:
    """Parse a PUBSUB log line and feed TIMEBASE records to the singleton."""
    global _first_system_ns, _first_gnss_ns, _gnss_epoch_offset_ns

    line = line.strip()
    if not line.startswith("TIMEBASE "):
        return

    try:
        blob = json.loads(line[9:])  # skip "TIMEBASE "
        payload = blob.get("payload", blob)
    except (json.JSONDecodeError, ValueError):
        return

    timebase.set(payload)

    with _records_lock:
        # Epoch alignment for delta computation
        if _first_gnss_ns is None:
            gnss_ns = payload.get("teensy_gnss_ns")
            if gnss_ns is not None:
                _first_gnss_ns = int(gnss_ns)
                _first_system_ns = system_clock_ns()

        # GNSS absolute epoch anchor (refresh every record)
        #
        # At each PPS edge:
        #   gnss_time_utc = absolute UTC (whole second, authoritative)
        #   teensy_gnss_ns = campaign-relative GNSS ns (has 100ns quantization)
        #
        # The offset maps teensy_gnss_ns → absolute epoch ns.
        # Sub-second residual in teensy_gnss_ns (e.g., 999999900) is GNSS
        # quantization noise — the offset absorbs it correctly because
        # gnss_time_utc IS the true time at this edge.
        gnss_time_str = payload.get("gnss_time_utc")
        gnss_ns_val = payload.get("teensy_gnss_ns")
        if gnss_time_str and gnss_ns_val is not None:
            abs_ns = _parse_gnss_utc_to_epoch_ns(gnss_time_str)
            if abs_ns is not None:
                _gnss_epoch_offset_ns = abs_ns - int(gnss_ns_val)


def _tail_log(path: str, stop_event: threading.Event) -> None:
    """
    Tail the PUBSUB log file for TIMEBASE records.

    Seeks to end of file on start (we only want live records),
    then polls for new lines.  Stops when stop_event is set.
    """
    while not stop_event.is_set():
        try:
            with open(path, "r") as f:
                f.seek(0, os.SEEK_END)
                while not stop_event.is_set():
                    line = f.readline()
                    if line:
                        _ingest_line(line)
                    else:
                        stop_event.wait(timeout=0.050)
        except FileNotFoundError:
            print(f"  Waiting for log file: {path}")
            stop_event.wait(timeout=2.0)
        except Exception as e:
            print(f"  Log tail error: {e}")
            stop_event.wait(timeout=1.0)


# ---------------------------------------------------------------------
# Welford's online statistics (for delta tracking)
# ---------------------------------------------------------------------

class _Welford:
    __slots__ = ("n", "mean", "m2")

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

    def update(self, x: float):
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2

    @property
    def stddev(self) -> float:
        return (self.m2 / (self.n - 1)) ** 0.5 if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / (self.n ** 0.5) if self.n >= 2 else 0.0


# ---------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------

def _ns_to_iso(epoch_ns: int) -> str:
    """Format Unix epoch nanoseconds as ISO8601 with nanosecond precision."""
    sec = epoch_ns // 1_000_000_000
    nsec = epoch_ns % 1_000_000_000
    dt = datetime.fromtimestamp(sec, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + f".{nsec:09d}Z"


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def main():
    setup_logging()

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DURATION_S
    base_interval = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_INTERVAL_S
    log_path = sys.argv[3] if len(sys.argv) > 3 else PUBSUB_LOG

    print(f"=== Timebase Interpolation Watch ({duration}s, ~{base_interval}s intervals ±{JITTER_FRACTION:.0%}) ===")
    print(f"    Log file: {log_path}")
    print()
    print("Comparing timebase.now('GNSS') against chrony-disciplined CLOCK_REALTIME.")
    print("Delta = (interpolated GNSS) - (system clock relative) in nanoseconds.")
    print("A stable delta means interpolation is tracking chrony accurately.")
    print()

    # Start log tail thread
    stop_event = threading.Event()
    tail_thread = threading.Thread(
        target=_tail_log,
        args=(log_path, stop_event),
        daemon=True,
        name="timebase-log-tail",
    )
    tail_thread.start()

    # Wait for foundation (need 2 TIMEBASE records)
    print("Waiting for TIMEBASE foundation (need 2 records)...")
    t0 = time.monotonic()
    while not timebase.ready:
        if time.monotonic() - t0 > 30:
            print("ERROR: No TIMEBASE received in 30s — is a campaign running?")
            sys.exit(1)
        time.sleep(0.5)

    print(f"Foundation ready at pps_count={timebase.pps_count} "
          f"({timebase.records_received} records received)")
    print()

    # Column headers
    print(f"{'elapsed':>8s}  "
          f"{'pps':>6s}  "
          f"{'gnss_interpolated':>33s}  "
          f"{'system_clock':>33s}  "
          f"{'delta_ns':>10s}  "
          f"{'mean_ns':>10s}  "
          f"{'stddev_ns':>10s}  "
          f"{'age_us':>8s}")
    print(f"{'─' * 8}  "
          f"{'─' * 6}  "
          f"{'─' * 33}  "
          f"{'─' * 33}  "
          f"{'─' * 10}  "
          f"{'─' * 10}  "
          f"{'─' * 10}  "
          f"{'─' * 8}")

    stats = _Welford()
    t_start = time.monotonic()

    while (time.monotonic() - t_start) < duration:
        # Jittered sleep
        jitter = base_interval * JITTER_FRACTION
        sleep_time = base_interval + random.uniform(-jitter, jitter)
        time.sleep(sleep_time)

        # Read cycle counter and system clock as close together as possible
        pi_cycles = read_pi_cycles()
        sys_ns = system_clock_ns()

        # Interpolate GNSS time
        gnss_now = timebase.now("GNSS", pi_cycles)
        if gnss_now is None:
            print("  (foundation not ready)")
            continue

        # Convert to absolute epoch nanoseconds
        with _records_lock:
            if _gnss_epoch_offset_ns is None:
                print("  (waiting for epoch alignment)")
                continue
            offset = _gnss_epoch_offset_ns
            gnss_absolute_ns = gnss_now + offset

        # Delta: absolute GNSS interpolation vs absolute system clock
        delta_ns = gnss_absolute_ns - sys_ns

        stats.update(float(delta_ns))

        # Foundation age
        age_us = 0.0
        age = timebase.age_ns(pi_cycles)
        if age is not None:
            age_us = age / 1000.0

        elapsed = time.monotonic() - t_start

        gnss_iso = _ns_to_iso(gnss_absolute_ns)
        sys_iso = _ns_to_iso(sys_ns)

        print(f"{elapsed:7.1f}s  "
              f"{timebase.pps_count or 0:6d}  "
              f"{gnss_iso:>33s}  "
              f"{sys_iso:>33s}  "
              f"{delta_ns:+10d}  "
              f"{stats.mean:+10.1f}  "
              f"{stats.stddev:10.1f}  "
              f"{age_us:8.1f}")

    print()
    print(f"=== Summary ({stats.n} samples) ===")
    print(f"  Mean delta:   {stats.mean:+.1f} ns")
    print(f"  Stddev:       {stats.stddev:.1f} ns")
    print(f"  Stderr:       {stats.stderr:.1f} ns")
    print()
    print("Interpretation:")
    print("  mean   ≈ systematic offset between GNSS interpolation and chrony")
    print("  stddev ≈ combined jitter of interpolation + chrony PLL")
    print("  A stddev under ~1000 ns means interpolation is sub-microsecond accurate")
    print()

    stop_event.set()
    tail_thread.join(timeout=2.0)
    print("Done.")


if __name__ == "__main__":
    main()