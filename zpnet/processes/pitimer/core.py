"""
ZPNet PITIMER Process — Pi Arch Timer PPS Capture

This service captures the ARM Generic Timer (CNTVCT_EL0) at each
chrony-disciplined second boundary, providing the Pi's counterpart
to the Teensy's DWT cycle counter.

Architecture:
  • A dedicated C program (pps_poll) runs on an isolated CPU core,
    busy-polling clock_gettime(CLOCK_REALTIME) for second rollovers
  • The instant a new second is detected, pps_poll captures
    CNTVCT_EL0 and writes the result to stdout
  • This Python process spawns pps_poll as a subprocess, reads its
    output line by line, and stashes the latest capture
  • Each capture is tagged with the current UTC second derived from
    the chrony-disciplined system clock and stored in a ring buffer
    of the last 5 captures
  • CLOCKS retrieves captures by time key via REPORT command,
    ensuring fragment/capture correlation by shared system clock

Correlation model:
  • pps_poll detects the second boundary via clock_gettime(CLOCK_REALTIME)
  • The capture reader calls system_time_z() to get the ISO8601 Zulu
    label for the second that just rolled over (e.g. "2026-02-23T21:45:07Z")
  • This label is stored as the ring buffer key
  • CLOCKS independently calls system_time_z() when processing a
    TIMEBASE_FRAGMENT, producing the same key (same disciplined clock,
    same UTC second)
  • CLOCKS then calls PITIMER.REPORT with gnss_time=<label> to fetch
    the matching capture
  • If the labels don't match, that's a hard fault — the system
    has lost correlation and must skip rather than publish garbage

  Both sides derive the key from the same source: the chrony-disciplined
  system clock, which tracks GNSS via PPS to ~39 ns.  No GNSS IPC is
  needed — the system clock IS the authority for "what second is it."

  NOTE: Previous versions called GNSS.GET_TIME for the correlation key.
  This created a race condition: the GNSS NMEA cache would be
  overwritten with the next second's forecast before CLOCKS could
  read it, causing systematic one-second mismatches.  Using the
  system clock eliminates this entirely — both sides read the same
  clock at moments guaranteed to be within the same UTC second.

Pi-side TDC (time-to-digital converter):
  • chrony disciplines the system clock to ~39 ns of GNSS via PPS
  • The second boundary in the system clock IS the PPS edge
  • Busy-polling on an isolated core detects the rollover within
    ~10-50 ns (measured) — no IRQ dispatch jitter
  • detect_ns = how many nanoseconds after the true second boundary
    the polling loop noticed the rollover (analogous to Teensy
    ISR dispatch latency)
  • correction_ticks = detect_ns converted to arch timer ticks
  • corrected_counter = raw_counter - correction_ticks
    (best estimate of where the counter WAS at the true PPS edge)
  • This is conceptually identical to the Teensy's software TDC:
    measure the detection delay, subtract it from the raw capture

REPORT command:
  • With gnss_time argument: returns the capture matching that
    time key, or success=False if not found
  • Without gnss_time argument: returns the most recent capture
    (for testing, recovery, and backward compatibility)

Ring buffer diagnostics in REPORT payload:
  • gnss_time         — the time key for this capture
  • buffer_size       — current number of captures in ring buffer
  • buffer_keys       — (only in unkeyed REPORT) list of available keys

Requirements:
  • isolcpus=3 in /boot/firmware/cmdline.txt
  • pps_poll compiled: gcc -O2 -o pps_poll pps_poll.c -lm
  • chrony running and disciplined by PPS

Process model:
  • One systemd service (zpnet-pitimer)
  • Two execution contexts:
      1) Capture reader thread (reads pps_poll stdout)
      2) Command socket server (serves REPORT)
"""

from __future__ import annotations

import collections
import json
import logging
import os
import subprocess
import threading
import time
from typing import Any, Dict, Optional

from zpnet.processes.processes import server_setup
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import system_time_z

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

# Path to the compiled pps_poll binary
PPSPOLL_BINARY = os.environ.get(
    "PPSPOLL_BINARY",
    "/home/mule/zpnet/zpnet/native/pps_poll/pps_poll",
)

# CPU core to pin pps_poll to (must be in isolcpus)
ISOLATED_CORE = 3

# ARM Generic Timer frequency (54 MHz on Pi 4/5)
# Read from hardware by pps_poll; we hardcode for Python-side math.
PI_TIMER_FREQ = 54_000_000
NS_PER_TICK = 1e9 / PI_TIMER_FREQ

# Ring buffer capacity — how many captures to retain.
# At one capture per second, 5 entries = 5 seconds of history.
# CLOCKS typically asks within milliseconds, so this is ample.
RING_BUFFER_CAPACITY = 5

# ---------------------------------------------------------------------
# Capture ring buffer — indexed by time key (ISO8601 Zulu string)
#
# OrderedDict preserves insertion order.  When capacity is exceeded,
# the oldest entry is evicted.  Access is protected by _ring_lock.
#
# Each entry is a full capture payload dict (same fields as before,
# plus "gnss_time" key for the correlation label).
# ---------------------------------------------------------------------

_ring: collections.OrderedDict[str, Dict[str, Any]] = collections.OrderedDict()
_ring_lock = threading.Lock()

# Also keep a reference to the most recent capture for unkeyed
# REPORT calls (testing, recovery, backward compat).
_last_capture: Optional[Dict[str, Any]] = None


def _ring_put(time_key: str, capture: Dict[str, Any]) -> None:
    """
    Store a capture in the ring buffer, keyed by time label.

    If the buffer is at capacity, the oldest entry is evicted.
    If the same key already exists (shouldn't happen in normal
    operation), it is overwritten.
    """
    global _last_capture

    with _ring_lock:
        # If key already exists, remove it so re-insert goes to end
        if time_key in _ring:
            del _ring[time_key]

        _ring[time_key] = capture
        _last_capture = capture

        # Evict oldest if over capacity
        while len(_ring) > RING_BUFFER_CAPACITY:
            _ring.popitem(last=False)


def _ring_get(time_key: str) -> Optional[Dict[str, Any]]:
    """
    Retrieve a capture by time key.

    Returns the capture dict or None if not found.
    """
    with _ring_lock:
        return _ring.get(time_key)


def _ring_latest() -> Optional[Dict[str, Any]]:
    """Return the most recent capture, or None if empty."""
    with _ring_lock:
        return _last_capture


def _ring_keys() -> list:
    """Return list of available time keys (oldest first)."""
    with _ring_lock:
        return list(_ring.keys())


def _ring_size() -> int:
    """Return current number of entries in the ring buffer."""
    with _ring_lock:
        return len(_ring)


# ---------------------------------------------------------------------
# Capture reader thread
# ---------------------------------------------------------------------

def _parse_capture_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse a single output line from pps_poll.

    pps_poll (enhanced) prints lines like:
         1    11510132462    11510132460     54000409       409    7574.074         40          2         40

    Fields (whitespace-separated):
      seq  counter  corrected  delta  residual  resid_ns  detect_ns  corr_ticks  rt_ns

    The first capture has "---" for delta/residual/resid_ns,
    and header/separator lines start with non-digit characters.

    Also handles legacy format (6 fields) for backward compatibility:
      seq  counter  delta  residual  resid_ns  detect_ns
    """
    line = line.strip()
    if not line:
        return None

    # Strip trailing Welford stats in parentheses
    paren_idx = line.find("(")
    if paren_idx != -1:
        line = line[:paren_idx].strip()

    parts = line.split()
    if len(parts) < 6:
        return None

    # Skip header, separator, and summary lines
    try:
        seq = int(parts[0])
    except ValueError:
        return None

    # Detect format by field count
    if len(parts) >= 9:
        # Enhanced format: seq counter corrected delta residual resid_ns detect_ns corr_ticks rt_ns
        counter_str = parts[1]
        corrected_str = parts[2]
        delta_str = parts[3]
        residual_str = parts[4]
        resid_ns_str = parts[5]
        detect_ns_str = parts[6]
        corr_ticks_str = parts[7]

        try:
            counter = int(counter_str)
            corrected = int(corrected_str)
            detect_ns = int(detect_ns_str)
            correction_ticks = int(corr_ticks_str)
        except ValueError:
            return None

        # Parse optional fields (first line has "---")
        delta = None
        residual = None
        residual_ns = None

        if delta_str != "---":
            try:
                delta = int(delta_str)
                residual = int(residual_str)
                residual_ns = float(resid_ns_str)
            except ValueError:
                pass

        return {
            "seq": seq,
            "counter": counter,
            "corrected": corrected,
            "delta": delta,
            "residual": residual,
            "residual_ns": residual_ns,
            "detect_ns": detect_ns,
            "correction_ticks": correction_ticks,
        }

    else:
        # Legacy format: seq counter delta residual resid_ns detect_ns
        counter_str = parts[1]
        delta_str = parts[2]
        residual_str = parts[3]
        resid_ns_str = parts[4]
        detect_ns_str = parts[5]

        try:
            counter = int(counter_str)
            detect_ns = int(detect_ns_str)
        except ValueError:
            return None

        # Compute correction from detect_ns (same math as enhanced pps_poll)
        correction_ticks = (detect_ns * PI_TIMER_FREQ) // 1_000_000_000
        corrected = counter - correction_ticks

        delta = None
        residual = None
        residual_ns = None

        if delta_str != "---":
            try:
                delta = int(delta_str)
                residual = int(residual_str)
                residual_ns = float(resid_ns_str)
            except ValueError:
                pass

        return {
            "seq": seq,
            "counter": counter,
            "corrected": corrected,
            "delta": delta,
            "residual": residual,
            "residual_ns": residual_ns,
            "detect_ns": detect_ns,
            "correction_ticks": correction_ticks,
        }


def _capture_reader(proc: subprocess.Popen) -> None:
    """
    Read pps_poll stdout line by line, generate time key from
    the system clock, and store each capture in the ring buffer.

    The time key is derived from the chrony-disciplined system clock
    via system_time_z().  Since pps_poll detects the second boundary
    from the same system clock, and this code runs within milliseconds
    of detection, the key always matches the second that just rolled.

    Runs as a daemon thread. If pps_poll exits or crashes,
    logs the error and the supervisor will restart it.
    """
    for raw_line in iter(proc.stdout.readline, b""):
        try:
            line = raw_line.decode("utf-8", errors="replace")
            capture = _parse_capture_line(line)

            if capture is not None:
                # Build the full payload with freq metadata
                payload = dict(capture)
                payload["freq"] = PI_TIMER_FREQ
                payload["ns_per_tick"] = round(NS_PER_TICK, 3)

                # Derive time key from the system clock.
                # pps_poll just detected the second boundary from
                # this same clock, so system_time_z() returns the
                # label for the second that just started.
                time_key = system_time_z()

                payload["gnss_time"] = time_key
                _ring_put(time_key, payload)

                # Log periodically (every 60 captures ≈ once per minute)
                if capture["seq"] % 60 == 0 and capture["delta"] is not None:
                    logging.info(
                        "⏱️ [pitimer] seq=%d counter=%d corrected=%d "
                        "residual=%.1f ns detect=%d ns corr=%d ticks "
                        "time_key=%s buffer=%d",
                        capture["seq"],
                        capture["counter"],
                        capture["corrected"],
                        capture["residual_ns"] or 0.0,
                        capture["detect_ns"],
                        capture["correction_ticks"],
                        time_key,
                        _ring_size(),
                    )

        except Exception:
            logging.exception("⚠️ [pitimer] error parsing pps_poll output")

    logging.warning("⚠️ [pitimer] pps_poll stdout closed")


def _spawn_ppspoll() -> subprocess.Popen:
    """
    Spawn the pps_poll C program pinned to the isolated core.

    Runs indefinitely (count=999999999) writing one line per second.
    """
    cmd = [
        "taskset", "-c", str(ISOLATED_CORE),
        PPSPOLL_BINARY,
        "999999999",  # effectively infinite
    ]

    logging.info(
        "🚀 [pitimer] spawning pps_poll on core %d: %s",
        ISOLATED_CORE,
        " ".join(cmd),
    )

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )

    return proc


def _supervisor() -> None:
    """
    Supervise the pps_poll subprocess.

    If it exits unexpectedly, log and restart after a delay.
    Runs as a daemon thread.
    """
    while True:
        try:
            proc = _spawn_ppspoll()

            # Start reader thread for this process instance
            reader = threading.Thread(
                target=_capture_reader,
                args=(proc,),
                daemon=True,
                name="pitimer-reader",
            )
            reader.start()

            # Wait for process to exit
            retcode = proc.wait()
            logging.warning(
                "⚠️ [pitimer] pps_poll exited with code %d — restarting in 5s",
                retcode,
            )

        except Exception:
            logging.exception("💥 [pitimer] supervisor error — restarting in 5s")

        time.sleep(5)


# ---------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------

def cmd_report(args: Optional[dict]) -> Dict[str, Any]:
    """
    Return a PPS capture from the ring buffer.

    With gnss_time argument:
      Returns the capture matching that time key.
      This is the primary path used by CLOCKS for correlation.
      If the key is not found, returns success=False — this is
      a hard fault indicating lost correlation.

    Without gnss_time argument:
      Returns the most recent capture (for testing, recovery,
      and backward compatibility).

    Payload includes all capture fields plus:
      • gnss_time      — the time key
      • buffer_size    — current ring buffer occupancy
      • buffer_keys    — (unkeyed only) available keys for debugging
    """
    gnss_time = None
    if args:
        gnss_time = args.get("gnss_time")

    if gnss_time is not None:
        # ----------------------------------------------------------
        # Keyed lookup — correlation path
        # ----------------------------------------------------------
        capture = _ring_get(gnss_time)

        if capture is None:
            available = _ring_keys()
            logging.warning(
                "⚠️ [pitimer] REPORT: no capture for gnss_time=%s "
                "(buffer has %d entries: %s)",
                gnss_time,
                len(available),
                available[-5:] if available else "empty",
            )
            return {
                "success": False,
                "message": "BAD",
                "payload": {
                    "error": f"unmatched time {gnss_time}",
                    "buffer_size": len(available),
                    "buffer_keys": available,
                },
            }

        result = dict(capture)
        result["buffer_size"] = _ring_size()
        return {
            "success": True,
            "message": "OK",
            "payload": result,
        }

    else:
        # ----------------------------------------------------------
        # Unkeyed — latest capture (testing/recovery)
        # ----------------------------------------------------------
        capture = _ring_latest()

        if capture is None:
            return {
                "success": True,
                "message": "OK",
                "payload": {
                    "state": "WAITING",
                },
            }

        result = dict(capture)
        result["buffer_size"] = _ring_size()
        result["buffer_keys"] = _ring_keys()
        return {
            "success": True,
            "message": "OK",
            "payload": result,
        }


COMMANDS = {
    "REPORT": cmd_report,
}


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    """
    Start the PITIMER process.

    Execution contexts:
      1) pps_poll supervisor thread (spawns + monitors C subprocess)
      2) Capture reader thread (reads stdout, derives time key from
         system clock, stores in ring buffer)
      3) Command socket server (serves REPORT with keyed lookup)
    """
    setup_logging()

    logging.info(
        "⏱️ [pitimer] starting — Pi arch timer PPS capture via chrony polling. "
        "freq=%d Hz, %.3f ns/tick, isolated core %d. "
        "TDC correction enabled (detect_ns → correction_ticks). "
        "Ring buffer capacity=%d captures, keyed by system clock (system_time_z). "
        "CLOCKS retrieves captures via REPORT(gnss_time=<key>).",
        PI_TIMER_FREQ,
        NS_PER_TICK,
        ISOLATED_CORE,
        RING_BUFFER_CAPACITY,
    )

    # Verify binary exists
    if not os.path.isfile(PPSPOLL_BINARY):
        logging.error(
            "❌ [pitimer] pps_poll binary not found at %s — "
            "build with: gcc -O2 -o pps_poll pps_poll.c -lm",
            PPSPOLL_BINARY,
        )
        return

    # Start supervisor thread (spawns pps_poll, restarts on crash)
    threading.Thread(
        target=_supervisor,
        daemon=True,
        name="pitimer-supervisor",
    ).start()

    # Start command socket server (blocks forever)
    server_setup(
        subsystem="PITIMER",
        commands=COMMANDS,
    )


if __name__ == "__main__":
    run()