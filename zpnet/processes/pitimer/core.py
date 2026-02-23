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
  • Each capture is tagged with the GNSS time for that PPS edge
    (fetched from the GNSS process via GET_TIME) and stored in
    a ring buffer of the last 60 captures
  • CLOCKS retrieves captures by GNSS time via REPORT command,
    ensuring fragment/capture correlation by physical truth

Correlation model:
  • The GNSS receiver's NMEA sentences "forecast" the UTC time
    of the next PPS edge.  The GNSS process caches this value.
  • After each pps_poll capture, PITIMER calls GNSS.GET_TIME
    to obtain the ISO8601 Zulu label for the PPS edge that
    just occurred (e.g. "2026-02-23T21:45:07Z")
  • This label is stored as the ring buffer key
  • CLOCKS independently calls GNSS.GET_TIME when processing
    a TIMEBASE_FRAGMENT, obtaining the same label
  • CLOCKS then calls PITIMER.REPORT with gnss_time=<label>
    to fetch the matching capture
  • If the labels don't match, that's a hard fault — the system
    has lost correlation and must stop rather than publish garbage

  The GNSS time label is an opaque correlation key, not a
  timestamp for arithmetic.  It is stable for ~1 second after
  PPS, so both PITIMER and CLOCKS will read the same value
  as long as they ask within a few hundred milliseconds of PPS.

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
    GNSS time label, or success=False if not found
  • Without gnss_time argument: returns the most recent capture
    (for testing, recovery, and backward compatibility)

Ring buffer diagnostics in REPORT payload:
  • gnss_time         — the GNSS time label for this capture
  • buffer_size       — current number of captures in ring buffer
  • buffer_keys       — (only in unkeyed REPORT) list of available keys

Requirements:
  • isolcpus=3 in /boot/firmware/cmdline.txt
  • pps_poll compiled: gcc -O2 -o pps_poll pps_poll.c -lm
  • chrony running and disciplined by PPS
  • GNSS process running (for GET_TIME correlation keys)

Process model:
  • One systemd service (zpnet-pitimer)
  • Two execution contexts:
      1) Capture reader thread (reads pps_poll stdout)
      2) Command socket server (serves REPORT)
"""

from __future__ import annotations

import collections
import logging
import os
import subprocess
import threading
import time
from typing import Any, Dict, Optional

from zpnet.processes.processes import server_setup, send_command
from zpnet.shared.logger import setup_logging

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
# At one capture per second, 60 entries = 60 seconds of history.
# CLOCKS typically asks within milliseconds, so this is enormous
# headroom.  If CLOCKS can't find its key in 60 seconds of history,
# something is fundamentally broken.
RING_BUFFER_CAPACITY = 60

# ---------------------------------------------------------------------
# Capture ring buffer — indexed by GNSS time (ISO8601 Zulu string)
#
# OrderedDict preserves insertion order.  When capacity is exceeded,
# the oldest entry is evicted.  Access is protected by _ring_lock.
#
# Each entry is a full capture payload dict (same fields as before,
# plus "gnss_time").
# ---------------------------------------------------------------------

_ring: collections.OrderedDict[str, Dict[str, Any]] = collections.OrderedDict()
_ring_lock = threading.Lock()

# Also keep a reference to the most recent capture for unkeyed
# REPORT calls (testing, recovery, backward compat).
_last_capture: Optional[Dict[str, Any]] = None


def _ring_put(gnss_time: str, capture: Dict[str, Any]) -> None:
    """
    Store a capture in the ring buffer, keyed by GNSS time.

    If the buffer is at capacity, the oldest entry is evicted.
    If the same gnss_time key already exists (shouldn't happen
    in normal operation), it is overwritten.
    """
    global _last_capture

    with _ring_lock:
        # If key already exists, remove it so re-insert goes to end
        if gnss_time in _ring:
            del _ring[gnss_time]

        _ring[gnss_time] = capture
        _last_capture = capture

        # Evict oldest if over capacity
        while len(_ring) > RING_BUFFER_CAPACITY:
            _ring.popitem(last=False)


def _ring_get(gnss_time: str) -> Optional[Dict[str, Any]]:
    """
    Retrieve a capture by GNSS time key.

    Returns the capture dict or None if not found.
    """
    with _ring_lock:
        return _ring.get(gnss_time)


def _ring_latest() -> Optional[Dict[str, Any]]:
    """Return the most recent capture, or None if empty."""
    with _ring_lock:
        return _last_capture


def _ring_keys() -> list:
    """Return list of available GNSS time keys (oldest first)."""
    with _ring_lock:
        return list(_ring.keys())


def _ring_size() -> int:
    """Return current number of entries in the ring buffer."""
    with _ring_lock:
        return len(_ring)


# ---------------------------------------------------------------------
# GNSS time fetcher
# ---------------------------------------------------------------------

def _get_gnss_time() -> Optional[str]:
    """
    Fetch the current GNSS time label via GNSS.GET_TIME.

    Returns the ISO8601 Zulu string (e.g. "2026-02-23T21:45:07Z")
    or None if GNSS is not yet available.
    """
    try:
        resp = send_command(
            machine="PI",
            subsystem="GNSS",
            command="GET_TIME",
        )
        if resp.get("success"):
            return resp["payload"]["gnss_time"]
        else:
            return None
    except Exception:
        logging.warning("⚠️ [pitimer] GNSS.GET_TIME failed")
        return None


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
    Read pps_poll stdout line by line, fetch GNSS time for
    correlation, and store each capture in the ring buffer.

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

                # Fetch GNSS time label for this PPS edge
                gnss_time = _get_gnss_time()

                if gnss_time is not None:
                    payload["gnss_time"] = gnss_time
                    _ring_put(gnss_time, payload)
                else:
                    # GNSS not yet available — store without key.
                    # This happens during cold start before GNSS
                    # has a fix.  We still update _last_capture
                    # so unkeyed REPORT works for recovery.
                    payload["gnss_time"] = None
                    with _ring_lock:
                        global _last_capture
                        _last_capture = payload

                # Log periodically (every 60 captures ≈ once per minute)
                if capture["seq"] % 60 == 0 and capture["delta"] is not None:
                    logging.info(
                        "⏱️ [pitimer] seq=%d counter=%d corrected=%d "
                        "residual=%.1f ns detect=%d ns corr=%d ticks "
                        "gnss_time=%s buffer=%d",
                        capture["seq"],
                        capture["counter"],
                        capture["corrected"],
                        capture["residual_ns"] or 0.0,
                        capture["detect_ns"],
                        capture["correction_ticks"],
                        gnss_time or "N/A",
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
      Returns the capture matching that GNSS time label.
      This is the primary path used by CLOCKS for correlation.
      If the key is not found, returns success=False — this is
      a hard fault indicating lost correlation.

    Without gnss_time argument:
      Returns the most recent capture (for testing, recovery,
      and backward compatibility).

    Payload includes all capture fields plus:
      • gnss_time      — the GNSS time label
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
      2) Capture reader thread (reads stdout, fetches GNSS time,
         stores in ring buffer)
      3) Command socket server (serves REPORT with keyed lookup)
    """
    setup_logging()

    logging.info(
        "⏱️ [pitimer] starting — Pi arch timer PPS capture via chrony polling. "
        "freq=%d Hz, %.3f ns/tick, isolated core %d. "
        "TDC correction enabled (detect_ns → correction_ticks). "
        "Ring buffer capacity=%d captures, indexed by GNSS time. "
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