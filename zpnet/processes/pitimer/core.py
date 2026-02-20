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
  • The CLOCKS process calls PITIMER.REPORT via send_command when
    a TIMEBASE_FRAGMENT arrives, receiving the most recent capture

Why this works:
  • chrony disciplines the system clock to ~39 ns of GNSS via PPS
  • The second boundary in the system clock IS the PPS edge
  • Busy-polling on an isolated core detects the rollover within
    ~10-50 ns (measured) — no IRQ dispatch jitter
  • The arch timer read (MRS CNTVCT_EL0) is a single instruction
  • Detection jitter is ~400 ns stddev vs ~1200 ns with kernel IRQ

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

import logging
import os
import subprocess
import threading
import time
from typing import Any, Dict, Optional

from zpnet.processes.processes import server_setup
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

# ---------------------------------------------------------------------
# Capture state
# ---------------------------------------------------------------------

_last_capture: Optional[Dict[str, Any]] = None
_capture_lock = threading.Lock()


# ---------------------------------------------------------------------
# Capture reader thread
# ---------------------------------------------------------------------

def _parse_capture_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse a single output line from pps_poll.

    pps_poll prints lines like:
         1    11510132462     54000409       409    7574.074         40

    Fields (whitespace-separated):
      seq  counter  delta  residual  resid_ns  detect_ns

    The first capture has "---" for delta/residual/resid_ns,
    and header/separator lines start with non-digit characters.
    """
    line = line.strip()
    if not line:
        return None

    parts = line.split()
    if len(parts) < 6:
        return None

    # Skip header, separator, and summary lines
    try:
        seq = int(parts[0])
    except ValueError:
        return None

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
        "delta": delta,
        "residual": residual,
        "residual_ns": residual_ns,
        "detect_ns": detect_ns,
    }


def _capture_reader(proc: subprocess.Popen) -> None:
    """
    Read pps_poll stdout line by line and stash the latest capture.

    Runs as a daemon thread. If pps_poll exits or crashes,
    logs the error and attempts restart after a delay.
    """
    global _last_capture

    for raw_line in iter(proc.stdout.readline, b""):
        try:
            line = raw_line.decode("utf-8", errors="replace")
            capture = _parse_capture_line(line)

            if capture is not None:
                with _capture_lock:
                    _last_capture = capture

                # Log periodically (every 60 captures ≈ once per minute)
                if capture["seq"] % 60 == 0 and capture["delta"] is not None:
                    logging.info(
                        "⏱️ [pitimer] seq=%d counter=%d residual=%.1f ns detect=%d ns",
                        capture["seq"],
                        capture["counter"],
                        capture["residual_ns"] or 0.0,
                        capture["detect_ns"],
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

def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    """
    Return the most recent PPS capture from pps_poll.

    Called by CLOCKS process when a TIMEBASE_FRAGMENT arrives.

    Returns:
      {
        "success": true,
        "message": "OK",
        "payload": {
          "seq": 42,
          "counter": 11510132462,
          "delta": 54000409,
          "residual": 409,
          "residual_ns": 7574.074,
          "detect_ns": 40,
          "freq": 54000000,
          "ns_per_tick": 18.519
        }
      }
    """
    with _capture_lock:
        capture = _last_capture

    if capture is None:
        return {
            "success": True,
            "message": "OK",
            "payload": {
                "state": "WAITING",
            },
        }

    payload = dict(capture)
    payload["freq"] = PI_TIMER_FREQ
    payload["ns_per_tick"] = round(NS_PER_TICK, 3)

    return {
        "success": True,
        "message": "OK",
        "payload": payload,
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
      2) Capture reader thread (reads stdout, stashes captures)
      3) Command socket server (serves REPORT to CLOCKS)
    """
    setup_logging()

    logging.info(
        "⏱️ [pitimer] starting — Pi arch timer PPS capture via chrony polling. "
        "freq=%d Hz, %.3f ns/tick, isolated core %d",
        PI_TIMER_FREQ,
        NS_PER_TICK,
        ISOLATED_CORE,
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