"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side)

Responsibilities:
  • Capture PPS pulse via kernel /dev/pps0 driver
  • Capture Pi cycle count and Pi nanoseconds at PPS
  • Receive TIMEBASE_FRAGMENT from Teensy
  • Augment fragment into full TIMEBASE record
  • Publish TIMEBASE
  • Persist TIMEBASE to PostgreSQL (append-only, best-effort)
  • Recover clocks after restart if campaign is active
  • Command GNSS into Time Only mode when campaign has a location

PPS capture:
  • The pps-gpio kernel driver owns GPIO18 and timestamps edges
    in interrupt context (best possible precision)
  • We read /dev/pps0 via PPS_FETCH ioctl (same as ppstest)
  • chrony also reads /dev/pps0 — multiple readers are supported
  • No gpiod dependency; no pin ownership conflict

Semantics:
  • No derived values
  • No smoothing, inference, or filtering
  • TIMEBASE records are sacred and immutable
"""

from __future__ import annotations

import ctypes
import fcntl
import logging
import os
import threading
import time
from datetime import datetime, timezone
from typing import Dict, Any, Optional

from zpnet.processes.processes import (
    server_setup,
    publish,
    send_command,
)
from zpnet.shared.constants import Payload
from zpnet.shared.db import open_db
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

PPS_DEVICE = "/dev/pps0"

# ---------------------------------------------------------------------
# Kernel PPS ioctl interface (linux/pps.h)
#
# struct pps_ktime {
#     __s64   sec;
#     __s32   nsec;
#     __u32   flags;
# };                              /* 16 bytes */
#
# struct pps_kinfo {
#     __u32   assert_sequence;
#     __u32   clear_sequence;
#     struct pps_ktime assert_tu;
#     struct pps_ktime clear_tu;
#     int     current_mode;
# };                              /* 44 bytes */
#
# struct pps_fdata {
#     struct pps_kinfo info;
#     struct pps_ktime timeout;
# };                              /* 60 bytes */
#
# #define PPS_FETCH  _IOWR('p', 0xa4, struct pps_fdata)
# ---------------------------------------------------------------------

class _PpsKtime(ctypes.Structure):
    _fields_ = [
        ("sec", ctypes.c_int64),
        ("nsec", ctypes.c_int32),
        ("flags", ctypes.c_uint32),
    ]

class _PpsKinfo(ctypes.Structure):
    _fields_ = [
        ("assert_sequence", ctypes.c_uint32),
        ("clear_sequence", ctypes.c_uint32),
        ("assert_tu", _PpsKtime),
        ("clear_tu", _PpsKtime),
        ("current_mode", ctypes.c_int32),
    ]

class _PpsFdata(ctypes.Structure):
    _fields_ = [
        ("info", _PpsKinfo),
        ("timeout", _PpsKtime),
    ]

# PPS_FETCH ioctl number: _IOWR('p', 0xa4, struct pps_fdata *)
#
# Note: the kernel header uses a POINTER to pps_fdata, so the size
# field in the ioctl number is sizeof(pointer), not sizeof(struct).
# On ARM64 (Pi 4/5): sizeof(void*) = 8  → ioctl = 0xC00870A4
# Confirmed via: gcc -include linux/pps.h -e 'printf("%lX", PPS_FETCH)'
_PPS_FETCH = 0xC00870A4

# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_pi_pps_count: int = 0

# Monotonic epoch — captured at campaign start, subtracted from all
# subsequent readings so pi_cycles and pi_ns are campaign-scoped
# (same zeroing semantics as Teensy's clocks_zero_all).
_pi_cycles_epoch: int = 0

# Latest Pi-side PPS capture
_last_pi_pps: Optional[Dict[str, Any]] = None

# Latest Teensy fragment awaiting augmentation
_last_teensy_fragment: Optional[Dict[str, Any]] = None

_state_lock = threading.Lock()

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _read_pi_cycle_count() -> int:
    """
    Campaign-scoped Pi cycle count.

    Uses time.monotonic_ns() as the raw clock substrate, offset by
    the epoch captured at campaign start — same zeroing semantics
    as Teensy's clocks_zero_all().

    Before any campaign starts, returns raw monotonic_ns (epoch=0).
    """
    return time.monotonic_ns() - _pi_cycles_epoch


def _read_pi_ns_from_cycles(cycles_ns: int) -> int:
    """
    Convert Pi cycles to nanoseconds.

    Pi monotonic_ns ticks at 1 GHz (1 ns per tick), so the
    conversion factor is 1:1.  Equivalent to Teensy's:
      DWT:  cycles * 5 / 3   (600 MHz)
      GNSS: ticks * 100      (10 MHz)
      Pi:   ticks * 1        (1 GHz)
    """
    return cycles_ns


def _get_gnss_time() -> str:
    """
    Fetch last-known GNSS UTC time from GNSS process.
    Returns ISO8601 Z string.
    """
    payload = send_command(
        machine="PI",
        subsystem="GNSS",
        command="REPORT",
    )["payload"]

    date = payload["date"]
    time_s = payload["time"]
    return f"{date}T{time_s}Z"


def _get_active_campaign() -> Optional[Dict[str, Any]]:
    """
    Return active campaign row or None.

    Returns dict with keys: campaign, location (nullable).
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT campaign, location
            FROM campaigns
            WHERE active = true
            ORDER BY started_at DESC
            LIMIT 1
            """
        )
        return cur.fetchone()


def _set_gnss_mode_to(location: str) -> Dict[str, Any]:
    """
    Command GNSS process to switch to Time Only mode at a location.

    Blocks until GNSS confirms the mode change (or times out).
    Returns the GNSS MODE command response dict.
    Raises on IPC failure.
    """
    return send_command(
        machine="PI",
        subsystem="GNSS",
        command="MODE",
        args={"mode": "TO", "location": location},
    )


def _set_gnss_mode_normal() -> Dict[str, Any]:
    """
    Command GNSS process to return to normal (CSS) mode.

    Blocks until GNSS confirms the mode change (or times out).
    Returns the GNSS MODE command response dict.
    Raises on IPC failure.
    """
    return send_command(
        machine="PI",
        subsystem="GNSS",
        command="MODE",
        args={"mode": "NORMAL"},
    )


def _persist_timebase(tb: Dict[str, Any]) -> None:
    """
    Best-effort TIMEBASE insert.
    Failure is logged and ignored.
    """
    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO timebase (
                    campaign,
                    gnss_time_utc,
                    teensy_dwt_cycles,
                    teensy_dwt_ns,
                    teensy_ocxo_ns,
                    teensy_rtc1_ns,
                    teensy_rtc2_ns,
                    pi_cycles,
                    pi_ns,
                    teensy_pps_count,
                    pi_pps_count
                )
                VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
                """,
                (
                    tb["campaign"],
                    tb["gnss_time_utc"],
                    tb["teensy_dwt_cycles"],
                    tb["teensy_dwt_ns"],
                    tb.get("teensy_ocxo_ns"),
                    tb.get("teensy_rtc1_ns"),
                    tb.get("teensy_rtc2_ns"),
                    tb["pi_cycles"],
                    tb["pi_ns"],
                    tb.get("teensy_pps_count"),
                    tb.get("pi_pps_count"),
                ),
            )
    except Exception:
        logging.exception("⚠️ [clocks] failed to persist TIMEBASE (ignored)")


def _pps_listener() -> None:
    """
    Listen for PPS assert events via kernel /dev/pps0 ioctl.

    Uses the PPS_FETCH ioctl which blocks until the next PPS edge.
    The kernel pps-gpio driver timestamps the edge in interrupt
    context — this is the best precision available on the Pi.

    Multiple readers (us + chrony) can read /dev/pps0 concurrently.
    """
    global _pi_pps_count
    global _last_pi_pps

    fd = os.open(PPS_DEVICE, os.O_RDONLY)
    logging.info("⏱️ [clocks] PPS listener started on %s (fd=%d)", PPS_DEVICE, fd)

    last_sequence = -1

    while True:
        try:
            # Set up fetch request with 2-second timeout
            fdata = _PpsFdata()
            fdata.timeout.sec = 2
            fdata.timeout.nsec = 0
            fdata.timeout.flags = 0  # valid timeout

            # Block until next PPS assert event
            fcntl.ioctl(fd, _PPS_FETCH, fdata)

            seq = fdata.info.assert_sequence

            # Skip if we already saw this sequence number
            if seq == last_sequence:
                continue
            last_sequence = seq

            # Kernel timestamp of the PPS edge (nanoseconds since epoch)
            pps_sec = fdata.info.assert_tu.sec
            pps_nsec = fdata.info.assert_tu.nsec
            pps_kernel_ns = pps_sec * 1_000_000_000 + pps_nsec

            # Capture Pi cycle counter ASAP after ioctl returns
            pi_cycles = _read_pi_cycle_count()

            # Convert cycles → ns
            pi_ns = _read_pi_ns_from_cycles(pi_cycles)

            with _state_lock:
                _last_pi_pps = {
                    "pps_kernel_ns": pps_kernel_ns,
                    "pi_cycles": pi_cycles,
                    "pi_ns": pi_ns,
                    "pi_pps_count": _pi_pps_count,
                }

            _pi_pps_count += 1

        except OSError as e:
            if e.errno == 110:  # ETIMEDOUT — no PPS pulse within timeout
                logging.warning("⚠️ [clocks] PPS fetch timed out (no pulse)")
                continue
            logging.exception("[_pps_listener] ioctl error")
            time.sleep(1)
        except Exception:
            logging.exception("[_pps_listener] unhandled exception")
            time.sleep(1)

# ---------------------------------------------------------------------
# Teensy fragment handler
# ---------------------------------------------------------------------

def on_timebase_fragment(payload: Payload) -> None:
    """
    Receive TIMEBASE_FRAGMENT from Teensy.
    Attempt to complete TIMEBASE if Pi PPS is available.
    """
    global _last_teensy_fragment

    with _state_lock:
        _last_teensy_fragment = payload.copy()

    _try_complete_timebase()


def _try_complete_timebase() -> None:
    """
    Combine Pi PPS + Teensy fragment + GNSS time into TIMEBASE.
    """
    global _last_pi_pps, _last_teensy_fragment

    with _state_lock:
        if _last_pi_pps is None or _last_teensy_fragment is None:
            return

        pi_pps = _last_pi_pps
        frag = _last_teensy_fragment

        # Clear state immediately (one-shot)
        _last_pi_pps = None
        _last_teensy_fragment = None

    row = _get_active_campaign()
    if row is None:
        logging.warning("⚠️ [clocks] PPS received with no active campaign")
        return

    campaign = row["campaign"]
    gnss_time = _get_gnss_time()

    system_time = datetime.now(timezone.utc).isoformat(timespec='milliseconds')

    timebase = {
        "campaign": campaign,
        "system_time_utc": system_time,
        "gnss_time_utc": gnss_time,

        # Teensy clocks
        "teensy_dwt_cycles": frag["dwt_cycles"],
        "teensy_dwt_ns": frag["dwt_ns"],
        "teensy_gnss_ns": frag["gnss_ns"],
        "teensy_ocxo_ns": frag.get("ocxo_ns"),
        "teensy_rtc1_ns": frag.get("rtc1_ns"),
        "teensy_rtc2_ns": frag.get("rtc2_ns"),

        # Pi clocks
        "pi_cycles": pi_pps["pi_cycles"],
        "pi_ns": pi_pps["pi_ns"],

        # Counters (for sanity checking)
        "teensy_pps_count": frag.get("teensy_pps_count"),
        "pi_pps_count": pi_pps["pi_pps_count"],
    }

    # Publish sacred tuple
    publish("TIMEBASE", timebase)

    # Persist best-effort
    _persist_timebase(timebase)


# ---------------------------------------------------------------------
# Command handlers (control plane)
# ---------------------------------------------------------------------

def cmd_start(args: Optional[dict]) -> dict:
    """
    Start a new campaign.

    Args:
        campaign (str):   Campaign name (required)
        location (str):   Profiled location name (optional)
                          When provided, GNSS is switched to Time Only
                          mode at this location before Teensy acquisition
                          begins. When omitted, GNSS mode is left as-is.

    Sequence:
      1) Deactivate any existing campaign
      2) Insert new campaign row (with optional location)
      3) If location specified: command GNSS → TO mode (blocking)
      4) Start Teensy CLOCKS acquisition
    """
    global _pi_pps_count, _pi_cycles_epoch
    _pi_pps_count = 0
    _pi_cycles_epoch = time.monotonic_ns()

    campaign = args.get("campaign")
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    location = (args.get("location") or "").strip() or None

    # ----------------------------------------------------------
    # 1) Deactivate existing + insert new campaign
    # ----------------------------------------------------------
    with open_db() as conn:
        cur = conn.cursor()

        cur.execute(
            "UPDATE campaigns SET active=false, stopped_at=now() WHERE active=true"
        )

        cur.execute(
            """
            INSERT INTO campaigns (campaign, location, started_at, active)
            VALUES (%s, %s, now(), true)
            """,
            (campaign, location),
        )

    # ----------------------------------------------------------
    # 2) If location specified, switch GNSS to Time Only mode
    # ----------------------------------------------------------
    if location:
        logging.info(
            "📡 [clocks] commanding GNSS → TO mode for location '%s'",
            location,
        )

        try:
            gnss_resp = _set_gnss_mode_to(location)
        except Exception:
            logging.exception("💥 [clocks] GNSS MODE=TO IPC failed")
            return {
                "success": False,
                "message": f"failed to command GNSS to TO mode for '{location}'",
            }

        if not gnss_resp.get("success"):
            msg = gnss_resp.get("message", "unknown error")
            logging.warning("⚠️ [clocks] GNSS MODE=TO failed: %s", msg)
            return {
                "success": False,
                "message": f"GNSS MODE=TO failed: {msg}",
                "payload": {"gnss_response": gnss_resp},
            }

        logging.info("✅ [clocks] GNSS confirmed TO mode")

    # ----------------------------------------------------------
    # 3) Start Teensy CLOCKS acquisition
    # ----------------------------------------------------------
    send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="START",
        args={"campaign": campaign},
    )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": campaign,
            "location": location,
        },
    }


def cmd_stop(_: Optional[dict]) -> dict:
    """
    Stop active campaign.

    Sequence:
      1) Read active campaign (to check if it had a location)
      2) Deactivate campaign in DB
      3) Stop Teensy CLOCKS acquisition
      4) If campaign had a location: restore GNSS → NORMAL (CSS)
    """

    # ----------------------------------------------------------
    # 1) Check if active campaign had a location
    # ----------------------------------------------------------
    row = _get_active_campaign()
    had_location = row["location"] if row else None

    # ----------------------------------------------------------
    # 2) Deactivate campaign
    # ----------------------------------------------------------
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            "UPDATE campaigns SET active=false, stopped_at=now() WHERE active=true"
        )

    # ----------------------------------------------------------
    # 3) Stop Teensy
    # ----------------------------------------------------------
    send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="STOP",
    )

    # ----------------------------------------------------------
    # 4) Restore GNSS mode if we changed it
    # ----------------------------------------------------------
    if had_location:
        logging.info(
            "📡 [clocks] restoring GNSS → NORMAL (campaign had location '%s')",
            had_location,
        )

        try:
            gnss_resp = _set_gnss_mode_normal()

            if not gnss_resp.get("success"):
                logging.warning(
                    "⚠️ [clocks] GNSS MODE=NORMAL failed: %s",
                    gnss_resp.get("message", "?"),
                )
        except Exception:
            logging.exception("⚠️ [clocks] GNSS MODE=NORMAL IPC failed (ignored)")

    return {"success": True, "message": "OK"}


def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    """
    Report current CLOCKS state.

    Semantics:
      • Observational only
      • No inference
      • No derived values
    """

    payload: Dict[str, Any] = {
        "campaign": {
            "active": False
        }
    }

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()

        # ------------------------------------------------------------
        # Fetch active campaign (authoritative)
        # ------------------------------------------------------------
        cur.execute(
            """
            SELECT id, campaign, location, started_at
            FROM campaigns
            WHERE active = true
            ORDER BY started_at DESC
            LIMIT 1
            """
        )
        campaign = cur.fetchone()

        if not campaign:
            return {
                "success": True,
                "message": "OK",
                "payload": payload,
            }

        campaign_id = campaign["id"]

        payload["campaign"] = {
            "active": True,
            "campaign_id": campaign_id,
            "campaign": campaign["campaign"],
            "location": campaign["location"],
            "started_at": campaign["started_at"].isoformat(),
            "last_timebase_id": None,
        }

        # ------------------------------------------------------------
        # Fetch most recent TIMEBASE for this campaign
        # ------------------------------------------------------------
        cur.execute(
            """
            SELECT *
            FROM timebase
            WHERE campaign = %s
            ORDER BY created_at DESC
            LIMIT 1
            """,
            (campaign["campaign"],)
        )
        tb = cur.fetchone()

        if tb:
            payload["campaign"]["last_timebase_id"] = tb["id"]

            payload["timebase"] = {
                "id": tb["id"],
                "created_at": tb["created_at"].isoformat(),
                "gnss_time_utc": tb["gnss_time_utc"].isoformat(),
                "teensy_dwt_cycles": tb["teensy_dwt_cycles"],
                "teensy_dwt_ns": tb["teensy_dwt_ns"],
                "teensy_ocxo_ns": tb["teensy_ocxo_ns"],
                "teensy_rtc1_ns": tb["teensy_rtc1_ns"],
                "teensy_rtc2_ns": tb["teensy_rtc2_ns"],
                "pi_cycles": tb["pi_cycles"],
                "pi_ns": tb["pi_ns"],
            }
        else:
            payload["timebase"] = None

    return {
        "success": True,
        "message": "OK",
        "payload": payload,
    }


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    # Start PPS listener thread (reads /dev/pps0 via kernel ioctl)
    threading.Thread(
        target=_pps_listener,
        daemon=True,
        name="pps-listener",
    ).start()

    # Start server
    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={
            "TIMEBASE_FRAGMENT": on_timebase_fragment,
        },
    )


if __name__ == "__main__":
    run()
