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

Campaign start synchronisation:
  • cmd_start sets _request_start = True (same as Teensy)
  • _pps_listener sees the flag on the next PPS edge and zeros
    the Pi cycle epoch at that exact moment
  • This mirrors the Teensy's request_start → PPS ISR →
    clocks_zero_all() pattern, so both machines zero on a PPS
    boundary and their nanosecond clocks track together

Recovery after restart:
  • On startup, if an active campaign exists with TIMEBASE rows
    in Postgres, the Pi orchestrates clock recovery
  • Patiently wait for GNSS to acquire satellites and produce
    a valid timestamp (may take minutes after cold start)
  • Wait for PPS edge → capture GNSS time → compute seconds
    elapsed since last TIMEBASE
  • Project all clock nanosecond values forward using tau
    (ratio of each clock's ns to GNSS ns, from last TIMEBASE)
  • Send RECOVER command to Teensy with projected dwt_cycles,
    gnss_ns, ocxo_ns — Teensy loads these at next PPS edge
  • Compute synthetic Pi epoch so pi_ns is continuous
  • Resume normal PPS capture and TIMEBASE augmentation

Pi PPS residual tracking:
  • At each PPS edge, the Pi cycle count is snapshotted
  • Delta between consecutive PPS edges is computed
  • Residual = delta - expected_ticks_per_pps
  • Running statistics via Welford's online algorithm
  • Identical structure to Teensy's pps_residual_t

Semantics:
  • No derived values (except during recovery projection)
  • No smoothing, inference, or filtering
  • TIMEBASE records are sacred and immutable
"""

from __future__ import annotations

import ctypes
import fcntl
import logging
import math
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

# Pi monotonic_ns ticks at 1 GHz (1 ns per tick).
# Expected ticks between consecutive PPS pulses = 1,000,000,000.
PI_EXPECTED_PER_PPS = 1_000_000_000

# Teensy DWT runs at 600 MHz.
# dwt_cycles = dwt_ns * 3 / 5   (inverse of ns = cycles * 5 / 3)
DWT_CYCLES_PER_NS_NUM = 3
DWT_CYCLES_PER_NS_DEN = 5

# How often to poll GNSS while waiting for a valid fix (seconds)
GNSS_POLL_INTERVAL = 5

# How often to log a "still waiting" message (seconds)
GNSS_WAIT_LOG_INTERVAL = 60

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
# PPS residual tracking — Welford's online algorithm
#
# Structurally identical to Teensy's pps_residual_t.
# Tracks PPS-to-PPS deltas for the Pi monotonic clock.
# ---------------------------------------------------------------------

class _PpsResidual:
    """Per-clock PPS residual state with Welford's online statistics."""

    __slots__ = (
        "ticks_at_last_pps",
        "delta",
        "residual",
        "valid",
        "n",
        "mean",
        "m2",
    )

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.ticks_at_last_pps: int = 0
        self.delta: int = 0
        self.residual: int = 0
        self.valid: bool = False
        self.n: int = 0
        self.mean: float = 0.0
        self.m2: float = 0.0

    def update(self, ticks_now: int, expected: int) -> None:
        """Update with a new PPS snapshot (mirrors Teensy residual_update)."""
        if self.ticks_at_last_pps > 0:
            self.delta = ticks_now - self.ticks_at_last_pps
            self.residual = self.delta - expected
            self.valid = True

            # Welford's online algorithm
            self.n += 1
            x = float(self.residual)
            d1 = x - self.mean
            self.mean += d1 / self.n
            d2 = x - self.mean
            self.m2 += d1 * d2
        else:
            self.delta = 0
            self.residual = 0
            self.valid = False

        self.ticks_at_last_pps = ticks_now

    @property
    def variance(self) -> float:
        return (self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stddev(self) -> float:
        return math.sqrt(self.variance) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return (self.stddev / math.sqrt(self.n)) if self.n >= 2 else 0.0


# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_pi_pps_count: int = 0

# Monotonic epoch — captured at the first PPS edge after campaign
# start, subtracted from raw cycle count to produce campaign-scoped
# pi_ns (same zeroing semantics as Teensy's clocks_zero_all called
# from the PPS ISR).
_pi_cycles_epoch: int = 0

# Request flags — set by cmd_start/cmd_stop, consumed by
# _pps_listener at the next PPS edge (mirrors Teensy pattern).
_request_start: bool = False
_request_stop: bool = False

# True while a campaign is actively acquiring (set/cleared at
# PPS boundary, never between pulses).
_campaign_active: bool = False

# Pi PPS residual tracker (mirrors Teensy's residual_dwt etc.)
_residual_pi = _PpsResidual()

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
    Raw Pi cycle count — unzeroed, free-running.

    Returns time.monotonic_ns() directly, which is the Pi's
    equivalent of reading the Teensy's DWT_CYCCNT extended to
    64-bit.  Never zeroed, never adjusted.
    """
    return time.monotonic_ns()


def _read_pi_ns_from_cycles(raw_cycles: int) -> int:
    """
    Campaign-scoped Pi nanoseconds derived from raw cycle count.

    Subtracts the epoch captured at the PPS-aligned campaign start,
    equivalent to the Teensy's clocks_zero_all() zeroing the 64-bit
    accumulators at the PPS boundary.

    Pi monotonic_ns ticks at 1 GHz (1 ns per tick), so the
    conversion factor is 1:1.  Equivalent to Teensy's:
      DWT:  cycles * 5 / 3   (600 MHz)
      GNSS: ticks * 100      (10 MHz)
      Pi:   ticks * 1        (1 GHz)
    """
    return raw_cycles - _pi_cycles_epoch


def _get_gnss_time() -> str:
    """
    Fetch last-known GNSS UTC time from GNSS process.
    Returns ISO8601 Z string.

    Raises KeyError if the GNSS process has no valid fix yet
    (date/time fields not present in REPORT payload).
    """
    payload = send_command(
        machine="PI",
        subsystem="GNSS",
        command="REPORT",
    )["payload"]

    date = payload["date"]
    time_s = payload["time"]
    return f"{date}T{time_s}Z"


def _wait_for_gnss_time() -> str:
    """
    Patiently wait for the GNSS receiver to produce a valid timestamp.

    After a cold start, the receiver may need minutes to acquire
    satellites and compute a fix.  This function polls the GNSS
    REPORT every GNSS_POLL_INTERVAL seconds and logs a "still
    waiting" message every GNSS_WAIT_LOG_INTERVAL seconds.

    Returns ISO8601 Z string once available.
    Never times out — the math works regardless of how long
    the wait is.
    """
    wait_start = time.monotonic()
    last_log = wait_start

    while True:
        try:
            payload = send_command(
                machine="PI",
                subsystem="GNSS",
                command="REPORT",
            )["payload"]

            date = payload.get("date")
            time_s = payload.get("time")

            if date and time_s:
                return f"{date}T{time_s}Z"

        except Exception:
            # GNSS process might not even be up yet — that's fine
            pass

        now = time.monotonic()
        elapsed = now - wait_start

        if now - last_log >= GNSS_WAIT_LOG_INTERVAL:
            logging.info(
                "⏳ [recovery] waiting for GNSS time... (%.0fs elapsed)",
                elapsed,
            )
            last_log = now

        time.sleep(GNSS_POLL_INTERVAL)


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
                    teensy_gnss_ns,
                    teensy_ocxo_ns,
                    teensy_rtc1_ns,
                    teensy_rtc2_ns,
                    pi_cycles,
                    pi_ns,
                    teensy_pps_count,
                    pi_pps_count
                )
                VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
                """,
                (
                    tb["campaign"],
                    tb["gnss_time_utc"],
                    tb["teensy_dwt_cycles"],
                    tb["teensy_dwt_ns"],
                    tb.get("teensy_gnss_ns"),
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


# ---------------------------------------------------------------------
# PPS fetch helper (used by both _pps_listener and _recover)
# ---------------------------------------------------------------------

def _pps_fetch_blocking(fd: int, timeout_sec: int = 2) -> Optional[_PpsFdata]:
    """
    Block until the next PPS assert event on the given fd.
    Returns the filled _PpsFdata, or None on timeout.
    """
    fdata = _PpsFdata()
    fdata.timeout.sec = timeout_sec
    fdata.timeout.nsec = 0
    fdata.timeout.flags = 0

    try:
        fcntl.ioctl(fd, _PPS_FETCH, fdata)
        return fdata
    except OSError as e:
        if e.errno == 110:  # ETIMEDOUT
            return None
        raise


# ---------------------------------------------------------------------
# Recovery — restore clocks after Pi restart mid-campaign
# ---------------------------------------------------------------------

def _recover_campaign() -> None:
    """
    Detect and recover an in-progress campaign after Pi restart.

    Called once at startup, before the PPS listener begins.

    Algorithm:
      1) Check for active campaign in Postgres
      2) Fetch the most recent TIMEBASE row for that campaign
      3) Wait patiently for GNSS to produce a valid timestamp
         (may take minutes after a cold start — logs once per
         minute while waiting)
      4) If campaign has a location, restore GNSS → TO mode
      5) Wait for PPS edge, capture GNSS time at that boundary
      6) Compute elapsed seconds since last TIMEBASE
      7) Compute tau for each Teensy clock domain
      8) Project all clock values forward to the NEXT PPS edge
         (one more second beyond the one we just captured)
      9) Send RECOVER to Teensy with projected values
     10) Compute synthetic Pi epoch for continuous pi_ns
     11) Activate campaign in _pps_listener (set _campaign_active)

    The Teensy RECOVER command loads the projected values at its
    next PPS edge, then resumes publishing TIMEBASE_FRAGMENTs.

    The wait for GNSS never times out — the projection math works
    correctly regardless of how many seconds have elapsed.
    """
    global _pi_cycles_epoch, _pi_pps_count, _campaign_active

    # ----------------------------------------------------------
    # 1) Check for active campaign
    # ----------------------------------------------------------
    row = _get_active_campaign()
    if row is None:
        logging.info("🔍 [recovery] no active campaign — nothing to recover")
        return

    campaign_name = row["campaign"]
    campaign_location = row.get("location")
    logging.info(
        "🔍 [recovery] active campaign found: '%s' (location: %s)",
        campaign_name,
        campaign_location or "none",
    )

    # ----------------------------------------------------------
    # 2) Fetch most recent TIMEBASE for this campaign
    # ----------------------------------------------------------
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT *
            FROM timebase
            WHERE campaign = %s
            ORDER BY created_at DESC
            LIMIT 1
            """,
            (campaign_name,),
        )
        last_tb = cur.fetchone()

    if last_tb is None:
        logging.warning(
            "⚠️ [recovery] campaign '%s' has no TIMEBASE rows — cannot recover",
            campaign_name,
        )
        return

    last_gnss_utc: datetime = last_tb["gnss_time_utc"]
    last_dwt_cycles = int(last_tb["teensy_dwt_cycles"])
    last_dwt_ns = int(last_tb["teensy_dwt_ns"])
    last_gnss_ns = int(last_tb["teensy_gnss_ns"] or 0) if "teensy_gnss_ns" in last_tb else 0
    last_ocxo_ns = int(last_tb["teensy_ocxo_ns"] or 0)
    last_pi_ns = int(last_tb["pi_ns"])

    logging.info(
        "🔍 [recovery] last TIMEBASE: gnss_utc=%s  gnss_ns=%d  dwt_ns=%d  ocxo_ns=%d  pi_ns=%d",
        last_gnss_utc.isoformat(),
        last_gnss_ns,
        last_dwt_ns,
        last_ocxo_ns,
        last_pi_ns,
    )

    # If gnss_ns is not in the DB schema, fall back to computing
    # from dwt_ns (tau_dwt ≈ 1.0 at this scale, gnss is truth)
    if last_gnss_ns == 0:
        logging.warning(
            "⚠️ [recovery] teensy_gnss_ns not available in TIMEBASE — "
            "using dwt_ns as GNSS proxy (tau_dwt assumed ≈ 1.0)"
        )
        last_gnss_ns = last_dwt_ns

    # ----------------------------------------------------------
    # 3) Wait for GNSS to produce a valid timestamp
    #
    #    After a cold start the receiver may need minutes to
    #    acquire satellites.  We poll patiently, logging once
    #    per minute.  There is no timeout — the projection math
    #    works regardless of how long we wait.
    # ----------------------------------------------------------
    logging.info("⏳ [recovery] waiting for GNSS time to become available...")

    gnss_time_preflight = _wait_for_gnss_time()

    logging.info(
        "✅ [recovery] GNSS time available: %s",
        gnss_time_preflight,
    )

    # ----------------------------------------------------------
    # 4) If campaign has a location, restore GNSS → TO mode
    # ----------------------------------------------------------
    if campaign_location:
        logging.info(
            "📡 [recovery] restoring GNSS → TO mode for location '%s'",
            campaign_location,
        )
        try:
            gnss_resp = _set_gnss_mode_to(campaign_location)
            if gnss_resp.get("success"):
                logging.info("✅ [recovery] GNSS confirmed TO mode")
            else:
                logging.warning(
                    "⚠️ [recovery] GNSS MODE=TO failed: %s",
                    gnss_resp.get("message", "?"),
                )
        except Exception:
            logging.exception("⚠️ [recovery] GNSS MODE=TO IPC failed (continuing)")

    # ----------------------------------------------------------
    # 5) Wait for PPS edge and capture GNSS time
    # ----------------------------------------------------------
    logging.info("⏳ [recovery] waiting for PPS edge...")

    fd = os.open(PPS_DEVICE, os.O_RDONLY)
    try:
        fdata = _pps_fetch_blocking(fd, timeout_sec=5)
        if fdata is None:
            logging.error("❌ [recovery] PPS timeout — cannot recover")
            os.close(fd)
            return

        # Capture monotonic_ns at this PPS edge (for Pi epoch calc)
        mono_at_pps = time.monotonic_ns()
    finally:
        os.close(fd)

    # Get GNSS time at this PPS boundary (GNSS is known to be
    # available — we confirmed it in step 3)
    gnss_time_now_str = _get_gnss_time()
    gnss_time_now = datetime.fromisoformat(
        gnss_time_now_str.replace("Z", "+00:00")
    )

    logging.info("⏱️ [recovery] PPS captured — GNSS time now: %s", gnss_time_now_str)

    # ----------------------------------------------------------
    # 6) Compute elapsed seconds since last TIMEBASE
    # ----------------------------------------------------------
    # Both are UTC datetimes aligned to PPS second boundaries
    elapsed_td = gnss_time_now - last_gnss_utc
    elapsed_seconds = int(elapsed_td.total_seconds())

    if elapsed_seconds <= 0:
        logging.error(
            "❌ [recovery] elapsed seconds is %d — time went backwards?",
            elapsed_seconds,
        )
        return

    logging.info(
        "📐 [recovery] elapsed since last TIMEBASE: %d seconds",
        elapsed_seconds,
    )

    # ----------------------------------------------------------
    # 7) Compute tau for each clock domain
    # ----------------------------------------------------------
    # tau = clock_ns / gnss_ns (dimensionless ratio)
    # GNSS is truth, so tau_gnss = 1.0 by definition.

    tau_dwt = float(last_dwt_ns) / float(last_gnss_ns) if last_gnss_ns > 0 else 1.0
    tau_ocxo = float(last_ocxo_ns) / float(last_gnss_ns) if (last_gnss_ns > 0 and last_ocxo_ns > 0) else 1.0
    tau_pi = float(last_pi_ns) / float(last_gnss_ns) if last_gnss_ns > 0 else 1.0

    logging.info(
        "📐 [recovery] tau — dwt=%.12f  ocxo=%.12f  pi=%.12f",
        tau_dwt,
        tau_ocxo,
        tau_pi,
    )

    # ----------------------------------------------------------
    # 8) Project clock values to the NEXT PPS edge
    #    (Teensy will load these at its next PPS, which is one
    #    second after the PPS we just captured)
    # ----------------------------------------------------------
    project_seconds = elapsed_seconds + 1
    ns_per_second = 1_000_000_000

    projected_gnss_ns = last_gnss_ns + (project_seconds * ns_per_second)
    projected_dwt_ns = last_dwt_ns + int(project_seconds * ns_per_second * tau_dwt)
    projected_ocxo_ns = last_ocxo_ns + int(project_seconds * ns_per_second * tau_ocxo) if last_ocxo_ns > 0 else 0
    projected_pi_ns = last_pi_ns + int(project_seconds * ns_per_second * tau_pi)

    # Convert DWT ns back to cycles: cycles = ns * 3 / 5
    projected_dwt_cycles = (projected_dwt_ns * DWT_CYCLES_PER_NS_NUM) // DWT_CYCLES_PER_NS_DEN

    logging.info(
        "📐 [recovery] projected to PPS+1 — gnss_ns=%d  dwt_cycles=%d  dwt_ns=%d  ocxo_ns=%d  pi_ns=%d",
        projected_gnss_ns,
        projected_dwt_cycles,
        projected_dwt_ns,
        projected_ocxo_ns,
        projected_pi_ns,
    )

    # ----------------------------------------------------------
    # 9) Send RECOVER to Teensy
    # ----------------------------------------------------------
    logging.info("🚀 [recovery] sending RECOVER to Teensy...")

    try:
        recover_resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="RECOVER",
            args={
                "dwt_cycles": str(projected_dwt_cycles),
                "gnss_ns": str(projected_gnss_ns),
                "ocxo_ns": str(projected_ocxo_ns),
            },
        )

        if recover_resp.get("success", True):
            logging.info("✅ [recovery] Teensy RECOVER accepted")
        else:
            logging.warning(
                "⚠️ [recovery] Teensy RECOVER response: %s",
                recover_resp.get("message", "?"),
            )
    except Exception:
        logging.exception("❌ [recovery] Teensy RECOVER IPC failed")
        return

    # ----------------------------------------------------------
    # 10) Compute synthetic Pi epoch for continuous pi_ns
    #
    #    We need: pi_ns = monotonic_ns - epoch = projected_pi_ns
    #    at the NEXT PPS edge.  The next PPS is ~1 second from
    #    mono_at_pps.  So:
    #
    #    epoch = (mono_at_pps + 1e9) - projected_pi_ns
    #
    #    This is approximate (the actual next PPS will be a few
    #    hundred ns off from mono_at_pps + 1e9), but the error
    #    is sub-microsecond and corrects on the first real PPS
    #    capture.
    # ----------------------------------------------------------
    _pi_cycles_epoch = (mono_at_pps + ns_per_second) - projected_pi_ns
    _pi_pps_count = 0
    _residual_pi.reset()

    # ----------------------------------------------------------
    # 11) Activate campaign — PPS listener will begin capturing
    # ----------------------------------------------------------
    _campaign_active = True

    logging.info(
        "✅ [recovery] campaign '%s' recovered — "
        "Pi epoch set, Teensy RECOVER sent, awaiting first TIMEBASE_FRAGMENT",
        campaign_name,
    )


# ---------------------------------------------------------------------
# PPS listener
# ---------------------------------------------------------------------

def _pps_listener() -> None:
    """
    Listen for PPS assert events via kernel /dev/pps0 ioctl.

    Uses the PPS_FETCH ioctl which blocks until the next PPS edge.
    The kernel pps-gpio driver timestamps the edge in interrupt
    context — this is the best precision available on the Pi.

    Multiple readers (us + chrony) can read /dev/pps0 concurrently.

    State transitions (start/stop) are applied at PPS boundaries,
    mirroring the Teensy's pattern where request flags are set by
    commands and consumed inside the PPS ISR.
    """
    global _pi_pps_count
    global _pi_cycles_epoch
    global _last_pi_pps
    global _request_start
    global _request_stop
    global _campaign_active

    fd = os.open(PPS_DEVICE, os.O_RDONLY)
    logging.info("⏱️ [clocks] PPS listener started on %s (fd=%d)", PPS_DEVICE, fd)

    last_sequence = -1

    while True:
        try:
            fdata = _pps_fetch_blocking(fd)
            if fdata is None:
                logging.warning("⚠️ [clocks] PPS fetch timed out (no pulse)")
                continue

            seq = fdata.info.assert_sequence

            # Skip if we already saw this sequence number
            if seq == last_sequence:
                continue
            last_sequence = seq

            # ----------------------------------------------------------
            # PPS boundary — process request flags (mirrors Teensy ISR)
            # ----------------------------------------------------------

            if _request_stop:
                _campaign_active = False
                _request_stop = False
                logging.info("⏹️ [clocks] campaign stopped at PPS boundary")

            if _request_start:
                # Zero the Pi cycle epoch at this exact PPS edge
                # (equivalent to Teensy's clocks_zero_all in pps_isr)
                _pi_cycles_epoch = time.monotonic_ns()
                _pi_pps_count = 0
                _last_pi_pps = None
                _residual_pi.reset()
                _campaign_active = True
                _request_start = False
                logging.info("▶️ [clocks] campaign started at PPS boundary (epoch zeroed)")

            # ----------------------------------------------------------
            # Only capture clocks if a campaign is active
            # ----------------------------------------------------------

            if not _campaign_active:
                continue

            # Kernel timestamp of the PPS edge (nanoseconds since epoch)
            pps_sec = fdata.info.assert_tu.sec
            pps_nsec = fdata.info.assert_tu.nsec
            pps_kernel_ns = pps_sec * 1_000_000_000 + pps_nsec

            # Capture Pi cycle counter ASAP after ioctl returns
            pi_cycles = _read_pi_cycle_count()

            # Convert cycles → campaign-scoped ns
            pi_ns = _read_pi_ns_from_cycles(pi_cycles)

            # ----------------------------------------------------------
            # PPS residual tracking (mirrors Teensy residual_update)
            # ----------------------------------------------------------
            _residual_pi.update(pi_cycles, PI_EXPECTED_PER_PPS)

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
      4) Set _request_start flag (Pi epoch zeroed at next PPS)
      5) Start Teensy CLOCKS acquisition (Teensy also zeros at PPS)

    Both Pi and Teensy zero their clocks at the next PPS boundary
    after receiving the start command.  This ensures nanosecond
    counters on both machines begin from the same PPS pulse.
    """
    global _request_start, _request_stop

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
    # 3) Arm start — Pi epoch will be zeroed at next PPS edge
    # ----------------------------------------------------------
    _request_stop = False
    _request_start = True

    # ----------------------------------------------------------
    # 4) Start Teensy CLOCKS acquisition
    #    Teensy also zeros at its next PPS edge via request_start
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
      3) Set _request_stop flag (applied at next PPS boundary)
      4) Stop Teensy CLOCKS acquisition
      5) If campaign had a location: restore GNSS → NORMAL (CSS)
    """
    global _request_start, _request_stop

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
    # 3) Arm stop — applied at next PPS boundary
    # ----------------------------------------------------------
    _request_start = False
    _request_stop = True

    # ----------------------------------------------------------
    # 4) Stop Teensy
    # ----------------------------------------------------------
    send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="STOP",
    )

    # ----------------------------------------------------------
    # 5) Restore GNSS mode if we changed it
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

    Includes Pi clock statistics (pi_ns_now, PPS residuals) in the
    same format as the Teensy CLOCKS REPORT, so the status readout
    can display the Pi as a fourth clock domain.

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

    # ----------------------------------------------------------------
    # Pi clock state (always available, independent of DB)
    # ----------------------------------------------------------------

    pi_cycles_now = _read_pi_cycle_count()
    pi_ns_now = _read_pi_ns_from_cycles(pi_cycles_now) if _campaign_active else 0

    payload["pi_cycles_now"] = pi_cycles_now
    payload["pi_ns_now"] = pi_ns_now
    payload["pi_pps_count"] = _pi_pps_count
    payload["pi_campaign_active"] = _campaign_active

    # Pi PPS residual statistics (same keys as Teensy uses)
    r = _residual_pi
    payload["pi_pps_valid"] = r.valid
    payload["pi_pps_delta"] = r.delta
    payload["pi_pps_residual"] = r.residual
    payload["pi_pps_n"] = r.n
    payload["pi_pps_mean"] = f"{r.mean:.3f}"
    payload["pi_pps_stddev"] = f"{r.stddev:.3f}"
    payload["pi_pps_stderr"] = f"{r.stderr:.3f}"

    # ----------------------------------------------------------------
    # Campaign + TIMEBASE from DB
    # ----------------------------------------------------------------

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

    # ----------------------------------------------------------
    # Recovery check — before anything else, see if we need to
    # resume a campaign that was running when we last restarted.
    # This must happen before the PPS listener starts so that
    # _campaign_active and _pi_cycles_epoch are set correctly.
    # ----------------------------------------------------------
    try:
        _recover_campaign()
    except Exception:
        logging.exception("❌ [recovery] unhandled exception during recovery")

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