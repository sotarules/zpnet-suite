"""
ZPNet PITIMER Process — Pi Arch Timer PPS Capture (v3 — START/STOP lifecycle)

Intent (v2026-02-28+):
  PITIMER is the Pi clock authority.  It owns both the raw CNTVCT_EL0
  cycle count AND the derived Pi nanosecond clock.

  This makes the architecture symmetric:
    • Teensy owns GNSS ns, DWT ns/cycles, OCXO ns — delivered in TIMEBASE_FRAGMENT
    • PITIMER owns Pi ns and Pi ticks — served via GET_CAPTURE
    • CLOCKS is a pure traffic cop: correlates by pps_count, joins, persists

  v3 adds explicit START/STOP lifecycle:

    STOPPED (default):
      Capture loop runs continuously to keep delta tracking warm, but
      captures are NOT buffered, NOT assigned pps_count, and evictions
      are impossible.  PITIMER is calm.  This is the idle state between
      campaigns and at boot.

    RUNNING (after START):
      Captures are buffered by pps_count for CLOCKS consumption.
      Metrics are tracked, evictions are warnings, pi_ns is computed
      if epoch is set.  This is the active campaign state.

    START transitions to RUNNING.  It takes pps_count (required) and
    optionally pi_tick_epoch.  If epoch is not provided, CLOCKS calls
    SET_EPOCH separately after the sync fragment arrives.

    STOP clears all campaign state and returns to idle.  The capture
    loop continues uninterrupted — only buffering stops.

  SET_PPS_COUNT and SET_EPOCH remain as independent primitives.
  SET_PPS_COUNT can be used without START (e.g. re-arming mid-campaign).
  SET_EPOCH can be called at any time to update the epoch.

  Captures are stored in a dynamic buffer indexed by pps_count.  CLOCKS
  fetches captures by identity (GET_CAPTURE pps_count=N), not by timing.
  This eliminates the race condition where a late TIMEBASE_FRAGMENT causes
  PITIMER to have already advanced past the needed capture.

  Epoch management:
    • CLOCKS calls START(pps_count, [pi_tick_epoch]) or SET_EPOCH later.
    • PITIMER computes pi_ns = (corrected - epoch) * 1e9 / freq for every
      capture, storing it in the buffer alongside the raw ticks.
    • CLOCKS never computes pi_ns — it reads it from GET_CAPTURE.

  Supports two capture modes:

    LOOP (default):
      Uses libppscapture.so via ctypes to detect second boundaries by
      polling clock_gettime(CLOCK_REALTIME).

    PPS:
      Uses the kernel PPS subsystem (/dev/pps0) via ioctl(PPS_FETCH).

Contract:
  • CLOCKS MUST call START before captures are buffered.
  • CLOCKS SHOULD call SET_EPOCH before pi_ns is valid (or pass
    pi_tick_epoch in START).
  • GET_CAPTURE(pps_count=N) returns and REMOVES the capture from the buffer.
  • GET_CAPTURE(pps_count=N, peek=true) returns WITHOUT removing (for epoch).
  • REPORT returns the latest capture (non-destructive) plus buffer state.
  • STOP returns PITIMER to idle — capture loop continues, buffering stops.

Command surface:
  • START         — arm with pps_count, optionally set epoch, begin buffering
  • STOP          — stop buffering, clear buffer, return to idle
  • SET_PPS_COUNT — re-arm pps_count (without lifecycle transition)
  • SET_EPOCH     — set Pi tick epoch for pi_ns computation
  • GET_CAPTURE   — fetch a capture by pps_count (consume or peek)
  • REPORT        — latest capture + buffer state (non-destructive)
  • NS            — current chrony-disciplined time
  • SET_MODE      — switch LOOP/PPS
  • GET_MODE      — query current mode
"""

from __future__ import annotations

import ctypes
import logging
import os
import threading
import time
from collections import OrderedDict
from typing import Any, Dict, Optional

from zpnet.processes.processes import server_setup
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import system_time_z

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

PPSCAPTURE_LIB = os.environ.get(
    "PPSCAPTURE_LIB",
    "/home/mule/zpnet/zpnet/native/ppscapture/libppscapture.so",
)

CNTVCT_LIB = os.environ.get(
    "CNTVCT_LIB",
    "/home/mule/zpnet/zpnet/native/cntvct/libcntvct.so",
)

PPSFETCH_LIB = os.environ.get(
    "PPSFETCH_LIB",
    "/home/mule/zpnet/zpnet/native/ppsfetch/libppsfetch.so",
)

PPS_DEVICE = os.environ.get("PPS_DEVICE", "/dev/pps0")

PI_TIMER_FREQ = 54_000_000
NS_PER_SECOND = 1_000_000_000
NS_PER_TICK = 1e9 / PI_TIMER_FREQ

# Safety limit: if buffer grows beyond this, oldest entries are pruned.
# Under normal operation the buffer should never exceed ~5 entries.
# This prevents unbounded memory growth if CLOCKS stops consuming.
BUFFER_MAX_SIZE = 120

# Capture mode: "loop" or "pps"
_capture_mode: str = "loop"
_capture_mode_lock = threading.Lock()

# ---------------------------------------------------------------------
# Native library interface (LOOP mode)
# ---------------------------------------------------------------------

_lib: Optional[ctypes.CDLL] = None


def _load_lib() -> ctypes.CDLL:
    """Load libppscapture.so and configure function signatures."""
    lib = ctypes.CDLL(PPSCAPTURE_LIB)
    lib.ppscapture_next.argtypes = [
        ctypes.POINTER(ctypes.c_uint64),  # counter_out
        ctypes.POINTER(ctypes.c_int64),   # detect_ns_out
        ctypes.POINTER(ctypes.c_int64),   # rt_sec_out
        ctypes.POINTER(ctypes.c_int64),   # rt_nsec_out
    ]
    lib.ppscapture_next.restype = ctypes.c_int
    return lib


# ---------------------------------------------------------------------
# ARM Generic Timer read (PPS mode — via libcntvct.so)
# ---------------------------------------------------------------------

_cntvct_lib: Optional[ctypes.CDLL] = None


def _load_cntvct_lib() -> ctypes.CDLL:
    """Load libcntvct.so and configure function signature."""
    lib = ctypes.CDLL(CNTVCT_LIB)
    lib.read_cntvct_el0.argtypes = []
    lib.read_cntvct_el0.restype = ctypes.c_uint64
    return lib


def _read_cntvct() -> int:
    """Read CNTVCT_EL0 via libcntvct.so."""
    return _cntvct_lib.read_cntvct_el0()


# ---------------------------------------------------------------------
# PPS device interface (PPS mode — via libppsfetch.so)
# ---------------------------------------------------------------------

_pps_lib: Optional[ctypes.CDLL] = None
_pps_fd: Optional[int] = None


def _load_pps_lib() -> ctypes.CDLL:
    """Load libppsfetch.so and configure function signatures."""
    lib = ctypes.CDLL(PPSFETCH_LIB)

    lib.pps_open.argtypes = [ctypes.c_char_p]
    lib.pps_open.restype = ctypes.c_int

    lib.pps_fetch_assert.argtypes = [
        ctypes.c_int,                       # fd
        ctypes.c_int,                       # timeout_sec
        ctypes.POINTER(ctypes.c_int64),     # sec_out
        ctypes.POINTER(ctypes.c_int32),     # nsec_out
        ctypes.POINTER(ctypes.c_uint32),    # seq_out
    ]
    lib.pps_fetch_assert.restype = ctypes.c_int

    return lib


def _open_pps_device() -> int:
    """Open /dev/pps0 via libppsfetch and return the file descriptor."""
    fd = _pps_lib.pps_open(PPS_DEVICE.encode("utf-8"))
    if fd < 0:
        raise OSError(f"Cannot open {PPS_DEVICE}")
    logging.info("✅ [pitimer] opened %s (fd=%d)", PPS_DEVICE, fd)
    return fd


def _pps_fetch(fd: int, timeout_sec: int = 3) -> Dict[str, Any]:
    """
    Block until next PPS assert edge via libppsfetch (ioctl PPS_FETCH).

    Returns dict with:
      - pps_sec:   kernel CLOCK_REALTIME seconds at GPIO interrupt
      - pps_nsec:  kernel CLOCK_REALTIME nanoseconds at GPIO interrupt
      - pps_seq:   kernel PPS assert sequence number
    """
    sec = ctypes.c_int64()
    nsec = ctypes.c_int32()
    seq = ctypes.c_uint32()

    rc = _pps_lib.pps_fetch_assert(
        fd, timeout_sec,
        ctypes.byref(sec), ctypes.byref(nsec), ctypes.byref(seq),
    )
    if rc != 0:
        raise OSError(f"PPS_FETCH failed (fd={fd}, timeout={timeout_sec}s)")

    return {
        "pps_sec": sec.value,
        "pps_nsec": nsec.value,
        "pps_seq": seq.value,
    }


# ---------------------------------------------------------------------
# Capture functions (one per mode)
# ---------------------------------------------------------------------


def _capture_one_loop() -> Dict[str, Any]:
    """LOOP mode: block until next second boundary via libppscapture."""
    counter = ctypes.c_uint64()
    detect_ns = ctypes.c_int64()
    rt_sec = ctypes.c_int64()
    rt_nsec = ctypes.c_int64()

    _lib.ppscapture_next(
        ctypes.byref(counter), ctypes.byref(detect_ns),
        ctypes.byref(rt_sec), ctypes.byref(rt_nsec),
    )

    raw = counter.value
    dns = detect_ns.value

    # TDC correction: subtract detection latency
    correction_ticks = (dns * PI_TIMER_FREQ) // NS_PER_SECOND
    corrected = raw - correction_ticks

    # Chrono-disciplined realtime: full nanoseconds since epoch
    chrony_ns = (rt_sec.value * NS_PER_SECOND) + rt_nsec.value

    return {
        "counter": raw,
        "corrected": corrected,
        "detect_ns": dns,
        "correction_ticks": correction_ticks,
        "chrony_ns": chrony_ns,
        "chrony_sec": rt_sec.value,
        "chrony_nsec": rt_nsec.value,
        "freq": PI_TIMER_FREQ,
        "ns_per_tick": round(NS_PER_TICK, 3),
        "capture_mode": "loop",
    }


def _capture_one_pps() -> Dict[str, Any]:
    """
    PPS mode: block on ioctl(PPS_FETCH), then read CNTVCT_EL0.
    """
    global _pps_fd

    if _pps_fd is None:
        _pps_fd = _open_pps_device()

    pps = _pps_fetch(_pps_fd)
    raw = _read_cntvct()

    now_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
    pps_ns = (pps["pps_sec"] * NS_PER_SECOND) + pps["pps_nsec"]
    detect_ns = now_ns - pps_ns

    correction_ticks = (detect_ns * PI_TIMER_FREQ) // NS_PER_SECOND
    corrected = raw - correction_ticks

    chrony_ns = pps_ns

    return {
        "counter": raw,
        "corrected": corrected,
        "detect_ns": detect_ns,
        "correction_ticks": correction_ticks,
        "chrony_ns": chrony_ns,
        "chrony_sec": pps["pps_sec"],
        "chrony_nsec": pps["pps_nsec"],
        "freq": PI_TIMER_FREQ,
        "ns_per_tick": round(NS_PER_TICK, 3),
        "capture_mode": "pps",
        "pps_seq": pps["pps_seq"],
    }


def _capture_one() -> Dict[str, Any]:
    """Dispatch to the active capture mode."""
    with _capture_mode_lock:
        mode = _capture_mode

    if mode == "pps":
        return _capture_one_pps()
    else:
        return _capture_one_loop()


# ---------------------------------------------------------------------
# Pi nanosecond computation (epoch-relative)
# ---------------------------------------------------------------------


def _compute_pi_ns(corrected: int) -> Optional[int]:
    """
    Compute campaign-relative Pi nanoseconds from corrected tick count.

    Returns None if epoch is not set.
    Uses integer arithmetic: (campaign_ticks * 1e9) // freq
    """
    if not _epoch_set:
        return None
    campaign_ticks = int(corrected) - int(_pi_tick_epoch)
    return (campaign_ticks * NS_PER_SECOND) // PI_TIMER_FREQ


# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Capture loop
    "captures_total": 0,
    "captures_idle": 0,
    "capture_exceptions": 0,

    # Buffer management
    "buffer_inserts": 0,
    "buffer_evictions": 0,
    "buffer_hits": 0,
    "buffer_peeks": 0,
    "buffer_misses": 0,
    "buffer_size_current": 0,
    "buffer_size_max_seen": 0,

    # PPS-count continuity
    "pps_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_regress": 0,
    "pps_count_jump": 0,

    # Epoch
    "epoch_set_requests": 0,
    "epoch_set_applied": 0,
    "epoch_value": None,

    # Lifecycle
    "starts": 0,
    "stops": 0,
    "last_start": {},
    "last_stop": {},

    # Last-seen values
    "last_time_key": None,
    "last_pps_count": None,
    "last_seq": None,

    # Last anomaly snapshots
    "last_pps_count_anomaly": {},
    "last_set_event": {},
    "last_capture_exception": {},
    "last_eviction": {},

    # Mode transitions
    "mode_changes": 0,
    "last_mode_change": {},
}

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

_state_lock = threading.Lock()

# Lifecycle: RUNNING means captures are buffered for CLOCKS consumption.
# STOPPED means the capture loop still runs but nothing is buffered.
_running: bool = False

# Capture buffer: pps_count -> capture dict (insertion-ordered)
_capture_buffer: OrderedDict[int, Dict[str, Any]] = OrderedDict()

# Latest capture reference (non-destructive, for REPORT).
# Updated in both RUNNING and STOPPED modes so REPORT always has
# the most recent capture for diagnostics.
_last_capture: Optional[Dict[str, Any]] = None
_capture_seq: int = 0

# PPS count assignment (only meaningful when RUNNING)
_pps_armed: bool = False
_pps_next_count: int = 0
_last_pps_count_seen: Optional[int] = None

# Epoch (Pi tick epoch for ns computation)
_pi_tick_epoch: int = 0
_epoch_set: bool = False

# Delta tracking (always active — keeps running across START/STOP)
_last_corrected: Optional[int] = None

# ---------------------------------------------------------------------
# PPS count assignment (MANDATORY when RUNNING)
# ---------------------------------------------------------------------


def _assign_pps_count() -> int:
    """
    Assign pps_count for THIS capture and increment for next.
    Called under _state_lock when _running and _pps_armed are True.
    """
    global _pps_next_count, _last_pps_count_seen

    k = int(_pps_next_count)
    _pps_next_count += 1

    _diag["pps_count_seen"] += 1
    _diag["last_pps_count"] = k

    if _last_pps_count_seen is not None:
        if k == _last_pps_count_seen:
            _diag["pps_count_repeat"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": system_time_z(),
                "reason": "pps_count_repeat",
                "pps_count": k,
                "prev_pps_count": int(_last_pps_count_seen),
            }
        elif k < _last_pps_count_seen:
            _diag["pps_count_regress"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": system_time_z(),
                "reason": "pps_count_regress",
                "pps_count": k,
                "prev_pps_count": int(_last_pps_count_seen),
            }
        elif k > (_last_pps_count_seen + 1):
            _diag["pps_count_jump"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": system_time_z(),
                "reason": "pps_count_jump",
                "pps_count": k,
                "prev_pps_count": int(_last_pps_count_seen),
                "jump": int(k - _last_pps_count_seen),
            }

    _last_pps_count_seen = k
    return k


# ---------------------------------------------------------------------
# Buffer management
# ---------------------------------------------------------------------


def _buffer_insert(pps_count: int, capture: Dict[str, Any]) -> None:
    """
    Insert a capture into the buffer, keyed by pps_count.
    Evicts oldest entries if buffer exceeds BUFFER_MAX_SIZE.
    Called under _state_lock.  Only called when _running is True.
    """
    _capture_buffer[pps_count] = capture
    _diag["buffer_inserts"] += 1

    size = len(_capture_buffer)
    _diag["buffer_size_current"] = size
    if size > _diag["buffer_size_max_seen"]:
        _diag["buffer_size_max_seen"] = size

    # Evict oldest if over limit
    while len(_capture_buffer) > BUFFER_MAX_SIZE:
        evicted_pps, evicted_cap = _capture_buffer.popitem(last=False)
        _diag["buffer_evictions"] += 1
        _diag["last_eviction"] = {
            "ts_utc": system_time_z(),
            "evicted_pps_count": evicted_pps,
            "buffer_size": len(_capture_buffer),
        }
        logging.warning(
            "⚠️ [pitimer] buffer eviction: pps_count=%d (buffer at max=%d)",
            evicted_pps, BUFFER_MAX_SIZE,
        )


def _buffer_pop(pps_count: int) -> Optional[Dict[str, Any]]:
    """
    Remove and return a capture by pps_count, or None if not found.
    Called under _state_lock.
    """
    cap = _capture_buffer.pop(pps_count, None)
    if cap is not None:
        _diag["buffer_hits"] += 1
    else:
        _diag["buffer_misses"] += 1
    _diag["buffer_size_current"] = len(_capture_buffer)
    return cap


def _buffer_peek(pps_count: int) -> Optional[Dict[str, Any]]:
    """
    Return a COPY of the capture for pps_count WITHOUT removing it.
    Called under _state_lock.
    """
    cap = _capture_buffer.get(pps_count)
    if cap is not None:
        _diag["buffer_peeks"] += 1
        return dict(cap)
    else:
        _diag["buffer_misses"] += 1
    return None


# ---------------------------------------------------------------------
# Capture loop thread
# ---------------------------------------------------------------------


def _capture_loop() -> None:
    """
    Blocking capture loop: one call to _capture_one() per PPS edge.
    Each call blocks ~1s then returns exactly one capture.

    When RUNNING: captures are assigned pps_count, stored in buffer,
    and pi_ns is computed if epoch is set.

    When STOPPED: captures still update delta tracking and _last_capture
    for diagnostics, but are not buffered or assigned pps_count.
    The loop is calm — no warnings, no eviction concern.
    """
    global _last_capture, _capture_seq, _last_corrected

    logging.info("🚀 [pitimer] capture loop started — mode=%s", _capture_mode)

    while True:
        try:
            cap = _capture_one()
            _diag["captures_total"] += 1
            _capture_seq += 1

            cap["seq"] = _capture_seq

            # Delta from previous capture (always tracked regardless of state)
            corrected = cap["corrected"]
            if _last_corrected is not None:
                delta = corrected - _last_corrected
                residual = delta - PI_TIMER_FREQ
                residual_ns = residual * NS_PER_TICK
                cap["delta"] = delta
                cap["residual"] = residual
                cap["residual_ns"] = round(residual_ns, 3)
            else:
                cap["delta"] = None
                cap["residual"] = None
                cap["residual_ns"] = None
            _last_corrected = corrected

            # Timestamp and edge source (always set)
            time_key = system_time_z()
            cap["gnss_time"] = time_key
            cap["edge_source"] = cap.get("capture_mode", "loop").upper()

            _diag["last_time_key"] = time_key
            _diag["last_seq"] = _capture_seq

            with _state_lock:
                if not _running:
                    # STOPPED: update last_capture for REPORT visibility
                    # but do NOT buffer, do NOT assign pps_count.
                    _last_capture = cap
                    _diag["captures_idle"] += 1
                    continue

                if not _pps_armed:
                    # RUNNING but not yet armed (should not normally happen
                    # since START arms, but defensive)
                    _last_capture = cap
                    _diag["captures_idle"] += 1
                    continue

                # RUNNING + armed: assign pps_count, compute pi_ns, buffer
                pps_count = _assign_pps_count()
                cap["pps_count"] = int(pps_count)

                # Compute pi_ns if epoch is set
                pi_ns = _compute_pi_ns(corrected)
                cap["pi_ns"] = pi_ns

                # Store in buffer and update latest reference
                _buffer_insert(pps_count, cap)
                _last_capture = cap

        except Exception as e:
            _diag["capture_exceptions"] += 1
            _diag["last_capture_exception"] = {
                "ts_utc": system_time_z(),
                "exception": str(e),
            }
            logging.exception("💥 [pitimer] capture error — retrying in 1s")
            time.sleep(1)


# ---------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------


def cmd_start(args: Optional[dict]) -> Dict[str, Any]:
    """
    START(pps_count, [pi_tick_epoch])

    Transition to RUNNING state:
      • Clears buffer and resets pps_count tracking.
      • Arms pps_count — next capture will be labeled pps_count==k.
      • Optionally sets epoch for pi_ns computation.
      • If epoch is not provided, CLOCKS must call SET_EPOCH separately.

    Idempotent: calling START while already RUNNING restarts cleanly
    (stops, clears, re-arms).
    """
    global _running, _pps_armed, _pps_next_count, _last_pps_count_seen
    global _last_capture, _pi_tick_epoch, _epoch_set

    if not args or "pps_count" not in args:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    try:
        k = int(args["pps_count"])
    except Exception:
        return {"success": False, "message": "BAD", "payload": {"error": "invalid pps_count"}}

    if k < 0:
        return {"success": False, "message": "BAD", "payload": {"error": "pps_count must be >= 0"}}

    # Optional epoch
    epoch = args.get("pi_tick_epoch")
    has_epoch = epoch is not None
    if has_epoch:
        try:
            epoch = int(epoch)
        except (ValueError, TypeError):
            return {"success": False, "message": "BAD", "payload": {"error": "invalid pi_tick_epoch"}}

    with _state_lock:
        _running = True
        _pps_armed = True
        _pps_next_count = int(k)
        _last_pps_count_seen = None
        _last_capture = None
        _capture_buffer.clear()

        if has_epoch:
            _pi_tick_epoch = epoch
            _epoch_set = True
        else:
            # Clear stale epoch from prior campaign
            _pi_tick_epoch = 0
            _epoch_set = False

    _diag["starts"] += 1
    _diag["buffer_size_current"] = 0
    _diag["last_start"] = {
        "ts_utc": system_time_z(),
        "pps_count": int(k),
        "pi_tick_epoch": epoch if has_epoch else None,
        "epoch_provided": has_epoch,
    }
    _diag["last_set_event"] = {
        "ts_utc": system_time_z(),
        "event": "start",
        "value": int(k),
    }

    if has_epoch:
        _diag["epoch_set_requests"] += 1
        _diag["epoch_set_applied"] += 1
        _diag["epoch_value"] = epoch

    logging.info(
        "▶️ [pitimer] START: pps_count=%d epoch=%s (buffer cleared, RUNNING)",
        k,
        str(epoch) if has_epoch else "deferred",
    )

    payload: Dict[str, Any] = {"pps_count": int(k), "running": True}
    if has_epoch:
        payload["pi_tick_epoch"] = epoch

    return {"success": True, "message": "OK", "payload": payload}


def cmd_stop(_: Optional[dict]) -> Dict[str, Any]:
    """
    STOP

    Transition to STOPPED state:
      • Stops buffering — captures continue but are not stored.
      • Clears buffer and epoch.
      • Delta tracking continues uninterrupted.

    Idempotent: calling STOP while already STOPPED is a no-op success.
    """
    global _running, _pps_armed, _pps_next_count, _last_pps_count_seen
    global _pi_tick_epoch, _epoch_set

    with _state_lock:
        was_running = _running
        _running = False
        _pps_armed = False
        _pps_next_count = 0
        _last_pps_count_seen = None
        _capture_buffer.clear()
        _pi_tick_epoch = 0
        _epoch_set = False

    _diag["stops"] += 1
    _diag["buffer_size_current"] = 0
    _diag["last_stop"] = {
        "ts_utc": system_time_z(),
        "was_running": was_running,
    }

    logging.info(
        "⏹️ [pitimer] STOP: buffering disabled (was %s)",
        "RUNNING" if was_running else "already STOPPED",
    )

    return {
        "success": True, "message": "OK",
        "payload": {"running": False, "was_running": was_running},
    }


def cmd_set_pps_count(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_PPS_COUNT(k)

    Arms PITIMER for mandatory pps_count tracking.
      • Next capture will be labeled pps_count==k, then increments each PPS.
      • Buffer is cleared — all prior captures are discarded.
      • Does NOT change RUNNING/STOPPED lifecycle state.
      • Does NOT clear epoch (unlike START which clears stale epoch).

    This is the low-level primitive.  Prefer START for campaign transitions.
    """
    global _pps_armed, _pps_next_count, _last_pps_count_seen, _last_capture

    if not args or "pps_count" not in args:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    try:
        k = int(args["pps_count"])
    except Exception:
        return {"success": False, "message": "BAD", "payload": {"error": "invalid pps_count"}}

    if k < 0:
        return {"success": False, "message": "BAD", "payload": {"error": "pps_count must be >= 0"}}

    with _state_lock:
        _pps_armed = True
        _pps_next_count = int(k)
        _last_pps_count_seen = None
        _last_capture = None
        _capture_buffer.clear()

    _diag["buffer_size_current"] = 0
    _diag["last_set_event"] = {
        "ts_utc": system_time_z(),
        "event": "set_pps_count",
        "value": int(k),
    }

    logging.info("🎯 [pitimer] SET_PPS_COUNT: next capture will be pps_count=%d (buffer cleared)", k)

    return {"success": True, "message": "OK", "payload": {"pps_count": int(k)}}


def cmd_set_epoch(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_EPOCH(pi_tick_epoch)

    Sets the Pi tick epoch for campaign-relative pi_ns computation.
    All subsequent captures will have pi_ns = (corrected - epoch) * 1e9 / freq.

    Called by CLOCKS after the sync fragment arrives (when epoch was not
    provided in START).
    """
    global _pi_tick_epoch, _epoch_set

    if not args or "pi_tick_epoch" not in args:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pi_tick_epoch"}}

    try:
        epoch = int(args["pi_tick_epoch"])
    except (ValueError, TypeError):
        return {"success": False, "message": "BAD", "payload": {"error": "invalid pi_tick_epoch"}}

    _pi_tick_epoch = epoch
    _epoch_set = True

    _diag["epoch_set_requests"] += 1
    _diag["epoch_set_applied"] += 1
    _diag["epoch_value"] = epoch

    logging.info("📐 [pitimer] epoch set: pi_tick_epoch=%d", epoch)

    return {
        "success": True, "message": "OK",
        "payload": {"pi_tick_epoch": epoch},
    }


def cmd_get_capture(args: Optional[dict]) -> Dict[str, Any]:
    """
    GET_CAPTURE(pps_count=N, [peek=false])

    Fetch a capture for a specific pps_count from the buffer.

    Default (peek=false):
      Returns and REMOVES the capture from the buffer.
      This is the normal consumption path used by the CLOCKS processor
      thread when building TIMEBASE records.

    Peek mode (peek=true):
      Returns a COPY of the capture WITHOUT removing it from the buffer.
      The capture remains available for subsequent GET_CAPTURE (consume)
      or peek calls.  This is used by CLOCKS during START/RECOVER epoch
      computation to read the corrected value at the sync edge without
      stealing it from the processor thread.

      This eliminates the epoch race condition where _pitimer_report()
      could return a later edge's corrected value if PPS edges advanced
      between the sync fragment arriving and the REPORT IPC completing.

    Returns success=false with message="NOT_FOUND" if the pps_count
    is not in the buffer.
    """
    if not args or "pps_count" not in args:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    try:
        target = int(args["pps_count"])
    except (ValueError, TypeError):
        return {"success": False, "message": "BAD", "payload": {"error": "invalid pps_count"}}

    # Peek mode: return copy without removing
    peek = False
    peek_raw = args.get("peek")
    if peek_raw is not None:
        if isinstance(peek_raw, bool):
            peek = peek_raw
        elif isinstance(peek_raw, str):
            peek = peek_raw.lower() in ("true", "1", "yes")
        else:
            peek = bool(peek_raw)

    with _state_lock:
        if peek:
            cap = _buffer_peek(target)
        else:
            cap = _buffer_pop(target)
        buf_keys = list(_capture_buffer.keys())

    if cap is None:
        return {
            "success": False, "message": "NOT_FOUND",
            "payload": {
                "pps_count": target,
                "peek": peek,
                "running": _running,
                "buffer_size": len(buf_keys),
                "buffer_range": [buf_keys[0], buf_keys[-1]] if buf_keys else None,
                "buffer_keys": buf_keys[:10],  # first 10 for diagnostics
            },
        }

    return {"success": True, "message": "OK", "payload": cap}


def cmd_set_mode(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_MODE(mode=loop|pps)

    Dynamically switch capture mode.  Takes effect on the next capture
    cycle (~1s).  The capture loop, pps_count sequence, and delta
    tracking are uninterrupted.
    """
    global _capture_mode, _pps_fd

    if not args or "mode" not in args:
        return {"success": False, "message": "BAD", "payload": {"error": "missing mode"}}

    new_mode = str(args["mode"]).lower().strip()
    if new_mode not in ("loop", "pps"):
        return {
            "success": False, "message": "BAD",
            "payload": {"error": f"invalid mode '{new_mode}' — must be 'loop' or 'pps'"},
        }

    # Pre-flight check for PPS mode
    if new_mode == "pps":
        if _pps_lib is None:
            return {
                "success": False, "message": "BAD",
                "payload": {"error": "PPS helper library not loaded — PPS mode unavailable"},
            }
        if not os.path.exists(PPS_DEVICE):
            return {
                "success": False, "message": "BAD",
                "payload": {"error": f"PPS device {PPS_DEVICE} not found — is pps_gpio loaded?"},
            }
        if _pps_fd is None:
            try:
                _pps_fd = _open_pps_device()
            except OSError as e:
                return {
                    "success": False, "message": "BAD",
                    "payload": {"error": f"cannot open {PPS_DEVICE}: {e}"},
                }

    with _capture_mode_lock:
        old_mode = _capture_mode
        _capture_mode = new_mode

    _diag["mode_changes"] += 1
    _diag["last_mode_change"] = {
        "ts_utc": system_time_z(),
        "old_mode": old_mode,
        "new_mode": new_mode,
    }

    logging.info("🔄 [pitimer] capture mode: %s → %s", old_mode, new_mode)

    return {
        "success": True, "message": "OK",
        "payload": {"mode": new_mode, "previous": old_mode},
    }


def cmd_get_mode(_: Optional[dict]) -> Dict[str, Any]:
    """GET_MODE — return the current capture mode."""
    with _capture_mode_lock:
        mode = _capture_mode

    return {
        "success": True, "message": "OK",
        "payload": {
            "mode": mode,
            "pps_device": PPS_DEVICE,
            "pps_device_exists": os.path.exists(PPS_DEVICE),
            "pps_fd_open": _pps_fd is not None,
        },
    }


def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    """
    REPORT

    Returns the latest capture (non-destructive) plus buffer state.
    Works in both RUNNING and STOPPED modes — always has the most
    recent capture for diagnostics.
    """
    with _state_lock:
        running = bool(_running)
        armed = bool(_pps_armed)
        cap = dict(_last_capture) if _last_capture else None
        next_pps = int(_pps_next_count)
        buf_keys = list(_capture_buffer.keys())

    with _capture_mode_lock:
        mode = _capture_mode

    if running:
        state = "OK" if cap else "WAITING"
    else:
        state = "STOPPED"

    payload: Dict[str, Any] = {
        "state": state,
        "running": running,
        "capture_mode": mode,
        "pps_armed": armed,
        "epoch_set": _epoch_set,
        "pi_tick_epoch": _pi_tick_epoch if _epoch_set else None,
        "next_pps_count": next_pps if running else None,
        "freq": PI_TIMER_FREQ,
        "ns_per_tick": round(NS_PER_TICK, 3),
        "buffer": {
            "size": len(buf_keys),
            "range": [buf_keys[0], buf_keys[-1]] if buf_keys else None,
            "keys": buf_keys[:20],  # first 20 for diagnostics
            "max_size": BUFFER_MAX_SIZE,
        },
        "diag": {
            "captures_total": _diag["captures_total"],
            "captures_idle": _diag["captures_idle"],
            "capture_exceptions": _diag["capture_exceptions"],
            "buffer_inserts": _diag["buffer_inserts"],
            "buffer_evictions": _diag["buffer_evictions"],
            "buffer_hits": _diag["buffer_hits"],
            "buffer_peeks": _diag["buffer_peeks"],
            "buffer_misses": _diag["buffer_misses"],
            "buffer_size_max_seen": _diag["buffer_size_max_seen"],
            "pps_count_seen": _diag["pps_count_seen"],
            "pps_count_repeat": _diag["pps_count_repeat"],
            "pps_count_regress": _diag["pps_count_regress"],
            "pps_count_jump": _diag["pps_count_jump"],
            "epoch_set_requests": _diag["epoch_set_requests"],
            "epoch_set_applied": _diag["epoch_set_applied"],
            "last_seq": _diag["last_seq"],
            "last_time_key": _diag["last_time_key"],
            "last_pps_count": _diag["last_pps_count"],
            "starts": _diag["starts"],
            "stops": _diag["stops"],
            "mode_changes": _diag["mode_changes"],
            "last_mode_change": _diag.get("last_mode_change", {}),
        },
        "last_start": _diag.get("last_start", {}),
        "last_stop": _diag.get("last_stop", {}),
        "last_set_event": _diag.get("last_set_event", {}),
        "last_pps_count_anomaly": _diag.get("last_pps_count_anomaly", {}),
        "last_capture_exception": _diag.get("last_capture_exception", {}),
        "last_eviction": _diag.get("last_eviction", {}),
    }

    if cap:
        payload.update(
            {
                "gnss_time": cap.get("gnss_time"),
                "pps_count": cap.get("pps_count"),
                "seq": cap.get("seq"),
                "counter": cap.get("counter"),
                "corrected": cap.get("corrected"),
                "pi_ns": cap.get("pi_ns"),
                "detect_ns": cap.get("detect_ns"),
                "correction_ticks": cap.get("correction_ticks"),
                "edge_source": cap.get("edge_source"),
                "delta": cap.get("delta"),
                "residual": cap.get("residual"),
                "residual_ns": cap.get("residual_ns"),
                "chrony_ns": cap.get("chrony_ns"),
                "chrony_sec": cap.get("chrony_sec"),
                "chrony_nsec": cap.get("chrony_nsec"),
                "capture_mode": cap.get("capture_mode"),
                "pps_seq": cap.get("pps_seq"),
            }
        )

    return {"success": True, "message": "OK", "payload": payload}


def cmd_ns(_: Optional[dict]) -> Dict[str, Any]:
    """
    NS — return the current chrony-disciplined time as ISO8601 with nanosecond precision.
    """
    ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
    sec, frac = divmod(ns, NS_PER_SECOND)
    from datetime import datetime, timezone
    dt = datetime.fromtimestamp(sec, tz=timezone.utc)
    iso = dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%09dZ" % frac

    return {"success": True, "message": "OK", "payload": {"ns": ns, "iso": iso}}


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "SET_PPS_COUNT": cmd_set_pps_count,
    "SET_EPOCH": cmd_set_epoch,
    "GET_CAPTURE": cmd_get_capture,
    "REPORT": cmd_report,
    "NS": cmd_ns,
    "SET_MODE": cmd_set_mode,
    "GET_MODE": cmd_get_mode,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------


def run() -> None:
    global _lib, _cntvct_lib, _pps_lib

    setup_logging()

    logging.info(
        "⏱️ [pitimer] starting — Pi Clock Authority (v3 START/STOP lifecycle). "
        "freq=%d Hz, %.3f ns/tick. "
        "STOPPED at boot — capture loop runs, buffering disabled. "
        "START(pps_count, [epoch]) begins buffering. "
        "STOP returns to idle. "
        "SET_PPS_COUNT and SET_EPOCH available as independent primitives. "
        "Captures buffered by pps_count (dynamic, max=%d). "
        "GET_CAPTURE consumes by pps_count identity (peek=true for non-destructive). "
        "Default mode: LOOP. "
        "Commands: START, STOP, SET_PPS_COUNT, SET_EPOCH, GET_CAPTURE, "
        "REPORT, NS, SET_MODE, GET_MODE.",
        PI_TIMER_FREQ,
        NS_PER_TICK,
        BUFFER_MAX_SIZE,
    )

    # Load libppscapture for LOOP mode
    if not os.path.isfile(PPSCAPTURE_LIB):
        logging.error(
            "❌ [pitimer] libppscapture.so not found at %s — run build-native",
            PPSCAPTURE_LIB,
        )
        return

    _lib = _load_lib()
    logging.info("✅ [pitimer] loaded %s (LOOP mode ready)", PPSCAPTURE_LIB)

    # Load libcntvct.so for PPS mode counter reads
    if os.path.isfile(CNTVCT_LIB):
        try:
            _cntvct_lib = _load_cntvct_lib()
            logging.info("✅ [pitimer] loaded %s (PPS mode CNTVCT ready)", CNTVCT_LIB)
        except Exception as e:
            logging.warning("⚠️ [pitimer] could not load %s: %s", CNTVCT_LIB, e)
    else:
        logging.info("ℹ️ [pitimer] %s not found — PPS mode CNTVCT unavailable", CNTVCT_LIB)

    # Load libppsfetch.so for PPS mode ioctl
    if os.path.isfile(PPSFETCH_LIB):
        try:
            _pps_lib = _load_pps_lib()
            logging.info("✅ [pitimer] loaded %s (PPS mode ioctl ready)", PPSFETCH_LIB)
        except Exception as e:
            logging.warning("⚠️ [pitimer] could not load %s: %s", PPSFETCH_LIB, e)
    else:
        logging.info("ℹ️ [pitimer] %s not found — PPS mode ioctl unavailable", PPSFETCH_LIB)

    # Check PPS device availability
    if os.path.exists(PPS_DEVICE):
        logging.info("✅ [pitimer] PPS device %s available", PPS_DEVICE)
    else:
        logging.info("ℹ️ [pitimer] PPS device %s not found — PPS mode requires pps_gpio", PPS_DEVICE)

    threading.Thread(
        target=_capture_loop,
        daemon=True,
        name="pitimer-capture",
    ).start()

    server_setup(
        subsystem="PITIMER",
        commands=COMMANDS,
    )


if __name__ == "__main__":
    run()