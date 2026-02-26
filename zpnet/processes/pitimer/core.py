"""
ZPNet PITIMER Process — Pi Arch Timer PPS Capture (MINIMAL SURFACE)

Intent (v2026-02-26+):
  PITIMER is a single-edge capture service with a strict identity contract.

  Supports two capture modes:

    LOOP (default):
      Uses a native C shared library (libppscapture.so) via ctypes to detect
      second boundaries by polling clock_gettime(CLOCK_REALTIME). One blocking
      call per PPS edge — no subprocess, no stdout parsing, no ambiguity.

    PPS:
      Uses the kernel PPS subsystem (/dev/pps0) via ioctl(PPS_FETCH) to obtain
      CLOCK_REALTIME timestamps captured at the actual GPIO interrupt — the true
      PPS edge.  CNTVCT_EL0 is read immediately after the ioctl returns.

      This mode eliminates the tautology in LOOP mode where CLOCK_REALTIME's
      own second rollover is used as the PPS surrogate, making the realtime
      measurement self-referential.

      PPS mode coexists peacefully with chrony reading /dev/pps0 — PPS_FETCH
      is non-destructive and multiple consumers can read the same timestamps.

Contract:
  • CLOCKS sets PITIMER's pps_count via SET_PPS_COUNT(k).
  • The next PPS capture MUST be labeled pps_count==k, then PITIMER increments by 1 each PPS.
  • PITIMER retains ONLY the most recent capture (no history, no ring buffers).

Command surface:
  • SET_PPS_COUNT
  • REPORT
  • NS
  • SET_MODE
  • GET_MODE
"""

from __future__ import annotations

import ctypes
import logging
import os
import threading
import time
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
NS_PER_TICK = 1e9 / PI_TIMER_FREQ

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
    correction_ticks = (dns * PI_TIMER_FREQ) // 1_000_000_000
    corrected = raw - correction_ticks

    # Chrono-disciplined realtime: full nanoseconds since epoch
    chrony_ns = (rt_sec.value * 1_000_000_000) + rt_nsec.value

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

    The kernel timestamps the GPIO interrupt with CLOCK_REALTIME at the
    actual PPS edge — no polling, no tautology.  CNTVCT_EL0 is captured
    immediately after the ioctl returns (microseconds of jitter from
    syscall return latency).
    """
    global _pps_fd

    if _pps_fd is None:
        _pps_fd = _open_pps_device()

    # Block until the next PPS assert edge
    pps = _pps_fetch(_pps_fd)

    # Read CNTVCT_EL0 as close to the edge as we can get
    raw = _read_cntvct()

    # The time elapsed between the actual PPS edge (kernel timestamp)
    # and our CNTVCT_EL0 read is our "detect_ns" equivalent.
    # We know current CLOCK_REALTIME ≈ now, and the PPS timestamp
    # was pps_sec.pps_nsec.  The difference is our latency.
    now_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
    pps_ns = (pps["pps_sec"] * 1_000_000_000) + pps["pps_nsec"]
    detect_ns = now_ns - pps_ns

    # TDC correction: subtract detection latency from counter
    correction_ticks = (detect_ns * PI_TIMER_FREQ) // 1_000_000_000
    corrected = raw - correction_ticks

    # In PPS mode, chrony_ns is the kernel's interrupt-level timestamp
    # at the actual GPIO edge — the ground truth for PiHR.
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
# Diagnostics (monotonic counters + last snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Capture loop
    "captures_total": 0,
    "capture_exceptions": 0,

    # PPS-count control/continuity
    "pps_count_set_requests": 0,
    "pps_count_set_applied": 0,
    "pps_count_enabled": False,
    "pps_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_regress": 0,
    "pps_count_jump": 0,

    # Last-seen values
    "last_time_key": None,
    "last_pps_count": None,
    "last_seq": None,

    # Last anomaly snapshots
    "last_pps_count_anomaly": {},
    "last_set_event": {},
    "last_capture_exception": {},

    # Mode transitions
    "mode_changes": 0,
    "last_mode_change": {},
}

# ---------------------------------------------------------------------
# Single-capture state
# ---------------------------------------------------------------------

_state_lock = threading.Lock()

_last_capture: Optional[Dict[str, Any]] = None
_capture_seq: int = 0

_pps_count_enabled: bool = False
_pps_next_count: int = 0
_last_pps_count_seen: Optional[int] = None

# Delta tracking
_last_corrected: Optional[int] = None

# ---------------------------------------------------------------------
# PPS count assignment
# ---------------------------------------------------------------------


def _assign_pps_count() -> Optional[int]:
    """
    If enabled, assign pps_count for THIS capture and increment for next.
    """
    global _pps_next_count, _last_pps_count_seen

    if not _pps_count_enabled:
        return None

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
# Capture loop thread
# ---------------------------------------------------------------------


def _capture_loop() -> None:
    """
    Blocking capture loop: one call to _capture_one() per PPS edge.
    Each call blocks ~1s then returns exactly one capture.
    """
    global _last_capture, _capture_seq, _last_corrected

    logging.info("🚀 [pitimer] capture loop started — mode=%s", _capture_mode)

    while True:
        try:
            cap = _capture_one()
            _diag["captures_total"] += 1
            _capture_seq += 1

            cap["seq"] = _capture_seq

            # Delta from previous capture
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

            # Timestamp and edge source
            time_key = system_time_z()
            cap["gnss_time"] = time_key
            cap["edge_source"] = cap.get("capture_mode", "loop").upper()

            _diag["last_time_key"] = time_key
            _diag["last_seq"] = _capture_seq

            with _state_lock:
                pps_count = _assign_pps_count()
                if pps_count is not None:
                    cap["pps_count"] = int(pps_count)
                _last_capture = cap

            # Periodic log (every 60 captures)
            if _capture_seq % 60 == 0:
                logging.info(
                    "⏱️ [pitimer] seq=%d pps_count=%s corrected=%d detect=%d corr=%d mode=%s time=%s",
                    _capture_seq,
                    str(cap.get("pps_count", "—")),
                    corrected,
                    cap["detect_ns"],
                    cap["correction_ticks"],
                    cap.get("capture_mode", "loop"),
                    time_key,
                )

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


def cmd_set_pps_count(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_PPS_COUNT(k)

    Semantics:
      • Enables pps_count mode.
      • Next capture will be labeled pps_count==k, then increments each PPS.
    """
    global _pps_count_enabled, _pps_next_count, _last_pps_count_seen

    if not args or "pps_count" not in args:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    try:
        k = int(args["pps_count"])
    except Exception:
        return {"success": False, "message": "BAD", "payload": {"error": "invalid pps_count"}}

    if k < 0:
        return {"success": False, "message": "BAD", "payload": {"error": "pps_count must be >= 0"}}

    _diag["pps_count_set_requests"] += 1

    with _state_lock:
        _pps_count_enabled = True
        _pps_next_count = int(k)
        _last_pps_count_seen = None

    _diag["pps_count_set_applied"] += 1
    _diag["pps_count_enabled"] = True
    _diag["last_set_event"] = {
        "ts_utc": system_time_z(),
        "event": "set_pps_count",
        "value": int(k),
    }

    return {"success": True, "message": "OK", "payload": {"pps_count": int(k)}}


def cmd_set_mode(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_MODE(mode=loop|pps)

    Dynamically switch capture mode.  Takes effect on the next capture
    cycle (~1s).  The capture loop, pps_count sequence, and delta
    tracking are uninterrupted.

    Note: switching to PPS mode requires /dev/pps0 to be available
    (pps_gpio kernel module loaded).  If it cannot be opened, the
    mode change is rejected.
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
        # Pre-open the device to fail fast
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

    Returns the latest capture plus current pps_count mode state.
    """
    with _state_lock:
        cap = dict(_last_capture) if _last_capture else None
        enabled = bool(_pps_count_enabled)
        next_pps = int(_pps_next_count) if enabled else None

    with _capture_mode_lock:
        mode = _capture_mode

    payload: Dict[str, Any] = {
        "state": "OK" if cap else "WAITING",
        "capture_mode": mode,
        "pps_count_enabled": enabled,
        "next_pps_count": next_pps,
        "freq": PI_TIMER_FREQ,
        "ns_per_tick": round(NS_PER_TICK, 3),
        "diag": {
            "captures_total": _diag["captures_total"],
            "capture_exceptions": _diag["capture_exceptions"],
            "pps_count_seen": _diag["pps_count_seen"],
            "pps_count_repeat": _diag["pps_count_repeat"],
            "pps_count_regress": _diag["pps_count_regress"],
            "pps_count_jump": _diag["pps_count_jump"],
            "last_seq": _diag["last_seq"],
            "last_time_key": _diag["last_time_key"],
            "last_pps_count": _diag["last_pps_count"],
            "mode_changes": _diag["mode_changes"],
            "last_mode_change": _diag.get("last_mode_change", {}),
        },
        "last_set_event": _diag.get("last_set_event", {}),
        "last_pps_count_anomaly": _diag.get("last_pps_count_anomaly", {}),
        "last_capture_exception": _diag.get("last_capture_exception", {}),
    }

    if cap:
        payload.update(
            {
                "gnss_time": cap.get("gnss_time"),
                "pps_count": cap.get("pps_count"),
                "seq": cap.get("seq"),
                "counter": cap.get("counter"),
                "corrected": cap.get("corrected"),
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

    Calls clock_gettime(CLOCK_REALTIME) directly via Python's time module.
    No PPS edge needed — this is available at any moment.
    """
    ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
    sec, frac = divmod(ns, 1_000_000_000)
    from datetime import datetime, timezone
    dt = datetime.fromtimestamp(sec, tz=timezone.utc)
    iso = dt.strftime("%Y-%m-%dT%H:%M:%S") + ".%09dZ" % frac

    return {"success": True, "message": "OK", "payload": {"ns": ns, "iso": iso}}


COMMANDS = {
    "SET_PPS_COUNT": cmd_set_pps_count,
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
        "⏱️ [pitimer] starting — Pi arch timer PPS capture. "
        "freq=%d Hz, %.3f ns/tick. "
        "Default mode: LOOP (libppscapture). "
        "Commands: SET_PPS_COUNT, REPORT, NS, SET_MODE, GET_MODE.",
        PI_TIMER_FREQ,
        NS_PER_TICK,
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