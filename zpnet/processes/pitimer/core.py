"""
ZPNet PITIMER Process — Pi Arch Timer PPS Capture (MINIMAL SURFACE)

Intent (v2026-02-26+):
  PITIMER is a single-edge capture service with a strict identity contract.

  Uses a native C shared library (libppscapture.so) via ctypes to detect
  second boundaries by polling clock_gettime(CLOCK_REALTIME). One blocking
  call per PPS edge — no subprocess, no stdout parsing, no ambiguity.

Contract:
  • CLOCKS sets PITIMER's pps_count via SET_PPS_COUNT(k).
  • The next PPS capture MUST be labeled pps_count==k, then PITIMER increments by 1 each PPS.
  • PITIMER retains ONLY the most recent capture (no history, no ring buffers).

Command surface (minimal):
  • SET_PPS_COUNT
  • REPORT
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

PI_TIMER_FREQ = 54_000_000
NS_PER_TICK = 1e9 / PI_TIMER_FREQ

# ---------------------------------------------------------------------
# Native library interface
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


def _capture_one() -> Dict[str, Any]:
    """Block until next second boundary, return capture dict."""
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

    # Chrony-disciplined realtime: full nanoseconds since epoch
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
    }


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

    logging.info("🚀 [pitimer] capture loop started — calling libppscapture directly")

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
            cap["edge_source"] = "TDC" if cap["detect_ns"] >= 0 else "RAW"

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
                    "⏱️ [pitimer] seq=%d pps_count=%s corrected=%d detect=%d corr=%d time=%s",
                    _capture_seq,
                    str(cap.get("pps_count", "—")),
                    corrected,
                    cap["detect_ns"],
                    cap["correction_ticks"],
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
# Command handlers (MINIMAL)
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


def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    """
    REPORT

    Returns the latest capture plus current pps_count mode state.
    """
    with _state_lock:
        cap = dict(_last_capture) if _last_capture else None
        enabled = bool(_pps_count_enabled)
        next_pps = int(_pps_next_count) if enabled else None

    payload: Dict[str, Any] = {
        "state": "OK" if cap else "WAITING",
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
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------


def run() -> None:
    global _lib

    setup_logging()

    logging.info(
        "⏱️ [pitimer] starting — Pi arch timer PPS capture via libppscapture (ctypes). "
        "freq=%d Hz, %.3f ns/tick. "
        "Single-capture model (no subprocess, no parsing, no rings). "
        "Commands: SET_PPS_COUNT, REPORT.",
        PI_TIMER_FREQ,
        NS_PER_TICK,
    )

    if not os.path.isfile(PPSCAPTURE_LIB):
        logging.error(
            "❌ [pitimer] libppscapture.so not found at %s — run build-native",
            PPSCAPTURE_LIB,
        )
        return

    _lib = _load_lib()
    logging.info("✅ [pitimer] loaded %s", PPSCAPTURE_LIB)

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