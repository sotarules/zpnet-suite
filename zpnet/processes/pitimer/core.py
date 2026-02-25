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
    output line by line, and stashes captures in ring buffers
  • CLOCKS retrieves captures either by time key (legacy) or by
    campaign-scoped pps_count (new, recommended)

NEW (v2026-02-25 pps_count identity channel):
  • RESET_PPS_COUNT command:
      - Arms a reset such that the *next* detected PPS capture becomes pps_count = 0
      - Subsequent captures increment pps_count monotonically (0,1,2,...)
      - This is campaign-scoped identity, designed to match Teensy "teensy_pps_count"
  • A dedicated ring buffer indexed by pps_count is maintained
      - Enables deterministic, event-identity correlation
      - Avoids "which UTC second did I label this?" failure modes

Diagnostics:
  • PITIMER_INFO exposes:
      - subprocess lifecycle counters
      - stdout ingestion / parse counters
      - time-key overwrite counters
      - pps_count mode state + anomalies (repeat / jump / regress)
      - ring snapshots (time-key ring + pps_count ring)
      - last anomaly snapshots
"""

from __future__ import annotations

import collections
import logging
import os
import subprocess
import threading
import time
from typing import Any, Dict, Optional, List

from zpnet.processes.processes import server_setup
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import system_time_z

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

PPSPOLL_BINARY = os.environ.get(
    "PPSPOLL_BINARY",
    "/home/mule/zpnet/zpnet/native/pps_poll/pps_poll",
)

ISOLATED_CORE = 3

PI_TIMER_FREQ = 54_000_000
NS_PER_TICK = 1e9 / PI_TIMER_FREQ

# Time-key ring capacity (legacy correlation window)
RING_BUFFER_CAPACITY = 5

# PPS-count ring capacity (campaign window)
# If CLOCKS polls within tens/hundreds of ms, 16–64 is ample.
# Set higher if you anticipate pauses or heavy startup contention.
PPS_COUNT_RING_CAPACITY = 64

# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last anomaly snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Subprocess lifecycle
    "ppspoll_spawns": 0,
    "ppspoll_exits": 0,
    "ppspoll_last_exit_code": None,

    # Stdout ingestion
    "stdout_lines_total": 0,
    "stdout_lines_decoded": 0,
    "stdout_lines_parsed_ok": 0,
    "stdout_lines_parsed_none": 0,   # headers / separators / blanks
    "stdout_parse_exceptions": 0,

    # Time-key ring behavior
    "ring_put_total": 0,
    "ring_key_overwrite": 0,
    "ring_evictions": 0,

    # PPS-count mode
    "pps_count_enabled": False,
    "pps_count_reset_armed": False,
    "pps_count_reset_requests": 0,
    "pps_count_reset_applied": 0,

    # PPS-count ring behavior
    "pps_count_put_total": 0,
    "pps_count_evictions": 0,
    "pps_count_lookup_hits": 0,
    "pps_count_lookup_misses": 0,

    # PPS-count continuity
    "pps_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_regress": 0,
    "pps_count_jump": 0,

    # Last-seen values
    "last_time_key": None,
    "last_ppspoll_seq": None,          # seq as printed by pps_poll (free-running)
    "last_pps_count": None,            # campaign-scoped pps_count

    # Last anomaly snapshots
    "last_key_overwrite": {},
    "last_pps_count_anomaly": {},
    "last_parse_exception": {},
    "last_reset_event": {},
}

# ---------------------------------------------------------------------
# Time-key ring buffer (legacy)
# ---------------------------------------------------------------------

_ring: collections.OrderedDict[str, Dict[str, Any]] = collections.OrderedDict()
_ring_lock = threading.Lock()
_last_capture: Optional[Dict[str, Any]] = None

# ---------------------------------------------------------------------
# PPS-count ring buffer (campaign identity)
# ---------------------------------------------------------------------

_pps_ring: collections.OrderedDict[int, Dict[str, Any]] = collections.OrderedDict()
_pps_lock = threading.Lock()

# PPS-count control state
_pps_count_enabled: bool = False
_pps_reset_armed: bool = False
_pps_next_count: int = 0
_last_pps_count_seen: Optional[int] = None

# ---------------------------------------------------------------------
# Ring helpers
# ---------------------------------------------------------------------

def _ring_put(time_key: str, capture: Dict[str, Any]) -> None:
    global _last_capture

    _diag["ring_put_total"] += 1

    with _ring_lock:
        if time_key in _ring:
            _diag["ring_key_overwrite"] += 1
            prev = _ring.get(time_key) or {}
            _diag["last_key_overwrite"] = {
                "ts_utc": system_time_z(),
                "time_key": time_key,
                "prev_ppspoll_seq": prev.get("seq"),
                "new_ppspoll_seq": capture.get("seq"),
                "prev_corrected": prev.get("corrected"),
                "new_corrected": capture.get("corrected"),
            }
            del _ring[time_key]

        _ring[time_key] = capture
        _last_capture = capture

        while len(_ring) > RING_BUFFER_CAPACITY:
            _ring.popitem(last=False)
            _diag["ring_evictions"] += 1


def _ring_get(time_key: str) -> Optional[Dict[str, Any]]:
    with _ring_lock:
        return _ring.get(time_key)


def _ring_latest() -> Optional[Dict[str, Any]]:
    with _ring_lock:
        return _last_capture


def _ring_keys() -> list:
    with _ring_lock:
        return list(_ring.keys())


def _ring_size() -> int:
    with _ring_lock:
        return len(_ring)


def _ring_snapshot_rows() -> List[Dict[str, Any]]:
    with _ring_lock:
        rows: List[Dict[str, Any]] = []
        for k, v in _ring.items():
            rows.append({
                "gnss_time": k,
                "ppspoll_seq": v.get("seq"),
                "corrected": v.get("corrected"),
                "detect_ns": v.get("detect_ns"),
                "correction_ticks": v.get("correction_ticks"),
            })
        return rows


def _pps_ring_put(pps_count: int, capture: Dict[str, Any]) -> None:
    _diag["pps_count_put_total"] += 1

    with _pps_lock:
        # Overwrite is allowed (shouldn't happen normally) — deterministic last-write-wins
        if pps_count in _pps_ring:
            # Treat overwrite as an anomaly worth recording
            prev = _pps_ring.get(pps_count) or {}
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": system_time_z(),
                "reason": "pps_count_overwrite",
                "pps_count": pps_count,
                "prev_ppspoll_seq": prev.get("seq"),
                "new_ppspoll_seq": capture.get("seq"),
                "prev_corrected": prev.get("corrected"),
                "new_corrected": capture.get("corrected"),
            }
            del _pps_ring[pps_count]

        _pps_ring[pps_count] = capture

        while len(_pps_ring) > PPS_COUNT_RING_CAPACITY:
            _pps_ring.popitem(last=False)
            _diag["pps_count_evictions"] += 1


def _pps_ring_get(pps_count: int) -> Optional[Dict[str, Any]]:
    with _pps_lock:
        return _pps_ring.get(pps_count)


def _pps_ring_keys() -> List[int]:
    with _pps_lock:
        return list(_pps_ring.keys())


def _pps_ring_size() -> int:
    with _pps_lock:
        return len(_pps_ring)


def _pps_ring_snapshot_rows(limit: int = 32) -> List[Dict[str, Any]]:
    """
    Return a compact snapshot of the most recent pps_count captures.
    """
    with _pps_lock:
        rows: List[Dict[str, Any]] = []
        items = list(_pps_ring.items())[-limit:]
        for k, v in items:
            rows.append({
                "pps_count": k,
                "ppspoll_seq": v.get("seq"),
                "time_key": v.get("gnss_time"),
                "corrected": v.get("corrected"),
                "detect_ns": v.get("detect_ns"),
                "correction_ticks": v.get("correction_ticks"),
            })
        return rows

# ---------------------------------------------------------------------
# Capture parser
# ---------------------------------------------------------------------

def _parse_capture_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse a single output line from pps_poll.

    Enhanced format (>= 9 fields):
      seq counter corrected delta residual resid_ns detect_ns corr_ticks rt_ns

    Legacy format (6 fields):
      seq counter delta residual resid_ns detect_ns
    """
    line = line.strip()
    if not line:
        return None

    paren_idx = line.find("(")
    if paren_idx != -1:
        line = line[:paren_idx].strip()

    parts = line.split()
    if len(parts) < 6:
        return None

    try:
        seq = int(parts[0])
    except ValueError:
        return None

    if len(parts) >= 9:
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

    # Legacy format
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

# ---------------------------------------------------------------------
# PPS-count logic
# ---------------------------------------------------------------------

def _pps_count_on_capture(capture: Dict[str, Any]) -> Optional[int]:
    """
    Determine (and advance) campaign-scoped pps_count for this capture.

    Semantics:
      • pps_count is DISABLED until RESET_PPS_COUNT is called.
      • RESET_PPS_COUNT arms a reset so that the *next* capture becomes pps_count=0.
      • Once enabled, pps_count increments by 1 on every parsed capture.
    """
    global _pps_count_enabled, _pps_reset_armed, _pps_next_count, _last_pps_count_seen

    if not _pps_count_enabled and not _pps_reset_armed:
        return None

    if _pps_reset_armed:
        # Apply reset on this capture
        _pps_count_enabled = True
        _pps_reset_armed = False
        _pps_next_count = 0
        _last_pps_count_seen = None

        _diag["pps_count_reset_applied"] += 1
        _diag["pps_count_enabled"] = True
        _diag["pps_count_reset_armed"] = False
        _diag["last_reset_event"] = {
            "ts_utc": system_time_z(),
            "event": "reset_applied",
            "ppspoll_seq": capture.get("seq"),
            "time_key": capture.get("gnss_time"),
        }

    pps_count = _pps_next_count
    _pps_next_count += 1

    # Continuity bookkeeping (should be strictly increasing by 1)
    _diag["pps_count_seen"] += 1
    _diag["last_pps_count"] = pps_count

    if _last_pps_count_seen is not None:
        if pps_count == _last_pps_count_seen:
            _diag["pps_count_repeat"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": system_time_z(),
                "reason": "pps_count_repeat",
                "pps_count": pps_count,
                "prev_pps_count": _last_pps_count_seen,
                "ppspoll_seq": capture.get("seq"),
                "time_key": capture.get("gnss_time"),
            }
        elif pps_count < _last_pps_count_seen:
            _diag["pps_count_regress"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": system_time_z(),
                "reason": "pps_count_regress",
                "pps_count": pps_count,
                "prev_pps_count": _last_pps_count_seen,
                "ppspoll_seq": capture.get("seq"),
                "time_key": capture.get("gnss_time"),
            }
        elif pps_count > _last_pps_count_seen + 1:
            _diag["pps_count_jump"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": system_time_z(),
                "reason": "pps_count_jump",
                "pps_count": pps_count,
                "prev_pps_count": _last_pps_count_seen,
                "jump": pps_count - _last_pps_count_seen,
                "ppspoll_seq": capture.get("seq"),
                "time_key": capture.get("gnss_time"),
            }

    _last_pps_count_seen = pps_count
    return pps_count

# ---------------------------------------------------------------------
# Capture reader thread
# ---------------------------------------------------------------------

def _capture_reader(proc: subprocess.Popen) -> None:
    """
    Read pps_poll stdout line by line, derive time key, store in rings,
    and (optionally) assign campaign-scoped pps_count.
    """
    for raw_line in iter(proc.stdout.readline, b""):
        _diag["stdout_lines_total"] += 1

        try:
            line = raw_line.decode("utf-8", errors="replace")
            _diag["stdout_lines_decoded"] += 1

            capture = _parse_capture_line(line)
            if capture is None:
                _diag["stdout_lines_parsed_none"] += 1
                continue

            _diag["stdout_lines_parsed_ok"] += 1

            payload = dict(capture)
            payload["freq"] = PI_TIMER_FREQ
            payload["ns_per_tick"] = round(NS_PER_TICK, 3)

            time_key = system_time_z()
            payload["gnss_time"] = time_key

            _diag["last_time_key"] = time_key
            _diag["last_ppspoll_seq"] = payload.get("seq")

            # Store legacy time-key ring
            _ring_put(time_key, payload)

            # Store pps_count ring if enabled/armed
            pps_count = _pps_count_on_capture(payload)
            if pps_count is not None:
                payload["pps_count"] = pps_count
                _pps_ring_put(pps_count, payload)

            # Periodic log
            if payload.get("seq") and int(payload["seq"]) % 60 == 0 and payload.get("delta") is not None:
                logging.info(
                    "⏱️ [pitimer] ppspoll_seq=%d pps_count=%s corrected=%d "
                    "residual=%.1f ns detect=%d ns corr=%d ticks "
                    "time_key=%s ring=%d pps_ring=%d",
                    int(payload["seq"]),
                    str(payload.get("pps_count", "—")),
                    int(payload.get("corrected") or 0),
                    payload.get("residual_ns") or 0.0,
                    int(payload.get("detect_ns") or 0),
                    int(payload.get("correction_ticks") or 0),
                    time_key,
                    _ring_size(),
                    _pps_ring_size(),
                )

        except Exception as e:
            _diag["stdout_parse_exceptions"] += 1
            _diag["last_parse_exception"] = {
                "ts_utc": system_time_z(),
                "exception": str(e),
            }
            logging.exception("⚠️ [pitimer] error parsing pps_poll output")

    logging.warning("⚠️ [pitimer] pps_poll stdout closed")

# ---------------------------------------------------------------------
# Subprocess supervisor
# ---------------------------------------------------------------------

def _spawn_ppspoll() -> subprocess.Popen:
    cmd = [
        "taskset", "-c", str(ISOLATED_CORE),
        PPSPOLL_BINARY,
        "999999999",
    ]

    logging.info("🚀 [pitimer] spawning pps_poll on core %d: %s", ISOLATED_CORE, " ".join(cmd))

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )

    _diag["ppspoll_spawns"] += 1
    return proc


def _supervisor() -> None:
    while True:
        try:
            proc = _spawn_ppspoll()

            reader = threading.Thread(
                target=_capture_reader,
                args=(proc,),
                daemon=True,
                name="pitimer-reader",
            )
            reader.start()

            retcode = proc.wait()
            _diag["ppspoll_exits"] += 1
            _diag["ppspoll_last_exit_code"] = retcode
            logging.warning("⚠️ [pitimer] pps_poll exited with code %d — restarting in 5s", retcode)

        except Exception:
            logging.exception("💥 [pitimer] supervisor error — restarting in 5s")

        time.sleep(5)

# ---------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------

def cmd_report(args: Optional[dict]) -> Dict[str, Any]:
    """
    REPORT — legacy interface.

    With gnss_time:
      returns capture by time key (legacy correlation)

    Without gnss_time:
      returns latest capture and ring keys
    """
    gnss_time = None
    if args:
        gnss_time = args.get("gnss_time")

    if gnss_time is not None:
        capture = _ring_get(gnss_time)
        if capture is None:
            available = _ring_keys()
            logging.warning(
                "⚠️ [pitimer] REPORT: no capture for gnss_time=%s (buffer has %d entries: %s)",
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
        return {"success": True, "message": "OK", "payload": result}

    capture = _ring_latest()
    if capture is None:
        return {"success": True, "message": "OK", "payload": {"state": "WAITING"}}

    result = dict(capture)
    result["buffer_size"] = _ring_size()
    result["buffer_keys"] = _ring_keys()
    return {"success": True, "message": "OK", "payload": result}


def cmd_reset_pps_count(_: Optional[dict]) -> Dict[str, Any]:
    """
    RESET_PPS_COUNT — arm campaign identity reset.

    Semantics:
      • The *next* parsed PPS capture becomes pps_count = 0
      • Subsequent captures increment pps_count monotonically
      • Does not stop pps_poll or affect legacy time-key ring
      • Clears the pps_count ring buffer immediately (fresh campaign view)
    """
    global _pps_reset_armed, _pps_count_enabled, _pps_next_count, _last_pps_count_seen

    _diag["pps_count_reset_requests"] += 1

    with _pps_lock:
        _pps_ring.clear()

    _pps_reset_armed = True
    _pps_count_enabled = False
    _pps_next_count = 0
    _last_pps_count_seen = None

    _diag["pps_count_reset_armed"] = True
    _diag["pps_count_enabled"] = False
    _diag["last_reset_event"] = {
        "ts_utc": system_time_z(),
        "event": "reset_armed",
    }

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "status": "reset_armed",
            "note": "next PPS capture will be pps_count=0",
            "pps_count_ring_cleared": True,
        },
    }


def cmd_report_by_pps_count(args: Optional[dict]) -> Dict[str, Any]:
    """
    REPORT_PPS_COUNT — return capture by campaign pps_count.

    Args:
      { "pps_count": <int> }

    Returns:
      success=False if pps_count mode is not enabled or capture not present.
    """
    if not args or "pps_count" not in args:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    try:
        k = int(args["pps_count"])
    except Exception:
        return {"success": False, "message": "BAD", "payload": {"error": "invalid pps_count"}}

    if not _pps_count_enabled and not _pps_reset_armed:
        return {
            "success": False,
            "message": "BAD",
            "payload": {
                "error": "pps_count not enabled",
                "hint": "call RESET_PPS_COUNT first",
            },
        }

    cap = _pps_ring_get(k)
    if cap is None:
        _diag["pps_count_lookup_misses"] += 1
        return {
            "success": False,
            "message": "BAD",
            "payload": {
                "error": "pps_count not found",
                "pps_count": k,
                "pps_ring_size": _pps_ring_size(),
                "pps_ring_keys_tail": _pps_ring_keys()[-10:],
            },
        }

    _diag["pps_count_lookup_hits"] += 1
    out = dict(cap)
    out["pps_ring_size"] = _pps_ring_size()
    return {"success": True, "message": "OK", "payload": out}


def cmd_pitimer_info(_: Optional[dict]) -> Dict[str, Any]:
    """
    PITIMER_INFO — instrumentation snapshot.

    Includes:
      • Monotonic counters + last anomaly snapshots
      • Time-key ring snapshot
      • PPS-count ring snapshot
      • PPS-count mode state
    """
    payload = {
        "diag": dict(_diag),
        "pps_count_mode": {
            "enabled": _pps_count_enabled,
            "reset_armed": _pps_reset_armed,
            "next_pps_count": _pps_next_count if _pps_count_enabled else (0 if _pps_reset_armed else None),
            "pps_ring_size": _pps_ring_size(),
            "pps_ring_keys_tail": _pps_ring_keys()[-10:],
        },
        "ring_time_key": {
            "buffer_size": _ring_size(),
            "buffer_keys": _ring_keys(),
            "buffer": _ring_snapshot_rows(),
        },
        "ring_pps_count": {
            "buffer_size": _pps_ring_size(),
            "buffer": _pps_ring_snapshot_rows(limit=32),
        },
    }
    return {"success": True, "message": "OK", "payload": payload}


COMMANDS = {
    "REPORT": cmd_report,
    "RESET_PPS_COUNT": cmd_reset_pps_count,
    "REPORT_PPS_COUNT": cmd_report_by_pps_count,
    "PITIMER_INFO": cmd_pitimer_info,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    logging.info(
        "⏱️ [pitimer] starting — Pi arch timer PPS capture via chrony polling. "
        "freq=%d Hz, %.3f ns/tick, isolated core %d. "
        "TDC correction enabled (detect_ns → correction_ticks). "
        "Time-key ring capacity=%d. pps_count ring capacity=%d. "
        "PPS-count identity available (RESET_PPS_COUNT + REPORT_PPS_COUNT). "
        "Instrumentation enabled (PITIMER_INFO).",
        PI_TIMER_FREQ,
        NS_PER_TICK,
        ISOLATED_CORE,
        RING_BUFFER_CAPACITY,
        PPS_COUNT_RING_CAPACITY,
    )

    if not os.path.isfile(PPSPOLL_BINARY):
        logging.error(
            "❌ [pitimer] pps_poll binary not found at %s — build with: gcc -O2 -o pps_poll pps_poll.c -lm",
            PPSPOLL_BINARY,
        )
        return

    threading.Thread(
        target=_supervisor,
        daemon=True,
        name="pitimer-supervisor",
    ).start()

    server_setup(
        subsystem="PITIMER",
        commands=COMMANDS,
    )


if __name__ == "__main__":
    run()