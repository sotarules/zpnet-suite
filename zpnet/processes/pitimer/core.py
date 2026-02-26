"""
ZPNet PITIMER Process — Pi Arch Timer PPS Capture (MINIMAL SURFACE)

Intent (v2026-02-25+):
  PITIMER is a single-edge capture service with a strict identity contract.

Contract:
  • CLOCKS sets PITIMER's pps_count via SET_PPS_COUNT(k).
  • The next PPS capture MUST be labeled pps_count==k, then PITIMER increments by 1 each PPS.
  • PITIMER retains ONLY the most recent capture (no history, no ring buffers).

Command surface (minimal):
  • SET_PPS_COUNT
  • REPORT

REPORT payload includes:
  • pps_count (for the latest capture, if available)
  • counter, corrected, detect_ns, correction_ticks
  • seq, gnss_time label, edge_source
  • pps_count_mode (enabled + next_pps_count)
  • basic diag counters
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

EDGE_SOURCE_TDC = "TDC"
EDGE_SOURCE_RAW = "RAW"

# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last snapshots)
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
    "stdout_lines_parsed_none": 0,
    "stdout_parse_exceptions": 0,

    # PPS-count control/continuity
    "pps_count_set_requests": 0,
    "pps_count_set_applied": 0,
    "pps_count_enabled": False,
    "pps_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_regress": 0,
    "pps_count_jump": 0,

    # Edge-source instrumentation
    "edge_source_tdc": 0,
    "edge_source_raw": 0,
    "last_edge_source": None,

    # Last-seen values
    "last_ppspoll_seq": None,
    "last_time_key": None,
    "last_pps_count": None,

    # Last anomaly snapshots
    "last_pps_count_anomaly": {},
    "last_parse_exception": {},
    "last_set_event": {},
}

# ---------------------------------------------------------------------
# Single-capture state
# ---------------------------------------------------------------------

_state_lock = threading.Lock()

_last_capture: Optional[Dict[str, Any]] = None

_pps_count_enabled: bool = False
_pps_next_count: int = 0
_last_pps_count_seen: Optional[int] = None

# ---------------------------------------------------------------------
# Capture parser
# ---------------------------------------------------------------------


def _parse_capture_line(line: str) -> Optional[Dict[str, Any]]:
    """
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

    # Enhanced format
    if len(parts) >= 9:
        try:
            counter = int(parts[1])
            corrected = int(parts[2])
            delta_str = parts[3]
            residual_str = parts[4]
            resid_ns_str = parts[5]
            detect_ns = int(parts[6])
            correction_ticks = int(parts[7])
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
    try:
        counter = int(parts[1])
        delta_str = parts[2]
        residual_str = parts[3]
        resid_ns_str = parts[4]
        detect_ns = int(parts[5])
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


def _classify_edge_source(capture: Dict[str, Any]) -> str:
    detect_ns = capture.get("detect_ns")
    if isinstance(detect_ns, int) and detect_ns >= 0:
        return EDGE_SOURCE_TDC
    return EDGE_SOURCE_RAW


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
# Capture reader thread
# ---------------------------------------------------------------------


def _capture_reader(proc: subprocess.Popen) -> None:
    """
    Read pps_poll stdout and retain ONLY the latest capture.
    """
    global _last_capture

    for raw_line in iter(proc.stdout.readline, b""):
        _diag["stdout_lines_total"] += 1

        try:
            line = raw_line.decode("utf-8", errors="replace")
            _diag["stdout_lines_decoded"] += 1

            cap = _parse_capture_line(line)
            if cap is None:
                _diag["stdout_lines_parsed_none"] += 1
                continue

            _diag["stdout_lines_parsed_ok"] += 1

            payload = dict(cap)
            payload["freq"] = PI_TIMER_FREQ
            payload["ns_per_tick"] = round(NS_PER_TICK, 3)

            edge_source = _classify_edge_source(payload)
            payload["edge_source"] = edge_source
            _diag["last_edge_source"] = edge_source
            if edge_source == EDGE_SOURCE_TDC:
                _diag["edge_source_tdc"] += 1
            else:
                _diag["edge_source_raw"] += 1

            time_key = system_time_z()
            payload["gnss_time"] = time_key

            _diag["last_time_key"] = time_key
            _diag["last_ppspoll_seq"] = payload.get("seq")

            with _state_lock:
                pps_count = _assign_pps_count()
                if pps_count is not None:
                    payload["pps_count"] = int(pps_count)

                _last_capture = payload

            if payload.get("seq") and int(payload["seq"]) % 60 == 0:
                logging.info(
                    "⏱️ [pitimer] seq=%d pps_count=%s edge=%s corrected=%d detect=%d corr=%d time=%s",
                    int(payload.get("seq") or 0),
                    str(payload.get("pps_count", "—")),
                    str(payload.get("edge_source", "—")),
                    int(payload.get("corrected") or 0),
                    int(payload.get("detect_ns") or 0),
                    int(payload.get("correction_ticks") or 0),
                    time_key,
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

            threading.Thread(
                target=_capture_reader,
                args=(proc,),
                daemon=True,
                name="pitimer-reader",
            ).start()

            retcode = proc.wait()
            _diag["ppspoll_exits"] += 1
            _diag["ppspoll_last_exit_code"] = retcode
            logging.warning("⚠️ [pitimer] pps_poll exited with code %d — restarting in 5s", retcode)

        except Exception:
            logging.exception("💥 [pitimer] supervisor error — restarting in 5s")

        time.sleep(5)

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
            # keep this small but useful
            "ppspoll_spawns": _diag["ppspoll_spawns"],
            "ppspoll_exits": _diag["ppspoll_exits"],
            "stdout_lines_total": _diag["stdout_lines_total"],
            "stdout_lines_parsed_ok": _diag["stdout_lines_parsed_ok"],
            "stdout_parse_exceptions": _diag["stdout_parse_exceptions"],
            "pps_count_seen": _diag["pps_count_seen"],
            "pps_count_repeat": _diag["pps_count_repeat"],
            "pps_count_regress": _diag["pps_count_regress"],
            "pps_count_jump": _diag["pps_count_jump"],
            "edge_source_tdc": _diag["edge_source_tdc"],
            "edge_source_raw": _diag["edge_source_raw"],
            "last_ppspoll_seq": _diag["last_ppspoll_seq"],
            "last_time_key": _diag["last_time_key"],
            "last_pps_count": _diag["last_pps_count"],
        },
        "last_set_event": _diag.get("last_set_event", {}),
        "last_pps_count_anomaly": _diag.get("last_pps_count_anomaly", {}),
        "last_parse_exception": _diag.get("last_parse_exception", {}),
    }

    if cap:
        # core facts for correlation + debug
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
            }
        )

    return {"success": True, "message": "OK", "payload": payload}


COMMANDS = {
    "SET_PPS_COUNT": cmd_set_pps_count,
    "REPORT": cmd_report,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------


def run() -> None:
    setup_logging()

    logging.info(
        "⏱️ [pitimer] starting (MINIMAL) — Pi arch timer PPS capture via chrony polling. "
        "freq=%d Hz, %.3f ns/tick, isolated core %d. "
        "Single-capture model (no rings, no history). "
        "Commands: SET_PPS_COUNT, REPORT.",
        PI_TIMER_FREQ,
        NS_PER_TICK,
        ISOLATED_CORE,
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