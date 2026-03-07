from __future__ import annotations

import ctypes
import logging
import math
import os
import threading
import time
from collections import OrderedDict
from typing import Any, Dict, Optional

import gpiod

from zpnet.processes.processes import server_setup
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import system_time_z

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

CNTVCT_LIB = os.environ.get(
    "CNTVCT_LIB",
    "/home/mule/zpnet/zpnet/native/cntvct/libcntvct.so",
)

PI_TIMER_FREQ = 54_000_000
NS_PER_SECOND = 1_000_000_000
NS_PER_TICK = 1e9 / PI_TIMER_FREQ

BUFFER_MAX_SIZE = 120

# Dedicated PITIMER PPS input on the Pi.
GPIO_CHIP_PATH = os.environ.get("PITIMER_GPIO_CHIP", "/dev/gpiochip0")
GPIO_LINE_OFFSET = int(os.environ.get("PITIMER_GPIO_LINE", "25"))
GPIO_CONSUMER = "pitimer-pps"

CAPTURE_CORE = int(os.environ.get("PITIMER_CAPTURE_CORE", "3"))
LOOP_SHADOW_CORE = int(os.environ.get("PITIMER_LOOP_SHADOW_CORE", "3"))

ENABLE_LOOP_SHADOW = True

# We treat the rising edge as the sacred PPS identity boundary.
# Falling edges are observed diagnostically only.
SACRED_EDGE = "rising"

# ---------------------------------------------------------------------
# Native library interface — CNTVCT
# ---------------------------------------------------------------------

_cntvct_lib: Optional[ctypes.CDLL] = None


def _load_cntvct_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(CNTVCT_LIB)
    lib.read_cntvct_el0.argtypes = []
    lib.read_cntvct_el0.restype = ctypes.c_uint64
    return lib


def _read_cntvct() -> int:
    return int(_cntvct_lib.read_cntvct_el0())


# ---------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Capture loop
    "captures_total": 0,
    "capture_exceptions": 0,
    "captures_idle": 0,
    "rising_edges_seen": 0,
    "falling_edges_seen": 0,

    # Edge timing / histogram surface
    "same_edge_period_ticks_last": None,
    "same_edge_period_ticks_mean": 0.0,
    "same_edge_period_ticks_stddev": 0.0,
    "same_edge_period_ticks_min": None,
    "same_edge_period_ticks_max": None,

    # Loop shadow (diagnostic only)
    "loop_shadow_enabled": ENABLE_LOOP_SHADOW,
    "loop_shadow_samples": 0,
    "loop_shadow_last_counter": None,
    "loop_shadow_last_ns": None,
    "loop_shadow_delta_ticks_last": None,
    "loop_shadow_delta_ticks_mean": 0.0,
    "loop_shadow_delta_ticks_stddev": 0.0,
    "loop_shadow_delta_ticks_min": None,
    "loop_shadow_delta_ticks_max": None,

    # Buffer management
    "buffer_inserts": 0,
    "buffer_evictions": 0,
    "buffer_hits": 0,
    "buffer_peeks": 0,
    "buffer_misses": 0,
    "buffer_size_current": 0,
    "buffer_size_max_seen": 0,
    "last_eviction": {},

    # pps_count continuity
    "pps_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_regress": 0,
    "pps_count_jump": 0,
    "last_pps_count": None,
    "last_pps_count_anomaly": {},

    # Epoch / pi_ns derivation
    "epoch_derived_from_pi_ns": 0,
    "epoch_set_requests": 0,
    "epoch_set_applied": 0,
    "epoch_value": None,

    # Lifecycle
    "starts": 0,
    "stops": 0,
    "recovers": 0,
    "last_start": {},
    "last_stop": {},
    "last_recover": {},

    # Last-seen values
    "last_capture": {},
    "last_set_event": {},

    # Thread / scheduler
    "capture_thread_started": False,
    "shadow_thread_started": False,
}

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

_state_lock = threading.Lock()

# RUNNING means sacred rising-edge captures are assigned pps_count and buffered.
# STOPPED means edge loop still runs, but captures are observational only.
_running: bool = False

_capture_buffer: OrderedDict[int, Dict[str, Any]] = OrderedDict()
_last_capture: Optional[Dict[str, Any]] = None
_capture_seq: int = 0

_pps_armed: bool = False
_pps_next_count: int = 0
_last_pps_count_seen: Optional[int] = None

_pi_tick_epoch: int = 0
_epoch_set: bool = False
_pending_pi_ns: Optional[int] = None

_last_rising_cntvct: Optional[int] = None
_last_falling_cntvct: Optional[int] = None

# ---------------------------------------------------------------------
# Loop shadow state
# ---------------------------------------------------------------------

_shadow_counter: int = 0
_shadow_ns: int = 0
_shadow_seq: int = 0
_shadow_lock = threading.Lock()

# Welford — loop shadow delta ticks (captured_cntvct - shadow_counter)
_shadow_n = 0
_shadow_mean = 0.0
_shadow_m2 = 0.0
_shadow_min: Optional[int] = None
_shadow_max: Optional[int] = None

# Welford — same-edge period ticks (rising→rising)
_period_n = 0
_period_mean = 0.0
_period_m2 = 0.0
_period_min: Optional[int] = None
_period_max: Optional[int] = None


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------


def _pin_this_thread_to_core(core: int) -> None:
    try:
        os.sched_setaffinity(0, {int(core)})
        logging.info("📌 [pitimer] pinned thread to core %d", core)
    except Exception:
        logging.exception("⚠️ [pitimer] failed to pin thread to core %d", core)


def _compute_pi_ns(corrected: int) -> Optional[int]:
    if not _epoch_set:
        return None
    campaign_ticks = int(corrected) - int(_pi_tick_epoch)
    return (campaign_ticks * NS_PER_SECOND) // PI_TIMER_FREQ


def _derive_epoch_from_pi_ns(corrected: int, pi_ns: int) -> int:
    pi_ticks_elapsed = (int(pi_ns) * PI_TIMER_FREQ) // NS_PER_SECOND
    return int(corrected) - int(pi_ticks_elapsed)


def _assign_pps_count() -> int:
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


def _buffer_insert(pps_count: int, capture: Dict[str, Any]) -> None:
    _capture_buffer[pps_count] = capture
    _diag["buffer_inserts"] += 1

    size = len(_capture_buffer)
    _diag["buffer_size_current"] = size
    if size > _diag["buffer_size_max_seen"]:
        _diag["buffer_size_max_seen"] = size

    while len(_capture_buffer) > BUFFER_MAX_SIZE:
        evicted_pps, _ = _capture_buffer.popitem(last=False)
        _diag["buffer_evictions"] += 1
        _diag["last_eviction"] = {
            "ts_utc": system_time_z(),
            "evicted_pps_count": evicted_pps,
            "buffer_size": len(_capture_buffer),
        }
        logging.warning(
            "⚠️ [pitimer] buffer eviction: pps_count=%d (buffer max=%d)",
            evicted_pps,
            BUFFER_MAX_SIZE,
        )


def _buffer_pop(pps_count: int) -> Optional[Dict[str, Any]]:
    cap = _capture_buffer.pop(pps_count, None)
    if cap is not None:
        _diag["buffer_hits"] += 1
    else:
        _diag["buffer_misses"] += 1
    _diag["buffer_size_current"] = len(_capture_buffer)
    return cap


def _buffer_peek(pps_count: int) -> Optional[Dict[str, Any]]:
    cap = _capture_buffer.get(pps_count)
    if cap is not None:
        _diag["buffer_peeks"] += 1
        return dict(cap)
    _diag["buffer_misses"] += 1
    return None


def _shadow_snapshot() -> Dict[str, int]:
    with _shadow_lock:
        return {
            "counter": int(_shadow_counter),
            "ns": int(_shadow_ns),
            "seq": int(_shadow_seq),
        }


def _shadow_update_stats(delta_ticks: int) -> None:
    global _shadow_n, _shadow_mean, _shadow_m2, _shadow_min, _shadow_max

    _shadow_n += 1
    d1 = float(delta_ticks) - _shadow_mean
    _shadow_mean += d1 / _shadow_n
    d2 = float(delta_ticks) - _shadow_mean
    _shadow_m2 += d1 * d2

    _shadow_min = delta_ticks if _shadow_min is None else min(_shadow_min, delta_ticks)
    _shadow_max = delta_ticks if _shadow_max is None else max(_shadow_max, delta_ticks)

    if _shadow_n >= 2:
        variance = _shadow_m2 / (_shadow_n - 1)
        _diag["loop_shadow_delta_ticks_stddev"] = math.sqrt(variance)
    else:
        _diag["loop_shadow_delta_ticks_stddev"] = 0.0

    _diag["loop_shadow_delta_ticks_last"] = delta_ticks
    _diag["loop_shadow_delta_ticks_mean"] = _shadow_mean
    _diag["loop_shadow_delta_ticks_min"] = _shadow_min
    _diag["loop_shadow_delta_ticks_max"] = _shadow_max


def _period_update_stats(period_ticks: int) -> None:
    global _period_n, _period_mean, _period_m2, _period_min, _period_max

    _period_n += 1
    d1 = float(period_ticks) - _period_mean
    _period_mean += d1 / _period_n
    d2 = float(period_ticks) - _period_mean
    _period_m2 += d1 * d2

    _period_min = period_ticks if _period_min is None else min(_period_min, period_ticks)
    _period_max = period_ticks if _period_max is None else max(_period_max, period_ticks)

    if _period_n >= 2:
        variance = _period_m2 / (_period_n - 1)
        _diag["same_edge_period_ticks_stddev"] = math.sqrt(variance)
    else:
        _diag["same_edge_period_ticks_stddev"] = 0.0

    _diag["same_edge_period_ticks_last"] = period_ticks
    _diag["same_edge_period_ticks_mean"] = _period_mean
    _diag["same_edge_period_ticks_min"] = _period_min
    _diag["same_edge_period_ticks_max"] = _period_max


def _event_type_name(ev: gpiod.EdgeEvent) -> str:
    et = ev.event_type
    name = str(et).upper()
    if "RISING" in name:
        return "rising"
    if "FALLING" in name:
        return "falling"
    if et == 1:
        return "rising"
    if et == 2:
        return "falling"
    return f"unknown:{et}"


# ---------------------------------------------------------------------
# Loop shadow thread (diagnostic only)
# ---------------------------------------------------------------------


def _loop_shadow_thread() -> None:
    global _shadow_counter, _shadow_ns, _shadow_seq

    _pin_this_thread_to_core(LOOP_SHADOW_CORE)
    _diag["shadow_thread_started"] = True

    while True:
        c = _read_cntvct()
        n = time.monotonic_ns()
        with _shadow_lock:
            _shadow_counter = int(c)
            _shadow_ns = int(n)
            _shadow_seq += 1
        _diag["loop_shadow_samples"] += 1
        _diag["loop_shadow_last_counter"] = int(c)
        _diag["loop_shadow_last_ns"] = int(n)


# ---------------------------------------------------------------------
# GPIO PPS capture — operational authority
# ---------------------------------------------------------------------


def _open_gpio_request() -> gpiod.LineRequest:
    chip = gpiod.Chip(GPIO_CHIP_PATH)
    return chip.request_lines(
        consumer=GPIO_CONSUMER,
        config={
            GPIO_LINE_OFFSET: gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT,
                edge_detection=gpiod.line.Edge.BOTH,
                bias=gpiod.line.Bias.DISABLED,
            )
        },
    )


def _capture_loop() -> None:
    global _last_capture, _capture_seq
    global _pi_tick_epoch, _epoch_set, _pending_pi_ns
    global _last_rising_cntvct, _last_falling_cntvct

    _pin_this_thread_to_core(CAPTURE_CORE)
    _diag["capture_thread_started"] = True

    request = _open_gpio_request()
    logging.info(
        "⏱️ [pitimer] listening on %s line %d (sacred edge=%s)",
        GPIO_CHIP_PATH,
        GPIO_LINE_OFFSET,
        SACRED_EDGE,
    )

    while True:
        try:
            if not request.wait_edge_events(timeout=1.0):
                continue

            events = request.read_edge_events()
            for ev in events:
                kind = _event_type_name(ev)
                if kind == "rising":
                    _diag["rising_edges_seen"] += 1
                elif kind == "falling":
                    _diag["falling_edges_seen"] += 1

                corrected = _read_cntvct()
                monotonic_ns_after = time.monotonic_ns()
                shadow = _shadow_snapshot() if ENABLE_LOOP_SHADOW else {"counter": None, "ns": None, "seq": None}

                shadow_counter = shadow["counter"]
                shadow_ns = shadow["ns"]
                shadow_seq = shadow["seq"]

                shadow_delta_ticks: Optional[int]
                shadow_delta_ns: Optional[int]
                if ENABLE_LOOP_SHADOW and shadow_counter is not None:
                    shadow_delta_ticks = int(corrected - shadow_counter)
                    shadow_delta_ns = int(round(shadow_delta_ticks * NS_PER_TICK))
                    _shadow_update_stats(shadow_delta_ticks)
                else:
                    shadow_delta_ticks = None
                    shadow_delta_ns = None

                prev_same_edge_cntvct: Optional[int]
                if kind == "rising":
                    prev_same_edge_cntvct = _last_rising_cntvct
                    _last_rising_cntvct = corrected
                elif kind == "falling":
                    prev_same_edge_cntvct = _last_falling_cntvct
                    _last_falling_cntvct = corrected
                else:
                    prev_same_edge_cntvct = None

                if prev_same_edge_cntvct is not None:
                    same_edge_period_ticks = int(corrected - prev_same_edge_cntvct)
                    same_edge_period_ns = int(round(same_edge_period_ticks * NS_PER_TICK))
                    if kind == SACRED_EDGE:
                        _period_update_stats(same_edge_period_ticks)
                else:
                    same_edge_period_ticks = None
                    same_edge_period_ns = None

                cap: Dict[str, Any] = {
                    "gpio_chip": GPIO_CHIP_PATH,
                    "gpio_line": GPIO_LINE_OFFSET,
                    "edge": kind,
                    "event_timestamp_ns": int(ev.timestamp_ns),
                    "event_monotonic_ns_after": int(monotonic_ns_after),
                    "corrected": int(corrected),
                    "freq": PI_TIMER_FREQ,
                    "ns_per_tick": round(NS_PER_TICK, 6),
                    "capture_mode": "gpio_irq",
                    "loop_shadow_enabled": ENABLE_LOOP_SHADOW,
                    "loop_shadow_counter": shadow_counter,
                    "loop_shadow_ns": shadow_ns,
                    "loop_shadow_seq": shadow_seq,
                    "loop_shadow_delta_ticks": shadow_delta_ticks,
                    "loop_shadow_delta_ns": shadow_delta_ns,
                    "same_edge_period_ticks": same_edge_period_ticks,
                    "same_edge_period_ns": same_edge_period_ns,
                }

                with _state_lock:
                    _capture_seq += 1
                    cap["capture_seq"] = int(_capture_seq)
                    cap["system_time"] = system_time_z()
                    _diag["captures_total"] += 1

                    # Only the sacred edge advances operational state.
                    if kind == SACRED_EDGE and _running and _pps_armed:
                        pps_count = _assign_pps_count()
                        cap["pps_count"] = int(pps_count)

                        if _pending_pi_ns is not None:
                            _pi_tick_epoch = _derive_epoch_from_pi_ns(corrected, int(_pending_pi_ns))
                            _epoch_set = True
                            _diag["epoch_derived_from_pi_ns"] += 1
                            _diag["epoch_value"] = int(_pi_tick_epoch)
                            _pending_pi_ns = None

                        cap["pi_ns"] = _compute_pi_ns(corrected)
                        _buffer_insert(pps_count, cap)
                    else:
                        _diag["captures_idle"] += 1
                        cap["pps_count"] = None
                        cap["pi_ns"] = _compute_pi_ns(corrected) if (_epoch_set and kind == SACRED_EDGE) else None

                    _last_capture = dict(cap)
                    _diag["last_capture"] = {
                        "ts_utc": system_time_z(),
                        "capture_seq": cap.get("capture_seq"),
                        "edge": cap.get("edge"),
                        "pps_count": cap.get("pps_count"),
                        "corrected": cap.get("corrected"),
                        "pi_ns": cap.get("pi_ns"),
                        "loop_shadow_delta_ticks": cap.get("loop_shadow_delta_ticks"),
                        "same_edge_period_ticks": cap.get("same_edge_period_ticks"),
                    }

        except Exception:
            _diag["capture_exceptions"] += 1
            logging.exception("💥 [pitimer] GPIO PPS capture failed")
            time.sleep(0.1)


# ---------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------


def cmd_start(args: Dict[str, Any]) -> Dict[str, Any]:
    global _running, _pps_armed, _pps_next_count, _capture_buffer
    global _pending_pi_ns, _last_pps_count_seen

    pps_count = args.get("pps_count")
    if pps_count is None:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    pi_ns = args.get("pi_ns")

    with _state_lock:
        _running = True
        _pps_armed = True
        _pps_next_count = int(pps_count)
        _capture_buffer.clear()
        _diag["buffer_size_current"] = 0
        _last_pps_count_seen = None
        _pending_pi_ns = int(pi_ns) if pi_ns is not None else None

        _diag["starts"] += 1
        _diag["last_start"] = {
            "ts_utc": system_time_z(),
            "pps_count": int(pps_count),
            "pi_ns": int(pi_ns) if pi_ns is not None else None,
        }

    payload = {
        "status": "ok",
        "running": True,
        "pps_armed": True,
        "pps_next_count": int(pps_count),
        "pending_pi_ns": int(pi_ns) if pi_ns is not None else None,
    }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_stop(args: Dict[str, Any]) -> Dict[str, Any]:
    global _running, _pps_armed, _capture_buffer, _pending_pi_ns, _last_pps_count_seen

    with _state_lock:
        _running = False
        _pps_armed = False
        _capture_buffer.clear()
        _diag["buffer_size_current"] = 0
        _pending_pi_ns = None
        _last_pps_count_seen = None

        _diag["stops"] += 1
        _diag["last_stop"] = {
            "ts_utc": system_time_z(),
        }

    payload = {
        "status": "ok",
        "running": False,
        "pps_armed": False,
    }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_recover(args: Dict[str, Any]) -> Dict[str, Any]:
    global _running, _pps_armed, _pps_next_count, _capture_buffer
    global _pending_pi_ns, _last_pps_count_seen

    pps_count = args.get("pps_count")
    pi_ns = args.get("pi_ns")

    if pps_count is None or pi_ns is None:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count or pi_ns"}}

    with _state_lock:
        _running = True
        _pps_armed = True
        _pps_next_count = int(pps_count)
        _capture_buffer.clear()
        _diag["buffer_size_current"] = 0
        _last_pps_count_seen = None
        _pending_pi_ns = int(pi_ns)

        _diag["recovers"] += 1
        _diag["last_recover"] = {
            "ts_utc": system_time_z(),
            "pps_count": int(pps_count),
            "pi_ns": int(pi_ns),
        }

    payload = {
        "status": "ok",
        "running": True,
        "pps_armed": True,
        "pps_next_count": int(pps_count),
        "pending_pi_ns": int(pi_ns),
    }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_set_pps_count(args: Dict[str, Any]) -> Dict[str, Any]:
    global _pps_armed, _pps_next_count, _last_pps_count_seen

    pps_count = args.get("pps_count")
    if pps_count is None:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    with _state_lock:
        _pps_armed = True
        _pps_next_count = int(pps_count)
        _last_pps_count_seen = None
        _diag["last_set_event"] = {
            "ts_utc": system_time_z(),
            "kind": "SET_PPS_COUNT",
            "pps_count": int(pps_count),
        }

    payload = {
        "status": "ok",
        "pps_armed": True,
        "pps_next_count": int(pps_count),
    }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_set_epoch(args: Dict[str, Any]) -> Dict[str, Any]:
    global _pi_tick_epoch, _epoch_set

    epoch_ticks = args.get("epoch_ticks")
    if epoch_ticks is None:
        return {"success": False, "message": "BAD", "payload": {"error": "missing epoch_ticks"}}

    with _state_lock:
        _diag["epoch_set_requests"] += 1
        _pi_tick_epoch = int(epoch_ticks)
        _epoch_set = True
        _diag["epoch_set_applied"] += 1
        _diag["epoch_value"] = int(_pi_tick_epoch)
        _diag["last_set_event"] = {
            "ts_utc": system_time_z(),
            "kind": "SET_EPOCH",
            "epoch_ticks": int(epoch_ticks),
        }

    payload = {
        "status": "ok",
        "epoch_ticks": int(_pi_tick_epoch),
    }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_get_capture(args: Dict[str, Any]) -> Dict[str, Any]:
    pps_count = args.get("pps_count")
    if pps_count is None:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    peek = bool(args.get("peek", False))

    with _state_lock:
        if peek:
            cap = _buffer_peek(int(pps_count))
        else:
            cap = _buffer_pop(int(pps_count))

    if cap is None:
        payload = {
            "hit": False,
            "pps_count": int(pps_count),
        }
        return {"success": True, "message": "OK", "payload": payload}

    payload = dict(cap)
    payload["hit"] = True
    return {"success": True, "message": "OK", "payload": payload}


def cmd_peek_capture(args: Dict[str, Any]) -> Dict[str, Any]:
    with _state_lock:
        if _last_capture is None:
            payload = {"hit": False}
        else:
            payload = {"hit": True, **dict(_last_capture)}
    return {"success": True, "message": "OK", "payload": payload}


def cmd_ns(args: Dict[str, Any]) -> Dict[str, Any]:
    with _state_lock:
        if _last_capture is None:
            return {"success": False, "message": "BAD", "payload": {"error": "no capture yet"}}
        pi_ns = _last_capture.get("pi_ns")
        if pi_ns is None:
            return {"success": False, "message": "BAD", "payload": {"error": "epoch not set or last edge not sacred"}}
        payload = {
            "pi_ns": int(pi_ns),
            "corrected": int(_last_capture["corrected"]),
            "capture_seq": int(_last_capture["capture_seq"]),
            "edge": _last_capture.get("edge"),
        }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_report(args: Dict[str, Any]) -> Dict[str, Any]:
    with _state_lock:
        payload: Dict[str, Any] = {
            "running": _running,
            "pps_armed": _pps_armed,
            "pps_next_count": int(_pps_next_count),
            "epoch_set": _epoch_set,
            "pi_tick_epoch": int(_pi_tick_epoch) if _epoch_set else None,
            "pending_pi_ns": int(_pending_pi_ns) if _pending_pi_ns is not None else None,
            "buffer_size": len(_capture_buffer),
            "freq": PI_TIMER_FREQ,
            "ns_per_tick": round(NS_PER_TICK, 6),
            "gpio_chip": GPIO_CHIP_PATH,
            "gpio_line": GPIO_LINE_OFFSET,
            "sacred_edge": SACRED_EDGE,
            "diag": dict(_diag),
        }
        if _last_capture is not None:
            payload["last_capture"] = dict(_last_capture)
    return {"success": True, "message": "OK", "payload": payload}


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "RECOVER": cmd_recover,
    "SET_PPS_COUNT": cmd_set_pps_count,
    "SET_EPOCH": cmd_set_epoch,
    "GET_CAPTURE": cmd_get_capture,
    "PEEK_CAPTURE": cmd_peek_capture,
    "NS": cmd_ns,
    "REPORT": cmd_report,
}


def run() -> None:
    global _cntvct_lib

    setup_logging()

    logging.info(
        "⏱️ [pitimer] GPIO IRQ capture authority. "
        "Operational truth comes from GPIO edge events on %s:%d + immediate CNTVCT_EL0 read. "
        "Loop shadow is diagnostic only. "
        "Commands: START, STOP, RECOVER, SET_PPS_COUNT, SET_EPOCH, GET_CAPTURE, PEEK_CAPTURE, NS, REPORT.",
        GPIO_CHIP_PATH,
        GPIO_LINE_OFFSET,
    )

    _cntvct_lib = _load_cntvct_lib()

    if ENABLE_LOOP_SHADOW:
        threading.Thread(
            target=_loop_shadow_thread,
            daemon=True,
            name="pitimer-loop-shadow",
        ).start()

    threading.Thread(
        target=_capture_loop,
        daemon=True,
        name="pitimer-capture",
    ).start()

    server_setup(
        subsystem="PITIMER",
        commands=COMMANDS,
        subscriptions={},
        blocking=True,
    )


if __name__ == "__main__":
    run()
