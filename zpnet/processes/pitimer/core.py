from __future__ import annotations

import ctypes
import logging
import math
import os
import signal
import threading
import time
from collections import OrderedDict
from typing import Any, Dict, Optional

from zpnet.processes.processes import server_setup
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import system_time_z

# ---------------------------------------------------------------------
# Optional userspace GPIO backend
# ---------------------------------------------------------------------

try:
    import gpiod  # type: ignore
except Exception:
    gpiod = None

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

# Defensive default:
#   gpio_user  -> userspace libgpiod edge events (safer, latency-prone)
#   ppslatch   -> native hardware-edge latch engine
CAPTURE_BACKEND = os.environ.get("PITIMER_BACKEND", "ppslatch").strip().lower()

PPSLATCH_LIB = os.environ.get(
    "PPSLATCH_LIB",
    "/home/mule/zpnet/zpnet/native/ppslatch/libppslatch.so",
)

CNTVCT_LIB = os.environ.get(
    "CNTVCT_LIB",
    "/home/mule/zpnet/zpnet/native/cntvct/libcntvct.so",
)

PI_TIMER_FREQ = 54_000_000
NS_PER_SECOND = 1_000_000_000
NS_PER_TICK = 1e9 / PI_TIMER_FREQ

BUFFER_MAX_SIZE = 120
PPSLATCH_RING_SIZE = 4096

GPIO_CHIP_PATH = os.environ.get("PITIMER_GPIO_CHIP", "/dev/gpiochip0")
GPIO_LINE_OFFSET = int(os.environ.get("PITIMER_GPIO_LINE", "25"))
GPIO_CONSUMER = "pitimer-pps"

SACRED_EDGE = "rising"
ENABLE_LOOP_SHADOW = True
USER_GPIO_POLL_TIMEOUT_S = 1.0

# Core 3 is dedicated to the ppslatch spin loop.
LATCH_CPU_CORE = int(os.environ.get("PITIMER_LATCH_CORE", "3"))

# ---------------------------------------------------------------------
# Native library interface — cntvct
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
# Native library interface — ppslatch
# ---------------------------------------------------------------------
#
# IMPORTANT — struct layout must match ppslatch.c exactly.
#
# The C ring uses _Alignas(64) on each atomic field, so each occupies
# a full 64-byte cache line.  The ctypes structs below mirror this
# with explicit padding bytes.  If the C struct ever changes, re-run
# ppslatch_ring_struct_size() / ppslatch_entry_size() and verify.
# ---------------------------------------------------------------------

_ppslatch_lib: Optional[ctypes.CDLL] = None


class PPSLatchEntry(ctypes.Structure):
    """Mirror of ppslatch_entry_t in ppslatch.c.

    Fields:
        captured_cntvct  uint64  — CNTVCT_EL0 at sacred edge
        shadow_cntvct    uint64  — last shadow read before edge
        delta_ticks      uint64  — captured - shadow
        edge_seq         uint64  — monotonic edge counter
        shadow_seq       uint64  — monotonic shadow counter
        flags            uint32  — reserved
    """
    _fields_ = [
        ("captured_cntvct", ctypes.c_uint64),
        ("shadow_cntvct", ctypes.c_uint64),
        ("delta_ticks", ctypes.c_uint64),
        ("edge_seq", ctypes.c_uint64),
        ("shadow_seq", ctypes.c_uint64),
        ("flags", ctypes.c_uint32),
    ]


class PPSLatchRing(ctypes.Structure):
    """Mirror of ppslatch_ring_t in ppslatch.c.

    Each atomic field in the C struct is _Alignas(64), occupying a full
    cache line.  We model this with 64-byte sub-structs so that the
    overall size and field offsets match exactly.
    """

    class _CacheLine_u64(ctypes.Structure):
        _fields_ = [
            ("value", ctypes.c_uint64),
            ("_pad", ctypes.c_uint8 * 56),
        ]

    class _CacheLine_i32(ctypes.Structure):
        _fields_ = [
            ("value", ctypes.c_int32),
            ("_pad", ctypes.c_uint8 * 60),
        ]

    _fields_ = [
        ("head",               _CacheLine_u64),   # atomic_uint_fast64_t
        ("tail",               _CacheLine_u64),
        ("edges_total",        _CacheLine_u64),
        ("overflows",          _CacheLine_u64),
        ("last_shadow_cntvct", _CacheLine_u64),
        ("last_shadow_seq",    _CacheLine_u64),
        ("running",            _CacheLine_i32),    # atomic_int
        ("stop",               _CacheLine_i32),
        ("entries",            PPSLatchEntry * PPSLATCH_RING_SIZE),
    ]


def _load_ppslatch_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL(PPSLATCH_LIB)

    lib.ppslatch_run.argtypes = [ctypes.POINTER(PPSLatchRing)]
    lib.ppslatch_run.restype = ctypes.c_int

    lib.ppslatch_stop.argtypes = [ctypes.POINTER(PPSLatchRing)]
    lib.ppslatch_stop.restype = None

    lib.ppslatch_cleanup.argtypes = []
    lib.ppslatch_cleanup.restype = None

    lib.ppslatch_drain.argtypes = [
        ctypes.POINTER(PPSLatchRing),
        ctypes.POINTER(PPSLatchEntry),
        ctypes.c_uint32,
    ]
    lib.ppslatch_drain.restype = ctypes.c_uint32

    lib.ppslatch_ring_size.argtypes = []
    lib.ppslatch_ring_size.restype = ctypes.c_uint32

    lib.ppslatch_entry_size.argtypes = []
    lib.ppslatch_entry_size.restype = ctypes.c_uint64

    lib.ppslatch_ring_struct_size.argtypes = []
    lib.ppslatch_ring_struct_size.restype = ctypes.c_uint64

    # Yield tuning — new in ppslatch with sched_yield() support
    lib.ppslatch_set_yield.argtypes = [ctypes.c_uint32]
    lib.ppslatch_set_yield.restype = None

    lib.ppslatch_get_yield.argtypes = []
    lib.ppslatch_get_yield.restype = ctypes.c_uint32

    return lib


def _verify_ppslatch_abi() -> None:
    """Verify ctypes struct sizes match the C ABI at startup."""
    c_entry_size = int(_ppslatch_lib.ppslatch_entry_size())
    c_ring_size = int(_ppslatch_lib.ppslatch_ring_struct_size())
    py_entry_size = ctypes.sizeof(PPSLatchEntry)
    py_ring_size = ctypes.sizeof(PPSLatchRing)

    logging.info(
        "⏱️ [pitimer] ppslatch ABI check: "
        "C entry_size=%d py=%d | C ring_struct_size=%d py=%d | ring_capacity=%d",
        c_entry_size, py_entry_size,
        c_ring_size, py_ring_size,
        int(_ppslatch_lib.ppslatch_ring_size()),
    )

    if py_entry_size != c_entry_size:
        logging.error(
            "🚨 [pitimer] PPSLatchEntry size MISMATCH: C=%d Python=%d — "
            "captures will be corrupt!",
            c_entry_size, py_entry_size,
        )

    if py_ring_size != c_ring_size:
        logging.error(
            "🚨 [pitimer] PPSLatchRing size MISMATCH: C=%d Python=%d — "
            "ring buffer will be corrupt!",
            c_ring_size, py_ring_size,
        )


# ---------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    "backend": CAPTURE_BACKEND,

    # Thread / backend state
    "capture_thread_started": False,
    "shadow_thread_started": False,
    "latch_thread_started": False,
    "drain_thread_started": False,
    "native_run_rc": None,
    "native_running": False,
    "native_edges_total": 0,
    "native_overflows": 0,
    "native_last_edge_seq": None,
    "native_last_shadow_seq": None,

    # Core affinity
    "latch_core_requested": LATCH_CPU_CORE,
    "latch_core_affinity_set": False,
    "latch_core_affinity_actual": None,
    "latch_yield_interval": None,

    # Capture processing
    "captures_total": 0,
    "capture_exceptions": 0,
    "captures_idle": 0,
    "rising_edges_seen": 0,
    "falling_edges_seen": 0,

    # Delta / histogram surface
    "delta_ticks_last": None,
    "delta_ticks_mean": 0.0,
    "delta_ticks_stddev": 0.0,
    "delta_ticks_min": None,
    "delta_ticks_max": None,

    "same_edge_period_ticks_last": None,
    "same_edge_period_ticks_mean": 0.0,
    "same_edge_period_ticks_stddev": 0.0,
    "same_edge_period_ticks_min": None,
    "same_edge_period_ticks_max": None,

    # Loop shadow
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
}

# ---------------------------------------------------------------------
# State
# ---------------------------------------------------------------------

_state_lock = threading.Lock()

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

_last_captured_cntvct: Optional[int] = None

# ---------------------------------------------------------------------
# Loop shadow state
# ---------------------------------------------------------------------

_shadow_counter: int = 0
_shadow_ns: int = 0
_shadow_seq: int = 0
_shadow_lock = threading.Lock()

_delta_n = 0
_delta_mean = 0.0
_delta_m2 = 0.0
_delta_min: Optional[int] = None
_delta_max: Optional[int] = None

_period_n = 0
_period_mean = 0.0
_period_m2 = 0.0
_period_min: Optional[int] = None
_period_max: Optional[int] = None

_ring = PPSLatchRing()
_native_thread: Optional[threading.Thread] = None

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------


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


def _delta_update_stats(delta_ticks: int) -> None:
    global _delta_n, _delta_mean, _delta_m2, _delta_min, _delta_max

    _delta_n += 1
    d1 = float(delta_ticks) - _delta_mean
    _delta_mean += d1 / _delta_n
    d2 = float(delta_ticks) - _delta_mean
    _delta_m2 += d1 * d2

    _delta_min = delta_ticks if _delta_min is None else min(_delta_min, delta_ticks)
    _delta_max = delta_ticks if _delta_max is None else max(_delta_max, delta_ticks)

    if _delta_n >= 2:
        variance = _delta_m2 / (_delta_n - 1)
        _diag["delta_ticks_stddev"] = math.sqrt(variance)
    else:
        _diag["delta_ticks_stddev"] = 0.0

    _diag["delta_ticks_last"] = delta_ticks
    _diag["delta_ticks_mean"] = _delta_mean
    _diag["delta_ticks_min"] = _delta_min
    _diag["delta_ticks_max"] = _delta_max


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


def _shadow_snapshot() -> Dict[str, int]:
    with _shadow_lock:
        return {
            "counter": int(_shadow_counter),
            "ns": int(_shadow_ns),
            "seq": int(_shadow_seq),
        }


def _shadow_update(counter: int, ns: int) -> None:
    global _shadow_counter, _shadow_ns, _shadow_seq
    with _shadow_lock:
        _shadow_counter = int(counter)
        _shadow_ns = int(ns)
        _shadow_seq += 1
    _diag["loop_shadow_samples"] += 1
    _diag["loop_shadow_last_counter"] = int(counter)
    _diag["loop_shadow_last_ns"] = int(ns)


def _shadow_thread_loop() -> None:
    _diag["shadow_thread_started"] = True
    while True:
        try:
            _shadow_update(_read_cntvct(), time.monotonic_ns())
        except Exception:
            _diag["capture_exceptions"] += 1
            logging.exception("💥 [pitimer] shadow loop failed")
            time.sleep(0.05)


def _process_capture(corrected: int, edge_meta: Dict[str, Any]) -> None:
    global _last_capture, _capture_seq
    global _pi_tick_epoch, _epoch_set, _pending_pi_ns
    global _last_captured_cntvct

    _diag["captures_total"] += 1

    shadow = _shadow_snapshot() if ENABLE_LOOP_SHADOW else {"counter": None, "ns": None, "seq": None}
    shadow_cntvct = shadow["counter"]
    shadow_ns = shadow["ns"]
    shadow_seq = shadow["seq"]

    if shadow_cntvct is not None:
        delta_ticks = int(corrected - shadow_cntvct)
        delta_ns = int(round(delta_ticks * NS_PER_TICK))
        _delta_update_stats(delta_ticks)
        _diag["loop_shadow_delta_ticks_last"] = delta_ticks
        _diag["loop_shadow_delta_ticks_mean"] = _diag["delta_ticks_mean"]
        _diag["loop_shadow_delta_ticks_stddev"] = _diag["delta_ticks_stddev"]
        _diag["loop_shadow_delta_ticks_min"] = _diag["delta_ticks_min"]
        _diag["loop_shadow_delta_ticks_max"] = _diag["delta_ticks_max"]
    else:
        delta_ticks = None
        delta_ns = None

    if _last_captured_cntvct is not None:
        same_edge_period_ticks = int(corrected - _last_captured_cntvct)
        same_edge_period_ns = int(round(same_edge_period_ticks * NS_PER_TICK))
        _period_update_stats(same_edge_period_ticks)
    else:
        same_edge_period_ticks = None
        same_edge_period_ns = None
    _last_captured_cntvct = corrected

    cap: Dict[str, Any] = {
        "capture_mode": CAPTURE_BACKEND,
        "capture_seq": None,
        "corrected": int(corrected),
        "captured_cntvct": int(corrected),
        "shadow_cntvct": shadow_cntvct,
        "loop_shadow_counter": shadow_cntvct,
        "loop_shadow_ns": shadow_ns,
        "loop_shadow_seq": shadow_seq,
        "loop_shadow_delta_ticks": delta_ticks,
        "loop_shadow_delta_ns": delta_ns,
        "delta_ticks": delta_ticks,
        "delta_ns": delta_ns,
        "same_edge_period_ticks": same_edge_period_ticks,
        "same_edge_period_ns": same_edge_period_ns,
        "freq": PI_TIMER_FREQ,
        "ns_per_tick": round(NS_PER_TICK, 6),
        **edge_meta,
    }

    with _state_lock:
        _capture_seq += 1
        cap["capture_seq"] = int(_capture_seq)
        cap["system_time"] = system_time_z()

        if _running and _pps_armed:
            pps_count = _assign_pps_count()
            cap["pps_count"] = int(pps_count)

            if _pending_pi_ns is not None:
                _pi_tick_epoch = _derive_epoch_from_pi_ns(int(corrected), int(_pending_pi_ns))
                _epoch_set = True
                _diag["epoch_derived_from_pi_ns"] += 1
                _diag["epoch_value"] = int(_pi_tick_epoch)
                _pending_pi_ns = None

            cap["pi_ns"] = _compute_pi_ns(int(corrected))
            _buffer_insert(pps_count, cap)
        else:
            _diag["captures_idle"] += 1
            cap["pps_count"] = None
            cap["pi_ns"] = _compute_pi_ns(int(corrected)) if _epoch_set else None

        _last_capture = dict(cap)
        _diag["last_capture"] = {
            "ts_utc": system_time_z(),
            "capture_seq": cap.get("capture_seq"),
            "corrected": cap.get("corrected"),
            "pi_ns": cap.get("pi_ns"),
            "pps_count": cap.get("pps_count"),
            "edge": cap.get("edge"),
            "delta_ticks": cap.get("delta_ticks"),
            "same_edge_period_ticks": cap.get("same_edge_period_ticks"),
        }


# ---------------------------------------------------------------------
# Native ppslatch backend
# ---------------------------------------------------------------------


def _native_run_thread() -> None:
    """Run the ppslatch spin loop on a dedicated core.

    This thread MUST be pinned to LATCH_CPU_CORE before entering the C
    spin loop.  The spin loop calls sched_yield() periodically (see
    ppslatch_set_yield) to keep the kernel watchdog happy, but core
    isolation is the primary defence against starving the scheduler.
    """
    _diag["latch_thread_started"] = True
    try:
        # ── Pin to dedicated latch core ──────────────────────────
        try:
            os.sched_setaffinity(0, {LATCH_CPU_CORE})
            actual = os.sched_getaffinity(0)
            _diag["latch_core_affinity_set"] = True
            _diag["latch_core_affinity_actual"] = sorted(actual)
            logging.info(
                "⏱️ [pitimer] ppslatch latch thread pinned to core %d "
                "(affinity=%s)",
                LATCH_CPU_CORE, actual,
            )
        except OSError as exc:
            _diag["latch_core_affinity_set"] = False
            _diag["latch_core_affinity_actual"] = None
            logging.error(
                "🚨 [pitimer] FAILED to pin latch thread to core %d: %s — "
                "system stability at risk without core isolation!",
                LATCH_CPU_CORE, exc,
            )

        # ── Log yield interval for diagnostics ───────────────────
        try:
            yi = int(_ppslatch_lib.ppslatch_get_yield())
            _diag["latch_yield_interval"] = yi
            logging.info(
                "⏱️ [pitimer] ppslatch yield interval=%d "
                "(0=disabled, for isolated core with SCHED_FIFO)",
                yi,
            )
        except Exception:
            logging.warning("⚠️ [pitimer] could not query ppslatch yield interval")

        # ── Enter the sacred spin loop (blocks until stop) ───────
        logging.info("[pitimer] ppslatch native run thread starting")
        rc = int(_ppslatch_lib.ppslatch_run(ctypes.byref(_ring)))
        _diag["native_run_rc"] = rc
        logging.info("[pitimer] ppslatch native run thread exited rc=%d", rc)

    except Exception:
        _diag["capture_exceptions"] += 1
        logging.exception("💥 [pitimer] ppslatch native run failed")


def _native_drain_loop() -> None:
    _diag["drain_thread_started"] = True
    drain_buf = (PPSLatchEntry * 1024)()

    while True:
        try:
            count = int(_ppslatch_lib.ppslatch_drain(
                ctypes.byref(_ring),
                drain_buf,
                len(drain_buf),
            ))

            _diag["native_edges_total"] = int(_ring.edges_total.value)
            _diag["native_overflows"] = int(_ring.overflows.value)
            _diag["native_running"] = bool(_ring.running.value)

            if count == 0:
                time.sleep(0.001)
                continue

            for i in range(count):
                ent = drain_buf[i]
                _diag["rising_edges_seen"] += 1
                _diag["native_last_edge_seq"] = int(ent.edge_seq)
                _diag["native_last_shadow_seq"] = int(ent.shadow_seq)

                _process_capture(
                    int(ent.captured_cntvct),
                    {
                        "edge": SACRED_EDGE,
                        "event_seq": int(ent.edge_seq),
                        "shadow_seq": int(ent.shadow_seq),
                    },
                )

        except Exception:
            _diag["capture_exceptions"] += 1
            logging.exception("💥 [pitimer] native drain loop failed")
            time.sleep(0.05)


# ---------------------------------------------------------------------
# Userspace GPIO backend
# ---------------------------------------------------------------------


def _event_type_name(ev: Any) -> str:
    et = getattr(ev, "event_type", None)
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


def _gpio_request():
    if gpiod is None:
        raise RuntimeError("python gpiod bindings unavailable")
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


def _gpio_capture_loop() -> None:
    _diag["capture_thread_started"] = True
    request = _gpio_request()

    while True:
        try:
            if not request.wait_edge_events(timeout=USER_GPIO_POLL_TIMEOUT_S):
                continue

            events = request.read_edge_events()
            for ev in events:
                kind = _event_type_name(ev)
                if kind == "rising":
                    _diag["rising_edges_seen"] += 1
                elif kind == "falling":
                    _diag["falling_edges_seen"] += 1

                corrected = _read_cntvct()
                edge_meta = {
                    "edge": kind,
                    "event_timestamp_ns": int(getattr(ev, "timestamp_ns", 0)),
                    "event_monotonic_ns_after": int(time.monotonic_ns()),
                    "gpio_chip": GPIO_CHIP_PATH,
                    "gpio_line": GPIO_LINE_OFFSET,
                }

                if kind == SACRED_EDGE:
                    _process_capture(corrected, edge_meta)
                else:
                    with _state_lock:
                        global _last_capture
                        _last_capture = {
                            "capture_mode": CAPTURE_BACKEND,
                            "capture_seq": _capture_seq,
                            "corrected": int(corrected),
                            "edge": kind,
                            **edge_meta,
                            "system_time": system_time_z(),
                        }
                        _diag["captures_total"] += 1
                        _diag["captures_idle"] += 1

        except Exception:
            _diag["capture_exceptions"] += 1
            logging.exception("💥 [pitimer] gpio capture loop failed")
            time.sleep(0.05)


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
        _diag["last_stop"] = {"ts_utc": system_time_z()}

    payload = {"status": "ok", "running": False, "pps_armed": False}
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

    payload = {"status": "ok", "pps_armed": True, "pps_next_count": int(pps_count)}
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

    payload = {"status": "ok", "epoch_ticks": int(_pi_tick_epoch)}
    return {"success": True, "message": "OK", "payload": payload}


def cmd_get_capture(args: Dict[str, Any]) -> Dict[str, Any]:
    pps_count = args.get("pps_count")
    if pps_count is None:
        return {"success": False, "message": "BAD", "payload": {"error": "missing pps_count"}}

    peek = bool(args.get("peek", False))

    with _state_lock:
        cap = _buffer_peek(int(pps_count)) if peek else _buffer_pop(int(pps_count))

    if cap is None:
        payload = {"hit": False, "pps_count": int(pps_count)}
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
            return {"success": False, "message": "BAD", "payload": {"error": "epoch not set"}}
        payload = {
            "pi_ns": int(pi_ns),
            "corrected": int(_last_capture["corrected"]),
            "capture_seq": int(_last_capture["capture_seq"]),
        }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_report(args: Dict[str, Any]) -> Dict[str, Any]:
    with _state_lock:
        payload: Dict[str, Any] = {
            "backend": CAPTURE_BACKEND,
            "running": _running,
            "pps_armed": _pps_armed,
            "pps_next_count": int(_pps_next_count),
            "epoch_set": _epoch_set,
            "pi_tick_epoch": int(_pi_tick_epoch) if _epoch_set else None,
            "pending_pi_ns": int(_pending_pi_ns) if _pending_pi_ns is not None else None,
            "buffer_size": len(_capture_buffer),
            "freq": PI_TIMER_FREQ,
            "ns_per_tick": round(NS_PER_TICK, 6),
            "diag": dict(_diag),
        }
        if CAPTURE_BACKEND == "gpio_user":
            payload["gpio_chip"] = GPIO_CHIP_PATH
            payload["gpio_line"] = GPIO_LINE_OFFSET
            payload["sacred_edge"] = SACRED_EDGE
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


def _graceful_shutdown(signum, frame):
    """Ensure ppslatch spin loop exits and GPIO is cleaned up before death.

    Without this, a SIGTERM during the spin loop leaves the latch thread
    killed mid-spin, gpio_cleanup() never runs, and the GPIO peripheral
    can end up in a dirty state that hangs I2C and other shared buses.
    """
    signame = signal.Signals(signum).name if hasattr(signal, 'Signals') else str(signum)
    logging.info("⏱️ [pitimer] %s received — initiating graceful ppslatch shutdown", signame)

    if _ppslatch_lib is not None:
        try:
            _ppslatch_lib.ppslatch_stop(ctypes.byref(_ring))
            logging.info("⏱️ [pitimer] ppslatch_stop signalled, waiting for latch thread...")
        except Exception:
            logging.exception("💥 [pitimer] ppslatch_stop failed")

        if _native_thread is not None and _native_thread.is_alive():
            _native_thread.join(timeout=3.0)
            if _native_thread.is_alive():
                logging.error(
                    "🚨 [pitimer] latch thread did not exit within 3s — "
                    "forcing cleanup anyway"
                )

        try:
            _ppslatch_lib.ppslatch_cleanup()
            logging.info("⏱️ [pitimer] ppslatch_cleanup complete — GPIO released")
        except Exception:
            logging.exception("💥 [pitimer] ppslatch_cleanup failed")

    # Re-raise so the process actually exits
    raise SystemExit(0)


def run() -> None:
    global _cntvct_lib, _ppslatch_lib, _native_thread

    setup_logging()

    logging.info(
        "⏱️ [pitimer] Defensive capture backend=%s. "
        "Default is userspace GPIO edge consumption; native ppslatch is opt-in. "
        "Commands: START, STOP, RECOVER, SET_PPS_COUNT, SET_EPOCH, GET_CAPTURE, PEEK_CAPTURE, NS, REPORT.",
        CAPTURE_BACKEND,
    )

    _cntvct_lib = _load_cntvct_lib()

    if ENABLE_LOOP_SHADOW:
        threading.Thread(
            target=_shadow_thread_loop,
            daemon=True,
            name="pitimer-shadow",
        ).start()

    if CAPTURE_BACKEND == "ppslatch":
        _ppslatch_lib = _load_ppslatch_lib()
        _verify_ppslatch_abi()

        # Trap SIGTERM/SIGINT so we can cleanly stop the spin loop
        # and release GPIO before the process dies.
        signal.signal(signal.SIGTERM, _graceful_shutdown)
        signal.signal(signal.SIGINT, _graceful_shutdown)
        logging.info("⏱️ [pitimer] SIGTERM/SIGINT handlers installed for graceful GPIO cleanup")

        _native_thread = threading.Thread(
            target=_native_run_thread,
            daemon=True,
            name="pitimer-ppslatch-native",
        )
        _native_thread.start()

        threading.Thread(
            target=_native_drain_loop,
            daemon=True,
            name="pitimer-ppslatch-drain",
        ).start()

    elif CAPTURE_BACKEND == "gpio_user":
        if gpiod is None:
            raise RuntimeError("PITIMER_BACKEND=gpio_user but python gpiod bindings are unavailable")

        threading.Thread(
            target=_gpio_capture_loop,
            daemon=True,
            name="pitimer-gpio-user",
        ).start()
    else:
        raise RuntimeError(f"unknown PITIMER_BACKEND '{CAPTURE_BACKEND}'")

    server_setup(
        subsystem="PITIMER",
        commands=COMMANDS,
        subscriptions={},
        blocking=True,
    )


if __name__ == "__main__":
    run()