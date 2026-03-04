"""pulsecapture_watch — Verify 10 KHz GPIO capture via libpulsecapture.so

Loads libpulsecapture.so, allocates the ring buffer, launches the
capture loop on a dedicated thread pinned to an isolated core, and
drains/reports from the main thread.

Usage:
  sudo -E zt pulsecapture_watch [duration_sec]

Requirements:
  - libpulsecapture.so built via build-native
  - /dev/gpiomem accessible (add user to gpio group, or run as root)
  - 10 KHz signal on GPIO 25 from Teensy pin 9
  - PITIMER stopped (it competes for the isolated core)
"""

import atexit
import ctypes
import os
import signal
import sys
import threading
import time

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

LIB_PATH = os.environ.get(
    "PULSECAPTURE_LIB",
    os.path.expanduser("~/zpnet/zpnet/native/pulsecapture/libpulsecapture.so"),
)

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
REPORT_INTERVAL = 2.0

PI_TIMER_FREQ = 54_000_000
NS_PER_TICK = 1e9 / PI_TIMER_FREQ
EXPECTED_EDGE_HZ = 10_000

ISOLATED_CORE = 3
DRAIN_BATCH = 25000

# ---------------------------------------------------------------------
# Ring buffer structures (must match pulsecapture.c exactly)
# ---------------------------------------------------------------------

PULSE_RING_SIZE = 32768
PULSE_FLAG_PPS = 1


class PulseEntry(ctypes.Structure):
    _fields_ = [
        ("cntvct", ctypes.c_uint64),
        ("edge_seq", ctypes.c_uint64),
        ("flags", ctypes.c_uint32),
        ("pps_rt_sec", ctypes.c_int64),
        ("pps_rt_nsec", ctypes.c_int64),
    ]


class PulseRing(ctypes.Structure):
    _fields_ = [
        ("head", ctypes.c_uint64),
        ("_pad_head", ctypes.c_uint8 * (64 - 8)),
        ("tail", ctypes.c_uint64),
        ("_pad_tail", ctypes.c_uint8 * (64 - 8)),
        ("edges_total", ctypes.c_uint64),
        ("_pad_edges", ctypes.c_uint8 * (64 - 8)),
        ("pps_total", ctypes.c_uint64),
        ("_pad_pps", ctypes.c_uint8 * (64 - 8)),
        ("overflows", ctypes.c_uint64),
        ("_pad_overflows", ctypes.c_uint8 * (64 - 8)),
        ("running", ctypes.c_int),
        ("_pad_running", ctypes.c_uint8 * (64 - 4)),
        ("stop", ctypes.c_int),
        ("_pad_stop", ctypes.c_uint8 * (64 - 4)),
        ("entries", PulseEntry * PULSE_RING_SIZE),
    ]


DrainArray = PulseEntry * DRAIN_BATCH

# ---------------------------------------------------------------------
# Load library
# ---------------------------------------------------------------------

if not os.path.isfile(LIB_PATH):
    print(f"❌ libpulsecapture.so not found at {LIB_PATH}")
    print("   Run: build-native")
    sys.exit(1)

print(f"Loading {LIB_PATH}...")
lib = ctypes.CDLL(LIB_PATH)

lib.pulsecapture_run.argtypes = [ctypes.POINTER(PulseRing)]
lib.pulsecapture_run.restype = ctypes.c_int

lib.pulsecapture_stop.argtypes = [ctypes.POINTER(PulseRing)]
lib.pulsecapture_stop.restype = None

lib.pulsecapture_cleanup.argtypes = []
lib.pulsecapture_cleanup.restype = None

lib.pulsecapture_drain.argtypes = [
    ctypes.POINTER(PulseRing),
    ctypes.POINTER(PulseEntry),
    ctypes.c_uint32,
]
lib.pulsecapture_drain.restype = ctypes.c_uint32

lib.pulsecapture_ring_struct_size.argtypes = []
lib.pulsecapture_ring_struct_size.restype = ctypes.c_uint64

# Verify struct sizes match
c_size = lib.pulsecapture_ring_struct_size()
py_size = ctypes.sizeof(PulseRing)
print(f"Ring struct size: C={c_size}, Python={py_size}")
if c_size != py_size:
    print(f"⚠️  WARNING: struct size mismatch! C={c_size} vs Python={py_size}")
    print("   Ring buffer layout may be incorrect. Check alignment/padding.")
    sys.exit(1)

# ---------------------------------------------------------------------
# Allocate ring buffer and drain buffer
# ---------------------------------------------------------------------

ring = PulseRing()
drain_buf = DrainArray()
print(f"Ring allocated: {PULSE_RING_SIZE} entries, drain buffer: {DRAIN_BATCH}")

# ---------------------------------------------------------------------
# Clean shutdown — CRITICAL to prevent Pi freeze
#
# Disables GPREN0 edge detection and clears GPEDS0.  Registered
# as both an atexit handler and a signal handler so cleanup happens
# regardless of how the process exits.
# ---------------------------------------------------------------------

_cleanup_done = False


def cleanup():
    global _cleanup_done
    if _cleanup_done:
        return
    _cleanup_done = True
    print("\n🧹 Cleaning up GPIO edge detection...")
    lib.pulsecapture_stop(ctypes.byref(ring))
    time.sleep(0.2)  # Give capture loop time to exit and call gpio_cleanup
    lib.pulsecapture_cleanup()  # Belt and suspenders
    print("✅ GPIO cleanup complete")


atexit.register(cleanup)


def signal_handler(signum, frame):
    cleanup()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ---------------------------------------------------------------------
# Launch capture loop on a background thread
# ---------------------------------------------------------------------


def capture_thread():
    os.sched_setaffinity(0, {ISOLATED_CORE})
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
    except PermissionError:
        print("⚠️  Could not set SCHED_FIFO — run with sudo for best results")

    print(f"Capture thread pinned to core {ISOLATED_CORE} (SCHED_FIFO)")

    rc = lib.pulsecapture_run(ctypes.byref(ring))
    if rc != 0:
        print(f"❌ pulsecapture_run returned {rc} — GPIO init failed?")
    else:
        print("Capture loop exited cleanly.")


t = threading.Thread(target=capture_thread, daemon=True, name="pulsecapture")
t.start()

time.sleep(1.0)

if not ring.running:
    print("❌ Capture loop did not start. Exiting.")
    cleanup()
    sys.exit(1)

print(f"✅ Capture loop running (edges_total={ring.edges_total})")
print(f"\n=== PULSECAPTURE 10 KHz monitor ({DURATION}s at {REPORT_INTERVAL}s intervals) ===\n")
print(f"{'elapsed':>8s}  {'edges':>8s}  {'freq_hz':>10s}  {'pps':>4s}  "
      f"{'overflows':>9s}  {'ring_used':>9s}  {'status':>10s}")
print(f"{'─'*8}  {'─'*8}  {'─'*10}  {'─'*4}  {'─'*9}  {'─'*9}  {'─'*10}")

# ---------------------------------------------------------------------
# Drain loop
# ---------------------------------------------------------------------

t0 = time.monotonic()
last_edges = 0
last_time = t0
last_pps = 0

try:
    while (time.monotonic() - t0) < DURATION:
        time.sleep(REPORT_INTERVAL)

        now = time.monotonic()
        dt = now - last_time

        edges = ring.edges_total
        pps = ring.pps_total
        overflows = ring.overflows

        delta_edges = edges - last_edges
        delta_pps = pps - last_pps

        freq = delta_edges / dt if dt > 0 else 0

        n = lib.pulsecapture_drain(ctypes.byref(ring), drain_buf, DRAIN_BATCH)
        ring_used = ring.head - ring.tail

        if delta_edges == 0:
            status = "NO SIGNAL"
        elif abs(freq - EXPECTED_EDGE_HZ) < 50:
            status = "OK"
        elif abs(freq - EXPECTED_EDGE_HZ) < 200:
            status = "MARGINAL"
        else:
            status = "BAD"

        elapsed = now - t0
        print(f"{elapsed:7.1f}s  {delta_edges:8d}  {freq:10.1f}  {delta_pps:4d}  "
              f"{overflows:9d}  {ring_used:9d}  {status:>10s}")

        if n >= 10:
            seq_deltas_ns = []

            for i in range(1, n):
                d = drain_buf[i].cntvct - drain_buf[i - 1].cntvct
                if 0 < d < PI_TIMER_FREQ:
                    ns = d * NS_PER_TICK
                    seq_deltas_ns.append(ns)

            if seq_deltas_ns:
                mean_ns = sum(seq_deltas_ns) / len(seq_deltas_ns)
                min_ns = min(seq_deltas_ns)
                max_ns = max(seq_deltas_ns)
                variance = sum((x - mean_ns) ** 2 for x in seq_deltas_ns) / len(seq_deltas_ns)
                stddev_ns = variance ** 0.5

                print(f"         edge-to-edge: mean={mean_ns:.1f}ns  "
                      f"min={min_ns:.1f}ns  max={max_ns:.1f}ns  "
                      f"stddev={stddev_ns:.1f}ns  (n={len(seq_deltas_ns)})")

        last_edges = edges
        last_pps = pps
        last_time = now

except KeyboardInterrupt:
    pass

# Clean shutdown
cleanup()

total_elapsed = time.monotonic() - t0
total_edges = ring.edges_total
total_pps = ring.pps_total
total_overflows = ring.overflows

if total_elapsed > 0:
    avg_freq = total_edges / total_elapsed
    print(f"\nSummary:")
    print(f"  Edges:     {total_edges} in {total_elapsed:.1f}s = {avg_freq:.1f} Hz")
    print(f"  PPS:       {total_pps}")
    print(f"  Overflows: {total_overflows}")
    print(f"  Expected:  {EXPECTED_EDGE_HZ} Hz")

print("Done.")