"""timepulse_watch — Verify 10 KHz GNSS relay signal on Pi GPIO 25

Counts rising edges on GPIO 25 using lgpio, reports frequency and
edge-to-edge timing statistics at a configurable interval.

Usage:
  python3 timepulse_watch.py [interval_sec] [duration_sec]

Defaults: 2 second reporting interval, 60 second duration.

Requirements:
  pip install lgpio
"""

import lgpio
import sys
import time

GPIO_CHIP = 0
GPIO_PIN = 25
EXPECTED_HZ = 10000

interval = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

h = lgpio.gpiochip_open(GPIO_CHIP)

# Configure as input
lgpio.gpio_claim_input(h, GPIO_PIN)

# Use alert callback to count edges and capture timestamps
edge_count = 0
edge_times = []
lock = None  # lgpio callbacks are serialized, no lock needed

def on_edge(chip, gpio, level, timestamp_ns):
    global edge_count
    edge_count += 1
    # Keep last N timestamps for jitter analysis
    if len(edge_times) < 200:
        edge_times.append(timestamp_ns)

lgpio.gpio_claim_alert(h, GPIO_PIN, eFlags=lgpio.RISING_EDGE)
cb = lgpio.callback(h, GPIO_PIN, lgpio.RISING_EDGE, on_edge)

print(f"=== TIMEPULSE 10 KHz signal monitor — GPIO {GPIO_PIN} ({duration}s at {interval}s intervals) ===\n")
print(f"{'elapsed':>8s}  {'edges':>8s}  {'freq_hz':>10s}  {'error_ppm':>10s}  {'status':>12s}")
print(f"{'─'*8}  {'─'*8}  {'─'*10}  {'─'*10}  {'─'*12}")

t0 = time.monotonic()
last_count = 0
last_time = t0

try:
    while (time.monotonic() - t0) < duration:
        time.sleep(interval)

        now = time.monotonic()
        dt = now - last_time
        current_count = edge_count
        delta_edges = current_count - last_count

        if dt > 0:
            freq = delta_edges / dt
            error_ppm = ((freq - EXPECTED_HZ) / EXPECTED_HZ) * 1e6
        else:
            freq = 0
            error_ppm = 0

        # Classify signal health
        if delta_edges == 0:
            status = "NO SIGNAL"
        elif abs(freq - EXPECTED_HZ) < 10:
            status = "OK"
        elif abs(freq - EXPECTED_HZ) < 100:
            status = "MARGINAL"
        else:
            status = f"BAD ({freq:.0f})"

        elapsed = now - t0
        print(f"{elapsed:7.1f}s  {delta_edges:8d}  {freq:10.1f}  {error_ppm:+10.1f}  {status:>12s}")

        last_count = current_count
        last_time = now

        # Jitter analysis: if we collected edge timestamps, report
        if len(edge_times) >= 10:
            deltas_us = []
            for i in range(1, len(edge_times)):
                delta_ns = edge_times[i] - edge_times[i-1]
                deltas_us.append(delta_ns / 1000.0)

            if deltas_us:
                mean_us = sum(deltas_us) / len(deltas_us)
                min_us = min(deltas_us)
                max_us = max(deltas_us)
                variance = sum((x - mean_us) ** 2 for x in deltas_us) / len(deltas_us)
                stddev_us = variance ** 0.5

                print(f"         edge-to-edge: mean={mean_us:.2f}µs  "
                      f"min={min_us:.2f}µs  max={max_us:.2f}µs  "
                      f"stddev={stddev_us:.3f}µs  (n={len(deltas_us)})")

        edge_times.clear()

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    cb.cancel()
    lgpio.gpiochip_close(h)

    total_elapsed = time.monotonic() - t0
    if total_elapsed > 0:
        avg_freq = edge_count / total_elapsed
        print(f"\nSummary: {edge_count} edges in {total_elapsed:.1f}s = {avg_freq:.1f} Hz")
    print("Done.")