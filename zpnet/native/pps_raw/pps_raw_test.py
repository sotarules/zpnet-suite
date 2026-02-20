#!/usr/bin/env python3
"""
pps_raw_test — Read and display raw PPS captures from /dev/pps_raw

Usage:
    python3 pps_raw_test.py [count]

Reads PPS captures and displays raw CNTVCT_EL0 values, deltas,
and residuals vs expected 54 MHz tick count.

Default: run forever (Ctrl-C to stop)
Optional: specify number of captures to read
"""

import os
import struct
import sys
import math

DEVICE = "/dev/pps_raw"
CAPTURE_SIZE = 16  # sizeof(pps_raw_capture): u64 + u32 + u32

# ARM Generic Timer frequency on Pi 4/5
TIMER_FREQ = 54_000_000
NS_PER_TICK = 1_000_000_000 / TIMER_FREQ


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else None

    fd = os.open(DEVICE, os.O_RDONLY)
    print(f"Reading from {DEVICE} (timer freq = {TIMER_FREQ} Hz, "
          f"{NS_PER_TICK:.3f} ns/tick)")
    print(f"{'seq':>6}  {'counter':>20}  {'delta':>12}  "
          f"{'residual':>10}  {'residual_ns':>12}  {'missed':>6}")
    print("-" * 80)

    last_counter = None
    n = 0
    welford_n = 0
    welford_mean = 0.0
    welford_m2 = 0.0

    try:
        while count is None or n < count:
            data = os.read(fd, CAPTURE_SIZE)
            if len(data) < CAPTURE_SIZE:
                print("Short read — device removed?")
                break

            counter, sequence, missed = struct.unpack("=QII", data)

            if last_counter is not None:
                delta = counter - last_counter
                residual = delta - TIMER_FREQ
                residual_ns = residual * NS_PER_TICK

                # Welford's online algorithm
                welford_n += 1
                d1 = residual_ns - welford_mean
                welford_mean += d1 / welford_n
                d2 = residual_ns - welford_mean
                welford_m2 += d1 * d2

                stddev = math.sqrt(welford_m2 / (welford_n - 1)) if welford_n >= 2 else 0.0
                stderr = stddev / math.sqrt(welford_n) if welford_n >= 2 else 0.0

                print(f"{sequence:>6}  {counter:>20}  {delta:>12}  "
                      f"{residual:>10}  {residual_ns:>12.3f}  {missed:>6}"
                      f"  (mean={welford_mean:.3f} sd={stddev:.3f} "
                      f"se={stderr:.3f} n={welford_n})")
            else:
                print(f"{sequence:>6}  {counter:>20}  {'---':>12}  "
                      f"{'---':>10}  {'---':>12}  {missed:>6}")

            last_counter = counter
            n += 1

    except KeyboardInterrupt:
        print(f"\n--- {welford_n} residuals captured ---")
        if welford_n > 0:
            stddev = math.sqrt(welford_m2 / (welford_n - 1)) if welford_n >= 2 else 0.0
            stderr = stddev / math.sqrt(welford_n) if welford_n >= 2 else 0.0
            print(f"mean = {welford_mean:.3f} ns")
            print(f"sd   = {stddev:.3f} ns")
            print(f"se   = {stderr:.3f} ns")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()