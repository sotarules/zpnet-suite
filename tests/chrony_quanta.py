"""chrony_quanta3 — Detect tick quantization in the range Python can reach"""
#!/usr/bin/env python3
#
# chrony_quanta2 showed minimum delta is ~462 ns (Python overhead).
# This test focuses the histogram on the 400-800 ns range where
# deltas actually land, and checks whether they fall on the
# 500/27 ns (~18.519 ns) quantization grid.
#
# Usage:
#     zt chrony_quanta3 [samples]    # default 2,000,000

import sys
import time
from collections import Counter

SAMPLES = int(sys.argv[1]) if len(sys.argv) > 1 else 2_000_000
FREQ = 54_000_000
NS_PER_TICK = 1e9 / FREQ


def main():
    print(f"Sampling consecutive clock_gettime_ns deltas x {SAMPLES:,}")
    print(f"Tick period: 500/27 = {NS_PER_TICK:.6f} ns")
    print()

    # Collect consecutive deltas
    deltas = []
    prev = time.clock_gettime_ns(time.CLOCK_REALTIME)

    t0 = time.monotonic()
    for _ in range(SAMPLES):
        now = time.clock_gettime_ns(time.CLOCK_REALTIME)
        d = now - prev
        if d > 0:
            deltas.append(d)
        prev = now
    elapsed = time.monotonic() - t0

    print(f"Collected {len(deltas):,} nonzero deltas in {elapsed:.3f}s")
    print()

    # Focus on the typical range
    min_d = min(deltas)
    max_d = max(deltas)
    median_d = sorted(deltas)[len(deltas) // 2]
    print(f"Min: {min_d} ns  Median: {median_d} ns  Max: {max_d} ns")
    print()

    # Histogram of deltas in the tight range around the minimum
    LO = min_d - 5
    HI = min_d + 200
    focus = [d for d in deltas if LO <= d <= HI]
    counts = Counter(focus)

    print(f"=== Delta histogram ({LO}-{HI} ns) ===")
    print(f"Deltas in range: {len(focus):,} out of {len(deltas):,}")
    print()

    # Show every value with nonzero count
    max_count = max(counts.values()) if counts else 1
    print("    ns   count  ticks    bar")
    print("   ---   -----  -----    ---")
    for ns_val in range(LO, HI + 1):
        c = counts.get(ns_val, 0)
        if c > 0:
            ticks = ns_val / NS_PER_TICK
            bar = "#" * max(1, int(50 * c / max_count))
            print(f"  {ns_val:4d}  {c:6d}  {ticks:5.1f}    {bar}")

    print()

    # Analysis: are the populated values evenly spaced?
    populated = sorted(counts.keys())
    if len(populated) >= 2:
        spacings = [populated[i+1] - populated[i] for i in range(len(populated) - 1)]
        spacing_counts = Counter(spacings)

        print("=== Spacing between adjacent populated values ===")
        for sp, cnt in sorted(spacing_counts.items()):
            ticks = sp / NS_PER_TICK
            print(f"  spacing={sp:2d} ns ({ticks:.2f} ticks): occurs {cnt} times")

        print()
        print(f"Expected spacing for 1-tick quantization: ~18 or 19 ns")
        print(f"  (because 500/27 = 18.519..., integer ns alternates)")

    print()

    # Analysis: check if values align to the 500/27 grid
    # For each populated ns value, compute how close it is to a tick multiple
    print("=== Grid alignment check ===")
    print("For each delta value, distance to nearest exact tick multiple:")
    print()
    off_grid = 0
    on_grid = 0
    for ns_val in populated[:40]:  # first 40
        ticks_exact = ns_val * FREQ / 1e9
        nearest_tick = round(ticks_exact)
        nearest_ns = nearest_tick * 1e9 / FREQ
        error = abs(ns_val - nearest_ns)
        marker = "✅" if error < 1.0 else "❌"
        if error < 1.0:
            on_grid += 1
        else:
            off_grid += 1
        print(f"  {ns_val:4d} ns = {ticks_exact:.3f} ticks  "
              f"nearest={nearest_tick} ({nearest_ns:.1f} ns)  "
              f"error={error:.1f} ns {marker}")

    print()
    print(f"On grid (<1ns error): {on_grid}  Off grid: {off_grid}")


if __name__ == "__main__":
    main()