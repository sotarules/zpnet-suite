"""kernel_mult — Read kernel clocksource mult/shift, compute effective tick rate"""

import subprocess
import re

print("=== Kernel Clocksource Conversion Factor ===\n")

NOMINAL_FREQ = 54_000_000

# The kernel's tick-to-ns conversion: ns = (ticks * mult) >> shift
# If mult/shift perfectly matches the real crystal freq, CLOCK_REALTIME
# won't drift even without NTP discipline.
#
# We can read mult and shift from /sys or by parsing kernel debug info.

# Method 1: Try debugfs (requires root)
try:
    result = subprocess.run(
        ["sudo", "cat", "/sys/kernel/debug/clocks/arch_sys_counter"],
        capture_output=True, text=True, timeout=5,
    )
    if result.returncode == 0:
        print(f"debugfs: {result.stdout.strip()}")
except Exception:
    pass

# Method 2: Compute from two known clocks
# Read CNTVCT_EL0 ticks and CLOCK_REALTIME ns at two points,
# derive the kernel's effective conversion ratio.
print("Measuring kernel's effective tick-to-ns ratio over 30 seconds...\n")

import ctypes
import ctypes.util
import time
import tempfile
import os

# Build a tiny C program that reads CNTVCT_EL0 and CLOCK_REALTIME atomically
src = r"""
#include <stdio.h>
#include <stdint.h>
#include <time.h>

static inline uint64_t read_cntvct(void) {
    uint64_t val;
    __asm__ __volatile__("isb\n\tmrs %0, cntvct_el0" : "=r"(val) :: "memory");
    return val;
}

int main() {
    struct timespec ts;
    uint64_t ticks;

    /* Read both as close together as possible */
    clock_gettime(CLOCK_REALTIME, &ts);
    ticks = read_cntvct();

    printf("%lu %ld %ld\n", ticks, (long)ts.tv_sec, ts.tv_nsec);
    return 0;
}
"""

with tempfile.NamedTemporaryFile(suffix=".c", mode="w", delete=False) as f:
    f.write(src)
    c_path = f.name
bin_path = c_path.replace(".c", "")

result = subprocess.run(["gcc", "-O2", "-o", bin_path, c_path],
                        capture_output=True, text=True)
if result.returncode != 0:
    print(f"Compile failed: {result.stderr}")
    raise SystemExit(1)

# Sample 1
r1 = subprocess.run([bin_path], capture_output=True, text=True)
parts1 = r1.stdout.strip().split()
ticks1 = int(parts1[0])
ns1 = int(parts1[1]) * 1_000_000_000 + int(parts1[2])

# Wait
duration = 30
print(f"Waiting {duration} seconds...", flush=True)
time.sleep(duration)

# Sample 2
r2 = subprocess.run([bin_path], capture_output=True, text=True)
parts2 = r2.stdout.strip().split()
ticks2 = int(parts2[0])
ns2 = int(parts2[1]) * 1_000_000_000 + int(parts2[2])

os.unlink(c_path)
os.unlink(bin_path)

delta_ticks = ticks2 - ticks1
delta_ns = ns2 - ns1

# Kernel's effective ratio: how many ns per tick?
ns_per_tick = delta_ns / delta_ticks
effective_freq = 1e9 / ns_per_tick

# Compare to nominal
deviation_hz = effective_freq - NOMINAL_FREQ
deviation_ppb = (deviation_hz / NOMINAL_FREQ) * 1e9

print(f"\nRaw CNTVCT_EL0 ticks:  {delta_ticks:,}")
print(f"CLOCK_REALTIME ns:     {delta_ns:,}")
print(f"Kernel ns/tick:        {ns_per_tick:.12f}")
print(f"Nominal ns/tick:       {1e9/NOMINAL_FREQ:.12f}")
print(f"Effective frequency:   {effective_freq:.3f} Hz")
print(f"Nominal frequency:     {NOMINAL_FREQ} Hz")
print(f"Deviation:             {deviation_hz:+.3f} Hz ({deviation_ppb:+.1f} ppb)")
print()

if abs(deviation_ppb) < 100:
    print("✅ Kernel conversion matches nominal — no hidden calibration")
    print("   (PiHR 0ppb drift must come from somewhere else)")
else:
    print(f"🔴 Kernel conversion differs from nominal by {deviation_ppb:+.1f} ppb")
    print(f"   This explains why CLOCK_REALTIME drifts differently than raw ticks")
    expected_pi_ppb = deviation_ppb
    print(f"   Pi clock should show ~{expected_pi_ppb:+.0f} ppb (kernel compensates)")