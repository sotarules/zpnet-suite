"""timer_freq — Read ARM Generic Timer frequency, compare to nominal 54 MHz"""

import struct
import os
import subprocess
import tempfile

NOMINAL_FREQ = 54_000_000

print("=== ARM Generic Timer Frequency ===\n")

# Method 1: Device tree
dt_path = "/proc/device-tree/timer/clock-frequency"
if os.path.exists(dt_path):
    with open(dt_path, "rb") as f:
        raw = f.read(4)
        freq = struct.unpack(">I", raw)[0]
    delta_ppb = (freq - NOMINAL_FREQ) / NOMINAL_FREQ * 1e9
    print(f"Device tree:  {freq} Hz")
    print(f"Nominal:      {NOMINAL_FREQ} Hz")
    print(f"Difference:   {freq - NOMINAL_FREQ:+d} Hz")
    print(f"Deviation:    {delta_ppb:+.1f} ppb")
else:
    print(f"Device tree:  {dt_path} not found")

print()

# Method 2: CNTFRQ_EL0 register via compiled C
try:
    src = r"""
    #include <stdio.h>
    #include <stdint.h>
    int main() {
        uint64_t freq;
        __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(freq));
        printf("%lu\n", freq);
        return 0;
    }
    """
    with tempfile.NamedTemporaryFile(suffix=".c", mode="w", delete=False) as f:
        f.write(src)
        c_path = f.name
    bin_path = c_path.replace(".c", "")
    result = subprocess.run(
        ["gcc", "-o", bin_path, c_path],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        result = subprocess.run([bin_path], capture_output=True, text=True)
        freq = int(result.stdout.strip())
        delta_ppb = (freq - NOMINAL_FREQ) / NOMINAL_FREQ * 1e9
        print(f"CNTFRQ_EL0:   {freq} Hz")
        print(f"Nominal:      {NOMINAL_FREQ} Hz")
        print(f"Difference:   {freq - NOMINAL_FREQ:+d} Hz")
        print(f"Deviation:    {delta_ppb:+.1f} ppb")
    else:
        print(f"CNTFRQ_EL0:   compile failed: {result.stderr.strip()}")
    os.unlink(c_path)
    if os.path.exists(bin_path):
        os.unlink(bin_path)
except Exception as e:
    print(f"CNTFRQ_EL0:   {e}")

print()

# Method 3: Current clocksource
cs_path = "/sys/devices/system/clocksource/clocksource0/current_clocksource"
if os.path.exists(cs_path):
    print(f"Clocksource:  {open(cs_path).read().strip()}")

# Method 4: adjtimex kernel state
try:
    result = subprocess.run(["adjtimex", "--print"], capture_output=True, text=True)
    if result.returncode == 0:
        print(f"\n=== Kernel timekeeping (adjtimex) ===\n")
        for line in result.stdout.splitlines():
            line = line.strip()
            if any(k in line for k in ["tick", "frequency", "status", "precision"]):
                print(f"  {line}")
except FileNotFoundError:
    print("\nadjtimex not installed (apt install adjtimex)")