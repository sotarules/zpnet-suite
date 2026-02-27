"""adjtimex_watch — Watch kernel adjtimex frequency convergence in real time

Computes total steering from BOTH coarse (tick) and fine (freq) fields:
  tick_ppb  = (tick - 10000) * 100,000     # each tick unit = 100 PPM
  freq_ppb  = (freq / 65536) * 1000        # scaled-PPM to PPB
  total_ppb = tick_ppb + freq_ppb           # net kernel correction
"""

import subprocess
import sys
import time

interval = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

print(f"=== Watching adjtimex frequency ({duration}s at {interval}s intervals) ===\n")
print(f"{'elapsed':>8s}  {'tick':>6s}  {'frequency':>12s}  {'freq_ppb':>10s}  {'total_ppb':>10s}  {'offset':>10s}  {'status':>8s}")
print(f"{'─'*8}  {'─'*6}  {'─'*12}  {'─'*10}  {'─'*10}  {'─'*10}  {'─'*8}")

t0 = time.monotonic()
last_total = None

while (time.monotonic() - t0) < duration:
    result = subprocess.run(["adjtimex", "--print"], capture_output=True, text=True)
    if result.returncode != 0:
        print("adjtimex failed")
        break

    vals = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        for key in ("frequency", "offset", "status", "tick"):
            if line.startswith(f"{key}:"):
                vals[key] = int(line.split(":")[1].strip())

    tick = vals.get("tick", 10000)
    freq = vals.get("frequency", 0)
    offset = vals.get("offset", 0)
    status = vals.get("status", 0)

    tick_ppb = (tick - 10000) * 100_000
    freq_ppb = freq / 65536 * 1000
    total_ppb = tick_ppb + freq_ppb

    delta = ""
    if last_total is not None:
        d = total_ppb - last_total
        if abs(d) > 0.05:
            delta = f"  Δ{d:+.1f}"
    last_total = total_ppb

    elapsed = time.monotonic() - t0
    print(f"{elapsed:7.1f}s  {tick:6d}  {freq:12d}  {freq_ppb:+10.1f}  {total_ppb:+10.1f}  {offset:10d}  {status:8d}{delta}")

    time.sleep(interval)

print("\nDone.")