"""adjtimex_watch — Watch kernel adjtimex frequency convergence in real time"""

import subprocess
import sys
import time

interval = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

print(f"=== Watching adjtimex frequency ({duration}s at {interval}s intervals) ===\n")
print(f"{'elapsed':>8s}  {'frequency':>12s}  {'ppb':>10s}  {'offset':>10s}  {'status':>8s}")
print(f"{'─'*8}  {'─'*12}  {'─'*10}  {'─'*10}  {'─'*8}")

t0 = time.monotonic()
last_freq = None

while (time.monotonic() - t0) < duration:
    result = subprocess.run(["adjtimex", "--print"], capture_output=True, text=True)
    if result.returncode != 0:
        print("adjtimex failed")
        break

    vals = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        for key in ("frequency", "offset", "status"):
            if line.startswith(f"{key}:"):
                vals[key] = int(line.split(":")[1].strip())

    freq = vals.get("frequency", 0)
    ppb = freq / 65536 * 1000
    offset = vals.get("offset", 0)
    status = vals.get("status", 0)

    delta = ""
    if last_freq is not None:
        d = freq - last_freq
        if d != 0:
            delta = f"  Δ{d:+d}"
    last_freq = freq

    elapsed = time.monotonic() - t0
    print(f"{elapsed:7.1f}s  {freq:12d}  {ppb:+10.1f}  {offset:10d}  {status:8d}{delta}")

    time.sleep(interval)

print("\nDone.")