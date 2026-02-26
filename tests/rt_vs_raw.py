"""rt_vs_raw — Compare CLOCK_REALTIME vs CLOCK_MONOTONIC_RAW to detect hidden steering"""

import sys
import time

duration = int(sys.argv[1]) if len(sys.argv) > 1 else 10

print(f"=== CLOCK_REALTIME vs CLOCK_MONOTONIC_RAW ({duration}s) ===\n")
print("MONOTONIC_RAW is guaranteed immune to ALL adjustments.")
print("If REALTIME diverges from it, something is steering the clock.\n")
print("Sampling...", flush=True)

t0_rt = time.clock_gettime_ns(time.CLOCK_REALTIME)
t0_raw = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)

time.sleep(duration)

t1_rt = time.clock_gettime_ns(time.CLOCK_REALTIME)
t1_raw = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)

delta_rt = t1_rt - t0_rt
delta_raw = t1_raw - t0_raw

diff_ns = delta_rt - delta_raw
diff_ppb = (diff_ns / delta_raw) * 1e9

print(f"\nCLOCK_REALTIME delta:      {delta_rt:,} ns")
print(f"CLOCK_MONOTONIC_RAW delta: {delta_raw:,} ns")
print(f"Difference:                {diff_ns:+,} ns over {duration}s")
print(f"Rate difference:           {diff_ppb:+.1f} ppb")
print()

if abs(diff_ppb) < 10:
    print("✅ No steering detected — both clocks running at same rate")
elif abs(diff_ppb) < 100:
    print("⚠️  Small rate difference — residual kernel correction may be active")
else:
    print("🔴 Significant steering — something is adjusting CLOCK_REALTIME")