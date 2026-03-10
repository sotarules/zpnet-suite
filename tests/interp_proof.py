"""
ZPNet Interpolation Proof — continuous accuracy test

Usage:
    python -m zpnet.tests.interp_proof [interval_seconds] [reset]
    .zt interp_proof
    .zt interp_proof 1
    .zt interp_proof 2 reset

Calls INTERP_PROOF on the Teensy once per interval (default 1 second).
Each call takes one sample at whatever position in the second the
command happens to arrive, accumulating Welford stats on the Teensy.

Prints a running table showing:
  n       — sample count
  err     — this sample's error (ns)
  mean    — running mean error (should converge to ~0 if math is right)
  stddev  — running stddev (should converge to jitter floor)
  stderr  — standard error of the mean (shrinks as 1/sqrt(n))
  pos     — position in second (0.0 to 1.0) where this sample landed
  corr    — Pearson r(error, position) — should be ~0 if ratio is right
  min/max — running min/max error

Press Ctrl-C to stop.  The Teensy retains the accumulators — you can
restart this script and it continues from where it left off.  Send
reset=1 to clear and start fresh.
"""

from __future__ import annotations

import sys
import time

from zpnet.processes.processes import send_command


def run(interval: float = 1.0, reset: bool = False) -> None:
    args = {}
    if reset:
        args["reset"] = "1"
        print("Resetting accumulators...")

    # Header
    print()
    print(f"  {'n':>5s}  {'err':>7s}  {'mean':>8s}  {'stddev':>8s}"
          f"  {'stderr':>8s}  {'pos':>5s}  {'corr':>7s}"
          f"  {'min':>7s}  {'max':>7s}  {'out':>4s}")
    print(f"  {'─'*5}  {'─'*7}  {'─'*8}  {'─'*8}"
          f"  {'─'*8}  {'─'*5}  {'─'*7}"
          f"  {'─'*7}  {'─'*7}  {'─'*4}")

    first = True
    try:
        while True:
            try:
                resp = send_command(
                    machine="TEENSY",
                    subsystem="CLOCKS",
                    command="INTERP_PROOF",
                    args=args,
                )
            except Exception as e:
                print(f"  command failed: {e}")
                time.sleep(interval)
                continue

            # Only send reset on the first call
            if first:
                args = {}
                first = False

            if not resp or not resp.get("success"):
                err_msg = resp.get("payload", {}).get("error", "unknown") if resp else "no response"
                print(f"  error: {err_msg}")
                time.sleep(interval)
                continue

            p = resp.get("payload", {})

            n       = p.get("n", 0)
            err     = p.get("error_ns", 0)
            mean    = p.get("mean_ns", 0.0)
            stddev  = p.get("stddev_ns", 0.0)
            stderr  = p.get("stderr_ns", 0.0)
            pos     = p.get("position", 0.0)
            corr    = p.get("correlation", 0.0)
            err_min = p.get("min_ns", 0)
            err_max = p.get("max_ns", 0)
            outside = p.get("outside_100ns", 0)

            print(f"  {n:>5d}  {err:>+7d}  {mean:>+8.1f}  {stddev:>8.1f}"
                  f"  {stderr:>8.2f}  {pos:>5.3f}  {corr:>+7.4f}"
                  f"  {err_min:>+7d}  {err_max:>+7d}  {outside:>4d}")

            time.sleep(interval)

    except KeyboardInterrupt:
        print()
        print(f"  Stopped after {n} samples.")
        if n >= 2:
            print(f"  Final mean: {mean:+.1f} ns  stderr: {stderr:.2f} ns")
            if abs(mean) < 2 * stderr:
                print(f"  ✅ Mean is within 2σ of zero — no systematic bias detected.")
            else:
                print(f"  ⚠️  Mean is {abs(mean)/stderr:.1f}σ from zero — possible systematic bias.")
            if abs(corr) < 0.1:
                print(f"  ✅ No correlation with position (r={corr:+.4f})")
            else:
                print(f"  ⚠️  Correlation with position: r={corr:+.4f}")
        print()


def main():
    interval = 1.0
    reset = False

    for arg in sys.argv[1:]:
        if arg == "reset":
            reset = True
        else:
            try:
                interval = float(arg)
            except ValueError:
                pass

    run(interval=interval, reset=reset)


if __name__ == "__main__":
    main()