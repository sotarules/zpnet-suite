"""
ZPNet Interpolation Proof — Head-to-Head Accuracy Test

Compares two interpolation strategies against VCLOCK ground truth:

  Path A (PPS anchor):  Interpolate from PPS-edge DWT using 1-second
                        prediction.  Current production path.  Window
                        is 0-1 seconds depending on query position.

  Path B (Spin anchor): Interpolate from TDC-corrected spin capture DWT.
                        Currently same window as Path A (spin fires once
                        per PPS), but the anchor is more precise (~1.3 ns
                        vs ~47-51 cycle ISR jitter).  When 10 KHz anchoring
                        is added, the window shrinks to ~100 µs.

Both paths are compared to the same VCLOCK reading at the same instant.
The difference in stddev answers: "Is the spin anchor worth it?"

The correlation with position (0.0 = just after PPS, 1.0 = just before
next PPS) reveals whether error grows with distance from the anchor.
If Path A shows correlation and Path B doesn't, the spin anchor is
eliminating drift-dependent error.

Usage:
    python -m zpnet.tests.interp_proof [interval_seconds] [reset]
    .zt interp_proof
    .zt interp_proof 1 reset
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

    print()
    print("  ── Path A: PPS anchor (production)  │  Path B: Spin anchor (TDC-corrected)")
    print()
    print(
        f"  {'n':>5s}"
        f"  {'pos':>5s}"
        f"  │ {'a_err':>7s}"
        f"  {'a_mean':>8s}"
        f"  {'a_sd':>7s}"
        f"  {'a_corr':>7s}"
        f"  │ {'b_err':>7s}"
        f"  {'b_mean':>8s}"
        f"  {'b_sd':>7s}"
        f"  {'b_corr':>7s}"
        f"  │ {'winner':>7s}"
    )
    print(
        f"  {'─'*5}"
        f"  {'─'*5}"
        f"  │ {'─'*7}"
        f"  {'─'*8}"
        f"  {'─'*7}"
        f"  {'─'*7}"
        f"  │ {'─'*7}"
        f"  {'─'*8}"
        f"  {'─'*7}"
        f"  {'─'*7}"
        f"  │ {'─'*7}"
    )

    n = 0
    a_stddev = 0.0
    b_stddev = 0.0
    a_corr = 0.0
    b_corr = 0.0
    a_mean = 0.0
    b_mean = 0.0

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

            if first:
                args = {}
                first = False

            if not resp or not resp.get("success"):
                err_msg = resp.get("payload", {}).get("error", "unknown") if resp else "no response"
                print(f"  error: {err_msg}")
                time.sleep(interval)
                continue

            p = resp.get("payload", {})

            n        = p.get("a_n", 0)
            pos      = p.get("position", 0.0)

            a_err    = p.get("a_err", 0)
            a_mean   = p.get("a_mean", 0.0)
            a_stddev = p.get("a_stddev", 0.0)
            a_corr   = p.get("a_corr", 0.0)

            b_n      = p.get("b_n", 0)
            b_err    = p.get("b_err", 0)
            b_mean   = p.get("b_mean", 0.0)
            b_stddev = p.get("b_stddev", 0.0)
            b_corr   = p.get("b_corr", 0.0)

            if b_n > 0 and n >= 2 and b_stddev > 0:
                if a_stddev > 0 and b_stddev > 0:
                    ratio = a_stddev / b_stddev
                    winner = f"B {ratio:.1f}x" if ratio > 1.0 else f"A {1.0/ratio:.1f}x" if ratio < 1.0 else "TIE"
                else:
                    winner = "---"

                print(
                    f"  {n:>5d}"
                    f"  {pos:>5.3f}"
                    f"  │ {a_err:>+7d}"
                    f"  {a_mean:>+8.1f}"
                    f"  {a_stddev:>7.1f}"
                    f"  {a_corr:>+7.4f}"
                    f"  │ {b_err:>+7d}"
                    f"  {b_mean:>+8.1f}"
                    f"  {b_stddev:>7.1f}"
                    f"  {b_corr:>+7.4f}"
                    f"  │ {winner:>7s}"
                )
            else:
                print(
                    f"  {n:>5d}"
                    f"  {pos:>5.3f}"
                    f"  │ {a_err:>+7d}"
                    f"  {a_mean:>+8.1f}"
                    f"  {a_stddev:>7.1f}"
                    f"  {a_corr:>+7.4f}"
                    f"  │ {'(waiting)':>7s}"
                    f"  {'':>8s}"
                    f"  {'':>7s}"
                    f"  {'':>7s}"
                    f"  │ {'---':>7s}"
                )

            time.sleep(interval)

    except KeyboardInterrupt:
        print()
        print("=" * 78)
        print(f"  RESULTS after {n} samples")
        print("=" * 78)
        print()

        if n >= 2:
            print(f"  Path A (PPS anchor — production):")
            print(f"    mean:    {a_mean:+.1f} ns")
            print(f"    stddev:  {a_stddev:.1f} ns")
            print(f"    corr:    {a_corr:+.4f}  ({'position-dependent' if abs(a_corr) > 0.1 else 'position-independent'})")
            print()

            if b_stddev > 0:
                print(f"  Path B (Spin anchor — TDC-corrected):")
                print(f"    mean:    {b_mean:+.1f} ns")
                print(f"    stddev:  {b_stddev:.1f} ns")
                print(f"    corr:    {b_corr:+.4f}  ({'position-dependent' if abs(b_corr) > 0.1 else 'position-independent'})")
                print()

                if a_stddev > 0 and b_stddev > 0:
                    ratio = a_stddev / b_stddev
                    if ratio > 1.05:
                        print(f"  VERDICT: Spin anchor wins by {ratio:.2f}x lower stddev")
                        print(f"           ({a_stddev:.1f} ns → {b_stddev:.1f} ns)")
                    elif ratio < 0.95:
                        print(f"  VERDICT: PPS anchor wins by {1.0/ratio:.2f}x lower stddev")
                        print(f"           ({a_stddev:.1f} ns vs {b_stddev:.1f} ns)")
                    else:
                        print(f"  VERDICT: No significant difference ({a_stddev:.1f} ns vs {b_stddev:.1f} ns)")

                    if abs(a_corr) > 0.1 and abs(b_corr) < 0.1:
                        print(f"           Path A error correlates with position (r={a_corr:+.3f})")
                        print(f"           Path B eliminates this — anchor precision matters more than window size")
                    elif abs(a_corr) > 0.1 and abs(b_corr) > 0.1:
                        print(f"           Both paths show position correlation — 10 KHz anchoring will help")
            else:
                print(f"  Path B: insufficient data (spin capture may not be active)")

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