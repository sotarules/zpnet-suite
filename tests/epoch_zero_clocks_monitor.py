"""
Epoch Zero + Clocks Monitor

Purpose:
    Repeatedly run:
      1) TEENSY EPOCH ZERO
      2) wait N seconds (default 2.0)
      3) TEENSY EPOCH CLOCKS

    Then print one row per cycle with compact synchronization metrics:
      - summary dwt64 cycle total
      - summary vclock/ocxo1/ocxo2 ns64 totals
      - ocxo deltas vs vclock (from detail if present, fallback from summary)
      - request/status metadata

Usage:
    .zt epoch_zero_clocks_monitor
    .zt epoch_zero_clocks_monitor 2.0
    .zt epoch_zero_clocks_monitor 2.0 0.5

Args:
    settle_seconds   Delay between ZERO and CLOCKS (default 2.0)
    loop_pause       Optional pause between cycles after CLOCKS (default 0.0)
"""

from __future__ import annotations

import sys
import time
from typing import Any, Optional

from zpnet.processes.processes import send_command


def fmt_int(v: Optional[int], width: int) -> str:
    if v is None:
        return f"{'n/a':>{width}s}"
    return f"{v:>{width},d}"


def fmt_signed(v: Optional[int], width: int) -> str:
    if v is None:
        return f"{'n/a':>{width}s}"
    return f"{v:+{width},d}"


def as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def cmd_epoch_zero() -> dict:
    return send_command(
        machine="TEENSY",
        subsystem="EPOCH",
        command="ZERO",
        args={},
    )


def cmd_epoch_clocks() -> dict:
    return send_command(
        machine="TEENSY",
        subsystem="EPOCH",
        command="CLOCKS",
        args={},
    )


def run(settle_seconds: float = 2.0, loop_pause: float = 0.0) -> None:
    print()
    print("Epoch Zero + Clocks Monitor")
    print(
        "cycle  req  zero_ok acc state           "
        "dwt64_total        vclock_ns64      ocxo1_ns64       ocxo2_ns64       "
        "ocxo1-vclk   ocxo2-vclk"
    )

    cycle = 0

    try:
        while True:
            cycle += 1

            zero_resp = cmd_epoch_zero()
            zero_ok = bool(zero_resp and zero_resp.get("success"))
            zero_payload = zero_resp.get("payload", {}) if zero_resp else {}

            accepted = zero_payload.get("accepted", None)
            req_id = as_int(zero_payload.get("request_id", None))
            zero_state = str(zero_payload.get("state", "n/a"))

            time.sleep(settle_seconds)

            clocks_resp = cmd_epoch_clocks()
            clocks_ok = bool(clocks_resp and clocks_resp.get("success"))
            clocks_payload = clocks_resp.get("payload", {}) if clocks_resp else {}

            summary = clocks_payload.get("summary", {}) if isinstance(clocks_payload, dict) else {}
            detail = clocks_payload.get("detail", {}) if isinstance(clocks_payload, dict) else {}

            dwt64_total = as_int(summary.get("dwt64_cycles_total"))
            vclock_ns64 = as_int(summary.get("vclock_ns64_total"))
            ocxo1_ns64 = as_int(summary.get("ocxo1_ns64_total"))
            ocxo2_ns64 = as_int(summary.get("ocxo2_ns64_total"))

            d1 = as_int(detail.get("ocxo1_norm_ticks64_delta_vs_vclock"))
            d2 = as_int(detail.get("ocxo2_norm_ticks64_delta_vs_vclock"))

            # Fallback if detail deltas are unavailable.
            if d1 is None and vclock_ns64 is not None and ocxo1_ns64 is not None:
                d1 = (ocxo1_ns64 - vclock_ns64) // 100
            if d2 is None and vclock_ns64 is not None and ocxo2_ns64 is not None:
                d2 = (ocxo2_ns64 - vclock_ns64) // 100

            ok_tag = "OK" if (zero_ok and clocks_ok) else "ERR"
            acc_tag = "Y" if accepted is True else ("N" if accepted is False else "?")

            print(
                f"{cycle:5d} "
                f"{fmt_int(req_id, 4)} "
                f"{ok_tag:>7s} "
                f"{acc_tag:>3s} "
                f"{zero_state:>15s} "
                f"{fmt_int(dwt64_total, 16)} "
                f"{fmt_int(vclock_ns64, 15)} "
                f"{fmt_int(ocxo1_ns64, 15)} "
                f"{fmt_int(ocxo2_ns64, 15)} "
                f"{fmt_signed(d1, 11)} "
                f"{fmt_signed(d2, 11)}"
            )

            if loop_pause > 0.0:
                time.sleep(loop_pause)

    except KeyboardInterrupt:
        print("\nstopped\n")


def main() -> None:
    settle_seconds = 2.0
    loop_pause = 0.0

    if len(sys.argv) >= 2:
        settle_seconds = float(sys.argv[1])
    if len(sys.argv) >= 3:
        loop_pause = float(sys.argv[2])

    run(settle_seconds=settle_seconds, loop_pause=loop_pause)


if __name__ == "__main__":
    main()
