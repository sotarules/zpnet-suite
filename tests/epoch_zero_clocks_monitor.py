"""
Epoch Zero + CLOCKS Report Monitor

Purpose:
    Repeatedly run:
      1) TEENSY CLOCKS ZERO
      2) wait N seconds (default 2.0)
      3) TEENSY CLOCKS REPORT

    Then print one row per cycle with compact synchronization metrics from the
    current CLOCKS report schema:

      payload.summary.report_dwt
      payload.summary.dwt64_cycles
      payload.summary.dwt64_ns
      payload.summary.gnss_ns
      payload.summary.vclock_ns
      payload.summary.ocxo1_ns
      payload.summary.ocxo2_ns
      payload.summary.{vclock,ocxo1,ocxo2}_valid

    Deltas are printed in ns and ticks relative to VCLOCK/GNSS.

Usage:
    .zt epoch_zero_clocks_monitor_clocksv2
    .zt epoch_zero_clocks_monitor_clocksv2 2.0
    .zt epoch_zero_clocks_monitor_clocksv2 2.0 0.5

Args:
    settle_seconds   Delay between ZERO and REPORT (default 2.0)
    loop_pause       Optional pause between cycles after REPORT (default 0.0)
"""

from __future__ import annotations

import sys
import time
from typing import Any, Optional

from zpnet.processes.processes import send_command


NS_PER_TICK = 100


def as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def as_bool(v: Any) -> Optional[bool]:
    if isinstance(v, bool):
        return v
    if v is None:
        return None
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("true", "1", "yes", "y"):
            return True
        if s in ("false", "0", "no", "n"):
            return False
    return None


def fmt_int(v: Optional[int], width: int) -> str:
    if v is None:
        return f"{'n/a':>{width}s}"
    return f"{v:>{width},d}"


def fmt_signed(v: Optional[int], width: int) -> str:
    if v is None:
        return f"{'n/a':>{width}s}"
    return f"{v:+{width},d}"


def fmt_bool(v: Optional[bool]) -> str:
    if v is True:
        return "Y"
    if v is False:
        return "N"
    return "?"


def payload(resp: Optional[dict]) -> dict:
    if not isinstance(resp, dict):
        return {}
    p = resp.get("payload", {})
    return p if isinstance(p, dict) else {}


def summary_from_payload(p: dict) -> dict:
    s = p.get("summary", {})
    return s if isinstance(s, dict) else {}


def delta_ns(a: Optional[int], b: Optional[int]) -> Optional[int]:
    if a is None or b is None:
        return None
    return a - b


def delta_ticks(a: Optional[int], b: Optional[int]) -> Optional[int]:
    d = delta_ns(a, b)
    if d is None:
        return None
    return d // NS_PER_TICK


def cmd_clocks_zero() -> dict:
    return send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="ZERO",
        args={},
    )


def cmd_clocks_report() -> dict:
    return send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="REPORT",
        args={},
    )


def print_header() -> None:
    print()
    print("Epoch Zero + CLOCKS Report Monitor")
    print(
        "cycle zero report zinst epseq capseq win "
        "report_dwt      "
        "gnss_ns        vclock_ns      ocxo1_ns       ocxo2_ns       "
        "o1-vclk ns   o2-vclk ns   o1-vclk t  o2-vclk t  "
        "dwt-vclk ns  valid"
    )


def run(settle_seconds: float = 2.0, loop_pause: float = 0.0) -> None:
    print_header()
    cycle = 0

    try:
        while True:
            cycle += 1

            zero_resp = cmd_clocks_zero()
            zero_ok = bool(zero_resp and zero_resp.get("success"))
            zp = payload(zero_resp)

            zero_installed = as_bool(zp.get("zero_installed"))
            epoch_sequence = as_int(zp.get("epoch_sequence"))
            capture_sequence = as_int(zp.get("epoch_capture_sequence"))
            capture_window = as_int(zp.get("epoch_capture_window_cycles"))

            time.sleep(settle_seconds)

            report_resp = cmd_clocks_report()
            report_ok = bool(report_resp and report_resp.get("success"))
            rp = payload(report_resp)
            s = summary_from_payload(rp)

            report_dwt = as_int(s.get("report_dwt"))
            dwt64_ns = as_int(s.get("dwt64_ns"))
            gnss_ns = as_int(s.get("gnss_ns"))
            vclock_ns = as_int(s.get("vclock_ns"))
            ocxo1_ns = as_int(s.get("ocxo1_ns"))
            ocxo2_ns = as_int(s.get("ocxo2_ns"))

            vclock_valid = as_bool(s.get("vclock_valid"))
            ocxo1_valid = as_bool(s.get("ocxo1_valid"))
            ocxo2_valid = as_bool(s.get("ocxo2_valid"))

            d_o1_ns = delta_ns(ocxo1_ns, vclock_ns)
            d_o2_ns = delta_ns(ocxo2_ns, vclock_ns)
            d_o1_ticks = delta_ticks(ocxo1_ns, vclock_ns)
            d_o2_ticks = delta_ticks(ocxo2_ns, vclock_ns)
            d_dwt_ns = delta_ns(dwt64_ns, vclock_ns)

            ok_tag = "OK" if (zero_ok and report_ok) else "ERR"
            z_tag = fmt_bool(zero_installed)
            valid_tag = (
                fmt_bool(vclock_valid)
                + fmt_bool(ocxo1_valid)
                + fmt_bool(ocxo2_valid)
            )

            print(
                f"{cycle:5d} "
                f"{ok_tag:>4s} "
                f"{'OK' if report_ok else 'ERR':>6s} "
                f"{z_tag:>5s} "
                f"{fmt_int(epoch_sequence, 5)} "
                f"{fmt_int(capture_sequence, 6)} "
                f"{fmt_int(capture_window, 3)} "
                f"{fmt_int(report_dwt, 12)} "
                f"{fmt_int(gnss_ns, 14)} "
                f"{fmt_int(vclock_ns, 14)} "
                f"{fmt_int(ocxo1_ns, 14)} "
                f"{fmt_int(ocxo2_ns, 14)} "
                f"{fmt_signed(d_o1_ns, 12)} "
                f"{fmt_signed(d_o2_ns, 12)} "
                f"{fmt_signed(d_o1_ticks, 10)} "
                f"{fmt_signed(d_o2_ticks, 10)} "
                f"{fmt_signed(d_dwt_ns, 12)} "
                f"{valid_tag:>5s}"
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
