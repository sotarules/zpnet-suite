"""
VCLOCK Perturbation Monitor

Purpose:
    Print the GNSS/QTimer clock in simple decimal form:
      - current 32-bit raw value and delta from prior poll
      - current accumulated 64-bit raw value and delta from prior poll
      - current GNSS nanoseconds and delta from prior poll
      - PPS count for context
      - QTimer high/low/high read diagnostics:
          * total reads
          * same-hi fast-path count
          * hi-changed retry count
          * per-poll deltas for those counters
          * per-poll retry percentage
          * last hi1/lo/hi2/lo2 samples

Usage:
    .zt vclock_perturbation_monitor
    .zt vclock_perturbation_monitor 1.0
    .zt vclock_perturbation_monitor 1.0 5 100000000

Args:
    interval_seconds   Poll interval in seconds (default 1.0)
    fire_every         Fire VCLOCK_TEST every N polls (0 = never, default 0)
    vclock_test_ns     Requested VCLOCK_TEST interval in ns (default 100000000)
"""

from __future__ import annotations

import sys
import time
from typing import Optional

from zpnet.processes.processes import send_command


def fmt_i64(value: Optional[int], width: int = 16) -> str:
    if value is None:
        return f"{'n/a':>{width}s}"
    return f"{value:>{width},d}"


def fmt_i64_signed(value: Optional[int], width: int = 16) -> str:
    if value is None:
        return f"{'n/a':>{width}s}"
    return f"{value:+{width},d}"


def fmt_pct(value: Optional[float], width: int = 6, prec: int = 2) -> str:
    if value is None:
        return f"{'n/a':>{width}s}"
    return f"{value:>{width}.{prec}f}"


def opt_int(payload: dict, key: str) -> Optional[int]:
    value = payload.get(key, None)
    if value is None:
        return None
    return int(value)


def u32_delta(curr: Optional[int], prev: Optional[int]) -> Optional[int]:
    if curr is None or prev is None:
        return None
    return (curr - prev) & 0xFFFFFFFF


def fetch_clocks_info() -> Optional[dict]:
    resp = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="CLOCKS_INFO",
        args={},
    )
    if not resp or not resp.get("success"):
        return None
    return resp.get("payload", {})


def fire_vclock_test(ns_val: int) -> str:
    try:
        resp = send_command(
            machine="TEENSY",
            subsystem="TIMEPOP",
            command="VCLOCK_TEST",
            args={"ns": str(ns_val)},
        )
    except Exception as e:
        return f"VCLOCK_TEST failed: {e}"

    if not resp or not resp.get("success"):
        payload = resp.get("payload", {}) if resp else {}
        return f"VCLOCK_TEST error: {payload.get('error', 'unknown')}"

    payload = resp.get("payload", {})
    req_ns = int(payload.get("req_ns", ns_val))
    arm_raw = payload.get("arm_vclock_raw", None)
    target_raw = payload.get("target_vclock_raw", None)
    arm_pps = payload.get("arm_pps", None)

    arm_raw_s = str(int(arm_raw)) if arm_raw is not None else "n/a"
    target_raw_s = str(int(target_raw)) if target_raw is not None else "n/a"
    arm_pps_s = str(int(arm_pps)) if arm_pps is not None else "n/a"

    return (
        f"VCLOCK_TEST armed "
        f"req_ns={req_ns} "
        f"arm_raw={arm_raw_s} "
        f"target_raw={target_raw_s} "
        f"arm_pps={arm_pps_s}"
    )


def run(
    interval: float = 1.0,
    fire_every: int = 0,
    vclock_test_ns: int = 100_000_000,
) -> None:
    print()
    print("VCLOCK Perturbation Monitor")
    print()

    prev_pps: Optional[int] = None
    prev_raw32: Optional[int] = None
    prev_raw64: Optional[int] = None
    prev_gnss_ns: Optional[int] = None

    prev_qtot: Optional[int] = None
    prev_qsame: Optional[int] = None
    prev_qretry: Optional[int] = None

    poll_n = 0

    try:
        while True:
            poll_n += 1

            payload = fetch_clocks_info()
            if payload is None:
                print(f"poll={poll_n:3d} ERROR reading CLOCKS_INFO")
                time.sleep(interval)
                continue

            sec = opt_int(payload, "campaign_seconds")
            pps = opt_int(payload, "time_pps_count")
            raw32 = opt_int(payload, "gnss_raw_now")
            raw64 = opt_int(payload, "gnss_raw_64")
            gnss_ns = opt_int(payload, "gnss_ns_now")

            qtot = opt_int(payload, "qread_total")
            qsame = opt_int(payload, "qread_same_hi")
            qretry = opt_int(payload, "qread_retry_hi_changed")

            q_hi1 = opt_int(payload, "qread_last_hi1")
            q_lo = opt_int(payload, "qread_last_lo")
            q_hi2 = opt_int(payload, "qread_last_hi2")
            q_lo2 = opt_int(payload, "qread_last_lo2")

            d_pps = None if (pps is None or prev_pps is None) else (pps - prev_pps)
            d_raw32 = u32_delta(raw32, prev_raw32)
            d_raw64 = None if (raw64 is None or prev_raw64 is None) else (raw64 - prev_raw64)
            d_gnss = None if (gnss_ns is None or prev_gnss_ns is None) else (gnss_ns - prev_gnss_ns)

            d_qtot = None if (qtot is None or prev_qtot is None) else (qtot - prev_qtot)
            d_qsame = None if (qsame is None or prev_qsame is None) else (qsame - prev_qsame)
            d_qretry = None if (qretry is None or prev_qretry is None) else (qretry - prev_qretry)

            retry_pct = None
            if d_qtot is not None and d_qtot > 0 and d_qretry is not None:
                retry_pct = (100.0 * float(d_qretry)) / float(d_qtot)

            event_msg = ""
            if fire_every > 0 and (poll_n % fire_every) == 0:
                event_msg = fire_vclock_test(vclock_test_ns)

            sec_s = str(sec) if sec is not None else "n/a"

            print(
                f"poll={poll_n:3d} "
                f"sec={sec_s:>5s} "
                f"pps={fmt_i64(pps, 6)} "
                f"dpps={fmt_i64_signed(d_pps, 4)} "
                f"raw32={fmt_i64(raw32, 12)} "
                f"d_raw32={fmt_i64_signed(d_raw32, 11)} "
                f"raw64={fmt_i64(raw64, 18)} "
                f"d_raw64={fmt_i64_signed(d_raw64, 12)} "
                f"gnss_ns={fmt_i64(gnss_ns, 18)} "
                f"d_gnss={fmt_i64_signed(d_gnss, 14)} "
                f"qtot={fmt_i64(qtot, 8)} "
                f"d_qtot={fmt_i64_signed(d_qtot, 8)} "
                f"qsame={fmt_i64(qsame, 8)} "
                f"d_qsame={fmt_i64_signed(d_qsame, 8)} "
                f"qretry={fmt_i64(qretry, 8)} "
                f"d_qretry={fmt_i64_signed(d_qretry, 8)} "
                f"retry%={fmt_pct(retry_pct, 6, 2)} "
                f"hi1={fmt_i64(q_hi1, 6)} "
                f"lo={fmt_i64(q_lo, 6)} "
                f"hi2={fmt_i64(q_hi2, 6)} "
                f"lo2={fmt_i64(q_lo2, 6)} "
                f"{event_msg}"
            )

            prev_pps = pps
            prev_raw32 = raw32
            prev_raw64 = raw64
            prev_gnss_ns = gnss_ns

            prev_qtot = qtot
            prev_qsame = qsame
            prev_qretry = qretry

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nstopped\n")


def main():
    interval = 1.0
    fire_every = 0
    vclock_test_ns = 100_000_000

    if len(sys.argv) >= 2:
        interval = float(sys.argv[1])
    if len(sys.argv) >= 3:
        fire_every = int(sys.argv[2])
    if len(sys.argv) >= 4:
        vclock_test_ns = int(sys.argv[3])

    run(
        interval=interval,
        fire_every=fire_every,
        vclock_test_ns=vclock_test_ns,
    )


if __name__ == "__main__":
    main()