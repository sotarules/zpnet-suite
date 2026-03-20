"""
ZPNet TimePop Counter Sanity Monitor

Polls TIMEPOP DIAG directly from the Teensy and performs a simple
sanity check on the core counter relationships.

It is intentionally dumb and external-facing:
we are not trying to prove the time model correct here, only to answer:

  • Is PPS advancing sensibly?
  • Is anchor_qtimer_at_pps advancing by about 20,000,000 raw counts/PPS?
  • Is anchor_dwt_at_pps advancing by about dwt_cycles_per_s/PPS?
  • Is diag_qtimer_now advancing at about 20 MHz raw?
  • Is diag_qtimer_now - anchor_qtimer_at_pps staying within a sane sub-second window?

This is useful when a more fundamental counter/configuration issue may
be poisoning higher-level TimePop logic.

Usage:
    python -m zpnet.tests.timepop_counter_sanity
    python -m zpnet.tests.timepop_counter_sanity 1.0
    .zt timepop_counter_sanity
    .zt timepop_counter_sanity 1.0
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

from zpnet.processes.processes import send_command


QTIMER_RAW_PER_SEC = 20_000_000
QTIMER_RAW_PER_PPS = 20_000_000
QTIMER_RAW_PER_NS = 1.0 / 50.0
NS_PER_RAW_TICK = 50

QTIMER_PCT_TOL = 2.0
DWT_PCT_TOL = 2.0
QTIMER_SINCE_ANCHOR_LIMIT = 22_000_000  # ~1.1 s raw


def u32_delta(new: int, old: int) -> int:
    return (new - old) & 0xFFFFFFFF


def pct_err(actual: float, expected: float) -> float:
    if expected == 0:
        return 0.0
    return ((actual - expected) / expected) * 100.0


@dataclass
class Sample:
    wall_time: float
    pps: int
    dwt_at_pps: int
    dwt_cycles_per_s: int
    qtimer_at_pps: int
    qtimer_now: int

    @staticmethod
    def from_payload(p: dict) -> "Sample":
        return Sample(
            wall_time=time.time(),
            pps=int(p.get("anchor_pps_count", 0)),
            dwt_at_pps=int(p.get("anchor_dwt_at_pps", 0)),
            dwt_cycles_per_s=int(p.get("anchor_dwt_cycles_per_s", 0)),
            qtimer_at_pps=int(p.get("anchor_qtimer_at_pps", 0)),
            qtimer_now=int(p.get("diag_qtimer_now", 0)),
        )


def run(interval: float = 1.0) -> None:
    print()
    print("  ZPNet TimePop Counter Sanity Monitor")
    print()
    print(
        f"  {'pps':>5s}"
        f"  │ {'qt_since':>10s}"
        f"  {'since_ms':>9s}"
        f"  {'since':>5s}"
        f"  │ {'dpps':>4s}"
        f"  {'dqt_anchor':>11s}"
        f"  {'qt%':>8s}"
        f"  {'qt':>5s}"
        f"  │ {'ddwt_anchor':>11s}"
        f"  {'dwt%':>8s}"
        f"  {'dwt':>5s}"
        f"  │ {'dqt_now':>10s}"
        f"  {'now%':>8s}"
        f"  {'now':>5s}"
    )
    print(
        f"  {'─'*5}"
        f"  │ {'─'*10}"
        f"  {'─'*9}"
        f"  {'─'*5}"
        f"  │ {'─'*4}"
        f"  {'─'*11}"
        f"  {'─'*8}"
        f"  {'─'*5}"
        f"  │ {'─'*11}"
        f"  {'─'*8}"
        f"  {'─'*5}"
        f"  │ {'─'*10}"
        f"  {'─'*8}"
        f"  {'─'*5}"
    )

    prev: Sample | None = None

    try:
        while True:
            try:
                resp = send_command(
                    machine="TEENSY",
                    subsystem="TIMEPOP",
                    command="DIAG",
                    args={},
                )
            except Exception as e:
                print(f"  command failed: {e}")
                time.sleep(interval)
                continue

            if not resp or not resp.get("success"):
                err_msg = resp.get("payload", {}).get("error", "unknown") if resp else "no response"
                print(f"  error: {err_msg}")
                time.sleep(interval)
                continue

            payload = resp.get("payload", {})
            s = Sample.from_payload(payload)

            qt_since_anchor = u32_delta(s.qtimer_now, s.qtimer_at_pps)
            qt_since_anchor_ms = (qt_since_anchor * NS_PER_RAW_TICK) / 1_000_000.0
            qt_since_ok = qt_since_anchor <= QTIMER_SINCE_ANCHOR_LIMIT

            if prev is None:
                print(
                    f"  {s.pps:>5d}"
                    f"  │ {qt_since_anchor:>10,d}"
                    f"  {qt_since_anchor_ms:>9.3f}"
                    f"  {'OK' if qt_since_ok else 'BAD':>5s}"
                    f"  │ {'--':>4s}"
                    f"  {'--':>11s}"
                    f"  {'--':>8s}"
                    f"  {'--':>5s}"
                    f"  │ {'--':>11s}"
                    f"  {'--':>8s}"
                    f"  {'--':>5s}"
                    f"  │ {'--':>10s}"
                    f"  {'--':>8s}"
                    f"  {'--':>5s}"
                )
                prev = s
                time.sleep(interval)
                continue

            dpps = s.pps - prev.pps
            dqt_anchor = u32_delta(s.qtimer_at_pps, prev.qtimer_at_pps)
            ddwt_anchor = u32_delta(s.dwt_at_pps, prev.dwt_at_pps)
            dqt_now = u32_delta(s.qtimer_now, prev.qtimer_now)

            wall_dt = s.wall_time - prev.wall_time

            exp_qt_anchor = dpps * QTIMER_RAW_PER_PPS
            exp_dwt_anchor = dpps * s.dwt_cycles_per_s
            exp_qt_now = wall_dt * QTIMER_RAW_PER_SEC

            qt_anchor_ok = dpps > 0 and abs(pct_err(dqt_anchor, exp_qt_anchor)) <= QTIMER_PCT_TOL
            dwt_anchor_ok = dpps > 0 and abs(pct_err(ddwt_anchor, exp_dwt_anchor)) <= DWT_PCT_TOL
            qt_now_ok = abs(pct_err(dqt_now, exp_qt_now)) <= QTIMER_PCT_TOL

            qt_anchor_pct = pct_err(dqt_anchor, exp_qt_anchor) if dpps > 0 else 0.0
            dwt_anchor_pct = pct_err(ddwt_anchor, exp_dwt_anchor) if dpps > 0 else 0.0
            qt_now_pct = pct_err(dqt_now, exp_qt_now)

            print(
                f"  {s.pps:>5d}"
                f"  │ {qt_since_anchor:>10,d}"
                f"  {qt_since_anchor_ms:>9.3f}"
                f"  {('OK' if qt_since_ok else 'BAD'):>5s}"
                f"  │ {dpps:>4d}"
                f"  {dqt_anchor:>11,d}"
                f"  {qt_anchor_pct:>+8.3f}"
                f"  {('OK' if qt_anchor_ok else 'BAD'):>5s}"
                f"  │ {ddwt_anchor:>11,d}"
                f"  {dwt_anchor_pct:>+8.3f}"
                f"  {('OK' if dwt_anchor_ok else 'BAD'):>5s}"
                f"  │ {dqt_now:>10,d}"
                f"  {qt_now_pct:>+8.3f}"
                f"  {('OK' if qt_now_ok else 'BAD'):>5s}"
            )

            prev = s
            time.sleep(interval)

    except KeyboardInterrupt:
        print()
        print("  stopped")
        print()


def main():
    interval = 1.0

    for arg in sys.argv[1:]:
        try:
            interval = float(arg)
        except ValueError:
            pass

    run(interval=interval)


if __name__ == "__main__":
    main()