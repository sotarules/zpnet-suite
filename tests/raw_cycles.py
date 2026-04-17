"""
ZPNet Raw Cycles — honest cross-check of integrator vs simple subtraction

Reads TIMEBASE rows for a campaign and asks the question:

    Does the rolling integrator's smoothed synthetic boundary differ
    meaningfully from what a simple "subtract this PPS's raw DWT capture
    from the previous PPS's raw DWT capture" would yield?

If yes, the integration is doing real work — averaging away QuadTimer
compare-path jitter that a single capture would otherwise inject into
every measurement.

If no, the integration is bookkeeping over a jitter-free substrate, and
the synthetic boundary is structurally equivalent to the raw capture.
This would prove that the entire ring-buffer apparatus is unnecessary
for VCLOCK (and by extension for all three lanes, since they use the
same QuadTimer compare mechanism).

The two quantities being compared, per PPS:

  integ_diff  = dwt_cycle_count_at_pps[N] - dwt_cycle_count_at_pps[N-1]
                ↑ The integrator's synthetic boundary delta

  raw_diff    = pps_diag_dwt_isr_entry_raw[N] - pps_diag_dwt_isr_entry_raw[N-1]
                ↑ What simple subtraction of the actual ISR-entry DWT
                  capture would yield (no integration, no smoothing)

  integ_minus_raw = integ_diff - raw_diff
                ↑ The diagnostic that answers the question

Plus the GPIO PPS witness for context.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


DWT_EXPECTED_PER_PPS = 1_008_000_000  # nominal CPU cycles per second (1.008 GHz)


# ---------------------------------------------------------------------
# Welford online accumulator
# ---------------------------------------------------------------------

class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        if x < self.min_val:
            self.min_val = x
        if x > self.max_val:
            self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


# ---------------------------------------------------------------------
# DB access
# ---------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::bigint ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        result.append(payload)

    return result


# ---------------------------------------------------------------------
# Field access — fragment-aware
# ---------------------------------------------------------------------

def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    if not isinstance(inner, dict):
        return {}
    frag = inner.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


# ---------------------------------------------------------------------
# DWT u32 wrap-aware delta
# ---------------------------------------------------------------------

def _u32_diff(curr: int, prev: int) -> int:
    """Unsigned delta of two u32 DWT samples, accounting for wrap."""
    return (curr - prev) & 0xFFFFFFFF


# ---------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------

def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _print_welford(name: str, w: Welford, unit: str = "") -> None:
    if w.n < 2:
        return
    suffix = f" {unit}" if unit else ""
    print(f"  {name}")
    print(f"    n       = {w.n:,}")
    print(f"    mean    = {w.mean:+,.3f}{suffix}")
    print(f"    stddev  = {w.stddev:,.3f}{suffix}")
    print(f"    stderr  = {w.stderr:,.3f}{suffix}")
    print(f"    min     = {w.min_val:+,.0f}{suffix}")
    print(f"    max     = {w.max_val:+,.0f}{suffix}")
    print(f"    range   = {w.max_val - w.min_val:,.0f}{suffix}")
    print()


# ---------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows):,} rows)")
    print(f"DWT_EXPECTED_PER_PPS = {DWT_EXPECTED_PER_PPS:,}")
    print()
    print("Cross-check question: does the integrator's smoothed synthetic")
    print("boundary differ from a simple subtraction of raw ISR-entry DWT?")
    print()

    # Header.
    header = (
        f"{'pps':>6s}  "
        f"{'integ_diff':>13s}  "
        f"{'raw_diff':>13s}  "
        f"{'integ-raw':>10s}  "
        f"{'integ_res':>9s}  "
        f"{'raw_res':>9s}  "
        f"{'gpio_ns':>14s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*13}  "
        f"{'─'*13}  "
        f"{'─'*10}  "
        f"{'─'*9}  "
        f"{'─'*9}  "
        f"{'─'*14}"
    )
    print(header)
    print(sep)

    # Welford accumulators.
    w_integ_diff       = Welford()  # synthetic boundary delta
    w_raw_diff         = Welford()  # simple-subtraction equivalent
    w_integ_minus_raw  = Welford()  # THE answer — does integration matter?
    w_integ_res        = Welford()  # integ_diff vs nominal
    w_raw_res          = Welford()  # raw_diff vs nominal
    w_gpio             = Welford()
    w_gpio_drift       = Welford()  # vs running mean

    samples = []
    shown = 0
    gaps = 0
    prev_pps: Optional[int] = None
    prev_integ: Optional[int] = None
    prev_raw: Optional[int] = None

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        # Both timing surfaces.
        integ_now = _as_int(frag.get("dwt_cycle_count_at_pps"))
        raw_now = _as_int(frag.get("pps_diag_dwt_isr_entry_raw"))
        gpio_ns = _as_int(frag.get("pps_diag_gpio_minus_synthetic_ns"))

        # Detect gaps.
        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps:,} → {pps:,} ---")
            # Reset deltas across a gap.
            prev_integ = None
            prev_raw = None
        prev_pps = pps

        # Compute deltas (only if we have a previous sample).
        integ_diff = None
        raw_diff = None
        integ_minus_raw = None
        integ_res = None
        raw_res = None

        if integ_now is not None and prev_integ is not None:
            integ_diff = _u32_diff(integ_now, prev_integ)
            integ_res = integ_diff - DWT_EXPECTED_PER_PPS

        if raw_now is not None and prev_raw is not None:
            raw_diff = _u32_diff(raw_now, prev_raw)
            raw_res = raw_diff - DWT_EXPECTED_PER_PPS

        if integ_diff is not None and raw_diff is not None:
            integ_minus_raw = integ_diff - raw_diff

        # Update Welford.
        if integ_diff is not None:
            w_integ_diff.update(float(integ_diff))
        if raw_diff is not None:
            w_raw_diff.update(float(raw_diff))
        if integ_minus_raw is not None:
            w_integ_minus_raw.update(float(integ_minus_raw))
        if integ_res is not None:
            w_integ_res.update(float(integ_res))
        if raw_res is not None:
            w_raw_res.update(float(raw_res))
        if gpio_ns is not None:
            w_gpio.update(float(gpio_ns))

        samples.append({"pps": pps, "gpio_ns": gpio_ns})

        # Save state for next iteration.
        if integ_now is not None:
            prev_integ = integ_now
        if raw_now is not None:
            prev_raw = raw_now

        # Print row.
        print(
            f"{pps:>6d}  "
            f"{_fmt_int(integ_diff, 13)}  "
            f"{_fmt_int(raw_diff, 13)}  "
            f"{_fmt_int(integ_minus_raw, 10, signed=True)}  "
            f"{_fmt_int(integ_res, 9, signed=True)}  "
            f"{_fmt_int(raw_res, 9, signed=True)}  "
            f"{_fmt_int(gpio_ns, 14, signed=True)}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    # Drift-residual pass for GPIO.
    gpio_mean = w_gpio.mean if w_gpio.n > 0 else 0.0
    for s in samples:
        if s["gpio_ns"] is not None:
            w_gpio_drift.update(float(s["gpio_ns"]) - gpio_mean)

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()
    print("─" * 70)
    print("Summary statistics")
    print("─" * 70)
    print()

    print("Synthetic boundary deltas (integrator output):")
    _print_welford("integ_diff (cycles per synthetic second)",
                   w_integ_diff, "cycles")
    _print_welford("integ_res (integ_diff - nominal)",
                   w_integ_res, "cycles")
    print()

    print("Raw ISR-entry DWT deltas (simple subtraction equivalent):")
    _print_welford("raw_diff (cycles between consecutive ISR-entry captures)",
                   w_raw_diff, "cycles")
    _print_welford("raw_res (raw_diff - nominal)",
                   w_raw_res, "cycles")
    print()

    print("THE CROSS-CHECK — does integration buy us anything?")
    _print_welford("integ_minus_raw = integ_diff - raw_diff",
                   w_integ_minus_raw, "cycles")
    print()

    print("GPIO PPS witness:")
    _print_welford("gpio_ns (raw, includes DC bias)", w_gpio, "ns")
    _print_welford("gpio_drift_ns (witness - running mean)", w_gpio_drift, "ns")

    # Headline interpretation.
    print("─" * 70)
    print("Interpretation")
    print("─" * 70)
    print()

    if w_integ_minus_raw.n >= 2:
        stddev = w_integ_minus_raw.stddev
        rng = w_integ_minus_raw.max_val - w_integ_minus_raw.min_val
        mean = w_integ_minus_raw.mean

        print(f"  integ_minus_raw stddev: {stddev:.2f} cycles "
              f"({stddev:.2f} ns)")
        print(f"  integ_minus_raw range:  {rng:.0f} cycles peak-to-peak")
        print(f"  integ_minus_raw mean:   {mean:+.2f} cycles "
              f"(constant ISR-entry latency, expected)")
        print()

        if stddev < 5.0:
            print("  CONCLUSION: integration adds essentially nothing.")
            print("  The QuadTimer compare path is jitter-free at the cycle")
            print("  level. Simple subtraction of raw ISR-entry DWT captures")
            print("  yields the same rate measurement as the SMA over 1000")
            print("  intervals. The ring buffer is bookkeeping, not filtering.")
        elif stddev < 50.0:
            print("  CONCLUSION: integration provides modest smoothing.")
            print("  Per-tick QuadTimer compare jitter is small but nonzero.")
            print("  The SMA averages it down by ~31x; the residual difference")
            print("  is what the integrator is genuinely filtering.")
        else:
            print("  CONCLUSION: integration is doing real work.")
            print("  Per-tick QuadTimer compare jitter is significant. The")
            print("  integrator is averaging away substantial measurement")
            print("  noise that simple subtraction would expose directly.")

    print()


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()