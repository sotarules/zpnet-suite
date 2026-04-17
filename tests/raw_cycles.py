"""
ZPNet Raw Cycles — raw endpoint vs firmware integrator, side by side.

Reads TIMEBASE rows for a campaign and shows four cycle-domain views
alongside per-tick and witness diagnostics:

  raw_cycles   = dwt_isr_raw[N] - dwt_isr_raw[N-1]
                 Endpoint-to-endpoint, instantaneous cycle count between
                 consecutive PPS ISR captures.  The "what the chip
                 actually did" number.

  raw_res      = raw_cycles - nominal (1,008,000,000)
                 Residual versus the expected 1.008 GHz CPU nominal.

  integ_cycles = dwt_effective_cycles_per_second, the firmware
                 integrator's published rate as consumed by alpha's
                 pps_callback.  This is the SMA over the most recent
                 1000 inter-tick intervals.

  integ_res    = integ_cycles - nominal

Important note on what this actually compares:

  At the VCLOCK boundary ISR, the ring-SMA equals the raw endpoint
  delta by telescoping identity — the 1000 intervals in the ring
  sum to exactly dwt_isr_raw[N] - dwt_isr_raw[N-1].  They are the
  SAME number at that instant.

  integ_cycles (read by beta a few ms later, via dwt_effective_
  cycles_per_second) may differ by a handful of cycles because by
  the time beta reads it, 1-5 new 1 kHz ticks have fired and the
  ring has slid forward by that much.  Those small deltas are
  dispatch latency, not smoothing.

  The firmware integrator does NOT smooth at the 1-Hz timescale
  this analyzer samples at.  Its SMA window is exactly 1 second
  wide, so at any PPS-aligned sampling moment the SMA output
  equals the raw endpoint.  If you want to see what real time-
  domain smoothing would look like, apply it Python-side to raw_res.

Also shown:

  tick_stddev  = per-PPS RMS of the 1000 intervals inside that second,
                 from the firmware's Welford over the ring.  This is
                 the actual per-tick ISR-entry jitter.
  gpio_ns      = GPIO ISR entry minus VCLOCK synthetic boundary,
                 differential witness.

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


def _as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
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


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 2) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _print_welford(name: str, w: Welford, unit: str = "",
                   decimals: int = 3) -> None:
    if w.n < 2:
        return
    suffix = f" {unit}" if unit else ""
    print(f"  {name}")
    print(f"    n       = {w.n:,}")
    print(f"    mean    = {w.mean:+,.{decimals}f}{suffix}")
    print(f"    stddev  = {w.stddev:,.{decimals}f}{suffix}")
    print(f"    stderr  = {w.stderr:,.{decimals}f}{suffix}")
    print(f"    min     = {w.min_val:+,.{decimals}f}{suffix}")
    print(f"    max     = {w.max_val:+,.{decimals}f}{suffix}")
    print(f"    range   = {w.max_val - w.min_val:,.{decimals}f}{suffix}")
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
    print("Four views of the substrate:")
    print("  raw_cycles   = dwt_isr_raw[N] - dwt_isr_raw[N-1]")
    print("                 Endpoint-to-endpoint, computed by subtracting")
    print("                 consecutive ISR-entry DWT captures.")
    print("  raw_res      = raw_cycles - nominal")
    print("  integ_cycles = dwt_effective_cycles_per_second")
    print("                 Firmware integrator's SMA output as consumed")
    print("                 by alpha at PPS time.  See interpretation")
    print("                 section re: its relation to raw_cycles.")
    print("  integ_res    = integ_cycles - nominal")
    print("  tick_stddev  = per-tick RMS of the 1000 intervals")
    print("  gpio_ns      = GPIO ISR vs VCLOCK synthetic differential")
    print()

    # Header.
    header = (
        f"{'pps':>6s}  "
        f"{'raw_cycles':>13s}  "
        f"{'raw_res':>9s}  "
        f"{'integ_cycles':>13s}  "
        f"{'integ_res':>9s}  "
        f"{'tick_stddev':>11s}  "
        f"{'gpio_ns':>14s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*13}  "
        f"{'─'*9}  "
        f"{'─'*13}  "
        f"{'─'*9}  "
        f"{'─'*11}  "
        f"{'─'*14}"
    )
    print(header)
    print(sep)

    # Welford accumulators.
    w_raw_cycles      = Welford()  # dwt_isr_raw delta
    w_raw_res         = Welford()  # raw_cycles - nominal
    w_integ_cycles    = Welford()  # dwt_effective_cycles_per_second
    w_integ_res       = Welford()  # integ_cycles - nominal
    w_integ_minus_raw = Welford()  # integ_cycles - raw_cycles (dispatch-latency artifact)
    w_tick_stddev     = Welford()  # per-PPS tick stddev
    w_tick_range      = Welford()
    w_tick_mean       = Welford()
    w_gpio            = Welford()
    w_gpio_drift      = Welford()

    samples = []
    shown = 0
    gaps = 0
    prev_pps: Optional[int] = None
    prev_raw: Optional[int] = None
    first_max_ever: Optional[int] = None
    last_max_ever: Optional[int] = None
    first_min_ever: Optional[int] = None
    last_min_ever: Optional[int] = None

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        # Raw ISR-entry capture (the endpoint surface).
        raw_now = _as_int(frag.get("pps_diag_dwt_isr_entry_raw"))

        # Firmware integrator's rate, as consumed by beta at PPS time.
        integ_cycles = _as_int(frag.get("dwt_effective_cycles_per_second"))

        # Per-tick stats.
        tick_stddev = _as_float(
            frag.get("pps_diag_integrator_interval_window_stddev_cycles"))
        tick_mean = _as_float(
            frag.get("pps_diag_integrator_interval_window_mean_cycles"))
        tick_min = _as_int(
            frag.get("pps_diag_integrator_interval_window_min_cycles"))
        tick_max = _as_int(
            frag.get("pps_diag_integrator_interval_window_max_cycles"))
        max_ever = _as_int(
            frag.get("pps_diag_integrator_interval_max_ever_cycles"))
        min_ever = _as_int(
            frag.get("pps_diag_integrator_interval_min_ever_cycles"))

        tick_range: Optional[int] = None
        if tick_min is not None and tick_max is not None:
            tick_range = tick_max - tick_min

        gpio_ns = _as_int(frag.get("pps_diag_gpio_minus_synthetic_ns"))

        # Detect gaps.
        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps:,} → {pps:,} ---")
            prev_raw = None
        prev_pps = pps

        # Compute raw endpoint delta + residual.
        raw_cycles: Optional[int] = None
        raw_res: Optional[int] = None
        if raw_now is not None and prev_raw is not None:
            raw_cycles = _u32_diff(raw_now, prev_raw)
            raw_res = raw_cycles - DWT_EXPECTED_PER_PPS

        # Integrator residual.
        integ_res: Optional[int] = None
        if integ_cycles is not None:
            integ_res = integ_cycles - DWT_EXPECTED_PER_PPS

        # Integ vs raw cross-check (captures dispatch-latency drift).
        integ_minus_raw: Optional[int] = None
        if integ_cycles is not None and raw_cycles is not None:
            integ_minus_raw = integ_cycles - raw_cycles

        # Update Welford.
        if raw_cycles is not None:
            w_raw_cycles.update(float(raw_cycles))
        if raw_res is not None:
            w_raw_res.update(float(raw_res))
        if integ_cycles is not None:
            w_integ_cycles.update(float(integ_cycles))
        if integ_res is not None:
            w_integ_res.update(float(integ_res))
        if integ_minus_raw is not None:
            w_integ_minus_raw.update(float(integ_minus_raw))
        if tick_stddev is not None:
            w_tick_stddev.update(tick_stddev)
        if tick_range is not None:
            w_tick_range.update(float(tick_range))
        if tick_mean is not None:
            w_tick_mean.update(tick_mean)
        if gpio_ns is not None:
            w_gpio.update(float(gpio_ns))

        # Track ever-min/max progression across the campaign.
        if max_ever is not None:
            if first_max_ever is None:
                first_max_ever = max_ever
            last_max_ever = max_ever
        if min_ever is not None and min_ever > 0:
            if first_min_ever is None:
                first_min_ever = min_ever
            last_min_ever = min_ever

        samples.append({"pps": pps, "gpio_ns": gpio_ns})

        if raw_now is not None:
            prev_raw = raw_now

        # Print row.
        print(
            f"{pps:>6d}  "
            f"{_fmt_int(raw_cycles, 13)}  "
            f"{_fmt_int(raw_res, 9, signed=True)}  "
            f"{_fmt_int(integ_cycles, 13)}  "
            f"{_fmt_int(integ_res, 9, signed=True)}  "
            f"{_fmt_float(tick_stddev, 11, decimals=2)}  "
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

    # -----------------------------------------------------------------
    # Raw endpoint statistics
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Raw endpoint statistics")
    print("  (consecutive dwt_isr_raw captures, subtracted)")
    print("═" * 70)
    print()
    _print_welford("raw_cycles (cycles per second between consecutive PPS)",
                   w_raw_cycles, "cycles")
    _print_welford("raw_res    (raw_cycles - nominal)",
                   w_raw_res, "cycles")

    # -----------------------------------------------------------------
    # Integrator rate statistics
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Integrator rate statistics")
    print("  (firmware SMA output, read by alpha at PPS time)")
    print("═" * 70)
    print()
    if w_integ_cycles.n == 0:
        print("  (dwt_effective_cycles_per_second not present in fragment)")
        print()
    else:
        _print_welford("integ_cycles (firmware SMA over last 1000 intervals)",
                       w_integ_cycles, "cycles")
        _print_welford("integ_res    (integ_cycles - nominal)",
                       w_integ_res, "cycles")

    # -----------------------------------------------------------------
    # Integ vs raw cross-check (dispatch-latency indicator)
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Integ vs raw cross-check")
    print("  (nonzero values indicate ring slippage during dispatch")
    print("   latency between VCLOCK ISR and beta's pps_callback)")
    print("═" * 70)
    print()
    if w_integ_minus_raw.n > 0:
        _print_welford("integ_cycles - raw_cycles",
                       w_integ_minus_raw, "cycles")

    # -----------------------------------------------------------------
    # Per-tick interval statistics
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Per-tick interval statistics")
    print("  (what's REALLY happening at 1 kHz inside each second)")
    print("═" * 70)
    print()

    if w_tick_stddev.n == 0:
        print("  (no per-tick fields found in fragment)")
        print()
    else:
        _print_welford(
            "tick_stddev (per-PPS RMS of the 1000 intervals in that second)",
            w_tick_stddev, "cycles", decimals=2)
        _print_welford(
            "tick_range (per-PPS peak-to-peak of those 1000 intervals)",
            w_tick_range, "cycles", decimals=0)
        _print_welford(
            "tick_mean (per-PPS mean of those 1000 intervals)",
            w_tick_mean, "cycles", decimals=3)

    # -----------------------------------------------------------------
    # All-time extremes
    # -----------------------------------------------------------------
    print("═" * 70)
    print("All-time interval extremes (cumulative since baseline)")
    print("═" * 70)
    print()

    if first_max_ever is not None and last_max_ever is not None:
        grew = last_max_ever - first_max_ever
        print(f"  tick_max_ever:")
        print(f"    first seen in campaign = {first_max_ever:,} cycles")
        print(f"    last seen  in campaign = {last_max_ever:,} cycles")
        print(f"    growth during campaign = {grew:+,} cycles")
        if grew > 0:
            print(f"    → at least one new max excursion occurred this run")
        print()
    if first_min_ever is not None and last_min_ever is not None:
        shrank = first_min_ever - last_min_ever
        print(f"  tick_min_ever:")
        print(f"    first seen in campaign = {first_min_ever:,} cycles")
        print(f"    last seen  in campaign = {last_min_ever:,} cycles")
        print(f"    shrink during campaign = {shrank:+,} cycles")
        if shrank > 0:
            print(f"    → at least one new min excursion occurred this run")
        print()

    # -----------------------------------------------------------------
    # GPIO witness
    # -----------------------------------------------------------------
    print("═" * 70)
    print("GPIO PPS witness (differential: GPIO ISR vs VCLOCK synthetic)")
    print("═" * 70)
    print()

    _print_welford("gpio_ns (raw, includes DC bias from CH3 bootstrap phase)",
                   w_gpio, "ns", decimals=3)
    _print_welford("gpio_drift_ns (witness - campaign mean)",
                   w_gpio_drift, "ns", decimals=3)

    # -----------------------------------------------------------------
    # Interpretation
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Interpretation")
    print("═" * 70)
    print()

    if w_raw_cycles.n < 2:
        print("  (insufficient data for comparative interpretation)")
        print()
        return

    print(f"  raw_res stddev:   {w_raw_res.stddev:9.3f} cycles")
    if w_integ_res.n >= 2:
        print(f"  integ_res stddev: {w_integ_res.stddev:9.3f} cycles")
        if w_integ_minus_raw.n >= 2:
            imr_std = w_integ_minus_raw.stddev
            imr_mean = w_integ_minus_raw.mean
            imr_max_abs = max(abs(w_integ_minus_raw.min_val),
                              abs(w_integ_minus_raw.max_val))
            print(f"  integ-raw mean:   {imr_mean:+9.3f} cycles")
            print(f"  integ-raw stddev: {imr_std:9.3f} cycles")
            print(f"  integ-raw |max|:  {imr_max_abs:9.3f} cycles")
    print()

    # Narrative.
    if w_integ_res.n == 0:
        print("  Only raw_cycles available.  Add dwt_effective_cycles_per_second")
        print("  to the fragment for the integrator side of the picture.")
    elif w_integ_minus_raw.n >= 2 and w_integ_minus_raw.stddev < 20.0:
        print("  raw_cycles and integ_cycles are essentially identical, as")
        print("  expected.  The firmware integrator's SMA window is exactly")
        print("  1 second wide, so at any PPS-aligned sampling moment the")
        print("  SMA sum equals the raw endpoint delta by telescoping")
        print("  identity — the same reason integ_minus_raw=0 in the earlier")
        print("  cross-check.  The small nonzero integ-raw values you may")
        print("  see are dispatch-latency artifacts: beta reads the rate a")
        print("  few ms after the VCLOCK boundary ISR fires, during which")
        print("  1-5 new ticks have slid the ring forward by that much.")
        print()
        print("  The firmware integrator does NOT smooth at the 1-Hz")
        print("  timescale this analyzer samples at.  What it DOES smooth")
        print("  is the per-tick noise visible in tick_stddev — but that")
        print("  smoothing produces the ring_sum (= raw_cycles by telescoping)")
        print("  as output, which is what's in integ_cycles here.")
        print()
        print("  To see genuine time-domain smoothing, apply a Python-side")
        print("  multi-second EMA or rolling mean to raw_res and compare")
        print("  its stddev to raw_res's stddev.  Order-of-magnitude result")
        print("  for TEMPEST: smoothing over N seconds reduces stddev by")
        print("  ~sqrt(N), converging to the DAC-Welford floor over a full")
        print("  campaign.")
    else:
        print("  integ_cycles and raw_cycles differ by more than expected")
        print("  for pure dispatch-latency drift.  Investigate whether the")
        print("  beta pps_callback is running unusually late, or whether")
        print("  the integrator's ring has a stability issue.")

    print()

    # Heuristic flag for rare excursions.
    if last_max_ever is not None and last_max_ever > 2_000_000:
        print(f"  ⚠ tick_max_ever is {last_max_ever:,} cycles — that's a")
        print(f"    ~{last_max_ever / 1_000_000:.2f} ms single-interval excursion,")
        print(f"    far above the nominal 1 ms.  Worth investigating what")
        print(f"    blocked this lane's ISR for that long.")
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