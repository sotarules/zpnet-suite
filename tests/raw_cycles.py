"""
ZPNet Raw Cycles — raw endpoint vs firmware integrator, with GF-8802 side-by-side.

Reads TIMEBASE rows for a campaign and shows cycle-domain views alongside
the GF-8802 receiver's own compensatory diagnostics:

  raw_cycles   = dwt_isr_raw[N] - dwt_isr_raw[N-1]
                 Endpoint-to-endpoint, computed by subtracting consecutive
                 ISR-entry DWT captures.  The "what the chip actually did"
                 number.

  raw_res      = raw_cycles - nominal (1,008,000,000)
                 Residual versus expected 1.008 GHz CPU nominal.  Captures
                 the beat between the Teensy CPU crystal and the GF-8802's
                 disciplined 10 MHz output.

  tick_stddev  = per-tick RMS of the 1000 intervals inside that second,
                 from the firmware's Welford over the ring.  The actual
                 per-tick ISR-entry jitter.  Anomalously high tick_stddev
                 is usually the signature of a GNSS receiver phase event
                 or CPU substrate disturbance.

GF-8802 compensatory columns (from the gnss sub-dict):

  drift_ppb    = clock_drift_ppb
                 GF-8802's observed internal OCXO drift vs UTC (what
                 the receiver thinks its own oscillator is doing).

  freq_err     = freq_error_ppb
                 GF-8802's reported frequency error after discipline
                 (the "what's left uncorrected" number, should be small
                 under FINE_LOCK).

  pps_err_ns   = pps_timing_error_ns
                 GF-8802's reported PPS phase error vs UTC.  Spikes
                 here are what trigger the receiver's correction steps.

  pskip        = phase_skip
                 Counter.  Increments when the receiver issues a phase
                 correction to its PPS output.  The smoking gun for
                 visible compensation events in raw_res.

  mode         = freq_mode_name
                 FINE_LOCK, COARSE_LOCK, HOLDOVER, etc.  A mode change
                 is a major servo regime transition.

Also shown in summary:

  integ_cycles = dwt_effective_cycles_per_second (firmware SMA output).
                 Kept in the stats summary because it equals raw by
                 telescoping identity, but dropped from the row view.

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

# Threshold for flagging a raw_res step as a candidate "event" worth
# cross-referencing against GF-8802 diagnostics.  ~100 cycles at 1.008
# GHz is ~100 ns — well above normal second-to-second drift of ~4 cycles.
RAW_RES_STEP_THRESHOLD_CYCLES = 100


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
# Field access — fragment-aware and gnss-aware
# ---------------------------------------------------------------------

def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _gnss(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    g = root.get("gnss")
    return g if isinstance(g, dict) else {}


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


def _as_str(v: Any) -> Optional[str]:
    if v is None:
        return None
    try:
        return str(v)
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


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 2,
               signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        if signed:
            s = f"{v:+,.{decimals}f}"
        else:
            s = f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _fmt_str(v: Optional[str], width: int = 0) -> str:
    if v is None:
        s = "---"
    else:
        s = v[:width] if width else v
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
    print("Two worlds, side by side:")
    print("  Teensy side (what the chip measured):")
    print("    raw_cycles   = dwt_isr_raw[N] - dwt_isr_raw[N-1]")
    print("    raw_res      = raw_cycles - nominal")
    print("    tick_stddev  = per-tick RMS inside that second")
    print("  GF-8802 side (what the receiver thought):")
    print("    drift_ppb    = clock_drift_ppb (internal OCXO vs UTC)")
    print("    freq_err     = freq_error_ppb (residual after discipline)")
    print("    pps_err_ns   = pps_timing_error_ns (reported phase error)")
    print("    pskip        = phase_skip counter (event indicator)")
    print("    mode         = freq_mode_name (servo regime)")
    print()

    # Header.
    header = (
        f"{'pps':>6s}  "
        f"{'raw_cycles':>13s}  "
        f"{'raw_res':>9s}  "
        f"{'tick_stddev':>11s}  "
        f"{'drift_ppb':>10s}  "
        f"{'freq_err':>9s}  "
        f"{'pps_err_ns':>10s}  "
        f"{'pskip':>5s}  "
        f"{'mode':<10s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*13}  "
        f"{'─'*9}  "
        f"{'─'*11}  "
        f"{'─'*10}  "
        f"{'─'*9}  "
        f"{'─'*10}  "
        f"{'─'*5}  "
        f"{'─'*10}"
    )
    print(header)
    print(sep)

    # Welford accumulators — Teensy side.
    w_raw_cycles      = Welford()
    w_raw_res         = Welford()
    w_integ_cycles    = Welford()
    w_integ_res       = Welford()
    w_integ_minus_raw = Welford()
    w_tick_stddev     = Welford()
    w_tick_range      = Welford()
    w_tick_mean       = Welford()

    # Welford accumulators — GF-8802 side.
    w_drift_ppb       = Welford()
    w_freq_err_ppb    = Welford()
    w_pps_err_ns      = Welford()
    w_temp_c          = Welford()

    # Event detection and bookkeeping.
    step_events: List[Dict[str, Any]] = []
    pskip_events: List[Dict[str, Any]] = []
    mode_transitions: List[Dict[str, Any]] = []

    shown = 0
    gaps = 0
    prev_pps: Optional[int] = None
    prev_raw: Optional[int] = None
    prev_raw_res: Optional[int] = None
    prev_pskip: Optional[int] = None
    prev_mode: Optional[str] = None
    first_max_ever: Optional[int] = None
    last_max_ever: Optional[int] = None
    first_min_ever: Optional[int] = None
    last_min_ever: Optional[int] = None

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        gnss = _gnss(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        # Teensy-side fields.
        raw_now = _as_int(frag.get("pps_diag_dwt_isr_entry_raw"))
        integ_cycles = _as_int(frag.get("dwt_effective_cycles_per_second"))
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

        # GF-8802 fields.
        drift_ppb   = _as_float(gnss.get("clock_drift_ppb"))
        freq_err    = _as_float(gnss.get("freq_error_ppb"))
        pps_err_ns  = _as_int(gnss.get("pps_timing_error_ns"))
        pskip       = _as_int(gnss.get("phase_skip"))
        mode        = _as_str(gnss.get("freq_mode_name"))
        temp_c      = _as_float(gnss.get("temperature_c"))

        # Detect gaps.
        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps:,} → {pps:,} ---")
            prev_raw = None
            prev_raw_res = None

        prev_pps = pps

        # Compute raw endpoint delta + residual.
        raw_cycles: Optional[int] = None
        raw_res: Optional[int] = None
        if raw_now is not None and prev_raw is not None:
            raw_cycles = _u32_diff(raw_now, prev_raw)
            raw_res = raw_cycles - DWT_EXPECTED_PER_PPS

        # Integrator residual + cross-check.
        integ_res: Optional[int] = None
        integ_minus_raw: Optional[int] = None
        if integ_cycles is not None:
            integ_res = integ_cycles - DWT_EXPECTED_PER_PPS
            if raw_cycles is not None:
                integ_minus_raw = integ_cycles - raw_cycles

        # Event detection: raw_res step.
        if (raw_res is not None and prev_raw_res is not None and
                abs(raw_res - prev_raw_res) >= RAW_RES_STEP_THRESHOLD_CYCLES):
            step_events.append({
                "pps": pps,
                "prev_raw_res": prev_raw_res,
                "raw_res": raw_res,
                "delta": raw_res - prev_raw_res,
                "tick_stddev": tick_stddev,
                "drift_ppb": drift_ppb,
                "freq_err": freq_err,
                "pps_err_ns": pps_err_ns,
                "pskip": pskip,
                "mode": mode,
            })

        # Event detection: phase_skip increment.
        if (pskip is not None and prev_pskip is not None and
                pskip > prev_pskip):
            pskip_events.append({
                "pps": pps,
                "prev_pskip": prev_pskip,
                "pskip": pskip,
                "delta": pskip - prev_pskip,
                "raw_res": raw_res,
            })

        # Event detection: mode change.
        if (mode is not None and prev_mode is not None and
                mode != prev_mode):
            mode_transitions.append({
                "pps": pps,
                "from": prev_mode,
                "to": mode,
                "raw_res": raw_res,
            })

        # Update Welford — Teensy side.
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

        # Update Welford — GF-8802 side.
        if drift_ppb is not None:
            w_drift_ppb.update(drift_ppb)
        if freq_err is not None:
            w_freq_err_ppb.update(freq_err)
        if pps_err_ns is not None:
            w_pps_err_ns.update(float(pps_err_ns))
        if temp_c is not None:
            w_temp_c.update(temp_c)

        # Track ever-min/max progression.
        if max_ever is not None:
            if first_max_ever is None:
                first_max_ever = max_ever
            last_max_ever = max_ever
        if min_ever is not None and min_ever > 0:
            if first_min_ever is None:
                first_min_ever = min_ever
            last_min_ever = min_ever

        if raw_now is not None:
            prev_raw = raw_now
        if raw_res is not None:
            prev_raw_res = raw_res
        if pskip is not None:
            prev_pskip = pskip
        if mode is not None:
            prev_mode = mode

        # Print row.
        print(
            f"{pps:>6d}  "
            f"{_fmt_int(raw_cycles, 13)}  "
            f"{_fmt_int(raw_res, 9, signed=True)}  "
            f"{_fmt_float(tick_stddev, 11, decimals=2)}  "
            f"{_fmt_float(drift_ppb, 10, decimals=3, signed=True)}  "
            f"{_fmt_float(freq_err, 9, decimals=2, signed=True)}  "
            f"{_fmt_int(pps_err_ns, 10, signed=True)}  "
            f"{_fmt_int(pskip, 5)}  "
            f"{_fmt_str(mode, 10)}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()

    # -----------------------------------------------------------------
    # Teensy-side endpoint statistics
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Teensy-side endpoint statistics")
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
    # Integ vs raw cross-check
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
    # GF-8802 receiver diagnostics
    # -----------------------------------------------------------------
    print("═" * 70)
    print("GF-8802 receiver diagnostics")
    print("  (what the receiver thought about its own world)")
    print("═" * 70)
    print()

    if w_drift_ppb.n >= 2:
        _print_welford("clock_drift_ppb (internal OCXO drift vs UTC)",
                       w_drift_ppb, "ppb", decimals=3)
    if w_freq_err_ppb.n >= 2:
        _print_welford("freq_error_ppb (residual after discipline)",
                       w_freq_err_ppb, "ppb", decimals=3)
    if w_pps_err_ns.n >= 2:
        _print_welford("pps_timing_error_ns (reported phase error)",
                       w_pps_err_ns, "ns", decimals=1)
    if w_temp_c.n >= 2:
        _print_welford("temperature_c (GF-8802 internal)",
                       w_temp_c, "°C", decimals=2)

    # -----------------------------------------------------------------
    # Cross-correlation: raw_res steps vs GF-8802 activity
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Events — raw_res steps, phase_skip increments, mode changes")
    print("═" * 70)
    print()

    if step_events:
        print(f"  raw_res steps ≥ {RAW_RES_STEP_THRESHOLD_CYCLES} cycles:")
        print(f"    (|Δraw_res| threshold catches servo-step signatures)")
        print()
        for ev in step_events[:50]:
            pskip_s = f"{ev['pskip']}" if ev['pskip'] is not None else "---"
            mode_s = ev['mode'] if ev['mode'] else "---"
            print(
                f"    pps={ev['pps']:>6d}  "
                f"Δraw_res={ev['delta']:+7d}  "
                f"(from {ev['prev_raw_res']:+6d} to {ev['raw_res']:+6d})  "
                f"tick_std={ev['tick_stddev']:6.2f}  "
                f"drift_ppb={ev['drift_ppb']:+8.3f}  "
                f"pps_err={ev['pps_err_ns'] if ev['pps_err_ns'] is not None else 0:+5d}ns  "
                f"pskip={pskip_s}  "
                f"mode={mode_s}"
            )
        if len(step_events) > 50:
            print(f"    ... and {len(step_events) - 50:,} more")
        print()
    else:
        print(f"  No raw_res steps ≥ {RAW_RES_STEP_THRESHOLD_CYCLES} cycles detected.")
        print()

    if pskip_events:
        print(f"  phase_skip counter increments:")
        print(f"    (GF-8802 explicitly stepped its PPS output)")
        print()
        for ev in pskip_events[:20]:
            raw_res_s = (f"{ev['raw_res']:+6d}"
                         if ev['raw_res'] is not None else "---")
            print(
                f"    pps={ev['pps']:>6d}  "
                f"pskip: {ev['prev_pskip']} → {ev['pskip']}  "
                f"(Δ={ev['delta']:+d})  raw_res={raw_res_s}"
            )
        print()
    else:
        print("  No phase_skip increments.  (receiver did not admit to ")
        print("  stepping its PPS output during this campaign — any servo ")
        print("  activity was smooth frequency adjustment, not phase jump)")
        print()

    if mode_transitions:
        print("  freq_mode transitions:")
        print()
        for ev in mode_transitions:
            raw_res_s = (f"{ev['raw_res']:+6d}"
                         if ev['raw_res'] is not None else "---")
            print(
                f"    pps={ev['pps']:>6d}  "
                f"{ev['from']} → {ev['to']}  raw_res={raw_res_s}"
            )
        print()
    else:
        print("  No freq_mode transitions.  (servo regime was stable)")
        print()

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

    print(f"  raw_res stddev:        {w_raw_res.stddev:9.3f} cycles")
    print(f"  raw_res range:         {w_raw_res.max_val - w_raw_res.min_val:9.1f} cycles")
    if w_integ_res.n >= 2:
        print(f"  integ_res stddev:      {w_integ_res.stddev:9.3f} cycles")
    if w_drift_ppb.n >= 2:
        print(f"  drift_ppb stddev:      {w_drift_ppb.stddev:9.3f} ppb")
        # Convert drift_ppb to cycles/sec at 1.008 GHz.
        drift_as_cycles = w_drift_ppb.stddev * (DWT_EXPECTED_PER_PPS / 1e9)
        print(f"  drift_ppb (in cycles): {drift_as_cycles:9.3f} cycles")
    if w_pps_err_ns.n >= 2:
        print(f"  pps_err_ns stddev:     {w_pps_err_ns.stddev:9.1f} ns")
    print()

    n_steps = len(step_events)
    n_pskips = len(pskip_events)
    n_modes = len(mode_transitions)

    if n_steps == 0 and n_pskips == 0 and n_modes == 0:
        print("  Calm campaign.  No raw_res step events, no phase_skips,")
        print("  no mode changes.  The beat between Teensy CPU crystal and")
        print("  GF-8802's disciplined 10 MHz was smooth throughout.")
        print()
        print("  raw_res's slow drift (if present) is the Teensy CPU crystal")
        print("  aging/thermal response — the GF-8802's 10 MHz output is")
        print("  locked to UTC, so any slow walk in raw_res is local.")
    elif n_pskips > 0:
        print(f"  {n_pskips} phase_skip event(s) detected.  These are explicit")
        print("  admissions by the GF-8802 that it stepped its PPS output")
        print("  to correct accumulated phase.  Cross-reference with raw_res")
        print("  steps above to see the event's signature on the Teensy side.")
        print()
        print("  Expected pattern: phase_skip increments coincide with or")
        print("  precede by 1 PPS a corresponding step in raw_res.")
    elif n_steps > 0:
        print(f"  {n_steps} raw_res step event(s) without matching phase_skip.")
        print("  The receiver did not admit to stepping, so these are either:")
        print("    • Smooth frequency corrections (receiver held a different")
        print("      rate for several seconds, walking phase gradually)")
        print("    • CPU-side disturbances (thermal, DMA, cache eviction)")
        print("  Check tick_stddev in the step events above — if elevated,")
        print("  the substrate was disturbed regardless of cause.")

    if n_modes > 0:
        print()
        print(f"  {n_modes} mode transition(s) — these are major servo regime")
        print("  changes.  HOLDOVER indicates the receiver lost GNSS lock")
        print("  and was free-running on its internal OCXO.")

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