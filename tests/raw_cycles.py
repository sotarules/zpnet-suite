"""
ZPNet Raw Cycles — Teensy DWT rate vs GF-8802 receiver diagnostics.

Reads TIMEBASE rows for a campaign and shows the honest one-second-
subtraction DWT rate alongside the GF-8802 receiver's compensatory
diagnostics:

  dwt_cycles   = dwt_cycle_count_between_pps
                 The one-second subtraction of consecutive VCLOCK
                 one-second-event DWT captures, computed in alpha's
                 vclock_callback.  Post rolling-integrator removal,
                 this is the authoritative per-second DWT delta —
                 no synthesis, no SMA, no integrator.  The "what
                 the chip actually did" number.

  dwt_res      = dwt_cycles - nominal (1,008,000,000)
                 Residual versus expected 1.008 GHz CPU nominal.
                 Captures the beat between the Teensy CPU crystal
                 and the GF-8802's disciplined 10 MHz output.

  fw_stddev    = dwt_welford_stddev
                 Firmware-side cumulative stddev of the dwt_res
                 signal over the campaign.  Monotonic visibility
                 into real per-second DWT jitter.

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
                 visible compensation events in dwt_res.

  mode         = freq_mode_name
                 FINE_LOCK, COARSE_LOCK, HOLDOVER, etc.  A mode change
                 is a major servo regime transition.

Also shown in summary:

  GPIO-PPS witness offset
                 pps_diag_pps_edge_minus_event_ns — the physical PPS
                 GPIO edge position relative to the VCLOCK one-second
                 event, in nanoseconds.  Large DC offset is fixed
                 pipeline delay; stddev is real GPIO ISR latency
                 jitter.  New diagnostic exposed by the refactor.

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

# Threshold for flagging a dwt_res step as a candidate "event" worth
# cross-referencing against GF-8802 diagnostics.  ~100 cycles at 1.008
# GHz is ~100 ns — well above normal second-to-second drift of a few
# tens of cycles.
DWT_RES_STEP_THRESHOLD_CYCLES = 100


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
    print("    dwt_cycles   = dwt_cycle_count_between_pps")
    print("                   (one-second subtraction of consecutive")
    print("                    VCLOCK one-second-event DWT captures)")
    print("    dwt_res      = dwt_cycles - nominal")
    print("    fw_stddev    = dwt_welford_stddev (firmware cumulative)")
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
        f"{'dwt_cycles':>13s}  "
        f"{'dwt_res':>9s}  "
        f"{'fw_stddev':>10s}  "
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
        f"{'─'*10}  "
        f"{'─'*10}  "
        f"{'─'*9}  "
        f"{'─'*10}  "
        f"{'─'*5}  "
        f"{'─'*10}"
    )
    print(header)
    print(sep)

    # Welford accumulators — Teensy side (Pi-side reconstruction).
    w_dwt_cycles      = Welford()
    w_dwt_res         = Welford()

    # Welford accumulators — GF-8802 side.
    w_drift_ppb       = Welford()
    w_freq_err_ppb    = Welford()
    w_pps_err_ns      = Welford()
    w_temp_c          = Welford()

    # Welford accumulator — GPIO PPS witness offset from VCLOCK event.
    w_pps_witness_ns  = Welford()

    # Event detection and bookkeeping.
    step_events: List[Dict[str, Any]] = []
    pskip_events: List[Dict[str, Any]] = []
    mode_transitions: List[Dict[str, Any]] = []

    shown = 0
    gaps = 0
    prev_pps: Optional[int] = None
    prev_dwt_res: Optional[int] = None
    prev_pskip: Optional[int] = None
    prev_mode: Optional[str] = None

    # Track the last firmware-side Welford sample for end-of-report summary.
    last_fw_dwt_welford_n:      Optional[int]   = None
    last_fw_dwt_welford_mean:   Optional[float] = None
    last_fw_dwt_welford_stddev: Optional[float] = None
    last_fw_vclock_welford_n:      Optional[int]   = None
    last_fw_vclock_welford_mean:   Optional[float] = None
    last_fw_vclock_welford_stddev: Optional[float] = None
    last_fw_vclock_welford_min:    Optional[float] = None
    last_fw_vclock_welford_max:    Optional[float] = None

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        gnss = _gnss(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        # Teensy-side fields (new schema).
        dwt_cycles = _as_int(frag.get("dwt_cycle_count_between_pps"))
        fw_stddev  = _as_float(frag.get("dwt_welford_stddev"))

        # GPIO PPS witness offset.
        pps_witness_ns = _as_int(frag.get("pps_diag_pps_edge_minus_event_ns"))

        # Capture firmware-side Welford state (last-wins).
        fw_dwt_n      = _as_int(frag.get("dwt_welford_n"))
        fw_dwt_mean   = _as_float(frag.get("dwt_welford_mean"))
        fw_dwt_stddev = _as_float(frag.get("dwt_welford_stddev"))
        if fw_dwt_n is not None:
            last_fw_dwt_welford_n      = fw_dwt_n
            last_fw_dwt_welford_mean   = fw_dwt_mean
            last_fw_dwt_welford_stddev = fw_dwt_stddev

        fw_vclock_n      = _as_int(frag.get("vclock_welford_n"))
        fw_vclock_mean   = _as_float(frag.get("vclock_welford_mean"))
        fw_vclock_stddev = _as_float(frag.get("vclock_welford_stddev"))
        fw_vclock_min    = _as_float(frag.get("vclock_welford_min"))
        fw_vclock_max    = _as_float(frag.get("vclock_welford_max"))
        if fw_vclock_n is not None:
            last_fw_vclock_welford_n      = fw_vclock_n
            last_fw_vclock_welford_mean   = fw_vclock_mean
            last_fw_vclock_welford_stddev = fw_vclock_stddev
            last_fw_vclock_welford_min    = fw_vclock_min
            last_fw_vclock_welford_max    = fw_vclock_max

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
            prev_dwt_res = None

        prev_pps = pps

        # Compute dwt residual.
        dwt_res: Optional[int] = None
        if dwt_cycles is not None:
            dwt_res = dwt_cycles - DWT_EXPECTED_PER_PPS

        # Event detection: dwt_res step.
        if (dwt_res is not None and prev_dwt_res is not None and
                abs(dwt_res - prev_dwt_res) >= DWT_RES_STEP_THRESHOLD_CYCLES):
            step_events.append({
                "pps": pps,
                "prev_dwt_res": prev_dwt_res,
                "dwt_res": dwt_res,
                "delta": dwt_res - prev_dwt_res,
                "fw_stddev": fw_stddev,
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
                "dwt_res": dwt_res,
            })

        # Event detection: mode change.
        if (mode is not None and prev_mode is not None and
                mode != prev_mode):
            mode_transitions.append({
                "pps": pps,
                "from": prev_mode,
                "to": mode,
                "dwt_res": dwt_res,
            })

        # Update Welford — Teensy side (Pi-side reconstruction).
        if dwt_cycles is not None:
            w_dwt_cycles.update(float(dwt_cycles))
        if dwt_res is not None:
            w_dwt_res.update(float(dwt_res))

        # Update Welford — GPIO witness offset.
        if pps_witness_ns is not None:
            w_pps_witness_ns.update(float(pps_witness_ns))

        # Update Welford — GF-8802 side.
        if drift_ppb is not None:
            w_drift_ppb.update(drift_ppb)
        if freq_err is not None:
            w_freq_err_ppb.update(freq_err)
        if pps_err_ns is not None:
            w_pps_err_ns.update(float(pps_err_ns))
        if temp_c is not None:
            w_temp_c.update(temp_c)

        if dwt_res is not None:
            prev_dwt_res = dwt_res
        if pskip is not None:
            prev_pskip = pskip
        if mode is not None:
            prev_mode = mode

        # Print row.
        print(
            f"{pps:>6d}  "
            f"{_fmt_int(dwt_cycles, 13)}  "
            f"{_fmt_int(dwt_res, 9, signed=True)}  "
            f"{_fmt_float(fw_stddev, 10, decimals=2)}  "
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
    # Teensy-side DWT rate statistics (Pi-side reconstruction)
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Teensy-side DWT rate statistics")
    print("  (one-second subtraction of consecutive VCLOCK event")
    print("   DWT captures; post rolling-integrator removal)")
    print("═" * 70)
    print()
    _print_welford("dwt_cycles (DWT cycles per VCLOCK one-second interval)",
                   w_dwt_cycles, "cycles")
    _print_welford("dwt_res    (dwt_cycles - nominal)",
                   w_dwt_res, "cycles")

    # -----------------------------------------------------------------
    # Firmware-side cumulative Welford (authoritative long-run view)
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Firmware-side DWT residual Welford (cumulative since campaign start)")
    print("  (computed on Teensy, passes through verbatim — the authoritative")
    print("   long-run view; Pi-side stats above track this campaign's window)")
    print("═" * 70)
    print()
    if last_fw_dwt_welford_n is not None and last_fw_dwt_welford_n > 0:
        print(f"  dwt_welford_n      = {last_fw_dwt_welford_n:,}")
        if last_fw_dwt_welford_mean is not None:
            print(f"  dwt_welford_mean   = {last_fw_dwt_welford_mean:+,.3f} cycles")
        if last_fw_dwt_welford_stddev is not None:
            print(f"  dwt_welford_stddev = {last_fw_dwt_welford_stddev:,.3f} cycles")
        # Derive ppb from mean if we can.
        if last_fw_dwt_welford_mean is not None:
            ppb = -last_fw_dwt_welford_mean / DWT_EXPECTED_PER_PPS * 1e9
            print(f"  derived ppb        = {ppb:+,.3f} ppb")
            print(f"    (Teensy CPU crystal offset from GNSS-disciplined nominal;")
            print(f"     sign convention: positive ppb = crystal runs fast)")
        print()
    else:
        print("  (no firmware Welford sample found)")
        print()

    # -----------------------------------------------------------------
    # GPIO PPS witness vs VCLOCK event
    # -----------------------------------------------------------------
    print("═" * 70)
    print("GPIO PPS witness offset from VCLOCK one-second event")
    print("  (pps_diag_pps_edge_minus_event_ns — new diagnostic exposed")
    print("   by the refactor; large DC offset is fixed GF-8802 pipeline")
    print("   delay, stddev is real GPIO ISR latency jitter)")
    print("═" * 70)
    print()
    _print_welford("pps_witness_ns  (Pi-side reconstruction from fragment)",
                   w_pps_witness_ns, "ns", decimals=1)
    if last_fw_vclock_welford_n is not None and last_fw_vclock_welford_n > 0:
        print("  Firmware-side vclock_welford (cumulative):")
        print(f"    n       = {last_fw_vclock_welford_n:,}")
        if last_fw_vclock_welford_mean is not None:
            print(f"    mean    = {last_fw_vclock_welford_mean:+,.3f} ns")
        if last_fw_vclock_welford_stddev is not None:
            print(f"    stddev  = {last_fw_vclock_welford_stddev:,.3f} ns")
        if last_fw_vclock_welford_min is not None:
            print(f"    min     = {last_fw_vclock_welford_min:+,.1f} ns")
        if last_fw_vclock_welford_max is not None:
            print(f"    max     = {last_fw_vclock_welford_max:+,.1f} ns")
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
    # Cross-correlation: dwt_res steps vs GF-8802 activity
    # -----------------------------------------------------------------
    print("═" * 70)
    print("Events — dwt_res steps, phase_skip increments, mode changes")
    print("═" * 70)
    print()

    if step_events:
        print(f"  dwt_res steps ≥ {DWT_RES_STEP_THRESHOLD_CYCLES} cycles:")
        print(f"    (|Δdwt_res| threshold catches servo-step signatures)")
        print()
        for ev in step_events[:50]:
            pskip_s = f"{ev['pskip']}" if ev['pskip'] is not None else "---"
            mode_s = ev['mode'] if ev['mode'] else "---"
            fw_stddev_s = (f"{ev['fw_stddev']:6.2f}"
                           if ev['fw_stddev'] is not None else "   ---")
            print(
                f"    pps={ev['pps']:>6d}  "
                f"Δdwt_res={ev['delta']:+7d}  "
                f"(from {ev['prev_dwt_res']:+6d} to {ev['dwt_res']:+6d})  "
                f"fw_std={fw_stddev_s}  "
                f"drift_ppb={ev['drift_ppb']:+8.3f}  "
                f"pps_err={ev['pps_err_ns'] if ev['pps_err_ns'] is not None else 0:+5d}ns  "
                f"pskip={pskip_s}  "
                f"mode={mode_s}"
            )
        if len(step_events) > 50:
            print(f"    ... and {len(step_events) - 50:,} more")
        print()
    else:
        print(f"  No dwt_res steps ≥ {DWT_RES_STEP_THRESHOLD_CYCLES} cycles detected.")
        print()

    if pskip_events:
        print(f"  phase_skip counter increments:")
        print(f"    (GF-8802 explicitly stepped its PPS output)")
        print()
        for ev in pskip_events[:20]:
            dwt_res_s = (f"{ev['dwt_res']:+6d}"
                         if ev['dwt_res'] is not None else "---")
            print(
                f"    pps={ev['pps']:>6d}  "
                f"pskip: {ev['prev_pskip']} → {ev['pskip']}  "
                f"(Δ={ev['delta']:+d})  dwt_res={dwt_res_s}"
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
            dwt_res_s = (f"{ev['dwt_res']:+6d}"
                         if ev['dwt_res'] is not None else "---")
            print(
                f"    pps={ev['pps']:>6d}  "
                f"{ev['from']} → {ev['to']}  dwt_res={dwt_res_s}"
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

    if w_dwt_cycles.n < 2:
        print("  (insufficient data for comparative interpretation)")
        print()
        return

    print(f"  dwt_res stddev:        {w_dwt_res.stddev:9.3f} cycles  (Pi-side window)")
    print(f"  dwt_res range:         {w_dwt_res.max_val - w_dwt_res.min_val:9.1f} cycles")
    if last_fw_dwt_welford_stddev is not None:
        print(f"  dwt_res stddev (fw):   {last_fw_dwt_welford_stddev:9.3f} cycles  (firmware cumulative)")
    if w_drift_ppb.n >= 2:
        print(f"  drift_ppb stddev:      {w_drift_ppb.stddev:9.3f} ppb")
        # Convert drift_ppb to cycles/sec at 1.008 GHz.
        drift_as_cycles = w_drift_ppb.stddev * (DWT_EXPECTED_PER_PPS / 1e9)
        print(f"  drift_ppb (in cycles): {drift_as_cycles:9.3f} cycles")
    if w_pps_err_ns.n >= 2:
        print(f"  pps_err_ns stddev:     {w_pps_err_ns.stddev:9.1f} ns")
    if w_pps_witness_ns.n >= 2:
        print(f"  pps_witness stddev:    {w_pps_witness_ns.stddev:9.1f} ns  (GPIO ISR jitter)")
    print()

    n_steps = len(step_events)
    n_pskips = len(pskip_events)
    n_modes = len(mode_transitions)

    if n_steps == 0 and n_pskips == 0 and n_modes == 0:
        print("  Calm campaign.  No dwt_res step events, no phase_skips,")
        print("  no mode changes.  The beat between Teensy CPU crystal and")
        print("  GF-8802's disciplined 10 MHz was smooth throughout.")
        print()
        print("  dwt_res's slow drift (if present) is the Teensy CPU crystal")
        print("  aging/thermal response — the GF-8802's 10 MHz output is")
        print("  locked to UTC, so any slow walk in dwt_res is local.")
    elif n_pskips > 0:
        print(f"  {n_pskips} phase_skip event(s) detected.  These are explicit")
        print("  admissions by the GF-8802 that it stepped its PPS output")
        print("  to correct accumulated phase.  Cross-reference with dwt_res")
        print("  steps above to see the event's signature on the Teensy side.")
        print()
        print("  Expected pattern: phase_skip increments coincide with or")
        print("  precede by 1 PPS a corresponding step in dwt_res.")
    elif n_steps > 0:
        print(f"  {n_steps} dwt_res step event(s) without matching phase_skip.")
        print("  The receiver did not admit to stepping, so these are either:")
        print("    • Smooth frequency corrections (receiver held a different")
        print("      rate for several seconds, walking phase gradually)")
        print("    • CPU-side disturbances (thermal, DMA, cache eviction)")
        print("    • GPIO ISR latency excursion — check pps_witness stddev")
        print("      above; if elevated, the substrate was disturbed.")

    if n_modes > 0:
        print()
        print(f"  {n_modes} mode transition(s) — these are major servo regime")
        print("  changes.  HOLDOVER indicates the receiver lost GNSS lock")
        print("  and was free-running on its internal OCXO.")

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