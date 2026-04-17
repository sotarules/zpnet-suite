"""
ZPNet Raw Witness — four independent paths, two questions.

This is a forensic tool, not an everyday analyzer.  Its sole job is to
answer two questions:

  Q1: Are the raw cycle values actually moving between consecutive PPS,
      or is something in the pipeline hiding variance via smoothing?

  Q2: Is the firmware integrator's smoothing providing any measurable
      value at 1-Hz boundary-aligned sampling, or is it unnecessary?

Both questions are answered by reading FOUR independent surfaces from
every TIMEBASE_FRAGMENT and cross-checking them:

  Path A — pps_diag_dwt_isr_entry_raw (u32)
    Set in fill_diag() directly to the DWT cycle counter value
    captured as the FIRST INSTRUCTION of the VCLOCK CH3 ISR.
    Endpoint delta = raw_cycles.  No arithmetic between capture
    and publication.

  Path B — dwt_cycle_count_at_pps (u32)
    Set in alpha's on_event handler from event.dwt_at_event, which
    is r->last_synthetic_dwt = baseline + cumulative_cycles_since_
    baseline.  By the integrator fix, cumulative_cycles_since_baseline
    = dwt_isr_raw - baseline_dwt, so this collapses to the raw ISR
    capture — but via a completely separate bookkeeping path that
    maintains its own running accumulator.

  Path C — pps_diag_integrator_ring_sum (u64)
    Set in fill_diag() to the sum of the 1000 individual inter-tick
    intervals currently in the ring.  By telescoping identity, this
    sum equals dwt_isr_raw[boundary N] - dwt_isr_raw[boundary N-1].
    Computed via a third independent mechanism: incremental ring
    updates on every tick.

  Path D — dwt_cycle_count_between_pps (u32)
    Set in alpha's on_event from interrupt_vclock_cycles_per_second(),
    which returns (ring_sum << 32) / ring_fill * 1000 >> 32.
    Read at on_event time, which is ~1-5 ms after the boundary ISR
    fired, so the ring may have slid forward by a few ticks.
    Expected to differ from A by exactly -1 cycle (Q32.32 truncation
    bias) plus a small dispatch-latency term.

The mathematical expectation, post-fix:

    Path A  ==  Path B  ==  Path C     (bit-identical at every PPS)
    Path D  ==  Path A - 1 + slip

If Paths A, B, C agree bit-for-bit across every PPS, then:
  • The raw values shown by raw_cycles.py are the genuine ISR
    captures, not a smoothed view.  Three separate code paths
    maintaining three separate running accumulators cannot
    conspire to produce identical smoothed output — they'd have
    to smooth the same way with the same bugs, which is
    overwhelmingly unlikely.
  • Any variance we see in raw_cycles is actual per-second
    variance of the CPU's clock-versus-GNSS-disciplined 10 MHz
    substrate, visible through all three independent witnesses.

If Path D's stddev differs meaningfully from Path A's stddev:
  • The integrator's smoothing is measurably changing the signal.
Otherwise:
  • The integrator provides no smoothing benefit at 1-Hz sampling
    (as expected from the telescoping identity).  Its value lies
    elsewhere: mid-second rate estimates, per-tick diagnostics,
    excursion detection.

Usage:
    python -m zpnet.tests.raw_witness <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


DWT_EXPECTED_PER_PPS = 1_008_000_000  # nominal CPU cycles per second


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
# Fragment helpers
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


def _u32_diff(curr: int, prev: int) -> int:
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
        if w.n == 1:
            print(f"  {name}: n=1, value={w.mean:.{decimals}f}{' ' + unit if unit else ''}")
        return
    suffix = f" {unit}" if unit else ""
    print(f"  {name}")
    print(f"    n       = {w.n:,}")
    print(f"    mean    = {w.mean:+,.{decimals}f}{suffix}")
    print(f"    stddev  = {w.stddev:,.{decimals}f}{suffix}")
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
    print("Four independent witnesses to 'raw cycles for this second':")
    print()
    print("  A  = pps_diag_dwt_isr_entry_raw delta  (direct ISR capture)")
    print("  B  = dwt_cycle_count_at_pps delta      (synthetic anchor, alpha path)")
    print("  C  = pps_diag_integrator_ring_sum      (sum of 1000 individual intervals)")
    print("  D  = dwt_cycle_count_between_pps       (SMA through Q32.32, alpha path)")
    print()
    print("Per-row values shown as residuals from nominal (cycles - 1,008,000,000).")
    print("Three-path differences shown as raw cycles.")
    print()

    # Column headers.
    header = (
        f"{'pps':>6s}  "
        f"{'A_res':>9s}  "
        f"{'B_res':>9s}  "
        f"{'C_res':>9s}  "
        f"{'D_res':>9s}  "
        f"{'A-B':>5s}  "
        f"{'A-C':>5s}  "
        f"{'A-D':>6s}  "
        f"{'stddev':>7s}  "
        f"{'note':<10s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*9}  "
        f"{'─'*9}  "
        f"{'─'*9}  "
        f"{'─'*9}  "
        f"{'─'*5}  "
        f"{'─'*5}  "
        f"{'─'*6}  "
        f"{'─'*7}  "
        f"{'─'*10}"
    )
    print(header)
    print(sep)

    # Accumulators.
    w_A_res       = Welford()
    w_B_res       = Welford()
    w_C_res       = Welford()
    w_D_res       = Welford()
    w_A_minus_B   = Welford()
    w_A_minus_C   = Welford()
    w_A_minus_D   = Welford()
    w_tick_stddev = Welford()

    rows_mismatch_AB = 0
    rows_mismatch_AC = 0
    rows_AD_exceeds_bias = 0  # |A - D - 1| > 5 (beyond expected Q32.32 + tiny slip)
    rows_AD_large_slip = 0    # |A - D| > 100

    prev_isr_entry: Optional[int] = None
    prev_dwt_at_pps: Optional[int] = None
    prev_pps: Optional[int] = None

    shown = 0
    gaps = 0

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        # Read the four surfaces.
        isr_entry = _as_int(frag.get("pps_diag_dwt_isr_entry_raw"))
        dwt_at_pps = _as_int(frag.get("dwt_cycle_count_at_pps"))
        ring_sum = _as_int(frag.get("pps_diag_integrator_ring_sum"))
        between_pps = _as_int(frag.get("dwt_cycle_count_between_pps"))
        tick_stddev = _as_float(
            frag.get("pps_diag_integrator_interval_window_stddev_cycles"))

        # Detect gaps — invalidate prev captures so we don't compute a
        # cross-gap delta.
        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps:,} → {pps:,} ---")
            prev_isr_entry = None
            prev_dwt_at_pps = None
        prev_pps = pps

        # Compute the four per-second quantities.
        # Paths C and D are already deltas in the fragment.
        # Paths A and B require subtraction of consecutive captures.
        pathA: Optional[int] = None
        pathB: Optional[int] = None
        pathC: Optional[int] = ring_sum
        pathD: Optional[int] = between_pps

        if isr_entry is not None and prev_isr_entry is not None:
            pathA = _u32_diff(isr_entry, prev_isr_entry)
        if dwt_at_pps is not None and prev_dwt_at_pps is not None:
            pathB = _u32_diff(dwt_at_pps, prev_dwt_at_pps)

        # Residuals vs nominal.
        A_res = pathA - DWT_EXPECTED_PER_PPS if pathA is not None else None
        B_res = pathB - DWT_EXPECTED_PER_PPS if pathB is not None else None
        C_res = pathC - DWT_EXPECTED_PER_PPS if pathC is not None else None
        D_res = pathD - DWT_EXPECTED_PER_PPS if pathD is not None else None

        # Differences (A as reference — A is the purest raw).
        A_minus_B = (pathA - pathB) if (pathA is not None and pathB is not None) else None
        A_minus_C = (pathA - pathC) if (pathA is not None and pathC is not None) else None
        A_minus_D = (pathA - pathD) if (pathA is not None and pathD is not None) else None

        # Flag notable rows.
        notes = []
        if A_minus_B is not None and A_minus_B != 0:
            notes.append("A≠B")
            rows_mismatch_AB += 1
        if A_minus_C is not None and A_minus_C != 0:
            notes.append("A≠C")
            rows_mismatch_AC += 1
        # Expected A - D = +1 (Q32.32 bias).  Flag if it deviates by >5
        # (slight dispatch slip is normal) or >100 (actual excursion slip).
        if A_minus_D is not None:
            deviation = A_minus_D - 1
            if abs(A_minus_D) > 100:
                notes.append("SLIP")
                rows_AD_large_slip += 1
            elif abs(deviation) > 5:
                rows_AD_exceeds_bias += 1
        note_str = ",".join(notes)

        # Update Welfords.
        if A_res is not None:
            w_A_res.update(float(A_res))
        if B_res is not None:
            w_B_res.update(float(B_res))
        if C_res is not None:
            w_C_res.update(float(C_res))
        if D_res is not None:
            w_D_res.update(float(D_res))
        if A_minus_B is not None:
            w_A_minus_B.update(float(A_minus_B))
        if A_minus_C is not None:
            w_A_minus_C.update(float(A_minus_C))
        if A_minus_D is not None:
            w_A_minus_D.update(float(A_minus_D))
        if tick_stddev is not None:
            w_tick_stddev.update(tick_stddev)

        # Print row.
        print(
            f"{pps:>6d}  "
            f"{_fmt_int(A_res, 9, signed=True)}  "
            f"{_fmt_int(B_res, 9, signed=True)}  "
            f"{_fmt_int(C_res, 9, signed=True)}  "
            f"{_fmt_int(D_res, 9, signed=True)}  "
            f"{_fmt_int(A_minus_B, 5, signed=True)}  "
            f"{_fmt_int(A_minus_C, 5, signed=True)}  "
            f"{_fmt_int(A_minus_D, 6, signed=True)}  "
            f"{_fmt_float(tick_stddev, 7, decimals=2)}  "
            f"{note_str:<10s}"
        )

        # Save for next iteration.
        if isr_entry is not None:
            prev_isr_entry = isr_entry
        if dwt_at_pps is not None:
            prev_dwt_at_pps = dwt_at_pps

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()

    # -----------------------------------------------------------------
    # Summary — the four paths
    # -----------------------------------------------------------------
    print("═" * 72)
    print("Per-path residuals (cycles - 1,008,000,000)")
    print("═" * 72)
    print()
    _print_welford("A_res  (direct ISR capture delta)",       w_A_res, "cycles")
    _print_welford("B_res  (synthetic anchor delta)",         w_B_res, "cycles")
    _print_welford("C_res  (ring sum direct)",                w_C_res, "cycles")
    _print_welford("D_res  (SMA through Q32.32, alpha-read)", w_D_res, "cycles")

    # -----------------------------------------------------------------
    # Summary — the differences
    # -----------------------------------------------------------------
    print("═" * 72)
    print("Cross-path differences (A is reference)")
    print("═" * 72)
    print()
    _print_welford("A - B  (expected: exactly 0, always)",       w_A_minus_B, "cycles")
    _print_welford("A - C  (expected: exactly 0, always)",       w_A_minus_C, "cycles")
    _print_welford("A - D  (expected: +1 from Q32.32 bias,\n"
                   "         plus occasional small dispatch slip)", w_A_minus_D, "cycles")

    # -----------------------------------------------------------------
    # Anomaly counts
    # -----------------------------------------------------------------
    print("═" * 72)
    print("Anomaly tallies")
    print("═" * 72)
    print()
    print(f"  Rows where A ≠ B:              {rows_mismatch_AB:,}")
    print(f"  Rows where A ≠ C:              {rows_mismatch_AC:,}")
    print(f"  Rows where |A-D| > 5 (slip):   {rows_AD_exceeds_bias:,}")
    print(f"  Rows where |A-D| > 100 (big):  {rows_AD_large_slip:,}")
    print()

    # -----------------------------------------------------------------
    # Per-tick context
    # -----------------------------------------------------------------
    if w_tick_stddev.n > 0:
        print("═" * 72)
        print("Per-tick context (for reference)")
        print("═" * 72)
        print()
        _print_welford("tick_stddev  (per-PPS RMS of the 1000 intervals)",
                       w_tick_stddev, "cycles", decimals=2)

    # -----------------------------------------------------------------
    # The verdicts
    # -----------------------------------------------------------------
    print("═" * 72)
    print("Verdicts")
    print("═" * 72)
    print()

    if w_A_res.n < 2:
        print("  (insufficient data)")
        print()
        return

    # Q1: Is raw really raw?
    print("  Q1 — ARE THE RAW VALUES ACTUALLY MOVING?")
    print()
    raw_stddev = w_A_res.stddev
    if rows_mismatch_AB == 0 and rows_mismatch_AC == 0:
        print("  YES.  Paths A, B, C agree exactly at every PPS.  Three independent")
        print("  bookkeeping mechanisms (direct ISR capture, synthetic anchor via")
        print("  alpha's cumulative accumulator, per-tick ring sum via the")
        print("  integrator) all produce bit-identical per-second deltas.  For")
        print("  them to conspire to hide variance via smoothing, all three would")
        print("  have to smooth the same way with the same bugs — mathematically")
        print("  unavailable.")
        print()
        print(f"  The {raw_stddev:.1f}-cycle stddev you see in A_res IS the")
        print(f"  genuine per-second variance of the CPU's ~1.008 GHz substrate.")
        print(f"  It is not a smoothed view.  It is the ground truth.")
    else:
        print(f"  ⚠ UNEXPECTED.  A ≠ B in {rows_mismatch_AB:,} rows, A ≠ C in")
        print(f"  {rows_mismatch_AC:,} rows.  This indicates a plumbing bug — one")
        print("  of the firmware paths is not computing what its name implies.")
        print("  Investigate the divergent rows to identify which path is lying.")
    print()

    # Q2: Is smoothing providing any value?
    print("  Q2 — IS THE INTEGRATOR'S SMOOTHING PROVIDING MEASURABLE VALUE?")
    print()
    d_stddev = w_D_res.stddev
    ad_stddev = w_A_minus_D.stddev

    if d_stddev > 0:
        stddev_ratio = d_stddev / raw_stddev if raw_stddev > 0 else float('inf')
        print(f"  stddev(A_res) = {raw_stddev:7.3f} cycles  (raw)")
        print(f"  stddev(D_res) = {d_stddev:7.3f} cycles  (SMA through Q32.32)")
        print(f"  ratio D/A     = {stddev_ratio:7.4f}")
        print()

        # Interpretation.
        if abs(1.0 - stddev_ratio) < 0.02:
            print("  NO.  The SMA output has essentially the same stddev as the raw")
            print("  delta.  At PPS-aligned sampling, the integrator's SMA equals")
            print("  the raw endpoint delta by telescoping identity (sum of 1000")
            print("  per-tick intervals = dwt_raw[N] - dwt_raw[N-1]).  The ring")
            print("  smooths per-tick noise INTERNALLY, but its output at boundary")
            print("  time is mathematically equivalent to simple subtraction.")
            print()
            print("  The integrator's value lies elsewhere:")
            print("    • Mid-second rate estimates (for consumers querying between PPS)")
            print("    • Per-tick diagnostics (tick_stddev, max_ever, etc.)")
            print("    • Excursion detection via ring window statistics")
            print("    • Boundary anchoring (cumulative_cycles_since_baseline)")
            print()
            print("  For 1-Hz consumers who only need endpoint deltas, the integrator")
            print("  is ARCHITECTURALLY useful (it provides the telescoping identity")
            print("  as a single readable surface) but NUMERICALLY unnecessary.")
        elif stddev_ratio < 0.95:
            print("  SOMEWHAT.  The SMA output has lower stddev than the raw delta —")
            print("  the integrator is measurably smoothing.  This is surprising")
            print("  given the telescoping identity; investigate whether dispatch")
            print("  latency is consistently biasing the SMA read.")
        elif stddev_ratio > 1.05:
            print("  NEGATIVELY.  The SMA output has HIGHER stddev than the raw")
            print("  delta.  Dispatch latency is introducing variance on top of")
            print("  the ground truth.  Consider reading raw captures directly.")
    print()

    # Dispatch slip narrative.
    if w_A_minus_D.n >= 2:
        ad_mean = w_A_minus_D.mean
        ad_max = max(abs(w_A_minus_D.min_val), abs(w_A_minus_D.max_val))
        print(f"  Dispatch-slip signature (A - D):")
        print(f"    mean = {ad_mean:+.3f} cycles  (expected: +1 from Q32.32)")
        print(f"    |max| = {ad_max:.0f} cycles")
        if ad_max > 100:
            print(f"    → At least one large slip event occurred.  This means")
            print(f"    alpha's on_event read the integrator ring AFTER the ring")
            print(f"    had slid past an excursion tick.  Would not occur if alpha")
            print(f"    read the raw captures directly.")
    print()


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: raw_witness <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
