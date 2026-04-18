"""
ZPNet VCLOCK Health — Bridge Self-Test Against DWT and GNSS PPS.

VCLOCK is the GNSS-disciplined 10 MHz reference, now elevated to a
measured peer of OCXO1 / OCXO2.  Per-edge measurement runs through
alpha's vclock_apply_edge() (structurally identical to ocxo_apply_edge),
producing real values in g_vclock_clock and g_vclock_measurement.

Self-test property:

  Because the VCLOCK lane drives the DWT↔GNSS bridge anchor itself,
  the bridge-derived per-event delta should always equal exactly 1e9 ns
  modulo two small noise sources:

    1. dwt_effective_cycles_per_second integer-division round-off
       (one DWT cycle ≈ 0.99 ns at 1.008 GHz, so a few cycles of
       round-off translates to a few ns of residual)

    2. NVIC entry-latency jitter on the QTimer1 CH3 ISR (deterministic
       because the prespin shadow loop guarantees a known CPU state)

  Any non-zero vclock_second_residual_ns surfaces bridge interpolation
  inconsistency rather than crystal drift.  It should sit very close to
  zero with very small stddev.  Sustained mean drift would indicate a
  systematic bias in the bridge math; growing stddev would indicate
  ISR latency variance.

This tool shows three relationships per row:

  VCLOCK vs DWT:

    v_dwt_cyc          — vclock_dwt_cycles_between_edges (new path,
                         via g_vclock_measurement.prev_dwt_at_edge)

    dwt_cyc            — dwt_cycle_count_between_pps (existing path,
                         via g_prev_dwt_at_vclock_event)

    Δcyc = v_dwt_cyc - dwt_cyc

    Both fields track the same physical quantity through two
    independent prev-trackers.  Δcyc should be exactly 0 every row.
    Any non-zero value is a bug in one of the trackers.

  VCLOCK self-test:

    v_resid            — vclock_second_residual_ns
                         = 1e9 - vclock_gnss_ns_between_edges

    The bridge-interpolation noise floor.  Should hover within ±10 ns
    of zero.

  VCLOCK vs GNSS PPS baseline:

    gnss_drift_ppb     — extra_clocks.gnss_raw_drift_ppb

    The GF-8802's own self-reported drift against UTC/USNO.  This is
    receiver-side telemetry, completely independent of our DWT bridge.
    It tells us how disciplined the 10 MHz signal is at its source.

Welford summaries:

  vclock_residual    — mean & stddev of v_resid over the campaign.
  dwt_cycles_per_sec — mean & stddev of dwt_cyc.  Reveals the actual
                       Teensy CPU clock rate vs nominal 1,008,000,000.
  gnss_raw_drift_ppb — mean & stddev of GF-8802 self-reported drift.

Cross-checks flagged:

  Δcyc != 0          — dual prev-tracker mismatch
  vclock_window_mismatches > 0  — bridge interpolation exceeded
                                  CLOCK_WINDOW_TOLERANCE_NS (500 ns)

Usage:
    python -m zpnet.tests.vclock_health <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
DWT_NOMINAL_CYCLES_PER_SECOND = 1_008_000_000


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


def _extra(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    extra = root.get("extra_clocks")
    return extra if isinstance(extra, dict) else {}


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


def _fmt_i(v: Optional[int]) -> str:
    if v is None:
        return "---"
    return f"{v:,d}"


def _fmt_f(v: Optional[float], decimals: int = 3) -> str:
    if v is None:
        return "---"
    return f"{v:+,.{decimals}f}"


# ---------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows):,} rows)")
    print()
    print("VCLOCK is the GNSS-disciplined 10 MHz reference, measured as")
    print("a peer of OCXO1/OCXO2.  Because VCLOCK drives the DWT↔GNSS")
    print("bridge anchor itself, this analyzer is a self-test of bridge")
    print("interpolation consistency.")
    print()
    print("Per-row fields:")
    print()
    print("  gnss_ns         — VCLOCK-authored count at publish time")
    print("                    (= g_gnss_ns_count_at_pps).")
    print()
    print("  v_dwt_cyc       — vclock_dwt_cycles_between_edges, the new")
    print("                    measurement path (via g_vclock_measurement).")
    print()
    print("  dwt_cyc         — dwt_cycle_count_between_pps, the existing")
    print("                    path (via g_prev_dwt_at_vclock_event).")
    print()
    print("  Δcyc            — v_dwt_cyc − dwt_cyc.  Should be exactly 0")
    print("                    every row (dual prev-tracker sanity check).")
    print()
    print("  v_resid         — vclock_second_residual_ns")
    print("                    = 1e9 - vclock_gnss_ns_between_edges.")
    print("                    Bridge interpolation noise floor.  Should")
    print("                    hover within ±10 ns of zero.")
    print()
    print("  v_phase_off     — vclock_phase_offset_ns.  Warmup artifact")
    print("                    near 1e9 if the install missed first-cycle;")
    print("                    near 0 once measurement-edge alignment is")
    print("                    established.")
    print()
    print("  gnss_drift_ppb  — extra_clocks.gnss_raw_drift_ppb, the")
    print("                    GF-8802's own self-reported drift against")
    print("                    UTC/USNO.  Independent receiver telemetry.")
    print()

    header = (
        f"{'pps':>6s}  "
        f"{'gnss_ns':>16s}  "
        f"{'v_dwt_cyc':>13s}  "
        f"{'dwt_cyc':>13s}  "
        f"{'Δcyc':>6s}  "
        f"{'v_resid':>9s}  "
        f"{'v_phase_off':>14s}  "
        f"{'gnss_drift_ppb':>14s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*16}  "
        f"{'─'*13}  "
        f"{'─'*13}  "
        f"{'─'*6}  "
        f"{'─'*9}  "
        f"{'─'*14}  "
        f"{'─'*14}"
    )
    print(header)
    print(sep)

    w_v_resid = Welford()
    w_dwt_cyc = Welford()
    w_gnss_drift = Welford()

    shown = 0
    cyc_mismatches = 0
    cyc_compared = 0
    window_mismatches_max = 0
    window_checks_max = 0
    not_yet_zero_established = 0

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        extra = _extra(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        gnss_ns       = _as_int(frag.get("gnss_ns"))
        v_dwt_cyc     = _as_int(frag.get("vclock_dwt_cycles_between_edges"))
        dwt_cyc       = _as_int(frag.get("dwt_cycle_count_between_pps"))
        v_resid       = _as_int(frag.get("vclock_second_residual_ns"))
        v_phase_off   = _as_int(frag.get("vclock_phase_offset_ns"))
        v_zero_est    = bool(frag.get("vclock_zero_established"))
        v_win_chk     = _as_int(frag.get("vclock_window_checks")) or 0
        v_win_mm      = _as_int(frag.get("vclock_window_mismatches")) or 0
        gnss_drift    = _as_float(extra.get("gnss_raw_drift_ppb"))

        if not v_zero_est:
            not_yet_zero_established += 1

        # Δcyc — dual prev-tracker sanity check.  Both should be 0 in
        # warmup rows (before vclock_apply_edge has run twice), and
        # bit-exactly equal thereafter.
        delta_cyc: Optional[int] = None
        if v_dwt_cyc is not None and dwt_cyc is not None:
            # Ignore the warmup rows where v_dwt_cyc is still 0 because
            # vclock_apply_edge hasn't seen its second edge yet.  Only
            # cross-check once both rails are publishing real values.
            if v_dwt_cyc != 0:
                delta_cyc = v_dwt_cyc - dwt_cyc
                cyc_compared += 1
                if delta_cyc != 0:
                    cyc_mismatches += 1
                w_dwt_cyc.update(float(dwt_cyc))

        # VCLOCK residual Welford — only after zero established AND a
        # real measurement is present (vclock_gnss_ns_between_edges != 0).
        if v_zero_est and v_resid is not None and v_dwt_cyc is not None and v_dwt_cyc != 0:
            w_v_resid.update(float(v_resid))

        # GF-8802 drift Welford — independent of any VCLOCK warmup.
        if gnss_drift is not None:
            w_gnss_drift.update(gnss_drift)

        # Track window-check accounting.
        if v_win_chk > window_checks_max:
            window_checks_max = v_win_chk
        if v_win_mm > window_mismatches_max:
            window_mismatches_max = v_win_mm

        print(
            f"{pps:>6d}  "
            f"{_fmt_i(gnss_ns):>16s}  "
            f"{_fmt_i(v_dwt_cyc):>13s}  "
            f"{_fmt_i(dwt_cyc):>13s}  "
            f"{_fmt_i(delta_cyc):>6s}  "
            f"{_fmt_i(v_resid):>9s}  "
            f"{_fmt_i(v_phase_off):>14s}  "
            f"{_fmt_f(gnss_drift, 3):>14s}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown:                          {shown:,}")
    print(f"Rows before vclock_zero_established: {not_yet_zero_established:,}")
    print()

    print("═" * 78)
    print("VCLOCK residual summary — bridge interpolation self-test")
    print("═" * 78)
    print()
    print("  vclock_second_residual_ns = 1e9 - vclock_gnss_ns_between_edges.")
    print("  Mean should sit very close to zero; non-zero mean indicates a")
    print("  systematic bias in dwt_effective_cycles_per_second or in the")
    print("  bridge interpolation.  Stddev is the noise floor of our DWT")
    print("  bookkeeping.")
    print()
    if w_v_resid.n >= 1:
        print(f"  n       = {w_v_resid.n:,}")
        print(f"  mean    = {w_v_resid.mean:+,.3f} ns")
        print(f"  stddev  = {w_v_resid.stddev:,.3f} ns")
        print(f"  stderr  = {w_v_resid.stderr:,.3f} ns")
        print(f"  min     = {w_v_resid.min_val:+,.0f} ns")
        print(f"  max     = {w_v_resid.max_val:+,.0f} ns")
        print(f"  range   = {w_v_resid.max_val - w_v_resid.min_val:,.0f} ns")
    else:
        print("  (no rows with measurement established)")
    print()

    print("═" * 78)
    print("DWT cycles-per-second summary — actual measured Teensy rate")
    print("═" * 78)
    print()
    print(f"  Nominal: {DWT_NOMINAL_CYCLES_PER_SECOND:,} cycles/s")
    print()
    print("  Mean reveals the Teensy CPU's intrinsic crystal offset relative")
    print("  to the GNSS 10 MHz reference.  Stddev reveals VCLOCK ISR latency")
    print("  jitter (since the count interval IS one VCLOCK second).")
    print()
    if w_dwt_cyc.n >= 1:
        offset = w_dwt_cyc.mean - DWT_NOMINAL_CYCLES_PER_SECOND
        offset_ppb = offset / DWT_NOMINAL_CYCLES_PER_SECOND * 1e9
        print(f"  n       = {w_dwt_cyc.n:,}")
        print(f"  mean    = {w_dwt_cyc.mean:,.3f} cycles/s")
        print(f"  offset  = {offset:+,.3f} cycles/s")
        print(f"          = {offset_ppb:+,.3f} ppb")
        print(f"  stddev  = {w_dwt_cyc.stddev:,.3f} cycles")
        print(f"  stderr  = {w_dwt_cyc.stderr:,.3f} cycles")
        print(f"  min     = {w_dwt_cyc.min_val:,.0f} cycles")
        print(f"  max     = {w_dwt_cyc.max_val:,.0f} cycles")
        print(f"  range   = {w_dwt_cyc.max_val - w_dwt_cyc.min_val:,.0f} cycles")
    else:
        print("  (no rows with measurement established)")
    print()

    print("═" * 78)
    print("GF-8802 self-reported drift summary — independent receiver telemetry")
    print("═" * 78)
    print()
    print("  extra_clocks.gnss_raw_drift_ppb is what the GNSS receiver itself")
    print("  reports as its current drift against UTC/USNO.  Completely")
    print("  independent of any Teensy-side measurement.  Provides a sanity")
    print("  check on whether the 10 MHz signal feeding VCLOCK is actually")
    print("  GNSS-disciplined.")
    print()
    if w_gnss_drift.n >= 1:
        print(f"  n       = {w_gnss_drift.n:,}")
        print(f"  mean    = {w_gnss_drift.mean:+,.4f} ppb")
        print(f"  stddev  = {w_gnss_drift.stddev:,.4f} ppb")
        print(f"  stderr  = {w_gnss_drift.stderr:,.4f} ppb")
        print(f"  min     = {w_gnss_drift.min_val:+,.4f} ppb")
        print(f"  max     = {w_gnss_drift.max_val:+,.4f} ppb")
        print(f"  range   = {w_gnss_drift.max_val - w_gnss_drift.min_val:,.4f} ppb")
    else:
        print("  (no extra_clocks.gnss_raw_drift_ppb rows)")
    print()

    print("═" * 78)
    print("Dual prev-tracker sanity check — Δcyc must be 0")
    print("═" * 78)
    print()
    print("  vclock_dwt_cycles_between_edges and dwt_cycle_count_between_pps")
    print("  track the same physical quantity through two independent")
    print("  prev-trackers.  They must agree bit-exactly on every row.")
    print()
    print(f"  Rows compared (after warmup):  {cyc_compared:,}")
    print(f"  Rows where Δcyc ≠ 0:           {cyc_mismatches:,}")
    if cyc_compared > 0 and cyc_mismatches == 0:
        print("  ✓ Bit-exact agreement on all rows.")
    elif cyc_mismatches > 0:
        frac = 100.0 * cyc_mismatches / max(1, cyc_compared)
        print(f"  ⚠ {cyc_mismatches:,} divergent rows ({frac:.2f}%) — investigate.")
    print()

    print("═" * 78)
    print("VCLOCK window-check accounting")
    print("═" * 78)
    print()
    print("  vclock_apply_edge() flags any bridge-derived between-edges")
    print("  interval whose magnitude exceeds CLOCK_WINDOW_TOLERANCE_NS")
    print("  (500 ns).  For the canonical reference clock this should")
    print("  never trip.")
    print()
    print(f"  vclock_window_checks (max):       {window_checks_max:,}")
    print(f"  vclock_window_mismatches (max):   {window_mismatches_max:,}")
    if window_mismatches_max == 0:
        print("  ✓ All windows within ±500 ns tolerance.")
    else:
        print(f"  ⚠ {window_mismatches_max:,} window mismatch(es) — bridge exceeded tolerance.")
    print()

    print("═" * 78)
    print("Interpretation")
    print("═" * 78)
    print()
    print("  v_resid (vclock_second_residual_ns):")
    print("    Bridge interpolation noise.  At 1.008 GHz, one DWT cycle is")
    print("    ~0.99 ns, so ±4 ns residual is ±4 cycles of integer-division")
    print("    round-off in dwt_effective_cycles_per_second.  Welford mean")
    print("    converging to zero with shrinking stderr proves the bridge")
    print("    is unbiased to the precision of the measurement.")
    print()
    print("  dwt_cyc offset:")
    print("    The Teensy CPU's intrinsic crystal offset.  Negative offset")
    print("    means the CPU is slow vs nominal 1,008,000,000.  Typical")
    print("    Teensy 4.1 boards run a few ppm slow.  This value is honest")
    print("    measurement; it does not change the GNSS-ns timeline because")
    print("    the bridge uses the measured rate, not the nominal.")
    print()
    print("  gnss_drift_ppb:")
    print("    The GF-8802's own discipline-loop output.  Stable values")
    print("    mean the receiver is locked and tracking; large excursions")
    print("    or rapid variance indicate signal-quality problems at the")
    print("    antenna.")
    print()
    print("  v_phase_off (vclock_phase_offset_ns):")
    print("    A value near 1,000,000,000 in early rows is a warmup artifact:")
    print("    the first vclock_apply_edge call sets ns_count_at_edge=1e9")
    print("    while the bridge already reports ~2e9 (because the install")
    print("    event was skipped due to bridge-not-yet-valid).  After the")
    print("    second edge, this value stabilizes near zero.")
    print()


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: vclock_health <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()