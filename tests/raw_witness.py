"""
ZPNet Raw Witness — PPS Witness Relative to VCLOCK.

Under ZPNet's current doctrine, VCLOCK owns the canonical GNSS clock
and emits a synthetic 1-second boundary by pure arithmetic.  The
physical PPS GPIO edge is demoted to two narrow roles: a diagnostic
witness, and the trigger for TIMEBASE_FRAGMENT publication.

This tool answers the closure question:

    Where did the physical PPS edge land relative to the current
    VCLOCK-authored GNSS second boundary?

Two independent rails per row:

  BRIDGE-INDEPENDENT (pure ISR-captured facts, no bridge):

    ns_from_v = pps_diag_pps_edge_ns_from_vclock

    The DWT distance from the most recent VCLOCK one-second event to
    the physical PPS edge, converted via the firmware's own
    dwt_effective_cycles_per_second at pps_edge_callback time.  This
    is ground truth for the V-to-edge phase; robust to any bridge
    anchor mis-authorship.

  BRIDGE-DEPENDENT (goes through the DWT↔GNSS bridge):

    pps_edge_delta_ns = pps_diag_pps_edge_gnss_ns - gnss_ns

    Same moment, same edge, but the bridge maps through anchor state
    (pps_count, dwt_at_pps, dwt_cycles_per_s) and compares against
    g_gnss_ns_count_at_pps.  Sensitive to off-by-one in anchor
    authorship.

Decomposition expectation (essay §6):

  diff_vs_neg_1e9 = pps_edge_delta_ns - ns_from_v - (-1,000,000,000)

  If diff_vs_neg_1e9 ≈ 0, the bridge-dependent rail is exactly one
  second below the bridge-independent rail.  That -1e9 gap is the
  signature of the epoch-install off-by-one between
  g_gnss_ns_count_at_pps (set to 1e9 at install) and the bridge
  formula's (pps_count-1)*1e9 (which reads 0 at pps_count=1).

VCLOCK lockstep check:

  pps_edge_vclock_event_count should increment by exactly 1 per
  fragment.  If it increments by 2 or stays the same, a VCLOCK event
  interleaved with the GPIO ISR / foreground dispatch — a race that
  would invalidate the cross-row comparison.

Usage:
    python -m zpnet.tests.raw_witness <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


NEG_ONE_SECOND_NS = -1_000_000_000


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


def _fmt_i(v: Optional[int]) -> str:
    if v is None:
        return "---"
    return f"{v:,d}"


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
    print("Two GNSS-ns quantities shown verbatim per row:")
    print()
    print("  gnss_ns            — VCLOCK-authored count at publish time")
    print("                       (= g_gnss_ns_count_at_pps)")
    print()
    print("  pps_edge_gnss_ns   — bridge-translated GNSS ns of the GPIO")
    print("                       edge (= pps_diag_pps_edge_gnss_ns)")
    print()
    print("Plus two independent phase rails:")
    print()
    print("  ns_from_v          — bridge-INDEPENDENT phase (firmware DWT subtraction)")
    print("                       pps_diag_pps_edge_ns_from_vclock")
    print()
    print("  pps_edge_delta_ns  — bridge-DEPENDENT phase")
    print("                       pps_edge_gnss_ns - gnss_ns")
    print()
    print("Decomposition check (summary only):")
    print()
    print("  diff_vs_neg_1e9 = pps_edge_delta_ns - ns_from_v - (-1,000,000,000)")
    print()
    print("  If diff_vs_neg_1e9 ≈ 0, the bridge-dependent rail is exactly")
    print("  one second below the bridge-independent rail.  That -1e9 gap")
    print("  is the signature of the epoch-install off-by-one between")
    print("  g_gnss_ns_count_at_pps and the bridge formula's (pps_count-1)*1e9.")
    print()
    print("Lockstep check:")
    print()
    print("  Δvclk = pps_edge_vclock_event_count[N] - pps_edge_vclock_event_count[N-1]")
    print("          Expected: exactly 1 per fragment.")
    print("          Anything else = VCLOCK-vs-GPIO race.")
    print()

    header = (
        f"{'pps':>6s}  "
        f"{'seq':>6s}  "
        f"{'vclk_n':>7s}  "
        f"{'Δvclk':>5s}  "
        f"{'gnss_ns':>16s}  "
        f"{'pps_edge_gnss_ns':>18s}  "
        f"{'ns_from_v':>14s}  "
        f"{'pps_edge_delta_ns':>18s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*6}  "
        f"{'─'*7}  "
        f"{'─'*5}  "
        f"{'─'*16}  "
        f"{'─'*18}  "
        f"{'─'*14}  "
        f"{'─'*18}"
    )
    print(header)
    print(sep)

    w_ns_from_v   = Welford()
    w_edge_delta  = Welford()
    w_diff_residual = Welford()

    shown = 0
    missing_edge = 0
    missing_ns_from_v = 0
    missing_vclk_count = 0
    edge_fw_mismatches = 0
    race_count_delta_not_one = 0

    prev_vclk_n: Optional[int] = None

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        seq       = _as_int(frag.get("pps_diag_pps_edge_sequence"))
        gnss_ns   = _as_int(frag.get("gnss_ns"))
        pps_edge_gnss_ns = _as_int(frag.get("pps_diag_pps_edge_gnss_ns"))
        fw_edge_delta    = _as_int(frag.get("pps_diag_pps_edge_minus_event_ns"))
        ns_from_v = _as_int(frag.get("pps_diag_pps_edge_ns_from_vclock"))
        vclk_n    = _as_int(frag.get("pps_diag_pps_edge_vclock_event_count"))

        # Bridge-dependent rail (Pi-side recomputation; cross-checked vs firmware)
        edge_delta_ns: Optional[int] = None
        if gnss_ns is not None and pps_edge_gnss_ns is not None:
            edge_delta_ns = pps_edge_gnss_ns - gnss_ns
            w_edge_delta.update(float(edge_delta_ns))
            if fw_edge_delta is not None and fw_edge_delta != edge_delta_ns:
                edge_fw_mismatches += 1
        else:
            missing_edge += 1

        # Bridge-independent rail (firmware-authored, used as-is)
        if ns_from_v is not None:
            w_ns_from_v.update(float(ns_from_v))
        else:
            missing_ns_from_v += 1

        # Off-by-one residual: the bridge-dep rail minus the bridge-indep rail
        # should equal -1e9 if the epoch-install off-by-one is the only bug.
        diff_residual: Optional[int] = None
        if ns_from_v is not None and edge_delta_ns is not None:
            diff_residual = edge_delta_ns - ns_from_v - NEG_ONE_SECOND_NS
            w_diff_residual.update(float(diff_residual))

        # VCLOCK lockstep check
        delta_vclk: Optional[int] = None
        if vclk_n is not None:
            if prev_vclk_n is not None:
                delta_vclk = vclk_n - prev_vclk_n
                if delta_vclk != 1:
                    race_count_delta_not_one += 1
            prev_vclk_n = vclk_n
        else:
            missing_vclk_count += 1

        print(
            f"{pps:>6d}  "
            f"{_fmt_i(seq):>6s}  "
            f"{_fmt_i(vclk_n):>7s}  "
            f"{_fmt_i(delta_vclk):>5s}  "
            f"{_fmt_i(gnss_ns):>16s}  "
            f"{_fmt_i(pps_edge_gnss_ns):>18s}  "
            f"{_fmt_i(ns_from_v):>14s}  "
            f"{_fmt_i(edge_delta_ns):>18s}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown:                      {shown:,}")
    print(f"Missing bridge-dep edge rows:    {missing_edge:,}")
    print(f"Missing ns_from_v rows:          {missing_ns_from_v:,}")
    print(f"Missing vclk_event_count rows:   {missing_vclk_count:,}")
    print()

    print("═" * 78)
    print("ns_from_v summary — bridge-INDEPENDENT VCLOCK-to-edge phase")
    print("═" * 78)
    print()
    if w_ns_from_v.n >= 1:
        print(f"  n       = {w_ns_from_v.n:,}")
        print(f"  mean    = {w_ns_from_v.mean:+,.3f} ns   ({w_ns_from_v.mean/1_000_000.0:+,.3f} ms)")
        print(f"  stddev  = {w_ns_from_v.stddev:,.3f} ns   ({w_ns_from_v.stddev/1000.0:,.3f} µs)")
        print(f"  stderr  = {w_ns_from_v.stderr:,.3f} ns")
        print(f"  min     = {w_ns_from_v.min_val:+,.0f} ns   ({w_ns_from_v.min_val/1_000_000.0:+,.3f} ms)")
        print(f"  max     = {w_ns_from_v.max_val:+,.0f} ns   ({w_ns_from_v.max_val/1_000_000.0:+,.3f} ms)")
        print(f"  range   = {w_ns_from_v.max_val - w_ns_from_v.min_val:,.0f} ns")
    else:
        print("  (no ns_from_v rows)")
    print()

    print("═" * 78)
    print("pps_edge_delta_ns summary — bridge-DEPENDENT rail")
    print("═" * 78)
    print()
    if w_edge_delta.n >= 1:
        print(f"  n       = {w_edge_delta.n:,}")
        print(f"  mean    = {w_edge_delta.mean:+,.3f} ns   ({w_edge_delta.mean/1_000_000.0:+,.3f} ms)")
        print(f"  stddev  = {w_edge_delta.stddev:,.3f} ns   ({w_edge_delta.stddev/1000.0:,.3f} µs)")
        print(f"  stderr  = {w_edge_delta.stderr:,.3f} ns")
        print(f"  min     = {w_edge_delta.min_val:+,.0f} ns   ({w_edge_delta.min_val/1_000_000.0:+,.3f} ms)")
        print(f"  max     = {w_edge_delta.max_val:+,.0f} ns   ({w_edge_delta.max_val/1_000_000.0:+,.3f} ms)")
        print(f"  range   = {w_edge_delta.max_val - w_edge_delta.min_val:,.0f} ns")
    else:
        print("  (no bridge-dependent edge rows)")
    print()

    print("═" * 78)
    print("diff_vs_neg_1e9 residual — decomposition test")
    print("═" * 78)
    print()
    print("  Expectation: pps_edge_delta_ns - ns_from_v ≈ -1,000,000,000 ns,")
    print("  so diff_vs_neg_1e9 ≈ 0.  Near zero confirms the bridge-dep rail")
    print("  is exactly one second below the bridge-indep rail — the")
    print("  epoch-install off-by-one signature.")
    print()
    if w_diff_residual.n >= 1:
        print(f"  n       = {w_diff_residual.n:,}")
        print(f"  mean    = {w_diff_residual.mean:+,.3f} ns   ({w_diff_residual.mean/1000.0:+,.3f} µs)")
        print(f"  stddev  = {w_diff_residual.stddev:,.3f} ns   ({w_diff_residual.stddev/1000.0:,.3f} µs)")
        print(f"  stderr  = {w_diff_residual.stderr:,.3f} ns")
        print(f"  min     = {w_diff_residual.min_val:+,.0f} ns")
        print(f"  max     = {w_diff_residual.max_val:+,.0f} ns")
        print(f"  range   = {w_diff_residual.max_val - w_diff_residual.min_val:,.0f} ns")
    else:
        print("  (no rows with both rails present)")
    print()

    print("═" * 78)
    print("VCLOCK lockstep — pps_edge_vclock_event_count row-to-row delta")
    print("═" * 78)
    print()
    print("  Expected: Δvclk = 1 on every row (after the first).")
    print("  Anomalies indicate a VCLOCK one-second event fired between")
    print("  the GPIO ISR snapshot and the foreground pps_edge_callback.")
    print()
    lockstep_rows = max(0, shown - 1 - missing_vclk_count)
    print(f"  Rows with Δvclk computed:  {lockstep_rows:,}")
    print(f"  Rows where Δvclk ≠ 1:      {race_count_delta_not_one:,}")
    if race_count_delta_not_one == 0 and lockstep_rows > 0:
        print("  ✓ Clean lockstep — no VCLOCK-vs-GPIO race detected.")
    elif race_count_delta_not_one > 0:
        frac = 100.0 * race_count_delta_not_one / max(1, lockstep_rows)
        print(f"  ⚠ {race_count_delta_not_one:,} race events ({frac:.2f}%).")
    print()

    print("═" * 78)
    print("Firmware cross-check — bridge-dependent rail")
    print("═" * 78)
    print()
    print("pps_diag_pps_edge_minus_event_ns (firmware) vs our Pi-side")
    print("computation of pps_edge_gnss_ns - gnss_ns should be bit-exact.")
    print()
    present_fw = sum(
        1 for rec in rows[:shown]
        if _as_int(_frag(rec).get("pps_diag_pps_edge_minus_event_ns")) is not None
    )
    print(f"  Rows with firmware edge delta present: {present_fw:,}")
    print(f"  Rows where firmware ≠ computed:        {edge_fw_mismatches:,}")
    if present_fw > 0 and edge_fw_mismatches == 0:
        print("  ✓ Bit-exact agreement.")
    elif edge_fw_mismatches > 0:
        print(f"  ⚠ {edge_fw_mismatches:,} divergent rows — investigate.")
    print()

    print("═" * 78)
    print("Interpretation")
    print("═" * 78)
    print()
    print("  ns_from_v:")
    print("    The bridge-independent rail.  Measures the DWT distance from")
    print("    the most recent VCLOCK one-second event to the physical PPS")
    print("    edge that followed it.  This is what the phase actually is,")
    print("    regardless of any off-by-one in bridge authorship.")
    print()
    print("    Interpretation:")
    print("      ~+300 ns      → VCLOCK fires almost exactly at the physical")
    print("                      PPS, just a hair before.  The +300 ns is")
    print("                      GPIO ISR hardware latency.  This is the")
    print("                      target state after phase-alignment fix.")
    print("      ~+807 ms      → VCLOCK fires 807 ms AFTER each physical")
    print("                      PPS (equivalently, 193 ms before the next).")
    print("                      Boot-phase offset case — VCLOCK started at")
    print("                      an arbitrary moment during boot.")
    print("      any other     → Something interesting.  Dig in.")
    print()
    print("  pps_edge_delta_ns:")
    print("    The bridge-dependent rail.  Same moment, same edge, but the")
    print("    bridge maps it through anchor state and compares against")
    print("    g_gnss_ns_count_at_pps.  If the two are off by a full second")
    print("    (epoch-install off-by-one), this rail sits -1e9 below")
    print("    ns_from_v.")
    print()
    print("  diff_vs_neg_1e9:")
    print("    pps_edge_delta_ns - ns_from_v, offset by +1e9.  Mean near")
    print("    zero confirms the off-by-one story.  Drift means the story")
    print("    is more complicated.")
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