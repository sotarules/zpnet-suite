"""
ZPNet Raw Witness — PPS Witness Relative to VCLOCK.

Under ZPNet's current doctrine, VCLOCK owns the canonical GNSS clock
and emits a synthetic 1-second boundary by pure arithmetic.  The
physical PPS GPIO edge is demoted to two narrow roles: a diagnostic
witness, and the trigger for TIMEBASE_FRAGMENT publication.

This tool answers the closure question:

    Where did the physical PPS edge land relative to the current
    VCLOCK-authored GNSS second boundary?

For every TIMEBASE / TIMEBASE_FRAGMENT, it reports both:

    pps_edge_delta_ns   = pps_diag_pps_edge_gnss_ns - gnss_ns
    boundary_delta_ns   = pps_diag_boundary_dwt_isr_entry_gnss_ns - pps_diag_gnss_ns_at_event

Where:

  • gnss_ns                               — VCLOCK's canonical synthetic
                                            1-second boundary in GNSS ns
  • pps_diag_pps_edge_sequence            — PPS witness sequence captured
                                            in the GPIO ISR snapshot path
  • pps_diag_pps_edge_gnss_ns             — GNSS ns of the physical PPS edge
                                            as translated through the bridge
  • pps_diag_pps_edge_minus_event_ns      — firmware's own PPS-edge delta
                                            against current fragment gnss_ns
  • pps_diag_boundary_dwt_isr_entry_*     — boundary-emitter ISR facts
                                            (VCLOCK-side, not PPS-side)

A positive pps_edge_delta means the PPS edge landed AFTER the VCLOCK boundary.
A negative value means the PPS edge landed BEFORE it.

Usage:
    python -m zpnet.tests.raw_witness <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


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


def _fmt_f(v: Optional[float], decimals: int = 3) -> str:
    if v is None:
        return "---"
    return f"{v:,.{decimals}f}"


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
    print("For each PPS boundary:")
    print()
    print("  pps_edge_delta_ns = pps_diag_pps_edge_gnss_ns - gnss_ns")
    print()
    print("Where gnss_ns is the current VCLOCK-authored synthetic boundary and")
    print("pps_diag_pps_edge_gnss_ns is the physical PPS witness translated into")
    print("the GNSS nanosecond timeline.")
    print()
    print("We also show the VCLOCK boundary-emitter ISR rail for contrast:")
    print()
    print("  boundary_delta_ns =")
    print("      pps_diag_boundary_dwt_isr_entry_gnss_ns - pps_diag_gnss_ns_at_event")
    print()

    header = (
        f"{'pps':>6s}  "
        f"{'seq':>6s}  "
        f"{'gnss_ns':>16s}  "
        f"{'pps_edge_gnss_ns':>18s}  "
        f"{'pps_edge_delta_ns':>18s}  "
        f"{'fw_edge_delta':>14s}  "
        f"{'boundary_delta_ns':>18s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*6}  "
        f"{'─'*16}  "
        f"{'─'*18}  "
        f"{'─'*18}  "
        f"{'─'*14}  "
        f"{'─'*18}"
    )
    print(header)
    print(sep)

    w_edge_delta = Welford()
    w_boundary_delta = Welford()

    shown = 0
    missing_edge = 0
    missing_boundary = 0
    edge_fw_mismatches = 0

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        seq = _as_int(frag.get("pps_diag_pps_edge_sequence"))
        gnss_ns = _as_int(frag.get("gnss_ns"))
        pps_edge_gnss_ns = _as_int(frag.get("pps_diag_pps_edge_gnss_ns"))
        fw_edge_delta = _as_int(frag.get("pps_diag_pps_edge_minus_event_ns"))

        boundary_gnss_ns = _as_int(frag.get("pps_diag_boundary_dwt_isr_entry_gnss_ns"))
        boundary_event_gnss_ns = _as_int(frag.get("pps_diag_gnss_ns_at_event"))
        fw_boundary_delta = _as_int(frag.get("pps_diag_boundary_dwt_isr_entry_minus_event_ns"))

        edge_delta_ns: Optional[int] = None
        boundary_delta_ns: Optional[int] = None

        if gnss_ns is not None and pps_edge_gnss_ns is not None:
            edge_delta_ns = pps_edge_gnss_ns - gnss_ns
            w_edge_delta.update(float(edge_delta_ns))
            if fw_edge_delta is not None and fw_edge_delta != edge_delta_ns:
                edge_fw_mismatches += 1
        else:
            missing_edge += 1

        if boundary_gnss_ns is not None and boundary_event_gnss_ns is not None:
            boundary_delta_ns = boundary_gnss_ns - boundary_event_gnss_ns
            w_boundary_delta.update(float(boundary_delta_ns))
        else:
            missing_boundary += 1

        print(
            f"{pps:>6d}  "
            f"{_fmt_i(seq):>6s}  "
            f"{_fmt_i(gnss_ns):>16s}  "
            f"{_fmt_i(pps_edge_gnss_ns):>18s}  "
            f"{_fmt_i(edge_delta_ns):>18s}  "
            f"{_fmt_i(fw_edge_delta):>14s}  "
            f"{_fmt_i(boundary_delta_ns):>18s}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown:                 {shown:,}")
    print(f"Missing PPS-edge rows:      {missing_edge:,}")
    print(f"Missing boundary rows:      {missing_boundary:,}")
    print()

    print("═" * 78)
    print("PPS-edge summary — physical PPS witness minus VCLOCK boundary")
    print("═" * 78)
    print()
    if w_edge_delta.n >= 1:
        print(f"  n       = {w_edge_delta.n:,}")
        print(f"  mean    = {w_edge_delta.mean:+,.3f} ns   ({w_edge_delta.mean/1000.0:+,.3f} µs)")
        print(f"  stddev  = {w_edge_delta.stddev:,.3f} ns   ({w_edge_delta.stddev/1000.0:,.3f} µs)")
        print(f"  stderr  = {w_edge_delta.stderr:,.3f} ns")
        print(f"  min     = {w_edge_delta.min_val:+,.0f} ns   ({w_edge_delta.min_val/1000.0:+,.3f} µs)")
        print(f"  max     = {w_edge_delta.max_val:+,.0f} ns   ({w_edge_delta.max_val/1000.0:+,.3f} µs)")
        print(f"  range   = {w_edge_delta.max_val - w_edge_delta.min_val:,.0f} ns")
    else:
        print("  (no PPS-edge rows)")
    print()

    print("═" * 78)
    print("Boundary summary — boundary ISR entry minus boundary event")
    print("═" * 78)
    print()
    if w_boundary_delta.n >= 1:
        print(f"  n       = {w_boundary_delta.n:,}")
        print(f"  mean    = {w_boundary_delta.mean:+,.3f} ns   ({w_boundary_delta.mean/1000.0:+,.3f} µs)")
        print(f"  stddev  = {w_boundary_delta.stddev:,.3f} ns   ({w_boundary_delta.stddev/1000.0:,.3f} µs)")
        print(f"  stderr  = {w_boundary_delta.stderr:,.3f} ns")
        print(f"  min     = {w_boundary_delta.min_val:+,.0f} ns   ({w_boundary_delta.min_val/1000.0:+,.3f} µs)")
        print(f"  max     = {w_boundary_delta.max_val:+,.0f} ns   ({w_boundary_delta.max_val/1000.0:+,.3f} µs)")
        print(f"  range   = {w_boundary_delta.max_val - w_boundary_delta.min_val:,.0f} ns")
    else:
        print("  (no boundary rows)")
    print()

    print("═" * 78)
    print("Firmware cross-check")
    print("═" * 78)
    print()
    print("The firmware publishes the PPS-edge delta as")
    print("pps_diag_pps_edge_minus_event_ns.  This should match our")
    print("computed pps_edge_delta_ns exactly for every row where both are present.")
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
    print("  pps_edge_delta_ns:")
    print("    This is the rail to trust for physical PPS closure work.")
    print("    It compares the physical PPS witness against the current")
    print("    VCLOCK-authored second boundary.")
    print()
    print("  boundary_delta_ns:")
    print("    This is NOT PPS latency.  It is the boundary-emitter ISR-entry")
    print("    lateness for the VCLOCK lane itself, kept here as a sanity rail.")
    print()
    print("  pps_diag_pps_edge_sequence:")
    print("    Track this alongside pps_count.  If sequence is stale, resets,")
    print("    or drifts unexpectedly, the witness path itself may be stale.")
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
