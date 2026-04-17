"""
ZPNet Raw Witness — PPS Lateness Relative to VCLOCK.

Under ZPNet's current doctrine, VCLOCK owns the canonical GNSS clock
and emits a synthetic 1-second boundary by pure arithmetic.  The
physical PPS GPIO edge is demoted to two narrow roles: a diagnostic
witness, and the trigger for TIMEBASE_FRAGMENT publication.

This tool answers one question:

    How long after the VCLOCK synthetic 1-second boundary does the
    physical PPS ISR actually fire?

For every TIMEBASE_FRAGMENT, it computes:

    delta_ns = pps_diag_dwt_isr_entry_gnss_ns - gnss_ns

  • gnss_ns                         — VCLOCK's synthetic 1-second boundary,
                                       expressed in GNSS nanoseconds (always
                                       an integer multiple of 1e9 by
                                       construction)

  • pps_diag_dwt_isr_entry_gnss_ns  — the GNSS nanosecond at which the PPS
                                       ISR was invoked, inferred from the
                                       ARM_DWT_CYCCNT captured as the first
                                       instruction of the ISR, translated
                                       through the DWT↔GNSS bridge

A positive delta means the PPS ISR fired AFTER the VCLOCK boundary.
Over time, the mean of delta is the static phase offset between the
two clocks; the stddev is the run-to-run jitter of the PPS arrival.

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
    print("  delta_ns = pps_diag_dwt_isr_entry_gnss_ns − gnss_ns")
    print()
    print("How long after the VCLOCK synthetic 1-second boundary the PPS ISR")
    print("was invoked, measured in the GNSS nanosecond timeline.")
    print()
    print("A positive delta means the PPS ISR fired AFTER the VCLOCK boundary.")
    print()

    # Column headers.
    header = (
        f"{'pps':>6s}  "
        f"{'gnss_ns':>22s}  "
        f"{'isr_entry_gnss_ns':>22s}  "
        f"{'delta_ns':>12s}  "
        f"{'delta_µs':>12s}"
    )
    sep = (
        f"{'─'*6}  "
        f"{'─'*22}  "
        f"{'─'*22}  "
        f"{'─'*12}  "
        f"{'─'*12}"
    )
    print(header)
    print(sep)

    w_delta = Welford()
    w_firmware_delta = Welford()  # for cross-check

    shown = 0
    missing = 0
    firmware_mismatches = 0

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _as_int(root.get("pps_count"))
        if pps is None:
            pps = _as_int(frag.get("teensy_pps_count"))
        if pps is None:
            continue

        gnss_ns = _as_int(frag.get("gnss_ns"))
        isr_entry_gnss_ns = _as_int(frag.get("pps_diag_dwt_isr_entry_gnss_ns"))
        firmware_delta = _as_int(frag.get("pps_diag_dwt_isr_entry_minus_event_ns"))

        if gnss_ns is None or isr_entry_gnss_ns is None:
            missing += 1
            continue

        delta_ns = isr_entry_gnss_ns - gnss_ns
        delta_us = delta_ns / 1000.0

        # Cross-check against the firmware's own computation (if present).
        if firmware_delta is not None:
            w_firmware_delta.update(float(firmware_delta))
            if firmware_delta != delta_ns:
                firmware_mismatches += 1

        w_delta.update(float(delta_ns))

        print(
            f"{pps:>6d}  "
            f"{gnss_ns:>22,d}  "
            f"{isr_entry_gnss_ns:>22,d}  "
            f"{delta_ns:>+12,d}  "
            f"{delta_us:>+12,.3f}"
        )

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Missing:    {missing:,}  (rows where either field was absent)")
    print()

    if w_delta.n < 2:
        print("(insufficient data for summary)")
        return

    # -----------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------
    print("═" * 78)
    print("Delta summary — PPS ISR entry minus VCLOCK synthetic boundary")
    print("═" * 78)
    print()
    print(f"  n       = {w_delta.n:,}")
    print(f"  mean    = {w_delta.mean:+,.3f} ns   ({w_delta.mean/1000.0:+,.3f} µs)")
    print(f"  stddev  = {w_delta.stddev:,.3f} ns   ({w_delta.stddev/1000.0:,.3f} µs)")
    print(f"  stderr  = {w_delta.stderr:,.3f} ns")
    print(f"  min     = {w_delta.min_val:+,.0f} ns   ({w_delta.min_val/1000.0:+,.3f} µs)")
    print(f"  max     = {w_delta.max_val:+,.0f} ns   ({w_delta.max_val/1000.0:+,.3f} µs)")
    print(f"  range   = {w_delta.max_val - w_delta.min_val:,.0f} ns")
    print()

    # -----------------------------------------------------------------
    # Firmware cross-check
    # -----------------------------------------------------------------
    if w_firmware_delta.n > 0:
        print("═" * 78)
        print("Firmware cross-check")
        print("═" * 78)
        print()
        print("The firmware publishes its own computation of the same delta as")
        print("pps_diag_dwt_isr_entry_minus_event_ns.  This should match our")
        print("computed delta_ns exactly for every row.")
        print()
        print(f"  Rows with firmware_delta present: {w_firmware_delta.n:,}")
        print(f"  Rows where firmware ≠ computed:   {firmware_mismatches:,}")
        if firmware_mismatches == 0:
            print(f"  ✓ Bit-exact agreement.")
        else:
            print(f"  ⚠ {firmware_mismatches:,} divergent rows — investigate.")
        print()

    # -----------------------------------------------------------------
    # Interpretation
    # -----------------------------------------------------------------
    print("═" * 78)
    print("Interpretation")
    print("═" * 78)
    print()

    mean_ns = w_delta.mean
    stddev_ns = w_delta.stddev
    range_ns = w_delta.max_val - w_delta.min_val

    print(f"  mean ({mean_ns:+,.0f} ns):")
    print(f"    The static phase offset between VCLOCK's synthetic boundary")
    print(f"    and the physical PPS arrival.  Components:")
    print(f"      • Phase set when the VCLOCK CH3 lane was bootstrapped")
    print(f"      • Fixed propagation delay in the GF-8802's PPS output path")
    print(f"      • Fixed GPIO ISR entry latency (NVIC + ISR preamble)")
    print()

    print(f"  stddev ({stddev_ns:,.1f} ns):")
    print(f"    Run-to-run jitter of the PPS arrival.  This is the direct")
    print(f"    measurement of PPS edge quality in our pipeline:")
    if stddev_ns < 100:
        verdict = "EXCELLENT — GPIO edge is tightly disciplined"
    elif stddev_ns < 500:
        verdict = "NORMAL — typical for consumer GNSS PPS output"
    elif stddev_ns < 1000:
        verdict = "ELEVATED — significant per-edge jitter in PPS output"
    else:
        verdict = "POOR — investigate USB/IRQ contention or scheduling"
    print(f"    → {verdict}")
    print()

    print(f"  range ({range_ns:,.0f} ns):")
    ratio_6sigma = range_ns / (6 * stddev_ns) if stddev_ns > 0 else float("inf")
    print(f"    Worst-case excursion.  range / 6σ = {ratio_6sigma:.2f}")
    if ratio_6sigma < 1.2:
        print(f"    → Distribution is approximately Gaussian.")
    elif ratio_6sigma < 2.5:
        print(f"    → Mildly heavy-tailed — a handful of outlier edges.")
    else:
        print(f"    → Heavy-tailed — a few pathological edges dominate the range.")
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