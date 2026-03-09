"""
ZPNet DWT Cycle Cross-Check Analyzer

Usage:
    python -m zpnet.tests.cycle_analyzer <campaign_name>
    .zt cycle_analyzer Baseline1

Compares two independent views of the DWT cycle count at each PPS edge:

  1. RAW SHADOW: diag_raw_shadow_cyc (32-bit DWT_CYCCNT from spin loop)
     Delta computed with 32-bit wraparound: (shadow[i] - shadow[i-1]) & 0xFFFFFFFF

  2. ACCUMULATOR: teensy_dwt_cycles (64-bit accumulator from dwt_at_pps_shadow())
     Delta computed as simple 64-bit subtraction.

If these agree, the 64-bit extension is faithful.
If they disagree, there is a bug in advance-then-rewind or zeroing.

Also shows the per-second nanosecond values and their deltas for
comparison with the cycle-domain data, to expose any normalization
errors in the ns conversion path.

v1: Initial version for v6 phase-coherent architecture.
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

DWT_FREQ_HZ = 1_008_000_000
DWT_NS_PER_CYCLE = 125.0 / 126.0
NS_PER_SECOND = 1_000_000_000


# ---------------------------------------------------------------------
# Fetch campaign TIMEBASE rows
# ---------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch all TIMEBASE rows for a campaign, ordered by pps_count."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        result.append(p)

    return result


# ---------------------------------------------------------------------
# Welford's online stats
# ---------------------------------------------------------------------

class WelfordStats:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float):
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
# Main analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"❌ No TIMEBASE rows found for campaign '{campaign}'")
        return

    print("=" * 70)
    print(f"ZPNet DWT CYCLE CROSS-CHECK — Campaign: {campaign}")
    print(f"  TIMEBASE records: {len(rows)}")
    print("=" * 70)

    # ── Extract all four series ──
    records = []
    for r in rows:
        pps = r.get("pps_count")
        shadow = r.get("diag_raw_shadow_cyc")
        accum = r.get("teensy_dwt_cycles")
        dwt_ns = r.get("teensy_dwt_ns")
        gnss_ns = r.get("teensy_gnss_ns")
        isr_cyc = r.get("diag_raw_isr_cyc")

        if pps is None:
            continue

        records.append({
            "pps": int(pps),
            "shadow": int(shadow) if shadow is not None else None,
            "accum": int(accum) if accum is not None else None,
            "dwt_ns": int(dwt_ns) if dwt_ns is not None else None,
            "gnss_ns": int(gnss_ns) if gnss_ns is not None else None,
            "isr_cyc": int(isr_cyc) if isr_cyc is not None else None,
        })

    if len(records) < 2:
        print("❌ Need at least 2 records")
        return

    print(f"  Valid records: {len(records)}")
    print(f"  PPS range: {records[0]['pps']} → {records[-1]['pps']}")

    # ── Compute deltas ──
    deltas = []
    for i in range(1, len(records)):
        prev = records[i - 1]
        curr = records[i]

        pps = curr["pps"]

        # Raw shadow delta (32-bit wraparound)
        shadow_delta = None
        if prev["shadow"] is not None and curr["shadow"] is not None:
            shadow_delta = (curr["shadow"] - prev["shadow"]) & 0xFFFFFFFF

        # Accumulator delta (64-bit)
        accum_delta = None
        if prev["accum"] is not None and curr["accum"] is not None:
            accum_delta = curr["accum"] - prev["accum"]

        # DWT ns delta
        dwt_ns_delta = None
        if prev["dwt_ns"] is not None and curr["dwt_ns"] is not None:
            dwt_ns_delta = curr["dwt_ns"] - prev["dwt_ns"]

        # GNSS ns delta
        gnss_ns_delta = None
        if prev["gnss_ns"] is not None and curr["gnss_ns"] is not None:
            gnss_ns_delta = curr["gnss_ns"] - prev["gnss_ns"]

        # ISR raw delta (32-bit wraparound)
        isr_delta = None
        if prev["isr_cyc"] is not None and curr["isr_cyc"] is not None:
            isr_delta = (curr["isr_cyc"] - prev["isr_cyc"]) & 0xFFFFFFFF

        # Discrepancy: shadow vs accumulator
        discrepancy = None
        if shadow_delta is not None and accum_delta is not None:
            discrepancy = accum_delta - shadow_delta

        deltas.append({
            "pps": pps,
            "shadow_delta": shadow_delta,
            "accum_delta": accum_delta,
            "isr_delta": isr_delta,
            "dwt_ns_delta": dwt_ns_delta,
            "gnss_ns_delta": gnss_ns_delta,
            "discrepancy": discrepancy,
            # Residuals (delta - expected)
            "shadow_residual": shadow_delta - DWT_FREQ_HZ if shadow_delta else None,
            "accum_residual": accum_delta - DWT_FREQ_HZ if accum_delta else None,
            "isr_residual": isr_delta - DWT_FREQ_HZ if isr_delta else None,
            "dwt_ns_residual": dwt_ns_delta - NS_PER_SECOND if dwt_ns_delta else None,
            "gnss_ns_residual": gnss_ns_delta - NS_PER_SECOND if gnss_ns_delta else None,
        })

    # ── Cross-check: shadow vs accumulator ──
    print()
    print("-" * 70)
    print("CROSS-CHECK: Raw Shadow vs 64-bit Accumulator")
    print("-" * 70)
    print()

    discrepancies = [(d["pps"], d["discrepancy"]) for d in deltas if d["discrepancy"] is not None]
    nonzero = [(pps, disc) for pps, disc in discrepancies if disc != 0]

    print(f"  Pairs compared: {len(discrepancies)}")
    print(f"  Exact matches:  {len(discrepancies) - len(nonzero)}")
    print(f"  Discrepancies:  {len(nonzero)}")

    if nonzero:
        print()
        print("  ⚠️  DISCREPANCIES FOUND — 64-bit extension disagrees with raw hardware:")
        print()
        disc_stats = WelfordStats()
        for pps, disc in nonzero:
            disc_stats.update(disc)

        print(f"  Discrepancy mean:   {disc_stats.mean:,.1f} cycles ({disc_stats.mean * DWT_NS_PER_CYCLE:,.1f} ns)")
        print(f"  Discrepancy stddev: {disc_stats.stddev:,.1f} cycles ({disc_stats.stddev * DWT_NS_PER_CYCLE:,.1f} ns)")
        print(f"  Discrepancy range:  {disc_stats.min_val:,.0f} to {disc_stats.max_val:,.0f} cycles")

        print()
        print("  First 20 discrepancies:")
        for pps, disc in nonzero[:20]:
            print(f"    pps={pps:>6d}  accumulator - shadow = {disc:+,d} cycles ({disc * DWT_NS_PER_CYCLE:+,.1f} ns)")
    else:
        print()
        print("  ✅ Perfect agreement — 64-bit extension is faithful to raw hardware")

    # ── Cross-check: shadow vs ISR ──
    print()
    print("-" * 70)
    print("CROSS-CHECK: Raw Shadow vs ISR Snapshot")
    print("-" * 70)
    print()

    shadow_isr_diffs = []
    for d in deltas:
        if d["shadow_delta"] is not None and d["isr_delta"] is not None:
            diff = d["isr_delta"] - d["shadow_delta"]
            shadow_isr_diffs.append((d["pps"], diff))

    if shadow_isr_diffs:
        diff_stats = WelfordStats()
        for _, diff in shadow_isr_diffs:
            diff_stats.update(diff)

        print(f"  Pairs compared: {len(shadow_isr_diffs)}")
        print(f"  Mean (ISR - shadow): {diff_stats.mean:+,.2f} cycles ({diff_stats.mean * DWT_NS_PER_CYCLE:+,.2f} ns)")
        print(f"  Stddev:              {diff_stats.stddev:,.2f} cycles ({diff_stats.stddev * DWT_NS_PER_CYCLE:,.2f} ns)")
        print(f"  Range:               {diff_stats.min_val:+,.0f} to {diff_stats.max_val:+,.0f} cycles")
        print()
        print("  (Positive = ISR captures more cycles = ISR is later than shadow)")
        print("  (This should be ~50 cycles = the TDC dispatch delta)")

    # ── Residual comparison: cycles vs nanoseconds ──
    print()
    print("-" * 70)
    print("RESIDUAL COMPARISON: Cycle domain vs Nanosecond domain")
    print("-" * 70)
    print()

    shadow_stats = WelfordStats()
    accum_stats = WelfordStats()
    dwt_ns_stats = WelfordStats()
    gnss_ns_stats = WelfordStats()

    for d in deltas:
        if d["shadow_residual"] is not None:
            shadow_stats.update(d["shadow_residual"])
        if d["accum_residual"] is not None:
            accum_stats.update(d["accum_residual"])
        if d["dwt_ns_residual"] is not None:
            dwt_ns_stats.update(d["dwt_ns_residual"])
        if d["gnss_ns_residual"] is not None:
            gnss_ns_stats.update(d["gnss_ns_residual"])

    hdr = f"  {'SOURCE':<24s} {'n':>6s}  {'mean':>14s}  {'stddev':>12s}  {'min':>12s}  {'max':>12s}"
    sep = f"  {'─' * 24} {'─' * 6}  {'─' * 14}  {'─' * 12}  {'─' * 12}  {'─' * 12}"

    print(hdr)
    print(sep)

    for label, stats, unit in [
        ("shadow (cycles)", shadow_stats, "cyc"),
        ("accumulator (cycles)", accum_stats, "cyc"),
        ("shadow (as ns)", shadow_stats, "ns"),
        ("accumulator (as ns)", accum_stats, "ns"),
        ("dwt_ns delta", dwt_ns_stats, "ns"),
        ("gnss_ns delta", gnss_ns_stats, "ns"),
    ]:
        if stats.n == 0:
            continue

        if unit == "ns" and "cycles" in label:
            # Convert cycles to ns for display
            m = stats.mean * DWT_NS_PER_CYCLE
            s = stats.stddev * DWT_NS_PER_CYCLE
            mn = stats.min_val * DWT_NS_PER_CYCLE
            mx = stats.max_val * DWT_NS_PER_CYCLE
        else:
            m = stats.mean
            s = stats.stddev
            mn = stats.min_val
            mx = stats.max_val

        u = "ns" if unit == "ns" or (unit == "ns" and "cycles" not in label) else "cyc"
        if "cycles" in label and unit == "cyc":
            u = "cyc"
        elif unit == "ns":
            u = "ns"

        print(f"  {label:<24s} {stats.n:>6d}  {m:>+14,.1f}  {s:>12,.1f}  {mn:>12,.1f}  {mx:>12,.1f}  {u}")

    # ── Nanosecond conversion check ──
    print()
    print("-" * 70)
    print("NANOSECOND CONVERSION CHECK")
    print("-" * 70)
    print()
    print("  For each record, compare:")
    print("    expected_ns = accum_cycles × 125 / 126")
    print("    actual_ns   = teensy_dwt_ns (from firmware)")
    print()

    ns_check_errs = []
    for r in records:
        if r["accum"] is not None and r["dwt_ns"] is not None:
            expected_ns = (r["accum"] * 125) // 126
            actual_ns = r["dwt_ns"]
            err = actual_ns - expected_ns
            ns_check_errs.append((r["pps"], err, r["accum"], actual_ns, expected_ns))

    if ns_check_errs:
        err_stats = WelfordStats()
        for _, err, _, _, _ in ns_check_errs:
            err_stats.update(err)

        nonzero_errs = [(pps, err) for pps, err, _, _, _ in ns_check_errs if err != 0]

        print(f"  Records checked: {len(ns_check_errs)}")
        print(f"  Exact matches:   {len(ns_check_errs) - len(nonzero_errs)}")
        print(f"  Nonzero errors:  {len(nonzero_errs)}")

        if err_stats.n > 0:
            print(f"  Error mean:      {err_stats.mean:+,.2f} ns")
            print(f"  Error stddev:    {err_stats.stddev:,.2f} ns")
            print(f"  Error range:     {err_stats.min_val:+,.0f} to {err_stats.max_val:+,.0f} ns")

        if nonzero_errs:
            print()
            print("  First 20 nonzero conversion errors:")
            for pps, err in nonzero_errs[:20]:
                print(f"    pps={pps:>6d}  actual_ns - expected_ns = {err:+,d} ns")

            # Check if errors are always the same sign / magnitude
            err_vals = [e for _, e in nonzero_errs]
            if all(e == err_vals[0] for e in err_vals):
                print(f"\n  ℹ️  All errors are identical: {err_vals[0]:+d} ns — likely integer truncation")
            elif all(e > 0 for e in err_vals):
                print(f"\n  ℹ️  All errors positive — firmware rounds up")
            elif all(e < 0 for e in err_vals):
                print(f"\n  ℹ️  All errors negative — firmware rounds down (integer truncation)")

    # ── Per-second detail dump (first 20, last 10) ──
    print()
    print("-" * 70)
    print("PER-SECOND DETAIL (first 20)")
    print("-" * 70)
    print()

    hdr2 = (f"  {'pps':>5s}  {'shadow_Δ':>14s}  {'accum_Δ':>14s}  {'disc':>6s}"
            f"  {'dwt_ns_Δ':>14s}  {'gnss_ns_Δ':>14s}  {'ns_ratio':>12s}")
    sep2 = f"  {'─' * 5}  {'─' * 14}  {'─' * 14}  {'─' * 6}  {'─' * 14}  {'─' * 14}  {'─' * 12}"

    print(hdr2)
    print(sep2)

    def print_delta_row(d):
        sd = f"{d['shadow_delta']:>14,}" if d['shadow_delta'] is not None else f"{'—':>14}"
        ad = f"{d['accum_delta']:>14,}" if d['accum_delta'] is not None else f"{'—':>14}"
        disc = f"{d['discrepancy']:>+6d}" if d['discrepancy'] is not None else f"{'—':>6}"
        dnd = f"{d['dwt_ns_delta']:>14,}" if d['dwt_ns_delta'] is not None else f"{'—':>14}"
        gnd = f"{d['gnss_ns_delta']:>14,}" if d['gnss_ns_delta'] is not None else f"{'—':>14}"

        # Compute ns/cycle ratio for this second
        ratio = ""
        if d['accum_delta'] and d['dwt_ns_delta'] and d['accum_delta'] > 0:
            r = d['dwt_ns_delta'] / d['accum_delta']
            ratio = f"{r:.9f}"

        print(f"  {d['pps']:>5d}  {sd}  {ad}  {disc}  {dnd}  {gnd}  {ratio:>12s}")

    for d in deltas[:20]:
        print_delta_row(d)

    if len(deltas) > 30:
        print()
        print(f"  ... ({len(deltas) - 30} rows omitted) ...")
        print()
        print(f"  {'pps':>5s}  {'shadow_Δ':>14s}  {'accum_Δ':>14s}  {'disc':>6s}"
              f"  {'dwt_ns_Δ':>14s}  {'gnss_ns_Δ':>14s}  {'ns_ratio':>12s}")
        print(sep2)
        for d in deltas[-10:]:
            print_delta_row(d)

    print()
    print("=" * 70)
    print("DONE")
    print("=" * 70)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: cycle_analyzer <campaign_name>")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT campaign, count(*) as cnt
                    FROM timebase
                    GROUP BY campaign
                    ORDER BY max(ts) DESC
                    LIMIT 10
                    """
                )
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'RECORDS':>8s}")
                print(f"  {'─' * 20} {'─' * 8}")
                for r in rows:
                    print(f"  {r['campaign']:<20s} {r['cnt']:>8d}")
        except Exception:
            pass
        sys.exit(1)

    campaign = sys.argv[1]
    analyze(campaign)


if __name__ == "__main__":
    main()