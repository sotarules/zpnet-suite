"""
ZPNet Phase Offset Report — Per-second OCXO phase capture validation

One row per PPS second showing the absolute clock state and the
phase detection outputs for both OCXOs.

Columns:
    PPS     — pps_count
    DWT cyc — teensy_dwt_cycles (64-bit total)
    GNSS ns — teensy_gnss_ns (64-bit total)
    OCXO1 ns — teensy_ocxo1_ns (64-bit total)
    OCXO2 ns — teensy_ocxo2_ns (64-bit total)
    Δ DWT   — DWT cycles this second (computed from consecutive records)
    Δ OX1   — OCXO1 ns this second (computed from consecutive records)
    φ1      — ocxo1_phase_offset_ns (0–99 ns)
    r1      — ocxo1_residual_ns (servo input)
    Δ OX2   — OCXO2 ns this second (computed from consecutive records)
    φ2      — ocxo2_phase_offset_ns (0–99 ns)
    r2      — ocxo2_residual_ns (servo input)

Running Welford statistics on phase offsets and residuals.
Gap detection resets deltas (no cross-gap differencing).

Usage:
    python -m zpnet.tests.phase_offset_report <campaign_name> [limit]
    .zt phase_offset_report Alpha1
    .zt phase_offset_report Alpha1 100
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


# ─────────────────────────────────────────────────────────────────────
# Welford online accumulator
# ─────────────────────────────────────────────────────────────────────

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

    def reset(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")


# ─────────────────────────────────────────────────────────────────────
# Field access — fragment-aware
# ─────────────────────────────────────────────────────────────────────

def _get(rec: Dict[str, Any], key: str, default=None):
    """
    Retrieve a field from a TIMEBASE record.

    New format: payload contains a 'fragment' dict with the raw
    TIMEBASE_FRAGMENT fields.  Top-level payload also has
    denormalized copies.  Check top-level first, then fragment.
    """
    val = rec.get(key)
    if val is not None:
        return val
    frag = rec.get("fragment")
    if frag is not None:
        val = frag.get(key)
        if val is not None:
            return val
    return default


def _int(rec: Dict[str, Any], key: str, default: Optional[int] = None) -> Optional[int]:
    val = _get(rec, key)
    return int(val) if val is not None else default


def _int64(rec: Dict[str, Any], key: str) -> Optional[int]:
    val = _get(rec, key)
    return int(val) if val is not None else None


# ─────────────────────────────────────────────────────────────────────
# Database fetch
# ─────────────────────────────────────────────────────────────────────

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
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


# ─────────────────────────────────────────────────────────────────────
# Analysis
# ─────────────────────────────────────────────────────────────────────

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()

    # ── Header ──

    print(
        f"  {'pps':>5s}"
        f"  {'DWT_cycles':>18s}"
        f"  {'GNSS_ns':>18s}"
        f"  {'OCXO1_ns':>18s}"
        f"  {'OCXO2_ns':>18s}"
        f"  {'Δ_DWT':>12s}"
        f"  {'Δ_OX1_ns':>12s}"
        f"  {'φ1':>4s}"
        f"  {'r1':>6s}"
        f"  {'Δ_OX2_ns':>12s}"
        f"  {'φ2':>4s}"
        f"  {'r2':>6s}"
    )
    print(
        f"  {'─' * 5}"
        f"  {'─' * 18}"
        f"  {'─' * 18}"
        f"  {'─' * 18}"
        f"  {'─' * 18}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 4}"
        f"  {'─' * 6}"
        f"  {'─' * 12}"
        f"  {'─' * 4}"
        f"  {'─' * 6}"
    )

    # ── Welford trackers ──

    w_phase1 = Welford()
    w_phase2 = Welford()
    w_resid1 = Welford()
    w_resid2 = Welford()
    w_delta_dwt = Welford()
    w_delta_ox1 = Welford()
    w_delta_ox2 = Welford()

    count = 0
    gaps = 0
    prev_pps: Optional[int] = None
    prev_dwt: Optional[int] = None
    prev_ox1: Optional[int] = None
    prev_ox2: Optional[int] = None

    detector_valid_count = 0
    detector_invalid_count = 0

    for rec in rows:
        pps = _int(rec, "pps_count")
        if pps is None:
            # Also check teensy_pps_count for compatibility
            pps = _int(rec, "teensy_pps_count")
        if pps is None:
            continue

        dwt_cyc   = _int64(rec, "teensy_dwt_cycles")
        gnss_ns   = _int64(rec, "teensy_gnss_ns")
        ocxo1_ns  = _int64(rec, "teensy_ocxo1_ns")
        ocxo2_ns  = _int64(rec, "teensy_ocxo2_ns")

        phase1    = _int(rec, "ocxo1_phase_offset_ns")
        phase2    = _int(rec, "ocxo2_phase_offset_ns")
        resid1    = _int(rec, "ocxo1_residual_ns")
        resid2    = _int(rec, "ocxo2_residual_ns")

        det_valid = _get(rec, "phase_detector_valid")
        res_valid = _get(rec, "phase_residual_valid")

        if det_valid:
            detector_valid_count += 1
        else:
            detector_invalid_count += 1

        # ── Gap detection ──

        if prev_pps is not None and pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  --- gap {prev_pps} → {pps} ({skipped}s lost) ---")
            prev_dwt = None
            prev_ox1 = None
            prev_ox2 = None

        # ── Compute deltas (only for consecutive records) ──

        delta_dwt_str = f"{'---':>12s}"
        delta_ox1_str = f"{'---':>12s}"
        delta_ox2_str = f"{'---':>12s}"

        if prev_dwt is not None and dwt_cyc is not None:
            d = dwt_cyc - prev_dwt
            delta_dwt_str = f"{d:>12,}"
            w_delta_dwt.update(float(d))

        if prev_ox1 is not None and ocxo1_ns is not None:
            d = ocxo1_ns - prev_ox1
            delta_ox1_str = f"{d:>12,}"
            w_delta_ox1.update(float(d))

        if prev_ox2 is not None and ocxo2_ns is not None:
            d = ocxo2_ns - prev_ox2
            delta_ox2_str = f"{d:>12,}"
            w_delta_ox2.update(float(d))

        # ── Phase & residual formatting ──

        phase1_str = f"{phase1:>4d}" if phase1 is not None else f"{'--':>4s}"
        phase2_str = f"{phase2:>4d}" if phase2 is not None else f"{'--':>4s}"
        resid1_str = f"{resid1:>+6d}" if resid1 is not None else f"{'---':>6s}"
        resid2_str = f"{resid2:>+6d}" if resid2 is not None else f"{'---':>6s}"

        # ── Welford updates (only when detector valid) ──

        if det_valid and phase1 is not None:
            w_phase1.update(float(phase1))
        if det_valid and phase2 is not None:
            w_phase2.update(float(phase2))
        if res_valid and resid1 is not None:
            w_resid1.update(float(resid1))
        if res_valid and resid2 is not None:
            w_resid2.update(float(resid2))

        # ── Print row ──

        dwt_str   = f"{dwt_cyc:>18,}" if dwt_cyc is not None else f"{'---':>18s}"
        gnss_str  = f"{gnss_ns:>18,}" if gnss_ns is not None else f"{'---':>18s}"
        ox1_str   = f"{ocxo1_ns:>18,}" if ocxo1_ns is not None else f"{'---':>18s}"
        ox2_str   = f"{ocxo2_ns:>18,}" if ocxo2_ns is not None else f"{'---':>18s}"

        print(
            f"  {pps:>5d}"
            f"  {dwt_str}"
            f"  {gnss_str}"
            f"  {ox1_str}"
            f"  {ox2_str}"
            f"  {delta_dwt_str}"
            f"  {delta_ox1_str}"
            f"  {phase1_str}"
            f"  {resid1_str}"
            f"  {delta_ox2_str}"
            f"  {phase2_str}"
            f"  {resid2_str}"
        )

        # ── Advance state ──

        prev_pps = pps
        prev_dwt = dwt_cyc
        prev_ox1 = ocxo1_ns
        prev_ox2 = ocxo2_ns

        count += 1
        if limit and count >= limit:
            break

    # ── Summary ──

    print()
    print(f"  {count} rows, {gaps} gap(s)")
    print(f"  Detector valid: {detector_valid_count}  invalid: {detector_invalid_count}")
    print()

    # ── Phase offset statistics ──

    print("── OCXO1 Phase Offset (φ1) ──")
    if w_phase1.n >= 2:
        print(f"    n       = {w_phase1.n:,}")
        print(f"    mean    = {w_phase1.mean:.2f} ns")
        print(f"    stddev  = {w_phase1.stddev:.2f} ns")
        print(f"    range   = [{w_phase1.min_val:.0f}, {w_phase1.max_val:.0f}] ns")
    else:
        print(f"    insufficient data (n={w_phase1.n})")
    print()

    print("── OCXO2 Phase Offset (φ2) ──")
    if w_phase2.n >= 2:
        print(f"    n       = {w_phase2.n:,}")
        print(f"    mean    = {w_phase2.mean:.2f} ns")
        print(f"    stddev  = {w_phase2.stddev:.2f} ns")
        print(f"    range   = [{w_phase2.min_val:.0f}, {w_phase2.max_val:.0f}] ns")
    else:
        print(f"    insufficient data (n={w_phase2.n})")
    print()

    # ── Residual statistics ──

    print("── OCXO1 Residual (r1) ──")
    if w_resid1.n >= 2:
        print(f"    n       = {w_resid1.n:,}")
        print(f"    mean    = {w_resid1.mean:+.2f} ns")
        print(f"    stddev  = {w_resid1.stddev:.2f} ns")
        print(f"    stderr  = {w_resid1.stderr:.3f} ns")
        print(f"    range   = [{w_resid1.min_val:+.0f}, {w_resid1.max_val:+.0f}] ns")
    else:
        print(f"    insufficient data (n={w_resid1.n})")
    print()

    print("── OCXO2 Residual (r2) ──")
    if w_resid2.n >= 2:
        print(f"    n       = {w_resid2.n:,}")
        print(f"    mean    = {w_resid2.mean:+.2f} ns")
        print(f"    stddev  = {w_resid2.stddev:.2f} ns")
        print(f"    stderr  = {w_resid2.stderr:.3f} ns")
        print(f"    range   = [{w_resid2.min_val:+.0f}, {w_resid2.max_val:+.0f}] ns")
    else:
        print(f"    insufficient data (n={w_resid2.n})")
    print()

    # ── Per-second delta statistics ──

    print("── Per-Second Deltas ──")
    if w_delta_dwt.n >= 2:
        print(f"    Δ DWT cycles:  mean={w_delta_dwt.mean:,.1f}  stddev={w_delta_dwt.stddev:.1f}")
    if w_delta_ox1.n >= 2:
        residual_ox1 = w_delta_ox1.mean - 1_000_000_000.0
        print(f"    Δ OCXO1 ns:    mean={w_delta_ox1.mean:,.1f}  (residual={residual_ox1:+.1f} ns/s)  stddev={w_delta_ox1.stddev:.1f}")
    if w_delta_ox2.n >= 2:
        residual_ox2 = w_delta_ox2.mean - 1_000_000_000.0
        print(f"    Δ OCXO2 ns:    mean={w_delta_ox2.mean:,.1f}  (residual={residual_ox2:+.1f} ns/s)  stddev={w_delta_ox2.stddev:.1f}")
    print()


# ─────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: phase_offset_report <campaign_name> [limit]")
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
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()