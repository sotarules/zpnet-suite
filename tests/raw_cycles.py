"""
ZPNet Raw Cycles — v6 Fragment-Aware DWT + OCXO Prediction Monitor

One row per second showing DWT and OCXO edge timing:

    predicted   = fragment.dwt_cycle_count_next_second_prediction
    actual      = fragment.dwt_cycle_count_between_pps
    residual    = actual - predicted

Also shows OCXO1/OCXO2 dwt_cycles_between_edges to surface the
42-cycle quantization artifact under investigation.

Running Welford statistics accumulate across the campaign, reset
on recovery gaps.

v6 changes:
  - Fragment-aware: reads all fields from the 'fragment' sub-dict
    inside the TIMEBASE payload (v2026-04+ format).
  - Falls back to top-level fields for backward compatibility with
    pre-fragment TIMEBASE records.
  - OCXO1/OCXO2 DWT delta columns added for quantization forensics.
  - Separate Welford trackers for DWT, OCXO1, and OCXO2 deltas.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    .zt raw_cycles Phase2
    .zt raw_cycles Phase2 50
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
    __slots__ = ("n", "mean", "m2")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2

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


# ─────────────────────────────────────────────────────────────────────
# Fragment-aware field access
# ─────────────────────────────────────────────────────────────────────

def _frag(rec: Dict[str, Any], key: str) -> Any:
    """Read a field from the fragment sub-dict, fall back to top-level."""
    frag = rec.get("fragment")
    if isinstance(frag, dict):
        v = frag.get(key)
        if v is not None:
            return v
    return rec.get(key)


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

    # Header
    print(
        f"  {'pps':>5s}"
        f"  {'predicted':>14s}"
        f"  {'actual':>14s}"
        f"  {'residual':>9s}"
        f"  {'mean':>8s}"
        f"  {'stddev':>8s}"
        f"  {'o1_dwt':>14s}"
        f"  {'o2_dwt':>14s}"
        f"  {'o1-pps':>9s}"
        f"  {'o2-pps':>9s}"
    )
    print(
        f"  {'─' * 5}"
        f"  {'─' * 14}"
        f"  {'─' * 14}"
        f"  {'─' * 9}"
        f"  {'─' * 8}"
        f"  {'─' * 8}"
        f"  {'─' * 14}"
        f"  {'─' * 14}"
        f"  {'─' * 9}"
        f"  {'─' * 9}"
    )

    w_dwt = Welford()
    w_o1 = Welford()
    w_o2 = Welford()
    count = 0
    gaps = 0
    prev_pps: Optional[int] = None

    for rec in rows:
        pps = rec.get("pps_count")
        if pps is None:
            continue
        pps = int(pps)

        # Gap detection — reset Welford on non-consecutive PPS
        if prev_pps is not None and pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  --- gap {prev_pps} → {pps} ({skipped}s lost, stats reset) ---")
            w_dwt.reset()
            w_o1.reset()
            w_o2.reset()

        prev_pps = pps

        # DWT fields — fragment-aware with fallback
        predicted = _frag(rec, "dwt_cycle_count_next_second_prediction")
        actual = _frag(rec, "dwt_cycle_count_between_pps")

        # Legacy fallback for pre-fragment records
        if predicted is None:
            predicted = rec.get("dwt_cycles_per_pps")
        if actual is None:
            actual = rec.get("dwt_delta_raw")

        # OCXO DWT deltas from fragment
        o1_dwt = _frag(rec, "ocxo1_dwt_cycles_between_edges")
        o2_dwt = _frag(rec, "ocxo2_dwt_cycles_between_edges")

        # DWT prediction residual
        if predicted is not None and actual is not None:
            predicted = int(predicted)
            actual = int(actual)
            residual = actual - predicted
            w_dwt.update(float(residual))

            res_str = f"{residual:>+9d}"
            mean_str = f"{w_dwt.mean:>+8.1f}" if w_dwt.n >= 1 else f"{'---':>8s}"
            sd_str = f"{w_dwt.stddev:>8.1f}" if w_dwt.n >= 2 else f"{'---':>8s}"
        else:
            actual = None
            res_str = f"{'---':>9s}"
            mean_str = f"{'---':>8s}"
            sd_str = f"{'---':>8s}"

        # OCXO delta vs PPS delta (shows quantization offset)
        if o1_dwt is not None:
            o1_dwt = int(o1_dwt)
            w_o1.update(float(o1_dwt))
        if o2_dwt is not None:
            o2_dwt = int(o2_dwt)
            w_o2.update(float(o2_dwt))

        o1_diff = o1_dwt - actual if (o1_dwt is not None and actual is not None) else None
        o2_diff = o2_dwt - actual if (o2_dwt is not None and actual is not None) else None

        print(
            f"  {pps:>5d}"
            f"  {predicted:>14,}" if predicted is not None else f"  {'---':>14s}",
            end=""
        )
        print(
            f"  {actual:>14,}" if actual is not None else f"  {'---':>14s}",
            end=""
        )
        print(
            f"  {res_str}"
            f"  {mean_str}"
            f"  {sd_str}",
            end=""
        )
        print(
            f"  {o1_dwt:>14,}" if o1_dwt is not None else f"  {'---':>14s}",
            end=""
        )
        print(
            f"  {o2_dwt:>14,}" if o2_dwt is not None else f"  {'---':>14s}",
            end=""
        )
        print(
            f"  {o1_diff:>+9d}" if o1_diff is not None else f"  {'---':>9s}",
            end=""
        )
        print(
            f"  {o2_diff:>+9d}" if o2_diff is not None else f"  {'---':>9s}"
        )

        count += 1
        if limit and count >= limit:
            break

    # ── Summary ──
    print()
    print(f"  {count} rows, {gaps} recovery gap(s)")
    print()

    if w_dwt.n >= 2:
        pred_ns = w_dwt.stddev * (125.0 / 126.0)
        print(f"  DWT prediction residual (actual - predicted):")
        print(f"    n       = {w_dwt.n:,}")
        print(f"    mean    = {w_dwt.mean:+.2f} cycles")
        print(f"    stddev  = {w_dwt.stddev:.2f} cycles")
        print(f"    stderr  = {w_dwt.stderr:.3f} cycles")
        print(f"    1σ interpolation uncertainty = {pred_ns:.1f} ns")
        print()

    if w_o1.n >= 2:
        print(f"  OCXO1 DWT cycles between edges:")
        print(f"    n       = {w_o1.n:,}")
        print(f"    mean    = {w_o1.mean:,.2f}")
        print(f"    stddev  = {w_o1.stddev:.2f} cycles")
        print(f"    range   = {w_o1.mean - 3*w_o1.stddev:,.0f} .. {w_o1.mean + 3*w_o1.stddev:,.0f} (3σ)")
        print()

    if w_o2.n >= 2:
        print(f"  OCXO2 DWT cycles between edges:")
        print(f"    n       = {w_o2.n:,}")
        print(f"    mean    = {w_o2.mean:,.2f}")
        print(f"    stddev  = {w_o2.stddev:.2f} cycles")
        print(f"    range   = {w_o2.mean - 3*w_o2.stddev:,.0f} .. {w_o2.mean + 3*w_o2.stddev:,.0f} (3σ)")
        print()

    # ── Quantization summary ──
    if w_o1.n >= 10 and w_dwt.n >= 10:
        print(f"  Quantization analysis:")
        print(f"    DWT PPS stddev:   {w_dwt.stddev:.2f} cycles")
        print(f"    OCXO1 edge stddev: {w_o1.stddev:.2f} cycles")
        print(f"    OCXO2 edge stddev: {w_o2.stddev:.2f} cycles")
        if w_o1.stddev > 15:
            print(f"    ⚠ OCXO1 bimodal regime likely (stddev >> DWT noise)")
        if w_o2.stddev > 15:
            print(f"    ⚠ OCXO2 bimodal regime likely (stddev >> DWT noise)")
        print()


# ─────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit]")
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