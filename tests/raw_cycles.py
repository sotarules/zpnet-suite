"""
ZPNet Raw Cycles — v5 DWT Prediction Residual Monitor

One row per second showing the DWT random walk prediction quality:

    predicted   = dwt_cycles_per_pps  (Teensy: prev_delta)
    actual      = dwt_delta_raw       (Teensy: measured this second)
    residual    = actual - predicted

Running Welford statistics (mean, stddev, stderr) accumulate across
the campaign, reset on recovery gaps.

The prediction residual stddev is the authoritative interpolation
uncertainty for sub-second DWT time.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    .zt raw_cycles Baseline8
    .zt raw_cycles Baseline8 50
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
        f"  {'stderr':>8s}"
    )
    print(
        f"  {'─' * 5}"
        f"  {'─' * 14}"
        f"  {'─' * 14}"
        f"  {'─' * 9}"
        f"  {'─' * 8}"
        f"  {'─' * 8}"
        f"  {'─' * 8}"
    )

    w = Welford()
    count = 0
    gaps = 0
    prev_pps: Optional[int] = None

    for rec in rows:
        pps = rec.get("pps_count")
        predicted = rec.get("dwt_cycles_per_pps")
        actual = rec.get("dwt_delta_raw")

        if pps is None:
            continue

        pps = int(pps)

        # Gap detection — reset Welford on non-consecutive PPS
        if prev_pps is not None and pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  --- gap {prev_pps} → {pps} ({skipped}s lost, stats reset) ---")
            w.reset()

        prev_pps = pps

        # Both fields required for a meaningful row
        if predicted is None or actual is None:
            continue

        predicted = int(predicted)
        actual = int(actual)
        residual = actual - predicted

        w.update(float(residual))

        mean_str = f"{w.mean:>+8.1f}" if w.n >= 1 else f"{'---':>8s}"
        sd_str = f"{w.stddev:>8.1f}" if w.n >= 2 else f"{'---':>8s}"
        se_str = f"{w.stderr:>8.3f}" if w.n >= 2 else f"{'---':>8s}"

        print(
            f"  {pps:>5d}"
            f"  {predicted:>14,}"
            f"  {actual:>14,}"
            f"  {residual:>+9d}"
            f"  {mean_str}"
            f"  {sd_str}"
            f"  {se_str}"
        )

        count += 1
        if limit and count >= limit:
            break

    # Summary
    print()
    print(f"  {count} rows, {gaps} recovery gap(s)")
    print()

    if w.n >= 2:
        pred_ns = w.stddev * (125.0 / 126.0)
        print(f"  DWT prediction residual (actual - predicted):")
        print(f"    n       = {w.n:,}")
        print(f"    mean    = {w.mean:+.2f} cycles")
        print(f"    stddev  = {w.stddev:.2f} cycles")
        print(f"    stderr  = {w.stderr:.3f} cycles")
        print()
        print(f"  Interpolation bound:")
        print(f"    1σ prediction uncertainty = {pred_ns:.1f} ns")
        print(f"    This is the authoritative sub-second interpolation error.")
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