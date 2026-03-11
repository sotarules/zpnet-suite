"""
ZPNet Raw DWT Cycles — v2 Trend-Aware Statistics Edition

Per-second DWT cycle table with two statistical tracks:

  1. RESIDUAL from nominal (1,008,000,000) — the existing metric.
     Welford stddev here measures thermal drift range over the
     campaign.  Useful for PPB characterization but misleading
     as an interpolation confidence metric.

  2. PREDICTION residual — linear extrapolation from the two prior
     deltas.  predicted = 2 * prev - prev_prev.  Welford stddev
     here measures the actual second-to-second rate stability —
     the true interpolation uncertainty.

Recovery-aware: gaps in pps_count are shown as marker lines and
excluded from both Welford accumulators.  Prediction history is
also reset across gaps.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    .zt raw_cycles Prediction1
    .zt raw_cycles Prediction1 50
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db

DWT_FREQ_HZ = 1_008_000_000


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


class Welford:
    """Minimal Welford online accumulator."""

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
        f"  {'actual':>14s}"
        f"  {'residual':>9s}"
        f"  {'Δresid':>7s}"
        f"  {'predicted':>14s}"
        f"  {'pred_err':>9s}"
        f"  {'res_sd':>8s}"
        f"  {'pred_sd':>8s}"
    )
    print(
        f"  {'─'*5}"
        f"  {'─'*14}"
        f"  {'─'*9}"
        f"  {'─'*7}"
        f"  {'─'*14}"
        f"  {'─'*9}"
        f"  {'─'*8}"
        f"  {'─'*8}"
    )

    # Welford accumulators
    w_residual = Welford()       # residual from nominal (existing)
    w_prediction = Welford()     # prediction residual (new)

    # Prediction history
    prev_delta: Optional[int] = None
    prev_prev_delta: Optional[int] = None
    prev_residual: Optional[int] = None

    count = 0
    gaps = 0

    for i in range(1, len(rows)):
        prev = rows[i - 1]
        curr = rows[i]

        pps = curr.get("pps_count")
        prev_pps = prev.get("pps_count")
        start = prev.get("teensy_dwt_cycles")
        end = curr.get("teensy_dwt_cycles")

        if pps is None or prev_pps is None or start is None or end is None:
            continue

        pps = int(pps)
        prev_pps = int(prev_pps)
        start = int(start)
        end = int(end)

        # Recovery gap
        if pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(
                f"  {'':>5s}"
                f"  {'--- recovery':>14s}"
                f"  gap {prev_pps} → {pps}"
                f"  ({skipped} seconds lost)  ---"
            )
            # Reset prediction history across gaps
            prev_delta = None
            prev_prev_delta = None
            prev_residual = None
            continue

        actual = end - start
        residual = actual - DWT_FREQ_HZ

        # First difference of residual (delta-of-delta)
        if prev_residual is not None:
            delta_residual = residual - prev_residual
            delta_residual_str = f"{delta_residual:>+7d}"
        else:
            delta_residual_str = f"{'---':>7s}"

        # Prediction: linear extrapolation from two prior deltas
        if prev_delta is not None and prev_prev_delta is not None:
            predicted = 2 * prev_delta - prev_prev_delta
            pred_err = actual - predicted
            predicted_str = f"{predicted:>14,}"
            pred_err_str = f"{pred_err:>+9d}"
            w_prediction.update(float(pred_err))
        else:
            predicted_str = f"{'---':>14s}"
            pred_err_str = f"{'---':>9s}"

        # Residual from nominal Welford
        w_residual.update(float(residual))

        # Format stddevs
        res_sd_str = f"{w_residual.stddev:>8.1f}" if w_residual.n >= 2 else f"{'---':>8s}"
        pred_sd_str = f"{w_prediction.stddev:>8.1f}" if w_prediction.n >= 2 else f"{'---':>8s}"

        print(
            f"  {pps:>5d}"
            f"  {actual:>14,}"
            f"  {residual:>+9,}"
            f"  {delta_residual_str}"
            f"  {predicted_str}"
            f"  {pred_err_str}"
            f"  {res_sd_str}"
            f"  {pred_sd_str}"
        )

        # Shift history
        prev_prev_delta = prev_delta
        prev_delta = actual
        prev_residual = residual

        count += 1
        if limit and count >= limit:
            break

    # Summary
    print()
    print(f"  {count} rows, {gaps} recovery gap(s) skipped")
    print()

    if w_residual.n >= 2:
        print(f"  Residual from nominal:")
        print(f"    n={w_residual.n}  mean={w_residual.mean:+.1f}  stddev={w_residual.stddev:.1f}  stderr={w_residual.stderr:.2f}")

    if w_prediction.n >= 2:
        print(f"  Prediction (linear extrapolation):")
        print(f"    n={w_prediction.n}  mean={w_prediction.mean:+.1f}  stddev={w_prediction.stddev:.1f}  stderr={w_prediction.stderr:.2f}")

        # Improvement factor
        if w_residual.stddev > 0:
            improvement = w_residual.stddev / w_prediction.stddev
            print(f"    improvement: {improvement:.1f}x tighter than residual stddev")

    print()


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
                print(f"  {'─'*20} {'─'*8}")
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