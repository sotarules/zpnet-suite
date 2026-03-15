"""
ZPNet Raw DWT Cycles — v4 Random Walk Prediction Quality

Per-second DWT cycle table showing how well the random walk predictor
(predicted = prev_delta) tracks the actual DWT cycle count.

The random walk model is the authoritative prediction approach for
ZPNet's thermally drifting crystal: the current rate is the best
predictor of the next rate.  This tool visualizes the prediction
residuals, their running statistics, and flags outliers.

Recovery-aware: gaps in pps_count reset prediction history.

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
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val: Optional[float] = None
        self.max_val: Optional[float] = None

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        if self.min_val is None or x < self.min_val:
            self.min_val = x
        if self.max_val is None or x > self.max_val:
            self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0

    @property
    def range(self) -> float:
        if self.min_val is not None and self.max_val is not None:
            return self.max_val - self.min_val
        return 0.0


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()

    print(
        f"  {'pps':>5s}"
        f"  {'actual':>14s}"
        f"  {'nominal_res':>11s}"
        f"  {'predicted':>14s}"
        f"  {'pred_err':>9s}"
        f"  {'mean':>8s}"
        f"  {'stddev':>8s}"
        f"  {'stderr':>8s}"
    )
    print(
        f"  {'─' * 5}"
        f"  {'─' * 14}"
        f"  {'─' * 11}"
        f"  {'─' * 14}"
        f"  {'─' * 9}"
        f"  {'─' * 8}"
        f"  {'─' * 8}"
        f"  {'─' * 8}"
    )

    w_pred = Welford()
    w_nominal = Welford()

    prev_delta: Optional[int] = None
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

        if pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  --- gap {prev_pps} → {pps} ({skipped}s lost) ---")
            prev_delta = None
            continue

        actual = end - start
        nominal_res = actual - DWT_FREQ_HZ
        w_nominal.update(float(nominal_res))

        if prev_delta is not None:
            pred_err = actual - prev_delta
            w_pred.update(float(pred_err))

            mean_str = f"{w_pred.mean:>+8.1f}" if w_pred.n >= 1 else f"{'---':>8s}"
            sd_str = f"{w_pred.stddev:>8.1f}" if w_pred.n >= 2 else f"{'---':>8s}"
            se_str = f"{w_pred.stderr:>8.2f}" if w_pred.n >= 2 else f"{'---':>8s}"

            print(
                f"  {pps:>5d}"
                f"  {actual:>14,}"
                f"  {nominal_res:>+11,}"
                f"  {prev_delta:>14,}"
                f"  {pred_err:>+9d}"
                f"  {mean_str}"
                f"  {sd_str}"
                f"  {se_str}"
            )
        else:
            print(
                f"  {pps:>5d}"
                f"  {actual:>14,}"
                f"  {nominal_res:>+11,}"
                f"  {'(seed)':>14s}"
                f"  {'':>9s}"
                f"  {'':>8s}"
                f"  {'':>8s}"
                f"  {'':>8s}"
            )

        prev_delta = actual
        count += 1
        if limit and count >= limit:
            break

    print()
    print(f"  {count} rows, {gaps} recovery gap(s)")
    print()

    if w_nominal.n >= 2:
        print(f"  Nominal residual (actual - {DWT_FREQ_HZ:,}):")
        print(f"    n       = {w_nominal.n:,}")
        print(f"    mean    = {w_nominal.mean:+.2f} cycles")
        print(f"    stddev  = {w_nominal.stddev:.2f} cycles  (thermal drift range)")
        print(f"    min     = {w_nominal.min_val:+.0f}")
        print(f"    max     = {w_nominal.max_val:+.0f}")
        print(f"    range   = {w_nominal.range:.0f}")
        print()

    if w_pred.n >= 2:
        print(f"  Random walk prediction (predicted = prev_delta):")
        print(f"    n       = {w_pred.n:,}")
        print(f"    mean    = {w_pred.mean:+.2f} cycles")
        print(f"    stddev  = {w_pred.stddev:.2f} cycles  (interpolation uncertainty)")
        print(f"    stderr  = {w_pred.stderr:.3f} cycles")
        print(f"    min     = {w_pred.min_val:+.0f}")
        print(f"    max     = {w_pred.max_val:+.0f}")
        print(f"    range   = {w_pred.range:.0f}")
        print()

        if w_nominal.n >= 2 and w_nominal.stddev > 0:
            ratio = w_nominal.stddev / w_pred.stddev
            print(f"  Prediction quality:")
            print(f"    nominal stddev / prediction stddev = {ratio:.1f}x")
            print(f"    The random walk predictor reduces uncertainty by {ratio:.1f}x")
            print(f"    compared to assuming a fixed nominal frequency.")
            print()

        pred_ns = w_pred.stddev * (125.0 / 126.0)
        print(f"  Interpolation bound:")
        print(f"    1σ prediction uncertainty = {pred_ns:.1f} ns")
        print(f"    This is the authoritative sub-second interpolation error.")
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