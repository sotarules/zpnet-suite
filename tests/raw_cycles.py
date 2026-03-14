"""
ZPNet Raw DWT Cycles — v3 Predictor Shootout Edition

Per-second DWT cycle table with three statistical tracks:

  1. RESIDUAL from nominal (1,008,000,000) — thermal drift range.

  2. LINEAR prediction — extrapolate from two prior deltas.
     predicted = 2 * prev - prev_prev.

  3. RANDOM WALK prediction — last delta IS the prediction.
     predicted = prev.

The random walk hypothesis: for a thermally drifting crystal,
the current rate is the best predictor of the next rate.  Linear
extrapolation assumes trends continue, but if the drift is a
random walk (direction changes unpredictably), extrapolation
overshoots on reversals and inflates prediction error.

The shootout compares both predictors head-to-head on the same
data, using only seconds where BOTH can score (i.e., where
linear extrapolation has enough history).

Recovery-aware: gaps in pps_count reset all prediction history.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    .zt raw_cycles Baseline4
    .zt raw_cycles Baseline4 50
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

    # Header
    print(
        f"  {'pps':>5s}"
        f"  {'actual':>14s}"
        f"  {'residual':>9s}"
        f"  {'Δresid':>7s}"
        f"  {'lin_pred':>14s}"
        f"  {'lin_err':>8s}"
        f"  {'rw_pred':>14s}"
        f"  {'rw_err':>8s}"
        f"  {'res_sd':>8s}"
        f"  {'lin_sd':>8s}"
        f"  {'rw_sd':>8s}"
    )
    print(
        f"  {'─'*5}"
        f"  {'─'*14}"
        f"  {'─'*9}"
        f"  {'─'*7}"
        f"  {'─'*14}"
        f"  {'─'*8}"
        f"  {'─'*14}"
        f"  {'─'*8}"
        f"  {'─'*8}"
        f"  {'─'*8}"
        f"  {'─'*8}"
    )

    # Welford accumulators
    w_residual = Welford()       # residual from nominal
    w_linear = Welford()         # linear extrapolation prediction error
    w_rw = Welford()             # random walk prediction error

    # Head-to-head: only scored when BOTH predictors fire
    w_h2h_linear = Welford()
    w_h2h_rw = Welford()

    # Wins tracking
    linear_wins = 0
    rw_wins = 0
    ties = 0

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
            prev_delta = None
            prev_prev_delta = None
            prev_residual = None
            continue

        actual = end - start
        residual = actual - DWT_FREQ_HZ

        # First difference of residual
        if prev_residual is not None:
            delta_residual = residual - prev_residual
            delta_residual_str = f"{delta_residual:>+7d}"
        else:
            delta_residual_str = f"{'---':>7s}"

        # Linear prediction: extrapolate from two prior deltas
        lin_pred_str = f"{'---':>14s}"
        lin_err_str = f"{'---':>8s}"
        lin_err_val: Optional[int] = None

        if prev_delta is not None and prev_prev_delta is not None:
            lin_predicted = 2 * prev_delta - prev_prev_delta
            lin_err = actual - lin_predicted
            lin_err_val = lin_err
            lin_pred_str = f"{lin_predicted:>14,}"
            lin_err_str = f"{lin_err:>+8d}"
            w_linear.update(float(lin_err))

        # Random walk prediction: last delta is the prediction
        rw_pred_str = f"{'---':>14s}"
        rw_err_str = f"{'---':>8s}"
        rw_err_val: Optional[int] = None

        if prev_delta is not None:
            rw_predicted = prev_delta
            rw_err = actual - rw_predicted
            rw_err_val = rw_err
            rw_pred_str = f"{rw_predicted:>14,}"
            rw_err_str = f"{rw_err:>+8d}"
            w_rw.update(float(rw_err))

        # Head-to-head scoring (only when both fire)
        if lin_err_val is not None and rw_err_val is not None:
            w_h2h_linear.update(float(lin_err_val))
            w_h2h_rw.update(float(rw_err_val))

            abs_lin = abs(lin_err_val)
            abs_rw = abs(rw_err_val)
            if abs_lin < abs_rw:
                linear_wins += 1
            elif abs_rw < abs_lin:
                rw_wins += 1
            else:
                ties += 1

        # Residual Welford
        w_residual.update(float(residual))

        # Format stddevs
        res_sd_str = f"{w_residual.stddev:>8.1f}" if w_residual.n >= 2 else f"{'---':>8s}"
        lin_sd_str = f"{w_linear.stddev:>8.1f}" if w_linear.n >= 2 else f"{'---':>8s}"
        rw_sd_str = f"{w_rw.stddev:>8.1f}" if w_rw.n >= 2 else f"{'---':>8s}"

        print(
            f"  {pps:>5d}"
            f"  {actual:>14,}"
            f"  {residual:>+9,}"
            f"  {delta_residual_str}"
            f"  {lin_pred_str}"
            f"  {lin_err_str}"
            f"  {rw_pred_str}"
            f"  {rw_err_str}"
            f"  {res_sd_str}"
            f"  {lin_sd_str}"
            f"  {rw_sd_str}"
        )

        # Shift history
        prev_prev_delta = prev_delta
        prev_delta = actual
        prev_residual = residual

        count += 1
        if limit and count >= limit:
            break

    # ── Summary ──
    print()
    print(f"  {count} rows, {gaps} recovery gap(s) skipped")
    print()

    print(f"  ═══════════════════════════════════════════════════════")
    print(f"  PREDICTOR SHOOTOUT")
    print(f"  ═══════════════════════════════════════════════════════")
    print()

    if w_residual.n >= 2:
        print(f"  Residual from nominal ({DWT_FREQ_HZ:,}):")
        print(f"    n={w_residual.n}  mean={w_residual.mean:+.2f}  stddev={w_residual.stddev:.2f}")
        print(f"    min={w_residual.min_val:+.0f}  max={w_residual.max_val:+.0f}  range={w_residual.range:.0f}")
        print()

    if w_linear.n >= 2:
        print(f"  Linear extrapolation (predicted = 2×prev - prev_prev):")
        print(f"    n={w_linear.n}  mean={w_linear.mean:+.2f}  stddev={w_linear.stddev:.2f}")
        print(f"    min={w_linear.min_val:+.0f}  max={w_linear.max_val:+.0f}  range={w_linear.range:.0f}")
        print()

    if w_rw.n >= 2:
        print(f"  Random walk (predicted = prev):")
        print(f"    n={w_rw.n}  mean={w_rw.mean:+.2f}  stddev={w_rw.stddev:.2f}")
        print(f"    min={w_rw.min_val:+.0f}  max={w_rw.max_val:+.0f}  range={w_rw.range:.0f}")
        print()

    # Head-to-head
    if w_h2h_linear.n >= 2 and w_h2h_rw.n >= 2:
        print(f"  ── Head-to-head (same {w_h2h_linear.n:,} seconds) ──")
        print()
        print(f"    {'':20s} {'LINEAR':>10s}  {'RANDOM WALK':>12s}")
        print(f"    {'':20s} {'──────────':>10s}  {'────────────':>12s}")
        print(f"    {'stddev':20s} {w_h2h_linear.stddev:>10.2f}  {w_h2h_rw.stddev:>12.2f}")
        print(f"    {'mean':20s} {w_h2h_linear.mean:>+10.2f}  {w_h2h_rw.mean:>+12.2f}")
        print(f"    {'min':20s} {w_h2h_linear.min_val:>+10.0f}  {w_h2h_rw.min_val:>+12.0f}")
        print(f"    {'max':20s} {w_h2h_linear.max_val:>+10.0f}  {w_h2h_rw.max_val:>+12.0f}")
        print(f"    {'range':20s} {w_h2h_linear.range:>10.0f}  {w_h2h_rw.range:>12.0f}")
        print()

        total_scored = linear_wins + rw_wins + ties
        print(f"    Per-second wins ({total_scored:,} scored):")
        print(f"      Linear wins:      {linear_wins:>8,}  ({100*linear_wins/total_scored:.1f}%)")
        print(f"      Random walk wins: {rw_wins:>8,}  ({100*rw_wins/total_scored:.1f}%)")
        print(f"      Ties:             {ties:>8,}  ({100*ties/total_scored:.1f}%)")
        print()

        if w_h2h_rw.stddev > 0 and w_h2h_linear.stddev > 0:
            ratio = w_h2h_linear.stddev / w_h2h_rw.stddev
            if ratio > 1.0:
                winner = "RANDOM WALK"
                improvement = ratio
            else:
                winner = "LINEAR"
                improvement = 1.0 / ratio

            print(f"    VERDICT: {winner} wins by {improvement:.2f}x lower stddev")
        print()

    print(f"  ═══════════════════════════════════════════════════════")
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