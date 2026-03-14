"""
ZPNet GNSS VCLOCK Forensics — QTimer1 Dual-Edge Analysis

Uses Pi-side TIMEBASE fields to analyze GNSS VCLOCK behavior.
Detects missed PPS edges (doubled deltas), tracks ISR residuals,
and analyzes alternation pattern.

Key fields used:
  • isr_residual_gnss   — ISR-level GNSS residual (raw 20 MHz space)
  • ocxo1_delta_raw     — OCXO1 delta (doubles when PPS missed)
  • ocxo2_delta_raw     — OCXO2 delta (doubles when PPS missed)
  • pps_rejected_total  — cumulative PPS rejection count
  • pps_rejected_remainder — remainder that caused rejection

Usage:
    python -m zpnet.tests.gnss_forensics <campaign_name> [limit]
    .zt gnss_forensics Calibrate2
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db

OCXO_EXPECTED = 10_000_000


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


def safe_int(val, default=0) -> int:
    if val is None:
        return default
    return int(val)


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()

    hdr = (
        f"  {'pps':>5s}"
        f"  {'isr_gn':>9s}"
        f"  {'o1_delta':>12s}"
        f"  {'o1_res':>7s}"
        f"  {'o2_delta':>12s}"
        f"  {'o2_res':>7s}"
        f"  {'rej_tot':>7s}"
        f"  {'rej_rem':>9s}"
        f"  {'isr_dw':>7s}"
        f"  {'status':>10s}"
    )
    sep = (
        f"  {'─'*5}"
        f"  {'─'*9}"
        f"  {'─'*12}"
        f"  {'─'*7}"
        f"  {'─'*12}"
        f"  {'─'*7}"
        f"  {'─'*7}"
        f"  {'─'*9}"
        f"  {'─'*7}"
        f"  {'─'*10}"
    )
    print(hdr)
    print(sep)

    w_isr = Welford()       # ISR residual (non-doubled only)
    prev_rej_tot = 0
    prev_pps: Optional[int] = None

    doubled = 0
    new_rejections = 0
    count = 0
    gaps = 0

    isr_vals = []           # for alternation analysis

    for rec in rows:
        pps = safe_int(rec.get("pps_count", rec.get("teensy_pps_count")))
        isr_gn = safe_int(rec.get("isr_residual_gnss"))
        o1_raw = safe_int(rec.get("ocxo1_delta_raw"))
        o2_raw = safe_int(rec.get("ocxo2_delta_raw"))
        isr_dw = safe_int(rec.get("isr_residual_dwt"))
        rej_tot = safe_int(rec.get("pps_rejected_total"))
        rej_rem = safe_int(rec.get("pps_rejected_remainder"))

        if prev_pps is not None and pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  --- gap {prev_pps} → {pps} ({skipped}s lost) ---")

        prev_pps = pps

        o1_res = o1_raw - OCXO_EXPECTED
        o2_res = o2_raw - OCXO_EXPECTED

        # Detect missed PPS: both OCXOs show ~2x normal delta
        is_doubled = (o1_raw > 15_000_000 or o2_raw > 15_000_000)

        # Detect new rejections since last record
        rej_new = rej_tot - prev_rej_tot
        if rej_new > 0:
            new_rejections += rej_new
        prev_rej_tot = rej_tot

        if is_doubled:
            doubled += 1
            status = "<<DOUBLED"
        elif rej_new > 0:
            status = f"<<REJ+{rej_new}"
        else:
            status = ""

        # Accumulate ISR stats for non-doubled
        if not is_doubled:
            w_isr.update(float(isr_gn))
            isr_vals.append(isr_gn)

        isr_sd_str = f""

        print(
            f"  {pps:>5d}"
            f"  {isr_gn:>+9d}"
            f"  {o1_raw:>12,}"
            f"  {o1_res:>+7d}"
            f"  {o2_raw:>12,}"
            f"  {o2_res:>+7d}"
            f"  {rej_tot:>7d}"
            f"  {rej_rem:>9d}"
            f"  {isr_dw:>+7d}"
            f"  {status}"
        )

        count += 1
        if limit and count >= limit:
            break

    print()
    print(f"  {count} rows, {gaps} gap(s), {doubled} doubled(s), {new_rejections} new rejection(s)")
    print()

    if w_isr.n >= 2:
        print(f"  ISR residual GNSS (non-doubled, raw 20 MHz space):")
        print(f"    n={w_isr.n}  mean={w_isr.mean:+.1f}  stddev={w_isr.stddev:.1f}")
        vals = isr_vals
        print(f"    min={min(vals):+,}  max={max(vals):+,}  range={max(vals)-min(vals):,}")
        print()

        # Value distribution
        from collections import Counter
        dist = Counter(vals)
        print(f"  ISR residual distribution:")
        for val in sorted(dist.keys()):
            bar = '#' * min(dist[val], 60)
            print(f"    {val:>+8d}  {dist[val]:>5d}x  {bar}")
        print()

    # Alternation analysis
    if len(isr_vals) >= 5:
        print(f"  ── Alternation analysis ──")
        diffs = [isr_vals[i+1] - isr_vals[i] for i in range(len(isr_vals)-1)]
        d_mean = sum(diffs) / len(diffs)
        d_var = sum((x - d_mean) ** 2 for x in diffs) / (len(diffs) - 1) if len(diffs) > 1 else 0
        print(f"    Consecutive Δisr: mean={d_mean:+.2f}  stddev={math.sqrt(d_var):.2f}")

        r_mean = sum(isr_vals) / len(isr_vals)
        r_var = sum((x - r_mean) ** 2 for x in isr_vals)
        if r_var > 0:
            print(f"    Autocorrelation:")
            for lag in [1, 2, 3]:
                if lag < len(isr_vals):
                    cov = sum((isr_vals[i] - r_mean) * (isr_vals[i + lag] - r_mean) for i in range(len(isr_vals) - lag))
                    acf = cov / r_var
                    print(f"      lag {lag}: {acf:+.4f}")
        print()


def main():
    if len(sys.argv) < 2:
        print("Usage: gnss_forensics <campaign_name> [limit]")
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