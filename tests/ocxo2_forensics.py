"""
ZPNet OCXO2 Forensics — Raw Capture Analysis

Per-second table showing every observable quantity for OCXO2:

  • ocxo2_delta_raw     — raw 20 MHz QTimer1 counts between PPS edges
  • ocxo2_delta_10mhz   — delta_raw / 2 (derived 10 MHz ticks)
  • residual_raw        — delta_raw - 20,000,000
  • residual_10mhz      — delta_10mhz - 10,000,000
  • isr_residual_ocxo2  — ISR-level residual (from TIMEBASE_FRAGMENT)
  • Δraw                — first difference of delta_raw (jitter)
  • pred / pred_err     — linear extrapolation prediction
  • ocxo1_delta_raw     — OCXO1 for comparison (should be ~10,000,000)
  • isr_residual_ocxo1  — OCXO1 ISR residual for comparison

Also shows: teensy_pps_count, isr_residual_gnss, isr_residual_dwt

Usage:
    python -m zpnet.tests.ocxo2_forensics <campaign_name> [limit]
    .zt ocxo2_forensics Rewire1
    .zt ocxo2_forensics Rewire1 20
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db

OCXO_RAW_EXPECTED = 20_000_000   # QTimer CM(2) dual-edge
OCXO_10MHZ_EXPECTED = 10_000_000
OCXO1_EXPECTED = 10_000_000      # GPT1 single-edge


def fetch_timebase_fragments(campaign: str) -> List[Dict[str, Any]]:
    """Fetch TIMEBASE_FRAGMENT payloads from the timebase table."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'teensy_pps_count')::int ASC
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

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


def safe_int(val, default=0) -> int:
    if val is None:
        return default
    return int(val)


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase_fragments(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()

    # ── Header ──
    hdr = (
        f"  {'pps':>5s}"
        f"  {'ocxo2_raw':>12s}"
        f"  {'res_raw':>9s}"
        f"  {'Δraw':>8s}"
        f"  {'res_10m':>9s}"
        f"  {'isr_o2':>8s}"
        f"  {'pred':>12s}"
        f"  {'p_err':>8s}"
        f"  {'o1_delta':>12s}"
        f"  {'isr_o1':>8s}"
        f"  {'isr_gn':>7s}"
        f"  {'isr_dw':>7s}"
        f"  {'raw_sd':>8s}"
        f"  {'p_sd':>8s}"
    )
    sep = (
        f"  {'─'*5}"
        f"  {'─'*12}"
        f"  {'─'*9}"
        f"  {'─'*8}"
        f"  {'─'*9}"
        f"  {'─'*8}"
        f"  {'─'*12}"
        f"  {'─'*8}"
        f"  {'─'*12}"
        f"  {'─'*8}"
        f"  {'─'*7}"
        f"  {'─'*7}"
        f"  {'─'*8}"
        f"  {'─'*8}"
    )
    print(hdr)
    print(sep)

    # ── Accumulators ──
    w_raw = Welford()         # residual from 20M nominal
    w_10m = Welford()         # residual from 10M nominal
    w_pred = Welford()        # prediction residual

    prev_raw: Optional[int] = None
    prev_prev_raw: Optional[int] = None
    prev_pps: Optional[int] = None

    # Track min/max for summary
    raw_min = None
    raw_max = None
    isr_o2_min = None
    isr_o2_max = None

    count = 0
    gaps = 0

    for rec in rows:
        pps = safe_int(rec.get("teensy_pps_count"))
        o2_raw = safe_int(rec.get("ocxo2_delta_raw"))
        o1_raw = safe_int(rec.get("ocxo1_delta_raw"))
        isr_o2 = safe_int(rec.get("isr_residual_ocxo2"))
        isr_o1 = safe_int(rec.get("isr_residual_ocxo1"))
        isr_gn = safe_int(rec.get("isr_residual_gnss"))
        isr_dw = safe_int(rec.get("isr_residual_dwt"))

        # Recovery gap detection
        if prev_pps is not None and pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  --- recovery gap {prev_pps} → {pps} ({skipped}s lost) ---")
            prev_raw = None
            prev_prev_raw = None

        prev_pps = pps

        if o2_raw == 0:
            # No data yet (first record or invalid)
            continue

        res_raw = o2_raw - OCXO_RAW_EXPECTED
        o2_10m = o2_raw // 2
        res_10m = o2_10m - OCXO_10MHZ_EXPECTED

        # Track extremes
        if raw_min is None or res_raw < raw_min:
            raw_min = res_raw
        if raw_max is None or res_raw > raw_max:
            raw_max = res_raw
        if isr_o2_min is None or isr_o2 < isr_o2_min:
            isr_o2_min = isr_o2
        if isr_o2_max is None or isr_o2 > isr_o2_max:
            isr_o2_max = isr_o2

        # First difference of raw delta
        if prev_raw is not None:
            d_raw = o2_raw - prev_raw
            d_raw_str = f"{d_raw:>+8d}"
        else:
            d_raw_str = f"{'---':>8s}"

        # Prediction
        if prev_raw is not None and prev_prev_raw is not None:
            predicted = 2 * prev_raw - prev_prev_raw
            pred_err = o2_raw - predicted
            pred_str = f"{predicted:>12,}"
            perr_str = f"{pred_err:>+8d}"
            w_pred.update(float(pred_err))
        else:
            pred_str = f"{'---':>12s}"
            perr_str = f"{'---':>8s}"

        w_raw.update(float(res_raw))
        w_10m.update(float(res_10m))

        raw_sd_str = f"{w_raw.stddev:>8.1f}" if w_raw.n >= 2 else f"{'---':>8s}"
        p_sd_str = f"{w_pred.stddev:>8.1f}" if w_pred.n >= 2 else f"{'---':>8s}"

        print(
            f"  {pps:>5d}"
            f"  {o2_raw:>12,}"
            f"  {res_raw:>+9,}"
            f"  {d_raw_str}"
            f"  {res_10m:>+9,}"
            f"  {isr_o2:>+8d}"
            f"  {pred_str}"
            f"  {perr_str}"
            f"  {o1_raw:>12,}"
            f"  {isr_o1:>+8d}"
            f"  {isr_gn:>+7d}"
            f"  {isr_dw:>+7d}"
            f"  {raw_sd_str}"
            f"  {p_sd_str}"
        )

        prev_prev_raw = prev_raw
        prev_raw = o2_raw

        count += 1
        if limit and count >= limit:
            break

    # ── Summary ──
    print()
    print(f"  {count} rows, {gaps} recovery gap(s)")
    print()

    if w_raw.n >= 2:
        print(f"  OCXO2 raw delta (from {OCXO_RAW_EXPECTED:,} nominal):")
        print(f"    n={w_raw.n}  mean={w_raw.mean:+.1f}  stddev={w_raw.stddev:.1f}  stderr={w_raw.stderr:.2f}")
        if raw_min is not None:
            print(f"    min={raw_min:+,}  max={raw_max:+,}  range={raw_max - raw_min:,}")
        print()

    if w_10m.n >= 2:
        print(f"  OCXO2 derived 10 MHz (from {OCXO_10MHZ_EXPECTED:,} nominal):")
        print(f"    n={w_10m.n}  mean={w_10m.mean:+.1f}  stddev={w_10m.stddev:.1f}  stderr={w_10m.stderr:.2f}")
        print()

    if w_pred.n >= 2:
        print(f"  OCXO2 prediction (linear extrapolation on raw):")
        print(f"    n={w_pred.n}  mean={w_pred.mean:+.1f}  stddev={w_pred.stddev:.1f}  stderr={w_pred.stderr:.2f}")
        if w_raw.stddev > 0:
            improvement = w_raw.stddev / w_pred.stddev
            print(f"    improvement: {improvement:.1f}x tighter than raw stddev")
        print()

    if isr_o2_min is not None:
        print(f"  ISR residual OCXO2:")
        print(f"    min={isr_o2_min:+,}  max={isr_o2_max:+,}  range={isr_o2_max - isr_o2_min:,}")
        print()

    # ── Correlation check: is raw residual correlated with ISR residual? ──
    # Re-scan to compute correlation
    if w_raw.n >= 3:
        print(f"  ── Pattern analysis ──")
        raws = []
        isrs = []
        for rec in rows:
            o2_raw = safe_int(rec.get("ocxo2_delta_raw"))
            isr_o2 = safe_int(rec.get("isr_residual_ocxo2"))
            if o2_raw > 0:
                raws.append(o2_raw - OCXO_RAW_EXPECTED)
                isrs.append(isr_o2)
                if limit and len(raws) >= limit:
                    break

        if len(raws) >= 3:
            # Check: does raw == 2 * isr?  (If ISR computes correctly)
            diffs = [r - 2 * i for r, i in zip(raws, isrs)]
            d_mean = sum(diffs) / len(diffs)
            d_var = sum((x - d_mean) ** 2 for x in diffs) / (len(diffs) - 1) if len(diffs) > 1 else 0
            d_sd = math.sqrt(d_var)
            print(f"    raw_residual - 2*isr_residual: mean={d_mean:+.1f}  stddev={d_sd:.1f}")
            print(f"    (If 0 ± small, ISR and accumulator agree perfectly)")
            print()

            # Check: correlation between consecutive raw values
            if len(raws) >= 4:
                diffs_consec = [raws[i+1] - raws[i] for i in range(len(raws)-1)]
                dc_mean = sum(diffs_consec) / len(diffs_consec)
                dc_var = sum((x - dc_mean) ** 2 for x in diffs_consec) / (len(diffs_consec) - 1) if len(diffs_consec) > 1 else 0
                dc_sd = math.sqrt(dc_var)
                print(f"    Consecutive Δraw: mean={dc_mean:+.1f}  stddev={dc_sd:.1f}")
                print()

            # Check: is there a periodic pattern?
            if len(raws) >= 10:
                # Autocorrelation at lag 1, 2, 3
                r_mean = sum(raws) / len(raws)
                r_var = sum((x - r_mean) ** 2 for x in raws)
                if r_var > 0:
                    print(f"    Autocorrelation:")
                    for lag in [1, 2, 3, 4, 5]:
                        if lag < len(raws):
                            cov = sum((raws[i] - r_mean) * (raws[i + lag] - r_mean) for i in range(len(raws) - lag))
                            acf = cov / r_var
                            print(f"      lag {lag}: {acf:+.4f}")
                    print()


def main():
    if len(sys.argv) < 2:
        print("Usage: ocxo2_forensics <campaign_name> [limit]")
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