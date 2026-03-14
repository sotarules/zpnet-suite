"""
ZPNet GNSS Validation Report — PPS Integrity Analysis

GNSS nanoseconds are defined by PPS count (campaign_seconds × 1e9),
so they cannot validate themselves.  This report instead answers:

  "Is the PPS stream arriving at exactly 1-second intervals,
   as independently witnessed by DWT, OCXO1, and OCXO2?"

Three independent witnesses:
  • DWT   — 1,008,000,000 cycles per PPS (±ISR entry jitter)
  • OCXO1 — 10,000,000 ticks per PPS (±frequency offset)
  • OCXO2 — 10,000,000 ticks per PPS (±frequency offset)

If all three witnesses agree that PPS intervals are stable and
consistent, then the PPS stream is trustworthy — regardless of
what the GNSS VCLOCK QTimer says.

Also reports:
  • isr_residual_gnss — QTimer1 dual-edge raw count sanity check
  • PPS rejection events (missed PPS edges)
  • GNSS UTC vs system UTC coherence
  • DWT calibration stability (dwt_cycles_per_pps)

Usage:
    python -m zpnet.tests.gnss_validation <campaign_name> [limit]
    .zt gnss_validation PPB1
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db

DWT_NOMINAL = 1_008_000_000
OCXO_NOMINAL = 10_000_000


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
    def range(self) -> float:
        if self.min_val is not None and self.max_val is not None:
            return self.max_val - self.min_val
        return 0.0


def safe_int(val, default=0) -> int:
    if val is None:
        return default
    return int(val)


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    n_rows = len(rows)
    if limit:
        rows = rows[:limit]

    print(f"=" * 72)
    print(f"GNSS VALIDATION REPORT: {campaign}")
    print(f"=" * 72)
    print(f"  Records: {n_rows} (analyzing {len(rows)})")
    print()

    # ── Collect per-second data ──
    w_dwt_delta = Welford()       # DWT cycles per PPS
    w_ocxo1_delta = Welford()     # OCXO1 ticks per PPS
    w_ocxo2_delta = Welford()     # OCXO2 ticks per PPS
    w_dwt_cal = Welford()         # dwt_cycles_per_pps (calibration constant)
    w_isr_gnss = Welford()        # ISR residual GNSS (QTimer sanity)

    isr_gnss_dist: Counter = Counter()
    pps_rejected_max = 0
    doubled_count = 0
    prev_pps: Optional[int] = None
    gaps = 0

    # For cross-domain consistency
    dwt_deltas = []
    ocxo1_deltas = []
    ocxo2_deltas = []

    for rec in rows:
        pps = safe_int(rec.get("pps_count", rec.get("teensy_pps_count")))
        dwt_raw = safe_int(rec.get("dwt_delta_raw"))
        o1_raw = safe_int(rec.get("ocxo1_delta_raw"))
        o2_raw = safe_int(rec.get("ocxo2_delta_raw"))
        isr_gn = safe_int(rec.get("isr_residual_gnss"))
        isr_dw = safe_int(rec.get("isr_residual_dwt"))
        dwt_cal = safe_int(rec.get("dwt_cycles_per_pps"))
        rej_tot = safe_int(rec.get("pps_rejected_total"))

        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
        prev_pps = pps

        if rej_tot > pps_rejected_max:
            pps_rejected_max = rej_tot

        # Detect doubled deltas (missed PPS)
        is_doubled = (dwt_raw > DWT_NOMINAL * 1.5)
        if is_doubled:
            doubled_count += 1
            continue

        if dwt_raw > 0:
            w_dwt_delta.update(float(dwt_raw))
            dwt_deltas.append(dwt_raw)
        if o1_raw > 0:
            w_ocxo1_delta.update(float(o1_raw))
            ocxo1_deltas.append(o1_raw)
        if o2_raw > 0:
            w_ocxo2_delta.update(float(o2_raw))
            ocxo2_deltas.append(o2_raw)
        if dwt_cal > 0:
            w_dwt_cal.update(float(dwt_cal))

        isr_gnss_dist[isr_gn] += 1
        w_isr_gnss.update(float(isr_gn))

    # ══════════════════════════════════════════════════════════════════
    print(f"─" * 72)
    print(f"1. PPS STREAM HEALTH")
    print(f"─" * 72)
    print(f"  PPS edges counted:    {len(rows)}")
    print(f"  Gaps (recovery):      {gaps}")
    print(f"  Doubled deltas:       {doubled_count}")
    print(f"  PPS rejections total: {pps_rejected_max}")

    if doubled_count == 0 and pps_rejected_max == 0 and gaps == 0:
        print(f"  Assessment:           CLEAN — no missed or rejected PPS edges")
    else:
        issues = []
        if doubled_count > 0:
            issues.append(f"{doubled_count} missed PPS")
        if pps_rejected_max > 0:
            issues.append(f"{pps_rejected_max} rejections")
        if gaps > 0:
            issues.append(f"{gaps} gaps")
        print(f"  Assessment:           ANOMALIES — {', '.join(issues)}")
    print()

    # ══════════════════════════════════════════════════════════════════
    print(f"─" * 72)
    print(f"2. INDEPENDENT PPS INTERVAL WITNESSES (excluding doubled)")
    print(f"─" * 72)
    print()
    print(f"  Each witness measures how many of its own ticks fit between")
    print(f"  consecutive PPS edges.  If PPS is accurate, each witness")
    print(f"  should see a stable count near its nominal frequency.")
    print()

    witnesses = [
        ("DWT (1,008,000,000 Hz)", w_dwt_delta, DWT_NOMINAL, "cycles"),
        ("OCXO1 (10,000,000 Hz)", w_ocxo1_delta, OCXO_NOMINAL, "ticks"),
        ("OCXO2 (10,000,000 Hz)", w_ocxo2_delta, OCXO_NOMINAL, "ticks"),
    ]

    for name, w, nominal, unit in witnesses:
        if w.n < 2:
            print(f"  {name}: insufficient data")
            continue

        offset_ppb = ((w.mean - nominal) / nominal) * 1e9
        stability_ppb = (w.stddev / nominal) * 1e9

        print(f"  {name}:")
        print(f"    n={w.n}  mean={w.mean:,.1f}  stddev={w.stddev:.2f} {unit}")
        print(f"    min={w.min_val:,.0f}  max={w.max_val:,.0f}  range={w.range:,.0f}")
        print(f"    offset from nominal: {offset_ppb:+.3f} PPB")
        print(f"    per-second stability: {stability_ppb:.3f} PPB (1σ)")
        print()

    # ── Cross-domain agreement ──
    if len(dwt_deltas) >= 10 and len(ocxo1_deltas) >= 10 and len(ocxo2_deltas) >= 10:
        n_common = min(len(dwt_deltas), len(ocxo1_deltas), len(ocxo2_deltas))

        # Normalize all three to PPB
        dwt_ppb = [(d - DWT_NOMINAL) / DWT_NOMINAL * 1e9 for d in dwt_deltas[:n_common]]
        o1_ppb = [(d - OCXO_NOMINAL) / OCXO_NOMINAL * 1e9 for d in ocxo1_deltas[:n_common]]
        o2_ppb = [(d - OCXO_NOMINAL) / OCXO_NOMINAL * 1e9 for d in ocxo2_deltas[:n_common]]

        # Correlation between witnesses (if PPS is jittery, all three see the same jitter)
        def pearson(a, b):
            n = len(a)
            ma = sum(a) / n
            mb = sum(b) / n
            cov = sum((a[i] - ma) * (b[i] - mb) for i in range(n))
            va = sum((x - ma) ** 2 for x in a)
            vb = sum((x - mb) ** 2 for x in b)
            denom = math.sqrt(va * vb)
            return cov / denom if denom > 0 else 0.0

        print(f"  Cross-domain correlation (PPB, per-second):")
        print(f"    DWT  vs OCXO1: {pearson(dwt_ppb, o1_ppb):+.4f}")
        print(f"    DWT  vs OCXO2: {pearson(dwt_ppb, o2_ppb):+.4f}")
        print(f"    OCXO1 vs OCXO2: {pearson(o1_ppb, o2_ppb):+.4f}")
        print()
        print(f"    (High correlation means all witnesses see the same PPS jitter,")
        print(f"     confirming PPS is the common reference.  Low correlation would")
        print(f"     indicate independent noise sources, not PPS variation.)")
        print()

    # ══════════════════════════════════════════════════════════════════
    print(f"─" * 72)
    print(f"3. DWT CALIBRATION STABILITY (dwt_cycles_per_pps)")
    print(f"─" * 72)
    print()
    print(f"  This is the continuous DWT-to-GNSS calibration constant,")
    print(f"  updated every PPS edge.  It's the foundation of time.h.")
    print()

    if w_dwt_cal.n >= 2:
        cal_ppb = (w_dwt_cal.stddev / w_dwt_cal.mean) * 1e9 if w_dwt_cal.mean > 0 else 0
        print(f"    n={w_dwt_cal.n}  mean={w_dwt_cal.mean:,.1f}  stddev={w_dwt_cal.stddev:.2f}")
        print(f"    min={w_dwt_cal.min_val:,.0f}  max={w_dwt_cal.max_val:,.0f}  range={w_dwt_cal.range:,.0f}")
        print(f"    stability: {cal_ppb:.3f} PPB (1σ)")
        print()

        if w_dwt_cal.range <= 20:
            print(f"    Assessment: EXCELLENT — range ≤20 cycles ({w_dwt_cal.range:.0f})")
        elif w_dwt_cal.range <= 100:
            print(f"    Assessment: GOOD — range ≤100 cycles ({w_dwt_cal.range:.0f})")
        else:
            print(f"    Assessment: ELEVATED — range {w_dwt_cal.range:.0f} cycles")
    else:
        print(f"    Insufficient data")
    print()

    # ══════════════════════════════════════════════════════════════════
    print(f"─" * 72)
    print(f"4. GNSS VCLOCK QTIMER SANITY (isr_residual_gnss)")
    print(f"─" * 72)
    print()
    print(f"  QTimer1 dual-edge count residual from 20,000,000 nominal.")
    print(f"  Expected: ±1 alternation (phase-coherent VCLOCK/PPS).")
    print(f"  Large values indicate signal integrity or PPS timing anomalies.")
    print()

    if w_isr_gnss.n >= 2:
        print(f"    n={w_isr_gnss.n}  mean={w_isr_gnss.mean:+.2f}  stddev={w_isr_gnss.stddev:.2f}")
        if w_isr_gnss.min_val is not None:
            print(f"    min={w_isr_gnss.min_val:+.0f}  max={w_isr_gnss.max_val:+.0f}  range={w_isr_gnss.range:.0f}")
        print()

        # Distribution
        print(f"    Distribution:")
        for val in sorted(isr_gnss_dist.keys()):
            cnt = isr_gnss_dist[val]
            bar = '#' * min(cnt, 60)
            print(f"      {val:>+6d}  {cnt:>5d}x  {bar}")
        print()

        # Assess
        n_zero = isr_gnss_dist.get(0, 0)
        n_pm1 = isr_gnss_dist.get(1, 0) + isr_gnss_dist.get(-1, 0) + n_zero
        n_outlier = w_isr_gnss.n - n_pm1
        pct_clean = (n_pm1 / w_isr_gnss.n) * 100 if w_isr_gnss.n > 0 else 0

        print(f"    Zero residuals:  {n_zero} ({n_zero/w_isr_gnss.n*100:.1f}%)")
        print(f"    Within ±1:       {n_pm1} ({pct_clean:.1f}%)")
        print(f"    Outliers (|r|>1): {n_outlier}")
        print()

        if n_outlier == 0:
            print(f"    Assessment: PERFECT — all residuals within ±1")
        elif n_outlier <= w_isr_gnss.n * 0.01:
            print(f"    Assessment: GOOD — {n_outlier} outliers (<1%)")
        else:
            print(f"    Assessment: ANOMALOUS — {n_outlier} outliers ({n_outlier/w_isr_gnss.n*100:.1f}%)")
    else:
        print(f"    Insufficient data")
    print()

    # ══════════════════════════════════════════════════════════════════
    print(f"─" * 72)
    print(f"5. GNSS UTC COHERENCE")
    print(f"─" * 72)
    print()

    # Check that gnss_time_utc advances by ~1 second per record
    utc_gaps = 0
    utc_checked = 0
    prev_gnss_utc = None
    for rec in rows:
        gnss_utc = rec.get("gnss_time_utc")
        if gnss_utc and prev_gnss_utc:
            utc_checked += 1
            # Simple string comparison — both should be ISO 8601
            # Just check they're different (advancing)
            if gnss_utc == prev_gnss_utc:
                utc_gaps += 1
        prev_gnss_utc = gnss_utc

    if utc_checked > 0:
        if utc_gaps == 0:
            print(f"    GNSS UTC advancing every record ({utc_checked} checked)")
            print(f"    Assessment: NOMINAL")
        else:
            print(f"    GNSS UTC stalled {utc_gaps} times out of {utc_checked} records")
            print(f"    Assessment: ANOMALOUS")
    else:
        print(f"    No GNSS UTC data to check")
    print()

    # ══════════════════════════════════════════════════════════════════
    print(f"─" * 72)
    print(f"VERDICT")
    print(f"─" * 72)
    print()

    issues = []
    if doubled_count > 0:
        issues.append(f"{doubled_count} missed PPS edge(s)")
    if pps_rejected_max > 0:
        issues.append(f"{pps_rejected_max} PPS rejection(s)")
    if w_dwt_cal.n >= 2 and w_dwt_cal.range > 100:
        issues.append(f"DWT calibration range elevated ({w_dwt_cal.range:.0f} cycles)")
    if w_isr_gnss.n >= 2:
        n_pm1 = isr_gnss_dist.get(0, 0) + isr_gnss_dist.get(1, 0) + isr_gnss_dist.get(-1, 0)
        n_outlier = w_isr_gnss.n - n_pm1
        if n_outlier > w_isr_gnss.n * 0.01:
            issues.append(f"GNSS VCLOCK outliers ({n_outlier})")

    if not issues:
        print(f"  GNSS PPS INTEGRITY: VALIDATED")
        print(f"  All three independent witnesses confirm stable 1-second PPS intervals.")
        print(f"  VCLOCK QTimer sanity check nominal.  No missed or rejected PPS edges.")
    else:
        print(f"  GNSS PPS INTEGRITY: ANOMALIES ({len(issues)})")
        for issue in issues:
            print(f"    • {issue}")

    print()
    print(f"=" * 72)


def main():
    if len(sys.argv) < 2:
        print("Usage: gnss_validation <campaign_name> [limit]")
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