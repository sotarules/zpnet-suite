"""
ZPNet OCXO Servo Characterization Analyzer

Examines the DAC trajectory, rate error, servo activity, and inter-OCXO
correlation over the life of a campaign.  Designed to support Holdover
modeling by answering:

  - What is the shape of the DAC trajectory?  (exponential settling,
    linear drift, staircase, smooth?)
  - How do the two OCXOs track each other?
  - Where is the servo active vs quiescent?
  - What is the rate error the servo is correcting, and how does
    it evolve over time?

The output is organized into time windows (configurable, default 10 min)
so you can see the ebb and flow of servo behavior across the campaign.

Usage:
    python -m zpnet.tests.servo_analyzer <campaign_name> [window_seconds]
    .zt servo_analyzer Shakeout9
    .zt servo_analyzer Shakeout10 300
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

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

    def summary(self, fmt: str = ".6f") -> str:
        if self.n == 0:
            return "no data"
        sd = f"{self.stddev:{fmt}}" if self.n >= 2 else "---"
        return f"n={self.n}  mean={self.mean:{fmt}}  sd={sd}  min={self.min_val:{fmt}}  max={self.max_val:{fmt}}"


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
# Helpers
# ─────────────────────────────────────────────────────────────────────

def safe_float(val: Any) -> Optional[float]:
    if val is None:
        return None
    try:
        return float(val)
    except (ValueError, TypeError):
        return None


def safe_int(val: Any) -> Optional[int]:
    if val is None:
        return None
    try:
        return int(val)
    except (ValueError, TypeError):
        return None


def seconds_to_hms(s: int) -> str:
    h = s // 3600
    m = (s % 3600) // 60
    sec = s % 60
    return f"{h:02d}:{m:02d}:{sec:02d}"


def compute_ppb(ocxo_ns: int, gnss_ns: int) -> Optional[float]:
    if gnss_ns == 0:
        return None
    return ((ocxo_ns - gnss_ns) / gnss_ns) * 1e9


# ─────────────────────────────────────────────────────────────────────
# Per-window accumulator
# ─────────────────────────────────────────────────────────────────────

class WindowAccum:
    def __init__(self, label: str, start_pps: int):
        self.label = label
        self.start_pps = start_pps
        self.end_pps = start_pps
        self.count = 0

        # DAC trajectory
        self.dac1_first: Optional[float] = None
        self.dac1_last: Optional[float] = None
        self.dac2_first: Optional[float] = None
        self.dac2_last: Optional[float] = None
        self.dac1_stats = Welford()
        self.dac2_stats = Welford()

        # PPB (cumulative, from campaign start)
        self.ppb1_first: Optional[float] = None
        self.ppb1_last: Optional[float] = None
        self.ppb2_first: Optional[float] = None
        self.ppb2_last: Optional[float] = None

        # ISR residuals
        self.isr1_stats = Welford()
        self.isr2_stats = Welford()

        # Servo adjustments (cumulative counter)
        self.adj1_first: Optional[int] = None
        self.adj1_last: Optional[int] = None
        self.adj2_first: Optional[int] = None
        self.adj2_last: Optional[int] = None

        # DAC delta tracking (second-to-second changes)
        self.dac1_delta_stats = Welford()
        self.dac2_delta_stats = Welford()

    def add(self, row: Dict[str, Any], prev_dac1: Optional[float], prev_dac2: Optional[float]):
        self.count += 1
        pps = safe_int(row.get("pps_count") or row.get("teensy_pps_count"))
        if pps is not None:
            self.end_pps = pps

        dac1 = safe_float(row.get("ocxo1_dac"))
        dac2 = safe_float(row.get("ocxo2_dac"))

        if dac1 is not None:
            if self.dac1_first is None:
                self.dac1_first = dac1
            self.dac1_last = dac1
            self.dac1_stats.update(dac1)
            if prev_dac1 is not None:
                self.dac1_delta_stats.update(dac1 - prev_dac1)

        if dac2 is not None:
            if self.dac2_first is None:
                self.dac2_first = dac2
            self.dac2_last = dac2
            self.dac2_stats.update(dac2)
            if prev_dac2 is not None:
                self.dac2_delta_stats.update(dac2 - prev_dac2)

        gnss_ns = safe_int(row.get("teensy_gnss_ns"))
        ocxo1_ns = safe_int(row.get("teensy_ocxo1_ns"))
        ocxo2_ns = safe_int(row.get("teensy_ocxo2_ns"))

        if gnss_ns and ocxo1_ns:
            ppb1 = compute_ppb(ocxo1_ns, gnss_ns)
            if ppb1 is not None:
                if self.ppb1_first is None:
                    self.ppb1_first = ppb1
                self.ppb1_last = ppb1

        if gnss_ns and ocxo2_ns:
            ppb2 = compute_ppb(ocxo2_ns, gnss_ns)
            if ppb2 is not None:
                if self.ppb2_first is None:
                    self.ppb2_first = ppb2
                self.ppb2_last = ppb2

        isr1 = safe_int(row.get("isr_residual_ocxo1"))
        isr2 = safe_int(row.get("isr_residual_ocxo2"))
        if isr1 is not None:
            self.isr1_stats.update(float(isr1))
        if isr2 is not None:
            self.isr2_stats.update(float(isr2))

        adj1 = safe_int(row.get("ocxo1_servo_adjustments"))
        adj2 = safe_int(row.get("ocxo2_servo_adjustments"))
        if adj1 is not None:
            if self.adj1_first is None:
                self.adj1_first = adj1
            self.adj1_last = adj1
        if adj2 is not None:
            if self.adj2_first is None:
                self.adj2_first = adj2
            self.adj2_last = adj2


# ─────────────────────────────────────────────────────────────────────
# Main analysis
# ─────────────────────────────────────────────────────────────────────

def analyze(campaign: str, window_seconds: int = 600) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    total = len(rows)

    first_pps = safe_int(rows[0].get("pps_count") or rows[0].get("teensy_pps_count")) or 0
    last_pps = safe_int(rows[-1].get("pps_count") or rows[-1].get("teensy_pps_count")) or 0
    campaign_seconds = last_pps - first_pps
    first_ts = rows[0].get("gnss_time_utc") or rows[0].get("system_time_utc", "?")
    last_ts = rows[-1].get("gnss_time_utc") or rows[-1].get("system_time_utc", "?")

    print("=" * 90)
    print(f"OCXO SERVO CHARACTERIZATION — Campaign: {campaign}")
    print("=" * 90)
    print()
    print(f"  Time range:     {first_ts}")
    print(f"               -> {last_ts}")
    print(f"  Records:        {total}")
    print(f"  Duration:       {seconds_to_hms(campaign_seconds)} ({campaign_seconds:,} seconds)")
    print(f"  Window size:    {window_seconds}s ({seconds_to_hms(window_seconds)})")

    # ── Campaign-wide DAC summary ──

    all_dac1: List[float] = []
    all_dac2: List[float] = []
    all_ppb1: List[Tuple[int, float]] = []
    all_ppb2: List[Tuple[int, float]] = []

    for row in rows:
        pps = safe_int(row.get("pps_count") or row.get("teensy_pps_count")) or 0
        d1 = safe_float(row.get("ocxo1_dac"))
        d2 = safe_float(row.get("ocxo2_dac"))
        if d1 is not None:
            all_dac1.append(d1)
        if d2 is not None:
            all_dac2.append(d2)

        gnss_ns = safe_int(row.get("teensy_gnss_ns"))
        o1_ns = safe_int(row.get("teensy_ocxo1_ns"))
        o2_ns = safe_int(row.get("teensy_ocxo2_ns"))
        if gnss_ns and o1_ns:
            ppb = compute_ppb(o1_ns, gnss_ns)
            if ppb is not None:
                all_ppb1.append((pps, ppb))
        if gnss_ns and o2_ns:
            ppb = compute_ppb(o2_ns, gnss_ns)
            if ppb is not None:
                all_ppb2.append((pps, ppb))

    print()
    print("-" * 90)
    print("CAMPAIGN-WIDE DAC SUMMARY")
    print("-" * 90)

    for label, dac_vals in [("OCXO1", all_dac1), ("OCXO2", all_dac2)]:
        if not dac_vals:
            print(f"\n  [{label}] No DAC data")
            continue

        w = Welford()
        for v in dac_vals:
            w.update(v)

        drift = dac_vals[-1] - dac_vals[0]
        rate = drift / campaign_seconds if campaign_seconds > 0 else 0.0

        print(f"\n  [{label}]")
        print(f"    Start:          {dac_vals[0]:.6f}")
        print(f"    End:            {dac_vals[-1]:.6f}")
        print(f"    Total drift:    {drift:+.6f}")
        print(f"    Drift rate:     {rate:+.9f} DAC/s  ({rate * 3600:+.6f} DAC/hr)")
        print(f"    Mean:           {w.mean:.6f}")
        print(f"    Stddev:         {w.stddev:.6f}")
        print(f"    Range:          {w.min_val:.6f} -> {w.max_val:.6f}  (span={w.max_val - w.min_val:.6f})")

    # ── DAC correlation: do the two OCXOs track each other? ──

    if all_dac1 and all_dac2 and len(all_dac1) == len(all_dac2):
        n = len(all_dac1)
        d1_mean = sum(all_dac1) / n
        d2_mean = sum(all_dac2) / n
        cov = sum((a - d1_mean) * (b - d2_mean) for a, b in zip(all_dac1, all_dac2)) / (n - 1) if n >= 2 else 0.0
        s1 = math.sqrt(sum((a - d1_mean)**2 for a in all_dac1) / (n - 1)) if n >= 2 else 0.0
        s2 = math.sqrt(sum((b - d2_mean)**2 for b in all_dac2) / (n - 1)) if n >= 2 else 0.0
        corr = cov / (s1 * s2) if (s1 > 0 and s2 > 0) else 0.0

        print()
        print(f"  DAC correlation (OCXO1 vs OCXO2):  r = {corr:+.6f}")
        if abs(corr) > 0.9:
            print(f"    → Strong {'positive' if corr > 0 else 'negative'} correlation — tracking common physics")
        elif abs(corr) > 0.5:
            print(f"    → Moderate correlation")
        else:
            print(f"    → Weak correlation — independent drift")

    # ── PPB trajectory: first/last quarter comparison ──

    print()
    print("-" * 90)
    print("PPB TRAJECTORY (cumulative rate error vs GNSS)")
    print("-" * 90)

    for label, ppb_data in [("OCXO1", all_ppb1), ("OCXO2", all_ppb2)]:
        if len(ppb_data) < 4:
            print(f"\n  [{label}] Insufficient data")
            continue

        ppb_vals = [v for _, v in ppb_data]
        q1_end = len(ppb_vals) // 4
        q4_start = 3 * len(ppb_vals) // 4
        q1_mean = sum(ppb_vals[:q1_end]) / q1_end
        q4_mean = sum(ppb_vals[q4_start:]) / (len(ppb_vals) - q4_start)

        print(f"\n  [{label}]")
        print(f"    First PPB:    {ppb_vals[0]:+.3f}")
        print(f"    Last PPB:     {ppb_vals[-1]:+.3f}")
        print(f"    Q1 mean PPB:  {q1_mean:+.3f}")
        print(f"    Q4 mean PPB:  {q4_mean:+.3f}")
        print(f"    Q4 - Q1:      {q4_mean - q1_mean:+.3f} ppb")

    # ── Windowed analysis ──

    print()
    print("-" * 90)
    print(f"WINDOWED SERVO ANALYSIS ({window_seconds}s windows)")
    print("-" * 90)
    print()

    # Header
    print(f"  {'WINDOW':<12s}"
          f"  {'ELAPSED':>8s}"
          f"  {'O1_DAC':>12s}"
          f"  {'O1_DRIFT':>10s}"
          f"  {'O1_ADJ':>7s}"
          f"  {'O1_PPB':>8s}"
          f"  │"
          f"  {'O2_DAC':>12s}"
          f"  {'O2_DRIFT':>10s}"
          f"  {'O2_ADJ':>7s}"
          f"  {'O2_PPB':>8s}")
    print(f"  {'─' * 12}"
          f"  {'─' * 8}"
          f"  {'─' * 12}"
          f"  {'─' * 10}"
          f"  {'─' * 7}"
          f"  {'─' * 8}"
          f"  ─"
          f"  {'─' * 12}"
          f"  {'─' * 10}"
          f"  {'─' * 7}"
          f"  {'─' * 8}")

    windows: List[WindowAccum] = []
    current_window: Optional[WindowAccum] = None
    window_start_pps = first_pps
    window_num = 0
    prev_dac1: Optional[float] = None
    prev_dac2: Optional[float] = None

    for row in rows:
        pps = safe_int(row.get("pps_count") or row.get("teensy_pps_count"))
        if pps is None:
            continue

        while pps >= window_start_pps + window_seconds:
            if current_window is not None:
                windows.append(current_window)
                _print_window(current_window)
            window_num += 1
            window_start_pps += window_seconds
            current_window = None

        if current_window is None:
            current_window = WindowAccum(f"W{window_num}", window_start_pps)

        current_window.add(row, prev_dac1, prev_dac2)

        d1 = safe_float(row.get("ocxo1_dac"))
        d2 = safe_float(row.get("ocxo2_dac"))
        if d1 is not None:
            prev_dac1 = d1
        if d2 is not None:
            prev_dac2 = d2

    if current_window is not None and current_window.count > 0:
        windows.append(current_window)
        _print_window(current_window)

    # ── Second-to-second DAC delta analysis ──

    print()
    print("-" * 90)
    print("DAC STEP ANALYSIS (second-to-second changes)")
    print("-" * 90)

    dac1_deltas = Welford()
    dac2_deltas = Welford()
    prev_d1: Optional[float] = None
    prev_d2: Optional[float] = None

    dac1_nonzero = 0
    dac2_nonzero = 0
    dac1_total = 0
    dac2_total = 0

    for row in rows:
        d1 = safe_float(row.get("ocxo1_dac"))
        d2 = safe_float(row.get("ocxo2_dac"))

        if d1 is not None and prev_d1 is not None:
            delta = d1 - prev_d1
            dac1_deltas.update(delta)
            dac1_total += 1
            if abs(delta) > 1e-12:
                dac1_nonzero += 1

        if d2 is not None and prev_d2 is not None:
            delta = d2 - prev_d2
            dac2_deltas.update(delta)
            dac2_total += 1
            if abs(delta) > 1e-12:
                dac2_nonzero += 1

        if d1 is not None:
            prev_d1 = d1
        if d2 is not None:
            prev_d2 = d2

    for label, stats, nonzero, total_count in [
        ("OCXO1", dac1_deltas, dac1_nonzero, dac1_total),
        ("OCXO2", dac2_deltas, dac2_nonzero, dac2_total),
    ]:
        if stats.n == 0:
            print(f"\n  [{label}] No data")
            continue

        zero_pct = ((total_count - nonzero) / total_count * 100) if total_count > 0 else 0

        print(f"\n  [{label}]")
        print(f"    Total steps:    {total_count}")
        print(f"    Non-zero steps: {nonzero}  ({100 - zero_pct:.1f}%)")
        print(f"    Quiescent:      {total_count - nonzero}  ({zero_pct:.1f}%)")
        print(f"    Step mean:      {stats.mean:+.9f}")
        print(f"    Step stddev:    {stats.stddev:.9f}")
        print(f"    Step min:       {stats.min_val:+.9f}")
        print(f"    Step max:       {stats.max_val:+.9f}")

        if nonzero > 0 and abs(stats.min_val) > 1e-12:
            # Check for quantization
            min_abs = min(abs(stats.min_val), abs(stats.max_val))
            if min_abs > 0.24 and min_abs < 0.26:
                print(f"    ⚠️  Quantized: smallest step ~0.25 (old integer servo math)")
            elif min_abs < 0.01:
                print(f"    ✓  Continuous: sub-0.01 steps present (new rate-error servo)")

    # ── Inter-OCXO PPB divergence ──

    if all_ppb1 and all_ppb2:
        print()
        print("-" * 90)
        print("INTER-OCXO DIVERGENCE (PPB difference over time)")
        print("-" * 90)

        ppb_diff = Welford()
        min_len = min(len(all_ppb1), len(all_ppb2))
        for i in range(min_len):
            diff = all_ppb1[i][1] - all_ppb2[i][1]
            ppb_diff.update(diff)

        print(f"\n  OCXO1 - OCXO2 PPB difference:")
        print(f"    Mean:   {ppb_diff.mean:+.3f} ppb")
        print(f"    Stddev: {ppb_diff.stddev:.3f} ppb")
        print(f"    Min:    {ppb_diff.min_val:+.3f} ppb")
        print(f"    Max:    {ppb_diff.max_val:+.3f} ppb")

    print()
    print("=" * 90)
    print("END OF SERVO CHARACTERIZATION")
    print("=" * 90)


def _print_window(w: WindowAccum) -> None:
    elapsed = seconds_to_hms(w.end_pps - w.start_pps + 1) if w.end_pps >= w.start_pps else "---"

    d1_dac = f"{w.dac1_last:.3f}" if w.dac1_last is not None else "---"
    d2_dac = f"{w.dac2_last:.3f}" if w.dac2_last is not None else "---"

    d1_drift = f"{w.dac1_last - w.dac1_first:+.6f}" if (w.dac1_first is not None and w.dac1_last is not None) else "---"
    d2_drift = f"{w.dac2_last - w.dac2_first:+.6f}" if (w.dac2_first is not None and w.dac2_last is not None) else "---"

    d1_adj = f"{w.adj1_last - w.adj1_first}" if (w.adj1_first is not None and w.adj1_last is not None) else "---"
    d2_adj = f"{w.adj2_last - w.adj2_first}" if (w.adj2_first is not None and w.adj2_last is not None) else "---"

    d1_ppb = f"{w.ppb1_last:+.3f}" if w.ppb1_last is not None else "---"
    d2_ppb = f"{w.ppb2_last:+.3f}" if w.ppb2_last is not None else "---"

    print(f"  {w.label:<12s}"
          f"  {elapsed:>8s}"
          f"  {d1_dac:>12s}"
          f"  {d1_drift:>10s}"
          f"  {d1_adj:>7s}"
          f"  {d1_ppb:>8s}"
          f"  │"
          f"  {d2_dac:>12s}"
          f"  {d2_drift:>10s}"
          f"  {d2_adj:>7s}"
          f"  {d2_ppb:>8s}")


# ─────────────────────────────────────────────────────────────────────
# Available campaigns listing
# ─────────────────────────────────────────────────────────────────────

def list_campaigns() -> None:
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT campaign, count(*) as cnt,
                       min((payload->>'pps_count')::int) as pps_min,
                       max((payload->>'pps_count')::int) as pps_max
                FROM timebase
                GROUP BY campaign
                ORDER BY max(ts) DESC
                LIMIT 15
                """
            )
            rows = cur.fetchall()
        if rows:
            print("Available campaigns:")
            print(f"  {'CAMPAIGN':<24s} {'RECORDS':>8s} {'DURATION':>10s}")
            print(f"  {'─' * 24} {'─' * 8} {'─' * 10}")
            for r in rows:
                dur = (r["pps_max"] or 0) - (r["pps_min"] or 0)
                print(f"  {r['campaign']:<24s} {r['cnt']:>8d} {seconds_to_hms(dur):>10s}")
    except Exception:
        pass


def main():
    if len(sys.argv) < 2:
        print("Usage: servo_analyzer <campaign_name> [window_seconds]")
        print()
        list_campaigns()
        sys.exit(1)

    campaign = sys.argv[1]
    window = int(sys.argv[2]) if len(sys.argv) > 2 else 600
    analyze(campaign, window)


if __name__ == "__main__":
    main()