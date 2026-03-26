
from __future__ import annotations

"""
ZPNet Edge TDC Analyzer — OCXO post-PPS edge bracket analyzer

Analyzes the new OCXO edge bracket instrumentation captured in TIMEBASE
to understand whether the polling loop brackets a unique 10 MHz edge.

Key fields from TIMEBASE rows:

  ocxo1_dwt_before / ocxo1_dwt_after
  ocxo1_gpt_before / ocxo1_gpt_after
  ocxo1_dwt_bracket_cycles
  ocxo1_edge_elapsed_ns
  ocxo1_phase_offset_ns
  ocxo1_residual_ns

  ocxo2_dwt_before / ocxo2_dwt_after
  ocxo2_gpt_before / ocxo2_gpt_after
  ocxo2_dwt_bracket_cycles
  ocxo2_edge_elapsed_ns
  ocxo2_phase_offset_ns
  ocxo2_residual_ns

This tool:
  1. Extracts per-OCXO bracket widths in DWT cycles / ns
  2. Computes GPT deltas across the successful bracket
  3. Builds bracket-width histograms
  4. Reports how often the bracket spans >1 OCXO edge
  5. Shows second-to-second stability of the observed elapsed time
  6. Prints per-second detail rows for visual sanity checking

Usage:
    python -m zpnet.tests.edge_tdc_analyzer <campaign_name> [limit]
    .zt edge_tdc_analyzer fix3
    .zt edge_tdc_analyzer fix3 200
"""

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db

DWT_FREQ_HZ = 1_008_000_000
DWT_NS_PER_CYCLE = 125.0 / 126.0
OCXO_PERIOD_NS = 100.0


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


def find_clusters(values: List[int], min_count: int = 3) -> List[Tuple[int, int]]:
    counter = Counter(values)
    clusters = [(val, cnt) for val, cnt in counter.items() if cnt >= min_count]
    clusters.sort(key=lambda x: x[0])
    return clusters


def _histogram(values: List[int], title: str) -> None:
    print("-" * 70)
    print(title)
    print("-" * 70)
    print()

    counter = Counter(values)
    all_vals = sorted(counter.keys())
    max_count = max(counter.values()) if counter else 1
    bar_width = 50

    last_val = None
    for val in all_vals:
        if last_val is not None and val - last_val > 3:
            print(f"       │ {'':.<{bar_width}s}  ···")
        cnt = counter[val]
        bar_len = int((cnt / max_count) * bar_width)
        bar = "█" * bar_len
        pct = (cnt / len(values)) * 100
        ns_val = val * DWT_NS_PER_CYCLE
        print(f"  {val:4d} │ {bar:<{bar_width}s} {cnt:5d} ({pct:5.1f}%)  [{ns_val:6.1f} ns]")
        last_val = val
    print()


def _analyze_ocxo(rows: List[Dict[str, Any]], key: str, limit: int = 0) -> None:
    upper = key.upper()

    brackets: List[int] = []
    gpt_deltas: List[int] = []
    elapsed_values: List[int] = []
    residuals: List[int] = []
    phases: List[int] = []
    pps_values: List[int] = []

    skipped_missing = 0

    for r in rows:
        if limit and len(brackets) >= limit:
            break

        dwt_before = r.get(f"{key}_dwt_before")
        dwt_after = r.get(f"{key}_dwt_after")
        gpt_before = r.get(f"{key}_gpt_before")
        gpt_after = r.get(f"{key}_gpt_after")
        bracket = r.get(f"{key}_dwt_bracket_cycles")
        elapsed = r.get(f"{key}_edge_elapsed_ns")
        phase = r.get(f"{key}_phase_offset_ns")
        residual = r.get(f"{key}_residual_ns")
        pps = r.get("pps_count", r.get("teensy_pps_count"))

        if None in (dwt_before, dwt_after, gpt_before, gpt_after, bracket, elapsed, phase, residual, pps):
            skipped_missing += 1
            continue

        brackets.append(int(bracket))
        gpt_deltas.append(int(gpt_after) - int(gpt_before))
        elapsed_values.append(int(elapsed))
        phases.append(int(phase))
        residuals.append(int(residual))
        pps_values.append(int(pps))

    print("=" * 70)
    print(f"EDGE TDC ANALYZER — {upper}")
    print("=" * 70)
    print()
    print(f"  Samples:               {len(brackets)}")
    print(f"  Skipped missing:       {skipped_missing}")
    print()

    if len(brackets) < 3:
        print("  Insufficient data")
        print()
        return

    w_bracket = Welford()
    for b in brackets:
        w_bracket.update(float(b))

    mean_ns = w_bracket.mean * DWT_NS_PER_CYCLE
    std_ns = w_bracket.stddev * DWT_NS_PER_CYCLE
    min_br = int(w_bracket.min_val)
    max_br = int(w_bracket.max_val)

    print("-" * 70)
    print("BRACKET WIDTH")
    print("-" * 70)
    print()
    print(f"  n       = {w_bracket.n:,}")
    print(f"  mean    = {w_bracket.mean:,.2f} cycles  ({mean_ns:,.1f} ns)")
    print(f"  stddev  = {w_bracket.stddev:,.2f} cycles  ({std_ns:,.1f} ns)")
    print(f"  min     = {min_br:,} cycles  ({min_br * DWT_NS_PER_CYCLE:.1f} ns)")
    print(f"  max     = {max_br:,} cycles  ({max_br * DWT_NS_PER_CYCLE:.1f} ns)")
    print(f"  range   = {int(w_bracket.range):,} cycles  ({w_bracket.range * DWT_NS_PER_CYCLE:,.1f} ns)")
    print()

    over_100ns = sum(1 for b in brackets if b * DWT_NS_PER_CYCLE >= OCXO_PERIOD_NS)
    over_80ns = sum(1 for b in brackets if b * DWT_NS_PER_CYCLE >= 80.0)
    print(f"  Brackets >= 80 ns:     {over_80ns}/{len(brackets)} ({100 * over_80ns / len(brackets):.1f}%)")
    print(f"  Brackets >= 100 ns:    {over_100ns}/{len(brackets)} ({100 * over_100ns / len(brackets):.1f}%)")
    print()

    if over_100ns:
        print("  Assessment: bracket sometimes spans a full 10 MHz period or more")
    elif over_80ns:
        print("  Assessment: bracket is often uncomfortably wide vs 100 ns OCXO period")
    else:
        print("  Assessment: bracket is comfortably below one 10 MHz period")
    print()

    print("-" * 70)
    print("GPT DELTA ACROSS BRACKET")
    print("-" * 70)
    print()

    gpt_counter = Counter(gpt_deltas)
    multi_edge = sum(1 for d in gpt_deltas if d > 1)
    zero_edge = sum(1 for d in gpt_deltas if d <= 0)
    for delta in sorted(gpt_counter.keys()):
        cnt = gpt_counter[delta]
        pct = 100.0 * cnt / len(gpt_deltas)
        print(f"  delta {delta:>2d}: {cnt:>5d} ({pct:5.1f}%)")
    print()
    print(f"  GPT delta > 1:         {multi_edge}/{len(gpt_deltas)} ({100 * multi_edge / len(gpt_deltas):.1f}%)")
    print(f"  GPT delta <= 0:        {zero_edge}/{len(gpt_deltas)} ({100 * zero_edge / len(gpt_deltas):.1f}%)")
    print()

    w_elapsed = Welford()
    for e in elapsed_values:
        w_elapsed.update(float(e))

    w_resid = Welford()
    for r in residuals:
        w_resid.update(float(r))

    print("-" * 70)
    print("ELAPSED / RESIDUAL STABILITY")
    print("-" * 70)
    print()
    print(f"  edge_elapsed_ns mean   = {w_elapsed.mean:,.2f} ns")
    print(f"  edge_elapsed_ns stddev = {w_elapsed.stddev:,.2f} ns")
    print(f"  edge_elapsed_ns range  = {int(w_elapsed.range):,} ns")
    print()
    print(f"  residual mean          = {w_resid.mean:+.2f} ns")
    print(f"  residual stddev        = {w_resid.stddev:.2f} ns")
    print(f"  residual min/max       = {int(w_resid.min_val):+d} / {int(w_resid.max_val):+d} ns")
    print()

    if len(elapsed_values) >= 2:
        w_diff = Welford()
        for i in range(1, len(elapsed_values)):
            w_diff.update(float(elapsed_values[i] - elapsed_values[i - 1]))
        print(f"  elapsed second-delta σ = {w_diff.stddev:.2f} ns")
        print(f"  elapsed second-delta   = {int(w_diff.min_val):+d} .. {int(w_diff.max_val):+d} ns")
        print()

    _histogram(brackets, f"{upper} BRACKET HISTOGRAM (cycles)")

    # Optional cluster analysis on bracket widths
    min_hits = max(2, len(brackets) // 50)
    clusters = find_clusters(brackets, min_count=min_hits)
    print("-" * 70)
    print("BRACKET CLUSTERS")
    print("-" * 70)
    print()
    if clusters:
        for val, cnt in clusters:
            pct = 100.0 * cnt / len(brackets)
            print(f"  {val:>4d} cycles ({val * DWT_NS_PER_CYCLE:6.1f} ns): {cnt:>5d} ({pct:5.1f}%)")
    else:
        print("  No stable bracket-width clusters found")
    print()

    print("-" * 70)
    print("PER-SECOND DETAIL (first 30)")
    print("-" * 70)
    print()
    print(
        f"  {'pps':>6s}  {'dwt_before':>12s}  {'dwt_after':>12s}  {'bracket':>8s}  "
        f"{'gpt_before':>11s}  {'gpt_after':>10s}  {'gptΔ':>5s}  "
        f"{'elapsed':>7s}  {'phase':>5s}  {'res':>5s}"
    )
    print(
        f"  {'─'*6}  {'─'*12}  {'─'*12}  {'─'*8}  "
        f"{'─'*11}  {'─'*10}  {'─'*5}  {'─'*7}  {'─'*5}  {'─'*5}"
    )

    shown = 0
    for r in rows:
        dwt_before = r.get(f"{key}_dwt_before")
        dwt_after = r.get(f"{key}_dwt_after")
        gpt_before = r.get(f"{key}_gpt_before")
        gpt_after = r.get(f"{key}_gpt_after")
        bracket = r.get(f"{key}_dwt_bracket_cycles")
        elapsed = r.get(f"{key}_edge_elapsed_ns")
        phase = r.get(f"{key}_phase_offset_ns")
        residual = r.get(f"{key}_residual_ns")
        pps = r.get("pps_count", r.get("teensy_pps_count"))

        if None in (dwt_before, dwt_after, gpt_before, gpt_after, bracket, elapsed, phase, residual, pps):
            continue

        gpt_delta = int(gpt_after) - int(gpt_before)

        print(
            f"  {int(pps):>6d}  {int(dwt_before):>12d}  {int(dwt_after):>12d}  {int(bracket):>8d}  "
            f"{int(gpt_before):>11d}  {int(gpt_after):>10d}  {gpt_delta:>+5d}  "
            f"{int(elapsed):>7d}  {int(phase):>5d}  {int(residual):>+5d}"
        )
        shown += 1
        if shown >= 30:
            remaining = len(brackets) - shown
            if remaining > 0:
                print(f"  ... {remaining} more rows ...")
            break
    print()


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print("=" * 70)
    print("ZPNet EDGE TDC ANALYZER")
    print(f"Campaign: {campaign}  ({len(rows)} TIMEBASE records)")
    print("=" * 70)
    print()

    _analyze_ocxo(rows, "ocxo1", limit=limit)
    _analyze_ocxo(rows, "ocxo2", limit=limit)

    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print()
    print("  Goal:")
    print("    Determine whether the OCXO edge-detection bracket is narrow enough")
    print("    to identify a unique 10 MHz edge after PPS.")
    print()
    print("  Red flags:")
    print("    - bracket >= 100 ns")
    print("    - GPT delta > 1 inside a single bracket")
    print("    - common-mode elapsed/residual swings across both OCXOs")
    print()
    print("  Good signs:")
    print("    - narrow, stable bracket-width histogram")
    print("    - GPT delta consistently == 1")
    print("    - elapsed second-deltas stay small and boring")
    print()
    print("=" * 70)


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: edge_tdc_analyzer <campaign_name> [limit]")
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
    limit_val = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit_val)


if __name__ == "__main__":
    main()
