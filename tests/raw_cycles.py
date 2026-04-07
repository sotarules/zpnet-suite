"""
ZPNet Raw Cycles — v8 Payload-Matched Quantization Monitor

This version is aligned to the actual TIMEBASE fragment fields observed in
current records.

Primary goals:
  1. Verify PPS DWT prediction remains clean.
  2. Compare OCXO alpha deltas vs diagnostic raw/adjusted ISR deltas.
  3. Surface mod-42 residue patterns directly in the report.
  4. Compute shadow_to_isr_cycles from:
         diag_dwt_isr_entry_raw - diag_shadow_dwt
     since that field is not stored explicitly.

Observed payload field names:
  ocxo1_diag_dwt_delta_raw
  ocxo1_diag_dwt_delta_adjusted
  ocxo1_diag_dwt_isr_entry_raw
  ocxo1_diag_shadow_dwt
  ocxo1_diag_approach_cycles
  ... and the analogous ocxo2 / pps fields.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    .zt raw_cycles gpt1
    .zt raw_cycles gpt1 100
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


MODULUS = 42
TOP_BUCKETS = 8


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

    def reset(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")


# ─────────────────────────────────────────────────────────────────────
# Fragment-aware field access
# ─────────────────────────────────────────────────────────────────────

def _frag(rec: Dict[str, Any], key: str) -> Any:
    frag = rec.get("fragment")
    if isinstance(frag, dict):
        v = frag.get(key)
        if v is not None:
            return v
    return rec.get(key)


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def _mod(v: Optional[int], mod: int = MODULUS) -> Optional[int]:
    if v is None:
        return None
    return v % mod


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        return f"{'---':>{width}s}" if width else "---"
    s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


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

    result: List[Dict[str, Any]] = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        result.append(p)

    return result


# ─────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────

def _bucket_summary(name: str, counter: Counter[int], limit: int = TOP_BUCKETS) -> None:
    print(f"  {name}:")
    if not counter:
        print("    (no data)")
        print()
        return

    total = sum(counter.values())
    for value, n in counter.most_common(limit):
        pct = (100.0 * n / total) if total else 0.0
        print(f"    {value:,}: {n:>4d}  ({pct:5.1f}%)")
    print()


def _residue_summary(name: str, counter: Counter[int], mod: int = MODULUS) -> None:
    print(f"  {name} mod {mod}:")
    if not counter:
        print("    (no data)")
        print()
        return

    total = sum(counter.values())
    for residue, n in sorted(counter.items()):
        pct = (100.0 * n / total) if total else 0.0
        print(f"    {residue:>2d}: {n:>4d}  ({pct:5.1f}%)")
    print()


def _diag_field(rec: Dict[str, Any], prefix: str, suffix: str) -> Optional[int]:
    return _as_int(_frag(rec, f"{prefix}_{suffix}"))


def _shadow_to_isr(rec: Dict[str, Any], prefix: str) -> Optional[int]:
    raw = _diag_field(rec, prefix, "diag_dwt_isr_entry_raw")
    shadow = _diag_field(rec, prefix, "diag_shadow_dwt")
    if raw is None or shadow is None:
        return None
    return raw - shadow


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

    print(
        f"  {'pps':>5s}"
        f"  {'pred':>12s}"
        f"  {'act':>12s}"
        f"  {'res':>7s}"
        f"  {'m42':>4s}"
        f"  {'o1':>12s}"
        f"  {'r1':>12s}"
        f"  {'a1':>12s}"
        f"  {'m1':>4s}"
        f"  {'s1':>4s}"
        f"  {'ap1':>5s}"
        f"  {'o2':>12s}"
        f"  {'r2':>12s}"
        f"  {'a2':>12s}"
        f"  {'m2':>4s}"
        f"  {'s2':>4s}"
        f"  {'ap2':>5s}"
    )
    print(
        f"  {'─' * 5}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 7}"
        f"  {'─' * 4}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 4}"
        f"  {'─' * 4}"
        f"  {'─' * 5}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 4}"
        f"  {'─' * 4}"
        f"  {'─' * 5}"
    )

    w_dwt = Welford()
    w_o1 = Welford()
    w_o2 = Welford()
    w_r1 = Welford()
    w_a1 = Welford()
    w_r2 = Welford()
    w_a2 = Welford()
    w_s1 = Welford()
    w_s2 = Welford()
    w_ap1 = Welford()
    w_ap2 = Welford()

    buckets_o1: Counter[int] = Counter()
    buckets_o2: Counter[int] = Counter()
    buckets_r1: Counter[int] = Counter()
    buckets_a1: Counter[int] = Counter()
    buckets_r2: Counter[int] = Counter()
    buckets_a2: Counter[int] = Counter()

    residues_pps: Counter[int] = Counter()
    residues_o1: Counter[int] = Counter()
    residues_r1: Counter[int] = Counter()
    residues_a1: Counter[int] = Counter()
    residues_o2: Counter[int] = Counter()
    residues_r2: Counter[int] = Counter()
    residues_a2: Counter[int] = Counter()

    count = 0
    gaps = 0
    prev_pps: Optional[int] = None

    for rec in rows:
        pps = rec.get("pps_count")
        if pps is None:
            continue
        pps = int(pps)

        if prev_pps is not None and pps != prev_pps + 1:
            skipped = pps - prev_pps - 1
            gaps += 1
            print(f"  {'':>5s}  --- gap {prev_pps} → {pps} ({skipped}s lost, stats reset) ---")
            for w in (w_dwt, w_o1, w_o2, w_r1, w_a1, w_r2, w_a2, w_s1, w_s2, w_ap1, w_ap2):
                w.reset()

        prev_pps = pps

        predicted = _as_int(_frag(rec, "dwt_cycle_count_next_second_prediction"))
        actual = _as_int(_frag(rec, "dwt_cycle_count_between_pps"))

        if predicted is None:
            predicted = _as_int(rec.get("dwt_cycles_per_pps"))
        if actual is None:
            actual = _as_int(rec.get("dwt_delta_raw"))

        o1 = _as_int(_frag(rec, "ocxo1_dwt_cycles_between_edges"))
        o2 = _as_int(_frag(rec, "ocxo2_dwt_cycles_between_edges"))

        r1 = _diag_field(rec, "ocxo1", "diag_dwt_delta_raw")
        a1 = _diag_field(rec, "ocxo1", "diag_dwt_delta_adjusted")
        r2 = _diag_field(rec, "ocxo2", "diag_dwt_delta_raw")
        a2 = _diag_field(rec, "ocxo2", "diag_dwt_delta_adjusted")

        s1 = _shadow_to_isr(rec, "ocxo1")
        s2 = _shadow_to_isr(rec, "ocxo2")

        ap1 = _diag_field(rec, "ocxo1", "diag_approach_cycles")
        ap2 = _diag_field(rec, "ocxo2", "diag_approach_cycles")

        residual = None
        if predicted is not None and actual is not None:
            residual = actual - predicted
            w_dwt.update(float(residual))
            residues_pps[_mod(actual)] += 1

        if o1 is not None:
            w_o1.update(float(o1))
            buckets_o1[o1] += 1
            residues_o1[_mod(o1)] += 1

        if o2 is not None:
            w_o2.update(float(o2))
            buckets_o2[o2] += 1
            residues_o2[_mod(o2)] += 1

        if r1 is not None:
            w_r1.update(float(r1))
            buckets_r1[r1] += 1
            residues_r1[_mod(r1)] += 1

        if a1 is not None:
            w_a1.update(float(a1))
            buckets_a1[a1] += 1
            residues_a1[_mod(a1)] += 1

        if r2 is not None:
            w_r2.update(float(r2))
            buckets_r2[r2] += 1
            residues_r2[_mod(r2)] += 1

        if a2 is not None:
            w_a2.update(float(a2))
            buckets_a2[a2] += 1
            residues_a2[_mod(a2)] += 1

        if s1 is not None:
            w_s1.update(float(s1))
        if s2 is not None:
            w_s2.update(float(s2))

        if ap1 is not None:
            w_ap1.update(float(ap1))
        if ap2 is not None:
            w_ap2.update(float(ap2))

        print(
            f"  {pps:>5d}"
            f"  {_fmt_int(predicted, 12)}"
            f"  {_fmt_int(actual, 12)}"
            f"  {_fmt_int(residual, 7, signed=True)}"
            f"  {_fmt_int(_mod(actual), 4)}"
            f"  {_fmt_int(o1, 12)}"
            f"  {_fmt_int(r1, 12)}"
            f"  {_fmt_int(a1, 12)}"
            f"  {_fmt_int(_mod(a1), 4)}"
            f"  {_fmt_int(s1, 4)}"
            f"  {_fmt_int(ap1, 5)}"
            f"  {_fmt_int(o2, 12)}"
            f"  {_fmt_int(r2, 12)}"
            f"  {_fmt_int(a2, 12)}"
            f"  {_fmt_int(_mod(a2), 4)}"
            f"  {_fmt_int(s2, 4)}"
            f"  {_fmt_int(ap2, 5)}"
        )

        count += 1
        if limit and count >= limit:
            break

    print()
    print(f"  {count} rows, {gaps} recovery gap(s)")
    print()

    if w_dwt.n >= 2:
        pred_ns = w_dwt.stddev * (125.0 / 126.0)
        print("  PPS DWT prediction residual (actual - predicted):")
        print(f"    n       = {w_dwt.n:,}")
        print(f"    mean    = {w_dwt.mean:+.2f} cycles")
        print(f"    stddev  = {w_dwt.stddev:.2f} cycles")
        print(f"    stderr  = {w_dwt.stderr:.3f} cycles")
        print(f"    1σ interpolation uncertainty = {pred_ns:.2f} ns")
        print()

    def print_series(name: str, w: Welford) -> None:
        if w.n < 2:
            return
        print(f"  {name}:")
        print(f"    n       = {w.n:,}")
        print(f"    mean    = {w.mean:,.2f}")
        print(f"    stddev  = {w.stddev:.2f}")
        print(f"    min/max = {w.min_val:,.0f} .. {w.max_val:,.0f}")
        print()

    print_series("OCXO1 alpha delta", w_o1)
    print_series("OCXO1 raw diagnostic delta", w_r1)
    print_series("OCXO1 adjusted diagnostic delta", w_a1)
    print_series("OCXO2 alpha delta", w_o2)
    print_series("OCXO2 raw diagnostic delta", w_r2)
    print_series("OCXO2 adjusted diagnostic delta", w_a2)
    print_series("OCXO1 shadow_to_isr_cycles", w_s1)
    print_series("OCXO2 shadow_to_isr_cycles", w_s2)
    print_series("OCXO1 approach_cycles", w_ap1)
    print_series("OCXO2 approach_cycles", w_ap2)

    print("  Residue summaries:")
    _residue_summary("PPS actual", residues_pps)
    _residue_summary("OCXO1 alpha", residues_o1)
    _residue_summary("OCXO1 raw diagnostic", residues_r1)
    _residue_summary("OCXO1 adjusted diagnostic", residues_a1)
    _residue_summary("OCXO2 alpha", residues_o2)
    _residue_summary("OCXO2 raw diagnostic", residues_r2)
    _residue_summary("OCXO2 adjusted diagnostic", residues_a2)

    print("  Dominant bucket summaries:")
    _bucket_summary("OCXO1 alpha buckets", buckets_o1)
    _bucket_summary("OCXO1 raw diagnostic buckets", buckets_r1)
    _bucket_summary("OCXO1 adjusted diagnostic buckets", buckets_a1)
    _bucket_summary("OCXO2 alpha buckets", buckets_o2)
    _bucket_summary("OCXO2 raw diagnostic buckets", buckets_r2)
    _bucket_summary("OCXO2 adjusted diagnostic buckets", buckets_a2)

    if w_o1.n >= 10 and w_dwt.n >= 10:
        print("  Quantization analysis:")
        print(f"    PPS DWT stddev:       {w_dwt.stddev:.2f} cycles")
        print(f"    OCXO1 alpha stddev:   {w_o1.stddev:.2f} cycles")
        print(f"    OCXO1 raw stddev:     {w_r1.stddev:.2f} cycles" if w_r1.n >= 2 else "    OCXO1 raw stddev:     ---")
        print(f"    OCXO1 adjusted stddev:{w_a1.stddev:.2f} cycles" if w_a1.n >= 2 else "    OCXO1 adjusted stddev:---")
        print(f"    OCXO2 alpha stddev:   {w_o2.stddev:.2f} cycles")
        print(f"    OCXO2 raw stddev:     {w_r2.stddev:.2f} cycles" if w_r2.n >= 2 else "    OCXO2 raw stddev:     ---")
        print(f"    OCXO2 adjusted stddev:{w_a2.stddev:.2f} cycles" if w_a2.n >= 2 else "    OCXO2 adjusted stddev:---")

        if w_o1.stddev > 15:
            print("    ⚠ OCXO1 bucketed / multimodal regime likely")
        if w_o2.stddev > 15:
            print("    ⚠ OCXO2 bucketed / multimodal regime likely")

        if w_r1.n >= 2 and w_a1.n >= 2:
            same1 = abs(w_r1.mean - w_a1.mean) < 1e-9 and abs(w_r1.stddev - w_a1.stddev) < 1e-9
            if same1:
                print("    ✓ OCXO1 raw and adjusted deltas are numerically identical")
        if w_r2.n >= 2 and w_a2.n >= 2:
            same2 = abs(w_r2.mean - w_a2.mean) < 1e-9 and abs(w_r2.stddev - w_a2.stddev) < 1e-9
            if same2:
                print("    ✓ OCXO2 raw and adjusted deltas are numerically identical")
        print()


# ─────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────

def main() -> None:
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