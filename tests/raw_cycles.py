"""
ZPNet Raw Cycles — v9 Best-Estimate vs Observed OCXO Timing Monitor

This version is aligned to the post-mitigation TIMEBASE fragment.

Primary goals:
  1. Verify PPS DWT prediction remains clean.
  2. Compare OCXO observed/raw deltas vs canonical best-estimate deltas.
  3. Show whether the best-estimate path escapes the 42-cycle lattice.
  4. Quantify improvement in GNSS interval residuals and DWT interval smoothness.
  5. Surface dequantizer state: enabled, inferred bucket size, confidence.

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
        f"  {'o1_obs':>12s}"
        f"  {'o1_best':>12s}"
        f"  {'mO1':>4s}"
        f"  {'mB1':>4s}"
        f"  {'ns1':>6s}"
        f"  {'dq1':>3s}"
        f"  {'b1':>4s}"
        f"  {'c1':>4s}"
        f"  {'o2_obs':>12s}"
        f"  {'o2_best':>12s}"
        f"  {'mO2':>4s}"
        f"  {'mB2':>4s}"
        f"  {'ns2':>6s}"
        f"  {'dq2':>3s}"
        f"  {'b2':>4s}"
        f"  {'c2':>4s}"
    )
    print(
        f"  {'─' * 5}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 7}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 4}"
        f"  {'─' * 4}"
        f"  {'─' * 6}"
        f"  {'─' * 3}"
        f"  {'─' * 4}"
        f"  {'─' * 4}"
        f"  {'─' * 12}"
        f"  {'─' * 12}"
        f"  {'─' * 4}"
        f"  {'─' * 4}"
        f"  {'─' * 6}"
        f"  {'─' * 3}"
        f"  {'─' * 4}"
        f"  {'─' * 4}"
    )

    w_dwt = Welford()

    w_o1_obs = Welford()
    w_o1_best = Welford()
    w_o2_obs = Welford()
    w_o2_best = Welford()

    w_o1_ns = Welford()
    w_o2_ns = Welford()

    w_s1 = Welford()
    w_s2 = Welford()
    w_ap1 = Welford()
    w_ap2 = Welford()

    buckets_o1_obs: Counter[int] = Counter()
    buckets_o1_best: Counter[int] = Counter()
    buckets_o2_obs: Counter[int] = Counter()
    buckets_o2_best: Counter[int] = Counter()

    residues_pps: Counter[int] = Counter()
    residues_o1_obs: Counter[int] = Counter()
    residues_o1_best: Counter[int] = Counter()
    residues_o2_obs: Counter[int] = Counter()
    residues_o2_best: Counter[int] = Counter()

    dequant_bucket_counts_1: Counter[int] = Counter()
    dequant_bucket_counts_2: Counter[int] = Counter()

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
            for w in (w_dwt, w_o1_obs, w_o1_best, w_o2_obs, w_o2_best, w_o1_ns, w_o2_ns, w_s1, w_s2, w_ap1, w_ap2):
                w.reset()

        prev_pps = pps

        predicted = _as_int(_frag(rec, "dwt_cycle_count_next_second_prediction"))
        actual = _as_int(_frag(rec, "dwt_cycle_count_between_pps"))

        if predicted is None:
            predicted = _as_int(rec.get("dwt_cycles_per_pps"))
        if actual is None:
            actual = _as_int(rec.get("dwt_delta_raw"))

        o1_obs = _diag_field(rec, "ocxo1", "diag_dwt_delta_observed")
        o1_best = _diag_field(rec, "ocxo1", "diag_dwt_delta_best")
        o2_obs = _diag_field(rec, "ocxo2", "diag_dwt_delta_observed")
        o2_best = _diag_field(rec, "ocxo2", "diag_dwt_delta_best")

        ns1 = _as_int(_frag(rec, "ocxo1_second_residual_ns"))
        ns2 = _as_int(_frag(rec, "ocxo2_second_residual_ns"))

        dq1 = _frag(rec, "ocxo1_diag_dequant_enabled")
        dq2 = _frag(rec, "ocxo2_diag_dequant_enabled")
        b1 = _diag_field(rec, "ocxo1", "diag_dequant_bucket_cycles")
        b2 = _diag_field(rec, "ocxo2", "diag_dequant_bucket_cycles")
        c1 = _diag_field(rec, "ocxo1", "diag_dequant_confidence")
        c2 = _diag_field(rec, "ocxo2", "diag_dequant_confidence")

        s1 = _shadow_to_isr(rec, "ocxo1")
        s2 = _shadow_to_isr(rec, "ocxo2")
        ap1 = _diag_field(rec, "ocxo1", "diag_approach_cycles")
        ap2 = _diag_field(rec, "ocxo2", "diag_approach_cycles")

        residual = None
        if predicted is not None and actual is not None:
            residual = actual - predicted
            w_dwt.update(float(residual))
            residues_pps[_mod(actual)] += 1

        if o1_obs is not None:
            w_o1_obs.update(float(o1_obs))
            buckets_o1_obs[o1_obs] += 1
            residues_o1_obs[_mod(o1_obs)] += 1

        if o1_best is not None:
            w_o1_best.update(float(o1_best))
            buckets_o1_best[o1_best] += 1
            residues_o1_best[_mod(o1_best)] += 1

        if o2_obs is not None:
            w_o2_obs.update(float(o2_obs))
            buckets_o2_obs[o2_obs] += 1
            residues_o2_obs[_mod(o2_obs)] += 1

        if o2_best is not None:
            w_o2_best.update(float(o2_best))
            buckets_o2_best[o2_best] += 1
            residues_o2_best[_mod(o2_best)] += 1

        if ns1 is not None:
            w_o1_ns.update(float(ns1))
        if ns2 is not None:
            w_o2_ns.update(float(ns2))

        if s1 is not None:
            w_s1.update(float(s1))
        if s2 is not None:
            w_s2.update(float(s2))

        if ap1 is not None:
            w_ap1.update(float(ap1))
        if ap2 is not None:
            w_ap2.update(float(ap2))

        if b1 is not None:
            dequant_bucket_counts_1[b1] += 1
        if b2 is not None:
            dequant_bucket_counts_2[b2] += 1

        dq1s = "Y" if dq1 is True else "N"
        dq2s = "Y" if dq2 is True else "N"

        print(
            f"  {pps:>5d}"
            f"  {_fmt_int(predicted, 12)}"
            f"  {_fmt_int(actual, 12)}"
            f"  {_fmt_int(residual, 7, signed=True)}"
            f"  {_fmt_int(o1_obs, 12)}"
            f"  {_fmt_int(o1_best, 12)}"
            f"  {_fmt_int(_mod(o1_obs), 4)}"
            f"  {_fmt_int(_mod(o1_best), 4)}"
            f"  {_fmt_int(ns1, 6, signed=True)}"
            f"  {dq1s:>3s}"
            f"  {_fmt_int(b1, 4)}"
            f"  {_fmt_int(c1, 4)}"
            f"  {_fmt_int(o2_obs, 12)}"
            f"  {_fmt_int(o2_best, 12)}"
            f"  {_fmt_int(_mod(o2_obs), 4)}"
            f"  {_fmt_int(_mod(o2_best), 4)}"
            f"  {_fmt_int(ns2, 6, signed=True)}"
            f"  {dq2s:>3s}"
            f"  {_fmt_int(b2, 4)}"
            f"  {_fmt_int(c2, 4)}"
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

    def print_series(name: str, w: Welford, unit: str = "cycles") -> None:
        if w.n < 2:
            return
        print(f"  {name}:")
        print(f"    n       = {w.n:,}")
        print(f"    mean    = {w.mean:,.2f} {unit}")
        print(f"    stddev  = {w.stddev:.2f} {unit}")
        print(f"    min/max = {w.min_val:,.0f} .. {w.max_val:,.0f}")
        print()

    print_series("OCXO1 observed delta", w_o1_obs)
    print_series("OCXO1 best delta", w_o1_best)
    print_series("OCXO2 observed delta", w_o2_obs)
    print_series("OCXO2 best delta", w_o2_best)
    print_series("OCXO1 second residual", w_o1_ns, "ns")
    print_series("OCXO2 second residual", w_o2_ns, "ns")
    print_series("OCXO1 shadow_to_isr_cycles", w_s1)
    print_series("OCXO2 shadow_to_isr_cycles", w_s2)
    print_series("OCXO1 approach_cycles", w_ap1)
    print_series("OCXO2 approach_cycles", w_ap2)

    print("  Residue summaries:")
    _residue_summary("PPS actual", residues_pps)
    _residue_summary("OCXO1 observed", residues_o1_obs)
    _residue_summary("OCXO1 best", residues_o1_best)
    _residue_summary("OCXO2 observed", residues_o2_obs)
    _residue_summary("OCXO2 best", residues_o2_best)

    print("  Dominant bucket summaries:")
    _bucket_summary("OCXO1 observed buckets", buckets_o1_obs)
    _bucket_summary("OCXO1 best buckets", buckets_o1_best)
    _bucket_summary("OCXO2 observed buckets", buckets_o2_obs)
    _bucket_summary("OCXO2 best buckets", buckets_o2_best)

    print("  Dequantizer inferred bucket counts:")
    _bucket_summary("OCXO1 inferred bucket size", dequant_bucket_counts_1)
    _bucket_summary("OCXO2 inferred bucket size", dequant_bucket_counts_2)

    if w_o1_obs.n >= 10 and w_o1_best.n >= 10:
        print("  Dequantization analysis:")
        print(f"    OCXO1 observed stddev: {w_o1_obs.stddev:.2f} cycles")
        print(f"    OCXO1 best stddev:     {w_o1_best.stddev:.2f} cycles")
        print(f"    OCXO2 observed stddev: {w_o2_obs.stddev:.2f} cycles")
        print(f"    OCXO2 best stddev:     {w_o2_best.stddev:.2f} cycles")
        print(f"    OCXO1 residual stddev: {w_o1_ns.stddev:.2f} ns" if w_o1_ns.n >= 2 else "    OCXO1 residual stddev: ---")
        print(f"    OCXO2 residual stddev: {w_o2_ns.stddev:.2f} ns" if w_o2_ns.n >= 2 else "    OCXO2 residual stddev: ---")

        if len(residues_o1_obs) == 1 and 0 in residues_o1_obs:
            print("    ✓ OCXO1 observed path remains locked to lattice")
        if len(residues_o1_best) > 1 or 0 not in residues_o1_best:
            print("    ✓ OCXO1 best path escapes pure lattice locking")

        if len(residues_o2_obs) == 1 and 0 in residues_o2_obs:
            print("    ✓ OCXO2 observed path remains locked to lattice")
        if len(residues_o2_best) > 1 or 0 not in residues_o2_best:
            print("    ✓ OCXO2 best path escapes pure lattice locking")
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