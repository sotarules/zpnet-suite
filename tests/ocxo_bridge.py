"""
ZPNet OCXO Bridge Hypothesis Analyzer

Purpose
-------
Analyze a campaign using TIMEBASE / TIMEBASE_FRAGMENT and test the hypothesis
that oscillation in ocxo*_gnss_ns_between_edges is dominated by a shared,
phase-dependent bridge/projection artifact rather than true OCXO physics.

This script is intentionally modeled after the user's campaign_analyzer.py,
but it is much more focused on:
  1) raw ocxo*_gnss_ns_between_edges behavior
  2) common-mode vs differential decomposition
  3) correlation with phase-related quantities
  4) whether both OCXOs move together more than independent physics would suggest

Usage
-----
    python -m zpnet.tests.ocxo_bridge_hypothesis_analyzer <campaign_name>
    .zt ocxo_bridge_hypothesis_analyzer Calibrate1
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000
PHASE_MOD_NS = 100


class WelfordStats:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float):
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
    def variance(self) -> float:
        return self.m2 / (self.n - 1) if self.n >= 2 else 0.0

    @property
    def stddev(self) -> float:
        return math.sqrt(self.variance)

    def summary(self, fmt: str = ".3f") -> str:
        if self.n == 0:
            return "no data"
        sd = f"{self.stddev:{fmt}}" if self.n >= 2 else "---"
        return (
            f"n={self.n}  mean={self.mean:+{fmt}}  sd={sd}  "
            f"min={self.min_val:{fmt}}  max={self.max_val:{fmt}}"
        )


class RunningCovariance:
    __slots__ = ("n", "mean_x", "mean_y", "c", "m2x", "m2y")

    def __init__(self):
        self.n = 0
        self.mean_x = 0.0
        self.mean_y = 0.0
        self.c = 0.0
        self.m2x = 0.0
        self.m2y = 0.0

    def update(self, x: float, y: float):
        self.n += 1
        dx = x - self.mean_x
        self.mean_x += dx / self.n
        dy = y - self.mean_y
        self.mean_y += dy / self.n
        self.c += dx * (y - self.mean_y)
        self.m2x += dx * (x - self.mean_x)
        self.m2y += dy * (y - self.mean_y)

    @property
    def corr(self) -> float:
        if self.n < 2 or self.m2x <= 0.0 or self.m2y <= 0.0:
            return 0.0
        return self.c / math.sqrt(self.m2x * self.m2y)


def _frag(row: Dict[str, Any], key: str, default=None):
    frag = row.get("fragment")
    if not frag:
        return default
    return frag.get(key, default)


def _frag_int(row: Dict[str, Any], key: str):
    v = _frag(row, key)
    return int(v) if v is not None else None


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """SELECT id, ts, payload
               FROM timebase
               WHERE campaign = %s
               ORDER BY (payload->>'pps_count')::int ASC""",
            (campaign,),
        )
        rows = cur.fetchall()

    result = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        payload["_db_id"] = row["id"]
        payload["_db_ts"] = str(row["ts"])
        result.append(payload)
    return result


def find_recovery_boundaries(rows: List[Dict[str, Any]]) -> set[int]:
    boundaries: set[int] = set()
    for i in range(1, len(rows)):
        prev = int(rows[i - 1]["pps_count"])
        curr = int(rows[i]["pps_count"])
        if curr > prev + 1:
            boundaries.add(curr)
    return boundaries


def signed_phase_diff(a: int, b: int, mod: int = PHASE_MOD_NS) -> int:
    half = mod // 2
    d = (a - b) % mod
    if d >= half:
        d -= mod
    return d


def print_campaign_overview(rows: List[Dict[str, Any]], recovery_boundaries: set[int]):
    pps_counts = [int(r["pps_count"]) for r in rows]
    pps_min = min(pps_counts)
    pps_max = max(pps_counts)
    expected = pps_max - pps_min + 1
    duration = pps_max - pps_min
    first_ts = rows[0].get("gnss_time_utc") or rows[0].get("system_time_utc", "?")
    last_ts = rows[-1].get("gnss_time_utc") or rows[-1].get("system_time_utc", "?")

    hrs = duration // 3600
    mins = (duration % 3600) // 60
    secs = duration % 60

    print("=" * 92)
    print(f"OCXO BRIDGE HYPOTHESIS ANALYZER: {rows[0].get('campaign', '?')}")
    print("=" * 92)
    print()
    print(f"  Time range:        {first_ts}")
    print(f"                  -> {last_ts}")
    print(f"  Location:          {rows[0].get('location', '?')}")
    print(f"  pps_count range:   {pps_min} -> {pps_max}")
    print(f"  Duration:          {hrs:02d}:{mins:02d}:{secs:02d}")
    print(f"  Expected records:  {expected:,}")
    print(f"  Actual records:    {len(rows):,}")
    print(f"  Missing records:   {expected - len(rows):,}")
    print(f"  Recovery events:   {len(recovery_boundaries)}")
    print(f"  Servo mode:        {_frag(rows[-1], 'calibrate_ocxo', '?')}")
    print()


def valid_rows(rows: List[Dict[str, Any]], recovery_boundaries: set[int]) -> List[Dict[str, Any]]:
    out = []
    for row in rows:
        pps = int(row["pps_count"])
        if pps in recovery_boundaries:
            continue

        ok1 = _frag(row, "ocxo1_zero_established")
        ok2 = _frag(row, "ocxo2_zero_established")
        g1 = _frag_int(row, "ocxo1_gnss_ns_between_edges")
        g2 = _frag_int(row, "ocxo2_gnss_ns_between_edges")
        r1 = _frag_int(row, "ocxo1_second_residual_ns")
        r2 = _frag_int(row, "ocxo2_second_residual_ns")

        if not ok1 or not ok2:
            continue
        if None in (g1, g2, r1, r2):
            continue
        if g1 <= 0 or g2 <= 0:
            continue
        out.append(row)
    return out


def analyze_raw_spans(rows: List[Dict[str, Any]]):
    print("-" * 92)
    print("1) RAW OCXO SPANS: ocxo*_gnss_ns_between_edges")
    print("-" * 92)

    for label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        stats = WelfordStats()
        residual_check = WelfordStats()
        mismatches = 0

        for row in rows:
            span = _frag_int(row, f"{prefix}_gnss_ns_between_edges")
            residual = _frag_int(row, f"{prefix}_second_residual_ns")
            if span is None or residual is None:
                continue
            stats.update(float(span))
            residual_check.update(float(NS_PER_SECOND - span))
            expected_residual = NS_PER_SECOND - span
            if expected_residual != residual:
                mismatches += 1

        print()
        print(f"  [{label}] GNSS ns between lawful OCXO second edges:")
        print(f"    {stats.summary('.3f')}")
        print(f"    Derived residual (1e9 - span):")
        print(f"    {residual_check.summary('.3f')}")
        print(f"    Residual formula mismatches: {mismatches}")


def analyze_common_mode(rows: List[Dict[str, Any]]):
    print()
    print("-" * 92)
    print("2) COMMON-MODE vs DIFFERENTIAL")
    print("-" * 92)

    raw_diff = WelfordStats()
    raw_common_dev = WelfordStats()
    residual_diff = WelfordStats()
    residual_common = WelfordStats()
    corr_raw = RunningCovariance()
    corr_res = RunningCovariance()

    same_sign = 0
    opposite_sign = 0
    same_integer = 0

    for row in rows:
        g1 = _frag_int(row, "ocxo1_gnss_ns_between_edges")
        g2 = _frag_int(row, "ocxo2_gnss_ns_between_edges")
        r1 = _frag_int(row, "ocxo1_second_residual_ns")
        r2 = _frag_int(row, "ocxo2_second_residual_ns")
        if None in (g1, g2, r1, r2):
            continue

        raw1_dev = g1 - NS_PER_SECOND
        raw2_dev = g2 - NS_PER_SECOND

        common_raw = (raw1_dev + raw2_dev) / 2.0
        diff_raw = raw1_dev - raw2_dev

        common_res = (r1 + r2) / 2.0
        diff_res = r1 - r2

        raw_common_dev.update(common_raw)
        raw_diff.update(diff_raw)
        residual_common.update(common_res)
        residual_diff.update(diff_res)

        corr_raw.update(float(raw1_dev), float(raw2_dev))
        corr_res.update(float(r1), float(r2))

        if (r1 > 0 and r2 > 0) or (r1 < 0 and r2 < 0):
            same_sign += 1
        elif (r1 > 0 > r2) or (r2 > 0 > r1):
            opposite_sign += 1

        if r1 == r2:
            same_integer += 1

    print()
    print("  Raw span deviations from 1e9 ns:")
    print(f"    Common-mode:   {raw_common_dev.summary('.3f')}")
    print(f"    Differential:  {raw_diff.summary('.3f')}")
    print(f"    Corr(raw1_dev, raw2_dev) = {corr_raw.corr:+.6f}")

    print()
    print("  Residual stream decomposition:")
    print(f"    Common-mode:   {residual_common.summary('.3f')}")
    print(f"    Differential:  {residual_diff.summary('.3f')}")
    print(f"    Corr(res1, res2) = {corr_res.corr:+.6f}")

    print()
    print("  Lockstep fingerprints:")
    print(f"    Same-sign residual seconds:      {same_sign}")
    print(f"    Opposite-sign residual seconds:  {opposite_sign}")
    print(f"    Exactly equal integer residuals: {same_integer}")

    if residual_common.stddev > 0:
        dominance = residual_diff.stddev / residual_common.stddev
        print(f"    Differential/common sd ratio:    {dominance:.3f}")
        if dominance < 0.50:
            print("    Signal: common-mode dominates differential -> strong bridge/common-path suspicion")
        elif dominance < 1.00:
            print("    Signal: common-mode still stronger than differential")
        else:
            print("    Signal: differential is not smaller than common-mode")


def analyze_phase_coupling(rows: List[Dict[str, Any]]):
    print()
    print("-" * 92)
    print("3) PHASE COUPLING TESTS")
    print("-" * 92)

    for label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        corr_res_phase = RunningCovariance()
        corr_span_phase = RunningCovariance()
        corr_res_step = RunningCovariance()
        phase_step_abs = WelfordStats()

        bins: dict[int, WelfordStats] = {}
        prev_phase = None

        for row in rows:
            phase = _frag_int(row, f"{prefix}_phase_offset_ns")
            res = _frag_int(row, f"{prefix}_second_residual_ns")
            span = _frag_int(row, f"{prefix}_gnss_ns_between_edges")
            if None in (phase, res, span):
                continue

            phase_mod = phase % PHASE_MOD_NS
            corr_res_phase.update(float(phase_mod), float(res))
            corr_span_phase.update(float(phase_mod), float(span - NS_PER_SECOND))

            if phase_mod not in bins:
                bins[phase_mod] = WelfordStats()
            bins[phase_mod].update(float(res))

            if prev_phase is not None:
                step = signed_phase_diff(phase_mod, prev_phase % PHASE_MOD_NS, PHASE_MOD_NS)
                phase_step_abs.update(float(abs(step)))
                corr_res_step.update(float(step), float(res))
            prev_phase = phase

        print()
        print(f"  [{label}]")
        print(f"    Corr(phase_mod_100, residual)      = {corr_res_phase.corr:+.6f}")
        print(f"    Corr(phase_mod_100, span-1e9)      = {corr_span_phase.corr:+.6f}")
        if phase_step_abs.n:
            print(f"    Mean |phase step| mod 100          = {phase_step_abs.mean:.3f} ns")
        else:
            print("    Mean |phase step| mod 100          = no data")
        print(f"    Corr(phase_step_mod_100, residual) = {corr_res_step.corr:+.6f}")

        populated = [(k, v.n, v.mean) for k, v in bins.items() if v.n >= 20]
        populated.sort(key=lambda x: x[0])

        if populated:
            means = [m for _, _, m in populated]
            spread = max(means) - min(means)
            print(f"    Residual mean spread across phase bins: {spread:.3f} ns")
            if spread > 5.0:
                print("    Signal: residual mean depends on phase bin -> phase-coupled artifact likely")
            print("    Phase-bin sample (phase_mod -> mean residual, n):")
            for phase_mod, n, mean in populated[:10]:
                print(f"      {phase_mod:>3d} -> mean={mean:+8.3f}  n={n}")
            if len(populated) > 10:
                print(f"      ... and {len(populated) - 10} more bins")
        else:
            print("    Not enough phase-bin population for a stable bin-spread test")


def analyze_dwt_bridge_proxies(rows: List[Dict[str, Any]]):
    print()
    print("-" * 92)
    print("4) DWT / BRIDGE PROXY TESTS")
    print("-" * 92)

    for label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        corr_shadow = RunningCovariance()
        corr_correction = RunningCovariance()
        corr_dwt_span = RunningCovariance()
        corr_raw_vs_res = RunningCovariance()

        shadow_stats = WelfordStats()
        corr_stats = WelfordStats()
        dwt_between_stats = WelfordStats()

        prev_dwt = None

        for row in rows:
            res = _frag_int(row, f"{prefix}_second_residual_ns")
            span = _frag_int(row, f"{prefix}_gnss_ns_between_edges")
            shadow = _frag_int(row, f"{prefix}_diag_shadow_to_isr_cycles")
            corr = _frag_int(row, f"{prefix}_diag_dwt_event_correction_cycles")
            dwt_at_edge = _frag_int(row, f"{prefix}_dwt_at_edge")
            if res is None or span is None:
                continue

            corr_raw_vs_res.update(float(span - NS_PER_SECOND), float(res))

            if shadow is not None:
                shadow_stats.update(float(shadow))
                corr_shadow.update(float(shadow), float(res))
            if corr is not None:
                corr_stats.update(float(corr))
                corr_correction.update(float(corr), float(res))
            if dwt_at_edge is not None and prev_dwt is not None:
                dwt_between = dwt_at_edge - prev_dwt
                dwt_between_stats.update(float(dwt_between))
                corr_dwt_span.update(float(dwt_between), float(span))
            if dwt_at_edge is not None:
                prev_dwt = dwt_at_edge

        print()
        print(f"  [{label}]")
        print(f"    Corr(span-1e9, residual)           = {corr_raw_vs_res.corr:+.6f}")
        if shadow_stats.n:
            print(f"    shadow_to_isr_cycles               = {shadow_stats.summary('.3f')}")
            print(f"    Corr(shadow_to_isr_cycles, residual)= {corr_shadow.corr:+.6f}")
        else:
            print("    No shadow_to_isr_cycles data")
        if corr_stats.n:
            print(f"    event_correction_cycles            = {corr_stats.summary('.3f')}")
            print(f"    Corr(correction_cycles, residual)  = {corr_correction.corr:+.6f}")
        else:
            print("    No correction_cycles data")
        if dwt_between_stats.n:
            print(f"    dwt cycles between edges           = {dwt_between_stats.summary('.3f')}")
            print(f"    Corr(dwt_between_edges, gnss_span) = {corr_dwt_span.corr:+.6f}")
        else:
            print("    No dwt_at_edge sequence data")


def analyze_temporal_patterns(rows: List[Dict[str, Any]]):
    print()
    print("-" * 92)
    print("5) TEMPORAL PATTERNING")
    print("-" * 92)

    for label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        delta_stats = WelfordStats()
        abs_delta_stats = WelfordStats()
        toggles = 0
        prev_res = None
        prev_delta = None

        for row in rows:
            res = _frag_int(row, f"{prefix}_second_residual_ns")
            if res is None:
                continue
            if prev_res is not None:
                d = res - prev_res
                delta_stats.update(float(d))
                abs_delta_stats.update(float(abs(d)))
                if prev_delta is not None and ((d > 0 > prev_delta) or (d < 0 < prev_delta)):
                    toggles += 1
                prev_delta = d
            prev_res = res

        print()
        print(f"  [{label}] residual first difference:")
        print(f"    delta residual:   {delta_stats.summary('.3f')}")
        print(f"    |delta residual|: {abs_delta_stats.summary('.3f')}")
        print(f"    Sign toggles in delta sequence: {toggles}")
        if abs_delta_stats.n and abs_delta_stats.mean > 5.0:
            print("    Signal: residual is moving around materially from second to second")


def print_verdict(rows: List[Dict[str, Any]]):
    corr_res = RunningCovariance()
    common_stats = WelfordStats()
    diff_stats = WelfordStats()

    for row in rows:
        r1 = _frag_int(row, "ocxo1_second_residual_ns")
        r2 = _frag_int(row, "ocxo2_second_residual_ns")
        if None in (r1, r2):
            continue
        corr_res.update(float(r1), float(r2))
        common_stats.update((r1 + r2) / 2.0)
        diff_stats.update(float(r1 - r2))

    print()
    print("=" * 92)
    print("VERDICT")
    print("=" * 92)

    if common_stats.n == 0:
        print("No valid OCXO seconds found.")
        return

    ratio = (diff_stats.stddev / common_stats.stddev) if common_stats.stddev > 0 else float("inf")
    print(f"Residual corr(OCXO1, OCXO2):  {corr_res.corr:+.6f}")
    print(f"Common-mode sd:               {common_stats.stddev:.3f} ns")
    print(f"Differential sd:              {diff_stats.stddev:.3f} ns")
    print(f"Differential/common ratio:    {ratio:.3f}")
    print()

    if corr_res.corr > 0.7 and ratio < 0.7:
        print("Strong evidence of a shared common-mode term in the OCXO residuals.")
        print("That is much more consistent with bridge/projection or reference-path artifact")
        print("than with two independent rock-solid OCXOs physically wandering together.")
    elif corr_res.corr > 0.4:
        print("There is meaningful shared motion between the two OCXO residual streams.")
        print("That still leans toward common-path influence, though less decisively.")
    else:
        print("The two OCXO residual streams do not show strong lockstep correlation.")
        print("This run does not strongly support the common-mode bridge-artifact hypothesis.")
    print("=" * 92)


def analyze(campaign: str):
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    recovery_boundaries = find_recovery_boundaries(rows)
    print_campaign_overview(rows, recovery_boundaries)

    good = valid_rows(rows, recovery_boundaries)
    print(f"Valid rows with both OCXOs established and spans present: {len(good):,}")
    print()

    analyze_raw_spans(good)
    analyze_common_mode(good)
    analyze_phase_coupling(good)
    analyze_dwt_bridge_proxies(good)
    analyze_temporal_patterns(good)
    print_verdict(good)


def main():
    if len(sys.argv) < 2:
        print("Usage: ocxo_bridge_hypothesis_analyzer <campaign_name>")
        sys.exit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()
