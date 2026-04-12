"""
ZPNet Campaign Analyzer — v11 (fragment-as-subdict schema)

Rewritten for the current TIMEBASE architecture where the Teensy's raw
TIMEBASE_FRAGMENT is embedded verbatim as payload.fragment.

v11 changes:
  - Prespin correlation: for each OCXO alarm second, captures the prespin
    diagnostic state (timed_out, fired, approach_cycles, shadow_to_isr)
    and correlates alarm seconds with prespin timeouts.
  - Full forensic snapshot at each alarm second: DWT values, prespin state,
    counter32, correction cycles — everything needed to diagnose root cause.
  - Directionality analysis: are deviations consistently short, long, or mixed?
    A consistent bias is a strong fingerprint for a specific failure mode.
  - Deviation magnitude histogram: bins alarm deviations to show distribution.
  - Temporal clustering: are alarm seconds randomly distributed or bunched?

Usage:
    python -m zpnet.tests.campaign_analyzer <campaign_name>
    .zt campaign_analyzer Calibrate1
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from typing import Any, Dict, List, Optional, Set, Tuple

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000
OCXO_SECOND_ALARM_NS = 10_000
OCXO_SECOND_WARN_NS  = 500
PHASE_STEP_ALARM_NS = 100_000
PPB_STEP_ALARM = 100.0
PPB_ABSOLUTE_THRESHOLD = 10_000
WELFORD_STDDEV_ALARM = 500.0


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
        if x < self.min_val: self.min_val = x
        if x > self.max_val: self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    def summary(self, fmt: str = ".3f") -> str:
        if self.n == 0:
            return "no data"
        sd = f"{self.stddev:{fmt}}" if self.n >= 2 else "---"
        return (f"n={self.n}  mean={self.mean:+{fmt}}  sd={sd}  "
                f"min={self.min_val:{fmt}}  max={self.max_val:{fmt}}")


def _frag(row, key, default=None):
    frag = row.get("fragment")
    if not frag: return default
    return frag.get(key, default)

def _frag_int(row, key):
    v = _frag(row, key)
    return int(v) if v is not None else None

def _frag_float(row, key):
    v = _frag(row, key)
    return float(v) if v is not None else None


def fetch_timebase(campaign):
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """SELECT id, ts, payload FROM timebase
               WHERE campaign = %s
               ORDER BY (payload->>'pps_count')::int ASC""",
            (campaign,),
        )
        rows = cur.fetchall()
    result = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str): p = json.loads(p)
        p["_db_id"] = row["id"]
        p["_db_ts"] = str(row["ts"])
        result.append(p)
    return result


def find_recovery_boundaries(rows):
    boundaries = set()
    for i in range(1, len(rows)):
        prev = int(rows[i - 1]["pps_count"])
        curr = int(rows[i]["pps_count"])
        if curr > prev + 1: boundaries.add(curr)
    return boundaries


def print_campaign_overview(rows, recovery_boundaries):
    total = len(rows)
    pps_counts = [int(r["pps_count"]) for r in rows]
    pps_min, pps_max = min(pps_counts), max(pps_counts)
    expected_count = pps_max - pps_min + 1
    campaign_seconds = pps_max - pps_min
    first_ts = rows[0].get("gnss_time_utc") or rows[0].get("system_time_utc", "?")
    last_ts = rows[-1].get("gnss_time_utc") or rows[-1].get("system_time_utc", "?")
    hrs = campaign_seconds // 3600
    mins = (campaign_seconds % 3600) // 60
    secs = campaign_seconds % 60

    print("=" * 78)
    print(f"CAMPAIGN ANALYSIS: {rows[0].get('campaign', '?')}  (v11 fragment schema)")
    print("=" * 78)
    print()
    print(f"  Time range:        {first_ts}")
    print(f"                  -> {last_ts}")
    print(f"  Location:          {rows[0].get('location', '?')}")
    print(f"  pps_count range:   {pps_min} -> {pps_max}")
    print(f"  Campaign seconds:  {campaign_seconds:,}")
    print(f"  Campaign duration: {hrs:02d}:{mins:02d}:{secs:02d}")
    print()
    print(f"  Expected records:  {expected_count:,}")
    print(f"  Actual records:    {total:,}")
    print(f"  Missing records:   {expected_count - total:,}")
    if recovery_boundaries:
        print(f"  Recovery events:   {len(recovery_boundaries)}")
    servo = _frag(rows[-1], "calibrate_ocxo", "?")
    print(f"  Servo mode:        {servo}")
    return campaign_seconds


def analyze_pps_continuity(rows, recovery_boundaries):
    anomalies = []
    print()
    print("-" * 78)
    print("PPS_COUNT CONTINUITY")
    print("-" * 78)
    jumps = recovery_jumps = regressions = repeats = 0
    for i in range(1, len(rows)):
        prev = int(rows[i - 1]["pps_count"])
        curr = int(rows[i]["pps_count"])
        delta = curr - prev
        if delta == 0:
            repeats += 1
            if repeats <= 5: print(f"  WARN REPEAT at pps_count={curr}")
        elif delta < 0:
            regressions += 1
            if regressions <= 5: print(f"  WARN REGRESSION: {prev} -> {curr} (delta={delta})")
        elif delta > 1:
            if curr in recovery_boundaries:
                recovery_jumps += 1
                print(f"  INFO RECOVERY GAP: {prev} -> {curr} (skipped {delta - 1})")
            else:
                jumps += 1
                if jumps <= 10: print(f"  WARN GAP: {prev} -> {curr} (skipped {delta - 1})")
    print(f"\n  Unexpected gaps:   {jumps}")
    print(f"  Recovery gaps:     {recovery_jumps}")
    print(f"  Repeats:           {repeats}")
    print(f"  Regressions:       {regressions}")
    if jumps:       anomalies.append(f"{jumps} unexpected pps_count gaps")
    if repeats:     anomalies.append(f"{repeats} pps_count repeats")
    if regressions: anomalies.append(f"{regressions} pps_count regressions")
    return anomalies


def analyze_ocxo_integrity(rows, recovery_boundaries):
    anomalies = []
    for ocxo_label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        print()
        print("-" * 78)
        print(f"{ocxo_label} PER-SECOND INTEGRITY")
        print("-" * 78)

        gnss_between_stats = WelfordStats()
        residual_stats = WelfordStats()
        window_error_stats = WelfordStats()
        phase_step_stats = WelfordStats()

        alarm_records = []
        second_warns = []
        ns_count_alarms = []
        phase_step_alarms = []
        ppb_step_alarms = []
        window_alarms = []

        prev_phase_offset = None
        prev_ppb = None
        prev_ns_count = None
        valid_seconds = 0
        null_count = 0

        for i, row in enumerate(rows):
            pps = int(row["pps_count"])
            if pps in recovery_boundaries:
                prev_phase_offset = prev_ppb = prev_ns_count = None
                continue

            gnss_between = _frag_int(row, f"{prefix}_gnss_ns_between_edges")
            residual = _frag_int(row, f"{prefix}_second_residual_ns")
            phase_offset = _frag_int(row, f"{prefix}_phase_offset_ns")
            ppb = _frag_float(row, f"{prefix}_ppb")
            ns_count = _frag_int(row, f"{prefix}_ns_count_at_edge")
            window_err = _frag_int(row, f"{prefix}_window_error_ns")
            zero_est = _frag(row, f"{prefix}_zero_established")

            if not zero_est:
                prev_phase_offset = prev_ppb = prev_ns_count = None
                continue

            if gnss_between is not None and gnss_between > 0:
                valid_seconds += 1
                gnss_between_stats.update(float(gnss_between))
                deviation = gnss_between - NS_PER_SECOND

                if abs(deviation) > OCXO_SECOND_ALARM_NS:
                    isr_raw = _frag_int(row, f"{prefix}_diag_dwt_isr_entry_raw")
                    shadow = _frag_int(row, f"{prefix}_diag_shadow_dwt")
                    s2i = (isr_raw - shadow) if (isr_raw is not None and shadow is not None) else None

                    alarm_records.append({
                        "pps": pps,
                        "deviation_ns": deviation,
                        "gnss_between": gnss_between,
                        "residual": residual,
                        "window_err": window_err,
                        "phase_offset": phase_offset,
                        "ppb": ppb,
                        "prespin_fired": _frag(row, f"{prefix}_diag_prespin_fired"),
                        "prespin_timed_out": _frag(row, f"{prefix}_diag_prespin_timed_out"),
                        "prespin_active": _frag(row, f"{prefix}_diag_prespin_active"),
                        "approach_cycles": _frag_int(row, f"{prefix}_diag_approach_cycles"),
                        "shadow_to_isr": s2i,
                        "correction_cycles": _frag_int(row, f"{prefix}_diag_dwt_event_correction_cycles"),
                        "anomaly_count": _frag_int(row, f"{prefix}_diag_anomaly_count"),
                        "timeout_count": _frag_int(row, f"{prefix}_diag_prespin_timeout_count"),
                        "dwt_at_event": _frag_int(row, f"{prefix}_diag_dwt_at_event_adjusted"),
                        "gnss_ns_at_event": _frag_int(row, f"{prefix}_diag_gnss_ns_at_event"),
                        "counter32": _frag_int(row, f"{prefix}_diag_counter32_at_event"),
                    })
                elif abs(deviation) > OCXO_SECOND_WARN_NS:
                    second_warns.append(f"pps={pps:>7d}  deviation={deviation:+,d} ns")
            elif gnss_between is None:
                null_count += 1

            if residual is not None:
                residual_stats.update(float(residual))

            if window_err is not None:
                window_error_stats.update(float(abs(window_err)))
                if abs(window_err) > OCXO_SECOND_WARN_NS:
                    window_alarms.append(f"pps={pps:>7d}  window_error={window_err:+,d} ns")

            if ns_count is not None and prev_ns_count is not None:
                ns_delta = ns_count - prev_ns_count
                if ns_delta != NS_PER_SECOND:
                    ns_count_alarms.append(
                        f"pps={pps:>7d}  ns_count_delta={ns_delta:,}  (expected {NS_PER_SECOND:,})")

            if phase_offset is not None and prev_phase_offset is not None:
                step = phase_offset - prev_phase_offset
                phase_step_stats.update(float(step))
                if abs(step) > PHASE_STEP_ALARM_NS:
                    phase_step_alarms.append(
                        f"pps={pps:>7d}  phase_step={step:+,d} ns  phase_offset={phase_offset:,}")

            if ppb is not None and prev_ppb is not None:
                ppb_step = ppb - prev_ppb
                if abs(ppb_step) > PPB_STEP_ALARM:
                    ppb_step_alarms.append(
                        f"pps={pps:>7d}  ppb_step={ppb_step:+.3f}  ppb={ppb:+.3f}  prev_ppb={prev_ppb:+.3f}")

            prev_phase_offset = phase_offset
            prev_ppb = ppb
            prev_ns_count = ns_count

        # ── Report ──

        print(f"\n  Valid seconds:     {valid_seconds:,}")
        print(f"  Null records:      {null_count}")

        print(f"\n  GNSS-measured OCXO second (gnss_ns_between_edges):")
        print(f"    {gnss_between_stats.summary()}")

        if alarm_records:
            print(f"\n    ALARM: {len(alarm_records)} seconds with "
                  f"|deviation| > {OCXO_SECOND_ALARM_NS:,} ns")
            anomalies.append(f"{ocxo_label}: {len(alarm_records)} ALARM seconds "
                             f"(|deviation| > {OCXO_SECOND_ALARM_NS:,} ns)")

            # ── Directionality ──
            deviations = [r["deviation_ns"] for r in alarm_records]
            short_count = sum(1 for d in deviations if d < 0)
            long_count = sum(1 for d in deviations if d > 0)

            print(f"\n    Directionality:")
            print(f"      Short (measured second < 1e9):  {short_count}")
            print(f"      Long  (measured second > 1e9):  {long_count}")
            if short_count > 0 and long_count == 0:
                print(f"      ** ALL deviations are SHORT — consistent bias fingerprint **")
            elif long_count > 0 and short_count == 0:
                print(f"      ** ALL deviations are LONG — consistent bias fingerprint **")
            print(f"      Min deviation:  {min(deviations):+,d} ns")
            print(f"      Max deviation:  {max(deviations):+,d} ns")
            mean_dev = sum(deviations) / len(deviations)
            print(f"      Mean deviation: {mean_dev:+,.0f} ns")

            # ── Deviation magnitude histogram ──
            bins = [0, 0, 0, 0, 0]
            for d in deviations:
                ad = abs(d)
                if   ad < 20000: bins[0] += 1
                elif ad < 30000: bins[1] += 1
                elif ad < 40000: bins[2] += 1
                elif ad < 50000: bins[3] += 1
                else:            bins[4] += 1

            print(f"\n    Deviation magnitude distribution:")
            print(f"      10-20 \u00b5s:  {bins[0]:>4d}  {'#' * min(40, bins[0])}")
            print(f"      20-30 \u00b5s:  {bins[1]:>4d}  {'#' * min(40, bins[1])}")
            print(f"      30-40 \u00b5s:  {bins[2]:>4d}  {'#' * min(40, bins[2])}")
            print(f"      40-50 \u00b5s:  {bins[3]:>4d}  {'#' * min(40, bins[3])}")
            print(f"      > 50 \u00b5s:   {bins[4]:>4d}  {'#' * min(40, bins[4])}")

            # ── Prespin correlation ──
            prespin_timed_out_count = sum(1 for r in alarm_records if r.get("prespin_timed_out") is True)
            prespin_fired_count = sum(1 for r in alarm_records if r.get("prespin_fired") is True)
            prespin_not_fired = sum(1 for r in alarm_records
                                    if r.get("prespin_fired") is False and r.get("prespin_timed_out") is False)

            print(f"\n    Prespin correlation (at alarm seconds):")
            print(f"      Prespin fired (normal):     {prespin_fired_count}")
            print(f"      Prespin timed out:           {prespin_timed_out_count}")
            print(f"      Prespin neither fired/TO:    {prespin_not_fired}")

            if prespin_timed_out_count == len(alarm_records):
                print(f"      ** PERFECT CORRELATION: every alarm second had a prespin timeout **")
                print(f"      ** Root cause is almost certainly the TDC fallback path **")
            elif prespin_timed_out_count > 0:
                pct = prespin_timed_out_count / len(alarm_records) * 100
                print(f"      Timeout rate at alarms: {pct:.0f}%  (vs campaign-wide rate below)")
            else:
                print(f"      No timeouts at alarm seconds — prespin is NOT the cause")

            total_timeouts = _frag_int(rows[-1], f"{prefix}_diag_prespin_timeout_count") or 0
            total_arms = _frag_int(rows[-1], f"{prefix}_diag_prespin_arm_count") or 0
            if total_arms > 0:
                campaign_to_rate = total_timeouts / total_arms * 100
                alarm_to_rate = prespin_timed_out_count / len(alarm_records) * 100
                print(f"\n      Campaign-wide timeout rate:  {campaign_to_rate:.1f}%  "
                      f"({total_timeouts}/{total_arms})")
                print(f"      Alarm-second timeout rate:   {alarm_to_rate:.1f}%  "
                      f"({prespin_timed_out_count}/{len(alarm_records)})")
                if alarm_to_rate > campaign_to_rate * 3 and prespin_timed_out_count > 2:
                    print(f"      ** ENRICHED: alarms are {alarm_to_rate/campaign_to_rate:.1f}x more likely "
                          f"to have prespin timeouts **")

            # ── Temporal clustering ──
            alarm_pps_list = sorted(r["pps"] for r in alarm_records)
            if len(alarm_pps_list) >= 2:
                gaps = [alarm_pps_list[i+1] - alarm_pps_list[i] for i in range(len(alarm_pps_list) - 1)]
                mean_gap = sum(gaps) / len(gaps)
                min_gap = min(gaps)
                max_gap = max(gaps)

                print(f"\n    Temporal distribution:")
                print(f"      Alarm seconds span:    pps {alarm_pps_list[0]} -> {alarm_pps_list[-1]}")
                print(f"      Mean gap between:      {mean_gap:.0f} seconds")
                print(f"      Min gap:               {min_gap} seconds")
                print(f"      Max gap:               {max_gap} seconds")
                close_pairs = sum(1 for g in gaps if g < 30)
                if close_pairs > 0:
                    print(f"      Close pairs (<30s):    {close_pairs}  — suggests bursty pathology")

            # ── Forensic table ──
            show_n = min(15, len(alarm_records))
            print(f"\n    Forensic detail (first {show_n}):")
            print(f"    {'PPS':>7s}  {'DEV_NS':>10s}  {'FIRED':>5s}  {'T/O':>3s}  "
                  f"{'APPR':>6s}  {'S->ISR':>6s}  {'CORR':>4s}  {'TO_CNT':>6s}")
            print(f"    {'---':>7s}  {'---':>10s}  {'---':>5s}  {'---':>3s}  "
                  f"{'---':>6s}  {'---':>6s}  {'---':>4s}  {'---':>6s}")
            for r in alarm_records[:show_n]:
                fired = "Y" if r.get("prespin_fired") else "N"
                to = "Y" if r.get("prespin_timed_out") else "N"
                appr = r.get("approach_cycles")
                s2i = r.get("shadow_to_isr")
                corr = r.get("correction_cycles")
                to_cnt = r.get("timeout_count")
                print(f"    {r['pps']:>7d}  {r['deviation_ns']:>+10,d}  "
                      f"{fired:>5s}  {to:>3s}  "
                      f"{str(appr) if appr is not None else '---':>6s}  "
                      f"{str(s2i) if s2i is not None else '---':>6s}  "
                      f"{str(corr) if corr is not None else '---':>4s}  "
                      f"{str(to_cnt) if to_cnt is not None else '---':>6s}")
            if len(alarm_records) > show_n:
                print(f"    ... and {len(alarm_records) - show_n} more")

        if second_warns:
            print(f"\n    WARN: {len(second_warns)} seconds with "
                  f"|deviation| > {OCXO_SECOND_WARN_NS} ns (of {valid_seconds:,} total)")

        print(f"\n  Per-second residual (1e9 - gnss_ns_between_edges):")
        print(f"    {residual_stats.summary()}")

        print(f"\n  Window error (firmware viability check):")
        print(f"    {window_error_stats.summary()}")
        if window_alarms:
            print(f"    WARN: {len(window_alarms)} seconds with |window_error| > {OCXO_SECOND_WARN_NS} ns")
            for msg in window_alarms[:10]: print(f"      {msg}")
            if len(window_alarms) > 10: print(f"      ... and {len(window_alarms) - 10} more")

        if ns_count_alarms:
            print(f"\n    ALARM: {len(ns_count_alarms)} seconds where "
                  f"ns_count_at_edge did NOT advance by exactly 1e9:")
            for msg in ns_count_alarms[:20]: print(f"      {msg}")
            if len(ns_count_alarms) > 20: print(f"      ... and {len(ns_count_alarms) - 20} more")
            anomalies.append(f"{ocxo_label}: {len(ns_count_alarms)} ns_count_at_edge anomalies")
        else:
            print(f"\n    ns_count_at_edge: OK — every delta is exactly 1,000,000,000")

        print(f"\n  Phase offset per-second step:")
        print(f"    {phase_step_stats.summary()}")
        if phase_step_alarms:
            print(f"\n    ALARM: {len(phase_step_alarms)} phase steps > {PHASE_STEP_ALARM_NS:,} ns:")
            for msg in phase_step_alarms[:20]: print(f"      {msg}")
            anomalies.append(f"{ocxo_label}: {len(phase_step_alarms)} phase offset step alarms")

        if ppb_step_alarms:
            print(f"\n    ALARM: {len(ppb_step_alarms)} ppb steps > {PPB_STEP_ALARM} ppb:")
            for msg in ppb_step_alarms[:20]: print(f"      {msg}")
            anomalies.append(f"{ocxo_label}: {len(ppb_step_alarms)} ppb step alarms")
        else:
            print(f"\n    PPB trajectory: OK — no steps > {PPB_STEP_ALARM} ppb")

        ocxo_issues = [a for a in anomalies if a.startswith(ocxo_label)]
        print(f"\n  {ocxo_label} VERDICT: {'PATHOLOGY DETECTED' if ocxo_issues else 'CLEAN'}")

    return anomalies


def analyze_ocxo_comparison(rows, recovery_boundaries):
    anomalies = []
    print()
    print("-" * 78)
    print("OCXO1 vs OCXO2 COMPARATIVE")
    print("-" * 78)

    divergence_stats = WelfordStats()
    large_divergences = []

    for row in rows:
        pps = int(row["pps_count"])
        if pps in recovery_boundaries: continue
        r1 = _frag_int(row, "ocxo1_second_residual_ns")
        r2 = _frag_int(row, "ocxo2_second_residual_ns")
        if r1 is None or r2 is None: continue
        diff = r1 - r2
        divergence_stats.update(float(diff))
        if abs(diff) > OCXO_SECOND_ALARM_NS:
            large_divergences.append(
                f"pps={pps:>7d}  ocxo1_res={r1:+,d}  ocxo2_res={r2:+,d}  diff={diff:+,d}")

    print(f"\n  Per-second residual difference (OCXO1 - OCXO2):")
    print(f"    {divergence_stats.summary()}")
    if large_divergences:
        print(f"\n    ALARM: {len(large_divergences)} seconds with "
              f"|OCXO1 - OCXO2| > {OCXO_SECOND_ALARM_NS:,} ns:")
        for msg in large_divergences[:10]: print(f"      {msg}")
        if len(large_divergences) > 10: print(f"      ... and {len(large_divergences) - 10} more")
        anomalies.append(f"OCXO divergence: {len(large_divergences)} seconds with "
                         f"|OCXO1 - OCXO2| > {OCXO_SECOND_ALARM_NS:,} ns")
    else:
        print(f"\n    Verdict: OK — both OCXOs track within {OCXO_SECOND_ALARM_NS:,} ns")
    return anomalies


def analyze_welford_contamination(rows, recovery_boundaries):
    anomalies = []
    print()
    print("-" * 78)
    print("WELFORD CONTAMINATION CHECK")
    print("-" * 78)

    for label, prefix in [("DWT", "dwt"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        stddev_values = []
        for row in rows:
            pps = int(row["pps_count"])
            sd = _frag_float(row, f"{prefix}_welford_stddev")
            if sd is not None: stddev_values.append((pps, sd))
        if not stddev_values:
            print(f"\n  [{label}] No Welford data")
            continue
        first_pps, first_sd = stddev_values[0]
        final_pps, final_sd = stddev_values[-1]
        max_pps, max_sd = max(stddev_values, key=lambda x: x[1])
        unit = "cycles" if label == "DWT" else "ns"
        print(f"\n  [{label}] Welford stddev trajectory ({unit}):")
        print(f"    First (pps={first_pps}):  {first_sd:.3f}")
        print(f"    Final (pps={final_pps}):  {final_sd:.3f}")
        print(f"    Peak  (pps={max_pps}):  {max_sd:.3f}")

        jumps = []
        for i in range(1, len(stddev_values)):
            pp, ps = stddev_values[i - 1]
            cp, cs = stddev_values[i]
            if cp in recovery_boundaries: continue
            if cs > ps * 2.0 and cs > 10.0:
                jumps.append(f"pps={cp:>7d}  stddev {ps:.3f} -> {cs:.3f}  (x{cs/ps:.1f})")
        if jumps:
            print(f"    WARN: {len(jumps)} sudden stddev jumps (>2x):")
            for msg in jumps[:10]: print(f"      {msg}")
            if len(jumps) > 10: print(f"      ... and {len(jumps) - 10} more")

        threshold = 200.0 if label == "DWT" else WELFORD_STDDEV_ALARM
        if final_sd > threshold:
            print(f"    ALARM: final stddev {final_sd:.3f} > {threshold} — Welford is contaminated")
            anomalies.append(f"{label}: Welford stddev contaminated ({final_sd:.3f} {unit})")
        else:
            print(f"    Verdict: OK")
    return anomalies


def analyze_ppb_sanity(rows, recovery_boundaries):
    anomalies = []
    print()
    print("-" * 78)
    print(f"PPB SANITY CHECK (threshold: \u00b1{PPB_ABSOLUTE_THRESHOLD:,} ppb)")
    print("-" * 78)

    for label, prefix in [("DWT", "dwt"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        ppb_stats = WelfordStats()
        violations = []
        for row in rows:
            pps = int(row["pps_count"])
            ppb = _frag_float(row, f"{prefix}_ppb")
            if ppb is None: continue
            ppb_stats.update(ppb)
            if abs(ppb) > PPB_ABSOLUTE_THRESHOLD:
                violations.append(f"pps={pps:>7d}  ppb={ppb:+,.1f}")
        print(f"\n  [{label}] Cumulative PPB:")
        print(f"    {ppb_stats.summary()}")
        if violations:
            print(f"    WARN: {len(violations)} records exceed \u00b1{PPB_ABSOLUTE_THRESHOLD:,} ppb")
            for msg in violations[:5]: print(f"      {msg}")
            if len(violations) > 5: print(f"      ... and {len(violations) - 5} more")
            anomalies.append(f"{label}: {len(violations)} records with |ppb| > {PPB_ABSOLUTE_THRESHOLD:,}")
        else:
            print(f"    Verdict: OK")
    return anomalies


def analyze_servo(rows):
    print()
    print("-" * 78)
    print("OCXO SERVO & DAC STATE")
    print("-" * 78)
    servo = _frag(rows[-1], "calibrate_ocxo", "?")
    print(f"\n  Servo mode: {servo}")
    for label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        dac = _frag_float(rows[-1], f"{prefix}_dac")
        dac_mean = _frag_float(rows[-1], f"{prefix}_dac_welford_mean")
        dac_sd = _frag_float(rows[-1], f"{prefix}_dac_welford_stddev")
        dac_n = _frag_int(rows[-1], f"{prefix}_dac_welford_n")
        dac_se = _frag_float(rows[-1], f"{prefix}_dac_welford_stderr")
        print(f"\n  [{label}]")
        if dac is not None: print(f"    Current DAC:     {dac:.6f}")
        if dac_mean is not None:
            print(f"    DAC Welford:     mean={dac_mean:.6f}  sd={dac_sd:.6f}  n={dac_n}  se={dac_se:.6f}")
        first_dac = _frag_float(rows[0], f"{prefix}_dac")
        if first_dac is not None and dac is not None:
            print(f"    DAC drift:       {first_dac:.6f} -> {dac:.6f}  (\u0394={dac - first_dac:+.6f})")


def analyze_prespin(rows):
    print()
    print("-" * 78)
    print("PRESPIN DIAGNOSTICS")
    print("-" * 78)
    for label, prefix in [("PPS", "pps"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        arm = _frag_int(rows[-1], f"{prefix}_diag_prespin_arm_count")
        complete = _frag_int(rows[-1], f"{prefix}_diag_prespin_complete_count")
        timeout = _frag_int(rows[-1], f"{prefix}_diag_prespin_timeout_count")
        anomaly = _frag_int(rows[-1], f"{prefix}_diag_anomaly_count")
        if arm is None:
            print(f"\n  [{label}] No prespin data")
            continue
        hit_rate = (complete / arm * 100) if arm > 0 else 0
        print(f"\n  [{label}]")
        print(f"    Armed:     {arm:,}")
        print(f"    Complete:  {complete:,}  ({hit_rate:.1f}%)")
        print(f"    Timeouts:  {timeout:,}")
        print(f"    Anomalies: {anomaly:,}")


def print_environment(rows):
    print()
    print("-" * 78)
    print("ENVIRONMENT (final record)")
    print("-" * 78)
    env = rows[-1].get("environment", {})
    if not env:
        print("  No environment data")
        return
    for key, label, unit in [
        ("ambient_temp_c", "Ambient", "\u00b0C"), ("teensy_temp_c", "Teensy", "\u00b0C"),
        ("gnss_temp_c", "GNSS", "\u00b0C"), ("pressure_hpa", "Pressure", "hPa"),
        ("humidity_pct", "Humidity", "%"),
    ]:
        v = env.get(key)
        if v is not None: print(f"  {label + ':':13s} {v:.1f} {unit}")
    batt = env.get("battery", {})
    if batt:
        pct = batt.get("remaining_pct")
        tte = batt.get("tte_minutes")
        health = batt.get("health_state")
        if pct is not None: print(f"  {'Battery:':13s} {pct:.0f}%  TTE={tte:.0f}m  {health}")


def analyze_clock_domains(rows, recovery_boundaries):
    anomalies = []
    print()
    print("-" * 78)
    print("CLOCK DOMAIN MONOTONICITY")
    print("-" * 78)
    for label, key in [("GNSS", "gnss_ns"), ("OCXO1", "ocxo1_ns"), ("OCXO2", "ocxo2_ns")]:
        values = []
        for row in rows:
            pps = int(row["pps_count"])
            v = _frag_int(row, key)
            if v is not None: values.append((pps, v))
        if not values:
            print(f"\n  [{label}] No data")
            continue
        delta_stats = WelfordStats()
        non_monotonic = zero_deltas = 0
        for i in range(1, len(values)):
            pp, pv = values[i - 1]
            cp, cv = values[i]
            if cp in recovery_boundaries: continue
            if cp == pp + 1:
                delta = cv - pv
                delta_stats.update(float(delta))
                if delta == 0: zero_deltas += 1
                elif delta < 0: non_monotonic += 1
            elif cv <= pv and cp not in recovery_boundaries:
                non_monotonic += 1
        print(f"\n  [{label}]")
        print(f"    Range: {values[0][1]:,} -> {values[-1][1]:,}")
        print(f"    Per-second delta: {delta_stats.summary()}")
        issues = []
        if zero_deltas: issues.append(f"{zero_deltas} zero")
        if non_monotonic: issues.append(f"{non_monotonic} non-monotonic")
        if issues:
            print(f"    Verdict: WARN — {', '.join(issues)}")
            anomalies.append(f"{label}: {', '.join(issues)} deltas")
        else:
            print(f"    Verdict: OK")
    return anomalies


def analyze(campaign):
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return
    recovery_boundaries = find_recovery_boundaries(rows)
    anomalies = []
    campaign_seconds = print_campaign_overview(rows, recovery_boundaries)
    anomalies.extend(analyze_pps_continuity(rows, recovery_boundaries))
    anomalies.extend(analyze_clock_domains(rows, recovery_boundaries))
    anomalies.extend(analyze_ocxo_integrity(rows, recovery_boundaries))
    anomalies.extend(analyze_ocxo_comparison(rows, recovery_boundaries))
    anomalies.extend(analyze_welford_contamination(rows, recovery_boundaries))
    anomalies.extend(analyze_ppb_sanity(rows, recovery_boundaries))
    analyze_servo(rows)
    analyze_prespin(rows)
    print_environment(rows)
    print()
    print("=" * 78)
    if anomalies:
        print(f"VERDICT: ANOMALIES FOUND ({len(anomalies)})")
        for a in anomalies: print(f"  * {a}")
    elif recovery_boundaries:
        print(f"VERDICT: CLEAN (with {len(recovery_boundaries)} recovery events)")
    else:
        print("VERDICT: CLEAN — all checks passed")
    print("=" * 78)


def main():
    if len(sys.argv) < 2:
        print("Usage: campaign_analyzer <campaign_name>")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute("""
                    SELECT campaign, active, count(*) as tb_count,
                           min((payload->>'pps_count')::int) as pps_min,
                           max((payload->>'pps_count')::int) as pps_max
                    FROM timebase t JOIN campaigns c USING (campaign)
                    GROUP BY campaign, active
                    ORDER BY max(t.ts) DESC LIMIT 20""")
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'ACTIVE':>7s} {'RECORDS':>8s} {'PPS RANGE':>20s}")
                print(f"  {'-'*20} {'-'*7} {'-'*8} {'-'*20}")
                for r in rows:
                    active = ">" if r["active"] else " "
                    pps_range = f"{r['pps_min']}-{r['pps_max']}"
                    print(f"  {r['campaign']:<20s} {active:>7s} {r['tb_count']:>8d} {pps_range:>20s}")
        except Exception:
            pass
        sys.exit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()