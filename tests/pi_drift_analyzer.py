"""
ZPNet Pi Clock Drift Forensic Analyzer

Usage:
    python -m zpnet.tests.pi_drift_analyzer <campaign_name>
    .zt pi_drift_analyzer Test1

Purpose:
    Diagnose why pi_ns cumulative PPB diverges from the expected ~+7500 ppb.
    The per-second residuals (diag_pi_residual) show healthy +7500 ppb behavior,
    but cumulative pi_ns vs gnss_ns shows ~-200,000 ppb.  This implies a
    one-time offset was baked into pi_ns at START or RECOVER.

Analysis performed:
    1. First-record sanity: is pi_ns ≈ 0 at pps_count=0?
    2. Recovery boundary epoch analysis: does pi_ns jump discontinuously?
    3. Per-second pi_ns delta vs expected (54,000,000 ticks * PI_NS_PER_TICK)
    4. Cumulative drift curve: where does the offset appear?
    5. Reverse-engineer the epoch error from the observed offset
    6. Cross-check: pi_corrected * NS_PER_TICK vs pi_ns at every record
    7. Check if pi_ns was None at critical early records (epoch race)
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Set, Tuple

from zpnet.shared.db import open_db


# ---------------------------------------------------------------------
# Constants (must match clocks core.py)
# ---------------------------------------------------------------------

PI_TIMER_FREQ = 54_000_000
PI_NS_PER_TICK = 1e9 / PI_TIMER_FREQ  # ~18.51851851... ns/tick
NS_PER_SECOND = 1_000_000_000


# ---------------------------------------------------------------------
# Fetch
# ---------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
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
        p["_db_id"] = row["id"]
        p["_db_ts"] = str(row["ts"])
        result.append(p)
    return result


def find_recovery_boundaries(rows: List[Dict[str, Any]]) -> Set[int]:
    boundaries: Set[int] = set()
    for i in range(1, len(rows)):
        prev = int(rows[i - 1]["pps_count"])
        curr = int(rows[i]["pps_count"])
        if curr > prev + 1:
            boundaries.add(curr)
    return boundaries


# ---------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"❌ No TIMEBASE rows for '{campaign}'")
        return

    recovery_boundaries = find_recovery_boundaries(rows)
    total = len(rows)

    print("=" * 78)
    print(f"PI CLOCK DRIFT FORENSIC ANALYSIS: {campaign}")
    print("=" * 78)

    pps_min = int(rows[0]["pps_count"])
    pps_max = int(rows[-1]["pps_count"])
    print(f"\n  Records: {total}   pps_count: {pps_min} -> {pps_max}")
    print(f"  Recovery boundaries: {sorted(recovery_boundaries) if recovery_boundaries else 'none'}")

    # =================================================================
    # 1. FIRST RECORD ANALYSIS
    # =================================================================
    print("\n" + "-" * 78)
    print("1. FIRST RECORD (pps_count=0 or earliest)")
    print("-" * 78)

    r0 = rows[0]
    pps0 = int(r0["pps_count"])
    gnss0 = r0.get("teensy_gnss_ns")
    pi_ns0 = r0.get("pi_ns")
    pi_corr0 = r0.get("pi_corrected")
    pi_raw0 = r0.get("pi_counter")

    print(f"  pps_count:      {pps0}")
    print(f"  teensy_gnss_ns: {gnss0}")
    print(f"  pi_ns:          {pi_ns0}")
    print(f"  pi_corrected:   {pi_corr0}")
    print(f"  pi_counter:     {pi_raw0}")
    print(f"  system_time:    {r0.get('system_time_utc')}")

    if pi_ns0 is not None and int(pi_ns0) != 0 and pps0 == 0:
        print(f"\n  ⚠️  pi_ns at pps_count=0 is {pi_ns0}, NOT 0!")
        print(f"      This means the epoch was wrong at START.")
        print(f"      Offset = {int(pi_ns0)} ns = {int(pi_ns0) / 1e9:.6f} seconds")
    elif pi_ns0 is None and pps0 == 0:
        print(f"\n  ⚠️  pi_ns is None at pps_count=0!")
        print(f"      This means the epoch wasn't set yet when this record was built.")
        print(f"      The processor thread won the race against SET_EPOCH.")

    # =================================================================
    # 2. FIRST FEW RECORDS — look for the offset appearing
    # =================================================================
    print("\n" + "-" * 78)
    print("2. FIRST 20 RECORDS — pi_ns vs gnss_ns offset")
    print("-" * 78)
    print(f"  {'pps':>6s}  {'gnss_ns':>20s}  {'pi_ns':>20s}  {'offset_ns':>14s}  {'offset_s':>10s}  {'ppb':>12s}")
    print(f"  {'─'*6}  {'─'*20}  {'─'*20}  {'─'*14}  {'─'*10}  {'─'*12}")

    for r in rows[:20]:
        pps = int(r["pps_count"])
        gnss = r.get("teensy_gnss_ns")
        pi_ns = r.get("pi_ns")

        if gnss is not None and pi_ns is not None:
            gnss_v = int(gnss)
            pi_v = int(pi_ns)
            offset = pi_v - gnss_v
            offset_s = offset / 1e9
            ppb = ((pi_v - gnss_v) / gnss_v * 1e9) if gnss_v > 0 else 0
            print(f"  {pps:6d}  {gnss_v:20d}  {pi_v:20d}  {offset:+14d}  {offset_s:+10.6f}  {ppb:+12.1f}")
        else:
            pi_str = str(pi_ns) if pi_ns is not None else "None"
            gnss_str = str(gnss) if gnss is not None else "None"
            print(f"  {pps:6d}  {gnss_str:>20s}  {pi_str:>20s}  {'---':>14s}  {'---':>10s}  {'---':>12s}")

    # =================================================================
    # 3. RECOVERY BOUNDARY ANALYSIS
    # =================================================================
    if recovery_boundaries:
        print("\n" + "-" * 78)
        print("3. RECOVERY BOUNDARY ANALYSIS")
        print("-" * 78)

        for boundary_pps in sorted(recovery_boundaries):
            # Find the records just before and at the boundary
            pre_idx = None
            at_idx = None
            for i, r in enumerate(rows):
                p = int(r["pps_count"])
                if p < boundary_pps:
                    pre_idx = i
                if p == boundary_pps:
                    at_idx = i
                    break

            if pre_idx is None or at_idx is None:
                print(f"\n  Recovery at pps_count={boundary_pps}: incomplete data")
                continue

            pre = rows[pre_idx]
            at = rows[at_idx]

            pre_pps = int(pre["pps_count"])
            gap = boundary_pps - pre_pps

            pre_gnss = int(pre.get("teensy_gnss_ns") or 0)
            at_gnss = int(at.get("teensy_gnss_ns") or 0)
            pre_pi = pre.get("pi_ns")
            at_pi = at.get("pi_ns")
            pre_corr = pre.get("pi_corrected")
            at_corr = at.get("pi_corrected")

            print(f"\n  Recovery at pps_count={boundary_pps} (gap={gap} seconds)")
            print(f"    BEFORE (pps={pre_pps}):")
            print(f"      gnss_ns:     {pre_gnss}")
            print(f"      pi_ns:       {pre_pi}")
            print(f"      pi_corrected:{pre_corr}")

            if pre_gnss > 0 and pre_pi is not None:
                pre_offset = int(pre_pi) - pre_gnss
                pre_ppb = (int(pre_pi) - pre_gnss) / pre_gnss * 1e9
                print(f"      offset:      {pre_offset:+d} ns ({pre_ppb:+.1f} ppb)")

            print(f"    AFTER (pps={boundary_pps}):")
            print(f"      gnss_ns:     {at_gnss}")
            print(f"      pi_ns:       {at_pi}")
            print(f"      pi_corrected:{at_corr}")

            if at_gnss > 0 and at_pi is not None:
                at_offset = int(at_pi) - at_gnss
                at_ppb = (int(at_pi) - at_gnss) / at_gnss * 1e9
                print(f"      offset:      {at_offset:+d} ns ({at_ppb:+.1f} ppb)")

                if pre_pi is not None:
                    offset_jump = at_offset - (int(pre_pi) - pre_gnss)
                    print(f"      offset JUMP: {offset_jump:+d} ns ({offset_jump / 1e9:+.6f} s)")
                    if abs(offset_jump) > 100_000_000:  # > 100ms
                        print(f"      ⚠️  LARGE OFFSET JUMP — likely epoch error at recovery!")

            if at_pi is None:
                print(f"      ⚠️  pi_ns is None at recovery boundary!")

            # Show first few records after boundary
            print(f"    First 5 records after recovery:")
            count = 0
            for r in rows[at_idx:at_idx + 5]:
                p = int(r["pps_count"])
                g = int(r.get("teensy_gnss_ns") or 0)
                pi = r.get("pi_ns")
                if g > 0 and pi is not None:
                    off = int(pi) - g
                    ppb = (int(pi) - g) / g * 1e9
                    print(f"      pps={p}  gnss={g}  pi_ns={pi}  offset={off:+d} ({ppb:+.1f} ppb)")
                else:
                    print(f"      pps={p}  gnss={g}  pi_ns={pi}")
                count += 1

    # =================================================================
    # 4. CUMULATIVE DRIFT CURVE (sampled)
    # =================================================================
    print("\n" + "-" * 78)
    print("4. CUMULATIVE DRIFT CURVE (pi_ns - gnss_ns)")
    print("-" * 78)

    # Sample at regular intervals
    sample_interval = max(1, total // 40)
    samples = []
    for i in range(0, total, sample_interval):
        r = rows[i]
        pps = int(r["pps_count"])
        gnss = r.get("teensy_gnss_ns")
        pi_ns = r.get("pi_ns")
        if gnss is not None and pi_ns is not None and int(gnss) > 0:
            gnss_v = int(gnss)
            pi_v = int(pi_ns)
            offset_ns = pi_v - gnss_v
            ppb = (pi_v - gnss_v) / gnss_v * 1e9
            samples.append((pps, offset_ns, ppb))

    # Always include last record
    r_last = rows[-1]
    gnss_last = r_last.get("teensy_gnss_ns")
    pi_last = r_last.get("pi_ns")
    if gnss_last is not None and pi_last is not None:
        pps_l = int(r_last["pps_count"])
        gnss_lv = int(gnss_last)
        pi_lv = int(pi_last)
        off_l = pi_lv - gnss_lv
        ppb_l = (pi_lv - gnss_lv) / gnss_lv * 1e9
        if not samples or samples[-1][0] != pps_l:
            samples.append((pps_l, off_l, ppb_l))

    if samples:
        print(f"  {'pps':>8s}  {'offset_ns':>16s}  {'offset_s':>12s}  {'cum_ppb':>12s}")
        print(f"  {'─'*8}  {'─'*16}  {'─'*12}  {'─'*12}")
        for pps, offset, ppb in samples:
            print(f"  {pps:8d}  {offset:+16d}  {offset/1e9:+12.6f}  {ppb:+12.1f}")

    # =================================================================
    # 5. PER-SECOND PI DELTA ANALYSIS
    # =================================================================
    print("\n" + "-" * 78)
    print("5. PER-SECOND pi_ns DELTA ANALYSIS")
    print("-" * 78)

    expected_pi_ns_per_sec = NS_PER_SECOND  # ideal: 1 billion ns/s

    delta_stats_ns = []
    anomalous_deltas = []

    prev_r = None
    for r in rows:
        if prev_r is not None:
            prev_pps = int(prev_r["pps_count"])
            curr_pps = int(r["pps_count"])
            if curr_pps == prev_pps + 1:  # adjacent only
                prev_pi = prev_r.get("pi_ns")
                curr_pi = r.get("pi_ns")
                if prev_pi is not None and curr_pi is not None:
                    delta = int(curr_pi) - int(prev_pi)
                    residual = delta - expected_pi_ns_per_sec
                    delta_stats_ns.append(residual)
                    if abs(residual) > 100_000:  # > 100 µs off
                        anomalous_deltas.append((curr_pps, delta, residual))
        prev_r = r

    if delta_stats_ns:
        n = len(delta_stats_ns)
        mean_r = sum(delta_stats_ns) / n
        var_r = sum((x - mean_r) ** 2 for x in delta_stats_ns) / (n - 1) if n >= 2 else 0
        sd_r = math.sqrt(var_r)
        min_r = min(delta_stats_ns)
        max_r = max(delta_stats_ns)

        print(f"  Samples: {n}")
        print(f"  Residual (pi_ns delta - 1e9):")
        print(f"    mean:   {mean_r:+.3f} ns")
        print(f"    stddev: {sd_r:.3f} ns")
        print(f"    min:    {min_r:+.3f} ns")
        print(f"    max:    {max_r:+.3f} ns")
        print(f"    Implied rate: {mean_r / 1e9 * 1e9:+.3f} ppb")

        if anomalous_deltas:
            print(f"\n  ⚠️  {len(anomalous_deltas)} anomalous deltas (|residual| > 100 µs):")
            for pps, delta, residual in anomalous_deltas[:20]:
                print(f"    pps={pps}  delta={delta}  residual={residual:+d} ns ({residual/1e6:+.3f} ms)")
    else:
        print("  No adjacent pi_ns pairs found")

    # =================================================================
    # 6. EPOCH REVERSE-ENGINEERING
    # =================================================================
    print("\n" + "-" * 78)
    print("6. EPOCH REVERSE-ENGINEERING")
    print("-" * 78)
    print("  pi_ns = (pi_corrected - epoch) * NS_PER_TICK")
    print(f"  NS_PER_TICK = {PI_NS_PER_TICK:.15f}")

    # For each record that has both pi_ns and pi_corrected, compute implied epoch
    epochs_computed = []
    for r in rows[:20]:
        pi_ns = r.get("pi_ns")
        pi_corr = r.get("pi_corrected")
        pps = int(r["pps_count"])

        if pi_ns is not None and pi_corr is not None:
            pi_ns_v = int(pi_ns)
            pi_corr_v = int(pi_corr)
            # pi_ns = (pi_corrected - epoch) * NS_PER_TICK
            # epoch = pi_corrected - (pi_ns / NS_PER_TICK)
            implied_ticks_elapsed = pi_ns_v / PI_NS_PER_TICK
            implied_epoch = pi_corr_v - implied_ticks_elapsed
            epochs_computed.append((pps, implied_epoch, pi_corr_v, pi_ns_v))

    if epochs_computed:
        print(f"\n  {'pps':>6s}  {'implied_epoch':>22s}  {'pi_corrected':>20s}  {'pi_ns':>20s}")
        print(f"  {'─'*6}  {'─'*22}  {'─'*20}  {'─'*20}")
        for pps, epoch, corr, ns in epochs_computed:
            print(f"  {pps:6d}  {epoch:22.1f}  {corr:20d}  {ns:20d}")

        # Check epoch consistency
        epoch_vals = [e for _, e, _, _ in epochs_computed]
        if len(epoch_vals) >= 2:
            epoch_spread = max(epoch_vals) - min(epoch_vals)
            print(f"\n  Epoch spread across first 20 records: {epoch_spread:.1f} ticks")
            print(f"  Epoch spread in ns: {epoch_spread * PI_NS_PER_TICK:.1f}")
            if epoch_spread > 100:
                print(f"  ⚠️  Epoch is NOT consistent — PITIMER changed epoch mid-stream!")

    # Also check at recovery boundaries
    if recovery_boundaries:
        print(f"\n  Epoch at recovery boundaries:")
        for boundary_pps in sorted(recovery_boundaries):
            for r in rows:
                if int(r["pps_count"]) == boundary_pps:
                    pi_ns = r.get("pi_ns")
                    pi_corr = r.get("pi_corrected")
                    if pi_ns is not None and pi_corr is not None:
                        elapsed = int(pi_ns) / PI_NS_PER_TICK
                        epoch = int(pi_corr) - elapsed
                        print(f"    pps={boundary_pps}: implied_epoch={epoch:.1f}")
                    else:
                        print(f"    pps={boundary_pps}: pi_ns={pi_ns} pi_corr={pi_corr}")
                    break

    # =================================================================
    # 7. WHAT THE OFFSET SHOULD BE (from per-second residuals)
    # =================================================================
    print("\n" + "-" * 78)
    print("7. EXPECTED vs ACTUAL CUMULATIVE OFFSET")
    print("-" * 78)

    if delta_stats_ns and samples:
        last_pps, last_offset, last_ppb = samples[-1]
        expected_offset = mean_r * last_pps  # cumulative expected from per-second rate
        actual_offset = last_offset

        print(f"  Per-second mean residual:  {mean_r:+.3f} ns/s")
        print(f"  Campaign length:           {last_pps} seconds")
        print(f"  Expected cumulative offset:{expected_offset:+.0f} ns ({expected_offset/1e9:+.6f} s)")
        print(f"  Actual cumulative offset:  {actual_offset:+d} ns ({actual_offset/1e9:+.6f} s)")
        print(f"  Discrepancy:               {actual_offset - expected_offset:+.0f} ns ({(actual_offset - expected_offset)/1e9:+.6f} s)")
        print()
        print(f"  This discrepancy is the epoch error — a one-time offset baked into pi_ns")
        print(f"  at START or RECOVER that shifted all subsequent values.")

        discrepancy_ticks = (actual_offset - expected_offset) / PI_NS_PER_TICK
        print(f"  Discrepancy in Pi ticks:   {discrepancy_ticks:+.0f}")
        print(f"  Discrepancy in PPS edges:  {discrepancy_ticks / PI_TIMER_FREQ:+.3f} seconds")

    # =================================================================
    # 8. pi_corrected DELTA ANALYSIS (tick-level, independent of epoch)
    # =================================================================
    print("\n" + "-" * 78)
    print("8. pi_corrected PER-SECOND DELTA (independent of epoch)")
    print("-" * 78)

    corr_residuals = []
    prev_r = None
    for r in rows:
        if prev_r is not None:
            prev_pps = int(prev_r["pps_count"])
            curr_pps = int(r["pps_count"])
            if curr_pps == prev_pps + 1:
                prev_c = prev_r.get("pi_corrected")
                curr_c = r.get("pi_corrected")
                if prev_c is not None and curr_c is not None:
                    delta = int(curr_c) - int(prev_c)
                    residual = delta - PI_TIMER_FREQ
                    corr_residuals.append(residual)
        prev_r = r

    if corr_residuals:
        n = len(corr_residuals)
        mean_c = sum(corr_residuals) / n
        var_c = sum((x - mean_c) ** 2 for x in corr_residuals) / (n - 1) if n >= 2 else 0
        sd_c = math.sqrt(var_c)

        print(f"  Samples: {n}")
        print(f"  pi_corrected delta - 54,000,000:")
        print(f"    mean:   {mean_c:+.3f} ticks/s")
        print(f"    stddev: {sd_c:.3f}")
        print(f"    Implied PPB: {mean_c / PI_TIMER_FREQ * 1e9:+.3f}")
        print(f"    In ns/s:     {mean_c * PI_NS_PER_TICK:+.3f}")

    print("\n" + "=" * 78)
    print("END OF ANALYSIS")
    print("=" * 78)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: pi_drift_analyzer <campaign_name>")
        sys.exit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()