"""
ZPNet DWT Cycle Report — Raw per-second cycle counts with diagnostics

Usage:
    python -m zpnet.tests.cycle_report <campaign_name> [limit]
    .zt cycle_report Shakeout1
    .zt cycle_report Shakeout1 50

Shows one row per PPS second with:
  - 64-bit cycle count at start and end of second
  - Actual cycles, ideal, residual
  - Whether shadow or ISR fallback was used
  - dispatch_delta_ns (shadow-to-ISR gap)
  - diag_fine_was_late flag
  - Pair sum with previous second (should be ~2× residual mean)

Flags anomalous seconds where the residual deviates significantly
from the local trend, and shows pair analysis to determine if
cycles are merely shifted between seconds (boundary jitter) or
genuinely lost/gained.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db

DWT_FREQ_HZ = 1_008_000_000
DWT_NS_PER_CYCLE = 125.0 / 126.0


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


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    # ── Extract per-second records ──
    records = []
    for r in rows:
        pps = r.get("pps_count")
        dwt_cycles = r.get("teensy_dwt_cycles")
        isr_cyc = r.get("diag_raw_isr_cyc")
        shadow_cyc = r.get("diag_raw_shadow_cyc")
        late = r.get("diag_fine_was_late") or r.get("diag_teensy_fine_was_late")
        delta_ns = r.get("dispatch_delta_ns") or r.get("diag_teensy_dispatch_delta_ns")

        if pps is None or dwt_cycles is None:
            continue

        records.append({
            "pps": int(pps),
            "dwt_cycles": int(dwt_cycles),
            "isr_cyc": int(isr_cyc) if isr_cyc is not None else None,
            "shadow_cyc": int(shadow_cyc) if shadow_cyc is not None else None,
            "late": bool(late) if late is not None else None,
            "delta_ns": int(delta_ns) if delta_ns is not None else None,
        })

    if len(records) < 2:
        print("Need at least 2 records")
        return

    # ── Compute deltas ──
    deltas = []
    for i in range(1, len(records)):
        prev = records[i - 1]
        curr = records[i]

        actual = curr["dwt_cycles"] - prev["dwt_cycles"]
        residual = actual - DWT_FREQ_HZ

        # Determine source: did we use shadow or ISR fallback?
        source = "?"
        if curr["shadow_cyc"] is not None and curr["isr_cyc"] is not None:
            # If shadow_cyc == isr_cyc, it's ISR fallback
            # If shadow_cyc != isr_cyc, shadow was used (they differ by ~50 cycles)
            isr_minus_shadow = (curr["isr_cyc"] - curr["shadow_cyc"]) & 0xFFFFFFFF
            if isr_minus_shadow > 0x80000000:
                isr_minus_shadow -= 0x100000000
            if 20 < isr_minus_shadow < 200:
                source = "shadow"
            elif isr_minus_shadow == 0:
                source = "ISR"
            else:
                source = f"?({isr_minus_shadow})"

        deltas.append({
            "pps": curr["pps"],
            "start": prev["dwt_cycles"],
            "end": curr["dwt_cycles"],
            "actual": actual,
            "residual": residual,
            "source": source,
            "late": curr["late"],
            "delta_ns": curr["delta_ns"],
        })

    # ── Compute pair sums ──
    for i in range(1, len(deltas)):
        deltas[i]["pair_sum"] = deltas[i]["residual"] + deltas[i - 1]["residual"]
    deltas[0]["pair_sum"] = None

    # ── Compute stats for anomaly flagging ──
    residuals = [d["residual"] for d in deltas]
    residuals_sorted = sorted(residuals)
    median = residuals_sorted[len(residuals_sorted) // 2]

    # Robust stddev from middle 80%
    lo = len(residuals_sorted) // 10
    hi = len(residuals_sorted) - lo
    core = residuals_sorted[lo:hi]
    core_mean = sum(core) / len(core)
    core_var = sum((x - core_mean) ** 2 for x in core) / (len(core) - 1)
    core_std = math.sqrt(core_var)

    anomaly_threshold = 3 * core_std  # flag if > 3σ from core mean

    # ── Print header ──
    print(f"Campaign: {campaign}  ({len(rows)} records, {len(deltas)} deltas)")
    print(f"Median residual: {median:,} cycles ({median * DWT_NS_PER_CYCLE:,.1f} ns)")
    print(f"Core mean: {core_mean:,.1f}  Core σ: {core_std:,.1f}  Anomaly threshold: ±{anomaly_threshold:,.0f}")
    print()

    hdr = (f"  {'pps':>5s}  {'residual':>10s}  {'pair_sum':>10s}  {'src':>7s}"
           f"  {'late':>5s}  {'Δns':>6s}  {'flag':>4s}")
    sep = f"  {'─'*5}  {'─'*10}  {'─'*10}  {'─'*7}  {'─'*5}  {'─'*6}  {'─'*4}"

    print(hdr)
    print(sep)

    count = 0
    anomaly_count = 0
    pair_preserved_count = 0
    pair_broken_count = 0

    for d in deltas:
        pps = d["pps"]
        res = d["residual"]
        ps = d["pair_sum"]

        src = d["source"]
        late = "LATE" if d["late"] else "" if d["late"] is not None else "?"
        dns = f"{d['delta_ns']:>6d}" if d["delta_ns"] is not None else f"{'—':>6}"

        # Flag anomalies
        flag = ""
        is_anomaly = abs(res - core_mean) > anomaly_threshold
        if is_anomaly:
            anomaly_count += 1
            # Check if pair sum is preserved
            if ps is not None:
                expected_pair = 2 * core_mean
                pair_err = abs(ps - expected_pair)
                if pair_err < anomaly_threshold:
                    flag = "↔"  # boundary shift — cycles preserved
                    pair_preserved_count += 1
                else:
                    flag = "⚠"  # genuinely lost or gained
                    pair_broken_count += 1
            else:
                flag = "?"

        ps_str = f"{ps:>+10,}" if ps is not None else f"{'—':>10}"

        print(f"  {pps:>5d}  {res:>+10,}  {ps_str}  {src:>7s}  {late:>5s}  {dns}  {flag:>4s}")

        count += 1
        if limit and count >= limit:
            break

    # ── Summary ──
    print()
    print(f"  {count} rows shown")
    print()
    print("-" * 70)
    print("ANOMALY SUMMARY")
    print("-" * 70)
    print()
    print(f"  Total seconds:       {len(deltas)}")
    print(f"  Normal seconds:      {len(deltas) - anomaly_count} ({(len(deltas) - anomaly_count) / len(deltas) * 100:.1f}%)")
    print(f"  Anomalous seconds:   {anomaly_count} ({anomaly_count / len(deltas) * 100:.1f}%)")
    print()
    if anomaly_count > 0:
        print(f"  Pair-preserved (↔):  {pair_preserved_count}  (boundary shift only — no cycles lost)")
        print(f"  Pair-broken (⚠):     {pair_broken_count}  (genuine cycle loss or gain)")
        print()
        if pair_preserved_count > 0 and pair_broken_count == 0:
            print("  ✅ All anomalies are boundary shifts — the DWT is not losing cycles.")
            print("     The spin loop shadow occasionally lands on the wrong side of")
            print("     a preemption event, shifting ~600 cycles between adjacent seconds.")
            print("     The campaign-total cycle count is exact.")
        elif pair_broken_count > 0:
            print(f"  ⚠️  {pair_broken_count} seconds show genuine cycle discrepancy.")
            print("     Investigate: USB SOF preemption, clock gating, or DWT stall.")

    # ── Source distribution ──
    print()
    print("-" * 70)
    print("SOURCE DISTRIBUTION")
    print("-" * 70)
    print()

    from collections import Counter
    src_counts = Counter(d["source"] for d in deltas)
    for src, cnt in src_counts.most_common():
        pct = cnt / len(deltas) * 100
        print(f"  {src:>10s}: {cnt:>6d} ({pct:5.1f}%)")

    late_counts = Counter()
    for d in deltas:
        if d["late"] is True:
            late_counts["LATE"] += 1
        elif d["late"] is False:
            late_counts["on-time"] += 1
        else:
            late_counts["unknown"] += 1

    print()
    for lbl, cnt in late_counts.most_common():
        pct = cnt / len(deltas) * 100
        print(f"  {lbl:>10s}: {cnt:>6d} ({pct:5.1f}%)")


def main():
    if len(sys.argv) < 2:
        print("Usage: cycle_report <campaign_name> [limit]")
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