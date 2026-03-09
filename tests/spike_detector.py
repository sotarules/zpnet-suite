"""
ZPNet TIMEBASE Spike Detector — Find correlated residual anomalies

Usage:
    python -m zpnet.tests.spike_detector <campaign_name> [threshold_ns]

Reads all TIMEBASE rows for a campaign and computes per-second deltas
for GNSS, DWT, and OCXO nanosecond values.  Flags records where:

  1. Any single domain has an abnormal delta (> threshold from expected)
  2. Multiple domains spike simultaneously (correlated — software cause)
  3. The GNSS residual is non-zero (should always be exactly 0)

A correlated spike across all three domains with near-identical magnitude
is the signature of a dropped or duplicated PPS edge, a pps_count
discontinuity, or a stale/replayed fragment — NOT a clock anomaly.

Default threshold: 1000 ns (1 µs).  Adjust for sensitivity.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000


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


def analyze(campaign: str, threshold_ns: float = 1000.0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"❌ No TIMEBASE rows for '{campaign}'")
        return

    print("=" * 80)
    print(f"SPIKE DETECTOR — Campaign: {campaign}")
    print(f"  Records: {len(rows)}    Threshold: {threshold_ns:.0f} ns")
    print("=" * 80)
    print()

    # ─── Pass 1: Compute deltas and find anomalies ───

    anomalies = []
    prev = None
    gnss_nonzero = []

    for i, r in enumerate(rows):
        pps = r.get("pps_count", r.get("teensy_pps_count"))
        gnss_ns = r.get("teensy_gnss_ns")
        dwt_ns = r.get("teensy_dwt_ns")
        ocxo_ns = r.get("teensy_ocxo_ns")
        isr_gnss = r.get("isr_residual_gnss")
        system_time = r.get("system_time_utc", "")

        if gnss_ns is None or dwt_ns is None:
            continue

        gnss_time = r.get("gnss_time_utc", "")

        gnss_ns = int(gnss_ns)
        dwt_ns = int(dwt_ns)
        ocxo_ns = int(ocxo_ns) if ocxo_ns is not None else None

        # Check GNSS ISR residual (should always be 0)
        if isr_gnss is not None and int(isr_gnss) != 0:
            gnss_nonzero.append({
                "idx": i,
                "pps_count": pps,
                "isr_residual_gnss": int(isr_gnss),
                "system_time": system_time,
            })

        if prev is not None:
            prev_pps, prev_gnss, prev_dwt, prev_ocxo, prev_system_time, prev_gnss_time = prev

            pps_gap = int(pps) - int(prev_pps)

            d_gnss = gnss_ns - prev_gnss
            d_dwt = dwt_ns - prev_dwt
            d_ocxo = (ocxo_ns - prev_ocxo) if (ocxo_ns is not None and prev_ocxo is not None) else None

            # Expected delta for this pps_gap
            expected = pps_gap * NS_PER_SECOND

            r_gnss = d_gnss - expected
            r_dwt = d_dwt - expected
            r_ocxo = (d_ocxo - expected) if d_ocxo is not None else None

            spikes = []
            if abs(r_gnss) > threshold_ns:
                spikes.append(("GNSS", r_gnss))
            if abs(r_dwt) > threshold_ns:
                spikes.append(("DWT", r_dwt))
            if r_ocxo is not None and abs(r_ocxo) > threshold_ns:
                spikes.append(("OCXO", r_ocxo))

            if spikes or pps_gap != 1:
                anomalies.append({
                    "idx": i,
                    "pps_count": int(pps),
                    "prev_pps_count": int(prev_pps),
                    "pps_gap": pps_gap,
                    "d_gnss": d_gnss,
                    "d_dwt": d_dwt,
                    "d_ocxo": d_ocxo,
                    "r_gnss": r_gnss,
                    "r_dwt": r_dwt,
                    "r_ocxo": r_ocxo,
                    "spikes": spikes,
                    "system_time": system_time,
                    "gnss_time": gnss_time,
                    "prev_system_time": prev_system_time,
                    "prev_gnss_time": prev_gnss_time,
                    "gnss_ns": gnss_ns,
                    "dwt_ns": dwt_ns,
                    "ocxo_ns": ocxo_ns,
                    "prev_gnss_ns": prev_gnss,
                    "prev_dwt_ns": prev_dwt,
                    "prev_ocxo_ns": prev_ocxo,
                })

        prev = (pps, gnss_ns, dwt_ns, ocxo_ns, system_time, gnss_time)

    # ─── Pass 2: Classify anomalies ───

    pps_gaps = [a for a in anomalies if a["pps_gap"] != 1]
    residual_spikes = [a for a in anomalies if a["spikes"]]
    correlated = [a for a in residual_spikes if len(a["spikes"]) >= 2]
    single_domain = [a for a in residual_spikes if len(a["spikes"]) == 1]

    # ─── Report ───

    print("-" * 80)
    print("SUMMARY")
    print("-" * 80)
    print(f"  Total records:              {len(rows)}")
    print(f"  PPS gaps (pps_count jump):  {len(pps_gaps)}")
    print(f"  Residual spikes:            {len(residual_spikes)}")
    print(f"    Correlated (2+ domains):  {len(correlated)}")
    print(f"    Single domain:            {len(single_domain)}")
    print(f"  GNSS ISR residual ≠ 0:      {len(gnss_nonzero)}")
    print()

    if pps_gaps:
        print("-" * 80)
        print("PPS COUNT GAPS (recovery events)")
        print("-" * 80)
        for a in pps_gaps:
            print(f"  pps_count {a['prev_pps_count']} → {a['pps_count']}  "
                  f"(gap={a['pps_gap']})  {a['system_time'][:19]}")
        print()

    if correlated:
        print("-" * 80)
        print("CORRELATED SPIKES (2+ domains — likely software cause)")
        print("-" * 80)
        print()
        for a in correlated:
            domains_hit = ", ".join(f"{name}" for name, _ in a["spikes"])
            print(f"  pps_count={a['pps_count']}  gap={a['pps_gap']}  domains=[{domains_hit}]")
            print(f"    system_time_utc:  {a['prev_system_time']}")
            print(f"                   →  {a['system_time']}")
            print(f"    gnss_time_utc:    {a['prev_gnss_time']}")
            print(f"                   →  {a['gnss_time']}")
            print(f"    GNSS:  delta={a['d_gnss']:,}  residual={a['r_gnss']:+,} ns")
            print(f"    DWT:   delta={a['d_dwt']:,}  residual={a['r_dwt']:+,} ns")
            if a['d_ocxo'] is not None:
                print(f"    OCXO:  delta={a['d_ocxo']:,}  residual={a['r_ocxo']:+,} ns")
            print(f"    gnss_ns:  {a['prev_gnss_ns']:,} → {a['gnss_ns']:,}")
            print(f"    dwt_ns:   {a['prev_dwt_ns']:,} → {a['dwt_ns']:,}")
            if a['ocxo_ns'] is not None:
                print(f"    ocxo_ns:  {a['prev_ocxo_ns']:,} → {a['ocxo_ns']:,}")
            print()

    if single_domain:
        print("-" * 80)
        print("SINGLE-DOMAIN SPIKES (may be genuine clock anomaly)")
        print("-" * 80)
        print()
        for a in single_domain[:20]:  # Cap at 20
            name, resid = a["spikes"][0]
            print(f"  pps_count={a['pps_count']}  {name}: residual={resid:+,} ns  "
                  f"{a['system_time'][:19]}")
        if len(single_domain) > 20:
            print(f"  ... and {len(single_domain) - 20} more")
        print()

    if gnss_nonzero:
        print("-" * 80)
        print("GNSS ISR RESIDUAL ≠ 0 (should be impossible with phase lock)")
        print("-" * 80)
        print()
        for g in gnss_nonzero[:20]:
            print(f"  pps_count={g['pps_count']}  isr_residual_gnss={g['isr_residual_gnss']}  "
                  f"{g['system_time'][:19]}")
        if len(gnss_nonzero) > 20:
            print(f"  ... and {len(gnss_nonzero) - 20} more")
        print()

    # ─── Welford impact analysis ───

    if correlated:
        print("-" * 80)
        print("WELFORD IMPACT ANALYSIS")
        print("-" * 80)
        print()
        print("  How much damage did the correlated spikes do to the running stats?")
        print()

        # Simulate Welford's with and without the spike records
        spike_pps = {a["pps_count"] for a in correlated}

        class MiniWelford:
            def __init__(self):
                self.n = 0
                self.mean = 0.0
                self.m2 = 0.0
            def update(self, x):
                self.n += 1
                d1 = x - self.mean
                self.mean += d1 / self.n
                d2 = x - self.mean
                self.m2 += d1 * d2
            @property
            def stddev(self):
                return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

        gnss_with = MiniWelford()
        gnss_without = MiniWelford()
        dwt_with = MiniWelford()
        dwt_without = MiniWelford()
        ocxo_with = MiniWelford()
        ocxo_without = MiniWelford()

        prev2 = None
        for r in rows:
            pps = r.get("pps_count", r.get("teensy_pps_count"))
            gnss_ns = r.get("teensy_gnss_ns")
            dwt_ns = r.get("teensy_dwt_ns")
            ocxo_ns = r.get("teensy_ocxo_ns")
            if gnss_ns is None or dwt_ns is None:
                continue
            gnss_ns = int(gnss_ns)
            dwt_ns = int(dwt_ns)
            ocxo_ns = int(ocxo_ns) if ocxo_ns is not None else None

            if prev2 is not None:
                pp, pg, pd, po = prev2
                pps_gap = int(pps) - int(pp)
                expected = pps_gap * NS_PER_SECOND

                rg = (gnss_ns - pg) - expected
                rd = (dwt_ns - pd) - expected
                ro = ((ocxo_ns - po) - expected) if (ocxo_ns is not None and po is not None) else None

                gnss_with.update(rg)
                dwt_with.update(rd)
                if ro is not None:
                    ocxo_with.update(ro)

                if int(pps) not in spike_pps:
                    gnss_without.update(rg)
                    dwt_without.update(rd)
                    if ro is not None:
                        ocxo_without.update(ro)

            prev2 = (pps, gnss_ns, dwt_ns, ocxo_ns)

        print(f"  {'':12s} {'WITH spikes':>16s}  {'WITHOUT spikes':>16s}")
        print(f"  {'─' * 12} {'─' * 16}  {'─' * 16}")

        for label, w, wo in [
            ("GNSS mean", gnss_with, gnss_without),
            ("GNSS sd", gnss_with, gnss_without),
            ("DWT mean", dwt_with, dwt_without),
            ("DWT sd", dwt_with, dwt_without),
            ("OCXO mean", ocxo_with, ocxo_without),
            ("OCXO sd", ocxo_with, ocxo_without),
        ]:
            if "mean" in label:
                v1 = f"{w.mean:,.1f}"
                v2 = f"{wo.mean:,.1f}"
            else:
                v1 = f"{w.stddev:,.1f}"
                v2 = f"{wo.stddev:,.1f}"
            print(f"  {label:12s} {v1:>16s}  {v2:>16s}")

        print()
        print(f"  Spike records excluded: {len(spike_pps)}")
        print(f"  If the 'WITHOUT' column shows sane values, the spikes are")
        print(f"  the sole cause of the blown stats — not genuine clock drift.")

    if not anomalies and not gnss_nonzero:
        print("  ✅ No anomalies detected. Campaign is clean.")


def main():
    if len(sys.argv) < 2:
        print("Usage: spike_detector <campaign_name> [threshold_ns]")
        sys.exit(1)

    campaign = sys.argv[1]
    threshold = float(sys.argv[2]) if len(sys.argv) > 2 else 1000.0
    analyze(campaign, threshold)


if __name__ == "__main__":
    main()