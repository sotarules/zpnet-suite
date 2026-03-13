"""
ZPNet DWT Calibration Diagnostic — v4 Teensy-Field Consistency

Traces the continuous DWT calibration and campaign-scoped Welford
statistics through TIMEBASE records to identify contamination sources.

v3 changes:
  - Table 3 now reads firmware fields (dwt_ns, ocxo1_ns, ocxo2_ns)
    directly instead of Pi-computed teensy_* fields.  The Pi-side
    teensy_dwt_ns etc. include rolling accumulator contributions that
    can be inflated on the first record after campaign start.
  - GNSS reference nanoseconds derived from pps_count * 1e9 (each
    pps_count represents one elapsed GNSS second).  The firmware
    gnss_ns field is campaign_seconds * 1e9, but campaign_seconds
    lags by one (incremented AFTER fragment publish), and teensy_gnss_ns
    includes rolling accumulator drift.
  - Firmware version detection improved: checks for dwt_pred_residual
    at top level (v12+) vs stats-only (v11.x).

v2 changes:
  - Reads firmware top-level fields (v12+) first, falls back to
    Pi-side stats dict (v11.x) when firmware fields are absent.
  - Detects firmware version from field presence and reports it.
  - Local Welford excludes records with 2x doubling bug.
  - Flags contaminated records with *2x* marker.

Shows per-second:
  - dwt_delta_raw (actual DWT cycles this second)
  - dwt_cycles_per_pps (what was used for interpolation)
  - dwt_pps_residual (campaign Welford: actual - nominal)
  - pred_residual (prediction tracker residual)
  - pred_stddev (prediction Welford stddev)
  - cal_valid / cal_pps (continuous calibration state, v12+ only)

Usage:
    python -m zpnet.tests.dwt_diag <campaign_name> [limit]
    .zt dwt_diag Calibrate1
    .zt dwt_diag Calibrate1 20
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List

from zpnet.shared.db import open_db

DWT_NOMINAL = 1_008_000_000


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


def get_int(d: dict, key: str, default: int = 0) -> int:
    v = d.get(key)
    if v is None:
        return default
    return int(v)


def get_float(d: dict, key: str, default: float = 0.0) -> float:
    v = d.get(key)
    if v is None:
        return default
    return float(v)


def get_bool(d: dict, key: str, default: bool = False) -> bool:
    v = d.get(key)
    if v is None:
        return default
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.lower() == "true"
    return bool(v)


def stats_get(rec: dict, domain: str, field: str, default=None):
    """Read a value from the Pi-side stats dict: stats.<domain>.<field>"""
    stats = rec.get("stats")
    if not isinstance(stats, dict):
        return default
    dom = stats.get(domain)
    if not isinstance(dom, dict):
        return default
    v = dom.get(field)
    return v if v is not None else default


def detect_firmware_version(rows: List[Dict]) -> str:
    """Detect firmware version from field presence in first record."""
    if not rows:
        return "unknown"
    rec = rows[0]
    if "dwt_cal_valid" in rec:
        return "v12+"
    if "dwt_pred_residual" in rec:
        return "v12 (no cal fields)"
    return "v11.x (stats-only)"


def get_pred(rec: dict, domain: str, field: str, default=0):
    """Get prediction field: firmware top-level first, then stats dict.

    For domain='dwt', checks rec['dwt_pred_<field>'] first.
    For domain='ocxo1', checks rec['ocxo1_pred_<field>'] first.
    Falls back to rec['stats'][domain]['pred_<field>'].
    """
    fw_key = f"{domain}_pred_{field}"
    v = rec.get(fw_key)
    if v is not None:
        return v

    stats_field = f"pred_{field}"
    return stats_get(rec, domain, stats_field, default)


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    fw_ver = detect_firmware_version(rows)
    print(f"Campaign: {campaign}  ({len(rows)} records, firmware: {fw_ver})")
    print()

    # ── Table 1: Per-second DWT forensics ──

    print("── Per-Second DWT Forensics ──")
    print()
    print(
        f"  {'pps':>5s}"
        f"  {'dwt_delta':>14s}"
        f"  {'cycles/pps':>14s}"
        f"  {'delta-c/p':>10s}"
        f"  {'pps_resid':>10s}"
        f"  {'pred_resid':>11s}"
        f"  {'pred_sd':>9s}"
        f"  {'pred_n':>7s}"
        f"  {'cal_valid':>9s}"
        f"  {'cal_pps':>8s}"
    )
    print(
        f"  {'─'*5}"
        f"  {'─'*14}"
        f"  {'─'*14}"
        f"  {'─'*10}"
        f"  {'─'*10}"
        f"  {'─'*11}"
        f"  {'─'*9}"
        f"  {'─'*7}"
        f"  {'─'*9}"
        f"  {'─'*8}"
    )

    # Local Welford to cross-check firmware
    local_n = 0
    local_mean = 0.0
    local_m2 = 0.0
    local_skipped = 0

    count = 0
    prev_pps = None

    for rec in rows:
        pps = get_int(rec, "pps_count", -1)
        if pps < 0:
            continue

        dwt_delta = get_int(rec, "dwt_delta_raw")
        cycles_per_pps = get_int(rec, "dwt_cycles_per_pps")
        pps_residual = get_int(rec, "dwt_pps_residual")

        # Prediction fields: firmware first, stats dict fallback
        pred_residual = int(get_pred(rec, "dwt", "residual", 0))
        pred_stddev = float(get_pred(rec, "dwt", "stddev", 0.0))
        pred_n = int(get_pred(rec, "dwt", "n", 0))

        # Calibration fields (v12+ only, default to absent)
        cal_valid = get_bool(rec, "dwt_cal_valid")
        cal_pps = get_int(rec, "dwt_cal_pps_count")

        # Gap detection
        if prev_pps is not None and pps != prev_pps + 1:
            print(f"  {'':>5s}  --- gap {prev_pps} → {pps} ---")

        # delta between dwt_delta and cycles_per_pps
        delta_diff = dwt_delta - cycles_per_pps if dwt_delta > 0 and cycles_per_pps > 0 else None
        delta_diff_str = f"{delta_diff:>+10d}" if delta_diff is not None else f"{'---':>10s}"

        # Detect 2x doubling bug: cycles_per_pps > 1.5x nominal
        is_doubled = cycles_per_pps > DWT_NOMINAL * 1.5

        # Local Welford — skip contaminated records
        local_residual = dwt_delta - DWT_NOMINAL if dwt_delta > 0 else None
        if local_residual is not None and dwt_delta > 0 and not is_doubled:
            local_n += 1
            d1 = local_residual - local_mean
            local_mean += d1 / local_n
            d2 = local_residual - local_mean
            local_m2 += d1 * d2
        elif is_doubled:
            local_skipped += 1

        flag = " *2x*" if is_doubled else ""

        print(
            f"  {pps:>5d}"
            f"  {dwt_delta:>14,}"
            f"  {cycles_per_pps:>14,}"
            f"  {delta_diff_str}"
            f"  {pps_residual:>+10d}"
            f"  {pred_residual:>+11d}"
            f"  {pred_stddev:>9.2f}"
            f"  {pred_n:>7d}"
            f"  {'YES' if cal_valid else 'NO':>9s}"
            f"  {cal_pps:>8d}"
            f"{flag}"
        )

        prev_pps = pps
        count += 1
        if limit and count >= limit:
            break

    # ── Table 2: OCXO residual forensics ──

    print()
    print("── Per-Second OCXO Residuals ──")
    print()
    print(
        f"  {'pps':>5s}"
        f"  {'ocxo1_delta':>12s}"
        f"  {'o1_pps_res':>10s}"
        f"  {'o1_pred_res':>11s}"
        f"  {'o1_pred_sd':>10s}"
        f"  {'ocxo2_delta':>12s}"
        f"  {'o2_pps_res':>10s}"
        f"  {'o2_pred_res':>11s}"
        f"  {'o2_pred_sd':>10s}"
    )
    print(
        f"  {'─'*5}"
        f"  {'─'*12}"
        f"  {'─'*10}"
        f"  {'─'*11}"
        f"  {'─'*10}"
        f"  {'─'*12}"
        f"  {'─'*10}"
        f"  {'─'*11}"
        f"  {'─'*10}"
    )

    count2 = 0
    prev_pps2 = None

    for rec in rows:
        pps = get_int(rec, "pps_count", -1)
        if pps < 0:
            continue

        o1_delta = get_int(rec, "ocxo1_delta_raw")
        o1_pps_res = get_int(rec, "ocxo1_pps_residual")
        o1_pred_res = int(get_pred(rec, "ocxo1", "residual", 0))
        o1_pred_sd = float(get_pred(rec, "ocxo1", "stddev", 0.0))

        o2_delta = get_int(rec, "ocxo2_delta_raw")
        o2_pps_res = get_int(rec, "ocxo2_pps_residual")
        o2_pred_res = int(get_pred(rec, "ocxo2", "residual", 0))
        o2_pred_sd = float(get_pred(rec, "ocxo2", "stddev", 0.0))

        if prev_pps2 is not None and pps != prev_pps2 + 1:
            print(f"  {'':>5s}  --- gap {prev_pps2} → {pps} ---")

        print(
            f"  {pps:>5d}"
            f"  {o1_delta:>12,}"
            f"  {o1_pps_res:>+10d}"
            f"  {o1_pred_res:>+11d}"
            f"  {o1_pred_sd:>10.2f}"
            f"  {o2_delta:>12,}"
            f"  {o2_pps_res:>+10d}"
            f"  {o2_pred_res:>+11d}"
            f"  {o2_pred_sd:>10.2f}"
        )

        prev_pps2 = pps
        count2 += 1
        if limit and count2 >= limit:
            break

    # ── Table 3: Accumulated ns and tau/ppb ──
    #
    # v4: Use teensy_* fields (Pi-side decorated).  The firmware fragment
    # fields (dwt_ns, ocxo1_ns, etc.) are renamed to teensy_* by the Pi
    # decorator before reaching the database.  Reading the bare names
    # returns 0.
    #
    # The first record after v12.3 campaign start may show inflated
    # teensy_dwt_ns (rolling accumulator includes time from zeroing PPS
    # to measurement PPS).  Flag it but don't suppress — operational
    # consistency trumps diagnostic purity.

    print()
    print("── Accumulated Nanoseconds & PPB ──")
    print()
    print(
        f"  {'pps':>5s}"
        f"  {'gnss_ns':>16s}"
        f"  {'dwt_ns':>16s}"
        f"  {'ocxo1_ns':>16s}"
        f"  {'ocxo2_ns':>16s}"
        f"  {'dwt_ppb':>10s}"
        f"  {'o1_ppb':>10s}"
        f"  {'o2_ppb':>10s}"
    )
    print(
        f"  {'─'*5}"
        f"  {'─'*16}"
        f"  {'─'*16}"
        f"  {'─'*16}"
        f"  {'─'*16}"
        f"  {'─'*10}"
        f"  {'─'*10}"
        f"  {'─'*10}"
    )

    count3 = 0

    for rec in rows:
        pps = get_int(rec, "pps_count", -1)
        if pps < 0:
            continue

        # Pi-decorated accumulated nanoseconds
        gnss_ns = get_int(rec, "teensy_gnss_ns")
        dwt_ns = get_int(rec, "teensy_dwt_ns")
        o1_ns = get_int(rec, "teensy_ocxo1_ns")
        o2_ns = get_int(rec, "teensy_ocxo2_ns")

        if gnss_ns > 0:
            dwt_ppb = ((dwt_ns - gnss_ns) / gnss_ns) * 1e9
            o1_ppb = ((o1_ns - gnss_ns) / gnss_ns) * 1e9
            o2_ppb = ((o2_ns - gnss_ns) / gnss_ns) * 1e9
        else:
            dwt_ppb = o1_ppb = o2_ppb = 0.0

        # Flag first-record inflation from rolling accumulators
        inflated = gnss_ns > 0 and dwt_ns > 0 and (dwt_ns / gnss_ns) > 1.5
        flag3 = " *inflated*" if inflated else ""

        print(
            f"  {pps:>5d}"
            f"  {gnss_ns:>16,}"
            f"  {dwt_ns:>16,}"
            f"  {o1_ns:>16,}"
            f"  {o2_ns:>16,}"
            f"  {dwt_ppb:>+10.3f}"
            f"  {o1_ppb:>+10.3f}"
            f"  {o2_ppb:>+10.3f}"
            f"{flag3}"
        )

        count3 += 1
        if limit and count3 >= limit:
            break

    # ── Summary ──

    print()
    print("── Summary ──")
    print(f"  Records: {len(rows)}")
    print(f"  Firmware: {fw_ver}")
    if local_n >= 2:
        local_sd = math.sqrt(local_m2 / (local_n - 1))
        print(f"  Local DWT residual Welford:  n={local_n}  mean={local_mean:+.1f}  stddev={local_sd:.1f}")
    if local_skipped > 0:
        print(f"  Local Welford excluded {local_skipped} record(s) (2x doubling bug)")
    print()


def main():
    if len(sys.argv) < 2:
        print("Usage: dwt_diag <campaign_name> [limit]")
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