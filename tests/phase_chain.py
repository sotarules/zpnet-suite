"""
ZPNet OCXO Phase Chain Report — End-to-end measurement visibility

One row per PPS second showing the complete DWT→GNSS conversion chain
for OCXO1 phase measurement.  Every intermediate value is exposed so
you can trace exactly how the phase offset is derived.

Columns:
    PPS     — pps_count
    GNSS_NS — teensy_gnss_ns (64-bit, PPS anchor in GNSS domain)
    DWT@PPS — dwt_cyccnt_at_pps (32-bit, best-available DWT at PPS edge)
    SHADOW  — ocxo1_phase_shadow_dwt (32-bit, last DWT written before GPT ISR)
    ISR_DWT — ocxo1_phase_isr_dwt (32-bit, raw DWT_CYCCNT from GPT1 ISR)
    Δ(I-S)  — delta_cycles = isr_dwt - shadow_dwt (ISR entry latency)
    EL_SHD  — shadow - dwt_at_pps (elapsed DWT cycles: PPS → shadow)
    EL_ISR  — isr_dwt - dwt_at_pps (elapsed DWT cycles: PPS → ISR)
    EL_ADJ  — (isr_dwt - GPT1_ENTRY) - dwt_at_pps (elapsed after fixed subtraction)
    NS_SHD  — elapsed_shadow * 1e9 / dwt_cycles_per_pps (shadow → GNSS ns)
    NS_ADJ  — elapsed_adjusted * 1e9 / dwt_cycles_per_pps (adjusted → GNSS ns)
    φ_SHD   — NS_SHD % 100 (phase offset from shadow path)
    φ_ADJ   — NS_ADJ % 100 (phase offset from adjusted path)
    φ_FW    — ocxo1_phase_offset_ns (firmware's reported phase)
    r_FW    — ocxo1_residual_ns (firmware's reported residual)
    ISR_R   — isr_residual_ocxo1 (tick-domain ground truth)

Usage:
    python -m zpnet.tests.phase_chain_report <campaign_name> [limit]
    .zt phase_chain_report Alpha5
    .zt phase_chain_report Alpha5 50
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db

GPT1_ISR_ENTRY_DWT_CYCLES = 49


# ─────────────────────────────────────────────────────────────────────
# Field access — fragment-aware
# ─────────────────────────────────────────────────────────────────────

def _frag(rec: Dict[str, Any], key: str, default=None):
    frag = rec.get("fragment")
    if frag is not None:
        val = frag.get(key)
        if val is not None:
            return val
    val = rec.get(key)
    return val if val is not None else default


def _int(rec: Dict[str, Any], key: str) -> Optional[int]:
    val = _frag(rec, key)
    return int(val) if val is not None else None


def _bool(rec: Dict[str, Any], key: str) -> bool:
    val = _frag(rec, key)
    return bool(val) if val is not None else False


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
# Analysis
# ─────────────────────────────────────────────────────────────────────

def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print(f"GPT1_ISR_ENTRY_DWT_CYCLES = {GPT1_ISR_ENTRY_DWT_CYCLES}")
    print()

    # ── Header ──
    print(
        f"  {'PPS':>5s}"
        f"  {'GNSS_NS':>16s}"
        f"  {'DWT@PPS':>12s}"
        f"  {'SHADOW':>12s}"
        f"  {'ISR_DWT':>12s}"
        f"  {'Δ(I-S)':>6s}"
        f"  {'EL_SHD':>8s}"
        f"  {'EL_ISR':>8s}"
        f"  {'EL_ADJ':>8s}"
        f"  {'NS_SHD':>8s}"
        f"  {'NS_ADJ':>8s}"
        f"  {'φSHD':>4s}"
        f"  {'φADJ':>4s}"
        f"  {'φFW':>4s}"
        f"  {'rFW':>5s}"
        f"  {'IR':>3s}"
    )
    print(
        f"  {'─'*5}"
        f"  {'─'*16}"
        f"  {'─'*12}"
        f"  {'─'*12}"
        f"  {'─'*12}"
        f"  {'─'*6}"
        f"  {'─'*8}"
        f"  {'─'*8}"
        f"  {'─'*8}"
        f"  {'─'*8}"
        f"  {'─'*8}"
        f"  {'─'*4}"
        f"  {'─'*4}"
        f"  {'─'*4}"
        f"  {'─'*5}"
        f"  {'─'*3}"
    )

    count = 0

    for rec in rows:
        pps = _int(rec, "pps_count") or _int(rec, "teensy_pps_count")
        if pps is None:
            continue

        if not _bool(rec, "phase_detector_valid"):
            continue

        gnss_ns      = _int(rec, "teensy_gnss_ns")
        dwt_at_pps   = _int(rec, "dwt_cyccnt_at_pps")
        shadow_dwt   = _int(rec, "ocxo1_phase_shadow_dwt")
        isr_dwt      = _int(rec, "ocxo1_phase_isr_dwt")
        delta_cycles = _int(rec, "ocxo1_phase_delta_cycles")
        dwt_per_pps  = _int(rec, "dwt_cycles_per_pps")
        phase_fw     = _int(rec, "ocxo1_phase_offset_ns")
        resid_fw     = _int(rec, "ocxo1_residual_ns")
        isr_r        = _int(rec, "isr_residual_ocxo1")

        if any(v is None for v in [gnss_ns, dwt_at_pps, shadow_dwt, isr_dwt, dwt_per_pps]):
            continue

        # ── Compute elapsed DWT cycles from PPS to each capture point ──
        # These are 32-bit unsigned subtractions (wraparound safe)
        el_shadow = (shadow_dwt - dwt_at_pps) & 0xFFFFFFFF
        el_isr    = (isr_dwt - dwt_at_pps) & 0xFFFFFFFF
        el_adj    = ((isr_dwt - GPT1_ISR_ENTRY_DWT_CYCLES) - dwt_at_pps) & 0xFFFFFFFF

        # ── Convert elapsed DWT cycles to GNSS nanoseconds ──
        # Same formula as time.cpp interpolate_gnss_ns:
        #   ns_into_second = (dwt_elapsed * 1e9 + dwt_per_pps/2) / dwt_per_pps
        ns_shadow = (el_shadow * 1_000_000_000 + dwt_per_pps // 2) // dwt_per_pps
        ns_adj    = (el_adj    * 1_000_000_000 + dwt_per_pps // 2) // dwt_per_pps

        # ── Phase offset = ns mod 100 ──
        phase_shadow = int(ns_shadow % 100)
        phase_adj    = int(ns_adj % 100)

        # ── Format ──
        delta_str = f"{delta_cycles:>6d}" if delta_cycles is not None else f"{'---':>6s}"
        phase_fw_str = f"{phase_fw:>4d}" if phase_fw is not None else f"{'--':>4s}"
        resid_fw_str = f"{resid_fw:>+5d}" if resid_fw is not None else f"{'---':>5s}"
        isr_r_str    = f"{isr_r:>+3d}" if isr_r is not None else f"{'--':>3s}"

        print(
            f"  {pps:>5d}"
            f"  {gnss_ns:>16,}"
            f"  {dwt_at_pps:>12,}"
            f"  {shadow_dwt:>12,}"
            f"  {isr_dwt:>12,}"
            f"  {delta_str}"
            f"  {el_shadow:>8,}"
            f"  {el_isr:>8,}"
            f"  {el_adj:>8,}"
            f"  {ns_shadow:>8,}"
            f"  {ns_adj:>8,}"
            f"  {phase_shadow:>4d}"
            f"  {phase_adj:>4d}"
            f"  {phase_fw_str}"
            f"  {resid_fw_str}"
            f"  {isr_r_str}"
        )

        count += 1
        if limit and count >= limit:
            break

    print()
    print(f"  {count} rows")
    print()

    # ── Legend ──
    print("  Legend:")
    print(f"    DWT@PPS  = best-available DWT at PPS edge (TDC-corrected or ISR-compensated)")
    print(f"    SHADOW   = last DWT written by spin loop before GPT1 ISR preempted")
    print(f"    ISR_DWT  = raw DWT_CYCCNT captured as first instruction in GPT1 ISR")
    print(f"    Δ(I-S)   = ISR_DWT - SHADOW (ISR entry latency in DWT cycles)")
    print(f"    EL_SHD   = SHADOW - DWT@PPS (elapsed cycles, shadow path)")
    print(f"    EL_ISR   = ISR_DWT - DWT@PPS (elapsed cycles, raw ISR path)")
    print(f"    EL_ADJ   = (ISR_DWT - {GPT1_ISR_ENTRY_DWT_CYCLES}) - DWT@PPS (elapsed, fixed-subtraction path)")
    print(f"    NS_SHD   = EL_SHD * 1e9 / dwt_cycles_per_pps (GNSS ns from shadow)")
    print(f"    NS_ADJ   = EL_ADJ * 1e9 / dwt_cycles_per_pps (GNSS ns from fixed subtraction)")
    print(f"    φSHD     = NS_SHD % 100 (phase from shadow)")
    print(f"    φADJ     = NS_ADJ % 100 (phase from fixed subtraction)")
    print(f"    φFW      = firmware-reported ocxo1_phase_offset_ns")
    print(f"    rFW      = firmware-reported ocxo1_residual_ns")
    print(f"    IR       = isr_residual_ocxo1 (10 MHz tick domain)")
    print()


# ─────────────────────────────────────────────────────────────────────
# Entrypoint
# ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: phase_chain_report <campaign_name> [limit]")
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