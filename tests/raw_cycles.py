"""
ZPNet Raw Cycles — PPS/VCLOCK DWT interval audit.

Reads TIMEBASE rows for a campaign and compares the one-second DWT interval
computed from two candidate timing surfaces:

  1. PPS / physical GPIO witness DWT, when available in TIMEBASE.
  2. PPS_VCLOCK / canonical VCLOCK-selected DWT, from current TIMEBASE schema.

Current TIMEBASE schema fields used:

  fragment.dwt_at_pps_vclock
      Canonical PPS_VCLOCK DWT coordinate.

  fragment.dwt_cycles_between_pps_vclock
      Firmware-authored one-second DWT interval between consecutive
      PPS_VCLOCK bookends.

  fragment.dwt_expected_per_pps_vclock
      Expected/nominal DWT cycles per PPS_VCLOCK second.

  fragment.prediction.vclock_static_prediction_cycles
  fragment.prediction.vclock_actual_cycles
  fragment.prediction.vclock_static_residual_cycles
      Compact Gamma prediction audit.

Legacy PPS/GPIO witness fields are optional. If a TIMEBASE row still carries
one of the older physical PPS DWT fields, the report computes PPS-to-PPS
intervals from it. If not, PPS columns show "---" and the report still
analyzes VCLOCK.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    python raw_cycles.py <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

from zpnet.shared.db import open_db


DEFAULT_DWT_EXPECTED_PER_PPS = 1_008_000_000


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
        self.min_val = min(self.min_val, x)
        self.max_val = max(self.max_val, x)

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                NULLIF(payload->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_count', '')::bigint
            ) ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        result.append(payload)
    return result


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _gnss(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    g = root.get("gnss")
    return g if isinstance(g, dict) else {}


def _prediction(frag: Dict[str, Any]) -> Dict[str, Any]:
    p = frag.get("prediction")
    return p if isinstance(p, dict) else {}


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def _as_str(v: Any) -> Optional[str]:
    if v is None:
        return None
    try:
        return str(v)
    except Exception:
        return None


def _first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = _as_int(v)
        if out is not None:
            return out
    return None


def _first_float(*values: Any) -> Optional[float]:
    for v in values:
        out = _as_float(v)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 2,
               signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _fmt_str(v: Optional[str], width: int = 0) -> str:
    if v is None:
        s = "---"
    else:
        s = v[:width] if width else v
    return f"{s:<{width}s}" if width else s


def _mod4(v: Optional[int]) -> Optional[int]:
    return None if v is None else int(v & 3)


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _print_welford(name: str, w: Welford, unit: str = "cycles",
                   decimals: int = 3) -> None:
    if w.n == 0:
        print(f"  {name:<52s} no samples")
        return
    print(
        f"  {name:<52s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+11,.{decimals}f} {unit:<6s}  "
        f"sd={w.stddev:10,.{decimals}f}  "
        f"se={w.stderr:9,.{decimals}f}  "
        f"min={w.min_val:+10,.{decimals}f}  "
        f"max={w.max_val:+10,.{decimals}f}"
    )


def physical_pps_dwt_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("pps_dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_normalized_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_raw_at_edge"),
        frag.get("pps_diag_pps_edge_dwt_isr_entry_raw"),
        frag.get("pps_edge_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
    )


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows):,} rows)")
    print()
    print("Current TIMEBASE schema:")
    print("  fragment.dwt_at_pps_vclock              canonical VCLOCK-selected DWT edge")
    print("  fragment.dwt_cycles_between_pps_vclock  firmware VCLOCK edge-to-edge interval")
    print("  fragment.dwt_expected_per_pps_vclock    expected DWT cycles per VCLOCK second")
    print("  fragment.prediction.vclock_*            Gamma VCLOCK prediction audit")
    print()
    print("PPS physical columns are computed only when legacy/raw PPS DWT fields exist.")
    print("If those fields are absent, PPS columns show '---'.")
    print()

    header = (
        f"{'pps':>6s}  "
        f"{'pps_dwt':>13s}  "
        f"{'vclk_dwt':>13s}  "
        f"{'phase':>9s}  "
        f"{'ph_step':>8s}  "
        f"{'ph%4':>4s}  "
        f"{'pps_cyc':>13s}  "
        f"{'pps_res':>9s}  "
        f"{'pps_pred':>8s}  "
        f"{'p%4':>3s}  "
        f"{'vclk_cyc':>13s}  "
        f"{'vclk_res':>9s}  "
        f"{'v_pred':>8s}  "
        f"{'v%4':>3s}  "
        f"{'gamma_act':>13s}  "
        f"{'g_stat':>13s}  "
        f"{'g_sres':>7s}  "
        f"{'v-p':>8s}  "
        f"{'dwt_ppb':>10s}  "
        f"{'gnss_drift':>10s}  "
        f"{'mode':<10s}"
    )
    print(header)
    print(
        f"{'─'*6}  {'─'*13}  {'─'*13}  {'─'*9}  {'─'*8}  {'─'*4}  "
        f"{'─'*13}  {'─'*9}  {'─'*8}  {'─'*3}  {'─'*13}  {'─'*9}  "
        f"{'─'*8}  {'─'*3}  {'─'*13}  {'─'*13}  {'─'*7}  {'─'*8}  "
        f"{'─'*10}  {'─'*10}  {'─'*10}"
    )

    expected = DEFAULT_DWT_EXPECTED_PER_PPS
    shown = 0
    gaps = 0

    prev_pps: Optional[int] = None
    prev_physical_pps_dwt: Optional[int] = None
    prev_vclock_dwt: Optional[int] = None
    prev_pps_cycles: Optional[int] = None
    prev_vclock_cycles: Optional[int] = None
    prev_phase: Optional[int] = None

    w_pps_cycles = Welford()
    w_pps_res = Welford()
    w_pps_pred = Welford()
    w_vclock_cycles = Welford()
    w_vclock_res = Welford()
    w_vclock_pred = Welford()
    w_phase = Welford()
    w_phase_step = Welford()
    w_v_minus_p = Welford()
    w_gamma_sres = Welford()

    schema_counts = {
        "rows": 0,
        "pps_dwt_present": 0,
        "vclock_dwt_present": 0,
        "vclock_cycles_present": 0,
        "gamma_present": 0,
    }

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        pred = _prediction(frag)
        gnss = _gnss(rec)

        pps = _first_int(root.get("pps_count"), frag.get("pps_count"),
                         frag.get("teensy_pps_vclock_count"),
                         frag.get("teensy_pps_count"))
        if pps is None:
            continue

        expected = _first_int(frag.get("dwt_expected_per_pps_vclock")) or expected

        physical_pps_dwt = physical_pps_dwt_from_schema(root, frag)
        vclock_dwt = _first_int(frag.get("dwt_at_pps_vclock"),
                                frag.get("dwt_cycle_count_at_pps"),
                                root.get("dwt_at_pps_vclock"))

        firmware_vclock_cycles = _first_int(
            frag.get("dwt_cycles_between_pps_vclock"),
            frag.get("vclock_dwt_cycles_between_edges"),
            frag.get("dwt_cycle_count_between_pps"),
        )

        gamma_actual = _first_int(pred.get("vclock_actual_cycles"))
        gamma_static = _first_int(pred.get("vclock_static_prediction_cycles"))
        gamma_sres = _first_int(pred.get("vclock_static_residual_cycles"))

        dwt_ppb = _first_float(frag.get("dwt_ppb"))
        gnss_drift = _first_float(gnss.get("clock_drift_ppb"))
        mode = _as_str(gnss.get("freq_mode_name"))

        schema_counts["rows"] += 1
        if physical_pps_dwt is not None:
            schema_counts["pps_dwt_present"] += 1
        if vclock_dwt is not None:
            schema_counts["vclock_dwt_present"] += 1
        if firmware_vclock_cycles is not None:
            schema_counts["vclock_cycles_present"] += 1
        if gamma_actual is not None or gamma_static is not None:
            schema_counts["gamma_present"] += 1

        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps:,} → {pps:,} ---")
            prev_physical_pps_dwt = None
            prev_vclock_dwt = None
            prev_pps_cycles = None
            prev_vclock_cycles = None
            prev_phase = None

        pps_cycles: Optional[int] = None
        if physical_pps_dwt is not None and prev_physical_pps_dwt is not None:
            pps_cycles = _delta_u32(physical_pps_dwt, prev_physical_pps_dwt)

        vclock_cycles_from_dwt: Optional[int] = None
        if vclock_dwt is not None and prev_vclock_dwt is not None:
            vclock_cycles_from_dwt = _delta_u32(vclock_dwt, prev_vclock_dwt)

        vclock_cycles = vclock_cycles_from_dwt if vclock_cycles_from_dwt is not None else firmware_vclock_cycles

        phase: Optional[int] = None
        if physical_pps_dwt is not None and vclock_dwt is not None:
            phase = (vclock_dwt - physical_pps_dwt) & 0xFFFFFFFF
            if phase > 0x7FFFFFFF:
                phase -= 0x100000000

        phase_step: Optional[int] = None
        if phase is not None and prev_phase is not None:
            phase_step = phase - prev_phase

        pps_res = (pps_cycles - expected) if pps_cycles is not None else None
        vclock_res = (vclock_cycles - expected) if vclock_cycles is not None else None
        pps_pred = (pps_cycles - prev_pps_cycles) if pps_cycles is not None and prev_pps_cycles is not None else None
        vclock_pred = (vclock_cycles - prev_vclock_cycles) if vclock_cycles is not None and prev_vclock_cycles is not None else None
        v_minus_p = (vclock_cycles - pps_cycles) if vclock_cycles is not None and pps_cycles is not None else None

        if pps_cycles is not None:
            w_pps_cycles.update(float(pps_cycles))
        if pps_res is not None:
            w_pps_res.update(float(pps_res))
        if pps_pred is not None:
            w_pps_pred.update(float(pps_pred))
        if vclock_cycles is not None:
            w_vclock_cycles.update(float(vclock_cycles))
        if vclock_res is not None:
            w_vclock_res.update(float(vclock_res))
        if vclock_pred is not None:
            w_vclock_pred.update(float(vclock_pred))
        if phase is not None:
            w_phase.update(float(phase))
        if phase_step is not None:
            w_phase_step.update(float(phase_step))
        if v_minus_p is not None:
            w_v_minus_p.update(float(v_minus_p))
        if gamma_sres is not None:
            w_gamma_sres.update(float(gamma_sres))

        print(
            f"{pps:>6d}  "
            f"{_fmt_int(physical_pps_dwt, 13)}  "
            f"{_fmt_int(vclock_dwt, 13)}  "
            f"{_fmt_int(phase, 9, signed=True)}  "
            f"{_fmt_int(phase_step, 8, signed=True)}  "
            f"{_fmt_int(_mod4(phase), 4)}  "
            f"{_fmt_int(pps_cycles, 13)}  "
            f"{_fmt_int(pps_res, 9, signed=True)}  "
            f"{_fmt_int(pps_pred, 8, signed=True)}  "
            f"{_fmt_int(_mod4(pps_cycles), 3)}  "
            f"{_fmt_int(vclock_cycles, 13)}  "
            f"{_fmt_int(vclock_res, 9, signed=True)}  "
            f"{_fmt_int(vclock_pred, 8, signed=True)}  "
            f"{_fmt_int(_mod4(vclock_cycles), 3)}  "
            f"{_fmt_int(gamma_actual, 13)}  "
            f"{_fmt_int(gamma_static, 13)}  "
            f"{_fmt_int(gamma_sres, 7, signed=True)}  "
            f"{_fmt_int(v_minus_p, 8, signed=True)}  "
            f"{_fmt_float(dwt_ppb, 10, 3, signed=True)}  "
            f"{_fmt_float(gnss_drift, 10, 3, signed=True)}  "
            f"{_fmt_str(mode, 10)}"
        )

        prev_pps = pps
        if physical_pps_dwt is not None:
            prev_physical_pps_dwt = physical_pps_dwt
        if vclock_dwt is not None:
            prev_vclock_dwt = vclock_dwt
        if pps_cycles is not None:
            prev_pps_cycles = pps_cycles
        if vclock_cycles is not None:
            prev_vclock_cycles = vclock_cycles
        if phase is not None:
            prev_phase = phase

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()
    print("Schema coverage")
    print("═══════════════")
    print(f"  rows                    = {schema_counts['rows']:,}")
    print(f"  physical PPS DWT present = {schema_counts['pps_dwt_present']:,}")
    print(f"  VCLOCK DWT present       = {schema_counts['vclock_dwt_present']:,}")
    print(f"  VCLOCK interval present  = {schema_counts['vclock_cycles_present']:,}")
    print(f"  Gamma prediction present = {schema_counts['gamma_present']:,}")
    print()

    print("Summary")
    print("═══════")
    _print_welford("PPS physical cycles", w_pps_cycles)
    _print_welford("PPS physical residual vs expected", w_pps_res)
    _print_welford("PPS physical one-step residual", w_pps_pred)
    _print_welford("VCLOCK cycles", w_vclock_cycles)
    _print_welford("VCLOCK residual vs expected", w_vclock_res)
    _print_welford("VCLOCK one-step residual", w_vclock_pred)
    _print_welford("PPS→VCLOCK phase", w_phase)
    _print_welford("PPS→VCLOCK phase step", w_phase_step)
    _print_welford("VCLOCK cycles minus PPS cycles", w_v_minus_p)
    _print_welford("Gamma VCLOCK static residual", w_gamma_sres)
    print()

    print("Interpretation cues")
    print("═══════════════════")
    print("  • If VCLOCK cycles are locked to a 4-cycle lattice but PPS cycles are")
    print("    smoother, the quantization was introduced by the VCLOCK/TimePop DWT rail.")
    print("  • If both PPS and VCLOCK are locked to the same lattice, the quantization")
    print("    is deeper than the canonical-edge choice.")
    print("  • phase_step shows how the physical PPS witness moves relative to the")
    print("    canonical PPS_VCLOCK DWT edge each second.")
    print("  • Current compact TIMEBASE may not include physical PPS DWT; in that case")
    print("    the report can only audit the VCLOCK rail until PPS witness fields are")
    print("    reintroduced.")
    print()


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit]")
        raise SystemExit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
