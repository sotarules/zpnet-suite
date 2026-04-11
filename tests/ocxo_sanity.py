
"""
ZPNet OCXO Sanity Check — focused OCXO1 lane analyzer

Purpose:
  1. Inspect whether OCXO1 pathological approach-cycle behavior correlates with
     residual drift / calibration state.
  2. Separate "clean" samples from timeout-contaminated samples.
  3. Compare OCXO1 against OCXO2 as a control lane.
  4. Surface whether huge approach values are isolated timeout artifacts or
     part of the normal lane behavior.

Usage:
    python -m zpnet.tests.ocxo_sanity <campaign_name> [limit]
    .zt ocxo_sanity Shakeout1
    .zt ocxo_sanity Shakeout1 200

Notes:
  - Reads TIMEBASE.payload rows from Postgres.
  - Uses fragment fields when available.
  - Treats timeout growth and absurd approach/runway values as contamination.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


ABSURD_APPROACH_CYCLES = 1_000_000      # ~1 ms at ~1 GHz; far above sane prespin runway
ABSURD_RUNWAY_NS       = 1_000_000      # 1 ms; beyond sane "close to edge" regime


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


def _as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
    except Exception:
        return None


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

    out: List[Dict[str, Any]] = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        out.append(p)
    return out


@dataclass
class LaneRow:
    pps: int
    residual_ns: Optional[int]
    gnss_ns_between_edges: Optional[int]
    approach_cycles: Optional[int]
    runway_ns: Optional[int]
    event_error_ns: Optional[int]
    timeout_count: Optional[int]
    anomaly_count: Optional[int]
    dac: Optional[float]
    phase_offset_ns: Optional[int]
    timed_out: Optional[bool]

    @property
    def absurd_approach(self) -> bool:
        return self.approach_cycles is not None and self.approach_cycles >= ABSURD_APPROACH_CYCLES

    @property
    def absurd_runway(self) -> bool:
        return self.runway_ns is not None and abs(self.runway_ns) >= ABSURD_RUNWAY_NS


def _lane_row(rec: Dict[str, Any], pps: int, prefix: str) -> LaneRow:
    return LaneRow(
        pps=pps,
        residual_ns=_as_int(_frag(rec, f"{prefix}_second_residual_ns")),
        gnss_ns_between_edges=_as_int(_frag(rec, f"{prefix}_gnss_ns_between_edges")),
        approach_cycles=_as_int(_frag(rec, f"{prefix}_diag_approach_cycles")),
        runway_ns=_as_int(_frag(rec, f"{prefix}_diag_event_minus_prespin_fire_ns")),
        event_error_ns=_as_int(_frag(rec, f"{prefix}_diag_event_error_ns")),
        timeout_count=_as_int(_frag(rec, f"{prefix}_diag_prespin_timeout_count")),
        anomaly_count=_as_int(_frag(rec, f"{prefix}_diag_anomaly_count")),
        dac=_as_float(_frag(rec, f"{prefix}_dac")),
        phase_offset_ns=_as_int(_frag(rec, f"{prefix}_phase_offset_ns")),
        timed_out=None if _frag(rec, f"{prefix}_diag_prespin_timed_out") is None else bool(_frag(rec, f"{prefix}_diag_prespin_timed_out")),
    )


def _fmt(v: Any, width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    elif isinstance(v, float):
        s = f"{v:+.3f}" if signed else f"{v:.3f}"
    elif isinstance(v, bool):
        s = "Y" if v else "N"
    else:
        s = f"{int(v):+,d}" if signed else f"{int(v):,d}"
    return f"{s:>{width}s}" if width else s


def _timeout_increment(curr: Optional[int], prev: Optional[int]) -> bool:
    if curr is None or prev is None:
        return False
    return curr > prev


def _correlation(xs: List[float], ys: List[float]) -> Optional[float]:
    if len(xs) < 3 or len(ys) < 3 or len(xs) != len(ys):
        return None
    mx = statistics.mean(xs)
    my = statistics.mean(ys)
    num = 0.0
    dx2 = 0.0
    dy2 = 0.0
    for x, y in zip(xs, ys):
        dx = x - mx
        dy = y - my
        num += dx * dy
        dx2 += dx * dx
        dy2 += dy * dy
    if dx2 <= 0 or dy2 <= 0:
        return None
    return num / math.sqrt(dx2 * dy2)


def _stats(series: List[float]) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    if not series:
        return None, None, None, None
    mean = statistics.mean(series)
    sd = statistics.stdev(series) if len(series) >= 2 else 0.0
    return mean, sd, min(series), max(series)


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows)} records)")
    print()

    print(
        f"{'pps':>6s}  "
        f"{'o1_res':>8s} {'o1_app':>10s} {'o1_run':>10s} {'o1_to':>5s} {'o1_an':>5s} {'o1_dac':>10s} "
        f"{'o2_res':>8s} {'o2_app':>10s} {'o2_run':>10s} {'o2_to':>5s} {'tag':>8s}"
    )
    print(
        f"{'─'*6}  "
        f"{'─'*8} {'─'*10} {'─'*10} {'─'*5} {'─'*5} {'─'*10} "
        f"{'─'*8} {'─'*10} {'─'*10} {'─'*5} {'─'*8}"
    )

    prev_o1_to: Optional[int] = None
    prev_o2_to: Optional[int] = None

    clean_o1_approach: List[float] = []
    dirty_o1_approach: List[float] = []
    clean_o1_residual: List[float] = []
    clean_o2_approach: List[float] = []
    clean_o2_residual: List[float] = []
    corr_x: List[float] = []
    corr_y: List[float] = []
    giant_o1: List[int] = []
    timeout_rows: List[int] = []

    count = 0

    for rec in rows:
        pps = _as_int(rec.get("pps_count"))
        if pps is None:
            continue

        o1 = _lane_row(rec, pps, "ocxo1")
        o2 = _lane_row(rec, pps, "ocxo2")

        o1_to_inc = _timeout_increment(o1.timeout_count, prev_o1_to)
        o2_to_inc = _timeout_increment(o2.timeout_count, prev_o2_to)

        contaminated_o1 = bool(o1_to_inc or o1.timed_out or o1.absurd_approach or o1.absurd_runway)
        contaminated_o2 = bool(o2_to_inc or o2.timed_out or o2.absurd_approach or o2.absurd_runway)

        if o1_to_inc:
            timeout_rows.append(pps)
        if o1.absurd_approach or o1.absurd_runway:
            giant_o1.append(pps)

        if contaminated_o1:
            tag = "O1_DIRTY"
        elif contaminated_o2:
            tag = "O2_DIRTY"
        else:
            tag = "clean"

        print(
            f"{pps:>6d}  "
            f"{_fmt(o1.residual_ns, 8, signed=True)} {_fmt(o1.approach_cycles, 10)} {_fmt(o1.runway_ns, 10, signed=True)} {_fmt(o1.timeout_count, 5)} {_fmt(o1.anomaly_count, 5)} {_fmt(o1.dac, 10)} "
            f"{_fmt(o2.residual_ns, 8, signed=True)} {_fmt(o2.approach_cycles, 10)} {_fmt(o2.runway_ns, 10, signed=True)} {_fmt(o2.timeout_count, 5)} {tag:>8s}"
        )

        if o1.approach_cycles is not None:
            (dirty_o1_approach if contaminated_o1 else clean_o1_approach).append(float(o1.approach_cycles))
        if (not contaminated_o1) and o1.residual_ns is not None:
            clean_o1_residual.append(float(o1.residual_ns))
        if (not contaminated_o2) and o2.approach_cycles is not None:
            clean_o2_approach.append(float(o2.approach_cycles))
        if (not contaminated_o2) and o2.residual_ns is not None:
            clean_o2_residual.append(float(o2.residual_ns))

        if (not contaminated_o1) and o1.residual_ns is not None and o1.approach_cycles is not None:
            corr_x.append(float(o1.residual_ns))
            corr_y.append(float(o1.approach_cycles))

        prev_o1_to = o1.timeout_count
        prev_o2_to = o2.timeout_count

        count += 1
        if limit and count >= limit:
            break

    print()
    print("Summary")
    print("───────")

    cmean, csd, cmin, cmax = _stats(clean_o1_approach)
    dmean, dsd, dmin, dmax = _stats(dirty_o1_approach)
    o2mean, o2sd, o2min, o2max = _stats(clean_o2_approach)
    r1mean, r1sd, r1min, r1max = _stats(clean_o1_residual)
    r2mean, r2sd, r2min, r2max = _stats(clean_o2_residual)

    print(f"OCXO1 clean approach cycles : n={len(clean_o1_approach):,} mean={cmean:.1f} sd={csd:.1f} min={cmin:.0f} max={cmax:.0f}" if cmean is not None else "OCXO1 clean approach cycles : ---")
    print(f"OCXO1 dirty approach cycles : n={len(dirty_o1_approach):,} mean={dmean:.1f} sd={dsd:.1f} min={dmin:.0f} max={dmax:.0f}" if dmean is not None else "OCXO1 dirty approach cycles : ---")
    print(f"OCXO2 clean approach cycles : n={len(clean_o2_approach):,} mean={o2mean:.1f} sd={o2sd:.1f} min={o2min:.0f} max={o2max:.0f}" if o2mean is not None else "OCXO2 clean approach cycles : ---")
    print()

    print(f"OCXO1 clean residual ns     : n={len(clean_o1_residual):,} mean={r1mean:+.1f} sd={r1sd:.1f} min={r1min:+.0f} max={r1max:+.0f}" if r1mean is not None else "OCXO1 clean residual ns     : ---")
    print(f"OCXO2 clean residual ns     : n={len(clean_o2_residual):,} mean={r2mean:+.1f} sd={r2sd:.1f} min={r2min:+.0f} max={r2max:+.0f}" if r2mean is not None else "OCXO2 clean residual ns     : ---")
    print()

    corr = _correlation(corr_x, corr_y)
    if corr is not None:
        print(f"OCXO1 residual↔approach correlation (clean only): {corr:+.3f}")
        if abs(corr) < 0.2:
            print("  Interpretation: little linear relationship; large approach spikes likely not explained by ordinary residual drift.")
        elif corr > 0:
            print("  Interpretation: larger positive residual tends to accompany larger approach.")
        else:
            print("  Interpretation: more negative residual tends to accompany larger approach.")
    else:
        print("OCXO1 residual↔approach correlation (clean only): ---")
    print()

    print(f"OCXO1 timeout-increment rows: {len(timeout_rows):,} -> {timeout_rows[:20]}{' ...' if len(timeout_rows) > 20 else ''}")
    print(f"OCXO1 absurd-approach/runway rows: {len(giant_o1):,} -> {giant_o1[:20]}{' ...' if len(giant_o1) > 20 else ''}")
    print()

    if o2mean is not None and cmean is not None:
        ratio = cmean / o2mean if o2mean else float("inf")
        print(f"OCXO1 clean approach / OCXO2 clean approach ratio: {ratio:.2f}x")
        if ratio > 3.0:
            print("  OCXO1 remains materially noisier than OCXO2 even after excluding timeout-contaminated rows.")
        else:
            print("  OCXO1 clean behavior is in the same ballpark as OCXO2; pathology is concentrated in timeout/dirty rows.")
    print()

    print("Rule of thumb:")
    print("  - If OCXO1 clean approach is near OCXO2, the huge spikes are mostly timeout-state artifacts.")
    print("  - If OCXO1 clean approach is still much larger, the lane itself is noisier even away from timeouts.")
    print("  - If residual↔approach correlation is weak, poor calibration alone is probably not the whole story.")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: ocxo_sanity <campaign_name> [limit]")
        sys.exit(1)

    campaign = sys.argv[1]
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    analyze(campaign, limit)


if __name__ == "__main__":
    main()
