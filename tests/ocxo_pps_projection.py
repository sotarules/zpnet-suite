"""
ZPNet OCXO PPS Projection Report — phase 4.

Purpose:
  Print one row per TIMEBASE PPS row for one OCXO lane, focusing on the
  PPS-projected OCXO nanosecond clock value and the assumptions immediately
  visible inside TIMEBASE_FRAGMENT_V2.

This is deliberately a nanosecond/projection report, not raw_cycles.  It asks:

  • What was the PPS DWT coordinate for this TIMEBASE row?
  • What was the canonical GNSS/PPS nanosecond value?
  • What OCXO boundary DWT did Alpha publish for the lane?
  • What OCXO nanosecond clock value did TIMEBASE publish at the PPS edge?
  • Do consecutive public OCXO ns values produce the same residual that
    TIMEBASE published in ocxoN.pps_residual.fast_residual_ns?
  • How do the static DWT prediction surfaces line up with the OCXO edge DWT?
  • How far is the projected PPS value from the old measured-GNSS side ledger?

Usage:
    python -m zpnet.tests.ocxo_pps_projection_report_v4 <campaign> [lane] [limit]
    python ocxo_pps_projection_report_v4.py <campaign> [lane] [limit]

Examples:
    python ocxo_pps_projection_report_v4.py gnssf ocxo1
    python ocxo_pps_projection_report_v4.py gnssf ocxo2 100
    python ocxo_pps_projection_report_v4.py gnssf both 80

Lane values:
    ocxo1, o1, 1
    ocxo2, o2, 2
    both, all     (renders each lane separately to keep rows readable)

Notes:
  TIMEBASE_FRAGMENT_V2 currently does not publish Alpha's projection
  target_delta_cycles / target_remaining_cycles from REPORT_OCXO_PPS_PROJECTION.
  This report will display those fields if they appear in future rows, but it
  does not invent them from other values.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


VALID_LANES = {
    "ocxo1": ("ocxo1",),
    "o1": ("ocxo1",),
    "1": ("ocxo1",),
    "ocxo2": ("ocxo2",),
    "o2": ("ocxo2",),
    "2": ("ocxo2",),
    "both": ("ocxo1", "ocxo2"),
    "all": ("ocxo1", "ocxo2"),
}


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
                NULLIF(payload->'fragment'->>'teensy_pps_count', '')::bigint,
                NULLIF(payload->'payload'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'pps_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'payload'->'fragment'->>'teensy_pps_count', '')::bigint
            ) ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    out: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            out.append(payload)
    return out


def root_of(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    payload = rec.get("payload")
    return payload if isinstance(payload, dict) else rec


def frag_of(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = root_of(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else root


def nested(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def as_bool(v: Any) -> Optional[bool]:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("true", "t", "yes", "y", "1"):
            return True
        if s in ("false", "f", "no", "n", "0"):
            return False
    return None


def first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = as_int(v)
        if out is not None:
            return out
    return None


def first_bool(*values: Any) -> Optional[bool]:
    for v in values:
        out = as_bool(v)
        if out is not None:
            return out
    return None


def first_str(*values: Any) -> Optional[str]:
    for v in values:
        if isinstance(v, str) and v:
            return v
    return None


def delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}}" if width else s


def fmt_str(v: Optional[str], width: int = 0) -> str:
    s = "---" if not v else v
    return f"{s:<{width}}" if width else s


def fmt_bool(v: Optional[bool], width: int = 0) -> str:
    if v is None:
        s = "---"
    else:
        s = "1" if v else "0"
    return f"{s:>{width}}" if width else s


def print_right_justified_table(headers: Sequence[str], rows: Sequence[Dict[str, str]]) -> None:
    """Print a single-line-per-row table with dynamically computed widths.

    Every value is rendered to a string first.  The width of each column is
    then max(len(header), len(rendered values...)), and every column is
    right-justified.  There are no hand-tuned numeric widths in the table body.
    """
    if not rows:
        return

    widths: Dict[str, int] = {h: len(h) for h in headers}
    for row in rows:
        for h in headers:
            widths[h] = max(widths[h], len(row.get(h, "---")))

    print("  ".join(h.rjust(widths[h]) for h in headers))
    print("  ".join("─" * widths[h] for h in headers))
    for row in rows:
        print("  ".join(row.get(h, "---").rjust(widths[h]) for h in headers))


def print_stats(label: str, w: Welford, unit: str = "ns", decimals: int = 3) -> None:
    if w.n == 0:
        print(f"  {label:<42s} no samples")
        return
    print(
        f"  {label:<42s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+12,.{decimals}f} {unit:<6s}  "
        f"sd={w.stddev:10,.{decimals}f}  "
        f"se={w.stderr:9,.{decimals}f}  "
        f"min={w.min_val:+12,.{decimals}f}  "
        f"max={w.max_val:+12,.{decimals}f}"
    )


def row_pps_count(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return first_int(
        nested(frag, "pps", "count"),
        frag.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        root.get("teensy_pps_count"),
    )


def row_pps_dwt(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return first_int(
        nested(frag, "pps", "dwt_at_edge"),
        frag.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        nested(root, "fragment", "pps", "dwt_at_edge"),
    )


def row_gnss_ns(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return first_int(
        nested(frag, "gnss", "ns"),
        nested(frag, "gnss", "pps_vclock_ns"),
        frag.get("gnss_ns"),
        root.get("gnss_ns"),
        nested(root, "fragment", "gnss", "ns"),
    )


def lane_obj(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Dict[str, Any]:
    obj = frag.get(lane)
    if isinstance(obj, dict):
        return obj
    obj = nested(root, "fragment", lane)
    return obj if isinstance(obj, dict) else {}


def lane_ns(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Optional[int]:
    lane_o = lane_obj(root, frag, lane)
    return first_int(
        nested(lane_o, "ns"),
        nested(lane_o, "ns_at_pps_vclock"),
        nested(lane_o, "pps_projected_ns"),
        nested(frag, "gnss", f"{lane}_ns"),
        frag.get(f"{lane}_ns"),
        root.get(f"{lane}_ns"),
    )


def lane_measured_ns(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Optional[int]:
    lane_o = lane_obj(root, frag, lane)
    return first_int(
        nested(lane_o, "measured_gnss_ns"),
        nested(lane_o, "measured_gnss_ns_at_pps_vclock"),
        nested(lane_o, "legacy_measured_gnss_ns"),
        nested(frag, "gnss", f"{lane}_measured_ns"),
    )


def lane_edge_dwt(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Optional[int]:
    lane_o = lane_obj(root, frag, lane)
    # Prefer Alpha's boundary/applied event DWT. The quiet-zone sample DWT is
    # visible below as a separate assumption surface in future reports, but the
    # public OCXO one-second edge is the boundary/applied DWT.
    return first_int(
        nested(lane_o, "forensics", "last_event_dwt"),
        nested(lane_o, "service", "boundary_dwt_at_event"),
        nested(lane_o, "service", "sample_dwt_at_event"),
        frag.get(f"{lane}_forensics_last_event_dwt"),
        root.get(f"{lane}_forensics_last_event_dwt"),
        nested(root, lane, "alpha_event", "last_event_dwt"),
    )


def lane_source(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Optional[str]:
    lane_o = lane_obj(root, frag, lane)
    return first_str(
        nested(lane_o, "ns_source"),
        root.get(f"{lane}_ns_source"),
    )


def lane_projected_valid(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Optional[bool]:
    lane_o = lane_obj(root, frag, lane)
    return first_bool(nested(lane_o, "pps_projected_valid"))


def lane_projected_minus_measured(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Optional[int]:
    lane_o = lane_obj(root, frag, lane)
    direct = first_int(nested(lane_o, "pps_projected_minus_measured_ns"))
    if direct is not None:
        return direct
    ns = lane_ns(root, frag, lane)
    measured = lane_measured_ns(root, frag, lane)
    if ns is None or measured is None:
        return None
    return ns - measured


def lane_pps_residual(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Tuple[Optional[bool], Optional[int], Optional[int], Optional[int]]:
    lane_o = lane_obj(root, frag, lane)
    return (
        first_bool(nested(lane_o, "pps_residual", "valid")),
        first_int(nested(lane_o, "pps_residual", "gnss_interval_ns")),
        first_int(nested(lane_o, "pps_residual", "clock_interval_ns")),
        first_int(nested(lane_o, "pps_residual", "fast_residual_ns")),
    )


def lane_prediction(frag: Dict[str, Any], lane: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    pred = frag.get("prediction")
    if not isinstance(pred, dict):
        pred = {}
    lane_pred = pred.get(lane)
    if not isinstance(lane_pred, dict):
        lane_pred = {}
    actual = first_int(lane_pred.get("actual_cycles"), pred.get(f"{lane}_actual_cycles"), frag.get(f"{lane}_actual_cycles"))
    prediction = first_int(lane_pred.get("prediction_cycles"), lane_pred.get("static_prediction_cycles"), pred.get(f"{lane}_prediction_cycles"), frag.get(f"{lane}_prediction_cycles"))
    residual = first_int(lane_pred.get("residual_cycles"), lane_pred.get("static_residual_cycles"), pred.get(f"{lane}_residual_cycles"), frag.get(f"{lane}_residual_cycles"))
    if residual is None and actual is not None and prediction is not None:
        residual = actual - prediction
    return actual, prediction, residual


def lane_projection_extra(root: Dict[str, Any], frag: Dict[str, Any], lane: str) -> Tuple[Optional[int], Optional[int]]:
    # These are currently only in REPORT_OCXO_PPS_PROJECTION, not TIMEBASE.
    # Keep readers in place so the same script lights up automatically if the
    # fields are later added to TIMEBASE.
    lane_o = lane_obj(root, frag, lane)
    return (
        first_int(
            nested(lane_o, "projection", "target_delta_cycles"),
            nested(lane_o, "pps_projection", "target_delta_cycles"),
            nested(lane_o, "projection_target_delta_cycles"),
        ),
        first_int(
            nested(lane_o, "projection", "target_remaining_cycles"),
            nested(lane_o, "pps_projection", "target_remaining_cycles"),
            nested(lane_o, "projection_target_remaining_cycles"),
        ),
    )


def render_lane(campaign: str, rows: Sequence[Dict[str, Any]], lane: str, limit: int) -> None:
    shown = 0
    gaps = 0
    projected_rows = 0
    fallback_rows = 0
    residual_mismatch_rows = 0

    prev_pps: Optional[int] = None
    prev_pps_dwt: Optional[int] = None
    prev_edge_dwt: Optional[int] = None
    prev_gnss_ns: Optional[int] = None
    prev_ocxo_ns: Optional[int] = None

    calc_res_stats = Welford()
    pub_res_stats = Welford()
    residual_delta_stats = Welford()
    offset_stats = Welford()
    pps_dwt_delta_stats = Welford()
    edge_dwt_delta_stats = Welford()
    ocxo_interval_stats = Welford()
    static_res_stats = Welford()
    proj_measured_stats = Welford()

    title = f"OCXO PPS projection audit v4 — campaign={campaign} lane={lane.upper()}"
    print(title)
    print("═" * len(title))
    print("  pps_dwt      physical PPS DWT-at-edge from TIMEBASE.")
    print("  pΔ           modulo-2^32 PPS DWT interval from prior row.")
    print("  gnss_ns      canonical GNSS/PPS ns at this TIMEBASE row.")
    print("  edge_dwt     Alpha OCXO boundary/applied event DWT for this lane.")
    print("  eΔ           modulo-2^32 OCXO edge-DWT interval from prior row.")
    print("  ocxo_ns      canonical PPS-projected OCXO ns at this row.")
    print("  oΔ           public OCXO ns interval from prior row.")
    print("  calc_res     oΔ - GNSS interval, computed by this report.")
    print("  pub_res      TIMEBASE ocxoN.pps_residual.fast_residual_ns.")
    print("  rΔ           calc_res - pub_res; should be zero when both are valid.")
    print("  act/pred/sr  firmware static DWT prediction surface for this OCXO lane.")
    print("  p-meas       projected OCXO ns minus legacy measured-GNSS side ledger.")
    print("  tgt/trem     projection target/remaining cycles if TIMEBASE publishes them; otherwise ---.")
    print()

    # Phase 4: render every cell first, compute each column width from the rendered
    # cell strings, then right-justify every column.  This keeps a single
    # physical row while preserving alignment even when values grow from --- to
    # signed comma-formatted numbers.
    headers = [
        "pps", "pps_dwt", "p_delta", "gnss_ns", "edge_dwt", "e_delta",
        "ocxo_ns", "ocxo_delta", "calc", "pub", "r_delta", "act",
        "pred", "sr", "p_minus_meas", "tgt", "trem", "src", "proj",
    ]
    table_rows: List[Dict[str, str]] = []

    for rec in rows:
        root = root_of(rec)
        frag = frag_of(rec)
        pps = row_pps_count(root, frag)
        if pps is None:
            continue

        pps_dwt = row_pps_dwt(root, frag)
        gnss_ns = row_gnss_ns(root, frag)
        edge_dwt = lane_edge_dwt(root, frag, lane)
        ocxo_ns = lane_ns(root, frag, lane)
        src = lane_source(root, frag, lane)
        projected = lane_projected_valid(root, frag, lane)
        p_meas = lane_projected_minus_measured(root, frag, lane)
        pres_valid, pres_gnss, pres_clock, pub_res = lane_pps_residual(root, frag, lane)
        act, pred, sr = lane_prediction(frag, lane)
        target_delta, target_remaining = lane_projection_extra(root, frag, lane)

        if prev_pps is not None and pps != prev_pps + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps:,} → {pps:,} ---")
            prev_pps_dwt = None
            prev_edge_dwt = None
            prev_gnss_ns = None
            prev_ocxo_ns = None

        p_delta = delta_u32(pps_dwt, prev_pps_dwt) if pps_dwt is not None and prev_pps_dwt is not None else None
        e_delta = delta_u32(edge_dwt, prev_edge_dwt) if edge_dwt is not None and prev_edge_dwt is not None else None
        gnss_delta = gnss_ns - prev_gnss_ns if gnss_ns is not None and prev_gnss_ns is not None else None
        ocxo_delta = ocxo_ns - prev_ocxo_ns if ocxo_ns is not None and prev_ocxo_ns is not None else None
        calc_res = ocxo_delta - gnss_delta if ocxo_delta is not None and gnss_delta is not None else None
        residual_delta = calc_res - pub_res if calc_res is not None and pub_res is not None else None
        offset = ocxo_ns - gnss_ns if ocxo_ns is not None and gnss_ns is not None else None

        if projected is True:
            projected_rows += 1
        elif projected is False:
            fallback_rows += 1

        if residual_delta is not None:
            residual_delta_stats.update(float(residual_delta))
            if residual_delta != 0:
                residual_mismatch_rows += 1
        if calc_res is not None:
            calc_res_stats.update(float(calc_res))
        if pub_res is not None and pres_valid is True:
            pub_res_stats.update(float(pub_res))
        if offset is not None:
            offset_stats.update(float(offset))
        if p_delta is not None:
            pps_dwt_delta_stats.update(float(p_delta))
        if e_delta is not None:
            edge_dwt_delta_stats.update(float(e_delta))
        if ocxo_delta is not None:
            ocxo_interval_stats.update(float(ocxo_delta))
        if sr is not None:
            static_res_stats.update(float(sr))
        if p_meas is not None:
            proj_measured_stats.update(float(p_meas))

        table_rows.append({
            "pps": f"{pps:,d}",
            "pps_dwt": fmt_int(pps_dwt),
            "p_delta": fmt_int(p_delta),
            "gnss_ns": fmt_int(gnss_ns),
            "edge_dwt": fmt_int(edge_dwt),
            "e_delta": fmt_int(e_delta),
            "ocxo_ns": fmt_int(ocxo_ns),
            "ocxo_delta": fmt_int(ocxo_delta),
            "calc": fmt_int(calc_res, signed=True),
            "pub": fmt_int(pub_res, signed=True),
            "r_delta": fmt_int(residual_delta, signed=True),
            "act": fmt_int(act),
            "pred": fmt_int(pred),
            "sr": fmt_int(sr, signed=True),
            "p_minus_meas": fmt_int(p_meas, signed=True),
            "tgt": fmt_int(target_delta),
            "trem": fmt_int(target_remaining),
            "src": src or "---",
            "proj": fmt_bool(projected),
        })

        prev_pps = pps
        prev_pps_dwt = pps_dwt if pps_dwt is not None else prev_pps_dwt
        prev_edge_dwt = edge_dwt if edge_dwt is not None else prev_edge_dwt
        prev_gnss_ns = gnss_ns if gnss_ns is not None else prev_gnss_ns
        prev_ocxo_ns = ocxo_ns if ocxo_ns is not None else prev_ocxo_ns

        shown += 1
        if limit and shown >= limit:
            break

    print_right_justified_table(headers, table_rows)

    print()
    print(f"Rows shown:              {shown:,}")
    print(f"Gaps:                    {gaps:,}")
    print(f"Projected rows:          {projected_rows:,}")
    print(f"Fallback/nonproject rows:{fallback_rows:>9,d}")
    print(f"Residual mismatches:     {residual_mismatch_rows:,}")
    print()

    print("Summary")
    print("═══════")
    print_stats("PPS DWT interval", pps_dwt_delta_stats, "cycles", 3)
    print_stats("OCXO edge-DWT interval", edge_dwt_delta_stats, "cycles", 3)
    print_stats("OCXO public ns interval", ocxo_interval_stats, "ns", 3)
    print_stats("Computed PPS residual", calc_res_stats, "ns", 3)
    print_stats("Published PPS residual", pub_res_stats, "ns", 3)
    print_stats("Computed minus published residual", residual_delta_stats, "ns", 3)
    print_stats("OCXO minus GNSS offset", offset_stats, "ns", 3)
    print_stats("Static prediction residual", static_res_stats, "cycles", 3)
    print_stats("Projected minus measured", proj_measured_stats, "ns", 3)
    print()

    print("Interpretation guards")
    print("═════════════════════")
    if residual_mismatch_rows == 0 and calc_res_stats.n > 0 and pub_res_stats.n > 0:
        print("  ✓ Row-delta residual math agrees with TIMEBASE pps_residual on every comparable row.")
    elif residual_mismatch_rows > 0:
        print("  ! calc_res differs from published pps_residual on at least one row; inspect rΔ.")
    if fallback_rows == 0 and projected_rows > 0:
        print("  ✓ All rows used PPS_PROJECTED authority for this lane.")
    elif fallback_rows > 0:
        print("  ! Some rows used fallback/nonprojected authority; do not treat those residuals as science-grade.")
    print("  • tgt/trem are placeholders until TIMEBASE publishes projection target_delta/remaining cycles.")
    print()


def analyze(campaign: str, lane_arg: str = "ocxo1", limit: int = 0) -> None:
    lanes = VALID_LANES.get(lane_arg.lower())
    if lanes is None:
        raise SystemExit("lane must be ocxo1, ocxo2, or both")

    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    for i, lane in enumerate(lanes):
        if i:
            print("\n")
        render_lane(campaign, rows, lane, limit)


def main(argv: Sequence[str] = sys.argv) -> None:
    if len(argv) < 2:
        print("Usage: ocxo_pps_projection_report_v4 <campaign> [ocxo1|ocxo2|both] [limit]")
        raise SystemExit(1)

    campaign = argv[1]
    lane = argv[2] if len(argv) >= 3 else "ocxo1"
    limit = int(argv[3]) if len(argv) >= 4 else 0
    analyze(campaign, lane, limit)


if __name__ == "__main__":
    main()
