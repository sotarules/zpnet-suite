"""
ZPNet Raw Cycles — compact prediction audit.

Reads TIMEBASE rows for a campaign and prints a compact one-second DWT-cycle
audit for four lanes:

  • PPS    — physical GPIO witness DWT, static prediction = previous PPS actual
  • VCLOCK — raw-cycle surface is the lattice PPS/VCLOCK DWT edge and
             firmware static prediction audit
  • OCXO1  — compact firmware prediction audit from TIMEBASE, plus optional
             Alpha-forensics DWT reconstruction when last_event_dwt is present
  • OCXO2  — compact firmware prediction audit from TIMEBASE, plus optional
             Alpha-forensics DWT reconstruction when last_event_dwt is present

This report intentionally removes the older modulo/transport clutter.
It focuses on actual cycles, static prediction, dynamic prediction, residuals,
and the scalar PPS→VCLOCK phase offset published as pps_vclock_phase_cycles.
For PPS there is no dynamic prediction surface; the dynamic columns are shown
as "---".

For OCXO lanes, the firmware prediction payload is now cross-checked against
Alpha forensics when available:

  forensic_actual = alpha_event.last_event_dwt[n] - alpha_event.last_event_dwt[n-1]
  forensic_delta  = forensic_actual - firmware_prediction_actual

A forensic_delta of zero means the prediction payload and Alpha forensic DWT
edge subtraction agree exactly.

OCXO counter32 surface (cdelta):

  Each TIMEBASE row carries the Alpha-forensics counter32 delta between
  consecutive OCXO 1 Hz events:

    o1_cdelta = ocxo1_forensics_counter32_delta_since_previous_event
    o2_cdelta = ocxo2_forensics_counter32_delta_since_previous_event

  These deltas are the synthetic VCLOCK-counter32 advance Alpha applies for
  each lane per 1 Hz event.  Under the OCXO cadence ladder, each 1 Hz event is
  TICKS_PER_SECOND_EVENT_OCXO × OCXO_INTERVAL_COUNTS = 10,000,000 OCXO ticks
  apart by construction, so the expected value is a flat 10,000,000.  Any
  deviation indicates a synthetic-counter advance that did not match the
  programmed cadence — useful for distinguishing firmware-asserted bookkeeping
  from any real OCXO/QTimer measurement variation.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit]
    python raw_cycles.py <campaign_name> [limit]
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

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


def _prediction(frag: Dict[str, Any]) -> Dict[str, Any]:
    pred = frag.get("prediction")
    return pred if isinstance(pred, dict) else {}


def _gnss(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    gnss = root.get("gnss")
    return gnss if isinstance(gnss, dict) else {}


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


def _as_bool(v: Any) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "y")
    return bool(v)


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


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _first_path_int(*paths: Tuple[Dict[str, Any], Tuple[str, ...]]) -> Optional[int]:
    for obj, path in paths:
        out = _as_int(_nested_get(obj, *path))
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


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _print_welford(name: str, w: Welford, unit: str = "cycles",
                   decimals: int = 3) -> None:
    if w.n == 0:
        print(f"  {name:<42s} no samples")
        return
    print(
        f"  {name:<42s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+12,.{decimals}f} {unit:<6s}  "
        f"sd={w.stddev:10,.{decimals}f}  "
        f"se={w.stderr:9,.{decimals}f}  "
        f"min={w.min_val:+12,.{decimals}f}  "
        f"max={w.max_val:+12,.{decimals}f}"
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


def vclock_preferred_dwt_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Tuple[Optional[int], str]:
    lattice_dwt = _first_int(
        frag.get("dwt_at_pps_vclock"),
        root.get("dwt_at_pps_vclock"),
    )
    if lattice_dwt is not None:
        return lattice_dwt, "lattice"

    # Legacy fallback for older TIMEBASE rows captured during the temporary
    # phase-estimate schema.  New rows publish only pps_vclock_phase_cycles,
    # not a separate estimated PPS/VCLOCK DWT coordinate.
    phase_estimate_dwt = _first_int(
        frag.get("pps_vclock_phase_estimated_dwt_at_edge"),
        root.get("pps_vclock_phase_estimated_dwt_at_edge"),
    )
    if phase_estimate_dwt is not None:
        return phase_estimate_dwt, "phase-old"

    return None, "---"


def pps_vclock_phase_cycles_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    """Return the scalar PPS→VCLOCK phase offset in DWT cycles.

    New firmware publishes exactly one phase field: pps_vclock_phase_cycles.
    It is the DWT-cycle offset within one 10 MHz VCLOCK cell and should be
    less than one VCLOCK tick worth of DWT cycles.  Older rows may lack this
    scalar; callers can fall back to reconstructing from DWT endpoints.
    """
    return _first_int(
        frag.get("pps_vclock_phase_cycles"),
        root.get("pps_vclock_phase_cycles"),
    )


def ocxo_forensic_last_event_dwt_from_schema(root: Dict[str, Any],
                                               frag: Dict[str, Any],
                                               lane: str) -> Optional[int]:
    """Return Alpha forensic last_event_dwt for an OCXO lane, if present.

    TIMEBASE schema has moved over time, so accept both nested report-style
    objects and flat migration keys.  This value is deliberately the Alpha
    event DWT endpoint, not a live hardware counter or projected report DWT.
    """
    flat_keys = (
        f"{lane}_alpha_event_last_event_dwt",
        f"{lane}_forensics_last_event_dwt",
        f"{lane}_last_event_dwt",
        f"{lane}_event_dwt",
        f"{lane}_dwt_at_edge",
    )

    flat = _first_int(*(frag.get(k) for k in flat_keys),
                      *(root.get(k) for k in flat_keys))
    if flat is not None:
        return flat

    # Report-style nested shapes, if TIMEBASE is carrying CLOCKS forensics.
    return _first_path_int(
        (frag, ("clock_forensics", lane, "alpha_event", "last_event_dwt")),
        (root, ("clock_forensics", lane, "alpha_event", "last_event_dwt")),
        (frag, ("forensics", lane, "alpha_event", "last_event_dwt")),
        (root, ("forensics", lane, "alpha_event", "last_event_dwt")),
        (frag, (lane, "alpha_event", "last_event_dwt")),
        (root, (lane, "alpha_event", "last_event_dwt")),
        (frag, ("clock_forensics", lane, "last_event_dwt")),
        (root, ("clock_forensics", lane, "last_event_dwt")),
    )


def ocxo_forensic_counter32_delta_from_schema(root: Dict[str, Any],
                                                frag: Dict[str, Any],
                                                lane: str) -> Optional[int]:
    """Return Alpha forensic counter32 delta since previous event for an OCXO lane.

    This is the synthetic VCLOCK-counter32 advance Alpha applied between this
    1 Hz event and the previous one.  Under the OCXO cadence ladder, each 1 Hz
    event is TICKS_PER_SECOND_EVENT_OCXO × OCXO_INTERVAL_COUNTS = 10,000,000
    OCXO ticks apart by construction, so this value is expected to be a flat
    10,000,000 every row.
    """
    flat_keys = (
        f"{lane}_forensics_counter32_delta_since_previous_event",
        f"{lane}_counter32_delta_since_previous_event",
    )
    flat = _first_int(*(frag.get(k) for k in flat_keys),
                      *(root.get(k) for k in flat_keys))
    if flat is not None:
        return flat

    return _first_path_int(
        (frag, ("clock_forensics", lane, "alpha_event",
                "counter32_delta_since_previous_event")),
        (root, ("clock_forensics", lane, "alpha_event",
                "counter32_delta_since_previous_event")),
        (frag, ("forensics", lane, "counter32_delta_since_previous_event")),
        (root, ("forensics", lane, "counter32_delta_since_previous_event")),
    )


def lane_prediction(pred: Dict[str, Any], lane: str) -> Tuple[Optional[int], Optional[int], Optional[int], Optional[int], Optional[int]]:
    actual = _first_int(pred.get(f"{lane}_actual_cycles"))
    static = _first_int(pred.get(f"{lane}_static_prediction_cycles"))
    dynamic = _first_int(pred.get(f"{lane}_dynamic_final_prediction_cycles"))
    static_res = _first_int(pred.get(f"{lane}_static_residual_cycles"))
    dynamic_res = _first_int(pred.get(f"{lane}_dynamic_residual_cycles"))

    if static_res is None and actual is not None and static is not None:
        static_res = actual - static
    if dynamic_res is None and actual is not None and dynamic is not None:
        dynamic_res = actual - dynamic

    return actual, static, static_res, dynamic, dynamic_res


def add_optional(w: Welford, v: Optional[int]) -> None:
    if v is not None:
        w.update(float(v))


def analyze(campaign: str, limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    print(f"Campaign: {campaign}  ({len(rows):,} rows)")
    print()
    print("Raw cycle surfaces:")
    print("  PPS    actual = physical PPS GPIO DWT delta; static prediction = previous PPS actual")
    print("  VCLOCK actual = lattice PPS/VCLOCK DWT delta; phase column uses scalar pps_vclock_phase_cycles")
    print("  OCXO   actual/static/dynamic/residual = compact firmware prediction audit")
    print("         fact/fΔ = Alpha-forensics DWT reconstruction and fact - actual")
    print("         cdelta  = Alpha-forensics counter32 delta since previous event")
    print("                   (expected: flat 10,000,000 by cadence-ladder construction)")
    print()

    header = (
        f"{'pps':>6s}  "
        f"{'pps_act':>13s} {'pps_stat':>13s} {'pps_sres':>9s} {'phase':>7s}  "
        f"{'v_act':>13s} {'v_stat':>13s} {'v_sres':>8s} {'v_dyn':>13s} {'v_dres':>8s} {'vsrc':>7s}  "
        f"{'o1_act':>13s} {'o1_stat':>13s} {'o1_sres':>8s} {'o1_fact':>13s} {'o1_fΔ':>8s} {'o1_dyn':>13s} {'o1_dres':>8s} {'o1_cdelta':>13s}  "
        f"{'o2_act':>13s} {'o2_stat':>13s} {'o2_sres':>8s} {'o2_fact':>13s} {'o2_fΔ':>8s} {'o2_dyn':>13s} {'o2_dres':>8s} {'o2_cdelta':>13s}  "
        f"{'dwt_ppb':>10s}"
    )
    print(header)
    print(
        f"{'─'*6}  "
        f"{'─'*13} {'─'*13} {'─'*9} {'─'*7}  "
        f"{'─'*13} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*7}  "
        f"{'─'*13} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*13}  "
        f"{'─'*13} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*13} {'─'*8} {'─'*13}  "
        f"{'─'*10}"
    )

    shown = 0
    gaps = 0

    prev_pps_count: Optional[int] = None
    prev_physical_pps_dwt: Optional[int] = None
    prev_vclock_dwt: Optional[int] = None
    prev_pps_actual: Optional[int] = None
    prev_ocxo1_forensic_dwt: Optional[int] = None
    prev_ocxo2_forensic_dwt: Optional[int] = None

    stats: Dict[str, Welford] = {
        "pps_actual": Welford(),
        "pps_static_residual": Welford(),
        "pps_to_vclock_phase": Welford(),
        "vclock_actual": Welford(),
        "vclock_static_residual": Welford(),
        "vclock_dynamic_residual": Welford(),
        "ocxo1_actual": Welford(),
        "ocxo1_static_residual": Welford(),
        "ocxo1_forensic_actual": Welford(),
        "ocxo1_forensic_delta": Welford(),
        "ocxo1_dynamic_residual": Welford(),
        "ocxo1_counter32_delta": Welford(),
        "ocxo2_actual": Welford(),
        "ocxo2_static_residual": Welford(),
        "ocxo2_forensic_actual": Welford(),
        "ocxo2_forensic_delta": Welford(),
        "ocxo2_dynamic_residual": Welford(),
        "ocxo2_counter32_delta": Welford(),
    }

    coverage = {
        "rows": 0,
        "pps_actual": 0,
        "vclock_phase_actual": 0,
        "vclock_lattice_actual": 0,
        "vclock_prediction": 0,
        "ocxo1_prediction": 0,
        "ocxo1_forensic_actual": 0,
        "ocxo1_forensic_delta": 0,
        "ocxo1_counter32_delta": 0,
        "ocxo2_prediction": 0,
        "ocxo2_forensic_actual": 0,
        "ocxo2_forensic_delta": 0,
        "ocxo2_counter32_delta": 0,
    }

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        pred = _prediction(frag)
        pps_count = _first_int(
            root.get("pps_count"),
            frag.get("pps_count"),
            frag.get("teensy_pps_vclock_count"),
            frag.get("teensy_pps_count"),
        )
        if pps_count is None:
            continue

        if prev_pps_count is not None and pps_count != prev_pps_count + 1:
            gaps += 1
            print(f"{'':>6s}  --- gap {prev_pps_count:,} → {pps_count:,} ---")
            prev_physical_pps_dwt = None
            prev_vclock_dwt = None
            prev_pps_actual = None
            prev_ocxo1_forensic_dwt = None
            prev_ocxo2_forensic_dwt = None

        physical_pps_dwt = physical_pps_dwt_from_schema(root, frag)
        vclock_dwt, vclock_source = vclock_preferred_dwt_from_schema(root, frag)

        pps_to_vclock_phase: Optional[int] = pps_vclock_phase_cycles_from_schema(root, frag)
        if pps_to_vclock_phase is None and physical_pps_dwt is not None and vclock_dwt is not None:
            # Legacy fallback for rows that predate the scalar field.
            pps_to_vclock_phase = abs(_signed_delta_u32(vclock_dwt, physical_pps_dwt))

        pps_actual: Optional[int] = None
        if physical_pps_dwt is not None and prev_physical_pps_dwt is not None:
            pps_actual = _delta_u32(physical_pps_dwt, prev_physical_pps_dwt)

        pps_static = prev_pps_actual
        pps_static_res = (
            pps_actual - pps_static
            if pps_actual is not None and pps_static is not None
            else None
        )

        vclock_actual_from_dwt: Optional[int] = None
        if vclock_dwt is not None and prev_vclock_dwt is not None:
            vclock_actual_from_dwt = _delta_u32(vclock_dwt, prev_vclock_dwt)

        vclock_gamma_actual, vclock_static, vclock_static_res, vclock_dynamic, vclock_dynamic_res = (
            lane_prediction(pred, "vclock")
        )
        # The report's VCLOCK actual should reflect the preferred raw DWT surface.
        # If this row cannot compute a DWT delta, fall back to Gamma/firmware actual.
        vclock_actual = vclock_actual_from_dwt
        if vclock_actual is None:
            vclock_actual = _first_int(
                vclock_gamma_actual,
                frag.get("dwt_cycles_between_pps_vclock"),
                frag.get("vclock_dwt_cycles_between_edges"),
            )

        # If we used the new raw actual, recompute residuals against Gamma predictions.
        if vclock_actual is not None and vclock_static is not None:
            vclock_static_res = vclock_actual - vclock_static
        if vclock_actual is not None and vclock_dynamic is not None:
            vclock_dynamic_res = vclock_actual - vclock_dynamic

        ocxo1_actual, ocxo1_static, ocxo1_static_res, ocxo1_dynamic, ocxo1_dynamic_res = (
            lane_prediction(pred, "ocxo1")
        )
        ocxo2_actual, ocxo2_static, ocxo2_static_res, ocxo2_dynamic, ocxo2_dynamic_res = (
            lane_prediction(pred, "ocxo2")
        )

        ocxo1_forensic_dwt = ocxo_forensic_last_event_dwt_from_schema(root, frag, "ocxo1")
        ocxo2_forensic_dwt = ocxo_forensic_last_event_dwt_from_schema(root, frag, "ocxo2")

        ocxo1_forensic_actual: Optional[int] = None
        if ocxo1_forensic_dwt is not None and prev_ocxo1_forensic_dwt is not None:
            ocxo1_forensic_actual = _delta_u32(ocxo1_forensic_dwt, prev_ocxo1_forensic_dwt)
        ocxo1_forensic_delta = (
            ocxo1_forensic_actual - ocxo1_actual
            if ocxo1_forensic_actual is not None and ocxo1_actual is not None
            else None
        )

        ocxo2_forensic_actual: Optional[int] = None
        if ocxo2_forensic_dwt is not None and prev_ocxo2_forensic_dwt is not None:
            ocxo2_forensic_actual = _delta_u32(ocxo2_forensic_dwt, prev_ocxo2_forensic_dwt)
        ocxo2_forensic_delta = (
            ocxo2_forensic_actual - ocxo2_actual
            if ocxo2_forensic_actual is not None and ocxo2_actual is not None
            else None
        )

        ocxo1_counter32_delta = ocxo_forensic_counter32_delta_from_schema(root, frag, "ocxo1")
        ocxo2_counter32_delta = ocxo_forensic_counter32_delta_from_schema(root, frag, "ocxo2")

        dwt_ppb = _as_float(frag.get("dwt_ppb"))
        coverage["rows"] += 1
        if pps_actual is not None:
            coverage["pps_actual"] += 1
        if vclock_actual is not None and vclock_source == "phase":
            coverage["vclock_phase_actual"] += 1
        if vclock_actual is not None and vclock_source == "lattice":
            coverage["vclock_lattice_actual"] += 1
        if vclock_gamma_actual is not None or vclock_static is not None or vclock_dynamic is not None:
            coverage["vclock_prediction"] += 1
        if ocxo1_actual is not None or ocxo1_static is not None or ocxo1_dynamic is not None:
            coverage["ocxo1_prediction"] += 1
        if ocxo1_forensic_actual is not None:
            coverage["ocxo1_forensic_actual"] += 1
        if ocxo1_forensic_delta is not None:
            coverage["ocxo1_forensic_delta"] += 1
        if ocxo1_counter32_delta is not None:
            coverage["ocxo1_counter32_delta"] += 1
        if ocxo2_actual is not None or ocxo2_static is not None or ocxo2_dynamic is not None:
            coverage["ocxo2_prediction"] += 1
        if ocxo2_forensic_actual is not None:
            coverage["ocxo2_forensic_actual"] += 1
        if ocxo2_forensic_delta is not None:
            coverage["ocxo2_forensic_delta"] += 1
        if ocxo2_counter32_delta is not None:
            coverage["ocxo2_counter32_delta"] += 1

        add_optional(stats["pps_actual"], pps_actual)
        add_optional(stats["pps_static_residual"], pps_static_res)
        add_optional(stats["pps_to_vclock_phase"], pps_to_vclock_phase)
        add_optional(stats["vclock_actual"], vclock_actual)
        add_optional(stats["vclock_static_residual"], vclock_static_res)
        add_optional(stats["vclock_dynamic_residual"], vclock_dynamic_res)
        add_optional(stats["ocxo1_actual"], ocxo1_actual)
        add_optional(stats["ocxo1_static_residual"], ocxo1_static_res)
        add_optional(stats["ocxo1_forensic_actual"], ocxo1_forensic_actual)
        add_optional(stats["ocxo1_forensic_delta"], ocxo1_forensic_delta)
        add_optional(stats["ocxo1_dynamic_residual"], ocxo1_dynamic_res)
        add_optional(stats["ocxo1_counter32_delta"], ocxo1_counter32_delta)
        add_optional(stats["ocxo2_actual"], ocxo2_actual)
        add_optional(stats["ocxo2_static_residual"], ocxo2_static_res)
        add_optional(stats["ocxo2_forensic_actual"], ocxo2_forensic_actual)
        add_optional(stats["ocxo2_forensic_delta"], ocxo2_forensic_delta)
        add_optional(stats["ocxo2_dynamic_residual"], ocxo2_dynamic_res)
        add_optional(stats["ocxo2_counter32_delta"], ocxo2_counter32_delta)

        print(
            f"{pps_count:>6d}  "
            f"{_fmt_int(pps_actual, 13)} {_fmt_int(pps_static, 13)} {_fmt_int(pps_static_res, 9, signed=True)} {_fmt_int(pps_to_vclock_phase, 7)}  "
            f"{_fmt_int(vclock_actual, 13)} {_fmt_int(vclock_static, 13)} {_fmt_int(vclock_static_res, 8, signed=True)} "
            f"{_fmt_int(vclock_dynamic, 13)} {_fmt_int(vclock_dynamic_res, 8, signed=True)} {_fmt_str(vclock_source, 7)}  "
            f"{_fmt_int(ocxo1_actual, 13)} {_fmt_int(ocxo1_static, 13)} {_fmt_int(ocxo1_static_res, 8, signed=True)} "
            f"{_fmt_int(ocxo1_forensic_actual, 13)} {_fmt_int(ocxo1_forensic_delta, 8, signed=True)} "
            f"{_fmt_int(ocxo1_dynamic, 13)} {_fmt_int(ocxo1_dynamic_res, 8, signed=True)} "
            f"{_fmt_int(ocxo1_counter32_delta, 13)}  "
            f"{_fmt_int(ocxo2_actual, 13)} {_fmt_int(ocxo2_static, 13)} {_fmt_int(ocxo2_static_res, 8, signed=True)} "
            f"{_fmt_int(ocxo2_forensic_actual, 13)} {_fmt_int(ocxo2_forensic_delta, 8, signed=True)} "
            f"{_fmt_int(ocxo2_dynamic, 13)} {_fmt_int(ocxo2_dynamic_res, 8, signed=True)} "
            f"{_fmt_int(ocxo2_counter32_delta, 13)}  "
            f"{_fmt_float(dwt_ppb, 10, 3, signed=True)}"
        )

        prev_pps_count = pps_count
        if physical_pps_dwt is not None:
            prev_physical_pps_dwt = physical_pps_dwt
        if vclock_dwt is not None:
            prev_vclock_dwt = vclock_dwt
        if pps_actual is not None:
            prev_pps_actual = pps_actual
        if ocxo1_forensic_dwt is not None:
            prev_ocxo1_forensic_dwt = ocxo1_forensic_dwt
        if ocxo2_forensic_dwt is not None:
            prev_ocxo2_forensic_dwt = ocxo2_forensic_dwt

        shown += 1
        if limit and shown >= limit:
            break

    print()
    print(f"Rows shown: {shown:,}")
    print(f"Gaps:       {gaps:,}")
    print()

    print("Schema coverage")
    print("═══════════════")
    print(f"  rows                         = {coverage['rows']:,}")
    print(f"  PPS actual cycles             = {coverage['pps_actual']:,}")
    print(f"  VCLOCK legacy phase actual    = {coverage['vclock_phase_actual']:,}")
    print(f"  VCLOCK lattice actual         = {coverage['vclock_lattice_actual']:,}")
    print(f"  VCLOCK prediction rows        = {coverage['vclock_prediction']:,}")
    print(f"  OCXO1 prediction rows         = {coverage['ocxo1_prediction']:,}")
    print(f"  OCXO1 forensic actual rows    = {coverage['ocxo1_forensic_actual']:,}")
    print(f"  OCXO1 forensic delta rows     = {coverage['ocxo1_forensic_delta']:,}")
    print(f"  OCXO1 counter32 delta rows    = {coverage['ocxo1_counter32_delta']:,}")
    print(f"  OCXO2 prediction rows         = {coverage['ocxo2_prediction']:,}")
    print(f"  OCXO2 forensic actual rows    = {coverage['ocxo2_forensic_actual']:,}")
    print(f"  OCXO2 forensic delta rows     = {coverage['ocxo2_forensic_delta']:,}")
    print(f"  OCXO2 counter32 delta rows    = {coverage['ocxo2_counter32_delta']:,}")
    print()

    print("Summary")
    print("═══════")
    _print_welford("PPS actual cycles", stats["pps_actual"])
    _print_welford("PPS static residual", stats["pps_static_residual"])
    _print_welford("PPS→VCLOCK phase offset", stats["pps_to_vclock_phase"])
    _print_welford("VCLOCK actual cycles", stats["vclock_actual"])
    _print_welford("VCLOCK static residual", stats["vclock_static_residual"])
    _print_welford("VCLOCK dynamic residual", stats["vclock_dynamic_residual"])
    _print_welford("OCXO1 actual cycles", stats["ocxo1_actual"])
    _print_welford("OCXO1 static residual", stats["ocxo1_static_residual"])
    _print_welford("OCXO1 forensic actual", stats["ocxo1_forensic_actual"])
    _print_welford("OCXO1 forensic delta", stats["ocxo1_forensic_delta"])
    _print_welford("OCXO1 dynamic residual", stats["ocxo1_dynamic_residual"])
    _print_welford("OCXO1 counter32 delta (ticks)", stats["ocxo1_counter32_delta"], unit="ticks")
    _print_welford("OCXO2 actual cycles", stats["ocxo2_actual"])
    _print_welford("OCXO2 static residual", stats["ocxo2_static_residual"])
    _print_welford("OCXO2 forensic actual", stats["ocxo2_forensic_actual"])
    _print_welford("OCXO2 forensic delta", stats["ocxo2_forensic_delta"])
    _print_welford("OCXO2 dynamic residual", stats["ocxo2_dynamic_residual"])
    _print_welford("OCXO2 counter32 delta (ticks)", stats["ocxo2_counter32_delta"], unit="ticks")
    print()

    print("Notes")
    print("═════")
    print("  • phase is pps_vclock_phase_cycles when present.")
    print("  • phase fallback reconstructs abs(VCLOCK_DWT - PPS_DWT) for legacy rows only.")
    print("  • vsrc=phase-old means an older pps_vclock_phase_estimated_dwt_at_edge row supplied VCLOCK actual.")
    print("  • PPS has no dynamic prediction surface; pps_sres is actual minus previous PPS actual.")
    print("  • VCLOCK residuals are recomputed against firmware predictions when the preferred DWT actual is used.")
    print("  • OCXO fact columns reconstruct cycles from Alpha forensic last_event_dwt when present.")
    print("  • OCXO fΔ = forensic_actual - prediction_actual; zero means exact agreement.")
    print("  • OCXO cdelta = ocxo*_forensics_counter32_delta_since_previous_event from the fragment.")
    print("    Expected value is a flat 10,000,000 by cadence-ladder construction; any deviation")
    print("    indicates a synthetic-counter advance that did not match the programmed cadence.")
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