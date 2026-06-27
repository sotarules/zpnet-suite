"""
ZPNet Raw Nanoseconds — DWT-edge -> GNSS-ns projection audit.

Reads TIMEBASE rows for a campaign and prints a compact one-row-per-record
report for one selected clock.  The report projects the selected clock's
subscriber-facing DWT-at-edge into GNSS nanoseconds using the row's canonical
PPS/VCLOCK anchor:

    edge_ns = pps_vclock_ns + signed_dwt(edge_dwt - pps_vclock_dwt) * 1e9 / cps

where cps is the row's effective DWT cycles per GNSS second.  The projected
edge times are then differenced to produce the selected clock's one-second
period in GNSS nanoseconds.

Usage:
    python -m zpnet.tests.raw_nanoseconds <campaign_name> [clock] [limit]
    python raw_nanoseconds.py <campaign_name> --clock OCXO2 --limit 300
    python raw_nanoseconds.py <campaign_name> OCXO1 200
    python raw_nanoseconds.py <campaign_name> OCXO1 --no-cum
    python raw_nanoseconds.py <campaign_name> OCXO1 --science-debug --mismatch-limit 40

Clock filter:
    PPS, VCLOCK, OCXO1, OCXO2

Column doctrine:
    pps       campaign PPS/VCLOCK row identity.
    pps_ns    canonical PPS/VCLOCK GNSS nanosecond value for this row.
    dwt_edge  sacred subscriber-facing DWT-at-edge for the selected clock.
              For PPS this is the physical PPS witness DWT.
    cyc       DWT cycles between this selected edge and the prior selected edge.
    prior_ns  projected GNSS ns at the prior selected edge.
    edge_ns   projected GNSS ns at this selected edge.
    ns_Δ      GNSS nanoseconds between selected edges.
    res_fast  1e9 - ns_Δ.  Positive means the selected clock is fast.
    tau_1s    frequency-ratio tau for this one selected-clock second:
              1e9 / ns_Δ.  This is the reciprocal of the period ratio.
    ppb_1s    (tau_1s - 1) * 1e9.
    tau_edge  running cumulative tau from summed selected-clock seconds over
              summed projected GNSS nanoseconds in this report segment.
    ppb_edge  (tau_edge - 1) * 1e9.
    tau_tb    Teensy/TIMEBASE campaign-total tau from fragment.stats.<clock>.tau.
    ppb_tb    Teensy/TIMEBASE campaign-total ppb from fragment.stats.<clock>.ppb.
    ppb_Δ     ppb_edge - ppb_tb.  This intentionally compares edge-derived
              report cumulative PPB against the firmware campaign-total PPB.

This report intentionally does not compute Welfords.  It is a projection,
residual, and TIMEBASE stats cross-check report.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
CLOCKS = ("PPS", "VCLOCK", "OCXO1", "OCXO2")
LANE_KEYS = {"VCLOCK": "vclock", "OCXO1": "ocxo1", "OCXO2": "ocxo2"}
LANE_MICRO_PREFIXES = {"vclock": "v", "ocxo1": "o1", "ocxo2": "o2"}


# -----------------------------------------------------------------------------
# Database and schema helpers
# -----------------------------------------------------------------------------


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    """Fetch TIMEBASE rows in campaign PPS order."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'teensy_pps_vclock_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'teensy_pps_vclock_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'fragment', 'teensy_pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'fragment', 'pps_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'fragment', 'teensy_pps_vclock_count'), '')::bigint,
                NULLIF(jsonb_extract_path_text(payload::jsonb, 'payload', 'fragment', 'teensy_pps_count'), '')::bigint
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


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics_root(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    forensic = root.get("forensics")
    return forensic if isinstance(forensic, dict) else {}


def _nested_get(obj: Dict[str, Any], *path: str) -> Any:
    cur: Any = obj
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


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


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 6,
               signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


# -----------------------------------------------------------------------------
# TIMEBASE field extraction
# -----------------------------------------------------------------------------


def pps_count_from_schema(root: Dict[str, Any],
                          frag: Dict[str, Any],
                          forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
        forensic.get("teensy_pps_vclock_count"),
        forensic.get("teensy_pps_count"),
    )


def pps_vclock_ns_from_schema(root: Dict[str, Any],
                              frag: Dict[str, Any],
                              forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("gnss_ns"),
        frag.get("pps_vclock_gnss_ns_at_edge"),
        forensic.get("gnss_ns"),
        forensic.get("pps_vclock_gnss_ns_at_edge"),
        root.get("gnss_ns"),
        _nested_get(frag, "gnss", "ns"),
        _nested_get(frag, "vclock", "ns"),
    )


def pps_vclock_dwt_from_schema(root: Dict[str, Any],
                               frag: Dict[str, Any],
                               forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("dwt_at_pps_vclock"),
        forensic.get("dwt_at_pps_vclock"),
        root.get("dwt_at_pps_vclock"),
        _nested_get(forensic, "pps_vclock_edge", "authority_dwt_at_edge"),
        _nested_get(frag, "pps_vclock_edge", "authority_dwt_at_edge"),
    )


def dwt_cps_from_schema(root: Dict[str, Any],
                        frag: Dict[str, Any],
                        forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("dwt_cycles_between_pps_vclock"),
        forensic.get("dwt_cycles_between_pps_vclock"),
        root.get("dwt_cycles_between_pps_vclock"),
        _nested_get(frag, "dwt", "cycles_between_pps_vclock"),
        _nested_get(forensic, "pps_vclock_edge", "effective_dwt_cycles_per_second"),
        _nested_get(forensic, "pps_vclock_edge", "dwt_cycles_between_edges"),
    )


def physical_pps_dwt_from_schema(root: Dict[str, Any],
                                 frag: Dict[str, Any],
                                 forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("pps_dwt_at_edge"),
        forensic.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        _nested_get(forensic, "pps_vclock_edge", "pps_dwt_at_edge"),
        _nested_get(frag, "pps_vclock_edge", "pps_dwt_at_edge"),
        _nested_get(frag, "pps", "dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
        frag.get("pps_edge_dwt_at_edge"),
    )


def _micro_first_int(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str,
                     *suffixes: str) -> Optional[int]:
    prefix = LANE_MICRO_PREFIXES.get(lane)
    if not prefix:
        return None

    values: List[Any] = []
    for suffix in suffixes:
        key = f"{prefix}_{suffix}"
        for source in (forensic, frag, root):
            if isinstance(source, dict):
                values.append(source.get(key))
    return _first_int(*values)


def lane_alpha_forensics(root: Dict[str, Any],
                         frag: Dict[str, Any],
                         forensic: Dict[str, Any],
                         lane: str) -> Dict[str, Any]:
    """Best-effort Alpha forensics object for one lane across schema versions."""
    candidates = (
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
        _nested_get(forensic, lane, "dwt_forensics"),
        _nested_get(frag, lane, "dwt_forensics"),
        _nested_get(root, "clock_forensics", lane, "alpha_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event"),
        _nested_get(forensic, lane, "alpha_event"),
        _nested_get(frag, lane, "alpha_event"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def selected_dwt_edge(root: Dict[str, Any],
                      frag: Dict[str, Any],
                      forensic: Dict[str, Any],
                      clock: str) -> Optional[int]:
    """Return the selected clock's sacred subscriber-facing DWT-at-edge."""
    if clock == "PPS":
        return physical_pps_dwt_from_schema(root, frag, forensic)
    if clock == "VCLOCK":
        return _first_int(
            _micro_first_int(root, frag, forensic, "vclock", "pub", "used"),
            pps_vclock_dwt_from_schema(root, frag, forensic),
            _nested_get(forensic, "pps_vclock_edge", "authority_dwt_at_edge"),
        )

    lane = LANE_KEYS[clock]
    f = lane_alpha_forensics(root, frag, forensic, lane)
    return _first_int(
        _micro_first_int(root, frag, forensic, lane, "pub", "used"),
        f.get("dwt_used_at_event"),
        f.get("last_event_dwt"),
        _nested_get(f, "dwt_authority", "used_at_event"),
        _nested_get(f, "dwt", "used_at_event"),
    )


def firmware_interval_hint(root: Dict[str, Any],
                           frag: Dict[str, Any],
                           forensic: Dict[str, Any],
                           clock: str) -> Optional[int]:
    """Report-only interval hint from firmware, used only for diagnostics/coverage."""
    if clock == "PPS":
        return _first_int(
            forensic.get("pps_obs"),
            _nested_get(frag, "pps", "dwt_cycles_between_edges"),
            frag.get("pps_dwt_cycles_between_edges"),
            root.get("pps_dwt_cycles_between_edges"),
        )
    lane = "vclock" if clock == "VCLOCK" else LANE_KEYS[clock]
    f = lane_alpha_forensics(root, frag, forensic, lane)
    return _first_int(
        _micro_first_int(root, frag, forensic, lane, "pub_cyc", "eff"),
        f.get("dwt_cycles_between_edges"),
        _nested_get(f, "dwt_interval_gate", "effective_cycles"),
    )


def published_ocxo_residual(root: Dict[str, Any],
                            frag: Dict[str, Any],
                            clock: str) -> Optional[int]:
    """Optional firmware residual check source."""
    if clock not in ("OCXO1", "OCXO2"):
        return None
    lane = LANE_KEYS[clock]
    return _first_int(
        _nested_get(frag, lane, "pps_residual", "fast_residual_ns"),
        _nested_get(root, "fragment", lane, "pps_residual", "fast_residual_ns"),
    )


def stats_tau_ppb_from_schema(root: Dict[str, Any],
                              frag: Dict[str, Any],
                              clock: str) -> Tuple[Optional[float], Optional[float]]:
    """Return TIMEBASE fragment.stats.<clock>.tau/ppb."""
    if clock == "PPS":
        return None, None
    lane = LANE_KEYS[clock]
    tau = _first_float(
        _nested_get(frag, "stats", lane, "tau"),
        _nested_get(root, "fragment", "stats", lane, "tau"),
        frag.get(f"{lane}_tau"),
        root.get(f"{lane}_tau"),
    )
    ppb = _first_float(
        _nested_get(frag, "stats", lane, "ppb"),
        _nested_get(root, "fragment", "stats", lane, "ppb"),
        frag.get(f"{lane}_ppb"),
        root.get(f"{lane}_ppb"),
    )
    return tau, ppb


def public_gnss_ns_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "gnss", "ns"),
        frag.get("gnss_ns"),
        root.get("gnss_ns"),
    )


def public_clock_ns_from_schema(root: Dict[str, Any],
                                frag: Dict[str, Any],
                                clock: str) -> Optional[int]:
    gnss_ns = public_gnss_ns_from_schema(root, frag)
    if clock in ("PPS", "VCLOCK"):
        return gnss_ns
    lane = LANE_KEYS[clock]
    return _first_int(
        _nested_get(frag, lane, "ns"),
        _nested_get(frag, "gnss", f"{lane}_ns"),
        frag.get(f"{lane}_ns"),
        root.get(f"{lane}_ns"),
    )


def ledger_tau_ppb_from_schema(root: Dict[str, Any],
                               frag: Dict[str, Any],
                               clock: str) -> Tuple[Optional[float], Optional[float]]:
    """Recompute TIMEBASE campaign-total tau/ppb from public ledgers."""
    gnss_ns = public_gnss_ns_from_schema(root, frag)
    clock_ns = public_clock_ns_from_schema(root, frag, clock)
    if gnss_ns is None or clock_ns is None or gnss_ns <= 0:
        return None, None
    tau = float(clock_ns) / float(gnss_ns)
    return tau, ppb_from_tau(tau)

# -----------------------------------------------------------------------------
# TIMEBASE clock-science extraction
# -----------------------------------------------------------------------------


SCIENCE_OBJECT_KEYS = {
    "PPS": ("pps", "science"),
    "VCLOCK": ("vclock", "science"),
    "OCXO1": ("ocxo1", "science"),
    "OCXO2": ("ocxo2", "science"),
}


def science_from_schema(frag: Dict[str, Any], clock: str) -> Dict[str, Any]:
    """Return fragment.<clock>.science when present."""
    path = SCIENCE_OBJECT_KEYS.get(clock)
    if not path:
        return {}
    obj = _nested_get(frag, *path)
    return obj if isinstance(obj, dict) else {}


def science_usable(sci: Dict[str, Any], clock: str) -> bool:
    if not sci or not bool(sci.get("valid")):
        return False
    if clock in ("OCXO1", "OCXO2", "VCLOCK"):
        return bool(sci.get("antecedents_complete", False))
    return True


def _round_nearest_int(v: Optional[float]) -> Optional[int]:
    """Round like the firmware's published integer ns fields for report checks."""
    if v is None:
        return None
    if v >= 0:
        return int(math.floor(v + 0.5))
    return int(math.ceil(v - 0.5))


def _frac_to_half(v: Optional[float]) -> Optional[float]:
    """Signed distance from the nearest half-ns boundary.

    Near-zero values identify rows where equivalent calculations can choose
    adjacent integer nanoseconds if one side rounded before/after a subtraction
    or used exact vs integer antecedents.
    """
    if v is None:
        return None
    frac = abs(v) - math.floor(abs(v))
    return frac - 0.5


def _fmt_delta_flag(*deltas: Optional[int]) -> str:
    bad = [d for d in deltas if d not in (None, 0)]
    return "!" if bad else "."


def _science_edge_dwt(sci: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        sci.get("clock_published_dwt_at_edge"),
        sci.get("clock_floorline_dwt_at_edge"),
        sci.get("pps_dwt_at_edge"),
        sci.get("pps_vclock_dwt_at_edge"),
    )


def science_external_calc(sci: Dict[str, Any], clock: str) -> Dict[str, Any]:
    """Independently recompute science values from fragment.<clock>.science antecedents."""
    out: Dict[str, Any] = {}
    if not science_usable(sci, clock):
        return out

    if clock == "PPS" or sci.get("frequency_source") in ("GNSS_IDENTITY", "GNSS_IDENTITY_WITNESS"):
        out["prior_ns_exact"] = _as_float(sci.get("prior_edge_gnss_ns_exact"))
        out["prior_ns"] = _as_int(sci.get("prior_edge_gnss_ns"))
        out["edge_ns_exact"] = _first_float(
            sci.get("current_edge_gnss_ns_exact"),
            sci.get("pps_vclock_gnss_ns_at_edge"),
        )
        out["edge_ns"] = _first_int(
            sci.get("current_edge_gnss_ns"),
            sci.get("pps_vclock_gnss_ns_at_edge"),
        )
        out["ns_between_exact"] = _as_float(sci.get("gnss_interval_ns_exact"))
        out["ns_between"] = _as_int(sci.get("gnss_interval_ns"))
        out["residual_fast_exact"] = _as_float(sci.get("fast_residual_ns_exact"))
        out["residual_fast"] = _as_int(sci.get("fast_residual_ns"))
        out["tau_1s"] = _as_float(sci.get("tau_1s"))
        out["ppb_1s"] = _as_float(sci.get("ppb_1s"))
        return out

    anchor_ns = _as_float(sci.get("pps_vclock_gnss_ns_at_edge"))
    anchor_dwt = _as_int(sci.get("pps_vclock_dwt_at_edge"))
    cps = _as_int(sci.get("projection_cps_cycles"))
    edge_dwt = _science_edge_dwt(sci)
    interval_cycles = _as_int(sci.get("clock_floorline_interval_cycles"))

    if anchor_ns is None or anchor_dwt is None or cps is None or cps <= 0 or edge_dwt is None:
        return out

    delta_cycles = _signed_delta_u32(edge_dwt, anchor_dwt)
    edge_ns_exact = anchor_ns + (float(delta_cycles) * float(NS_PER_SECOND) / float(cps))
    out["edge_ns_exact"] = edge_ns_exact
    out["edge_ns"] = _round_nearest_int(edge_ns_exact)

    if interval_cycles is not None and interval_cycles > 0:
        ns_between_exact = float(interval_cycles) * float(NS_PER_SECOND) / float(cps)
        prior_ns_exact = edge_ns_exact - ns_between_exact
        residual_exact = float(NS_PER_SECOND) - ns_between_exact
        tau_1s = float(cps) / float(interval_cycles)
        ppb_1s = ppb_from_tau(tau_1s)
        out.update({
            "prior_ns_exact": prior_ns_exact,
            "prior_ns": _round_nearest_int(prior_ns_exact),
            "ns_between_exact": ns_between_exact,
            "ns_between": _round_nearest_int(ns_between_exact),
            "residual_fast_exact": residual_exact,
            "residual_fast": _round_nearest_int(residual_exact),
            "tau_1s": tau_1s,
            "ppb_1s": ppb_1s,
        })
    return out


def science_total_calc_from_running(cum_clock_ns_exact: float,
                                    cum_gnss_ns_exact: float) -> Tuple[Optional[float], Optional[float]]:
    if cum_gnss_ns_exact <= 0.0:
        return None, None
    tau = cum_clock_ns_exact / cum_gnss_ns_exact
    return tau, ppb_from_tau(tau)


def _diff_float(a: Optional[float], b: Optional[float]) -> Optional[float]:
    if a is None or b is None:
        return None
    return a - b


def _diff_int(a: Optional[int], b: Optional[int]) -> Optional[int]:
    if a is None or b is None:
        return None
    return a - b


# -----------------------------------------------------------------------------
# Projection math
# -----------------------------------------------------------------------------


def project_ns_from_dwt(*,
                        edge_dwt: int,
                        anchor_dwt: int,
                        anchor_ns: int,
                        cps: int) -> Optional[int]:
    """Project DWT edge coordinate into GNSS ns using row PPS/VCLOCK anchor."""
    if cps <= 0:
        return None
    delta_cycles = _signed_delta_u32(edge_dwt, anchor_dwt)
    mag = abs(delta_cycles)
    delta_ns = (mag * NS_PER_SECOND + cps // 2) // cps
    return int(anchor_ns + delta_ns if delta_cycles >= 0 else anchor_ns - delta_ns)


def one_second_tau_from_period_ns(ns_between: int) -> Optional[float]:
    """Frequency-ratio tau from GNSS nanoseconds per selected-clock second."""
    if ns_between <= 0:
        return None
    return NS_PER_SECOND / float(ns_between)


def ppb_from_tau(tau: Optional[float]) -> Optional[float]:
    if tau is None:
        return None
    return (tau - 1.0) * 1.0e9


# -----------------------------------------------------------------------------
# Collection and formatting
# -----------------------------------------------------------------------------


def normalize_clock(clock: Optional[str]) -> str:
    if clock is None or not str(clock).strip():
        return "OCXO1"
    c = str(clock).strip().upper()
    if c in ("P", "PPS"):
        return "PPS"
    if c in ("V", "VC", "VCLOCK"):
        return "VCLOCK"
    if c in ("O1", "OCXO1"):
        return "OCXO1"
    if c in ("O2", "OCXO2"):
        return "OCXO2"
    raise ValueError(f"unknown clock '{clock}', expected PPS, VCLOCK, OCXO1, or OCXO2")


def collect_rows(records: Iterable[Dict[str, Any]], clock: str) -> Tuple[List[Dict[str, Any]], Dict[str, int]]:
    rows: List[Dict[str, Any]] = []
    stats = {
        "records_seen": 0,
        "rows_collected": 0,
        "science_rows": 0,
        "science_missing": 0,
        "science_math_int_mismatch_rows": 0,
        "science_abs_edge_int_mismatch_rows": 0,
        "science_float_mismatch_rows": 0,
        "projection_missing": 0,
        "dwt_missing": 0,
        "anchor_missing": 0,
        "cps_missing": 0,
        "gaps": 0,
    }

    prev_pps: Optional[int] = None
    prev_dwt: Optional[int] = None
    prev_ns: Optional[int] = None
    cum_clock_ns = 0
    cum_gnss_ns = 0
    science_cum_clock_ns_exact = 0.0
    science_cum_gnss_ns_exact = 0.0

    def reset_after_gap() -> None:
        nonlocal prev_dwt, prev_ns, cum_clock_ns, cum_gnss_ns
        nonlocal science_cum_clock_ns_exact, science_cum_gnss_ns_exact
        prev_dwt = None
        prev_ns = None
        cum_clock_ns = 0
        cum_gnss_ns = 0
        science_cum_clock_ns_exact = 0.0
        science_cum_gnss_ns_exact = 0.0

    for rec in records:
        stats["records_seen"] += 1
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics_root(rec)

        pps = pps_count_from_schema(root, frag, forensic)
        if pps is None:
            continue

        if prev_pps is not None and pps != prev_pps + 1:
            stats["gaps"] += 1
            reset_after_gap()

        pps_ns = pps_vclock_ns_from_schema(root, frag, forensic)
        anchor_dwt = pps_vclock_dwt_from_schema(root, frag, forensic)
        cps = dwt_cps_from_schema(root, frag, forensic)
        tb_tau, tb_ppb = stats_tau_ppb_from_schema(root, frag, clock)
        ledger_tau, ledger_ppb = ledger_tau_ppb_from_schema(root, frag, clock)

        sci = science_from_schema(frag, clock)
        use_science = science_usable(sci, clock)
        sci_calc = science_external_calc(sci, clock) if use_science else {}

        edge_dwt: Optional[int] = None
        edge_ns: Optional[int] = None
        cycles_between: Optional[int] = None
        ns_between: Optional[int] = None
        residual_fast: Optional[int] = None
        tau_1s: Optional[float] = None
        ppb_1s: Optional[float] = None
        tau_edge: Optional[float] = None
        ppb_edge: Optional[float] = None
        ppb_edge_minus_tb: Optional[float] = None
        prior_ns_for_row: Optional[int] = None

        if use_science:
            stats["science_rows"] += 1
            edge_dwt = _science_edge_dwt(sci)
            cycles_between = _first_int(sci.get("clock_floorline_interval_cycles"), sci.get("dwt_cycles_between_edges"), sci.get("cycles_between_edges"))
            prior_ns_for_row = _first_int(sci.get("prior_edge_gnss_ns"), sci_calc.get("prior_ns"))
            edge_ns = _first_int(sci.get("current_edge_gnss_ns"), sci.get("pps_vclock_gnss_ns_at_edge") if clock == "PPS" else None, sci_calc.get("edge_ns"))
            ns_between = _first_int(sci.get("gnss_interval_ns"), sci_calc.get("ns_between"))
            residual_fast = _first_int(sci.get("fast_residual_ns"), sci_calc.get("residual_fast"))
            tau_1s = _first_float(sci.get("tau_1s"), sci_calc.get("tau_1s"))
            ppb_1s = _first_float(sci.get("ppb_1s"), sci_calc.get("ppb_1s"))

            calc_clock_interval_exact = _first_float(sci_calc.get("clock_interval_ns_exact"), sci.get("clock_interval_ns_exact"), sci.get("clock_interval_ns"))
            if calc_clock_interval_exact is None and ns_between is not None:
                calc_clock_interval_exact = float(NS_PER_SECOND)
            calc_gnss_interval_exact = _first_float(sci_calc.get("ns_between_exact"), sci.get("gnss_interval_ns_exact"), sci.get("gnss_interval_ns"))
            if calc_clock_interval_exact is not None and calc_gnss_interval_exact is not None:
                science_cum_clock_ns_exact += calc_clock_interval_exact
                science_cum_gnss_ns_exact += calc_gnss_interval_exact
                tau_edge, ppb_edge = science_total_calc_from_running(science_cum_clock_ns_exact, science_cum_gnss_ns_exact)
            else:
                tau_edge = _as_float(sci.get("total_tau"))
                ppb_edge = _as_float(sci.get("total_ppb"))

            if tb_ppb is not None and ppb_edge is not None:
                ppb_edge_minus_tb = ppb_edge - tb_ppb

            calc_prior_ns = sci_calc.get("prior_ns")
            calc_edge_ns = sci_calc.get("edge_ns")
            calc_ns_between = sci_calc.get("ns_between")
            calc_residual_fast = sci_calc.get("residual_fast")
            tb_prior_ns = _as_int(sci.get("prior_edge_gnss_ns"))
            tb_edge_ns = _as_int(sci.get("current_edge_gnss_ns"))
            tb_ns_between = _as_int(sci.get("gnss_interval_ns"))
            tb_residual_fast = _as_int(sci.get("fast_residual_ns"))
            abs_edge_int_diffs = [
                _diff_int(calc_prior_ns, tb_prior_ns),
                _diff_int(calc_edge_ns, tb_edge_ns),
            ]
            math_int_diffs = [
                _diff_int(calc_ns_between, tb_ns_between),
                _diff_int(calc_residual_fast, tb_residual_fast),
            ]
            if any(d not in (None, 0) for d in abs_edge_int_diffs):
                stats["science_abs_edge_int_mismatch_rows"] += 1
            if any(d not in (None, 0) for d in math_int_diffs):
                stats["science_math_int_mismatch_rows"] += 1

            float_diffs = [
                _diff_float(sci_calc.get("edge_ns_exact"), _as_float(sci.get("current_edge_gnss_ns_exact"))),
                _diff_float(sci_calc.get("prior_ns_exact"), _as_float(sci.get("prior_edge_gnss_ns_exact"))),
                _diff_float(sci_calc.get("ns_between_exact"), _as_float(sci.get("gnss_interval_ns_exact"))),
                _diff_float(sci_calc.get("residual_fast_exact"), _as_float(sci.get("fast_residual_ns_exact"))),
                _diff_float(sci_calc.get("tau_1s"), _as_float(sci.get("tau_1s"))),
                _diff_float(sci_calc.get("ppb_1s"), _as_float(sci.get("ppb_1s"))),
                _diff_float(tau_edge, _as_float(sci.get("total_tau"))),
                _diff_float(ppb_edge, _as_float(sci.get("total_ppb"))),
            ]
            if any(d is not None and abs(d) > 5e-7 for d in float_diffs):
                stats["science_float_mismatch_rows"] += 1

        else:
            stats["science_missing"] += 1
            edge_dwt = selected_dwt_edge(root, frag, forensic, clock)

            if pps_ns is None or anchor_dwt is None:
                stats["anchor_missing"] += 1
            if cps is None or cps <= 0:
                stats["cps_missing"] += 1
            if edge_dwt is None:
                stats["dwt_missing"] += 1

            if pps_ns is not None and anchor_dwt is not None and cps is not None and cps > 0 and edge_dwt is not None:
                edge_ns = project_ns_from_dwt(edge_dwt=edge_dwt, anchor_dwt=anchor_dwt, anchor_ns=pps_ns, cps=cps)
            else:
                stats["projection_missing"] += 1

            if edge_dwt is not None and prev_dwt is not None:
                cycles_between = _delta_u32(edge_dwt, prev_dwt)

            prior_ns_for_row = prev_ns
            if edge_ns is not None and prev_ns is not None:
                ns_between = edge_ns - prev_ns
                residual_fast = NS_PER_SECOND - ns_between
                tau_1s = one_second_tau_from_period_ns(ns_between)
                ppb_1s = ppb_from_tau(tau_1s)

                if ns_between > 0:
                    cum_clock_ns += NS_PER_SECOND
                    cum_gnss_ns += ns_between
                    tau_edge = cum_clock_ns / float(cum_gnss_ns)
                    ppb_edge = ppb_from_tau(tau_edge)
                    if tb_ppb is not None and ppb_edge is not None:
                        ppb_edge_minus_tb = ppb_edge - tb_ppb

        tb_tau_minus_calc: Optional[float] = None
        tb_ppb_minus_calc: Optional[float] = None
        if tb_tau is not None and ledger_tau is not None:
            tb_tau_minus_calc = tb_tau - ledger_tau
        if tb_ppb is not None and ledger_ppb is not None:
            tb_ppb_minus_calc = tb_ppb - ledger_ppb

        science_total_tau = _as_float(sci.get("total_tau")) if use_science else None
        science_total_ppb = _as_float(sci.get("total_ppb")) if use_science else None
        science_reported_residual = _first_int(sci.get("reported_pps_fast_residual_ns"), published_ocxo_residual(root, frag, clock)) if use_science else published_ocxo_residual(root, frag, clock)

        rows.append({
            "pps": pps,
            "source": "SCI" if use_science else "LEG",
            "pps_ns": pps_ns,
            "dwt_edge": edge_dwt,
            "cycles_between": cycles_between,
            "prior_ns": prior_ns_for_row,
            "edge_ns": edge_ns,
            "ns_between": ns_between,
            "residual_fast": residual_fast,
            "tau_1s": tau_1s,
            "ppb_1s": ppb_1s,
            "tau_edge": tau_edge,
            "ppb_edge": ppb_edge,
            "tb_tau": tb_tau,
            "tb_ppb": tb_ppb,
            "ppb_edge_minus_tb": ppb_edge_minus_tb,
            "ledger_tau": ledger_tau,
            "ledger_ppb": ledger_ppb,
            "tb_tau_minus_calc": tb_tau_minus_calc,
            "tb_ppb_minus_calc": tb_ppb_minus_calc,
            "science_total_tau": science_total_tau,
            "science_total_ppb": science_total_ppb,
            "calc_total_tau_minus_science": _diff_float(tau_edge, science_total_tau),
            "calc_total_ppb_minus_science": _diff_float(ppb_edge, science_total_ppb),
            "calc_ppb_1s_minus_science": _diff_float(sci_calc.get("ppb_1s"), _as_float(sci.get("ppb_1s"))) if use_science else None,
            "calc_residual_minus_science": _diff_int(sci_calc.get("residual_fast"), _as_int(sci.get("fast_residual_ns"))) if use_science else None,
            "science_tb_prior_ns": _as_int(sci.get("prior_edge_gnss_ns")) if use_science else None,
            "science_calc_prior_ns": sci_calc.get("prior_ns") if use_science else None,
            "science_delta_prior_ns": _diff_int(sci_calc.get("prior_ns"), _as_int(sci.get("prior_edge_gnss_ns"))) if use_science else None,
            "science_tb_edge_ns": _as_int(sci.get("current_edge_gnss_ns")) if use_science else None,
            "science_calc_edge_ns": sci_calc.get("edge_ns") if use_science else None,
            "science_delta_edge_ns": _diff_int(sci_calc.get("edge_ns"), _as_int(sci.get("current_edge_gnss_ns"))) if use_science else None,
            "science_tb_ns_between": _as_int(sci.get("gnss_interval_ns")) if use_science else None,
            "science_calc_ns_between": sci_calc.get("ns_between") if use_science else None,
            "science_delta_ns_between": _diff_int(sci_calc.get("ns_between"), _as_int(sci.get("gnss_interval_ns"))) if use_science else None,
            "science_tb_residual": _as_int(sci.get("fast_residual_ns")) if use_science else None,
            "science_calc_residual": sci_calc.get("residual_fast") if use_science else None,
            "science_delta_residual": _diff_int(sci_calc.get("residual_fast"), _as_int(sci.get("fast_residual_ns"))) if use_science else None,
            "science_prior_exact": sci_calc.get("prior_ns_exact") if use_science else None,
            "science_edge_exact": sci_calc.get("edge_ns_exact") if use_science else None,
            "science_ns_between_exact": sci_calc.get("ns_between_exact") if use_science else None,
            "science_prior_halfdist": _frac_to_half(sci_calc.get("prior_ns_exact")) if use_science else None,
            "science_edge_halfdist": _frac_to_half(sci_calc.get("edge_ns_exact")) if use_science else None,
            "science_mismatch_flag": _fmt_delta_flag(
                _diff_int(sci_calc.get("prior_ns"), _as_int(sci.get("prior_edge_gnss_ns"))) if use_science else None,
                _diff_int(sci_calc.get("edge_ns"), _as_int(sci.get("current_edge_gnss_ns"))) if use_science else None,
                _diff_int(sci_calc.get("ns_between"), _as_int(sci.get("gnss_interval_ns"))) if use_science else None,
                _diff_int(sci_calc.get("residual_fast"), _as_int(sci.get("fast_residual_ns"))) if use_science else None,
            ) if use_science else ".",
            "anchor_dwt": anchor_dwt,
            "cps": cps,
            "firmware_interval_hint": firmware_interval_hint(root, frag, forensic, clock),
            "firmware_residual": science_reported_residual,
            "reported_minus_floorline_residual_ns": _as_int(sci.get("reported_minus_floorline_residual_ns")) if use_science else None,
        })
        stats["rows_collected"] += 1

        prev_pps = pps
        if edge_dwt is not None:
            prev_dwt = edge_dwt
        if edge_ns is not None:
            prev_ns = edge_ns

    return rows, stats


def _header(show_cumulative: bool, show_science_debug: bool) -> Tuple[str, str]:
    parts = [
        f"{'pps':>6s}",
        f"{'src':>3s}",
        f"{'pps_ns':>17s}",
        f"{'dwt_edge':>13s}",
        f"{'cyc':>13s}",
        f"{'prior_ns':>17s}",
        f"{'edge_ns':>17s}",
        f"{'ns_Δ':>13s}",
        f"{'res_fast':>9s}",
        f"{'tau_1s':>16s}",
        f"{'ppb_1s':>11s}",
    ]
    if show_cumulative:
        parts.extend([
            f"{'tau_edge':>16s}",
            f"{'tau_tb':>16s}",
            f"{'ppb_edge':>11s}",
            f"{'ppb_tb':>11s}",
            f"{'ppb_Δ':>11s}",
        ])
    if show_science_debug:
        parts.extend([
            f"{'!':>1s}",
            f"{'tb_prior_edge':>13s}",
            f"{'calc_prior_edge':>15s}",
            f"{'Δprior':>7s}",
            f"{'tb_cur_edge':>13s}",
            f"{'calc_cur_edge':>13s}",
            f"{'Δcur':>5s}",
            f"{'tb_interval':>11s}",
            f"{'calc_interval':>13s}",
            f"{'Δint':>5s}",
            f"{'tb_resid':>8s}",
            f"{'calc_resid':>10s}",
            f"{'Δres':>5s}",
            f"{'prior_half':>10s}",
            f"{'cur_half':>8s}",
        ])
    return "  ".join(parts), "  ".join("─" * len(p) for p in parts)


def _row_line(row: Dict[str, Any], show_cumulative: bool, show_science_debug: bool) -> str:
    parts = [
        f"{row['pps']:>6d}",
        f"{row.get('source', ''):>3s}",
        _fmt_int(row.get("pps_ns"), 17),
        _fmt_int(row.get("dwt_edge"), 13),
        _fmt_int(row.get("cycles_between"), 13),
        _fmt_int(row.get("prior_ns"), 17),
        _fmt_int(row.get("edge_ns"), 17),
        _fmt_int(row.get("ns_between"), 13),
        _fmt_int(row.get("residual_fast"), 9, signed=True),
        _fmt_float(row.get("tau_1s"), 16, 12),
        _fmt_float(row.get("ppb_1s"), 11, 3, signed=True),
    ]
    if show_cumulative:
        parts.extend([
            _fmt_float(row.get("tau_edge"), 16, 12),
            _fmt_float(row.get("tb_tau"), 16, 12),
            _fmt_float(row.get("ppb_edge"), 11, 3, signed=True),
            _fmt_float(row.get("tb_ppb"), 11, 3, signed=True),
            _fmt_float(row.get("ppb_edge_minus_tb"), 11, 3, signed=True),
        ])
    if show_science_debug:
        parts.extend([
            f"{row.get('science_mismatch_flag', '.'):>1s}",
            _fmt_int(row.get("science_tb_prior_ns"), 13),
            _fmt_int(row.get("science_calc_prior_ns"), 13),
            _fmt_int(row.get("science_delta_prior_ns"), 4, signed=True),
            _fmt_int(row.get("science_tb_edge_ns"), 13),
            _fmt_int(row.get("science_calc_edge_ns"), 13),
            _fmt_int(row.get("science_delta_edge_ns"), 4, signed=True),
            _fmt_int(row.get("science_tb_ns_between"), 11),
            _fmt_int(row.get("science_calc_ns_between"), 11),
            _fmt_int(row.get("science_delta_ns_between"), 4, signed=True),
            _fmt_int(row.get("science_tb_residual"), 7, signed=True),
            _fmt_int(row.get("science_calc_residual"), 8, signed=True),
            _fmt_int(row.get("science_delta_residual"), 4, signed=True),
            _fmt_float(row.get("science_prior_halfdist"), 8, 3, signed=True),
            _fmt_float(row.get("science_edge_halfdist"), 8, 3, signed=True),
        ])
    return "  ".join(parts)


def _max_abs(values: Iterable[Optional[float]]) -> Optional[float]:
    vals = [abs(float(v)) for v in values if v is not None]
    return max(vals) if vals else None


def analyze(campaign: str,
            clock: str = "OCXO1",
            limit: int = 0,
            show_cumulative: bool = True,
            show_science_debug: bool = False,
            mismatch_limit: int = 40) -> None:
    clock = normalize_clock(clock)
    records = fetch_timebase(campaign)
    if not records:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    rows_all, stats = collect_rows(records, clock)
    rows = rows_all[:limit] if limit else rows_all

    print(f"Campaign: {campaign}  ({len(records):,} TIMEBASE rows, view={clock})")
    print()
    print("Raw nanosecond / TIMEBASE science audit")
    print("════════════════════════════════")
    print("  Primary path: fragment.<clock>.science when present; legacy projection otherwise.")
    print("  External science calc: edge_ns = PPS/VCLOCK ns + signed(edge_dwt - pps_vclock_dwt) * 1e9 / cps")
    print("  ns_Δ is GNSS nanoseconds between selected clock edges.")
    print("  res_fast = 1e9 - ns_Δ; positive means selected clock fast.")
    print("  tau_1s = 1e9 / ns_Δ, ppb_1s = (tau_1s - 1) * 1e9.")
    if show_cumulative:
        print("  tau_edge/ppb_edge are running totals recomputed by this report from science antecedents.")
        print("  tau_tb/ppb_tb are Teensy/TIMEBASE campaign-total values from fragment.stats.")
        print("  ppb_Δ = ppb_edge - ppb_tb; nonzero means TIMEBASE stats still use a different authority.")
    if show_science_debug:
        print("  Debug columns compare TIMEBASE science integer fields to an independent recompute.")
        print("  Δprior/Δcur are absolute projected edge-coordinate differences.")
        print("  Δint/Δres are interval/residual math differences; these should remain zero.")
        print("  prior_half/cur_half show distance from the nearest half-ns rounding boundary.")
    print()

    header, sep = _header(show_cumulative, show_science_debug)
    print(header)
    print(sep)
    for row in rows:
        print(_row_line(row, show_cumulative, show_science_debug))

    print()
    print(f"Rows shown:          {len(rows):,}")
    print(f"Rows collected:      {stats['rows_collected']:,}")
    print(f"Science rows:        {stats['science_rows']:,}")
    print(f"Science missing:     {stats['science_missing']:,}")
    print(f"Science math int mismatch:    {stats['science_math_int_mismatch_rows']:,}")
    print(f"Science abs-edge int mismatch:{stats['science_abs_edge_int_mismatch_rows']:,}")
    print(f"Science fp mismatch:          {stats['science_float_mismatch_rows']:,}")
    print(f"Gaps/reset points:   {stats['gaps']:,}")
    print(f"Missing DWT edge:    {stats['dwt_missing']:,}")
    print(f"Missing anchor:      {stats['anchor_missing']:,}")
    print(f"Missing CPS:         {stats['cps_missing']:,}")
    print(f"Missing projection:  {stats['projection_missing']:,}")

    valid_res = [r["residual_fast"] for r in rows if r.get("residual_fast") is not None]
    if valid_res:
        mean_res = sum(valid_res) / len(valid_res)
        min_res = min(valid_res)
        max_res = max(valid_res)
        print()
        print("Quick residual view")
        print("═══════════════════")
        print(f"  samples={len(valid_res):,}  mean={mean_res:+.3f} ns  min={min_res:+d} ns  max={max_res:+d} ns")
        last = next((r for r in reversed(rows) if r.get("ppb_edge") is not None), None)
        if show_cumulative and last is not None:
            print(
                f"  final edge cumulative tau={last['tau_edge']:.12f}  "
                f"ppb={last['ppb_edge']:+.3f}"
            )
            if last.get("tb_ppb") is not None:
                print(
                    f"  final TIMEBASE stats tau={last['tb_tau']:.12f}  "
                    f"ppb={last['tb_ppb']:+.3f}  "
                    f"edge_minus_tb={last['ppb_edge_minus_tb']:+.3f} ppb"
                )

    science_shown = [r for r in rows if r.get("source") == "SCI"]
    if science_shown:
        print()
        print("TIMEBASE science cross-check")
        print("════════════════════════════")
        print("  External recompute uses only fragment.<clock>.science antecedents.")
        print("  Integer fields must match exactly; float fields are checked to published precision.")
        print(f"  rows_with_science={len(science_shown):,}")
        max_residual_err = _max_abs(r.get("calc_residual_minus_science") for r in science_shown)
        max_ppb_1s_err = _max_abs(r.get("calc_ppb_1s_minus_science") for r in science_shown)
        max_total_ppb_err = _max_abs(r.get("calc_total_ppb_minus_science") for r in science_shown)
        if max_residual_err is not None:
            print(f"  max |calc_residual_ns - science_residual_ns| = {max_residual_err:.0f} ns")
        if max_ppb_1s_err is not None:
            print(f"  max |calc_ppb_1s - science_ppb_1s| = {max_ppb_1s_err:.9f} ppb")
        if max_total_ppb_err is not None:
            print(f"  max |calc_total_ppb - science_total_ppb| = {max_total_ppb_err:.9f} ppb")

        mismatch_rows = [
            r for r in science_shown
            if any(r.get(k) not in (None, 0) for k in (
                "science_delta_prior_ns",
                "science_delta_edge_ns",
                "science_delta_ns_between",
                "science_delta_residual",
            ))
        ]
        if mismatch_rows:
            print()
            print("Science integer mismatch detail")
            print("═══════════════════════════════")
            print("  Δ fields are calc-minus-TIMEBASE.")
            print("  Δprior/Δcur are absolute projected edge-coordinate differences.")
            print("  Δint/Δres are interval/residual math differences; these should remain zero.")
            print("  prior_half/cur_half near 0 means the exact edge coordinate landed near")
            print("  a half-ns boundary, so ±1 ns edge-coordinate rounding differences are expected.")

            shown_mismatches = mismatch_rows[:mismatch_limit]

            def _mismatch_kind(r: Dict[str, Any]) -> str:
                edge_bad = any(r.get(k) not in (None, 0) for k in (
                    "science_delta_prior_ns", "science_delta_edge_ns"))
                math_bad = any(r.get(k) not in (None, 0) for k in (
                    "science_delta_ns_between", "science_delta_residual"))
                if edge_bad and math_bad:
                    return "both"
                if math_bad:
                    return "math"
                if edge_bad:
                    return "edge"
                return "."

            detail_columns = [
                ("pps", lambda r: f"{r['pps']:d}"),
                ("kind", _mismatch_kind),
                ("Δprior", lambda r: _fmt_int(r.get("science_delta_prior_ns"), signed=True)),
                ("Δcur", lambda r: _fmt_int(r.get("science_delta_edge_ns"), signed=True)),
                ("Δint", lambda r: _fmt_int(r.get("science_delta_ns_between"), signed=True)),
                ("Δres", lambda r: _fmt_int(r.get("science_delta_residual"), signed=True)),
                ("tb_prior_edge", lambda r: _fmt_int(r.get("science_tb_prior_ns"))),
                ("calc_prior_edge", lambda r: _fmt_int(r.get("science_calc_prior_ns"))),
                ("tb_cur_edge", lambda r: _fmt_int(r.get("science_tb_edge_ns"))),
                ("calc_cur_edge", lambda r: _fmt_int(r.get("science_calc_edge_ns"))),
                ("tb_interval", lambda r: _fmt_int(r.get("science_tb_ns_between"))),
                ("calc_interval", lambda r: _fmt_int(r.get("science_calc_ns_between"))),
                ("tb_resid", lambda r: _fmt_int(r.get("science_tb_residual"), signed=True)),
                ("calc_resid", lambda r: _fmt_int(r.get("science_calc_residual"), signed=True)),
                ("prior_half", lambda r: _fmt_float(r.get("science_prior_halfdist"), decimals=3, signed=True)),
                ("cur_half", lambda r: _fmt_float(r.get("science_edge_halfdist"), decimals=3, signed=True)),
            ]

            detail_rows = [[fn(r) for _, fn in detail_columns] for r in shown_mismatches]
            widths = []
            for i, (name, _) in enumerate(detail_columns):
                data_width = max((len(row[i]) for row in detail_rows), default=0)
                widths.append(max(len(name), data_width))

            print("  ".join(name.rjust(widths[i]) for i, (name, _) in enumerate(detail_columns)))
            print("  ".join(("─" * widths[i]) for i in range(len(detail_columns))))
            for row in detail_rows:
                print("  ".join(row[i].rjust(widths[i]) for i in range(len(detail_columns))))

            if len(mismatch_rows) > mismatch_limit:
                print(f"  ... {len(mismatch_rows) - mismatch_limit:,} more mismatch rows suppressed; use --mismatch-limit to show more.")

    if clock != "PPS":
        shown_with_stats = [r for r in rows if r.get("tb_ppb") is not None]
        if shown_with_stats:
            max_tau_err = _max_abs(r.get("tb_tau_minus_calc") for r in shown_with_stats)
            max_ppb_err = _max_abs(r.get("tb_ppb_minus_calc") for r in shown_with_stats)
            print()
            print("TIMEBASE stats cross-check")
            print("══════════════════════════")
            print("  Recomputed public-ledger tau/ppb = fragment.<clock>.ns / fragment.gnss.ns.")
            print("  Compared against fragment.stats.<clock>.tau/ppb.")
            print(f"  rows_with_stats={len(shown_with_stats):,}")
            if max_tau_err is not None:
                print(f"  max |stats_tau - ledger_tau| = {max_tau_err:.12e}")
            if max_ppb_err is not None:
                print(f"  max |stats_ppb - ledger_ppb| = {max_ppb_err:.6f} ppb")

    firmware_residual_rows = [
        r for r in rows
        if r.get("residual_fast") is not None and r.get("firmware_residual") is not None
    ]
    if firmware_residual_rows:
        diffs = [
            int(r["residual_fast"]) - int(r["firmware_residual"])
            for r in firmware_residual_rows
        ]
        print()
        print("Published residual side check")
        print("═════════════════════════════")
        print("  diff = projected_res_fast - fragment.<clock>.pps_residual.fast_residual_ns")
        print(f"  rows={len(diffs):,}  mean={sum(diffs)/len(diffs):+.3f} ns  min={min(diffs):+d} ns  max={max(diffs):+d} ns")

    print()
    print("Notes")
    print("═════")
    print("  • This report recomputes projected GNSS edge times from persisted TIMEBASE data;")
    print("    it does not call live Teensy time functions.")
    print("  • The DWT edge is the subscriber-facing published edge.  For current")
    print("    FloorLine firmware this should be the FloorLine endpoint verified by raw_cycles.")
    print("  • PPS rows use the physical PPS witness DWT projected against the canonical")
    print("    PPS/VCLOCK anchor for the row.")
    print("  • tau_edge/ppb_edge are edge-interval totals; tau_tb/ppb_tb are firmware")
    print("    campaign-public totals from TIMEBASE stats.  They should converge, but they")
    print("    are not identical definitions at campaign start or after gaps.")
    print("  • First rows and rows after PPS-count gaps have no prior edge, so interval")
    print("    columns intentionally show ---.  Firmware may have interval hints from before")
    print("    the campaign boundary, but this report keeps campaign-local arithmetic clean.")
    print()


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------


def parse_args(argv: List[str]) -> Tuple[str, str, int, bool, bool, int]:
    if len(argv) < 2:
        print("Usage: raw_nanoseconds <campaign_name> [clock] [limit]")
        print("       raw_nanoseconds <campaign_name> --clock OCXO2 --limit 300")
        print("       raw_nanoseconds <campaign_name> OCXO1 200")
        print("       raw_nanoseconds <campaign_name> OCXO1 --science-debug --mismatch-limit 40")
        raise SystemExit(1)

    campaign = argv[1]
    clock = "OCXO1"
    limit = 0
    show_cumulative = True
    show_science_debug = False
    mismatch_limit = 40

    i = 2
    while i < len(argv):
        arg = argv[i]
        if arg == "--clock":
            if i + 1 >= len(argv):
                raise SystemExit("--clock requires PPS, VCLOCK, OCXO1, or OCXO2")
            clock = argv[i + 1]
            i += 2
            continue
        if arg.startswith("--clock="):
            clock = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--limit":
            if i + 1 >= len(argv):
                raise SystemExit("--limit requires an integer")
            try:
                limit = int(argv[i + 1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 2
            continue
        if arg.startswith("--limit="):
            try:
                limit = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit("--limit requires an integer") from exc
            i += 1
            continue
        if arg == "--no-cum":
            show_cumulative = False
            i += 1
            continue
        if arg in ("--science-debug", "--debug-science", "--science-int"):
            show_science_debug = True
            i += 1
            continue
        if arg == "--mismatch-limit":
            if i + 1 >= len(argv):
                raise SystemExit("--mismatch-limit requires an integer")
            try:
                mismatch_limit = int(argv[i + 1])
            except ValueError as exc:
                raise SystemExit("--mismatch-limit requires an integer") from exc
            i += 2
            continue
        if arg.startswith("--mismatch-limit="):
            try:
                mismatch_limit = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit("--mismatch-limit requires an integer") from exc
            i += 1
            continue
        upper = arg.upper()
        if upper in CLOCKS or upper in ("P", "V", "VC", "O1", "O2"):
            clock = arg
            i += 1
            continue
        try:
            limit = int(arg)
        except ValueError as exc:
            raise SystemExit(f"unknown argument '{arg}'") from exc
        i += 1

    return campaign, normalize_clock(clock), limit, show_cumulative, show_science_debug, mismatch_limit


def main() -> None:
    campaign, clock, limit, show_cumulative, show_science_debug, mismatch_limit = parse_args(sys.argv)
    analyze(campaign, clock=clock, limit=limit, show_cumulative=show_cumulative, show_science_debug=show_science_debug, mismatch_limit=mismatch_limit)


if __name__ == "__main__":
    main()
