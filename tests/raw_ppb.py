"""
ZPNet Raw PPB — compact one-second cycle/residual report.

Reads TIMEBASE rows for one campaign and prints one line per campaign PPS row.
For each row it shows PPS, VCLOCK, OCXO1, and OCXO2 one-second DWT cycle
counts plus the sign-aligned nanosecond residual against the GNSS / selected
PPS-VCLOCK reference. Positive residual means the clock is fast.

Usage:
    python -m zpnet.tests.raw_ppb <campaign_name>
    python raw_ppb.py <campaign_name>

Columns:
    pps       Campaign PPS/VCLOCK row identity.
    *_cyc     One-second DWT cycle count for that rail.
    *_res     One-second fast residual in ns. Positive means clock fast.
    *_ppb     Running cumulative mean of that rail's residuals. Because each
              residual is measured over a one-second GNSS gate, ns/sec is ppb.

This intentionally has no clock selector, no limit, and no forensic side panels.
It is the "just show me the one-second cycle/residual and running mean"
companion to raw_nanoseconds.
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


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        out = _as_int(value)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 3, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _round_nearest_int(value: float) -> int:
    """Round like firmware integer-ns rendering, away from zero at half."""
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


# -----------------------------------------------------------------------------
# TIMEBASE extraction
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


def selected_reference_cycles(root: Dict[str, Any],
                              frag: Dict[str, Any],
                              forensic: Dict[str, Any]) -> Optional[int]:
    """Selected PPS/VCLOCK DWT edge interval for GNSS comparison.

    Prefer the exact selected PPS/VCLOCK edge-to-edge interval from
    TIMEBASE_FORENSICS. Fall back to the compact identity field, which is the
    effective DWT/GNSS second ruler on current schemas.
    """
    return _first_int(
        _nested_get(forensic, "pps_vclock_edge", "dwt_cycles_between_edges"),
        _nested_get(frag, "pps_vclock_edge", "dwt_cycles_between_edges"),
        forensic.get("dwt_cycles_between_pps_vclock"),
        frag.get("dwt_cycles_between_pps_vclock"),
        root.get("dwt_cycles_between_pps_vclock"),
        _nested_get(frag, "dwt", "cycles_between_pps_vclock"),
        _nested_get(forensic, "pps_vclock_edge", "effective_dwt_cycles_per_second"),
    )


def science_from_schema(frag: Dict[str, Any], clock: str) -> Dict[str, Any]:
    key = LANE_KEYS.get(clock)
    if not key:
        return {}
    obj = _nested_get(frag, key, "science")
    return obj if isinstance(obj, dict) else {}


def lane_alpha_forensics(root: Dict[str, Any],
                         frag: Dict[str, Any],
                         forensic: Dict[str, Any],
                         lane: str) -> Dict[str, Any]:
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


def pps_cycles(root: Dict[str, Any],
               frag: Dict[str, Any],
               forensic: Dict[str, Any],
               prev_pps_dwt: Optional[int]) -> Optional[int]:
    hinted = _first_int(
        forensic.get("pps_obs"),
        _nested_get(frag, "pps", "dwt_cycles_between_edges"),
        frag.get("pps_dwt_cycles_between_edges"),
        root.get("pps_dwt_cycles_between_edges"),
    )
    if hinted is not None:
        return hinted

    dwt = _first_int(
        frag.get("pps_dwt_at_edge"),
        forensic.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        _nested_get(forensic, "pps_vclock_edge", "pps_dwt_at_edge"),
        _nested_get(frag, "pps_vclock_edge", "pps_dwt_at_edge"),
        _nested_get(frag, "pps", "dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
        frag.get("pps_edge_dwt_at_edge"),
    )
    if dwt is None or prev_pps_dwt is None:
        return None
    return _delta_u32(dwt, prev_pps_dwt)


def pps_dwt_at_edge(root: Dict[str, Any],
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


def vclock_cycles(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  ref_cycles: Optional[int]) -> Optional[int]:
    sci = science_from_schema(frag, "VCLOCK")
    f = lane_alpha_forensics(root, frag, forensic, "vclock")
    return _first_int(
        sci.get("clock_observed_interval_cycles"),
        sci.get("clock_floorline_interval_cycles"),
        _micro_first_int(root, frag, forensic, "vclock", "pub_cyc", "eff"),
        f.get("dwt_cycles_between_edges"),
        _nested_get(f, "dwt_interval_gate", "effective_cycles"),
        ref_cycles,
    )


def ocxo_cycles_and_residual(root: Dict[str, Any],
                             frag: Dict[str, Any],
                             forensic: Dict[str, Any],
                             clock: str,
                             fallback_ref_cycles: Optional[int]) -> Tuple[Optional[int], Optional[int]]:
    lane = LANE_KEYS[clock]
    sci = science_from_schema(frag, clock)
    f = lane_alpha_forensics(root, frag, forensic, lane)

    cycles = _first_int(
        sci.get("delta_raw_clock_interval_cycles"),
        sci.get("clock_observed_interval_cycles"),
        sci.get("clock_floorline_interval_cycles"),
        _micro_first_int(root, frag, forensic, lane, "pub_cyc", "eff"),
        f.get("dwt_cycles_between_edges"),
        _nested_get(f, "dwt_interval_gate", "effective_cycles"),
    )
    residual = _first_int(
        sci.get("delta_raw_fast_residual_ns"),
        sci.get("fast_residual_ns"),
        _nested_get(frag, lane, "pps_residual", "fast_residual_ns"),
        _nested_get(root, "fragment", lane, "pps_residual", "fast_residual_ns"),
    )

    if residual is None:
        ref = _first_int(sci.get("delta_raw_reference_interval_cycles"), fallback_ref_cycles)
        residual = fast_residual_ns(ref, cycles)
    return cycles, residual


def fast_residual_ns(reference_cycles: Optional[int], clock_cycles: Optional[int]) -> Optional[int]:
    """Positive-fast ns residual from cycle interval comparison."""
    if reference_cycles is None or clock_cycles is None or reference_cycles <= 0:
        return None
    exact = (float(reference_cycles) - float(clock_cycles)) * float(NS_PER_SECOND) / float(reference_cycles)
    return _round_nearest_int(exact)


# -----------------------------------------------------------------------------
# Analysis and output
# -----------------------------------------------------------------------------


def collect_rows(records: Iterable[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], Dict[str, int]]:
    rows: List[Dict[str, Any]] = []
    stats = {
        "records_seen": 0,
        "rows_collected": 0,
        "gaps": 0,
        "missing_reference": 0,
        "missing_any_clock": 0,
    }

    prev_pps: Optional[int] = None
    prev_pps_dwt: Optional[int] = None

    residual_sums = {"pps_res": 0.0, "v_res": 0.0, "o1_res": 0.0, "o2_res": 0.0}
    residual_counts = {"pps_res": 0, "v_res": 0, "o1_res": 0, "o2_res": 0}

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
            prev_pps_dwt = None

        ref = selected_reference_cycles(root, frag, forensic)
        if ref is None:
            stats["missing_reference"] += 1

        pps_cyc = pps_cycles(root, frag, forensic, prev_pps_dwt)
        v_cyc = vclock_cycles(root, frag, forensic, ref)
        o1_cyc, o1_res = ocxo_cycles_and_residual(root, frag, forensic, "OCXO1", ref)
        o2_cyc, o2_res = ocxo_cycles_and_residual(root, frag, forensic, "OCXO2", ref)

        row = {
            "pps": pps,
            "pps_cyc": pps_cyc,
            "pps_res": fast_residual_ns(ref, pps_cyc),
            "v_cyc": v_cyc,
            "v_res": fast_residual_ns(ref, v_cyc),
            "o1_cyc": o1_cyc,
            "o1_res": o1_res,
            "o2_cyc": o2_cyc,
            "o2_res": o2_res,
        }

        # Running cumulative PPB is just the mean of one-second residuals.
        # A residual in ns over a one-second GNSS gate is numerically ppb.
        for residual_key, ppb_key in (
            ("pps_res", "pps_ppb"),
            ("v_res", "v_ppb"),
            ("o1_res", "o1_ppb"),
            ("o2_res", "o2_ppb"),
        ):
            residual = row.get(residual_key)
            if residual is not None:
                residual_sums[residual_key] += float(residual)
                residual_counts[residual_key] += 1
                row[ppb_key] = residual_sums[residual_key] / float(residual_counts[residual_key])
            else:
                row[ppb_key] = None

        if any(row[k] is None for k in ("pps_cyc", "pps_res", "v_cyc", "v_res", "o1_cyc", "o1_res", "o2_cyc", "o2_res")):
            stats["missing_any_clock"] += 1

        rows.append(row)
        stats["rows_collected"] += 1
        prev_pps = pps
        prev_pps_dwt = pps_dwt_at_edge(root, frag, forensic)

    return rows, stats


def print_table(rows: List[Dict[str, Any]]) -> None:
    columns = [
        ("pps", lambda r: _fmt_int(r.get("pps"))),
        ("pps_cyc", lambda r: _fmt_int(r.get("pps_cyc"))),
        ("pps_res", lambda r: _fmt_int(r.get("pps_res"), signed=True)),
        ("pps_ppb", lambda r: _fmt_float(r.get("pps_ppb"), signed=True)),
        ("v_cyc", lambda r: _fmt_int(r.get("v_cyc"))),
        ("v_res", lambda r: _fmt_int(r.get("v_res"), signed=True)),
        ("v_ppb", lambda r: _fmt_float(r.get("v_ppb"), signed=True)),
        ("o1_cyc", lambda r: _fmt_int(r.get("o1_cyc"))),
        ("o1_res", lambda r: _fmt_int(r.get("o1_res"), signed=True)),
        ("o1_ppb", lambda r: _fmt_float(r.get("o1_ppb"), signed=True)),
        ("o2_cyc", lambda r: _fmt_int(r.get("o2_cyc"))),
        ("o2_res", lambda r: _fmt_int(r.get("o2_res"), signed=True)),
        ("o2_ppb", lambda r: _fmt_float(r.get("o2_ppb"), signed=True)),
    ]

    rendered = [[fn(row) for _, fn in columns] for row in rows]
    widths: List[int] = []
    for i, (name, _) in enumerate(columns):
        data_width = max((len(row[i]) for row in rendered), default=0)
        widths.append(max(len(name), data_width))

    print("  ".join(name.rjust(widths[i]) for i, (name, _) in enumerate(columns)))
    print("  ".join(("─" * widths[i]) for i in range(len(columns))))
    for row in rendered:
        print("  ".join(row[i].rjust(widths[i]) for i in range(len(columns))))


def analyze(campaign: str) -> None:
    records = fetch_timebase(campaign)
    rows, stats = collect_rows(records)

    print(f"ZPNet raw_ppb — campaign={campaign}")
    print(f"records={stats['records_seen']:,}  rows={stats['rows_collected']:,}  gaps={stats['gaps']:,}")
    if stats["missing_reference"] or stats["missing_any_clock"]:
        print(f"missing_reference={stats['missing_reference']:,}  rows_with_missing_fields={stats['missing_any_clock']:,}")
    print()

    if not rows:
        print("No TIMEBASE rows found.")
        return

    print_table(rows)

    print()
    print("Notes")
    print("═════")
    print("  • *_cyc fields are one-second DWT cycle counts.")
    print("  • *_res fields are sign-aligned fast residuals in ns; positive means that rail is fast.")
    print("  • *_ppb fields are the running cumulative mean of that rail's residuals, computed by this report.")
    print("  • Because each residual is over a one-second GNSS gate, residual ns/sec is numerically ppb.")
    print("  • VCLOCK residual is normally zero because the selected PPS/VCLOCK interval is the GNSS reference.")
    print("  • OCXO residuals prefer fragment.<ocxo>.science.delta_raw_fast_residual_ns when present.")


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: raw_ppb <campaign_name>")
        raise SystemExit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()
