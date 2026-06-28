#!/usr/bin/env python3
"""
ZPNet Delta Cycles — OCXO cycle-domain residual report.

Reads TIMEBASE rows for a campaign and prints one row per TIMEBASE record.
For each row it shows the one-second DWT cycle interval for PPS, VCLOCK,
OCXO1, and OCXO2.  For each OCXO it also shows:

    traditional residual:
        fragment.ocxoN.science.fast_residual_ns
        (GNSS-nanosecond-at-edge projection path, positive = clock fast)

    cycle residual:
        delayed VCLOCK interval - OCXO interval
        (same DWT bookend species, positive = clock fast)

By default the report uses FloorLine DWT bookends because that is the closest
apples-to-apples comparison against the current traditional FloorLine/GNSS
projection residual.  Use --raw or --mode raw to switch to observed/raw DWT
bookends.

Usage:
    python delta_cycles.py <campaign>
    python delta_cycles.py <campaign> --limit 300
    python delta_cycles.py <campaign> --raw
    python delta_cycles.py <campaign> --floorline
    python delta_cycles.py <campaign> --mode observed
    python delta_cycles.py --file pasted_timebase.txt --campaign Delta1 --raw

Column doctrine:
    pps        TIMEBASE publication PPS/VCLOCK identity.
    ref        delayed VCLOCK reference row used for OCXO residuals.
    pps_cyc    physical PPS witness DWT interval for this row.
    v_cyc      selected VCLOCK interval in selected mode.
    o1_cyc     OCXO1 interval in selected mode.
    o1_trad    traditional projected-GNSS fast residual, ns, positive fast.
    o1_fast    cycle-domain fast residual = delayed_vclock_cyc - ocxo1_cyc.
    o1-td      o1_fast - o1_trad.
    o1_mean    running mean of o1_fast in this report segment.
    o2_*       same for OCXO2.

The OCXO residual intentionally compares the current OCXO interval to the
previous row's VCLOCK interval, matching the OCXO-is-late publication doctrine:
the OCXO interval for reference second n becomes visible in TIMEBASE row n+1.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


CLOCKS = ("PPS", "VCLOCK", "OCXO1", "OCXO2")
OCXOS = ("OCXO1", "OCXO2")
LANE_KEYS = {"VCLOCK": "vclock", "OCXO1": "ocxo1", "OCXO2": "ocxo2"}
LANE_MICRO_PREFIXES = {"vclock": "v", "ocxo1": "o1", "ocxo2": "o2"}


# -----------------------------------------------------------------------------
# Generic schema helpers
# -----------------------------------------------------------------------------


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


def _fmt_float(v: Optional[float], width: int = 0, decimals: int = 3,
               signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,.{decimals}f}" if signed else f"{v:,.{decimals}f}"
    return f"{s:>{width}s}" if width else s


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


# -----------------------------------------------------------------------------
# Input
# -----------------------------------------------------------------------------


def fetch_timebase_db(campaign: str) -> List[Dict[str, Any]]:
    """Fetch TIMEBASE rows in campaign PPS order from the ZPNet database."""
    from zpnet.shared.db import open_db  # Imported lazily so --file works anywhere.

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


def fetch_timebase_file(path: str, campaign: Optional[str] = None) -> List[Dict[str, Any]]:
    """Read pasted TIMEBASE lines or JSON/JSONL records from a file."""
    out: List[Dict[str, Any]] = []
    text = Path(path).read_text()

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("TIMEBASE "):
            line = line[len("TIMEBASE "):].strip()
        if not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(obj, dict):
            continue
        root = _root(obj)
        if campaign:
            row_campaign = _first_str(root.get("campaign"), _nested_get(root, "fragment", "campaign"))
            if row_campaign and row_campaign != campaign:
                continue
        out.append(obj)

    out.sort(key=lambda rec: pps_count(rec) if pps_count(rec) is not None else -1)
    return out


def _first_str(*values: Any) -> Optional[str]:
    for v in values:
        if isinstance(v, str) and v:
            return v
    return None


# -----------------------------------------------------------------------------
# TIMEBASE extraction
# -----------------------------------------------------------------------------


def pps_count(rec: Dict[str, Any]) -> Optional[int]:
    root = _root(rec)
    frag = _frag(rec)
    forensic = _forensics_root(rec)
    return _first_int(
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
        forensic.get("pps_count"),
        forensic.get("teensy_pps_vclock_count"),
        forensic.get("teensy_pps_count"),
    )


def campaign_name(rec: Dict[str, Any]) -> Optional[str]:
    root = _root(rec)
    frag = _frag(rec)
    forensic = _forensics_root(rec)
    return _first_str(root.get("campaign"), frag.get("campaign"), forensic.get("campaign"))


def science_object(rec: Dict[str, Any], clock: str) -> Dict[str, Any]:
    if clock == "PPS":
        return {}
    lane = LANE_KEYS[clock]
    obj = _nested_get(_frag(rec), lane, "science")
    return obj if isinstance(obj, dict) else {}


def _micro_first_int(rec: Dict[str, Any], lane: str, *suffixes: str) -> Optional[int]:
    prefix = LANE_MICRO_PREFIXES.get(lane)
    if not prefix:
        return None
    root = _root(rec)
    frag = _frag(rec)
    forensic = _forensics_root(rec)
    values: List[Any] = []
    for suffix in suffixes:
        key = f"{prefix}_{suffix}"
        values.extend([forensic.get(key), frag.get(key), root.get(key)])
    return _first_int(*values)


def physical_pps_dwt(rec: Dict[str, Any]) -> Optional[int]:
    root = _root(rec)
    frag = _frag(rec)
    forensic = _forensics_root(rec)
    return _first_int(
        frag.get("pps_dwt_at_edge"),
        forensic.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        _nested_get(forensic, "pps_vclock_edge", "pps_dwt_at_edge"),
        _nested_get(frag, "pps", "dwt_at_edge"),
    )


def edge_dwt(rec: Dict[str, Any], clock: str, mode: str) -> Optional[int]:
    """Return the DWT bookend endpoint for one row and selected mode."""
    if clock == "PPS":
        return physical_pps_dwt(rec)

    sci = science_object(rec, clock)
    if clock == "VCLOCK":
        lane = "vclock"
    else:
        lane = LANE_KEYS[clock]

    if mode == "floorline":
        return _first_int(
            sci.get("clock_floorline_dwt_at_edge"),
            _micro_first_int(rec, lane, "fl"),
            sci.get("clock_published_dwt_at_edge"),
            _micro_first_int(rec, lane, "pub", "used"),
        )

    # raw / observed
    return _first_int(
        sci.get("clock_raw_dwt_at_edge"),
        _micro_first_int(rec, lane, "raw", "orig"),
        sci.get("clock_observed_dwt_at_edge"),
    )


def interval_hint(rec: Dict[str, Any], clock: str, mode: str) -> Optional[int]:
    """Seed/fallback one-second interval from the TIMEBASE row itself."""
    root = _root(rec)
    frag = _frag(rec)
    forensic = _forensics_root(rec)

    if clock == "PPS":
        return _first_int(
            forensic.get("pps_obs"),
            _nested_get(frag, "pps", "dwt_cycles_between_edges"),
            frag.get("pps_dwt_cycles_between_edges"),
            root.get("pps_dwt_cycles_between_edges"),
        )

    sci = science_object(rec, clock)
    lane = "vclock" if clock == "VCLOCK" else LANE_KEYS[clock]

    if mode == "floorline":
        return _first_int(
            sci.get("clock_floorline_interval_cycles"),
            _micro_first_int(rec, lane, "fl_cyc"),
        )

    return _first_int(
        sci.get("clock_observed_interval_cycles"),
        _micro_first_int(rec, lane, "obs"),
    )


def traditional_fast_residual_ns(rec: Dict[str, Any], clock: str) -> Optional[int]:
    """Traditional projected-GNSS residual from fragment.ocxoN.science."""
    if clock not in OCXOS:
        return None
    lane = LANE_KEYS[clock]
    frag = _frag(rec)
    sci = science_object(rec, clock)
    return _first_int(
        sci.get("fast_residual_ns"),
        sci.get("reported_pps_fast_residual_ns"),
        _nested_get(frag, lane, "pps_residual", "fast_residual_ns"),
    )


def firmware_delta_fast_ns(rec: Dict[str, Any], clock: str, mode: str) -> Optional[int]:
    if clock not in OCXOS:
        return None
    sci = science_object(rec, clock)
    prefix = "delta_floorline" if mode == "floorline" else "delta_raw"
    if not bool(sci.get(f"{prefix}_valid", False)):
        return None
    return _first_int(sci.get(f"{prefix}_fast_residual_ns"))


def firmware_delta_reference(rec: Dict[str, Any], clock: str, mode: str) -> Optional[int]:
    if clock not in OCXOS:
        return None
    sci = science_object(rec, clock)
    prefix = "delta_floorline" if mode == "floorline" else "delta_raw"
    if not bool(sci.get(f"{prefix}_valid", False)):
        return None
    return _first_int(sci.get(f"{prefix}_reference_interval_cycles"))


def stats_ppb(rec: Dict[str, Any], clock: str) -> Optional[float]:
    if clock not in ("VCLOCK", "OCXO1", "OCXO2"):
        return None
    lane = LANE_KEYS[clock]
    frag = _frag(rec)
    return _first_float(_nested_get(frag, "stats", lane, "ppb"))


# -----------------------------------------------------------------------------
# Collection
# -----------------------------------------------------------------------------


def collect_rows(records: Iterable[Dict[str, Any]],
                 mode: str,
                 use_seed_hints: bool) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    stats: Dict[str, Any] = {
        "records_seen": 0,
        "rows_collected": 0,
        "gaps": 0,
        "mode": mode,
        "firmware_delta_checked": 0,
        "firmware_delta_mismatch": 0,
    }

    prev_pps: Optional[int] = None
    prev_edges: Dict[str, Optional[int]] = {clock: None for clock in CLOCKS}
    prev_intervals: Dict[str, Optional[int]] = {clock: None for clock in CLOCKS}
    sums = {"OCXO1": 0.0, "OCXO2": 0.0}
    counts = {"OCXO1": 0, "OCXO2": 0}
    trad_sums = {"OCXO1": 0.0, "OCXO2": 0.0}
    trad_counts = {"OCXO1": 0, "OCXO2": 0}

    def reset_state() -> None:
        nonlocal prev_edges, prev_intervals
        prev_edges = {clock: None for clock in CLOCKS}
        prev_intervals = {clock: None for clock in CLOCKS}

    for rec in records:
        stats["records_seen"] += 1
        pps = pps_count(rec)
        if pps is None:
            continue

        consecutive = prev_pps is not None and pps == prev_pps + 1
        if prev_pps is not None and not consecutive:
            stats["gaps"] += 1
            reset_state()

        edges = {clock: edge_dwt(rec, clock, mode) for clock in CLOCKS}
        intervals: Dict[str, Optional[int]] = {}

        for clock in CLOCKS:
            computed: Optional[int] = None
            if consecutive and edges[clock] is not None and prev_edges[clock] is not None:
                computed = _delta_u32(int(edges[clock]), int(prev_edges[clock]))
            if computed is None and use_seed_hints:
                computed = interval_hint(rec, clock, mode)
            intervals[clock] = computed

        row: Dict[str, Any] = {
            "pps": pps,
            "campaign": campaign_name(rec),
            "ref": prev_pps if consecutive and prev_intervals.get("VCLOCK") is not None else None,
            "pps_cyc": intervals.get("PPS"),
            "v_cyc": intervals.get("VCLOCK"),
            "o1_cyc": intervals.get("OCXO1"),
            "o2_cyc": intervals.get("OCXO2"),
            "v_ppb_tb": stats_ppb(rec, "VCLOCK"),
            "o1_ppb_tb": stats_ppb(rec, "OCXO1"),
            "o2_ppb_tb": stats_ppb(rec, "OCXO2"),
        }

        for clock, prefix in (("OCXO1", "o1"), ("OCXO2", "o2")):
            ref_cyc = prev_intervals.get("VCLOCK") if consecutive else None
            ocxo_cyc = intervals.get(clock)
            trad = traditional_fast_residual_ns(rec, clock)
            fast_cyc: Optional[int] = None
            diff_trad: Optional[int] = None

            if ref_cyc is not None and ocxo_cyc is not None:
                fast_cyc = int(ref_cyc) - int(ocxo_cyc)
                sums[clock] += float(fast_cyc)
                counts[clock] += 1
                if trad is not None:
                    diff_trad = fast_cyc - int(trad)
            if trad is not None:
                trad_sums[clock] += float(trad)
                trad_counts[clock] += 1

            fw = firmware_delta_fast_ns(rec, clock, mode)
            fw_ref = firmware_delta_reference(rec, clock, mode)
            fw_diff = None
            if fw is not None and fast_cyc is not None:
                stats["firmware_delta_checked"] += 1
                fw_diff = fast_cyc - fw
                if fw_diff != 0:
                    stats["firmware_delta_mismatch"] += 1

            row.update({
                f"{prefix}_ref": ref_cyc,
                f"{prefix}_trad": trad,
                f"{prefix}_fast": fast_cyc,
                f"{prefix}_diff_trad": diff_trad,
                f"{prefix}_mean": (sums[clock] / counts[clock]) if counts[clock] else None,
                f"{prefix}_trad_mean": (trad_sums[clock] / trad_counts[clock]) if trad_counts[clock] else None,
                f"{prefix}_fw": fw,
                f"{prefix}_fw_ref": fw_ref,
                f"{prefix}_fw_diff": fw_diff,
            })

        rows.append(row)
        stats["rows_collected"] += 1

        prev_pps = pps
        prev_edges = edges
        prev_intervals = intervals

    return rows, stats


# -----------------------------------------------------------------------------
# Formatting / analysis
# -----------------------------------------------------------------------------


def _header(show_tb: bool, show_fw: bool) -> Tuple[str, str]:
    parts = [
        f"{'pps':>6s}",
        f"{'ref':>6s}",
        f"{'pps_cyc':>13s}",
        f"{'v_cyc':>13s}",
        f"{'o1_cyc':>13s}",
        f"{'o1_ref':>13s}",
        f"{'o1_trad':>8s}",
        f"{'o1_fast':>8s}",
        f"{'o1-td':>7s}",
        f"{'o1_mean':>9s}",
        f"{'o2_cyc':>13s}",
        f"{'o2_ref':>13s}",
        f"{'o2_trad':>8s}",
        f"{'o2_fast':>8s}",
        f"{'o2-td':>7s}",
        f"{'o2_mean':>9s}",
    ]
    if show_tb:
        parts.extend([
            f"{'o1_ppb_tb':>10s}",
            f"{'o2_ppb_tb':>10s}",
        ])
    if show_fw:
        parts.extend([
            f"{'o1_fw':>7s}",
            f"{'o1_fwΔ':>7s}",
            f"{'o2_fw':>7s}",
            f"{'o2_fwΔ':>7s}",
        ])
    return "  ".join(parts), "  ".join("─" * len(p) for p in parts)


def _row_line(row: Dict[str, Any], show_tb: bool, show_fw: bool) -> str:
    parts = [
        f"{row['pps']:>6d}",
        _fmt_int(row.get("ref"), 6),
        _fmt_int(row.get("pps_cyc"), 13),
        _fmt_int(row.get("v_cyc"), 13),
        _fmt_int(row.get("o1_cyc"), 13),
        _fmt_int(row.get("o1_ref"), 13),
        _fmt_int(row.get("o1_trad"), 8, signed=True),
        _fmt_int(row.get("o1_fast"), 8, signed=True),
        _fmt_int(row.get("o1_diff_trad"), 7, signed=True),
        _fmt_float(row.get("o1_mean"), 9, 3, signed=True),
        _fmt_int(row.get("o2_cyc"), 13),
        _fmt_int(row.get("o2_ref"), 13),
        _fmt_int(row.get("o2_trad"), 8, signed=True),
        _fmt_int(row.get("o2_fast"), 8, signed=True),
        _fmt_int(row.get("o2_diff_trad"), 7, signed=True),
        _fmt_float(row.get("o2_mean"), 9, 3, signed=True),
    ]
    if show_tb:
        parts.extend([
            _fmt_float(row.get("o1_ppb_tb"), 10, 3, signed=True),
            _fmt_float(row.get("o2_ppb_tb"), 10, 3, signed=True),
        ])
    if show_fw:
        parts.extend([
            _fmt_int(row.get("o1_fw"), 7, signed=True),
            _fmt_int(row.get("o1_fw_diff"), 7, signed=True),
            _fmt_int(row.get("o2_fw"), 7, signed=True),
            _fmt_int(row.get("o2_fw_diff"), 7, signed=True),
        ])
    return "  ".join(parts)


def _mean(vals: List[int]) -> Optional[float]:
    return (sum(vals) / float(len(vals))) if vals else None


def _summary_line(label: str, rows: List[Dict[str, Any]], prefix: str) -> None:
    fast = [int(r[f"{prefix}_fast"]) for r in rows if r.get(f"{prefix}_fast") is not None]
    trad = [int(r[f"{prefix}_trad"]) for r in rows if r.get(f"{prefix}_trad") is not None]
    diff = [int(r[f"{prefix}_diff_trad"]) for r in rows if r.get(f"{prefix}_diff_trad") is not None]
    if not fast and not trad:
        return
    print(f"  {label}:")
    if fast:
        print(
            f"    cycle fast: rows={len(fast):,}  mean={_mean(fast):+.3f} cycles/ns-class  "
            f"min={min(fast):+d}  max={max(fast):+d}"
        )
    if trad:
        print(
            f"    traditional: rows={len(trad):,}  mean={_mean(trad):+.3f} ns  "
            f"min={min(trad):+d}  max={max(trad):+d}"
        )
    if diff:
        print(
            f"    cycle-traditional: mean={_mean(diff):+.3f}  "
            f"min={min(diff):+d}  max={max(diff):+d}"
        )


def analyze(campaign: str,
            records: List[Dict[str, Any]],
            mode: str,
            limit: int,
            use_seed_hints: bool,
            show_tb: bool,
            show_fw: bool) -> None:
    rows_all, stats = collect_rows(records, mode=mode, use_seed_hints=use_seed_hints)
    rows = rows_all[:limit] if limit else rows_all

    print(f"Campaign: {campaign or '(file)'}  ({len(records):,} TIMEBASE rows, mode={mode.upper()})")
    print()
    print("Delta cycles report")
    print("═══════════════════")
    print("  Row-oriented cycle-domain comparison of PPS, VCLOCK, OCXO1, and OCXO2.")
    print("  Default mode is FloorLine, using FloorLine DWT bookends.")
    print("  Raw/observed mode uses observed raw DWT bookends.")
    print("  OCXO cycle residual uses delayed VCLOCK reference: fast_cycles = ref_vclock_cycles - ocxo_cycles.")
    print("  Positive oN_fast means OCXO running fast, matching traditional fast_residual_ns.")
    print("  oN-td = cycle fast residual - traditional projected-GNSS residual.")
    if use_seed_hints:
        print("  First row cycle fields may be seeded from TIMEBASE interval fields when no prior bookend exists.")
    else:
        print("  First row and post-gap intervals require in-report bookends; no TIMEBASE seed hints are used.")
    print()

    header, sep = _header(show_tb=show_tb, show_fw=show_fw)
    print(header)
    print(sep)
    for row in rows:
        print(_row_line(row, show_tb=show_tb, show_fw=show_fw))

    print()
    print(f"Rows shown:          {len(rows):,}")
    print(f"Rows collected:      {stats['rows_collected']:,}")
    print(f"Gaps/reset points:   {stats['gaps']:,}")

    print()
    print("Residual summary")
    print("════════════════")
    _summary_line("OCXO1", rows, "o1")
    _summary_line("OCXO2", rows, "o2")

    if show_fw:
        print()
        print("Firmware delta field cross-check")
        print("══════════════════════════════")
        print("  fwΔ = computed cycle-fast residual - fragment.ocxoN.science.delta_<mode>_fast_residual_ns")
        print(f"  checked={stats['firmware_delta_checked']:,}  mismatches={stats['firmware_delta_mismatch']:,}")

    last = next((r for r in reversed(rows) if r.get("o1_mean") is not None or r.get("o2_mean") is not None), None)
    if last:
        print()
        print("Final running means")
        print("═══════════════════")
        if last.get("o1_mean") is not None:
            print(f"  OCXO1 cycle mean: {last['o1_mean']:+.6f} cycles/ns-class")
        if last.get("o2_mean") is not None:
            print(f"  OCXO2 cycle mean: {last['o2_mean']:+.6f} cycles/ns-class")
        if last.get("o1_trad_mean") is not None:
            print(f"  OCXO1 traditional mean: {last['o1_trad_mean']:+.6f} ns")
        if last.get("o2_trad_mean") is not None:
            print(f"  OCXO2 traditional mean: {last['o2_trad_mean']:+.6f} ns")

    print()
    print("Notes")
    print("═════")
    print("  • In FloorLine mode, per-row cycles are computed from FloorLine DWT bookends when")
    print("    adjacent bookends are available, not from slope-only interval fields.")
    print("  • The OCXO residual comparison is intentionally delayed by one TIMEBASE row")
    print("    because OCXO edges close after the PPS/VCLOCK publication row.")
    print("  • Cycle residuals are cycles, but at this scale they are ns-class residuals;")
    print("    the report keeps them in cycles so the common-mode DWT measurement story stays visible.")
    print()


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------


def normalize_mode(mode: str) -> str:
    m = str(mode).strip().lower()
    if m in ("fl", "floor", "floorline", "floor-line"):
        return "floorline"
    if m in ("raw", "obs", "observed", "observation"):
        return "raw"
    raise ValueError("mode must be floorline or raw/observed")


def parse_args(argv: List[str]) -> Dict[str, Any]:
    if len(argv) < 2:
        print("Usage: delta_cycles <campaign> [--floorline|--raw] [--limit N]")
        print("       delta_cycles --file pasted_timebase.txt --campaign Delta1 --raw")
        raise SystemExit(1)

    args: Dict[str, Any] = {
        "campaign": None,
        "file": None,
        "mode": "floorline",
        "limit": 0,
        "use_seed_hints": True,
        "show_tb": True,
        "show_fw": False,
    }

    positional: List[str] = []
    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg in ("--mode", "-m"):
            if i + 1 >= len(argv):
                raise SystemExit("--mode requires floorline or raw")
            args["mode"] = normalize_mode(argv[i + 1])
            i += 2
            continue
        if arg.startswith("--mode="):
            args["mode"] = normalize_mode(arg.split("=", 1)[1])
            i += 1
            continue
        if arg in ("--floorline", "--fl"):
            args["mode"] = "floorline"
            i += 1
            continue
        if arg in ("--raw", "--observed", "--obs"):
            args["mode"] = "raw"
            i += 1
            continue
        if arg == "--limit":
            if i + 1 >= len(argv):
                raise SystemExit("--limit requires an integer")
            args["limit"] = int(argv[i + 1])
            i += 2
            continue
        if arg.startswith("--limit="):
            args["limit"] = int(arg.split("=", 1)[1])
            i += 1
            continue
        if arg == "--file":
            if i + 1 >= len(argv):
                raise SystemExit("--file requires a path")
            args["file"] = argv[i + 1]
            i += 2
            continue
        if arg.startswith("--file="):
            args["file"] = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--campaign":
            if i + 1 >= len(argv):
                raise SystemExit("--campaign requires a name")
            args["campaign"] = argv[i + 1]
            i += 2
            continue
        if arg.startswith("--campaign="):
            args["campaign"] = arg.split("=", 1)[1]
            i += 1
            continue
        if arg == "--no-seed":
            args["use_seed_hints"] = False
            i += 1
            continue
        if arg == "--seed":
            args["use_seed_hints"] = True
            i += 1
            continue
        if arg == "--no-tb":
            args["show_tb"] = False
            i += 1
            continue
        if arg in ("--fw", "--firmware", "--firmware-check"):
            args["show_fw"] = True
            i += 1
            continue
        if arg in ("--no-fw", "--no-firmware"):
            args["show_fw"] = False
            i += 1
            continue

        try:
            args["limit"] = int(arg)
        except ValueError:
            positional.append(arg)
        i += 1

    if positional and not args["campaign"]:
        args["campaign"] = positional[0]
    if not args["campaign"] and not args["file"]:
        raise SystemExit("campaign name is required unless --file is used")

    return args


def main() -> None:
    args = parse_args(sys.argv)
    campaign = args["campaign"] or ""
    if args["file"]:
        records = fetch_timebase_file(args["file"], campaign=args["campaign"])
    else:
        records = fetch_timebase_db(campaign)

    if not records:
        target = f"file {args['file']}" if args["file"] else f"campaign '{campaign}'"
        print(f"No TIMEBASE rows found for {target}")
        return

    if not campaign:
        campaign = campaign_name(records[0]) or ""

    analyze(
        campaign=campaign,
        records=records,
        mode=args["mode"],
        limit=args["limit"],
        use_seed_hints=args["use_seed_hints"],
        show_tb=args["show_tb"],
        show_fw=args["show_fw"],
    )


if __name__ == "__main__":
    main()
