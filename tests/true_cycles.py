"""
ZPNet True Cycles — endpoint-to-endpoint raw ISR-derived DWT interval report.

This report answers one narrow question:

    Are the OCXO raw ISR-derived event endpoints themselves showing
    second-to-second cycle-count instability consistent with ISR latency?

Unlike raw_cycles.py, this report does NOT use dwt_interval_gate.observed_cycles,
EMA/gate effective intervals, static prediction surfaces, or subscriber-published
DWT values to form the cycle interval.

For each lane and each TIMEBASE row, it selects the latency-adjusted event DWT
endpoint derived from raw ISR entry:

    <lane>.forensics.dwt_event_from_isr_entry_raw

falling back to dwt_original_at_event only for older rows where the explicit
field is not present. Then it computes:

    cyc[n]  = endpoint[n] - endpoint[n-1]  (mod u32)
    cycΔ[n] = cyc[n] - cyc[n-1]

That is the raw truth interval path for ISR-latency investigation.

Usage:
    python -m zpnet.tests.true_cycles <campaign_name> [limit] [clock]
    python true_cycles.py <campaign_name> [limit] [clock]
    python true_cycles.py <campaign_name> --clock OCXO2 [limit]

Clock filter:
    VCLOCK, OCXO1, OCXO2
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db

CLOCKS = ("VCLOCK", "OCXO1", "OCXO2")
EXCURSION_THRESHOLD_CYCLES = 100


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

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
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


def _forensics_root(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    f = root.get("forensics")
    return f if isinstance(f, dict) else {}


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
    for v in values:
        out = _as_int(v)
        if out is not None:
            return out
    return None


def _signed_i32_from_u32_delta(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_str(v: Optional[str], width: int = 0) -> str:
    s = "---" if v is None else str(v)
    if width and len(s) > width:
        s = s[:width]
    return f"{s:>{width}s}" if width else s


def normalize_clock_filter(clock: Optional[str]) -> Optional[str]:
    if clock is None:
        return None
    c = clock.strip().upper()
    if c in ("V", "VC", "VCLOCK"):
        return "VCLOCK"
    if c in ("O1", "OCXO1"):
        return "OCXO1"
    if c in ("O2", "OCXO2"):
        return "OCXO2"
    raise ValueError(f"unknown clock '{clock}', expected VCLOCK, OCXO1, or OCXO2")


def selected_clocks(clock_filter: Optional[str]) -> Tuple[str, ...]:
    return CLOCKS if clock_filter is None else (clock_filter,)


def lane_forensics_obj(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Dict[str, Any]:
    candidates = (
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
        _nested_get(root, "fragment", lane, "forensics"),
        _nested_get(root, "clock_forensics", lane, "alpha_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def pps_count_from_schema(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def lane_endpoint_from_schema(root: Dict[str, Any],
                              frag: Dict[str, Any],
                              forensic: Dict[str, Any],
                              lane: str) -> Tuple[Optional[int], str]:
    """Return the latency-adjusted ISR-derived event endpoint for a lane.

    Preferred source is the explicit post-provenance field. Fallbacks are only
    for older records and are tagged so the report can show when it had to fall
    back away from the exact intended field.
    """
    f = lane_forensics_obj(root, frag, forensic, lane)
    prefix = f"{lane}_forensics_"

    explicit = _first_int(
        f.get("dwt_event_from_isr_entry_raw"),
        frag.get(prefix + "dwt_event_from_isr_entry_raw"),
        root.get(prefix + "dwt_event_from_isr_entry_raw"),
        frag.get(f"{lane}_dwt_event_from_isr_entry_raw"),
        root.get(f"{lane}_dwt_event_from_isr_entry_raw"),
    )
    if explicit is not None and explicit != 0:
        return explicit, "isr_evt"

    # Older rows: dwt_original_at_event was the raw observed/event endpoint
    # before we split out dwt_event_from_isr_entry_raw explicitly.
    original = _first_int(
        f.get("dwt_original_at_event"),
        frag.get(prefix + "dwt_original_at_event"),
        root.get(prefix + "dwt_original_at_event"),
    )
    if original is not None and original != 0:
        return original, "orig"

    # Last-ditch legacy alias. This may be subscriber/applied DWT, so the tag is
    # intentionally loud. New campaigns should not need this path.
    legacy = _first_int(
        f.get("last_event_dwt"),
        f.get("alpha_event_last_event_dwt"),
        frag.get(prefix + "last_event_dwt"),
        root.get(prefix + "last_event_dwt"),
        frag.get(f"{lane}_alpha_event_last_event_dwt"),
        root.get(f"{lane}_alpha_event_last_event_dwt"),
    )
    if legacy is not None and legacy != 0:
        return legacy, "legacy"

    return None, "---"


def lane_isr_raw_from_schema(root: Dict[str, Any],
                             frag: Dict[str, Any],
                             forensic: Dict[str, Any],
                             lane: str) -> Optional[int]:
    f = lane_forensics_obj(root, frag, forensic, lane)
    prefix = f"{lane}_forensics_"
    return _first_int(
        f.get("dwt_isr_entry_raw"),
        frag.get(prefix + "dwt_isr_entry_raw"),
        root.get(prefix + "dwt_isr_entry_raw"),
    )


def lane_correction_from_schema(root: Dict[str, Any],
                                frag: Dict[str, Any],
                                forensic: Dict[str, Any],
                                lane: str,
                                endpoint: Optional[int],
                                isr_raw: Optional[int]) -> Optional[int]:
    f = lane_forensics_obj(root, frag, forensic, lane)
    prefix = f"{lane}_forensics_"
    explicit = _first_int(
        f.get("dwt_isr_entry_to_event_correction_cycles"),
        frag.get(prefix + "dwt_isr_entry_to_event_correction_cycles"),
        root.get(prefix + "dwt_isr_entry_to_event_correction_cycles"),
    )
    if explicit is not None:
        return explicit
    if endpoint is not None and isr_raw is not None:
        return _signed_i32_from_u32_delta(endpoint, isr_raw)
    return None


def lane_used_minus_event_from_schema(root: Dict[str, Any],
                                      frag: Dict[str, Any],
                                      forensic: Dict[str, Any],
                                      lane: str,
                                      endpoint: Optional[int]) -> Optional[int]:
    f = lane_forensics_obj(root, frag, forensic, lane)
    prefix = f"{lane}_forensics_"
    explicit = _first_int(
        f.get("dwt_used_minus_event_cycles"),
        frag.get(prefix + "dwt_used_minus_event_cycles"),
        root.get(prefix + "dwt_used_minus_event_cycles"),
    )
    if explicit is not None:
        return explicit
    used = _first_int(
        f.get("dwt_used_at_event"),
        frag.get(prefix + "dwt_used_at_event"),
        root.get(prefix + "dwt_used_at_event"),
    )
    if used is not None and endpoint is not None:
        return _signed_i32_from_u32_delta(used, endpoint)
    return None


def collect_rows(rows: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int]:
    out: List[Dict[str, Any]] = []
    gaps = 0

    prev_pps_count: Optional[int] = None
    prev_endpoint: Dict[str, Optional[int]] = {c.lower(): None for c in CLOCKS}
    prev_cycles: Dict[str, Optional[int]] = {c.lower(): None for c in CLOCKS}

    def reset_after_gap() -> None:
        nonlocal prev_endpoint, prev_cycles
        prev_endpoint = {c.lower(): None for c in CLOCKS}
        prev_cycles = {c.lower(): None for c in CLOCKS}

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics_root(rec)
        pps_count = pps_count_from_schema(root, frag, forensic)
        if pps_count is None:
            continue

        if prev_pps_count is not None and pps_count != prev_pps_count + 1:
            gaps += 1
            reset_after_gap()

        row: Dict[str, Any] = {"pps_count": pps_count}

        for lane in CLOCKS:
            key = lane.lower()
            endpoint, source = lane_endpoint_from_schema(root, frag, forensic, key)
            isr_raw = lane_isr_raw_from_schema(root, frag, forensic, key)
            corr = lane_correction_from_schema(root, frag, forensic, key, endpoint, isr_raw)
            used_minus_event = lane_used_minus_event_from_schema(root, frag, forensic, key, endpoint)

            cycles: Optional[int] = None
            residual: Optional[int] = None
            if endpoint is not None and prev_endpoint[key] is not None:
                cycles = _delta_u32(endpoint, prev_endpoint[key])
                if prev_cycles[key] is not None:
                    residual = cycles - prev_cycles[key]

            row[key] = {
                "endpoint": endpoint,
                "source": source,
                "cycles": cycles,
                "residual": residual,
                "correction": corr,
                "used_minus_event": used_minus_event,
            }

            if endpoint is not None:
                prev_endpoint[key] = endpoint
            if cycles is not None:
                prev_cycles[key] = cycles

        out.append(row)
        prev_pps_count = pps_count

    return out, gaps


def _headers_for(clock_filter: Optional[str], verbose: bool) -> Tuple[str, str]:
    parts = [f"{'pps':>6s}"]
    for lane in selected_clocks(clock_filter):
        prefix = "v" if lane == "VCLOCK" else ("o1" if lane == "OCXO1" else "o2")
        if verbose:
            parts.extend([
                f"{prefix + '_evt':>13s}",
                f"{prefix + '_src':>7s}",
            ])
        parts.extend([
            f"{prefix + '_cyc':>13s}",
            f"{prefix + '_cycΔ':>8s}",
        ])
        if verbose:
            parts.extend([
                f"{prefix + '_corr':>7s}",
                f"{prefix + '_usedΔ':>8s}",
            ])
    header = "  ".join(parts)
    sep = "  ".join("─" * len(p) for p in parts)
    return header, sep


def _row_line(row: Dict[str, Any], clock_filter: Optional[str], verbose: bool) -> str:
    parts = [f"{row['pps_count']:>6d}"]
    for lane in selected_clocks(clock_filter):
        data = row[lane.lower()]
        if verbose:
            parts.extend([
                _fmt_int(data["endpoint"], 13),
                _fmt_str(data["source"], 7),
            ])
        parts.extend([
            _fmt_int(data["cycles"], 13),
            _fmt_int(data["residual"], 8, signed=True),
        ])
        if verbose:
            parts.extend([
                _fmt_int(data["correction"], 7, signed=True),
                _fmt_int(data["used_minus_event"], 8, signed=True),
            ])
    return "  ".join(parts)


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


def _print_welford(name: str, w: Welford) -> None:
    if w.n == 0:
        print(f"  {name:<38s} no samples")
        return
    print(
        f"  {name:<38s} n={w.n:>7,d}  "
        f"mean={w.mean:+12,.3f} cycles  "
        f"sd={w.stddev:10,.3f}  se={w.stderr:9,.3f}  "
        f"min={w.min_val:+12,.3f}  max={w.max_val:+12,.3f}"
    )


def analyze(campaign: str,
            limit: int = 0,
            clock_filter: Optional[str] = None,
            verbose: bool = False,
            excursions_only: bool = False) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    clock_filter = normalize_clock_filter(clock_filter) if clock_filter else None
    collected, gaps = collect_rows(rows)
    if limit:
        collected = collected[:limit]

    clock_label = "ALL" if clock_filter is None else clock_filter
    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={clock_label})")
    print("Doctrine: cyc = DWT event endpoint[n] - endpoint[n-1]; cycΔ = cyc[n] - cyc[n-1]")
    print("Endpoint source preference: dwt_event_from_isr_entry_raw, fallback dwt_original_at_event")
    if gaps:
        print(f"Canonical PPS gaps detected: {gaps:,}; interval/residual state reset after gaps")

    header, sep = _headers_for(clock_filter, verbose)
    print(header)
    print(sep)

    residual_stats: Dict[str, Welford] = {c.lower(): Welford() for c in CLOCKS}
    cycle_stats: Dict[str, Welford] = {c.lower(): Welford() for c in CLOCKS}
    common_stats = Welford()
    diff_stats = Welford()
    shown = 0

    for row in collected:
        for lane in CLOCKS:
            data = row[lane.lower()]
            if data["cycles"] is not None:
                cycle_stats[lane.lower()].update(float(data["cycles"]))
            if data["residual"] is not None:
                residual_stats[lane.lower()].update(float(data["residual"]))

        o1r = row["ocxo1"]["residual"]
        o2r = row["ocxo2"]["residual"]
        if o1r is not None and o2r is not None:
            common_stats.update((float(o1r) + float(o2r)) / 2.0)
            diff_stats.update(float(o2r) - float(o1r))

        if excursions_only:
            values = [row[l.lower()]["residual"] for l in selected_clocks(clock_filter)]
            if not any(v is not None and abs(v) >= EXCURSION_THRESHOLD_CYCLES for v in values):
                continue

        print(_row_line(row, clock_filter, verbose))
        shown += 1

    print()
    print(f"Rows shown: {shown:,} / {len(collected):,}")
    print("Cycle interval stats:")
    for lane in selected_clocks(clock_filter):
        _print_welford(f"{lane.lower()} endpoint interval", cycle_stats[lane.lower()])

    print("Cycle residual stats:")
    for lane in selected_clocks(clock_filter):
        _print_welford(f"{lane.lower()} interval first-difference", residual_stats[lane.lower()])

    if clock_filter is None:
        print("OCXO residual decomposition:")
        _print_welford("ocxo common-mode cycΔ", common_stats)
        _print_welford("ocxo differential cycΔ (o2-o1)", diff_stats)


def parse_args(argv: List[str]) -> Tuple[str, int, Optional[str], bool, bool]:
    if not argv or argv[0] in ("-h", "--help"):
        raise SystemExit(
            "Usage: true_cycles.py <campaign_name> [limit] [clock] "
            "[--clock VCLOCK|OCXO1|OCXO2] [--verbose] [--excursions]"
        )

    campaign = argv[0]
    limit = 0
    clock: Optional[str] = None
    verbose = False
    excursions = False

    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg == "--clock":
            if i + 1 >= len(argv):
                raise SystemExit("--clock requires VCLOCK, OCXO1, or OCXO2")
            clock = argv[i + 1]
            i += 2
        elif arg == "--verbose":
            verbose = True
            i += 1
        elif arg == "--excursions":
            excursions = True
            i += 1
        else:
            try:
                limit = int(arg)
            except ValueError:
                clock = arg
            i += 1

    return campaign, limit, clock, verbose, excursions


def main(argv: List[str]) -> None:
    campaign, limit, clock, verbose, excursions = parse_args(argv)
    analyze(campaign, limit, clock, verbose, excursions)


if __name__ == "__main__":
    main(sys.argv[1:])
