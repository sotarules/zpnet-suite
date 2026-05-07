"""
ZPNet Cycle Prediction Report

Analyze CLOCKS/Gamma lane-local prediction fields published in TIMEBASE_FRAGMENT.

This version is built for the structured TIMEBASE_FRAGMENT prediction object:

    fragment.prediction

Each TIMEBASE row prints VCLOCK, OCXO1, and OCXO2 side by side so cross-lane
interdependence is visible.  The row is intentionally wide.

Primary schema:

    fragment.prediction.<lane>_static_prediction_cycles
    fragment.prediction.<lane>_dynamic_final_prediction_cycles
    fragment.prediction.<lane>_actual_cycles
    fragment.prediction.<lane>_static_residual_cycles
    fragment.prediction.<lane>_dynamic_residual_cycles

where lane is one of:

    vclock
    ocxo1
    ocxo2

Compatibility fallback:

The report also understands the transitional flat top-level Gamma fields:

    <lane>_static_prediction_cycle_count
    <lane>_dynamic_prediction_cycle_count
    <lane>_actual_dwt_cycles_between_edges

and computes residuals from those fields when fragment.prediction is absent.

Usage:
    python cycle_prediction.py <campaign> [limit] [threshold]
    python -m zpnet.tests.cycle_prediction <campaign> [limit] [threshold]

Examples:
    python cycle_prediction.py F
    python cycle_prediction.py F 200
    python cycle_prediction.py F 0 25
"""

from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

from zpnet.shared.db import open_db

DEFAULT_EVENT_THRESHOLD_CYCLES = 25
LANES = ("vclock", "ocxo1", "ocxo2")


@dataclass
class LanePrediction:
    static_prediction_cycles: Optional[int] = None
    dynamic_final_prediction_cycles: Optional[int] = None
    actual_cycles: Optional[int] = None
    static_residual_cycles: Optional[int] = None
    dynamic_residual_cycles: Optional[int] = None
    source: str = "missing"

    @property
    def has_any(self) -> bool:
        return any(
            v is not None
            for v in (
                self.static_prediction_cycles,
                self.dynamic_final_prediction_cycles,
                self.actual_cycles,
                self.static_residual_cycles,
                self.dynamic_residual_cycles,
            )
        )


@dataclass
class Row:
    pps: int
    campaign: str
    lanes: Dict[str, LanePrediction]
    # Useful non-prediction clock facts to keep next to prediction behavior.
    vclock_dwt_cycles_between_edges: Optional[int]
    ocxo1_dwt_cycles_between_edges: Optional[int]
    ocxo2_dwt_cycles_between_edges: Optional[int]
    ocxo1_second_residual_ns: Optional[int]
    ocxo2_second_residual_ns: Optional[int]
    ocxo1_gnss_ns_between_edges: Optional[int]
    ocxo2_gnss_ns_between_edges: Optional[int]
    ocxo1_ppb: Optional[float]
    ocxo2_ppb: Optional[float]
    ocxo1_dac: Optional[float]
    ocxo2_dac: Optional[float]
    flags: str


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
                NULLIF(payload->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->>'pps_count', '')::bigint,
                NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                NULLIF(payload->'fragment'->>'pps_count', '')::bigint
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


def _fmt_float(v: Optional[float], width: int = 0, digits: int = 3, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+.{digits}f}" if signed else f"{v:.{digits}f}"
    return f"{s:>{width}s}" if width else s


def _fmt_str(v: str, width: int = 0) -> str:
    s = v[:width] if width else v
    return f"{s:<{width}s}" if width else s


def _print_stats(name: str, w: Welford, unit: str = "cycles") -> None:
    if w.n == 0:
        print(f"  {name:<34s} no samples")
        return
    print(
        f"  {name:<34s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+10.3f} {unit:<6s}  "
        f"sd={w.stddev:9.3f}  "
        f"se={w.stderr:8.3f}  "
        f"min={w.min_val:+8.0f}  "
        f"max={w.max_val:+8.0f}"
    )


def lane_prediction_from_fragment(frag: Dict[str, Any], lane: str) -> LanePrediction:
    pred = _prediction(frag)

    # Primary structured schema.
    structured = LanePrediction(
        static_prediction_cycles=_first_int(pred.get(f"{lane}_static_prediction_cycles")),
        dynamic_final_prediction_cycles=_first_int(pred.get(f"{lane}_dynamic_final_prediction_cycles")),
        actual_cycles=_first_int(pred.get(f"{lane}_actual_cycles")),
        static_residual_cycles=_first_int(pred.get(f"{lane}_static_residual_cycles")),
        dynamic_residual_cycles=_first_int(pred.get(f"{lane}_dynamic_residual_cycles")),
        source="prediction",
    )
    if structured.has_any:
        return structured

    # Transitional top-level Gamma fields.
    static_prediction = _first_int(frag.get(f"{lane}_static_prediction_cycle_count"))
    dynamic_final = _first_int(frag.get(f"{lane}_dynamic_prediction_cycle_count"))
    actual = _first_int(frag.get(f"{lane}_actual_dwt_cycles_between_edges"))
    static_residual = (
        actual - static_prediction
        if actual is not None and static_prediction is not None
        else None
    )
    dynamic_residual = (
        actual - dynamic_final
        if actual is not None and dynamic_final is not None
        else None
    )

    fallback = LanePrediction(
        static_prediction_cycles=static_prediction,
        dynamic_final_prediction_cycles=dynamic_final,
        actual_cycles=actual,
        static_residual_cycles=static_residual,
        dynamic_residual_cycles=dynamic_residual,
        source="top_level",
    )
    if fallback.has_any:
        return fallback

    # Very old DWT-only schema mapped to VCLOCK only.
    if lane == "vclock":
        static_prediction = _first_int(frag.get("dwt_static_prediction_cycle_count"))
        dynamic_final = _first_int(frag.get("dwt_dynamic_prediction_cycle_count"))
        actual = _first_int(
            frag.get("dwt_cycles_between_pps_vclock"),
            frag.get("vclock_dwt_cycles_between_edges"),
        )
        static_residual = (
            actual - static_prediction
            if actual is not None and static_prediction is not None
            else None
        )
        dynamic_residual = (
            actual - dynamic_final
            if actual is not None and dynamic_final is not None and dynamic_final != 0
            else None
        )
        legacy = LanePrediction(
            static_prediction_cycles=static_prediction,
            dynamic_final_prediction_cycles=dynamic_final,
            actual_cycles=actual,
            static_residual_cycles=static_residual,
            dynamic_residual_cycles=dynamic_residual,
            source="legacy_dwt",
        )
        if legacy.has_any:
            return legacy

    return LanePrediction()


def classify_row(row: Row, threshold: int) -> str:
    flags: List[str] = []

    for lane in LANES:
        lp = row.lanes[lane]
        if not lp.has_any:
            flags.append(f"{lane.upper()}_MISSING")
            continue

        if lp.static_residual_cycles is not None and abs(lp.static_residual_cycles) > threshold:
            flags.append(f"{lane.upper()}_STATIC")
        if lp.dynamic_residual_cycles is not None and abs(lp.dynamic_residual_cycles) > threshold:
            flags.append(f"{lane.upper()}_DYN")
        if lp.static_residual_cycles is not None and lp.dynamic_residual_cycles is not None:
            s = abs(lp.static_residual_cycles)
            d = abs(lp.dynamic_residual_cycles)
            if d < s:
                flags.append(f"{lane.upper()}_DYN_BETTER")
            elif d > s:
                flags.append(f"{lane.upper()}_DYN_WORSE")
            else:
                flags.append(f"{lane.upper()}_TIE")

    # Cross-lane hints.  These are deliberately simple; the point is to flag
    # rows where VCLOCK and OCXO residuals may be moving together.
    v = row.lanes["vclock"].dynamic_residual_cycles
    o1 = row.lanes["ocxo1"].dynamic_residual_cycles
    o2 = row.lanes["ocxo2"].dynamic_residual_cycles
    if v is not None and o1 is not None and o2 is not None:
        if (v > 0 and o1 > 0 and o2 > 0) or (v < 0 and o1 < 0 and o2 < 0):
            flags.append("DYN_SAME_SIGN_ALL")
        if abs(v) > threshold and (abs(o1) > threshold or abs(o2) > threshold):
            flags.append("VCLOCK_OCXO_SPIKE")

    return ",".join(flags)


def build_rows(raw_rows: Iterable[Dict[str, Any]], threshold: int) -> List[Row]:
    out: List[Row] = []

    for rec in raw_rows:
        root = _root(rec)
        frag = _frag(rec)

        pps = _first_int(
            root.get("teensy_pps_vclock_count"),
            root.get("pps_count"),
            frag.get("teensy_pps_vclock_count"),
            frag.get("pps_count"),
        )
        if pps is None:
            continue

        campaign = str(root.get("campaign") or frag.get("campaign") or "")

        lanes = {
            lane: lane_prediction_from_fragment(frag, lane)
            for lane in LANES
        }

        row = Row(
            pps=int(pps),
            campaign=campaign,
            lanes=lanes,
            vclock_dwt_cycles_between_edges=_first_int(
                frag.get("vclock_dwt_cycles_between_edges"),
                frag.get("dwt_cycles_between_pps_vclock"),
            ),
            ocxo1_dwt_cycles_between_edges=_first_int(frag.get("ocxo1_dwt_cycles_between_edges")),
            ocxo2_dwt_cycles_between_edges=_first_int(frag.get("ocxo2_dwt_cycles_between_edges")),
            ocxo1_second_residual_ns=_first_int(frag.get("ocxo1_second_residual_ns")),
            ocxo2_second_residual_ns=_first_int(frag.get("ocxo2_second_residual_ns")),
            ocxo1_gnss_ns_between_edges=_first_int(frag.get("ocxo1_gnss_ns_between_edges")),
            ocxo2_gnss_ns_between_edges=_first_int(frag.get("ocxo2_gnss_ns_between_edges")),
            ocxo1_ppb=_first_float(frag.get("ocxo1_ppb")),
            ocxo2_ppb=_first_float(frag.get("ocxo2_ppb")),
            ocxo1_dac=_first_float(frag.get("ocxo1_dac")),
            ocxo2_dac=_first_float(frag.get("ocxo2_dac")),
            flags="",
        )
        row.flags = classify_row(row, threshold)
        out.append(row)

    return out


def lane_columns(row: Row, lane: str) -> List[str]:
    lp = row.lanes[lane]
    return [
        _fmt_int(lp.static_prediction_cycles, 12),
        _fmt_int(lp.dynamic_final_prediction_cycles, 12),
        _fmt_int(lp.actual_cycles, 12),
        _fmt_int(lp.static_residual_cycles, 7, signed=True),
        _fmt_int(lp.dynamic_residual_cycles, 7, signed=True),
    ]


def print_table(rows: List[Row], limit: int) -> None:
    header_parts = [
        f"{'pps':>7s}",
        f"{'v_stat':>12s}", f"{'v_final':>12s}", f"{'v_actual':>12s}", f"{'v_sres':>7s}", f"{'v_dres':>7s}",
        f"{'o1_stat':>12s}", f"{'o1_final':>12s}", f"{'o1_actual':>12s}", f"{'o1_sres':>7s}", f"{'o1_dres':>7s}",
        f"{'o2_stat':>12s}", f"{'o2_final':>12s}", f"{'o2_actual':>12s}", f"{'o2_sres':>7s}", f"{'o2_dres':>7s}",
        f"{'o1_ns_res':>9s}", f"{'o2_ns_res':>9s}",
        f"{'o1_ppb':>9s}", f"{'o2_ppb':>9s}",
        f"{'o1_dac':>10s}", f"{'o2_dac':>10s}",
        f"{'flags':<60s}",
    ]
    print("  ".join(header_parts))
    print("  ".join("─" * len(part) for part in header_parts))

    shown = 0
    for r in rows:
        parts = [
            f"{r.pps:>7d}",
            *lane_columns(r, "vclock"),
            *lane_columns(r, "ocxo1"),
            *lane_columns(r, "ocxo2"),
            _fmt_int(r.ocxo1_second_residual_ns, 9, signed=True),
            _fmt_int(r.ocxo2_second_residual_ns, 9, signed=True),
            _fmt_float(r.ocxo1_ppb, 9, 3, signed=True),
            _fmt_float(r.ocxo2_ppb, 9, 3, signed=True),
            _fmt_float(r.ocxo1_dac, 10, 3),
            _fmt_float(r.ocxo2_dac, 10, 3),
            _fmt_str(r.flags, 60),
        ]
        print("  ".join(parts))
        shown += 1
        if limit and shown >= limit:
            break


def print_csv(rows: List[Row], limit: int) -> None:
    fields = ["pps"]
    for lane in LANES:
        fields.extend([
            f"{lane}_static_prediction_cycles",
            f"{lane}_dynamic_final_prediction_cycles",
            f"{lane}_actual_cycles",
            f"{lane}_static_residual_cycles",
            f"{lane}_dynamic_residual_cycles",
            f"{lane}_source",
        ])
    fields.extend([
        "vclock_dwt_cycles_between_edges",
        "ocxo1_dwt_cycles_between_edges",
        "ocxo2_dwt_cycles_between_edges",
        "ocxo1_second_residual_ns",
        "ocxo2_second_residual_ns",
        "ocxo1_gnss_ns_between_edges",
        "ocxo2_gnss_ns_between_edges",
        "ocxo1_ppb",
        "ocxo2_ppb",
        "ocxo1_dac",
        "ocxo2_dac",
        "flags",
    ])

    writer = csv.DictWriter(sys.stdout, fieldnames=fields)
    writer.writeheader()

    shown = 0
    for r in rows:
        rec: Dict[str, Any] = {"pps": r.pps}
        for lane in LANES:
            lp = r.lanes[lane]
            rec[f"{lane}_static_prediction_cycles"] = lp.static_prediction_cycles
            rec[f"{lane}_dynamic_final_prediction_cycles"] = lp.dynamic_final_prediction_cycles
            rec[f"{lane}_actual_cycles"] = lp.actual_cycles
            rec[f"{lane}_static_residual_cycles"] = lp.static_residual_cycles
            rec[f"{lane}_dynamic_residual_cycles"] = lp.dynamic_residual_cycles
            rec[f"{lane}_source"] = lp.source
        rec.update({
            "vclock_dwt_cycles_between_edges": r.vclock_dwt_cycles_between_edges,
            "ocxo1_dwt_cycles_between_edges": r.ocxo1_dwt_cycles_between_edges,
            "ocxo2_dwt_cycles_between_edges": r.ocxo2_dwt_cycles_between_edges,
            "ocxo1_second_residual_ns": r.ocxo1_second_residual_ns,
            "ocxo2_second_residual_ns": r.ocxo2_second_residual_ns,
            "ocxo1_gnss_ns_between_edges": r.ocxo1_gnss_ns_between_edges,
            "ocxo2_gnss_ns_between_edges": r.ocxo2_gnss_ns_between_edges,
            "ocxo1_ppb": r.ocxo1_ppb,
            "ocxo2_ppb": r.ocxo2_ppb,
            "ocxo1_dac": r.ocxo1_dac,
            "ocxo2_dac": r.ocxo2_dac,
            "flags": r.flags,
        })
        writer.writerow(rec)
        shown += 1
        if limit and shown >= limit:
            break


def corr(xs: List[float], ys: List[float]) -> Optional[float]:
    if len(xs) != len(ys) or len(xs) < 2:
        return None
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    vx = sum((x - mx) ** 2 for x in xs)
    vy = sum((y - my) ** 2 for y in ys)
    if vx == 0.0 or vy == 0.0:
        return None
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return cov / math.sqrt(vx * vy)


def analyze(campaign: str, limit: int = 0, threshold: int = DEFAULT_EVENT_THRESHOLD_CYCLES, *, csv_mode: bool = False) -> None:
    raw = fetch_timebase(campaign)
    if not raw:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    rows = build_rows(raw, threshold)

    if csv_mode:
        print_csv(rows, limit)
        return

    print(f"Campaign: {campaign}")
    print(f"Rows loaded: {len(rows):,}")
    print(f"Spike threshold: {threshold:,} cycles")
    print()
    print("Column semantics:")
    print("  *_stat   = static/before prediction for the completed lane-local second")
    print("  *_final  = dynamic-final/after prediction after Gamma courtroom samples")
    print("  *_actual = naive DWT cycles measured between lane-local one-second edges")
    print("  *_sres   = actual - static prediction")
    print("  *_dres   = actual - dynamic-final prediction")
    print("  o*_ns_res = OCXO fast-positive measured GNSS residual in ns")
    print()

    print_table(rows, limit)
    print()

    # Summary stats.
    stats: Dict[str, Welford] = {}
    for lane in LANES:
        stats[f"{lane}_static_residual"] = Welford()
        stats[f"{lane}_dynamic_residual"] = Welford()

    better = {lane: 0 for lane in LANES}
    worse = {lane: 0 for lane in LANES}
    tie = {lane: 0 for lane in LANES}
    missing = {lane: 0 for lane in LANES}
    sources: Dict[str, Dict[str, int]] = {lane: {} for lane in LANES}

    v_dyn: List[float] = []
    o1_dyn: List[float] = []
    o2_dyn: List[float] = []
    o1_ns: List[float] = []
    o2_ns: List[float] = []

    events: List[Row] = []

    for r in rows:
        for lane in LANES:
            lp = r.lanes[lane]
            sources[lane][lp.source] = sources[lane].get(lp.source, 0) + 1
            if not lp.has_any:
                missing[lane] += 1
                continue

            if lp.static_residual_cycles is not None:
                stats[f"{lane}_static_residual"].update(float(lp.static_residual_cycles))
            if lp.dynamic_residual_cycles is not None:
                stats[f"{lane}_dynamic_residual"].update(float(lp.dynamic_residual_cycles))

            if lp.static_residual_cycles is not None and lp.dynamic_residual_cycles is not None:
                s = abs(lp.static_residual_cycles)
                d = abs(lp.dynamic_residual_cycles)
                if d < s:
                    better[lane] += 1
                elif d > s:
                    worse[lane] += 1
                else:
                    tie[lane] += 1

        vd = r.lanes["vclock"].dynamic_residual_cycles
        o1d = r.lanes["ocxo1"].dynamic_residual_cycles
        o2d = r.lanes["ocxo2"].dynamic_residual_cycles
        if vd is not None and o1d is not None and o2d is not None:
            v_dyn.append(float(vd))
            o1_dyn.append(float(o1d))
            o2_dyn.append(float(o2d))
        if o1d is not None and r.ocxo1_second_residual_ns is not None:
            o1_ns.append(float(r.ocxo1_second_residual_ns))
        if o2d is not None and r.ocxo2_second_residual_ns is not None:
            o2_ns.append(float(r.ocxo2_second_residual_ns))

        if r.flags:
            events.append(r)

    print("Summary")
    print("═══════")
    for lane in LANES:
        _print_stats(f"{lane} static residual", stats[f"{lane}_static_residual"])
        _print_stats(f"{lane} dynamic residual", stats[f"{lane}_dynamic_residual"])
        print(
            f"  {lane:<34s} dynamic better / worse / tie = "
            f"{better[lane]:,} / {worse[lane]:,} / {tie[lane]:,}; "
            f"missing={missing[lane]:,}; sources={sources[lane]}"
        )
        print()

    print("Cross-lane correlations")
    print("═══════════════════════")
    c_v_o1 = corr(v_dyn, o1_dyn)
    c_v_o2 = corr(v_dyn, o2_dyn)
    c_o1_o2 = corr(o1_dyn, o2_dyn)
    print(f"  corr(vclock_dyn_res, ocxo1_dyn_res) = {c_v_o1 if c_v_o1 is not None else 'n/a'}")
    print(f"  corr(vclock_dyn_res, ocxo2_dyn_res) = {c_v_o2 if c_v_o2 is not None else 'n/a'}")
    print(f"  corr(ocxo1_dyn_res,  ocxo2_dyn_res) = {c_o1_o2 if c_o1_o2 is not None else 'n/a'}")
    print()

    if events:
        print("Events / suspicious rows")
        print("════════════════════════")
        print_table(events, limit=50)
        if len(events) > 50:
            print(f"... and {len(events) - 50:,} more event rows")
        print()
    else:
        print("Events / suspicious rows")
        print("════════════════════════")
        print("  None detected above threshold.")
        print()

    print("Notes")
    print("═════")
    print("  • The report prefers fragment.prediction and falls back to transitional")
    print("    top-level Gamma fields while firmware/Pi consumers migrate.")
    print("  • VCLOCK prediction residuals are DWT-ruler quality evidence.")
    print("  • OCXO prediction residuals are lane-local edge prediction evidence.")
    print("  • OCXO ns residuals are measured GNSS interval residuals and therefore")
    print("    include the VCLOCK/DWT ruler dependency.")
    print("  • Use --csv if the table is too wide for the terminal and you want to")
    print("    inspect it in a spreadsheet or pipe it into tooling.")
    print()


def main(argv: List[str]) -> None:
    if len(argv) < 2:
        print("Usage: cycle_prediction.py <campaign> [limit] [threshold] [--csv]")
        print("  limit=0 means all rows. threshold defaults to 25 cycles.")
        raise SystemExit(1)

    csv_mode = "--csv" in argv
    positional = [a for a in argv[1:] if a != "--csv"]

    campaign = positional[0]
    limit = int(positional[1]) if len(positional) >= 2 else 0
    threshold = int(positional[2]) if len(positional) >= 3 else DEFAULT_EVENT_THRESHOLD_CYCLES

    analyze(campaign, limit=limit, threshold=threshold, csv_mode=csv_mode)


if __name__ == "__main__":
    main(sys.argv)
