"""
ZPNet Raw Cycles — narrow selected-lane cycle/GNSS edge audit.

Reads TIMEBASE rows for a campaign and prints one tab-delimited row per second.
Each row contains only PPS count plus one selected lane: VCLOCK, OCXO1, or OCXO2.

For each clock the report shows:
  - cycle prediction / actual / residual
  - GNSS predicted interval ns, derived from cycle prediction and the row CPS
  - GNSS actual interval ns, derived from offline-projected edge clocks
  - GNSS edge residual ns = actual edge ns - predicted edge ns
  - GNSS predicted edge clock ns = prior actual edge ns + predicted interval ns
  - GNSS actual edge clock ns = offline projection of this row's DWT-at-edge

Important: the predicted edge clock is an implied absolute nanosecond clock
value, not a hard-coded 1,000,000,000 ns interval.

Usage:
    python raw_cycles.py <campaign_name> [lane] [limit]

Examples:
    python raw_cycles.py Raw1 vclock 50
    python raw_cycles.py Raw1 ocxo1
    python raw_cycles.py Raw1 100        # defaults to vclock
"""

from __future__ import annotations

import json
import sys
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


GNSS_NS_PER_SECOND = 1_000_000_000
DEFAULT_DWT_EXPECTED_PER_PPS = 1_008_000_000
ALL_LANES = ("pps", "vclock", "ocxo1", "ocxo2")
SELECTABLE_LANES = ("vclock", "ocxo1", "ocxo2")


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
    if isinstance(frag, dict):
        return frag
    if str(root.get("schema", "")).startswith("TIMEBASE_FRAGMENT"):
        return root
    return {}


def _forensics(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    fore = root.get("forensics")
    if isinstance(fore, dict):
        return fore
    if str(root.get("schema", "")).startswith("TIMEBASE_FORENSICS"):
        return root
    return {}


def _prediction(frag: Dict[str, Any]) -> Dict[str, Any]:
    pred = frag.get("prediction")
    return pred if isinstance(pred, dict) else {}


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


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _round_div_signed(numer: int, denom: int) -> Optional[int]:
    if denom == 0:
        return None
    sign = -1 if numer < 0 else 1
    n = abs(numer)
    return sign * ((n + denom // 2) // denom)


def _fmt(v: Optional[int], signed: bool = False) -> str:
    if v is None:
        return "---"
    if signed and v > 0:
        return f"+{v}"
    return str(v)


def pps_count_from_schema(root: Dict[str, Any],
                          frag: Dict[str, Any],
                          fore: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        fore.get("pps_count"),
        _nested_get(frag, "pps", "count"),
        _nested_get(fore, "pps", "count"),
        frag.get("teensy_pps_vclock_count"),
        fore.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
        fore.get("teensy_pps_count"),
    )


def lane_prediction(frag: Dict[str, Any], lane: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    pred = _prediction(frag)
    lane_pred = pred.get(lane) if isinstance(pred.get(lane), dict) else {}

    actual = _first_int(
        lane_pred.get("actual_cycles"),
        frag.get(f"{lane}_actual_cycles"),
        pred.get(f"{lane}_actual_cycles"),
    )
    predicted = _first_int(
        lane_pred.get("prediction_cycles"),
        lane_pred.get("static_prediction_cycles"),
        frag.get(f"{lane}_prediction_cycles"),
        pred.get(f"{lane}_prediction_cycles"),
        pred.get(f"{lane}_static_prediction_cycles"),
    )
    residual = _first_int(
        lane_pred.get("residual_cycles"),
        lane_pred.get("static_residual_cycles"),
        frag.get(f"{lane}_residual_cycles"),
        pred.get(f"{lane}_residual_cycles"),
        pred.get(f"{lane}_static_residual_cycles"),
    )
    if residual is None and actual is not None and predicted is not None:
        residual = actual - predicted
    return actual, predicted, residual


def dwt_cycles_per_second_from_schema(root: Dict[str, Any],
                                      frag: Dict[str, Any],
                                      fore: Dict[str, Any],
                                      pps_actual: Optional[int],
                                      vclock_actual: Optional[int]) -> Optional[int]:
    explicit = _first_int(
        _nested_get(frag, "stats", "dwt", "cycles_per_second"),
        _nested_get(frag, "stats", "dwt", "cycles_per_gnss_second"),
        _nested_get(frag, "stats", "dwt", "cps"),
        _nested_get(frag, "dwt", "cycles_per_second"),
        _nested_get(frag, "dwt", "cycles_per_gnss_second"),
        _nested_get(fore, "stats", "dwt", "cycles_per_second"),
        _nested_get(fore, "stats", "dwt", "cycles_per_gnss_second"),
        _nested_get(fore, "stats", "dwt", "cps"),
        frag.get("dwt_cycles_per_second"),
        frag.get("dwt_cycles_per_gnss_second"),
        frag.get("dwt_cps"),
        frag.get("dwt_cycles_per_pps"),
        fore.get("dwt_cycles_per_second"),
        fore.get("dwt_cycles_per_gnss_second"),
        fore.get("dwt_cps"),
        root.get("dwt_cycles_per_second"),
        root.get("dwt_cycles_per_gnss_second"),
        root.get("dwt_cps"),
    )
    if explicit is not None and explicit > 0:
        return explicit
    if pps_actual is not None and pps_actual > 0:
        return pps_actual
    if vclock_actual is not None and vclock_actual > 0:
        return vclock_actual
    return DEFAULT_DWT_EXPECTED_PER_PPS


def pps_vclock_anchor_ns_from_schema(root: Dict[str, Any],
                                     frag: Dict[str, Any],
                                     fore: Dict[str, Any],
                                     pps_count: Optional[int]) -> Tuple[Optional[int], str]:
    explicit = _first_int(
        _nested_get(frag, "pps_vclock", "gnss_ns_at_edge"),
        _nested_get(frag, "pps_vclock", "gnss_ns"),
        _nested_get(frag, "pps_vclock", "ns_at_edge"),
        _nested_get(frag, "vclock", "gnss_ns_at_edge"),
        _nested_get(frag, "vclock", "ns_at_edge"),
        _nested_get(frag, "time", "pps_vclock_ns"),
        _nested_get(frag, "time", "vclock_ns"),
        _nested_get(fore, "pps_vclock", "gnss_ns_at_edge"),
        _nested_get(fore, "pps_vclock", "gnss_ns"),
        _nested_get(fore, "vclock", "gnss_ns_at_edge"),
        _nested_get(fore, "time", "pps_vclock_ns"),
        frag.get("pps_vclock_ns"),
        frag.get("pps_vclock_gnss_ns"),
        frag.get("pps_vclock_gnss_ns_at_edge"),
        frag.get("vclock_ns"),
        frag.get("vclock_gnss_ns"),
        fore.get("pps_vclock_ns"),
        fore.get("pps_vclock_gnss_ns"),
        fore.get("vclock_ns"),
        root.get("pps_vclock_ns"),
        root.get("pps_vclock_gnss_ns"),
        root.get("vclock_ns"),
    )
    if explicit is not None:
        return explicit, "schema"
    if pps_count is not None:
        return pps_count * GNSS_NS_PER_SECOND, "pps_count_relative"
    return None, "missing"


def lane_dwt_at_edge_from_schema(root: Dict[str, Any],
                                 frag: Dict[str, Any],
                                 fore: Dict[str, Any],
                                 lane: str) -> Optional[int]:
    if lane == "pps":
        return _first_int(
            _nested_get(frag, "pps", "dwt_at_edge"),
            _nested_get(fore, "pps", "dwt_at_edge"),
            frag.get("pps_dwt_at_edge"),
            frag.get("physical_pps_dwt_at_edge"),
            frag.get("pps_diag_physical_pps_dwt_normalized_at_edge"),
            frag.get("pps_diag_physical_pps_dwt_raw_at_edge"),
            frag.get("pps_diag_pps_edge_dwt_isr_entry_raw"),
            frag.get("pps_edge_dwt_at_edge"),
            fore.get("pps_dwt_at_edge"),
            root.get("pps_dwt_at_edge"),
        )

    if lane == "vclock":
        return _first_int(
            _nested_get(frag, "dwt", "at_pps_vclock"),
            _nested_get(fore, "dwt", "at_pps_vclock"),
            _nested_get(frag, "pps_vclock", "dwt_at_edge"),
            _nested_get(fore, "pps_vclock", "dwt_at_edge"),
            _nested_get(frag, "vclock", "forensics", "last_event_dwt"),
            _nested_get(fore, "vclock", "forensics", "last_event_dwt"),
            _nested_get(frag, "vclock", "alpha_event", "last_event_dwt"),
            _nested_get(fore, "vclock", "alpha_event", "last_event_dwt"),
            _nested_get(frag, "vclock", "dwt_at_edge"),
            _nested_get(fore, "vclock", "dwt_at_edge"),
            frag.get("dwt_at_pps_vclock"),
            fore.get("dwt_at_pps_vclock"),
            root.get("dwt_at_pps_vclock"),
            frag.get("vclock_forensics_last_event_dwt"),
            fore.get("vclock_forensics_last_event_dwt"),
        )

    return _first_int(
        _nested_get(fore, lane, "forensics", "last_event_dwt"),
        _nested_get(frag, lane, "forensics", "last_event_dwt"),
        _nested_get(fore, lane, "alpha_event", "last_event_dwt"),
        _nested_get(frag, lane, "alpha_event", "last_event_dwt"),
        _nested_get(fore, lane, "service", "boundary_dwt_at_event"),
        _nested_get(frag, lane, "service", "boundary_dwt_at_event"),
        _nested_get(fore, lane, "service", "dwt_at_event"),
        _nested_get(frag, lane, "service", "dwt_at_event"),
        _nested_get(fore, lane, "service", "sample_dwt_at_event"),
        _nested_get(frag, lane, "service", "sample_dwt_at_event"),
        fore.get(f"{lane}_forensics_last_event_dwt"),
        frag.get(f"{lane}_forensics_last_event_dwt"),
        fore.get(f"{lane}_alpha_event_last_event_dwt"),
        frag.get(f"{lane}_alpha_event_last_event_dwt"),
        fore.get(f"{lane}_service_boundary_dwt_at_event"),
        frag.get(f"{lane}_service_boundary_dwt_at_event"),
        fore.get(f"{lane}_dwt_at_edge"),
        frag.get(f"{lane}_dwt_at_edge"),
    )


def ns_from_cycles(cycles: Optional[int], cps: Optional[int]) -> Optional[int]:
    if cycles is None or cps is None or cps <= 0:
        return None
    return _round_div_signed(cycles * GNSS_NS_PER_SECOND, cps)


def project_dwt_to_gnss_ns(dwt_at_edge: Optional[int],
                           anchor_dwt: Optional[int],
                           anchor_ns: Optional[int],
                           cps: Optional[int]) -> Optional[int]:
    if dwt_at_edge is None or anchor_dwt is None or anchor_ns is None:
        return None
    if cps is None or cps <= 0:
        return None
    delta_cycles = _signed_delta_u32(dwt_at_edge, anchor_dwt)
    delta_ns = _round_div_signed(delta_cycles * GNSS_NS_PER_SECOND, cps)
    if delta_ns is None:
        return None
    return anchor_ns + delta_ns




def normalize_lane(selected_lane: str) -> str:
    lane = selected_lane.strip().lower()
    aliases = {
        "v": "vclock",
        "vc": "vclock",
        "vclk": "vclock",
        "vclock": "vclock",
        "1": "ocxo1",
        "o1": "ocxo1",
        "ocxo1": "ocxo1",
        "2": "ocxo2",
        "o2": "ocxo2",
        "ocxo2": "ocxo2",
    }
    lane = aliases.get(lane, lane)
    if lane not in SELECTABLE_LANES:
        raise SystemExit(
            f"Invalid lane '{selected_lane}'. Choose one of: "
            f"{', '.join(SELECTABLE_LANES)}"
        )
    return lane


def _fmt_num(v: Optional[int], signed: bool = False) -> str:
    if v is None:
        return "---"
    if signed:
        return f"{v:+,d}"
    return f"{v:,d}"


def _render_table(headers: List[str], rows: List[List[str]]) -> None:
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))

    print("  ".join(h.rjust(widths[i]) for i, h in enumerate(headers)))
    print("  ".join("─" * widths[i] for i in range(len(headers))))
    for row in rows:
        print("  ".join(row[i].rjust(widths[i]) for i in range(len(headers))))


def analyze(campaign: str, selected_lane: str = "vclock", limit: int = 0) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    lane = normalize_lane(selected_lane)

    headers = [
        "pps",
        "cyc_pred",
        "cyc_actual",
        "cyc_res",
        "gnss_pred_ns",
        "gnss_actual_ns",
        "gnss_edge_res_ns",
        "gnss_pred_edge_ns",
        "gnss_actual_edge_ns",
    ]

    out_rows: List[List[str]] = []
    previous_pps_count: Optional[int] = None
    previous_edge_ns: Optional[int] = None
    shown = 0

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        fore = _forensics(rec)
        if not frag and not fore:
            continue

        pps_count = pps_count_from_schema(root, frag, fore)
        if pps_count is None:
            continue

        if previous_pps_count is not None and pps_count != previous_pps_count + 1:
            out_rows.append([
                f"gap {previous_pps_count:,}→{pps_count:,}",
                "---", "---", "---", "---", "---", "---", "---", "---",
            ])
            previous_edge_ns = None

        pps_actual, _, _ = lane_prediction(frag, "pps")
        v_actual, _, _ = lane_prediction(frag, "vclock")
        cps = dwt_cycles_per_second_from_schema(root, frag, fore, pps_actual, v_actual)
        anchor_ns, _anchor_src = pps_vclock_anchor_ns_from_schema(root, frag, fore, pps_count)
        anchor_dwt = lane_dwt_at_edge_from_schema(root, frag, fore, "vclock")

        actual_cycles, predicted_cycles, residual_cycles = lane_prediction(frag, lane)
        actual_edge_ns = project_dwt_to_gnss_ns(
            lane_dwt_at_edge_from_schema(root, frag, fore, lane),
            anchor_dwt,
            anchor_ns,
            cps,
        )
        predicted_interval_ns = ns_from_cycles(predicted_cycles, cps)

        actual_interval_ns: Optional[int] = None
        predicted_edge_ns: Optional[int] = None
        edge_residual_ns: Optional[int] = None

        if previous_edge_ns is not None and actual_edge_ns is not None:
            actual_interval_ns = actual_edge_ns - previous_edge_ns
        if previous_edge_ns is not None and predicted_interval_ns is not None:
            predicted_edge_ns = previous_edge_ns + predicted_interval_ns
        if actual_edge_ns is not None and predicted_edge_ns is not None:
            edge_residual_ns = actual_edge_ns - predicted_edge_ns

        out_rows.append([
            _fmt_num(pps_count),
            _fmt_num(predicted_cycles),
            _fmt_num(actual_cycles),
            _fmt_num(residual_cycles, signed=True),
            _fmt_num(predicted_interval_ns),
            _fmt_num(actual_interval_ns),
            _fmt_num(edge_residual_ns, signed=True),
            _fmt_num(predicted_edge_ns),
            _fmt_num(actual_edge_ns),
        ])

        if actual_edge_ns is not None:
            previous_edge_ns = actual_edge_ns
        previous_pps_count = pps_count

        shown += 1
        if limit and shown >= limit:
            break

    print(f"Campaign: {campaign}    lane: {lane}    rows: {shown:,}")
    print()
    _render_table(headers, out_rows)


def parse_args(argv: List[str]) -> Tuple[str, str, int]:
    if len(argv) < 2:
        print("Usage: raw_cycles <campaign_name> [lane] [limit]")
        print("  lane: vclock, ocxo1, or ocxo2")
        raise SystemExit(1)

    campaign = argv[1]
    lane = "vclock"
    limit = 0

    if len(argv) >= 3:
        arg2 = argv[2].strip().lower()
        lane_aliases = {"v", "vc", "vclk", "vclock", "1", "o1", "ocxo1", "2", "o2", "ocxo2"}
        if arg2 in lane_aliases:
            lane = arg2
            if len(argv) >= 4:
                limit = int(argv[3])
        else:
            limit = int(argv[2])

    return campaign, lane, limit


def main() -> None:
    campaign, lane, limit = parse_args(sys.argv)
    analyze(campaign, lane, limit)


if __name__ == "__main__":
    main()
