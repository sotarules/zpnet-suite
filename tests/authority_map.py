"""
ZPNet Authority Map v4 — TIMEBASE source/provenance audit.

Step-2 report for the OCXO residual cleanup plan.  This script does not alter
firmware behavior.  It reads persisted TIMEBASE rows and prints, row by row,
which timing surfaces are currently acting as authority versus witness.

Usage:
    python -m zpnet.tests.authority_map <campaign_name> [limit]
    python tests/authority_map.py <campaign_name> [limit]
    python tests/authority_map.py <campaign_name> --tail 40
    python tests/authority_map.py <campaign_name> --verbose

Primary questions answered:
    • Did Beta publish OCXO public ns from PPS projection or measured fallback?
    • Was the PPS projection ACTUAL_BRACKET or STATIC_NEXT_EDGE?
    • Is the public PPS residual moving differently than Alpha's measured-edge
      residual, computed from consecutive measured_gnss_ns values in the SAME
      sign convention as pps_residual.fast_residual_ns?
    • What was process_interrupt's subscriber-visible GNSS-at-event witness when
      that compact field is present?
    • Which DWT surface was published: ordinary event, synthetic, or yardstick
      authority?

This is intentionally a report-only courtroom surface.  It is designed to make
witness-vs-authority ambiguity visible before any authority-path changes.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


LANES = ("ocxo1", "ocxo2")
NS_PER_SECOND = 1_000_000_000

NS_SOURCE_NAMES = {
    0: "NONE",
    1: "MEASURED",
    2: "PPS_PROJECTED",
}

PPS_PROJECTION_SOURCE_NAMES = {
    0: "NONE",
    1: "ACTUAL_BRACKET",
    2: "STATIC_NEXT_EDGE",
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


def _root(rec: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(rec, dict):
        return {}
    inner = rec.get("payload") if "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics(rec: Dict[str, Any]) -> Dict[str, Any]:
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


def _as_bool(v: Any) -> Optional[bool]:
    if v is None:
        return None
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


def _first_int(*values: Any) -> Optional[int]:
    for v in values:
        out = _as_int(v)
        if out is not None:
            return out
    return None


def _first_bool(*values: Any) -> Optional[bool]:
    for v in values:
        out = _as_bool(v)
        if out is not None:
            return out
    return None


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_src(source: str, width: int) -> str:
    if len(source) > width:
        source = source[:width]
    return f"{source:>{width}s}"


def _delta(a: Optional[int], b: Optional[int]) -> Optional[int]:
    if a is None or b is None:
        return None
    return a - b


def source_name(source_id: Optional[int], mapping: Dict[int, str]) -> str:
    if source_id is None:
        return "---"
    return mapping.get(source_id, f"ID{source_id}")


def lane_obj(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Dict[str, Any]:
    # Fragment lane is the canonical compact science row.  Forensics lane is
    # used separately for raw/absolute supporting values.
    candidates = (
        frag.get(lane),
        root.get(lane),
        _nested_get(root, "fragment", lane),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_forensic_lane(forensic: Dict[str, Any], root: Dict[str, Any], lane: str) -> Dict[str, Any]:
    candidates = (
        forensic.get(lane),
        _nested_get(root, "forensics", lane),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_forensics(root: Dict[str, Any], frag: Dict[str, Any], forensic: Dict[str, Any], lane: str) -> Dict[str, Any]:
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


def lane_residual(*lane_payloads: Dict[str, Any]) -> Dict[str, Any]:
    for lane_payload in lane_payloads:
        residual = lane_payload.get("pps_residual") if isinstance(lane_payload, dict) else None
        if isinstance(residual, dict):
            return residual
    return {}


def dwt_yardstick_obj(f: Dict[str, Any]) -> Dict[str, Any]:
    y = f.get("dwt_yardstick")
    return y if isinstance(y, dict) else {}


def dwt_source(f: Dict[str, Any]) -> str:
    y = dwt_yardstick_obj(f)
    if _first_bool(y.get("authority"), f.get("dwt_yardstick_authority")):
        return "YARD_AUTH"
    if _first_bool(f.get("dwt_synthetic")):
        return "SYNTH"
    return "EVENT"


def pps_count(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
        root.get("pps_count"),
        root.get("teensy_pps_vclock_count"),
        root.get("teensy_pps_count"),
    )


def public_gnss_ns(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    return _first_int(frag.get("gnss_ns"), root.get("gnss_ns"), _nested_get(frag, "gnss", "ns"))


def public_lane_ns(*lane_payloads: Dict[str, Any]) -> Optional[int]:
    return _first_int(*(lp.get("ns") for lp in lane_payloads if isinstance(lp, dict)))


def measured_lane_ns(*lane_payloads: Dict[str, Any]) -> Optional[int]:
    return _first_int(*(lp.get("measured_gnss_ns") for lp in lane_payloads if isinstance(lp, dict)))


def subscriber_witness_ns(f: Dict[str, Any]) -> Optional[int]:
    # Present in focused Alpha reports; not currently present in compact paired
    # TIMEBASE rows.  Kept here so the report starts printing it automatically
    # if Beta later exposes the fields.
    return _first_int(f.get("sample_gnss_ns_at_event"), f.get("event_gnss_ns"))


def collect_rows(records: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    previous_measured_ns: Dict[str, Optional[int]] = {lane: None for lane in LANES}
    previous_public_ns: Dict[str, Optional[int]] = {lane: None for lane in LANES}

    for rec in records:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics(rec)
        pps = pps_count(root, frag)
        gnss = public_gnss_ns(root, frag)

        for lane in LANES:
            lp = lane_obj(root, frag, forensic, lane)
            flp = lane_forensic_lane(forensic, root, lane)
            f = lane_forensics(root, frag, forensic, lane)
            y = dwt_yardstick_obj(f)
            residual = lane_residual(lp, flp)
            cycle_diag = flp.get("cycle_residual_diagnostic") if isinstance(flp.get("cycle_residual_diagnostic"), dict) else {}

            ns_src_id = _first_int(lp.get("ns_source_id"), flp.get("ns_source_id"))
            pps_proj_src_id = _first_int(lp.get("pps_projection_source"), flp.get("pps_projection_source"))
            pub_ns = public_lane_ns(lp, flp)
            measured_ns = measured_lane_ns(lp, flp)
            witness_ns = subscriber_witness_ns(f)
            pps_proj_raw_ns = _first_int(flp.get("pps_projected_raw_ns"))

            prev_meas = previous_measured_ns[lane]
            prev_pub = previous_public_ns[lane]
            measured_interval_ns = _delta(measured_ns, prev_meas)
            # Same sign convention as pps_residual.fast_residual_ns:
            # positive means the clock interval is longer than the GNSS second
            # (clock_fast in the existing TIMEBASE residual object doctrine).
            measured_edge_fast_residual_ns = (
                measured_interval_ns - NS_PER_SECOND
                if measured_interval_ns is not None else None
            )
            public_interval_ns = _delta(pub_ns, prev_pub)
            public_fast_residual_from_ns = (
                public_interval_ns - NS_PER_SECOND
                if public_interval_ns is not None else None
            )

            pps_resid = _first_int(residual.get("fast_residual_ns"))
            residual_source_id = _first_int(residual.get("source_id"))
            residual_source = residual.get("source") if isinstance(residual.get("source"), str) else "---"
            if measured_ns is not None:
                previous_measured_ns[lane] = measured_ns
            if pub_ns is not None:
                previous_public_ns[lane] = pub_ns

            last_event_dwt = _first_int(f.get("last_event_dwt"))
            original_dwt = _first_int(f.get("dwt_original_at_event"))
            used_dwt = _first_int(f.get("dwt_used_at_event"))
            predicted_dwt = _first_int(f.get("dwt_predicted_at_event"))
            counter_delta = _first_int(f.get("counter32_delta_since_previous_event"))

            rows.append(
                {
                    "pps": pps,
                    "gnss_ns": gnss,
                    "lane": lane.upper(),
                    "ns_source": source_name(ns_src_id, NS_SOURCE_NAMES),
                    "pps_projection_source": source_name(pps_proj_src_id, PPS_PROJECTION_SOURCE_NAMES),
                    "public_ns": pub_ns,
                    "measured_ns": measured_ns,
                    "subscriber_witness_ns": witness_ns,
                    "pps_projected_raw_ns": pps_proj_raw_ns,
                    "public_minus_measured": _delta(pub_ns, measured_ns),
                    "witness_minus_measured": _delta(witness_ns, measured_ns),
                    "projected_raw_minus_measured": _delta(pps_proj_raw_ns, measured_ns),
                    "measured_interval_ns": measured_interval_ns,
                    "measured_edge_fast_residual_ns": measured_edge_fast_residual_ns,
                    "public_interval_ns": public_interval_ns,
                    "public_fast_residual_from_ns": public_fast_residual_from_ns,
                    "pps_fast_residual_ns": pps_resid,
                    "residual_source_id": residual_source_id,
                    "residual_source": residual_source,
                    "pps_minus_measured_residual": _delta(pps_resid, measured_edge_fast_residual_ns),
                    "public_minus_science_residual": _delta(public_fast_residual_from_ns, pps_resid),
                    "traditional_residual_valid": _first_bool(cycle_diag.get("traditional_residual_valid")),
                    "diagnostic_fast_residual_cycles": _first_int(cycle_diag.get("diagnostic_fast_residual_cycles")),
                    "clock_minus_gnss_actual_cycles": _first_int(cycle_diag.get("clock_minus_gnss_actual_cycles")),
                    "dwt_source": dwt_source(f),
                    "event_dwt": last_event_dwt,
                    "original_dwt": original_dwt,
                    "used_dwt": used_dwt,
                    "predicted_dwt": predicted_dwt,
                    "dwt_original_minus_event": _delta(original_dwt, last_event_dwt),
                    "dwt_used_minus_event": _delta(used_dwt, last_event_dwt),
                    "dwt_published_minus_event_cycles": _first_int(f.get("dwt_published_minus_event_cycles")),
                    "counter_delta": counter_delta,
                    "yardstick_valid": _first_bool(y.get("valid"), f.get("dwt_yardstick_valid")),
                    "yardstick_authority": _first_bool(y.get("authority"), f.get("dwt_yardstick_authority")),
                    "yardstick_auth_error": _first_int(y.get("auth_error_cycles"), f.get("dwt_yardstick_auth_error_cycles")),
                    "yardstick_inferred_minus_observed": _first_int(
                        y.get("inferred_minus_observed_cycles"),
                        f.get("dwt_yardstick_inferred_minus_observed_cycles"),
                    ),
                    "yardstick_gate_agree_count": _first_int(y.get("gate_agree_count"), f.get("dwt_yardstick_gate_agree_count")),
                    "yardstick_gate_excursion_count": _first_int(y.get("gate_excursion_count"), f.get("dwt_yardstick_gate_excursion_count")),
                }
            )
    return rows


def print_table(rows: List[Dict[str, Any]], verbose: bool = False) -> None:
    if not rows:
        print("No rows.")
        return

    print("Authority map")
    print("═════════════")
    print("ns_src: PPS_PROJECTED is Beta public authority; MEASURED is measured fallback.")
    print("proj: ACTUAL_BRACKET / STATIC_NEXT_EDGE are Alpha PPS-projection sources.")
    print("res_src: pps_residual.source when firmware exposes it; BRIDGE_EDGE_INTERVAL is Step-2 science.")
    print("meas_ns: compact Alpha measured_gnss_ns side-channel; meas_fast is computed from consecutive meas_ns values.")
    print("meas_fast/public_fast use the same sign convention as pps_resid: positive means clock_fast.")
    print()

    header = (
        f"{'pps':>6s}  {'lane':>5s}  {'ns_src':>13s}  {'proj':>16s}  {'res_src':>16s}  "
        f"{'pub-meas':>11s}  {'wit-meas':>11s}  "
        f"{'sci_resid':>9s}  {'meas_fast':>10s}  {'sci-meas':>9s}  "
        f"{'pub_fast':>9s}  {'pub-sci':>8s}  "
        f"{'dwt_src':>9s}  {'pub-event':>9s}  {'auth_err':>8s}  {'imo':>6s}  {'cΔ':>10s}"
    )
    print(header)
    print("─" * len(header))
    for r in rows:
        print(
            f"{_fmt_int(r['pps'], 6)}  "
            f"{r['lane']:>5s}  "
            f"{_fmt_src(r['ns_source'], 13)}  "
            f"{_fmt_src(r['pps_projection_source'], 16)}  "
            f"{_fmt_src(r['residual_source'], 16)}  "
            f"{_fmt_int(r['public_minus_measured'], 11, signed=True)}  "
            f"{_fmt_int(r['witness_minus_measured'], 11, signed=True)}  "
            f"{_fmt_int(r['pps_fast_residual_ns'], 9, signed=True)}  "
            f"{_fmt_int(r['measured_edge_fast_residual_ns'], 10, signed=True)}  "
            f"{_fmt_int(r['pps_minus_measured_residual'], 9, signed=True)}  "
            f"{_fmt_int(r['public_fast_residual_from_ns'], 9, signed=True)}  "
            f"{_fmt_int(r['public_minus_science_residual'], 8, signed=True)}  "
            f"{_fmt_src(r['dwt_source'], 9)}  "
            f"{_fmt_int(r['dwt_published_minus_event_cycles'], 9, signed=True)}  "
            f"{_fmt_int(r['yardstick_auth_error'], 8, signed=True)}  "
            f"{_fmt_int(r['yardstick_inferred_minus_observed'], 6, signed=True)}  "
            f"{_fmt_int(r['counter_delta'], 10)}"
        )
        if verbose:
            print(
                f"        public_ns={_fmt_int(r['public_ns'])}  "
                f"measured_ns={_fmt_int(r['measured_ns'])}  "
                f"witness_ns={_fmt_int(r['subscriber_witness_ns'])}  "
                f"pps_proj_raw_ns={_fmt_int(r['pps_projected_raw_ns'])}  "
                f"raw_minus_meas={_fmt_int(r['projected_raw_minus_measured'], signed=True)}  "
                f"residual_source_id={_fmt_int(r['residual_source_id'])}"
            )
            print(
                f"        event_dwt={_fmt_int(r['event_dwt'])}  "
                f"original_dwt={_fmt_int(r['original_dwt'])}  "
                f"used_dwt={_fmt_int(r['used_dwt'])}  "
                f"predicted_dwt={_fmt_int(r['predicted_dwt'])}  "
                f"diag_fast_cycles={_fmt_int(r['diagnostic_fast_residual_cycles'], signed=True)}  "
                f"clock_minus_gnss_cycles={_fmt_int(r['clock_minus_gnss_actual_cycles'], signed=True)}"
            )
    print()


def print_summary(rows: List[Dict[str, Any]]) -> None:
    if not rows:
        return

    print("Summary")
    print("═══════")
    for lane in ("OCXO1", "OCXO2"):
        lane_rows = [r for r in rows if r["lane"] == lane]
        src_counts = Counter(r["ns_source"] for r in lane_rows)
        proj_counts = Counter(r["pps_projection_source"] for r in lane_rows)
        dwt_src_counts = Counter(r["dwt_source"] for r in lane_rows)

        pub_meas = Welford()
        wit_meas = Welford()
        pps_resid = Welford()
        meas_resid = Welford()
        pps_minus_meas = Welford()
        pub_event = Welford()
        auth_err = Welford()
        imo = Welford()

        for r in lane_rows:
            if r["public_minus_measured"] is not None:
                pub_meas.update(float(r["public_minus_measured"]))
            if r["witness_minus_measured"] is not None:
                wit_meas.update(float(r["witness_minus_measured"]))
            if r["pps_fast_residual_ns"] is not None:
                pps_resid.update(float(r["pps_fast_residual_ns"]))
            if r["measured_edge_fast_residual_ns"] is not None:
                meas_resid.update(float(r["measured_edge_fast_residual_ns"]))
            if r["pps_minus_measured_residual"] is not None:
                pps_minus_meas.update(float(r["pps_minus_measured_residual"]))
            if r["dwt_published_minus_event_cycles"] is not None:
                pub_event.update(float(r["dwt_published_minus_event_cycles"]))
            if r["yardstick_auth_error"] is not None:
                auth_err.update(float(r["yardstick_auth_error"]))
            if r["yardstick_inferred_minus_observed"] is not None:
                imo.update(float(r["yardstick_inferred_minus_observed"]))

        print(f"  {lane}")
        print(f"    rows                         = {len(lane_rows)}")
        print(f"    ns_source_counts             = {dict(src_counts)}")
        print(f"    pps_projection_source_counts = {dict(proj_counts)}")
        print(f"    dwt_source_counts            = {dict(dwt_src_counts)}")
        _print_welford("public_minus_measured_ns", pub_meas, "ns")
        _print_welford("witness_minus_measured_ns", wit_meas, "ns")
        _print_welford("pps_fast_residual_ns", pps_resid, "ns")
        _print_welford("measured_edge_fast_residual_ns", meas_resid, "ns")
        _print_welford("pps_minus_measured_residual_ns", pps_minus_meas, "ns")
        _print_welford("dwt_published_minus_event_cycles", pub_event, "cycles")
        _print_welford("yardstick_auth_error_cycles", auth_err, "cycles")
        _print_welford("yardstick_inferred_minus_observed_cycles", imo, "cycles")
        print()

    print("Pass/fail hints")
    print("───────────────")
    print("  • Step 1 changes no firmware authority. Any residual change is unrelated to this report.")
    print("  • measured_edge_fast_residual_ns uses the same sign as pps_fast_residual_ns: positive means clock_fast.")
    print("  • After Step 2, res_src should be BRIDGE_EDGE_INTERVAL and sci_resid should follow Alpha bridge-edge science.")
    print("  • public_minus_science_residual shows how far the public PPS projection diverges from the science residual.")
    print("  • If pps_projection_source is mostly STATIC_NEXT_EDGE, the metrics panel is seeing open-second projection geometry.")
    print("  • If witness_minus_measured stays blank, TIMEBASE does not currently expose process_interrupt's GNSS witness field.")
    print("  • If you need bridge_resolved_count/fallback/phi/span row-by-row, Beta must add those compact Alpha fields to TIMEBASE_FORENSICS.")


def _print_welford(label: str, w: Welford, unit: str) -> None:
    if w.n == 0:
        print(f"    {label:<38s} no samples")
        return
    print(
        f"    {label:<38s} n={w.n:5d}  "
        f"mean={w.mean:+10.3f} {unit:<6s}  sd={w.stddev:10.3f}  "
        f"se={w.stderr:9.3f}  min={w.min_val:+10.0f}  max={w.max_val:+10.0f}"
    )


def parse_args(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Print TIMEBASE authority/witness source map for a campaign.")
    ap.add_argument("campaign", help="campaign name")
    ap.add_argument("limit", nargs="?", type=int, default=None, help="row limit from head unless --tail is used")
    ap.add_argument("--tail", nargs="?", const=40, type=int, default=None, help="show last N rows; default 40")
    ap.add_argument("--verbose", action="store_true", help="print full ns/DWT detail under each row")
    return ap.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    records = fetch_timebase(args.campaign)
    if not records:
        print(f"No TIMEBASE rows found for campaign {args.campaign!r}")
        return 1

    if args.tail is not None:
        records = records[-args.tail:]
    elif args.limit is not None:
        records = records[:args.limit]

    rows = collect_rows(records)
    print(f"Campaign: {args.campaign}  ({len(records)} TIMEBASE rows, {len(rows)} lane rows)")
    print()
    print_table(rows, verbose=args.verbose)
    print_summary(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
