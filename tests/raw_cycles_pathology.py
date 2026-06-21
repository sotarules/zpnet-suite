#!/usr/bin/env python3
"""
ZPNet Raw Cycles Pathology Probe

A focused companion to raw_cycles.py.

This report intentionally ignores normal rows and prints only rows where the
subscriber-facing published rail disagrees with the FloorLine rail by an
implausible amount. It is designed for the VCLOCK one-row endpoint-slip
pathology where:

  *_fl is sane,
  *_pub jumps by roughly one full DWT second,
  then *_pub snaps back on the next row.

Usage:
    python raw_cycles_pathology.py <campaign>
    python raw_cycles_pathology.py <campaign> --clock VCLOCK
    python raw_cycles_pathology.py <campaign> --all-clocks
    python raw_cycles_pathology.py <campaign> --threshold 1000
    python raw_cycles_pathology.py <campaign> --csv

Triggers:
    interval_mismatch:
        abs(pub_interval - fl_interval) >= threshold

    edge_mismatch:
        abs(pub_dwt - fl_dwt, signed u32) >= threshold

    pub_delta_spike:
        abs(pub_interval[n] - pub_interval[n-1]) >= threshold
        while the matching FloorLine delta is not also huge.

The output is deliberately small: only pathological rows, with provenance fields
needed to decide whether the bad value came from FloorLine, the published
endpoint, PPS_VCLOCK/canonical edge species, or parser fallback.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


CLOCKS = ("VCLOCK", "OCXO1", "OCXO2")
LANE_KEYS = {"VCLOCK": "vclock", "OCXO1": "ocxo1", "OCXO2": "ocxo2"}
LANE_NAMES = {v: k for k, v in LANE_KEYS.items()}
LANE_MICRO_PREFIXES = {"vclock": "v", "ocxo1": "o1", "ocxo2": "o2"}


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
    inner = rec.get("payload") if isinstance(rec, dict) and "payload" in rec else rec
    return inner if isinstance(inner, dict) else {}


def _frag(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    frag = root.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _forensics_root(rec: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(rec)
    f = root.get("forensics")
    return f if isinstance(f, dict) else {}


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


def _as_bool(v: Any) -> Optional[bool]:
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        t = v.strip().lower()
        if t in ("true", "t", "yes", "y", "1"):
            return True
        if t in ("false", "f", "no", "n", "0"):
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


def _flat_prefixes(lane: str) -> Tuple[str, ...]:
    return (
        f"{lane}_forensics_",
        f"{lane}_alpha_event_",
        f"{lane}_",
    )


def _micro_values(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  lane: str,
                  *suffixes: str) -> List[Any]:
    prefix = LANE_MICRO_PREFIXES.get(lane)
    if not prefix:
        return []
    values: List[Any] = []
    for suffix in suffixes:
        key = f"{prefix}_{suffix}"
        for source in (forensic, frag, root):
            if isinstance(source, dict):
                values.append(source.get(key))
    return values


def _micro_first_int(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str,
                     *suffixes: str) -> Optional[int]:
    return _first_int(*_micro_values(root, frag, forensic, lane, *suffixes))


def lane_alpha_event(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str) -> Dict[str, Any]:
    candidates = (
        _nested_get(root, "clock_forensics", lane, "alpha_event"),
        _nested_get(frag, "clock_forensics", lane, "alpha_event"),
        _nested_get(forensic, lane, "alpha_event"),
        _nested_get(frag, lane, "alpha_event"),
        _nested_get(root, lane, "alpha_event"),
        _nested_get(forensic, lane, "forensics"),
        _nested_get(frag, lane, "forensics"),
        _nested_get(root, lane, "forensics"),
    )
    for c in candidates:
        if isinstance(c, dict):
            return c
    return {}


def lane_prediction(frag: Dict[str, Any],
                    pred: Dict[str, Any],
                    lane: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    lane_pred = pred.get(lane) if isinstance(pred.get(lane), dict) else {}

    actual = _first_int(
        lane_pred.get("actual_cycles"),
        frag.get(f"{lane}_actual_cycles"),
        pred.get(f"{lane}_actual_cycles"),
    )
    static = _first_int(
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
    if residual is None and actual is not None and static is not None:
        residual = actual - static
    return actual, static, residual


def pps_count_from_schema(root: Dict[str, Any],
                          frag: Dict[str, Any],
                          forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        root.get("pps_count"),
        frag.get("pps_count"),
        forensic.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("teensy_pps_count"),
    )


def physical_pps_dwt_from_schema(root: Dict[str, Any],
                                 frag: Dict[str, Any],
                                 forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        _nested_get(frag, "pps", "dwt_at_edge"),
        frag.get("pps_dwt_at_edge"),
        forensic.get("pps_dwt_at_edge"),
        root.get("pps_dwt_at_edge"),
        frag.get("physical_pps_dwt_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_normalized_at_edge"),
        frag.get("pps_diag_physical_pps_dwt_raw_at_edge"),
        frag.get("pps_diag_pps_edge_dwt_isr_entry_raw"),
        frag.get("pps_edge_dwt_at_edge"),
    )


def pps_vclock_dwt_from_schema(root: Dict[str, Any],
                               frag: Dict[str, Any],
                               forensic: Dict[str, Any]) -> Optional[int]:
    return _first_int(
        frag.get("dwt_at_pps_vclock"),
        root.get("dwt_at_pps_vclock"),
        forensic.get("dwt_at_pps_vclock"),
        _nested_get(frag, "vclock", "dwt_at_edge"),
        _nested_get(root, "vclock", "dwt_at_edge"),
        _nested_get(forensic, "pps_vclock_edge", "authority_dwt_at_edge"),
        _nested_get(frag, "pps_vclock_edge", "authority_dwt_at_edge"),
        _nested_get(root, "pps_vclock_edge", "authority_dwt_at_edge"),
    )


def dwt_authority_diag(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Dict[str, Any]:
    f = lane_alpha_event(root, frag, forensic, lane)
    a = f.get("dwt_authority") if isinstance(f.get("dwt_authority"), dict) else {}

    return {
        "original_dwt": _first_int(
            _micro_first_int(root, frag, forensic, lane, "raw", "orig"),
            a.get("original_at_event"),
            f.get("dwt_original_at_event"),
            *[frag.get(prefix + "dwt_original_at_event") for prefix in _flat_prefixes(lane)],
            *[root.get(prefix + "dwt_original_at_event") for prefix in _flat_prefixes(lane)],
        ),
        "used_dwt": _first_int(
            _micro_first_int(root, frag, forensic, lane, "pub", "used"),
            a.get("used_at_event"),
            f.get("dwt_used_at_event"),
            f.get("last_event_dwt"),
            *[frag.get(prefix + "dwt_used_at_event") for prefix in _flat_prefixes(lane)],
            *[root.get(prefix + "dwt_used_at_event") for prefix in _flat_prefixes(lane)],
        ),
        "predicted_dwt": _first_int(
            _micro_first_int(root, frag, forensic, lane, "ema"),
            a.get("predicted_at_event"),
            f.get("dwt_predicted_at_event"),
            *[frag.get(prefix + "dwt_predicted_at_event") for prefix in _flat_prefixes(lane)],
            *[root.get(prefix + "dwt_predicted_at_event") for prefix in _flat_prefixes(lane)],
        ),
        "synthetic": _first_bool(a.get("synthetic"), f.get("dwt_synthetic")),
    }


def dwt_gate_diag(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  lane: str) -> Dict[str, Any]:
    f = lane_alpha_event(root, frag, forensic, lane)
    gate = f.get("dwt_interval_gate") if isinstance(f.get("dwt_interval_gate"), dict) else {}

    return {
        "observed_cycles": _first_int(
            _micro_first_int(root, frag, forensic, lane, "obs", "raw_cyc"),
            gate.get("observed_cycles"),
            f.get("dwt_interval_observed_cycles"),
        ),
        "effective_cycles": _first_int(
            _micro_first_int(root, frag, forensic, lane, "eff", "ema_cyc"),
            gate.get("effective_cycles"),
            f.get("dwt_interval_effective_cycles"),
        ),
        "residual_cycles": _first_int(
            _micro_first_int(root, frag, forensic, lane, "res"),
            gate.get("residual_cycles"),
            f.get("dwt_interval_residual_cycles"),
        ),
    }


def floorline_diag(root: Dict[str, Any],
                   frag: Dict[str, Any],
                   forensic: Dict[str, Any],
                   lane: str) -> Dict[str, Any]:
    f = lane_alpha_event(root, frag, forensic, lane)
    r = f.get("regression") if isinstance(f.get("regression"), dict) else {}

    def ri(name: str, flat_name: str = "") -> Optional[int]:
        key = flat_name or f"regression_{name}"
        vals: List[Any] = [r.get(name), f.get(key)]
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + key))
            vals.append(root.get(prefix + key))
        return _first_int(*vals)

    micro_fl_dwt = _micro_first_int(root, frag, forensic, lane, "fl")
    micro_observed_dwt = _micro_first_int(root, frag, forensic, lane, "raw", "orig")
    micro_fl_interval = _micro_first_int(root, frag, forensic, lane, "fl_cyc")
    micro_observed_interval = _micro_first_int(root, frag, forensic, lane, "obs")
    micro_fl_err = _micro_first_int(root, frag, forensic, lane, "fl_err")
    micro_present = (
        micro_fl_dwt is not None or
        micro_fl_interval is not None or
        micro_fl_err is not None
    )

    return {
        "valid": True if micro_present else _first_bool(r.get("valid"), f.get("regression_valid")),
        "sequence": ri("sequence"),
        "sample_count": _first_int(ri("sample_count"), 1 if micro_present else None),
        "observed_dwt": _first_int(micro_observed_dwt, ri("observed_dwt_at_event")),
        "inferred_dwt": _first_int(micro_fl_dwt, ri("inferred_dwt_at_event")),
        "reported_interval": _first_int(micro_fl_interval),
        "observed_interval": _first_int(micro_observed_interval),
        "edge_minus_observed": _first_int(micro_fl_err, ri("inferred_minus_observed_cycles")),
        "source": _micro_first_int(root, frag, forensic, lane, "fl_src"),
        "reason": _micro_first_int(root, frag, forensic, lane, "fl_reason"),
        "accepted": _micro_first_int(root, frag, forensic, lane, "fl_acc"),
        "rejected": _micro_first_int(root, frag, forensic, lane, "fl_rej"),
        "buckets": _micro_first_int(root, frag, forensic, lane, "fl_bkt"),
        "interval_error": _micro_first_int(root, frag, forensic, lane, "fl_ierr"),
    }


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _interval_from_endpoints(now: Optional[int], prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    return _delta_u32(now, prev)


def _sub(a: Optional[int], b: Optional[int]) -> Optional[int]:
    if a is None or b is None:
        return None
    return a - b


def _abs_ge(v: Optional[int], threshold: int) -> bool:
    return v is not None and abs(v) >= threshold


def _near(a: Optional[int], b: Optional[int], tol: int) -> bool:
    return a is not None and b is not None and abs(a - b) <= tol


def _fmt(v: Any, signed: bool = False) -> str:
    if v is None:
        return "---"
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return f"{v:+,d}" if signed else f"{v:,d}"
    return str(v)


def normalize_clock(clock: str) -> str:
    c = clock.strip().upper()
    if c in ("V", "VC", "VCLOCK"):
        return "VCLOCK"
    if c in ("O1", "OCXO1"):
        return "OCXO1"
    if c in ("O2", "OCXO2"):
        return "OCXO2"
    raise SystemExit(f"Unknown clock '{clock}'. Use VCLOCK, OCXO1, OCXO2, or --all-clocks.")


def classify(record: Dict[str, Any], previous_record: Optional[Dict[str, Any]],
             threshold: int, one_second_tolerance: int) -> str:
    edge = record.get("pub_edge_minus_fl_edge")
    interval = record.get("pub_minus_fl_interval")
    pub_delta = record.get("pub_delta")
    one_second = record.get("one_second_reference")

    if _near(abs(edge) if edge is not None else None, one_second, one_second_tolerance):
        return "PUB_EDGE_ONE_SECOND_AHEAD" if edge and edge > 0 else "PUB_EDGE_ONE_SECOND_BEHIND"

    if previous_record:
        prev_edge = previous_record.get("pub_edge_minus_fl_edge")
        if (
            _near(abs(prev_edge) if prev_edge is not None else None, one_second, one_second_tolerance)
            and edge is not None and abs(edge) < threshold
            and interval is not None and abs(interval) >= threshold
        ):
            return "SNAPBACK_AFTER_ENDPOINT_SLIP"

    if interval is not None and abs(interval) >= threshold:
        return "PUB_INTERVAL_MISMATCH"

    if pub_delta is not None and abs(pub_delta) >= threshold:
        return "PUB_DELTA_SPIKE"

    return "PATHOLOGY"


def collect(campaign: str, clocks: Tuple[str, ...],
            limit: int = 0) -> Tuple[List[Dict[str, Any]], int]:
    rows = fetch_timebase(campaign)
    if limit:
        rows = rows[:limit]

    out: List[Dict[str, Any]] = []
    gaps = 0
    prev_pps_count: Optional[int] = None
    prev_pps_dwt: Optional[int] = None
    prev_pps_interval: Optional[int] = None

    prev_dwt: Dict[str, Dict[str, Optional[int]]] = {
        c: {"raw": None, "fl": None, "pub": None} for c in CLOCKS
    }
    prev_interval: Dict[str, Dict[str, Optional[int]]] = {
        c: {"raw": None, "fl": None, "pub": None} for c in CLOCKS
    }

    for rec in rows:
        root = _root(rec)
        frag = _frag(rec)
        forensic = _forensics_root(rec)
        pred = _prediction(frag)

        pps_count = pps_count_from_schema(root, frag, forensic)
        if pps_count is None:
            continue

        if prev_pps_count is not None and pps_count != prev_pps_count + 1:
            gaps += 1
            prev_pps_dwt = None
            prev_pps_interval = None
            prev_dwt = {c: {"raw": None, "fl": None, "pub": None} for c in CLOCKS}
            prev_interval = {c: {"raw": None, "fl": None, "pub": None} for c in CLOCKS}

        pps_dwt = physical_pps_dwt_from_schema(root, frag, forensic)
        pps_actual, _, _ = lane_prediction(frag, pred, "pps")
        if pps_actual is None:
            pps_actual = _interval_from_endpoints(pps_dwt, prev_pps_dwt)

        row: Dict[str, Any] = {
            "pps": pps_count,
            "pps_actual": pps_actual,
            "pps_vclock_dwt": pps_vclock_dwt_from_schema(root, frag, forensic),
            "lanes": {},
        }

        for clock in clocks:
            lane = LANE_KEYS[clock]
            pub_actual, _, _ = lane_prediction(frag, pred, lane)
            auth = dwt_authority_diag(root, frag, forensic, lane)
            gate = dwt_gate_diag(root, frag, forensic, lane)
            fl = floorline_diag(root, frag, forensic, lane)

            original_dwt = auth["original_dwt"]
            used_dwt = auth["used_dwt"]
            fl_observed_dwt = fl["observed_dwt"]
            fl_dwt = fl["inferred_dwt"]

            raw_interval = gate.get("observed_cycles")
            if raw_interval is None:
                raw_interval = _interval_from_endpoints(original_dwt, prev_dwt[clock]["raw"])
            if raw_interval is None:
                raw_interval = _interval_from_endpoints(fl_observed_dwt, prev_dwt[clock]["raw"])
            if raw_interval is None:
                raw_interval = pub_actual

            fl_endpoint_interval = _interval_from_endpoints(fl_dwt, prev_dwt[clock]["fl"])
            fl_reported_interval = fl.get("reported_interval")
            fl_interval = fl_endpoint_interval if fl_endpoint_interval is not None else fl_reported_interval

            pub_interval = _interval_from_endpoints(used_dwt, prev_dwt[clock]["pub"])
            if pub_interval is None and used_dwt is not None:
                if fl_dwt is not None and used_dwt == fl_dwt and fl_interval is not None:
                    pub_interval = fl_interval
                elif original_dwt is not None and used_dwt == original_dwt and raw_interval is not None:
                    pub_interval = raw_interval
            if pub_interval is None:
                pub_interval = pub_actual

            raw_delta = _sub(raw_interval, prev_interval[clock]["raw"])
            fl_delta = _sub(fl_interval, prev_interval[clock]["fl"])
            pub_delta = _sub(pub_interval, prev_interval[clock]["pub"])

            pub_minus_fl_interval = _sub(pub_interval, fl_interval)
            pub_edge_minus_fl_edge = (
                _signed_delta_u32(used_dwt, fl_dwt)
                if used_dwt is not None and fl_dwt is not None
                else None
            )
            pub_minus_raw_interval = _sub(pub_interval, raw_interval)
            pub_minus_pps_vclock_edge = (
                _signed_delta_u32(used_dwt, row["pps_vclock_dwt"])
                if used_dwt is not None and row["pps_vclock_dwt"] is not None
                else None
            )

            one_second_reference = pps_actual or fl_interval or pub_interval or raw_interval

            data = {
                "clock": clock,
                "raw_interval": raw_interval,
                "fl_interval": fl_interval,
                "pub_interval": pub_interval,
                "raw_delta": raw_delta,
                "fl_delta": fl_delta,
                "pub_delta": pub_delta,
                "pub_minus_fl_interval": pub_minus_fl_interval,
                "pub_minus_raw_interval": pub_minus_raw_interval,
                "pub_edge_minus_fl_edge": pub_edge_minus_fl_edge,
                "pub_minus_pps_vclock_edge": pub_minus_pps_vclock_edge,
                "raw_dwt": _first_int(original_dwt, fl_observed_dwt),
                "fl_dwt": fl_dwt,
                "pub_dwt": used_dwt,
                "pps_vclock_dwt": row["pps_vclock_dwt"],
                "fl_fit_minus_endpoint_interval": _sub(fl_reported_interval, fl_endpoint_interval),
                "fl_edge_minus_observed": fl.get("edge_minus_observed"),
                "fl_src": fl.get("source"),
                "fl_reason": fl.get("reason"),
                "fl_acc": fl.get("accepted"),
                "fl_rej": fl.get("rejected"),
                "fl_bkt": fl.get("buckets"),
                "fl_ierr": fl.get("interval_error"),
                "pub_synthetic": auth.get("synthetic"),
                "one_second_reference": one_second_reference,
            }
            row["lanes"][clock] = data

            if raw_interval is not None:
                prev_interval[clock]["raw"] = raw_interval
            if fl_interval is not None:
                prev_interval[clock]["fl"] = fl_interval
            if pub_interval is not None:
                prev_interval[clock]["pub"] = pub_interval

            if original_dwt is not None:
                prev_dwt[clock]["raw"] = original_dwt
            elif fl_observed_dwt is not None:
                prev_dwt[clock]["raw"] = fl_observed_dwt
            if fl_dwt is not None:
                prev_dwt[clock]["fl"] = fl_dwt
            if used_dwt is not None:
                prev_dwt[clock]["pub"] = used_dwt

        out.append(row)

        prev_pps_count = pps_count
        if pps_dwt is not None:
            prev_pps_dwt = pps_dwt
        if pps_actual is not None:
            prev_pps_interval = pps_actual

    return out, gaps


def detect_pathologies(collected: List[Dict[str, Any]], clocks: Tuple[str, ...],
                       threshold: int, edge_threshold: int,
                       delta_threshold: int, one_second_tolerance: int) -> List[Dict[str, Any]]:
    found: List[Dict[str, Any]] = []
    previous_by_clock: Dict[str, Optional[Dict[str, Any]]] = {c: None for c in clocks}

    for row in collected:
        for clock in clocks:
            data = row["lanes"][clock]
            triggers: List[str] = []

            if _abs_ge(data.get("pub_minus_fl_interval"), threshold):
                triggers.append("interval")
            if _abs_ge(data.get("pub_edge_minus_fl_edge"), edge_threshold):
                triggers.append("edge")
            if _abs_ge(data.get("pub_delta"), delta_threshold):
                fl_delta = data.get("fl_delta")
                if fl_delta is None or abs(fl_delta) < delta_threshold:
                    triggers.append("pub_delta")

            if triggers:
                event = dict(data)
                event["pps"] = row["pps"]
                event["pps_actual"] = row["pps_actual"]
                event["trigger"] = "+".join(triggers)
                event["class"] = classify(event, previous_by_clock[clock], threshold, one_second_tolerance)
                found.append(event)

            previous_by_clock[clock] = data

    return found


FIELDS = [
    "pps", "clock", "class", "trigger",
    "pub_minus_fl_interval", "pub_edge_minus_fl_edge", "pub_delta", "fl_delta", "raw_delta",
    "pub_interval", "fl_interval", "raw_interval", "pps_actual",
    "fl_src", "fl_reason", "fl_acc", "fl_rej", "fl_bkt", "fl_ierr",
    "pub_dwt", "fl_dwt", "raw_dwt", "pps_vclock_dwt", "pub_minus_pps_vclock_edge",
    "fl_fit_minus_endpoint_interval", "fl_edge_minus_observed", "pub_synthetic",
]


def print_table(events: List[Dict[str, Any]]) -> None:
    widths = {
        "pps": 5,
        "clock": 6,
        "class": 29,
        "trigger": 14,
        "pub_minus_fl_interval": 13,
        "pub_edge_minus_fl_edge": 13,
        "pub_delta": 11,
        "fl_delta": 8,
        "raw_delta": 8,
        "pub_interval": 13,
        "fl_interval": 13,
        "raw_interval": 13,
        "pps_actual": 13,
        "fl_src": 6,
        "fl_reason": 9,
        "fl_acc": 7,
        "fl_rej": 7,
        "fl_bkt": 7,
        "fl_ierr": 8,
        "pub_dwt": 11,
        "fl_dwt": 11,
        "raw_dwt": 11,
        "pps_vclock_dwt": 11,
        "pub_minus_pps_vclock_edge": 13,
        "fl_fit_minus_endpoint_interval": 12,
        "fl_edge_minus_observed": 9,
        "pub_synthetic": 5,
    }
    labels = {
        "pub_minus_fl_interval": "pub-fl_i",
        "pub_edge_minus_fl_edge": "pub-fl_e",
        "pub_minus_pps_vclock_edge": "pub-pvc_e",
        "fl_fit_minus_endpoint_interval": "fit-end_i",
        "fl_edge_minus_observed": "fl-obs_e",
        "pub_synthetic": "synth",
    }

    def label(field: str) -> str:
        return labels.get(field, field)

    header = "  ".join(f"{label(f):>{widths[f]}}" for f in FIELDS)
    sep = "  ".join("─" * widths[f] for f in FIELDS)
    print(header)
    print(sep)
    for e in events:
        cells = []
        for f in FIELDS:
            signed = f in {
                "pub_minus_fl_interval", "pub_edge_minus_fl_edge", "pub_delta",
                "fl_delta", "raw_delta", "pub_minus_pps_vclock_edge",
                "fl_fit_minus_endpoint_interval", "fl_edge_minus_observed",
            }
            cells.append(f"{_fmt(e.get(f), signed=signed):>{widths[f]}}")
        print("  ".join(cells))


def print_csv(events: List[Dict[str, Any]]) -> None:
    writer = csv.DictWriter(sys.stdout, fieldnames=FIELDS, extrasaction="ignore")
    writer.writeheader()
    for e in events:
        writer.writerow({field: e.get(field) for field in FIELDS})


def summarize(events: List[Dict[str, Any]], collected: List[Dict[str, Any]],
              clocks: Tuple[str, ...], threshold: int) -> None:
    print()
    print("Pathology summary")
    print("═════════════════")
    print(f"Rows scanned:        {len(collected):,}")
    print(f"Pathological rows:   {len(events):,}")
    print(f"Threshold cycles:    {threshold:,}")
    if not events:
        print("No rows crossed the pathology threshold.")
        return

    by_clock: Dict[str, int] = {c: 0 for c in clocks}
    by_class: Dict[str, int] = {}
    for e in events:
        by_clock[e["clock"]] = by_clock.get(e["clock"], 0) + 1
        by_class[e["class"]] = by_class.get(e["class"], 0) + 1

    print("By clock:")
    for c in clocks:
        print(f"  {c:<6s} {by_clock.get(c, 0):,}")

    print("By class:")
    for cls, n in sorted(by_class.items()):
        print(f"  {cls:<32s} {n:,}")

    one_second_like = [
        e for e in events
        if e.get("class") in ("PUB_EDGE_ONE_SECOND_AHEAD", "PUB_EDGE_ONE_SECOND_BEHIND")
    ]
    if one_second_like:
        print()
        print("One-second endpoint slips:")
        for e in one_second_like:
            print(
                f"  pps={e['pps']:>5} {e['clock']:<6s} "
                f"edge_error={_fmt(e.get('pub_edge_minus_fl_edge'), signed=True):>13s} "
                f"interval_error={_fmt(e.get('pub_minus_fl_interval'), signed=True):>13s} "
                f"pub_dwt={_fmt(e.get('pub_dwt'))} fl_dwt={_fmt(e.get('fl_dwt'))}"
            )


def parse_args(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Report only raw_cycles published/FloorLine pathologies."
    )
    ap.add_argument("campaign", help="TIMEBASE campaign name")
    ap.add_argument("limit", nargs="?", type=int, default=0,
                    help="optional row limit")
    ap.add_argument("--clock", default="VCLOCK",
                    help="VCLOCK, OCXO1, or OCXO2. Default: VCLOCK")
    ap.add_argument("--all-clocks", action="store_true",
                    help="scan VCLOCK, OCXO1, and OCXO2")
    ap.add_argument("--threshold", type=int, default=1000,
                    help="interval pathology threshold in cycles. Default: 1000")
    ap.add_argument("--edge-threshold", type=int, default=None,
                    help="edge pathology threshold in cycles. Default: --threshold")
    ap.add_argument("--delta-threshold", type=int, default=None,
                    help="published first-difference spike threshold. Default: --threshold")
    ap.add_argument("--one-second-tolerance", type=int, default=64,
                    help="tolerance for classifying one-second endpoint slips. Default: 64 cycles")
    ap.add_argument("--csv", action="store_true",
                    help="emit CSV instead of aligned text")
    return ap.parse_args(argv[1:])


def main() -> None:
    args = parse_args(sys.argv)
    clocks = CLOCKS if args.all_clocks else (normalize_clock(args.clock),)
    edge_threshold = args.edge_threshold if args.edge_threshold is not None else args.threshold
    delta_threshold = args.delta_threshold if args.delta_threshold is not None else args.threshold

    collected, gaps = collect(args.campaign, clocks, args.limit)
    events = detect_pathologies(
        collected,
        clocks,
        threshold=args.threshold,
        edge_threshold=edge_threshold,
        delta_threshold=delta_threshold,
        one_second_tolerance=args.one_second_tolerance,
    )

    if args.csv:
        print_csv(events)
        return

    clock_label = "ALL" if args.all_clocks else clocks[0]
    print(f"Campaign: {args.campaign}  rows={len(collected):,}  gaps={gaps:,}  view={clock_label}")
    print("Raw-cycles pathology probe: published rail vs FloorLine rail")
    print("Only pathological rows are printed.")
    print()

    if events:
        print_table(events)
    summarize(events, collected, clocks, args.threshold)

    print()
    print("Reason codes from FloorLine firmware:")
    print("  0=accepted, 1=init, 2=sample_starved, 3=bucket_starved,")
    print("  4=fit_invalid, 5=fit_outlier, 6=interval_gate, 7=edge_gate")
    print("Source codes:")
    print("  0=observed, 1=floorline")


if __name__ == "__main__":
    main()
