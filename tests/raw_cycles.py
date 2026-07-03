"""
ZPNet Raw Cycles — raw / FloorLine / published cycle audit.

Reads TIMEBASE rows for a campaign and prints a compact one-second DWT-cycle
audit.  PPS is always shown as the base rail.  By default the report shows
VCLOCK, OCXO1, and OCXO2.  A clock filter narrows the view to PPS + one lane.

Usage:
    python -m zpnet.tests.raw_cycles <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> [limit] [clock]
    python raw_cycles.py <campaign_name> --clock OCXO2 [limit]
    python raw_cycles.py <campaign_name> --limit 300 --clock VCLOCK
    python raw_cycles.py <campaign_name> --align-ocxo
    python raw_cycles.py <campaign_name> --delay-pps-vclock
    python raw_cycles.py <campaign_name> --pathology-only
    python raw_cycles.py <campaign_name> --pathology-gate 500

Clock filter:
    VCLOCK, OCXO1, OCXO2

Column doctrine:
    p_act     physical PPS actual interval, in DWT cycles.
    p_Δ       physical PPS first-difference residual.
    p_voff    PPS→VCLOCK DWT-cycle phase offset:
              pps_vclock_dwt_at_edge - physical_pps_dwt_at_edge.

    *_raw     raw/evidence interval from dwt_interval_gate.observed_cycles
              when present, otherwise from original/observed DWT endpoints.
    *_rawΔ    raw interval first-difference residual:
              raw_interval[n] - raw_interval[n-1].
    *_fl      FloorLine inferred interval from consecutive FloorLine DWT
              endpoints when available.  The fitted/slope interval remains
              a fallback/audit surface only.
    *_flΔ     FloorLine endpoint interval first-difference residual:
              fl_interval[n] - fl_interval[n-1].
    *_pub     the subscriber-facing published interval.
              Current FloorLine-deprecated firmware publishes observation-
              derived authority: OCXO *_pub should follow *_raw, while
              VCLOCK *_pub follows physical PPS + p_voff.
    *_pubΔ    published interval first-difference residual:
              pub_interval[n] - pub_interval[n-1].
    *_cΔ      counter32 delta since the previous subscriber event.

First-row PPS0 prologue:
    Current firmware may publish private start_pps0_* interval evidence in
    TIMEBASE_FORENSICS.  When present, raw_cycles uses that private PPS0
    interval as the predecessor for public PPS 1, so p_act and *Δ columns are
    populated on row 1 instead of showing --- merely because PPS0 was not a
    public TIMEBASE row.

Core rail columns are always shown for each selected lane so the report shape
stays stable across current FloorLine firmware builds.  Truly auxiliary columns
(such as counter32 lineage) are still omitted when absent.

Alignment view:
    --align-ocxo / --delay-pps-vclock delays PPS and VCLOCK by one TIMEBASE
    row so the late-published OCXO one-second interval appears beside the
    PPS/VCLOCK interval it physically belongs with.  This intentionally
    misrepresents the original publication row relationship.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


CLOCKS = ("VCLOCK", "OCXO1", "OCXO2")
LANE_KEYS = {"VCLOCK": "vclock", "OCXO1": "ocxo1", "OCXO2": "ocxo2"}
PREFIXES = {"VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}
LANE_MICRO_PREFIXES = {"vclock": "v", "ocxo1": "o1", "ocxo2": "o2"}

PATHOLOGY_DEFAULT_GATE_CYCLES = 500
PATHOLOGY_EXPECTED_COUNTER_DELTA_TICKS = 10_000_000

# Mirrors process_interrupt.h's DWT publication tribunal mask bits.  Keep the
# report self-contained so a Pi-side raw_cycles run can decode the verdict
# without importing Teensy firmware headers.
DWT_PUBLICATION_VERDICT_BITS = (
    (1 << 0, "ZERO_DWT"),
    (1 << 1, "SOURCE_MISMATCH"),
    (1 << 2, "FLOORLINE_EDGE"),
    (1 << 3, "FLOORLINE_INTERVAL"),
    (1 << 4, "FLOORLINE_LATE"),
    (1 << 5, "COUNTER_LOW16"),
    (1 << 6, "SERVICE_OFFSET"),
    (1 << 7, "COUNTER_DELTA"),
    (1 << 8, "OBSERVED_INTERVAL"),
    (1 << 9, "PUBLISHED_INTERVAL"),
    (1 << 10, "CROSS_RAIL"),
    (1 << 11, "GNSS_PROJECTION"),
    (1 << 12, "COUNTER_ADJACENCY"),
    (1 << 13, "YARDSTICK_EXCURSION"),
)


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


def _as_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


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


def _as_str(v: Any) -> Optional[str]:
    if v is None:
        return None
    s = str(v)
    return s if s else None


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


def _first_bool(*values: Any) -> Optional[bool]:
    for v in values:
        out = _as_bool(v)
        if out is not None:
            return out
    return None


def _first_str(*values: Any) -> Optional[str]:
    for v in values:
        out = _as_str(v)
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
    s = "---" if v is None else str(v)
    if width and len(s) > width:
        s = s[:width]
    return f"{s:>{width}s}" if width else s


def _delta_u32(now: int, prev: int) -> int:
    return (now - prev) & 0xFFFFFFFF


def _signed_delta_u32(now: int, prev: int) -> int:
    delta = (now - prev) & 0xFFFFFFFF
    return delta - 0x100000000 if delta > 0x7FFFFFFF else delta


def _interval_delta(current: Optional[int], previous: Optional[int]) -> Optional[int]:
    if current is None or previous is None:
        return None
    return current - previous


def _interval_from_endpoints(now: Optional[int],
                             prev: Optional[int]) -> Optional[int]:
    if now is None or prev is None:
        return None
    return _delta_u32(now, prev)


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


def add_optional(w: Welford, v: Optional[int]) -> None:
    if v is not None:
        w.update(float(v))


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
    """Return the authored PPS/VCLOCK endpoint DWT when TIMEBASE exposes it.

    After FloorLine deprecation, this endpoint is the thing we need for the
    VCLOCK publication doctrine: physical PPS plus the live PPS→VCLOCK phase.
    Some transitional rows still expose a stale/zero phase scalar, while the
    endpoint itself is present and correct in fragment science / micro fields.
    """
    return _first_int(
        frag.get("dwt_at_pps_vclock"),
        forensic.get("dwt_at_pps_vclock"),
        root.get("dwt_at_pps_vclock"),
        _nested_get(frag, "vclock", "science", "pps_vclock_dwt_at_edge"),
        _nested_get(root, "fragment", "vclock", "science", "pps_vclock_dwt_at_edge"),
        _nested_get(root, "vclock", "science", "pps_vclock_dwt_at_edge"),
        forensic.get("v_pub"),
        frag.get("v_pub"),
        root.get("v_pub"),
    )


def pps_vclock_phase_cycles_from_schema(root: Dict[str, Any],
                                        frag: Dict[str, Any],
                                        forensic: Dict[str, Any],
                                        pps_dwt: Optional[int]) -> Optional[int]:
    """Return the DWT-cycle offset from physical PPS to PPS/VCLOCK.

    Prefer endpoint-derived phase when it is available.  That is the direct
    custody fact the report is auditing: pps_vclock_dwt_at_edge - pps_dwt_at_edge.
    The explicit pps_vclock_phase_cycles scalar is retained as a fallback, but
    transitional firmware can expose it as 0 while the endpoint itself clearly
    carries the real +~50 cycle phase.  In that case, trusting the scalar makes
    every row look pathological even though publication is doing the right
    thing.
    """
    explicit = _first_int(
        _nested_get(frag, "pps", "vclock_phase_cycles"),
        frag.get("pps_vclock_phase_cycles"),
        forensic.get("pps_vclock_phase_cycles"),
        root.get("pps_vclock_phase_cycles"),
    )

    vclock_dwt = pps_vclock_dwt_from_schema(root, frag, forensic)
    endpoint_phase = None
    if vclock_dwt is not None and pps_dwt is not None:
        endpoint_phase = _signed_delta_u32(vclock_dwt, pps_dwt)

    # A lawful PPS→VCLOCK phase is less than one 10 MHz tick.  Leave a broad
    # guard so the parser is robust to CPS changes and rounding, but do not let
    # an unrelated endpoint accidentally replace an explicit scalar.
    endpoint_phase_plausible = (
        endpoint_phase is not None and
        0 <= endpoint_phase <= 512
    )

    if endpoint_phase_plausible:
        if explicit is None:
            return endpoint_phase
        # Transitional deprecation rows can carry explicit 0 while the DWT
        # endpoints show the real phase.  Prefer the direct endpoint fact.
        if explicit == 0 and endpoint_phase != 0:
            return endpoint_phase
        # If both are present but disagree materially, the endpoint is the
        # observable custody fact needed by raw_cycles.
        if abs(endpoint_phase - explicit) > 8:
            return endpoint_phase

    return explicit


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


def lane_alpha_event(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str) -> Dict[str, Any]:
    """Best-effort TIMEBASE Alpha-event object for one lane.

    The firmware has gone through several schemas.  This function deliberately
    searches both current nested clock_forensics objects and older flattened
    lane-forensics locations.
    """
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
    """Values from the compact TIMEBASE_FORENSICS micro raw-cycle schema.

    Newer firmware emits flat fields directly under payload.forensics, e.g.
    v_raw/v_ema/v_fl/v_pub, o1_fl_cyc, o2_fl_err.  The older parser only knew
    how to look inside nested Alpha forensics, so these fields were present in
    TIMEBASE but invisible to raw_cycles.
    """
    prefix = LANE_MICRO_PREFIXES.get(lane)
    if not prefix:
        return []

    values: List[Any] = []
    sources = (forensic, frag, root)
    for suffix in suffixes:
        key = f"{prefix}_{suffix}"
        for source in sources:
            if isinstance(source, dict):
                values.append(source.get(key))
    return values


def _micro_first_int(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str,
                     *suffixes: str) -> Optional[int]:
    return _first_int(*_micro_values(root, frag, forensic, lane, *suffixes))


def _micro_first_bool(root: Dict[str, Any],
                      frag: Dict[str, Any],
                      forensic: Dict[str, Any],
                      lane: str,
                      *suffixes: str) -> Optional[bool]:
    return _first_bool(*_micro_values(root, frag, forensic, lane, *suffixes))


def _micro_first_str(root: Dict[str, Any],
                     frag: Dict[str, Any],
                     forensic: Dict[str, Any],
                     lane: str,
                     *suffixes: str) -> Optional[str]:
    return _first_str(*_micro_values(root, frag, forensic, lane, *suffixes))


def dwt_publication_court_diag(root: Dict[str, Any],
                               frag: Dict[str, Any],
                               forensic: Dict[str, Any],
                               lane: str) -> Dict[str, Any]:
    """Extract the final DWT-at-edge publication tribunal transcript.

    Newer firmware ferries process_interrupt's publication verdict through
    Alpha and into TIMEBASE_FORENSICS as compact flat micro fields:
    v_court_*, o1_court_*, o2_court_*.  Keep a few nested/flat fallbacks so
    focused report rows or transitional payloads can be decoded too.
    """
    f = lane_alpha_event(root, frag, forensic, lane)

    def ci(short_name: str, diag_name: str) -> Optional[int]:
        return _first_int(
            _micro_first_int(root, frag, forensic, lane, f"court_{short_name}"),
            f.get(f"dwt_publication_{diag_name}"),
            frag.get(f"{lane}_forensics_dwt_publication_{diag_name}"),
            root.get(f"{lane}_forensics_dwt_publication_{diag_name}"),
        )

    def cb(short_name: str, diag_name: str) -> Optional[bool]:
        return _first_bool(
            _micro_first_bool(root, frag, forensic, lane, f"court_{short_name}"),
            f.get(f"dwt_publication_{diag_name}"),
            frag.get(f"{lane}_forensics_dwt_publication_{diag_name}"),
            root.get(f"{lane}_forensics_dwt_publication_{diag_name}"),
        )

    def cs(short_name: str, diag_name: str) -> Optional[str]:
        return _first_str(
            _micro_first_str(root, frag, forensic, lane, f"court_{short_name}"),
            f.get(f"dwt_publication_{diag_name}"),
            frag.get(f"{lane}_forensics_dwt_publication_{diag_name}"),
            root.get(f"{lane}_forensics_dwt_publication_{diag_name}"),
        )

    return {
        "schema": _first_str(
            forensic.get("court_schema"),
            frag.get("court_schema"),
            root.get("court_schema"),
        ),
        "valid": cb("valid", "valid"),
        "mask": ci("mask", "verdict_mask"),
        "reason": cs("reason", "verdict_reason"),
        "watchdog_count": ci("wd", "watchdog_count"),
        "gate_cycles": ci("gate", "gate_cycles"),
        "cross_rail_gate_cycles": ci("xgate", "cross_rail_gate_cycles"),
        "service_offset_gate_ticks": ci("svc_gate", "service_offset_gate_ticks"),
        "expected_counter_delta_ticks": ci("exp_cnt", "expected_counter_delta_ticks"),
        "observed_counter_delta_ticks": ci("obs_cnt", "observed_counter_delta_ticks"),
        "expected_interval_cycles": ci("exp_int", "expected_interval_cycles"),
        "published_interval_cycles": ci("pub_int", "published_interval_cycles"),
        "observed_interval_cycles": ci("obs_int", "observed_interval_cycles"),
        "floorline_interval_cycles": ci("fl_int", "floorline_interval_cycles"),
        "published_interval_error_cycles": ci("pub_err", "published_interval_error_cycles"),
        "observed_interval_error_cycles": ci("obs_err", "observed_interval_error_cycles"),
        "floorline_interval_error_cycles": ci("fl_err", "floorline_interval_error_cycles"),
        "published_minus_observed_cycles": ci("pub_obs", "published_minus_observed_cycles"),
        "floorline_minus_observed_cycles": ci("fl_obs", "floorline_minus_observed_cycles"),
        "service_offset_signed_ticks": ci("svc_off", "service_offset_signed_ticks"),
        "vclock_gnss_error_ns": ci("gnss_err", "vclock_gnss_error_ns"),
    }


def dwt_authority_diag(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Dict[str, Any]:
    f = lane_alpha_event(root, frag, forensic, lane)
    a = f.get("dwt_authority") if isinstance(f.get("dwt_authority"), dict) else {}

    def fi(name: str, *legacy: str) -> Optional[int]:
        vals: List[Any] = [a.get(name), f.get(f"dwt_{name}")]
        vals.extend(f.get(k) for k in legacy)
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + "dwt_" + name))
            vals.append(root.get(prefix + "dwt_" + name))
        return _first_int(*vals)

    def fb(name: str, *legacy: str) -> Optional[bool]:
        vals: List[Any] = [a.get(name), f.get(f"dwt_{name}")]
        vals.extend(f.get(k) for k in legacy)
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + "dwt_" + name))
            vals.append(root.get(prefix + "dwt_" + name))
        return _first_bool(*vals)

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
        "event_from_isr_entry_raw": _first_int(
            a.get("event_from_isr_entry_raw"),
            f.get("dwt_event_from_isr_entry_raw"),
        ),
        "synthetic": _first_bool(a.get("synthetic"), f.get("dwt_synthetic")),
        "synthetic_error": _first_int(
            a.get("synthetic_error_cycles"),
            f.get("dwt_synthetic_error_cycles"),
        ),
        "published_minus_event_cycles": _first_int(
            a.get("published_minus_event_cycles"),
            f.get("dwt_published_minus_event_cycles"),
        ),
        "used_minus_event_cycles": _first_int(
            a.get("used_minus_event_cycles"),
            f.get("dwt_used_minus_event_cycles"),
        ),
    }


def dwt_gate_diag(root: Dict[str, Any],
                  frag: Dict[str, Any],
                  forensic: Dict[str, Any],
                  lane: str) -> Dict[str, Any]:
    f = lane_alpha_event(root, frag, forensic, lane)
    gate = f.get("dwt_interval_gate") if isinstance(f.get("dwt_interval_gate"), dict) else {}

    values = {}
    for name, direct in (
        ("valid", "dwt_interval_gate_valid"),
        ("accepted", "dwt_interval_sample_accepted"),
        ("rejected", "dwt_interval_sample_rejected"),
        ("ema_updated", "dwt_interval_ema_updated"),
        ("resync_applied", "dwt_interval_resync_applied"),
    ):
        values[name] = _first_bool(gate.get(name), f.get(direct))

    for name, direct in (
        ("observed_cycles", "dwt_interval_observed_cycles"),
        ("prediction_cycles", "dwt_interval_prediction_cycles"),
        ("effective_cycles", "dwt_interval_effective_cycles"),
        ("residual_cycles", "dwt_interval_residual_cycles"),
        ("threshold_cycles", "dwt_interval_gate_threshold_cycles"),
        ("accept_count", "dwt_interval_accept_count"),
        ("reject_count", "dwt_interval_reject_count"),
        ("resync_count", "dwt_interval_resync_count"),
        ("reject_streak", "dwt_interval_reject_streak"),
    ):
        values[name] = _first_int(gate.get(name), f.get(direct))

    values["observed_cycles"] = _first_int(
        _micro_first_int(root, frag, forensic, lane, "obs", "raw_cyc"),
        values.get("observed_cycles"),
    )
    values["effective_cycles"] = _first_int(
        _micro_first_int(root, frag, forensic, lane, "eff", "ema_cyc"),
        values.get("effective_cycles"),
    )
    values["residual_cycles"] = _first_int(
        _micro_first_int(root, frag, forensic, lane, "res"),
        values.get("residual_cycles"),
    )

    return values


def regression_diag(root: Dict[str, Any],
                    frag: Dict[str, Any],
                    forensic: Dict[str, Any],
                    lane: str) -> Dict[str, Any]:
    """Extract FloorLine / lower-envelope diagnostics.

    The firmware still transports the lower-envelope result through historical
    regression_* fields.  This function names it FloorLine at the report layer.
    """
    f = lane_alpha_event(root, frag, forensic, lane)
    r = f.get("regression") if isinstance(f.get("regression"), dict) else {}

    def ri(name: str, flat_name: str = "") -> Optional[int]:
        key = flat_name or f"regression_{name}"
        vals: List[Any] = [r.get(name), f.get(key)]
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + key))
            vals.append(root.get(prefix + key))
        return _first_int(*vals)

    def rb(name: str, flat_name: str = "") -> Optional[bool]:
        key = flat_name or f"regression_{name}"
        vals: List[Any] = [r.get(name), f.get(key)]
        for prefix in _flat_prefixes(lane):
            vals.append(frag.get(prefix + key))
            vals.append(root.get(prefix + key))
        return _first_bool(*vals)

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
        "valid": _first_bool(rb("valid"), True if micro_present else None),
        "sequence": ri("sequence"),
        "sample_count": _first_int(ri("sample_count"), 1 if micro_present else None),
        "observed_dwt": _first_int(micro_observed_dwt, ri("observed_dwt_at_event")),
        "inferred_dwt": _first_int(micro_fl_dwt, ri("inferred_dwt_at_event")),
        "inferred_interval_cycles": _first_int(micro_fl_interval),
        "observed_interval_cycles": _first_int(micro_observed_interval),
        "inferred_minus_observed": _first_int(micro_fl_err, ri("inferred_minus_observed_cycles")),
        "target_counter32": ri("target_counter32_at_event"),
        "target_hardware16": ri("target_hardware16_at_event"),
        "observed_hardware16": ri("observed_hardware16_at_event"),
        "fit_error_abs_gt4_count": ri("fit_error_abs_gt4_count"),
        "fit_error_max_cycles": ri("fit_error_max_cycles"),
        "fit_error_min_cycles": ri("fit_error_min_cycles"),
    }


def lane_micro_raw_cycles_diag(root: Dict[str, Any],
                               frag: Dict[str, Any],
                               forensic: Dict[str, Any],
                               lane: str) -> Dict[str, Any]:
    """Extract the flat MICRO_RAW_CYCLES_V1 lane fields verbatim.

    The normal report deliberately resolves several aliases into the three
    visible rails.  For pathology work we also want the raw ferry fields so we
    can tell whether a suspicious endpoint came from TIMEBASE_FORENSICS itself
    or from a parser fallback/provenance mistake.
    """
    return {
        "raw_dwt": _micro_first_int(root, frag, forensic, lane, "raw"),
        "orig_dwt": _micro_first_int(root, frag, forensic, lane, "orig"),
        "fl_dwt": _micro_first_int(root, frag, forensic, lane, "fl"),
        "pub_dwt": _micro_first_int(root, frag, forensic, lane, "pub"),
        "used_dwt": _micro_first_int(root, frag, forensic, lane, "used"),
        "ema_dwt": _micro_first_int(root, frag, forensic, lane, "ema"),
        "observed_interval_cycles": _micro_first_int(root, frag, forensic, lane, "obs"),
        "effective_interval_cycles": _micro_first_int(root, frag, forensic, lane, "eff"),
        "floorline_interval_cycles": _micro_first_int(root, frag, forensic, lane, "fl_cyc"),
        "residual_cycles": _micro_first_int(root, frag, forensic, lane, "res"),
        "pps_residual_cycles": _micro_first_int(root, frag, forensic, lane, "pps_res"),
        "floorline_error_cycles": _micro_first_int(root, frag, forensic, lane, "fl_err"),
        "floorline_interval_error_cycles": _micro_first_int(root, frag, forensic, lane, "fl_ierr"),
        "floorline_source": _micro_first_int(root, frag, forensic, lane, "fl_src"),
        "floorline_reason": _micro_first_int(root, frag, forensic, lane, "fl_reason"),
        "floorline_accept_count": _micro_first_int(root, frag, forensic, lane, "fl_acc"),
        "floorline_bucket_count": _micro_first_int(root, frag, forensic, lane, "fl_bkt"),
        "floorline_reject_count": _micro_first_int(root, frag, forensic, lane, "fl_rej"),
    }


def fragment_lane_science_diag(root: Dict[str, Any],
                               frag: Dict[str, Any],
                               lane: str) -> Dict[str, Any]:
    """Extract the paired TIMEBASE_FRAGMENT science fields for a lane.

    Pathology rows can be caused either by firmware publishing a bad endpoint or
    by raw_cycles selecting the wrong schema fallback.  The fragment science
    object is the independent subscriber-facing record, so printing it beside
    MICRO_RAW_CYCLES exposes which side carried the suspicious value.
    """
    lane_obj = frag.get(lane) if isinstance(frag.get(lane), dict) else {}
    if not lane_obj and isinstance(root.get("fragment"), dict):
        root_frag = root.get("fragment")
        lane_obj = root_frag.get(lane) if isinstance(root_frag.get(lane), dict) else {}
    science = lane_obj.get("science") if isinstance(lane_obj.get("science"), dict) else {}

    return {
        "valid": _as_bool(science.get("valid")),
        "schema": _as_str(science.get("schema")),
        "edge_species": _as_str(science.get("edge_species")),
        "frequency_source": _as_str(science.get("frequency_source")),
        "residual_source": _as_str(science.get("residual_source")),
        "public_count": _as_int(science.get("public_count")),
        "delta_reference_public_count": _as_int(science.get("delta_reference_public_count")),
        "delta_publication_public_count": _as_int(science.get("delta_publication_public_count")),
        "raw_dwt": _as_int(science.get("clock_raw_dwt_at_edge")),
        "floorline_dwt": _as_int(science.get("clock_floorline_dwt_at_edge")),
        "published_dwt": _as_int(science.get("clock_published_dwt_at_edge")),
        "pps_vclock_dwt": _as_int(science.get("pps_vclock_dwt_at_edge")),
        "observed_interval_cycles": _as_int(science.get("clock_observed_interval_cycles")),
        "floorline_interval_cycles": _as_int(science.get("clock_floorline_interval_cycles")),
        "delta_raw_clock_interval_cycles": _as_int(science.get("delta_raw_clock_interval_cycles")),
        "delta_raw_reference_interval_cycles": _as_int(science.get("delta_raw_reference_interval_cycles")),
        "delta_raw_residual_cycles": _as_int(science.get("delta_raw_residual_cycles")),
        "delta_floorline_clock_interval_cycles": _as_int(science.get("delta_floorline_clock_interval_cycles")),
        "delta_floorline_reference_interval_cycles": _as_int(science.get("delta_floorline_reference_interval_cycles")),
        "delta_floorline_residual_cycles": _as_int(science.get("delta_floorline_residual_cycles")),
        "reported_minus_canonical_residual_ns": _as_int(science.get("reported_minus_canonical_residual_ns")),
        "reported_minus_floorline_residual_ns": _as_int(science.get("reported_minus_floorline_residual_ns")),
        "clock_interval_ns": _as_int(science.get("clock_interval_ns")),
        "gnss_interval_ns": _as_int(science.get("gnss_interval_ns")),
        "fast_residual_ns": _as_int(science.get("fast_residual_ns")),
        "traditional_fast_residual_ns": _as_int(science.get("traditional_fast_residual_ns")),
        "ppb_1s": _as_float(science.get("ppb_1s")),
        "total_fast_residual_ns": _as_int(science.get("total_fast_residual_ns")),
        "total_ppb": _as_float(science.get("total_ppb")),
        "traditional_total_fast_residual_ns": _as_int(science.get("traditional_total_fast_residual_ns")),
        "traditional_total_ppb": _as_float(science.get("traditional_total_ppb")),
        "ns": _as_int(lane_obj.get("ns")),
        "measured_gnss_ns": _as_int(lane_obj.get("measured_gnss_ns")),
        "measured_minus_ns": _as_int(lane_obj.get("measured_minus_ns")),
        "ns_source_name": _as_str(lane_obj.get("ns_source_name")),
    }


def lane_counter_delta(root: Dict[str, Any],
                       frag: Dict[str, Any],
                       forensic: Dict[str, Any],
                       lane: str) -> Optional[int]:
    f = lane_alpha_event(root, frag, forensic, lane)
    return _first_int(
        f.get("counter32_delta_since_previous_event"),
        _nested_get(f, "dwt_interval_adjacency", "counter_delta_ticks"),
        _nested_get(f, "ocxo_service", "last_counter_delta_ticks"),
        frag.get(f"{lane}_forensics_counter32_delta_since_previous_event"),
        root.get(f"{lane}_forensics_counter32_delta_since_previous_event"),
        frag.get(f"{lane}_counter32_delta_since_previous_event"),
        root.get(f"{lane}_counter32_delta_since_previous_event"),
    )


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


def start_pps0_from_schema(root: Dict[str, Any],
                           frag: Dict[str, Any],
                           forensic: Dict[str, Any]) -> Dict[str, Any]:
    """Return private START-prologue PPS0 interval evidence, if present.

    PPS0 is intentionally not a public TIMEBASE row.  It is the private
    predecessor interval emitted by the firmware so public PPS 1 can be shown
    as a complete first science interval instead of carrying presentation-only
    dashes in the first-difference columns.
    """
    valid = _first_bool(
        forensic.get("start_pps0_valid"),
        frag.get("start_pps0_valid"),
        root.get("start_pps0_valid"),
    )
    if not valid:
        return {"valid": False, "pps": None, "lanes": {}}

    lanes: Dict[str, Dict[str, Optional[int]]] = {}
    for lane in CLOCKS:
        key = LANE_KEYS[lane]
        prefix = LANE_MICRO_PREFIXES[key]
        raw_prev = _first_int(
            forensic.get(f"{prefix}_prev_obs"),
            frag.get(f"{prefix}_prev_obs"),
            root.get(f"{prefix}_prev_obs"),
        )
        fl_prev = _first_int(
            forensic.get(f"{prefix}_prev_fl"),
            frag.get(f"{prefix}_prev_fl"),
            root.get(f"{prefix}_prev_fl"),
        )
        lanes[lane] = {
            "raw": raw_prev,
            "fl": fl_prev,
            # In the current FloorLine-authored firmware, published == FloorLine.
            # Fall back to raw only for older/partial prologue evidence.
            "pub": fl_prev if fl_prev is not None else raw_prev,
        }

    return {
        "valid": True,
        "pps": _first_int(
            forensic.get("pps_prev_obs"),
            frag.get("pps_prev_obs"),
            root.get("pps_prev_obs"),
        ),
        "lanes": lanes,
    }


def _interval_delta_with_private_prev(current: Optional[int],
                                      previous: Optional[int],
                                      private_previous: Optional[int]) -> Optional[int]:
    """First difference, using private PPS0 only when public predecessor is absent."""
    if current is None:
        return None
    if previous is not None:
        return current - previous
    if private_previous is not None:
        return current - private_previous
    return None


def dwt_ppb_from_schema(root: Dict[str, Any], frag: Dict[str, Any]) -> Optional[float]:
    return _first_float(
        _nested_get(frag, "stats", "dwt", "ppb"),
        _nested_get(root, "fragment", "stats", "dwt", "ppb"),
        frag.get("dwt_ppb"),
        root.get("dwt_ppb"),
    )


def collect_rows(rows: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int]:
    out: List[Dict[str, Any]] = []
    gaps = 0

    prev_pps_count: Optional[int] = None
    prev_pps_dwt: Optional[int] = None
    prev_pps_interval: Optional[int] = None
    prev_lane_dwt: Dict[str, Dict[str, Optional[int]]] = {
        lane: {"raw": None, "fl": None, "pub": None}
        for lane in CLOCKS
    }
    prev_lane_interval: Dict[str, Dict[str, Optional[int]]] = {
        lane: {"raw": None, "fl": None, "pub": None}
        for lane in CLOCKS
    }

    def reset_deltas_after_gap() -> None:
        nonlocal prev_pps_dwt, prev_pps_interval, prev_lane_dwt, prev_lane_interval
        prev_pps_dwt = None
        prev_pps_interval = None
        prev_lane_dwt = {
            lane: {"raw": None, "fl": None, "pub": None}
            for lane in CLOCKS
        }
        prev_lane_interval = {
            lane: {"raw": None, "fl": None, "pub": None}
            for lane in CLOCKS
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
            reset_deltas_after_gap()

        private_pps0 = (
            start_pps0_from_schema(root, frag, forensic)
            if prev_pps_count is None
            else {"valid": False, "pps": None, "lanes": {}}
        )

        pps_dwt = physical_pps_dwt_from_schema(root, frag, forensic)
        pps_vclock_phase_cycles = pps_vclock_phase_cycles_from_schema(
            root, frag, forensic, pps_dwt
        )
        pps_actual, pps_pred, _pps_residual = lane_prediction(frag, pred, "pps")
        pps_actual = _first_int(
            forensic.get("pps_obs"),
            frag.get("pps_obs"),
            root.get("pps_obs"),
            pps_actual,
        )
        if pps_actual is None:
            pps_actual = _interval_from_endpoints(pps_dwt, prev_pps_dwt)
        pps_delta = _interval_delta_with_private_prev(
            pps_actual,
            prev_pps_interval,
            private_pps0.get("pps") if private_pps0.get("valid") else None,
        )

        row: Dict[str, Any] = {
            "pps_count": pps_count,
            "pps_actual": pps_actual,
            "pps_delta": pps_delta,
            "pps_dwt": pps_dwt,
            "pps_vclock_phase_cycles": pps_vclock_phase_cycles,
            "dwt_ppb": dwt_ppb_from_schema(root, frag),
            "start_pps0_valid": bool(private_pps0.get("valid")),
            "lanes": {},
        }

        for lane in CLOCKS:
            lane_key = LANE_KEYS[lane]
            pub_actual, _pub_pred, _pub_res = lane_prediction(frag, pred, lane_key)
            f = lane_alpha_event(root, frag, forensic, lane_key)
            auth = dwt_authority_diag(root, frag, forensic, lane_key)
            gate = dwt_gate_diag(root, frag, forensic, lane_key)
            fl = regression_diag(root, frag, forensic, lane_key)
            court = dwt_publication_court_diag(root, frag, forensic, lane_key)

            original_dwt = auth["original_dwt"]
            used_dwt = auth["used_dwt"]
            predicted_dwt = auth["predicted_dwt"]
            fl_observed_dwt = fl["observed_dwt"]
            fl_inferred_dwt = fl["inferred_dwt"]

            raw_interval = gate.get("observed_cycles")
            if raw_interval is None:
                raw_interval = _interval_from_endpoints(original_dwt, prev_lane_dwt[lane]["raw"])
            if raw_interval is None:
                raw_interval = _interval_from_endpoints(fl_observed_dwt, prev_lane_dwt[lane]["raw"])
            if raw_interval is None:
                raw_interval = pub_actual

            # Legacy firmware used dwt_interval_gate.effective_cycles as an
            # EMA-authored interval.  Current EMA-retired firmware still emits
            # *_eff as a compatibility/effective interval, but it is no longer
            # a report rail.  Keep it only as a hidden fallback for old rows
            # whose published endpoint equals a predicted/effective endpoint.
            effective_interval = gate.get("effective_cycles")
            if effective_interval is None:
                effective_interval = _interval_from_endpoints(predicted_dwt, prev_lane_dwt[lane]["pub"])

            # FloorLine has two related cycle-count surfaces:
            #
            #   1. endpoint interval: fl_dwt[n] - fl_dwt[n-1]
            #   2. fitted/slope interval: the lower-envelope fit's current
            #      one-second interval estimate, emitted as *_fl_cyc
            #
            # The visible *_fl column must use the endpoint interval first,
            # because *_pub is also endpoint-derived.  Otherwise raw_cycles
            # compares a fitted interval against a published endpoint interval
            # and falsely suggests that FloorLine is not the publication source.
            fl_reported_interval = fl.get("inferred_interval_cycles")
            fl_endpoint_interval = _interval_from_endpoints(
                fl_inferred_dwt, prev_lane_dwt[lane]["fl"]
            )
            fl_interval = fl_endpoint_interval
            if fl_interval is None:
                fl_interval = fl_reported_interval

            pub_interval = _interval_from_endpoints(used_dwt, prev_lane_dwt[lane]["pub"])
            if pub_interval is None and used_dwt is not None:
                # The micro schema gives endpoint candidates plus per-algorithm
                # intervals.  When the published endpoint equals one candidate,
                # use that candidate's interval even on the first printed row.
                if (
                    fl_inferred_dwt is not None and
                    used_dwt == fl_inferred_dwt and
                    fl_interval is not None
                ):
                    pub_interval = fl_interval
                elif (
                    predicted_dwt is not None and
                    used_dwt == predicted_dwt and
                    effective_interval is not None
                ):
                    pub_interval = effective_interval
                elif (
                    original_dwt is not None and
                    used_dwt == original_dwt and
                    raw_interval is not None
                ):
                    pub_interval = raw_interval
            if pub_interval is None:
                pub_interval = _first_int(
                    f.get("dwt_cycles_between_edges"),
                    pub_actual,
                )

            private_lane_prev = (
                private_pps0.get("lanes", {}).get(lane, {})
                if private_pps0.get("valid")
                else {}
            )

            micro = lane_micro_raw_cycles_diag(root, frag, forensic, lane_key)
            science = fragment_lane_science_diag(root, frag, lane_key)

            raw_dwt = _first_int(original_dwt, fl_observed_dwt)

            # Publication authority changed when FloorLine was deprecated:
            #   VCLOCK publishes physical PPS + the live computed PPS→VCLOCK
            #   phase; OCXO lanes publish the latency-adjusted observed edge.
            # FloorLine remains a diagnostic/courtroom rail, not a publication
            # invariant.  Keep both old and new deltas so old campaigns are
            # still auditable while pathology classification follows the
            # current authority doctrine.
            expected_pub_dwt = None
            expected_pub_authority = None
            if lane == "VCLOCK":
                if pps_dwt is not None and pps_vclock_phase_cycles is not None:
                    expected_pub_dwt = (pps_dwt + pps_vclock_phase_cycles) & 0xFFFFFFFF
                    expected_pub_authority = "PPS_PLUS_PHASE"
                else:
                    expected_pub_dwt = science.get("pps_vclock_dwt")
                    expected_pub_authority = "PPS_VCLOCK_SCIENCE" if expected_pub_dwt is not None else None
            else:
                expected_pub_dwt = raw_dwt
                expected_pub_authority = "OBSERVED" if expected_pub_dwt is not None else None

            data = {
                "raw": raw_interval,
                "raw_delta": _interval_delta_with_private_prev(
                    raw_interval,
                    prev_lane_interval[lane]["raw"],
                    private_lane_prev.get("raw"),
                ),
                "fl": fl_interval,
                "fl_delta": _interval_delta_with_private_prev(
                    fl_interval,
                    prev_lane_interval[lane]["fl"],
                    private_lane_prev.get("fl"),
                ),
                "pub": pub_interval,
                "pub_delta": _interval_delta_with_private_prev(
                    pub_interval,
                    prev_lane_interval[lane]["pub"],
                    private_lane_prev.get("pub"),
                ),
                "counter_delta": lane_counter_delta(root, frag, forensic, lane_key),
                "fl_edge_minus_observed": fl.get("inferred_minus_observed"),
                "pub_minus_raw": _interval_delta(pub_interval, raw_interval),
                "pub_edge_minus_raw_edge": (
                    _signed_delta_u32(used_dwt, raw_dwt)
                    if used_dwt is not None and raw_dwt is not None
                    else None
                ),
                "pub_expected_authority": expected_pub_authority,
                "pub_expected_dwt": expected_pub_dwt,
                "pub_edge_minus_expected": (
                    _signed_delta_u32(used_dwt, expected_pub_dwt)
                    if used_dwt is not None and expected_pub_dwt is not None
                    else None
                ),
                "pub_minus_fl": _interval_delta(pub_interval, fl_interval),
                "pub_edge_minus_fl_edge": (
                    _signed_delta_u32(used_dwt, fl_inferred_dwt)
                    if used_dwt is not None and fl_inferred_dwt is not None
                    else None
                ),
                "fl_fit_minus_endpoint_interval": _interval_delta(
                    fl_reported_interval, fl_endpoint_interval
                ),
                "fl_sample_count": fl.get("sample_count"),
                "fl_valid": fl.get("valid"),
                "pub_synthetic": auth.get("synthetic"),
                "pub_used_dwt": used_dwt,
                "raw_dwt": raw_dwt,
                "fl_dwt": fl_inferred_dwt,
                "micro": micro,
                "science": science,
                "court": court,
            }
            row["lanes"][lane] = data

            for algo in ("raw", "fl", "pub"):
                if data[algo] is not None:
                    prev_lane_interval[lane][algo] = data[algo]

            if original_dwt is not None:
                prev_lane_dwt[lane]["raw"] = original_dwt
            elif fl_observed_dwt is not None:
                prev_lane_dwt[lane]["raw"] = fl_observed_dwt


            if fl_inferred_dwt is not None:
                prev_lane_dwt[lane]["fl"] = fl_inferred_dwt
            if used_dwt is not None:
                prev_lane_dwt[lane]["pub"] = used_dwt

        out.append(row)

        prev_pps_count = pps_count
        if pps_dwt is not None:
            prev_pps_dwt = pps_dwt
        if pps_actual is not None:
            prev_pps_interval = pps_actual

    return out, gaps


def align_pps_vclock_to_ocxo_rows(collected: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], int]:
    """Build an OCXO-aligned view by delaying PPS/VCLOCK one TIMEBASE row.

    TIMEBASE rows are emitted at PPS/VCLOCK.  The OCXO one-second edge is
    defined as the next OCXO edge after PPS/VCLOCK, so the corresponding OCXO
    interval is normally only available in the following TIMEBASE row.  This
    view keeps each current row's OCXO lanes but replaces PPS and VCLOCK with
    the previous row's PPS/VCLOCK values.

    The returned rows are presentation/statistics rows only; they are not
    canonical TIMEBASE rows.  Rows that cannot be aligned, including the first
    row and rows immediately after a PPS gap, are omitted.
    """
    aligned: List[Dict[str, Any]] = []
    skipped = 0

    for i in range(1, len(collected)):
        ref = collected[i - 1]
        cur = collected[i]

        if cur.get("pps_count") != ref.get("pps_count") + 1:
            skipped += 1
            continue

        lanes = {
            lane: dict(cur["lanes"][lane])
            for lane in CLOCKS
        }
        lanes["VCLOCK"] = dict(ref["lanes"]["VCLOCK"])

        row = dict(cur)
        row["pps_count"] = ref["pps_count"]
        row["pps_actual"] = ref["pps_actual"]
        row["pps_delta"] = ref["pps_delta"]
        row["pps_dwt"] = ref.get("pps_dwt")
        row["pps_vclock_phase_cycles"] = ref.get("pps_vclock_phase_cycles")
        row["dwt_ppb"] = ref.get("dwt_ppb")
        row["lanes"] = lanes
        row["alignment_publication_pps_count"] = cur["pps_count"]
        row["alignment_reference_pps_count"] = ref["pps_count"]
        aligned.append(row)

    return aligned, skipped + (1 if collected else 0)


def available_columns(collected: List[Dict[str, Any]],
                      clocks: Tuple[str, ...]) -> Dict[str, List[str]]:
    # The four algorithm rails are the purpose of this report.  Keep them
    # visible even when the current campaign/build does not expose one of the
    # rails yet; otherwise a missing FloorLine extraction looks like a report
    # formatting decision instead of useful evidence.
    always = [
        "raw", "raw_delta",
        "fl", "fl_delta",
        "pub", "pub_delta",
    ]
    optional = ["counter_delta"]

    out: Dict[str, List[str]] = {}
    for lane in clocks:
        cols = list(always)
        for col in optional:
            if any(row["lanes"][lane].get(col) is not None for row in collected):
                cols.append(col)
        out[lane] = cols
    return out


def _col_label(lane: str, col: str) -> str:
    p = PREFIXES[lane]
    return {
        "raw": f"{p}_raw",
        "raw_delta": f"{p}_rawΔ",
        "fl": f"{p}_fl",
        "fl_delta": f"{p}_flΔ",
        "pub": f"{p}_pub",
        "pub_delta": f"{p}_pubΔ",
        "counter_delta": f"{p}_cΔ",
    }[col]


def _col_width(col: str) -> int:
    return {
        "raw": 13,
        "raw_delta": 8,
        "fl": 13,
        "fl_delta": 8,
        "pub": 13,
        "pub_delta": 8,
        "counter_delta": 11,
    }[col]


def _col_signed(col: str) -> bool:
    return col in ("raw_delta", "fl_delta", "pub_delta")


def _headers_for(clocks: Tuple[str, ...],
                 cols: Dict[str, List[str]],
                 include_dwt_ppb: bool) -> Tuple[str, str]:
    parts = [
        f"{'pps':>6s}",
        f"{'p_act':>13s}",
        f"{'p_Δ':>8s}",
        f"{'p_voff':>8s}",
    ]
    for lane in clocks:
        for col in cols[lane]:
            label = _col_label(lane, col)
            parts.append(f"{label:>{_col_width(col)}s}")
    if include_dwt_ppb:
        parts.append(f"{'dwt_ppb':>10s}")
    return "  ".join(parts), "  ".join("─" * len(p) for p in parts)


def _row_line(row: Dict[str, Any],
              clocks: Tuple[str, ...],
              cols: Dict[str, List[str]],
              include_dwt_ppb: bool) -> str:
    parts = [
        f"{row['pps_count']:>6d}",
        _fmt_int(row["pps_actual"], 13),
        _fmt_int(row["pps_delta"], 8, signed=True),
        _fmt_int(row.get("pps_vclock_phase_cycles"), 8, signed=True),
    ]
    for lane in clocks:
        data = row["lanes"][lane]
        for col in cols[lane]:
            parts.append(
                _fmt_int(data[col], _col_width(col), signed=_col_signed(col))
            )
    if include_dwt_ppb:
        parts.append(_fmt_float(row["dwt_ppb"], 10, 3, signed=True))
    return "  ".join(parts)


def _has_any(collected: List[Dict[str, Any]], lane: str, key: str) -> bool:
    return any(row["lanes"][lane].get(key) is not None for row in collected)


def _verdict_mask_names(mask: Optional[int]) -> List[str]:
    if mask is None:
        return []
    return [name for bit, name in DWT_PUBLICATION_VERDICT_BITS if (mask & bit) != 0]


def _abs_int(value: Optional[int]) -> Optional[int]:
    if value is None:
        return None
    return abs(value)


def _court_watchdog_count(court: Dict[str, Any]) -> int:
    return _first_int(court.get("watchdog_count"), 0) or 0


def _court_reason(court: Dict[str, Any]) -> str:
    reason = court.get("reason")
    return str(reason).strip().lower() if reason is not None else ""


def _is_side_rail_court_notice(court: Dict[str, Any]) -> bool:
    """True when a nonzero court mask is diagnostic chatter, not conviction.

    process_interrupt can preserve side-rail diagnostics such as GNSS projection
    or yardstick disagreement while still publishing a clean FloorLine endpoint.
    Those rows should remain searchable in court collateral, but they should not
    make --pathology-only explode into every row.
    """
    mask = court.get("mask")
    return bool(mask) and _court_reason(court) == "side_rail_diagnostic" and _court_watchdog_count(court) == 0


def classify_pathologies(row: Dict[str, Any],
                          clocks: Tuple[str, ...],
                          gate_cycles: int) -> List[Dict[str, Any]]:
    """Return row-level custody/pathology issues.

    This is intentionally conservative.  It does not flag normal lattice-scale
    movement.  It flags impossible/implausible authority intervals, bad
    counter lineage, fatal/watchdog-bearing DWT publication court verdicts,
    and divergence from the current publication authority doctrine.

    FloorLine is no longer publication authority.  A nonzero published-minus-
    FloorLine delta is now expected diagnostic evidence, not a row pathology.
    Nonzero side_rail_diagnostic masks with no watchdog count are treated as
    court notices, not row pathologies.
    """
    issues: List[Dict[str, Any]] = []

    pps_actual = row.get("pps_actual")
    if pps_actual is not None and pps_actual <= 0:
        issues.append({
            "lane": "PPS",
            "kind": "zero_or_negative_interval",
            "message": f"PPS actual interval is {pps_actual:,d} cycles",
        })

    for lane in clocks:
        data = row["lanes"][lane]
        court = data.get("court") if isinstance(data.get("court"), dict) else {}
        expected = _first_int(court.get("expected_interval_cycles"), pps_actual)

        mask = court.get("mask")
        if mask and not _is_side_rail_court_notice(court):
            names = ", ".join(_verdict_mask_names(mask)) or f"0x{mask:X}"
            reason = court.get("reason") or "---"
            wd = _court_watchdog_count(court)
            issues.append({
                "lane": lane,
                "kind": "court_verdict",
                "message": f"court verdict mask=0x{mask:X} ({names}), reason={reason}, wd={wd:,d}",
            })

        for rail in ("raw", "pub"):
            value = data.get(rail)
            if value is None:
                continue
            if value <= 0:
                issues.append({
                    "lane": lane,
                    "rail": rail,
                    "kind": "zero_or_negative_interval",
                    "message": f"{rail} interval is {value:,d} cycles",
                })
                continue
            if expected is not None:
                error = value - expected
                if abs(error) > gate_cycles:
                    issues.append({
                        "lane": lane,
                        "rail": rail,
                        "kind": "interval_excursion",
                        "message": (
                            f"{rail} interval {value:,d} differs from "
                            f"expected {expected:,d} by {error:+,d} cycles"
                        ),
                    })

        pub_edge_minus_expected = data.get("pub_edge_minus_expected")
        if pub_edge_minus_expected not in (None, 0):
            authority = data.get("pub_expected_authority") or "current_authority"
            issues.append({
                "lane": lane,
                "kind": "published_authority_edge_divergence",
                "message": (
                    f"published edge - {authority} expected edge = "
                    f"{pub_edge_minus_expected:+,d} cycles"
                ),
            })

        # For OCXO lanes, the current publication authority is the latency-
        # adjusted observed edge.  The endpoint check above is primary; this
        # interval check catches parser/schema inconsistencies where endpoints
        # agree but the selected interval rail does not.  VCLOCK intentionally
        # publishes PPS+phase, so its interval is not compared to raw VCLOCK.
        if lane != "VCLOCK":
            pub_minus_raw = data.get("pub_minus_raw")
            if pub_minus_raw not in (None, 0):
                issues.append({
                    "lane": lane,
                    "kind": "published_observed_interval_divergence",
                    "message": f"published interval - observed interval = {pub_minus_raw:+,d} cycles",
                })

        counter_delta = data.get("counter_delta")
        if counter_delta is not None and counter_delta != PATHOLOGY_EXPECTED_COUNTER_DELTA_TICKS:
            issues.append({
                "lane": lane,
                "kind": "counter_delta",
                "message": (
                    f"counter32 delta {counter_delta:,d} ticks, expected "
                    f"{PATHOLOGY_EXPECTED_COUNTER_DELTA_TICKS:,d}"
                ),
            })

        court_checks = [
            ("published_interval_error_cycles", "court published interval error"),
            ("observed_interval_error_cycles", "court observed interval error"),
        ]
        if lane != "VCLOCK":
            # In the observed-authority OCXO doctrine, published-minus-observed
            # should remain zero.  For VCLOCK, published is PPS+phase by design
            # and may be far from the raw VCLOCK witness endpoint, so this
            # collateral is diagnostic-only.
            court_checks.append(("published_minus_observed_cycles", "court published - observed"))

        for key, label in court_checks:
            value = court.get(key)
            if value is not None and abs(value) > gate_cycles:
                issues.append({
                    "lane": lane,
                    "kind": "court_interval_collateral",
                    "message": f"{label} = {value:+,d} cycles",
                })

    return issues


def _court_summary(court: Dict[str, Any]) -> str:
    if not court:
        return "court transcript unavailable"
    mask = court.get("mask")
    names = ",".join(_verdict_mask_names(mask)) if mask else "OK"
    valid = court.get("valid")
    reason = court.get("reason") or ("ok" if not mask else "")
    parts = [
        f"valid={_fmt_str(str(valid) if valid is not None else None)}",
        f"mask={('0x%X' % mask) if mask is not None else '---'}",
        f"bits={names}",
        f"reason={reason or '---'}",
        f"wd={_fmt_int(court.get('watchdog_count'))}",
    ]
    return " ".join(parts)


def _format_optional_delta(value: Optional[int]) -> str:
    return "---" if value is None else f"{value:+,d}"


def _format_optional_float(value: Optional[float], decimals: int = 6) -> str:
    return "---" if value is None else f"{value:+,.{decimals}f}"


def _endpoint_agreement(selected: Optional[int], reference: Optional[int]) -> Optional[int]:
    if selected is None or reference is None:
        return None
    return _signed_delta_u32(selected, reference)


def _interesting_endpoint_notes(label: str, value: Optional[int]) -> List[str]:
    if value is None:
        return []
    notes: List[str] = []
    if value in (0, 1, 0xFFFFFFFF):
        notes.append(f"{label}={_fmt_int(value)} looks sentinel/poisoned")
    if value != 0 and value % 1_000_000_000 == 0:
        notes.append(f"{label}={_fmt_int(value)} is a round 1e9-domain value, suspicious for DWT")
    return notes


def _append_endpoint_provenance_lines(lines: List[str], lane: str, data: Dict[str, Any]) -> None:
    micro = data.get("micro") if isinstance(data.get("micro"), dict) else {}
    science = data.get("science") if isinstance(data.get("science"), dict) else {}

    raw_dwt = data.get("raw_dwt")
    fl_dwt = data.get("fl_dwt")
    pub_dwt = data.get("pub_used_dwt")

    lines.append(
        f"       {lane} selected endpoints: "
        f"raw={_fmt_int(raw_dwt)} "
        f"fl={_fmt_int(fl_dwt)} "
        f"pub={_fmt_int(pub_dwt)}"
    )
    lines.append(
        f"       {lane} micro endpoints: "
        f"raw={_fmt_int(micro.get('raw_dwt'))} "
        f"orig={_fmt_int(micro.get('orig_dwt'))} "
        f"fl={_fmt_int(micro.get('fl_dwt'))} "
        f"pub={_fmt_int(micro.get('pub_dwt'))} "
        f"used={_fmt_int(micro.get('used_dwt'))} "
        f"ema={_fmt_int(micro.get('ema_dwt'))}"
    )
    lines.append(
        f"       {lane} micro intervals: "
        f"obs={_fmt_int(micro.get('observed_interval_cycles'))} "
        f"eff={_fmt_int(micro.get('effective_interval_cycles'))} "
        f"fl_cyc={_fmt_int(micro.get('floorline_interval_cycles'))} "
        f"res={_format_optional_delta(micro.get('residual_cycles'))} "
        f"pps_res={_format_optional_delta(micro.get('pps_residual_cycles'))} "
        f"fl_err={_format_optional_delta(micro.get('floorline_error_cycles'))} "
        f"fl_ierr={_format_optional_delta(micro.get('floorline_interval_error_cycles'))}"
    )
    lines.append(
        f"       {lane} micro FloorLine: "
        f"src={_fmt_int(micro.get('floorline_source'))} "
        f"reason={_fmt_int(micro.get('floorline_reason'))} "
        f"acc={_fmt_int(micro.get('floorline_accept_count'))} "
        f"bkt={_fmt_int(micro.get('floorline_bucket_count'))} "
        f"rej={_fmt_int(micro.get('floorline_reject_count'))}"
    )
    lines.append(
        f"       {lane} fragment science endpoints: "
        f"raw={_fmt_int(science.get('raw_dwt'))} "
        f"fl={_fmt_int(science.get('floorline_dwt'))} "
        f"pub={_fmt_int(science.get('published_dwt'))} "
        f"pps_vclock={_fmt_int(science.get('pps_vclock_dwt'))} "
        f"edge={_fmt_str(science.get('edge_species'))} "
        f"source={_fmt_str(science.get('frequency_source'))}"
    )
    lines.append(
        f"       {lane} fragment science intervals: "
        f"obs={_fmt_int(science.get('observed_interval_cycles'))} "
        f"fl={_fmt_int(science.get('floorline_interval_cycles'))} "
        f"rawΔ={_fmt_int(science.get('delta_raw_clock_interval_cycles'))} "
        f"flΔ={_fmt_int(science.get('delta_floorline_clock_interval_cycles'))} "
        f"raw_ref={_fmt_int(science.get('delta_raw_reference_interval_cycles'))} "
        f"fl_ref={_fmt_int(science.get('delta_floorline_reference_interval_cycles'))} "
        f"raw_res={_format_optional_delta(science.get('delta_raw_residual_cycles'))} "
        f"fl_res={_format_optional_delta(science.get('delta_floorline_residual_cycles'))}"
    )
    lines.append(
        f"       {lane} fragment science residuals: "
        f"clock_ns={_fmt_int(science.get('clock_interval_ns'))} "
        f"gnss_ns={_fmt_int(science.get('gnss_interval_ns'))} "
        f"fast_ns={_format_optional_delta(science.get('fast_residual_ns'))} "
        f"trad_fast_ns={_format_optional_delta(science.get('traditional_fast_residual_ns'))} "
        f"reported-canon={_format_optional_delta(science.get('reported_minus_canonical_residual_ns'))} "
        f"reported-fl={_format_optional_delta(science.get('reported_minus_floorline_residual_ns'))}"
    )
    lines.append(
        f"       {lane} fragment science totals: "
        f"ns={_fmt_int(science.get('ns'))} "
        f"measured={_fmt_int(science.get('measured_gnss_ns'))} "
        f"measured-minus={_format_optional_delta(science.get('measured_minus_ns'))} "
        f"total_fast_ns={_format_optional_delta(science.get('total_fast_residual_ns'))} "
        f"total_ppb={_format_optional_float(science.get('total_ppb'))} "
        f"traditional_total_ppb={_format_optional_float(science.get('traditional_total_ppb'))}"
    )
    lines.append(
        f"       {lane} publication authority: "
        f"authority={_fmt_str(data.get('pub_expected_authority'))} "
        f"expected={_fmt_int(data.get('pub_expected_dwt'))} "
        f"pub-expected={_format_optional_delta(data.get('pub_edge_minus_expected'))} "
        f"pub-raw={_format_optional_delta(data.get('pub_edge_minus_raw_edge'))} "
        f"pub-fl={_format_optional_delta(data.get('pub_edge_minus_fl_edge'))}"
    )
    lines.append(
        f"       {lane} endpoint agreement selected-vs-micro: "
        f"raw={_format_optional_delta(_endpoint_agreement(raw_dwt, micro.get('raw_dwt')))} "
        f"fl={_format_optional_delta(_endpoint_agreement(fl_dwt, micro.get('fl_dwt')))} "
        f"pub={_format_optional_delta(_endpoint_agreement(pub_dwt, micro.get('pub_dwt')))} "
        f"used={_format_optional_delta(_endpoint_agreement(pub_dwt, micro.get('used_dwt')))}"
    )
    lines.append(
        f"       {lane} endpoint agreement selected-vs-fragment: "
        f"raw={_format_optional_delta(_endpoint_agreement(raw_dwt, science.get('raw_dwt')))} "
        f"fl={_format_optional_delta(_endpoint_agreement(fl_dwt, science.get('floorline_dwt')))} "
        f"pub={_format_optional_delta(_endpoint_agreement(pub_dwt, science.get('published_dwt')))}"
    )

    notes: List[str] = []
    for label, value in (
        ("raw", raw_dwt),
        ("fl", fl_dwt),
        ("pub", pub_dwt),
        ("micro_pub", micro.get("pub_dwt")),
        ("micro_used", micro.get("used_dwt")),
        ("science_pub", science.get("published_dwt")),
    ):
        notes.extend(_interesting_endpoint_notes(label, value))
    if notes:
        lines.append(f"       {lane} endpoint notes: " + "; ".join(notes))


def _pathology_interpretation(lane: str,
                              data: Dict[str, Any],
                              court: Dict[str, Any],
                              gate_cycles: int) -> List[str]:
    out: List[str] = []
    raw = data.get("raw")
    fl = data.get("fl")
    pub = data.get("pub")
    p_minus_fl = data.get("pub_minus_fl")
    pub_edge_minus_expected = data.get("pub_edge_minus_expected")
    court_mask = court.get("mask")
    court_pub_err = court.get("published_interval_error_cycles")
    court_obs_err = court.get("observed_interval_error_cycles")

    raw_bad = raw is not None and (
        raw <= 0 or
        (court.get("expected_interval_cycles") is not None and
         abs(raw - court.get("expected_interval_cycles")) > gate_cycles)
    )
    pub_ok_vs_authority = pub_edge_minus_expected in (None, 0)
    pub_ok_vs_court = (court_pub_err is None or abs(court_pub_err) <= gate_cycles)

    if raw_bad and pub_ok_vs_authority and pub_ok_vs_court:
        out.append(
            "interpretation: raw/evidence rail is pathological, but published matches "
            "the current authority doctrine; the bad witness does not appear to "
            "have become subscriber-facing truth."
        )
    elif pub_edge_minus_expected not in (None, 0):
        authority = data.get("pub_expected_authority") or "current authority"
        out.append(
            f"interpretation: published edge diverges from {authority}; this is "
            "a publication-boundary custody issue, not a FloorLine comparison."
        )
    elif court_mask and _is_side_rail_court_notice(court):
        out.append(
            "court notice: side_rail_diagnostic only; watchdog count is zero and the "
            "published edge should be judged against the current authority doctrine."
        )
    elif court_mask:
        out.append(
            "interpretation: process_interrupt's DWT publication court recorded a nonzero "
            "non-side-rail verdict for this row; decode the mask/reason before trusting the edge."
        )
    elif raw_bad and court_obs_err is not None and abs(court_obs_err) > gate_cycles:
        out.append(
            "interpretation: the court collateral also sees the observed/raw interval as "
            "bad, while the published surface should be judged separately."
        )
    elif raw_bad:
        out.append(
            "interpretation: this looks like a raw evidence excursion.  The court transcript "
            "does not show a published-edge failure in the compact fields available here."
        )

    if court and court.get("mask") in (None, 0) and raw_bad and pub_ok_vs_authority:
        out.append(
            "court note: mask is OK/zero, consistent with the court judging the published "
            "edge rather than the raw witness interval."
        )

    return out


def pathology_detail_lines(row: Dict[str, Any],
                           clocks: Tuple[str, ...],
                           gate_cycles: int) -> List[str]:
    issues = row.get("pathologies") or []
    if not issues:
        return []

    lines = [
        f"    └─ PATHOLOGY @ PPS {row['pps_count']:,d} "
        f"({len(issues)} issue{'s' if len(issues) != 1 else ''}, gate={gate_cycles:,d} cycles)"
    ]

    for issue in issues[:12]:
        lane = issue.get("lane", "?")
        lines.append(f"       • {lane}: {issue.get('message', '')}")
    if len(issues) > 12:
        lines.append(f"       • ... {len(issues) - 12} additional issue(s) suppressed in this block")

    lanes_with_issues = []
    for issue in issues:
        lane = issue.get("lane")
        if lane in clocks and lane not in lanes_with_issues:
            lanes_with_issues.append(lane)

    for lane in lanes_with_issues:
        data = row["lanes"][lane]
        court = data.get("court") if isinstance(data.get("court"), dict) else {}
        lines.append(
            f"       {lane} intervals: "
            f"raw={_fmt_int(data.get('raw'))} "
            f"fl={_fmt_int(data.get('fl'))} "
            f"pub={_fmt_int(data.get('pub'))} "
            f"pub-raw={_format_optional_delta(data.get('pub_minus_raw'))} "
            f"pub-fl={_format_optional_delta(data.get('pub_minus_fl'))} "
            f"pub_edge-expected={_format_optional_delta(data.get('pub_edge_minus_expected'))} "
            f"pub_edge-fl_edge={_format_optional_delta(data.get('pub_edge_minus_fl_edge'))}"
        )
        _append_endpoint_provenance_lines(lines, lane, data)
        lines.append(
            f"       {lane} court: {_court_summary(court)}"
        )
        lines.append(
            f"       {lane} court intervals: "
            f"exp={_fmt_int(court.get('expected_interval_cycles'))} "
            f"obs={_fmt_int(court.get('observed_interval_cycles'))} "
            f"fl={_fmt_int(court.get('floorline_interval_cycles'))} "
            f"pub={_fmt_int(court.get('published_interval_cycles'))}"
        )
        lines.append(
            f"       {lane} court errors: "
            f"obs_err={_format_optional_delta(court.get('observed_interval_error_cycles'))} "
            f"fl_err={_format_optional_delta(court.get('floorline_interval_error_cycles'))} "
            f"pub_err={_format_optional_delta(court.get('published_interval_error_cycles'))} "
            f"pub-obs={_format_optional_delta(court.get('published_minus_observed_cycles'))} "
            f"fl-obs={_format_optional_delta(court.get('floorline_minus_observed_cycles'))}"
        )
        lines.append(
            f"       {lane} court custody: "
            f"exp_cnt={_fmt_int(court.get('expected_counter_delta_ticks'))} "
            f"obs_cnt={_fmt_int(court.get('observed_counter_delta_ticks'))} "
            f"svc_off={_format_optional_delta(court.get('service_offset_signed_ticks'))} ticks "
            f"gnss_err={_format_optional_delta(court.get('vclock_gnss_error_ns'))} ns"
        )
        for statement in _pathology_interpretation(lane, data, court, gate_cycles):
            lines.append(f"       {statement}")

    return lines


def analyze(campaign: str,
            limit: int = 0,
            clock_filter: Optional[str] = None,
            slip_view: bool = False,
            align_ocxo: bool = False,
            pathology_only: bool = False,
            pathology_gate: int = PATHOLOGY_DEFAULT_GATE_CYCLES) -> None:
    if slip_view:
        raise SystemExit("--slip was retired from this compact FloorLine report; use an older raw_cycles if SlipLedger courtroom output is needed")

    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows for campaign '{campaign}'")
        return

    clock_filter = normalize_clock_filter(clock_filter) if clock_filter else None
    clocks = selected_clocks(clock_filter)

    collected, gaps = collect_rows(rows)
    alignment_skipped = 0
    if align_ocxo:
        collected, alignment_skipped = align_pps_vclock_to_ocxo_rows(collected)
    if limit:
        collected = collected[:limit]

    if pathology_gate <= 0:
        raise SystemExit("--pathology-gate must be a positive integer")

    for row in collected:
        row["pathologies"] = classify_pathologies(row, clocks, pathology_gate)

    displayed_rows = [
        row for row in collected
        if (not pathology_only or row.get("pathologies"))
    ]

    cols = available_columns(collected, clocks)
    include_dwt_ppb = any(row.get("dwt_ppb") is not None for row in collected)

    clock_label = "ALL" if clock_filter is None else clock_filter
    view_bits = [clock_label]
    if align_ocxo:
        view_bits.append("ALIGN_OCXO")
    if pathology_only:
        view_bits.append("PATHOLOGY_ONLY")
    view_label = ", ".join(view_bits)
    print(f"Campaign: {campaign}  ({len(rows):,} rows, view={view_label})")
    print()
    print("Raw / FloorLine / published cycle audit")
    print("══════════════════════════════════════")
    print("  PPS is always the base rail.")
    print("  *_raw = evidence interval, *_fl = FloorLine interval,")
    print("  *_pub = subscriber-facing published interval.")
    print("  *Δ columns are per-rail cycle-count residuals: interval[n] - interval[n-1].")
    print("  p_voff is the PPS→VCLOCK DWT-cycle phase offset.")
    print("  Endpoint-derived phase is preferred when the scalar is missing or stale.")
    if any(row.get("start_pps0_valid") for row in collected):
        print("  PPS 1 deltas use the private START PPS0 prologue interval when present.")
    print("  Core rail columns are always shown; missing FL data is shown as --- on purpose.")
    print(f"  Pathology gate: {pathology_gate:,d} cycles; pathological rows get a court-transcript block.")
    print("  Nonzero side_rail_diagnostic court masks with wd=0 are treated as court notices, not pathologies.")
    if pathology_only:
        print("  Pathology-only mode: normal rows are suppressed, summary still covers analyzed rows.")
    if align_ocxo:
        print("  Alignment view: PPS and VCLOCK are delayed by one TIMEBASE row.")
        print("  OCXO lanes remain on their original publication rows.")
    print()

    header, sep = _headers_for(clocks, cols, include_dwt_ppb)
    print(header)
    print(sep)
    for row in displayed_rows:
        print(_row_line(row, clocks, cols, include_dwt_ppb))
        for detail_line in pathology_detail_lines(row, clocks, pathology_gate):
            print(detail_line)

    print()
    print(f"Rows shown: {len(displayed_rows):,}")
    if pathology_only:
        print(f"Rows analyzed: {len(collected):,}")
    print(f"Gaps:       {gaps:,}")
    if align_ocxo:
        print(f"Align skip: {alignment_skipped:,}")
    print()

    stats: Dict[str, Welford] = {
        "pps_actual": Welford(),
        "pps_delta": Welford(),
        "pps_vclock_phase": Welford(),
    }
    for lane in clocks:
        for key in ("raw", "raw_delta",
                    "fl", "fl_delta", "pub", "pub_delta",
                    "counter_delta", "fl_edge_minus_observed",
                    "pub_minus_raw", "pub_edge_minus_raw_edge",
                    "pub_edge_minus_expected",
                    "pub_minus_fl", "pub_edge_minus_fl_edge",
                    "fl_fit_minus_endpoint_interval"):
            stats[f"{lane}.{key}"] = Welford()

    coverage: Dict[str, Dict[str, int]] = {
        lane: {key: 0 for key in ("raw", "fl", "pub", "counter_delta")}
        for lane in clocks
    }

    for row in collected:
        add_optional(stats["pps_actual"], row["pps_actual"])
        add_optional(stats["pps_delta"], row["pps_delta"])
        add_optional(stats["pps_vclock_phase"], row.get("pps_vclock_phase_cycles"))
        for lane in clocks:
            data = row["lanes"][lane]
            for key in ("raw", "raw_delta",
                        "fl", "fl_delta", "pub", "pub_delta",
                        "counter_delta", "fl_edge_minus_observed",
                        "pub_minus_raw", "pub_edge_minus_raw_edge",
                        "pub_edge_minus_expected",
                        "pub_minus_fl", "pub_edge_minus_fl_edge",
                        "fl_fit_minus_endpoint_interval"):
                add_optional(stats[f"{lane}.{key}"], data.get(key))
            for key in coverage[lane]:
                if data.get(key) is not None:
                    coverage[lane][key] += 1

    print("Schema coverage")
    print("═══════════════")
    print(f"  rows = {len(collected):,}")
    for lane in clocks:
        c = coverage[lane]
        print(
            f"  {lane:<6s} raw={c['raw']:,}  "
            f"fl={c['fl']:,}  pub={c['pub']:,}  cΔ={c['counter_delta']:,}"
        )
    print()

    print("Summary")
    print("═══════")
    _print_welford("PPS actual cycles", stats["pps_actual"])
    _print_welford("PPS first-difference residual", stats["pps_delta"])
    _print_welford("PPS→VCLOCK phase offset", stats["pps_vclock_phase"])

    for lane in clocks:
        print()
        _print_welford(f"{lane} raw interval", stats[f"{lane}.raw"])
        _print_welford(f"{lane} raw cycle-count residual", stats[f"{lane}.raw_delta"])
        _print_welford(f"{lane} FloorLine interval", stats[f"{lane}.fl"])
        _print_welford(f"{lane} FloorLine cycle-count residual", stats[f"{lane}.fl_delta"])
        _print_welford(f"{lane} published interval", stats[f"{lane}.pub"])
        _print_welford(f"{lane} published cycle-count residual", stats[f"{lane}.pub_delta"])
        _print_welford(f"{lane} published interval - observed interval", stats[f"{lane}.pub_minus_raw"])
        _print_welford(f"{lane} published edge - observed edge", stats[f"{lane}.pub_edge_minus_raw_edge"])
        _print_welford(f"{lane} published edge - current authority edge", stats[f"{lane}.pub_edge_minus_expected"])
        _print_welford(f"{lane} published interval - FloorLine interval", stats[f"{lane}.pub_minus_fl"])
        _print_welford(f"{lane} published edge - FloorLine edge", stats[f"{lane}.pub_edge_minus_fl_edge"])
        _print_welford(f"{lane} reported FL fit interval - endpoint FL interval", stats[f"{lane}.fl_fit_minus_endpoint_interval"])
        _print_welford(f"{lane} FL edge - observed edge", stats[f"{lane}.fl_edge_minus_observed"])
        _print_welford(f"{lane} counter32 delta", stats[f"{lane}.counter_delta"], unit="ticks")

    print()
    print("Notes")
    print("═════")
    print("  • The old EMA/effective columns were removed from the normal view;")
    print("    this report is now focused on the publication decision: raw vs FloorLine vs pub.")
    print("  • *_pub is the ground-truth subscriber-facing interval.")
    print("  • *_fl is now endpoint-derived first: fl_dwt[n] - fl_dwt[n-1].")
    print("    The fitted/slope FloorLine interval is used only as a fallback and")
    print("    audited as 'reported FL fit interval - endpoint FL interval'.")
    print("  • FloorLine is now diagnostic, not publication authority.")
    print("    Current authority: VCLOCK = physical PPS DWT + p_voff;")
    print("    OCXO = observed latency-adjusted DWT.  Published-vs-FloorLine")
    print("    divergence is expected and is not a pathology by itself.")
    print("  • p_voff is the DWT-cycle offset from the physical PPS edge to the")
    print("    PPS/VCLOCK edge; positive means VCLOCK is later than PPS.")
    print("  • Endpoint-derived p_voff is preferred over a stale/zero scalar when")
    print("    TIMEBASE exposes the PPS/VCLOCK endpoint directly.")
    print("  • *Δ columns are cycle-count residuals computed separately per rail:")
    print("    interval[n] - interval[n-1].")
    print("    On PPS 1, a private START PPS0 prologue interval is used as interval[n-1] when present.")
    print("  • FloorLine columns are intentionally always visible.  If *_fl stays ---,")
    print("    the TIMEBASE row stream did not expose a FloorLine endpoint that this parser found.")
    print("  • Use --clock VCLOCK, --clock OCXO1, or --clock OCXO2 to keep the row narrow.")
    print("  • Pathology blocks decode the DWT publication court transcript when")
    print("    TIMEBASE_FORENSICS exposes <lane>_court_* fields.")
    print("  • Pathology blocks also print MICRO_RAW_CYCLES and fragment science")
    print("    endpoint provenance, so parser fallbacks can be separated from")
    print("    firmware-published DWT custody faults.")
    print("  • side_rail_diagnostic court masks with wd=0 are court notices only;")
    print("    they do not make an otherwise clean row pathological.")
    print("  • Pathology classification follows the current publication doctrine;")
    print("    FloorLine deltas remain in the report as collateral only.")
    print("  • Use --pathology-only to suppress normal rows while retaining the same")
    print("    row format and per-pathology court collateral.")
    print("  • Use --pathology-gate N to change the interval-excursion threshold.")
    if align_ocxo:
        print("  • --align-ocxo / --delay-pps-vclock is an intentional view transform:")
        print("    PPS and VCLOCK are shown from the previous TIMEBASE row so late OCXO")
        print("    intervals appear beside the reference second they correspond to.")
        print("    Do not read the aligned view as original TIMEBASE publication order.")
    print()


def parse_args(argv: List[str]) -> Tuple[str, int, Optional[str], bool, bool, bool, int]:
    if len(argv) < 2:
        print("Usage: raw_cycles <campaign_name> [limit] [clock]")
        print("       raw_cycles <campaign_name> --clock OCXO2 [limit]")
        print("       raw_cycles <campaign_name> --limit 300 --clock VCLOCK")
        print("       raw_cycles <campaign_name> --align-ocxo")
        print("       raw_cycles <campaign_name> --delay-pps-vclock")
        print("       raw_cycles <campaign_name> --pathology-only")
        print("       raw_cycles <campaign_name> --pathology-gate 500")
        raise SystemExit(1)

    campaign = argv[1]
    limit = 0
    clock: Optional[str] = None
    slip_view = False
    align_ocxo = False
    pathology_only = False
    pathology_gate = PATHOLOGY_DEFAULT_GATE_CYCLES

    i = 2
    while i < len(argv):
        arg = argv[i]
        if arg in ("--align-ocxo", "--align-ocxos", "--delay-pps-vclock"):
            align_ocxo = True
            i += 1
            continue
        if arg == "--slip":
            slip_view = True
            i += 1
            continue
        if arg == "--pathology-only":
            pathology_only = True
            i += 1
            continue
        if arg == "--pathology-gate":
            if i + 1 >= len(argv):
                raise SystemExit("--pathology-gate requires an integer")
            try:
                pathology_gate = int(argv[i + 1])
            except ValueError as exc:
                raise SystemExit("--pathology-gate requires an integer") from exc
            i += 2
            continue
        if arg.startswith("--pathology-gate="):
            try:
                pathology_gate = int(arg.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit("--pathology-gate requires an integer") from exc
            i += 1
            continue
        if arg == "--clock":
            if i + 1 >= len(argv):
                raise SystemExit("--clock requires VCLOCK, OCXO1, or OCXO2")
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
        if arg.upper() in CLOCKS or arg.upper() in ("V", "VC", "O1", "O2"):
            clock = arg
            i += 1
            continue
        try:
            limit = int(arg)
        except ValueError as exc:
            raise SystemExit(f"unknown argument '{arg}'") from exc
        i += 1

    return (
        campaign,
        limit,
        normalize_clock_filter(clock) if clock else None,
        slip_view,
        align_ocxo,
        pathology_only,
        pathology_gate,
    )


def main() -> None:
    (
        campaign,
        limit,
        clock,
        slip_view,
        align_ocxo,
        pathology_only,
        pathology_gate,
    ) = parse_args(sys.argv)
    analyze(campaign, limit, clock, slip_view, align_ocxo, pathology_only, pathology_gate)


if __name__ == "__main__":
    main()
