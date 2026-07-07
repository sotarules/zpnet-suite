"""
ZPNet Campaign Analyzer — v16 (TIMEBASE_V3 / TIMEBASE_FRAGMENT_V4)

Rewritten for the current paired TIMEBASE architecture:

  payload.fragment   — compact Teensy-authored science spine
  payload.forensics  — paired diagnostic companion for the same PPS identity

The analyzer accepts both the newer nested TIMEBASE_FRAGMENT_V4 shape and the
older flat fragment aliases where practical, but all primary checks now read the
current fields:

  fragment.gnss.ns
  fragment.vclock.ns
  fragment.ocxo1.ns / fragment.ocxo2.ns
  fragment.ocxoN.pps_residual.*
  fragment.ocxoN.science.total_tau / .total_ppb
  fragment.dwt.cycle_count_total
  fragment.stats.<lane>.welford.* / .tau / .ppb

Recovery doctrine:
  PPS-count gaps are canonical when they represent recovery.  The analyzer does
  not treat missing rows as corruption, but it does certify that the first row
  after each gap is consistent with the prior public campaign ledgers projected
  to the new GNSS/PPS identity.

  Current clean-recovery doctrine:
    • transitional RECOVER rows may be discarded Pi-side before persistence
    • persisted post-recovery rows must have clean OCXO science
    • Welford cardinality is segment-aware: START private PPS0 lets DWT/OCXO
      enter public PPS1 with n=1, while each RECOVER boundary consumes one
      non-sample bookend for DWT/OCXO before science resumes
    • stats.<lane>.tau/.ppb must match the continuity-aligned campaign
      clockface ledger ratio and the compact science total_* fields
    • OCXO recovery must preserve the panel-facing total TAU/PPB experience:
      a recovery boundary may skip rows, but it must not introduce a visible
      OCXO clockface-ratio step

Usage:
    python -m zpnet.tests.campaign_analyzer <campaign_name>
    .zt campaign_analyzer Gate4
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Set, Tuple

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_PPS = 1_008_000_000

OCXO_SECOND_ALARM_NS = 10_000
OCXO_SECOND_WARN_NS  = 500
PHASE_STEP_ALARM_NS = 100_000
PPB_STEP_ALARM = 100.0
PPB_ABSOLUTE_THRESHOLD = 10_000
PPB_MISMATCH_ALARM = 0.01
TAU_MISMATCH_ALARM = 5.0e-12
TAU_ABSOLUTE_DEVIATION_THRESHOLD = PPB_ABSOLUTE_THRESHOLD / 1.0e9
WELFORD_STDDEV_ALARM = 500.0
WELFORD_N_TOLERANCE = 0

# Recovery projection tolerances.  The broad ns tolerance catches gross ledger
# corruption; the ppb-continuity tolerance catches the subtler user-visible
# problem where OCXO evidence reattaches cleanly but the panel-facing total
# TAU/PPB appears to restart from a fresh intercept.
RECOVERY_OCXO_PROJECTION_ALARM_NS = 5_000
RECOVERY_OCXO_CONTINUITY_PPB_ALARM = 0.25
RECOVERY_DWT_PROJECTION_ALARM_CYCLES = 100_000


class WelfordStats:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float):
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        if x < self.min_val:
            self.min_val = x
        if x > self.max_val:
            self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    def advance_missing_to_n(self, target_n: int) -> bool:
        """Advance cardinality without inventing new samples.

        This mirrors the firmware recovery-gap doctrine: a canonical TIMEBASE
        gap advances N so the published population cardinality continues to
        match campaign public identity, but the mean/stddev distribution is
        preserved until a real post-gap sample is accepted.
        """
        target = int(target_n)
        if target <= self.n:
            return False
        sd = self.stddev
        if self.n == 0:
            self.mean = 0.0
            self.min_val = 0.0
            self.max_val = 0.0
        self.n = target
        self.m2 = (sd * sd * float(target - 1)) if target >= 2 else 0.0
        return True

    def summary(self, fmt: str = ".3f") -> str:
        if self.n == 0:
            return "no data"
        sd = f"{self.stddev:{fmt}}" if self.n >= 2 else "---"
        return (f"n={self.n}  mean={self.mean:+{fmt}}  sd={sd}  "
                f"min={self.min_val:{fmt}}  max={self.max_val:{fmt}}")


# -----------------------------------------------------------------------------
# Schema helpers
# -----------------------------------------------------------------------------


def _as_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _fragment(row: Dict[str, Any]) -> Dict[str, Any]:
    return _as_dict(row.get("fragment"))


def _forensics(row: Dict[str, Any]) -> Dict[str, Any]:
    return _as_dict(row.get("forensics"))


def _extra(row: Dict[str, Any]) -> Dict[str, Any]:
    return _as_dict(row.get("extra_clocks"))


def _path_get(obj: Dict[str, Any], path: str, default: Any = None) -> Any:
    cur: Any = obj
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur.get(part)
    return default if cur is None else cur


def _first_value(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _to_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _to_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def _to_bool(v: Any) -> Optional[bool]:
    if isinstance(v, bool):
        return v
    if v is None:
        return None
    s = str(v).strip().lower()
    if s in ("true", "1", "yes", "on"):
        return True
    if s in ("false", "0", "no", "off"):
        return False
    return None


def _first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = _to_int(value)
        if parsed is not None:
            return parsed
    return None


def _first_float(*values: Any) -> Optional[float]:
    for value in values:
        parsed = _to_float(value)
        if parsed is not None:
            return parsed
    return None


def _row_pps_count(row: Dict[str, Any]) -> int:
    frag = _fragment(row)
    value = _first_int(
        row.get("teensy_pps_vclock_count"),
        row.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
    )
    if value is None:
        raise ValueError(f"TIMEBASE row missing PPS/VCLOCK identity: db_id={row.get('_db_id')}")
    return value


def _gnss_ns(row: Dict[str, Any]) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, "gnss.ns"),
        frag.get("gnss_ns"),
        row.get("teensy_gnss_ns"),
        row.get("gnss_ns"),
    )


def _vclock_ns(row: Dict[str, Any]) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, "vclock.ns"),
        _path_get(frag, "gnss.ns"),
        frag.get("vclock_ns"),
        row.get("vclock_ns"),
    )


def _ocxo_ns(row: Dict[str, Any], prefix: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, f"{prefix}.ns"),
        _path_get(frag, f"gnss.{prefix}_ns"),
        frag.get(f"{prefix}_ns"),
        frag.get(f"{prefix}_ns_at_pps_vclock"),
        row.get(f"teensy_{prefix}_ns"),
        row.get(f"{prefix}_ns"),
    )


def _dwt_cycles(row: Dict[str, Any]) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, "dwt.cycle_count_total"),
        _path_get(frag, "dwt.cycles"),
        _path_get(frag, "dwt.value"),
        frag.get("dwt_cycle_count_total"),
        row.get("teensy_dwt_cycles"),
        row.get("dwt_cycle_count_total"),
        row.get("dwt_cycles"),
        # Legacy compatibility only.  Current TIMEBASE uses cycles, not dwt.ns.
        _path_get(frag, "dwt.ns"),
        frag.get("dwt_ns"),
        row.get("teensy_dwt_ns"),
    )


def _dwt_interval_cycles(row: Dict[str, Any]) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, "dwt.cycles_between_pps_vclock"),
        frag.get("dwt_cycles_between_pps_vclock"),
        row.get("dwt_cycles_between_pps_vclock"),
    )


def _dwt_residual_cycles(row: Dict[str, Any]) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, "dwt.second_residual_cycles"),
        frag.get("dwt_second_residual_cycles"),
        row.get("dwt_second_residual_cycles"),
    )


def _stat(row: Dict[str, Any], prefix: str, field: str) -> Optional[float]:
    frag = _fragment(row)
    return _first_float(
        _path_get(frag, f"stats.{prefix}.{field}"),
        frag.get(f"{prefix}_{field}"),
        row.get(f"{prefix}_{field}"),
    )


def _welford(row: Dict[str, Any], prefix: str, field: str) -> Optional[float]:
    frag = _fragment(row)
    return _first_float(
        _path_get(frag, f"stats.{prefix}.welford.{field}"),
        _path_get(frag, f"stats.{prefix}.{field}"),
        frag.get(f"{prefix}_welford_{field}"),
        row.get(f"{prefix}_welford_{field}"),
    )


def _welford_int(row: Dict[str, Any], prefix: str, field: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, f"stats.{prefix}.welford.{field}"),
        _path_get(frag, f"stats.{prefix}.{field}"),
        frag.get(f"{prefix}_welford_{field}"),
        row.get(f"{prefix}_welford_{field}"),
    )


def _dac_welford(row: Dict[str, Any], prefix: str, field: str) -> Optional[float]:
    frag = _fragment(row)
    return _first_float(
        _path_get(frag, f"stats.dac.{prefix}.{field}"),
        frag.get(f"{prefix}_dac_welford_{field}"),
        row.get(f"{prefix}_dac_welford_{field}"),
    )


def _dac_welford_int(row: Dict[str, Any], prefix: str, field: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, f"stats.dac.{prefix}.{field}"),
        frag.get(f"{prefix}_dac_welford_{field}"),
        row.get(f"{prefix}_dac_welford_{field}"),
    )


def _ocxo_residual(row: Dict[str, Any], prefix: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, f"{prefix}.pps_residual.fast_residual_ns"),
        _path_get(frag, f"{prefix}.measurement.second_residual_ns"),
        frag.get(f"{prefix}_second_residual_ns"),
        row.get(f"{prefix}_second_residual_ns"),
    )


def _ocxo_clock_interval(row: Dict[str, Any], prefix: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, f"{prefix}.pps_residual.clock_interval_ns"),
        _path_get(frag, f"{prefix}.measurement.clock_interval_ns"),
        _path_get(frag, f"{prefix}.interval.clock_interval_ns"),
        frag.get(f"{prefix}_clock_interval_ns"),
    )


def _ocxo_gnss_interval(row: Dict[str, Any], prefix: str) -> Optional[int]:
    frag = _fragment(row)
    return _first_int(
        _path_get(frag, f"{prefix}.pps_residual.gnss_interval_ns"),
        _path_get(frag, f"{prefix}.measurement.gnss_ns_between_edges"),
        _path_get(frag, f"{prefix}.interval.gnss_ns_between_edges"),
        frag.get(f"{prefix}_gnss_ns_between_edges"),
    )


def _ocxo_valid(row: Dict[str, Any], prefix: str) -> bool:
    frag = _fragment(row)
    return bool(_to_bool(_path_get(frag, f"{prefix}.pps_residual.valid")) or
                _to_bool(_path_get(frag, f"{prefix}.pps_projected_valid")) or
                _ocxo_clock_interval(row, prefix))


def _ocxo_phase_offset(row: Dict[str, Any], prefix: str) -> Optional[int]:
    frag = _fragment(row)
    explicit = _first_int(
        _path_get(frag, f"{prefix}.phase_offset_ns"),
        frag.get(f"{prefix}_phase_offset_ns"),
        row.get(f"{prefix}_phase_offset_ns"),
    )
    if explicit is not None:
        return explicit
    v = _vclock_ns(row)
    o = _ocxo_ns(row, prefix)
    return (v - o) if v is not None and o is not None else None


def _servo_mode(row: Dict[str, Any]) -> str:
    frag = _fragment(row)
    return str(_first_value(
        _path_get(frag, "dac.servo_mode"),
        frag.get("servo_mode"),
        row.get("servo_mode"),
        row.get("calibrate_ocxo"),
        "?",
    ))


def _forensic_int(row: Dict[str, Any], key: str) -> Optional[int]:
    return _first_int(_forensics(row).get(key), _fragment(row).get(key), row.get(key))


def _forensic_bool(row: Dict[str, Any], key: str) -> Optional[bool]:
    return _to_bool(_first_value(_forensics(row).get(key), _fragment(row).get(key), row.get(key)))


def _science(row: Dict[str, Any], prefix: str) -> Dict[str, Any]:
    return _as_dict(_path_get(_fragment(row), f"{prefix}.science"))


def _science_bool(row: Dict[str, Any], prefix: str, field: str) -> Optional[bool]:
    return _to_bool(_science(row, prefix).get(field))


def _science_float(row: Dict[str, Any], prefix: str, field: str) -> Optional[float]:
    return _first_float(_science(row, prefix).get(field))


def _science_int(row: Dict[str, Any], prefix: str, field: str) -> Optional[int]:
    return _first_int(_science(row, prefix).get(field))


def _science_str(row: Dict[str, Any], prefix: str, field: str) -> Optional[str]:
    value = _science(row, prefix).get(field)
    return None if value is None else str(value)


def _campaign_total_tau(row: Dict[str, Any], prefix: str) -> Optional[float]:
    gnss = _gnss_ns(row)
    if gnss is None or gnss <= 0:
        return None
    if prefix == "dwt":
        cycles = _dwt_cycles(row)
        if cycles is None:
            return None
        expected_cycles = (float(gnss) / NS_PER_SECOND) * DWT_EXPECTED_PER_PPS
        return None if expected_cycles == 0.0 else float(cycles) / expected_cycles
    clock_ns = _vclock_ns(row) if prefix == "vclock" else _ocxo_ns(row, prefix)
    if clock_ns is None:
        return None
    return float(clock_ns) / float(gnss)


def _campaign_total_ppb(row: Dict[str, Any], prefix: str) -> Optional[float]:
    tau = _campaign_total_tau(row, prefix)
    return None if tau is None else (tau - 1.0) * 1e9


def _published_tau(row: Dict[str, Any], prefix: str) -> Optional[float]:
    if prefix == "gnss":
        return 1.0
    return _stat(row, prefix, "tau")


def _published_ppb(row: Dict[str, Any], prefix: str) -> Optional[float]:
    if prefix == "gnss":
        return 0.0
    return _stat(row, prefix, "ppb")


def _tau_from_ppb(ppb: Optional[float]) -> Optional[float]:
    return None if ppb is None else 1.0 + float(ppb) / 1.0e9


def _row_recovery_status(row: Dict[str, Any]) -> Dict[str, Any]:
    frag = _fragment(row)
    return {
        "reason": frag.get("recover_reattach_reason"),
        "degraded_active": bool(_to_bool(frag.get("recover_degraded_active"))),
        "degraded_science_hold": bool(_to_bool(frag.get("recover_degraded_science_hold"))),
    }


def _recovery_continuity_forensics(row: Dict[str, Any]) -> Dict[str, Any]:
    """Return RECOVER presentation-continuity proof from TIMEBASE_FORENSICS.

    New firmware publishes this as flat rc_* fields to avoid Payload
    add_object_json() pressure.  The nested object fallback is retained for the
    short-lived v15 experiment.
    """
    f = _forensics(row)
    rc = _path_get(f, "recovery_continuity")
    if isinstance(rc, dict):
        return rc
    if f.get("rc_schema"):
        return {
            "schema": f.get("rc_schema"),
            "aligned": f.get("rc_aligned"),
            "public_count": f.get("rc_public_count"),
            "align_count": f.get("rc_align_count"),
            "requested_public_count": f.get("rc_requested_public_count"),
            "reason": f.get("rc_reason"),
            "ocxo1_correction_ns": f.get("rc_o1_correction_ns"),
            "ocxo2_correction_ns": f.get("rc_o2_correction_ns"),
        }
    return {}


def _recovery_continuity_lane_correction(row: Dict[str, Any], prefix: str) -> Optional[int]:
    rc = _recovery_continuity_forensics(row)
    if not rc:
        return None
    return _first_int(rc.get(f"{prefix}_correction_ns"))


def _ocxo_science_clean(row: Dict[str, Any], prefix: str) -> bool:
    science = _science(row, prefix)
    if not science:
        return False
    return bool(
        _to_bool(science.get("valid")) and
        _to_bool(science.get("antecedents_complete")) and
        _to_bool(science.get("total_valid")) and
        _to_bool(science.get("reported_pps_residual_valid")) and
        _to_bool(_path_get(_fragment(row), f"{prefix}.pps_residual.valid")) and
        _first_int(science.get("clock_interval_ns")) not in (None, 0) and
        _first_int(science.get("gnss_interval_ns")) not in (None, 0)
    )


def _recovery_count_at_or_before(pps_count: int, recovery_boundaries: Set[int]) -> int:
    """Return how many persisted recovery boundaries have occurred by this row."""
    k = int(pps_count)
    return sum(1 for boundary in recovery_boundaries if int(boundary) <= k)


def _expected_welford_n(
    pps_count: int,
    prefix: str,
    recovery_boundaries: Optional[Set[int]] = None,
) -> Optional[int]:
    """Expected published Welford cardinality for the current TIMEBASE model.

    START and RECOVER have intentionally different bookend semantics.  START
    has a private PPS0/prologue, so DWT and OCXO public PPS1 may already carry
    n=1.  RECOVER cuts custody and the first persisted clean row after each
    recovery boundary is a reattachment/bookend row, not an additional DWT/OCXO
    science sample.  Therefore DWT/OCXO cardinality is public_count minus the
    number of persisted recovery boundaries at or before this row.

    VCLOCK remains an interval/self-test surface with no public PPS1 sample, so
    it stays pps_count - 1 across START/RECOVER.  DAC Welfords are row/intent
    surfaces and remain pps_count.
    """
    pps = int(pps_count)
    recoveries = _recovery_count_at_or_before(pps, recovery_boundaries or set())

    if prefix in ("dwt", "ocxo1", "ocxo2"):
        return max(0, pps - recoveries)
    if prefix == "vclock":
        return max(0, pps - 1)
    if prefix in ("ocxo1_dac", "ocxo2_dac"):
        return max(0, pps)
    if prefix == "gnss_raw":
        return None
    return None


# -----------------------------------------------------------------------------
# Data fetch
# -----------------------------------------------------------------------------


def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                       NULLIF(payload->>'teensy_pps_vclock_count', '')::bigint,
                       NULLIF(payload->>'pps_count', '')::bigint,
                       NULLIF(payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                       NULLIF(payload->'fragment'->>'pps_count', '')::bigint,
                       id::bigint
                     ) ASC,
                     id ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()
    result: List[Dict[str, Any]] = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        p["_db_id"] = row["id"]
        p["_db_ts"] = str(row["ts"])
        result.append(p)
    return result


def find_recovery_boundaries(rows: List[Dict[str, Any]]) -> Set[int]:
    boundaries: Set[int] = set()
    for i in range(1, len(rows)):
        prev = _row_pps_count(rows[i - 1])
        curr = _row_pps_count(rows[i])
        if curr > prev + 1:
            boundaries.add(curr)
    return boundaries


# -----------------------------------------------------------------------------
# Overview / continuity
# -----------------------------------------------------------------------------


def print_campaign_overview(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> int:
    total = len(rows)
    pps_counts = [_row_pps_count(r) for r in rows]
    pps_min, pps_max = min(pps_counts), max(pps_counts)
    expected_count = pps_max - pps_min + 1
    final_gnss_ns = _gnss_ns(rows[-1])
    campaign_seconds = (final_gnss_ns // NS_PER_SECOND) if final_gnss_ns is not None else pps_max
    first_ts = rows[0].get("gnss_time_utc") or rows[0].get("system_time_utc", "?")
    last_ts = rows[-1].get("gnss_time_utc") or rows[-1].get("system_time_utc", "?")
    hrs = campaign_seconds // 3600
    mins = (campaign_seconds % 3600) // 60
    secs = campaign_seconds % 60

    print("=" * 78)
    print(f"CAMPAIGN ANALYSIS: {rows[0].get('campaign', '?')}  (v16 TIMEBASE_V3/FRAGMENT_V4)")
    print("=" * 78)
    print()
    print(f"  Time range:        {first_ts}")
    print(f"                  -> {last_ts}")
    print(f"  Location:          {rows[0].get('location', '?')}")
    print(f"  pps_count range:   {pps_min} -> {pps_max}")
    print(f"  Campaign seconds:  {campaign_seconds:,}")
    print(f"  Campaign duration: {hrs:02d}:{mins:02d}:{secs:02d}")
    print()
    print(f"  Expected records:  {expected_count:,}")
    print(f"  Actual records:    {total:,}")
    print(f"  Missing records:   {expected_count - total:,}")
    if recovery_boundaries:
        print(f"  Recovery events:   {len(recovery_boundaries)}")
    print(f"  Servo mode:        {_servo_mode(rows[-1])}")
    return campaign_seconds


def analyze_pps_continuity(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    anomalies: List[str] = []
    print()
    print("-" * 78)
    print("PPS/VCLOCK COUNT CONTINUITY")
    print("-" * 78)
    jumps = recovery_jumps = regressions = repeats = 0
    for i in range(1, len(rows)):
        prev = _row_pps_count(rows[i - 1])
        curr = _row_pps_count(rows[i])
        delta = curr - prev
        if delta == 0:
            repeats += 1
            if repeats <= 5:
                print(f"  WARN REPEAT at pps_count={curr}")
        elif delta < 0:
            regressions += 1
            if regressions <= 5:
                print(f"  WARN REGRESSION: {prev} -> {curr} (delta={delta})")
        elif delta > 1:
            if curr in recovery_boundaries:
                recovery_jumps += 1
                print(f"  INFO RECOVERY GAP: {prev} -> {curr} (skipped {delta - 1})")
            else:
                jumps += 1
                if jumps <= 10:
                    print(f"  WARN GAP: {prev} -> {curr} (skipped {delta - 1})")
    print(f"\n  Unexpected gaps:   {jumps}")
    print(f"  Recovery gaps:     {recovery_jumps}")
    print(f"  Repeats:           {repeats}")
    print(f"  Regressions:       {regressions}")
    if jumps:
        anomalies.append(f"{jumps} unexpected pps_count gaps")
    if repeats:
        anomalies.append(f"{repeats} pps_count repeats")
    if regressions:
        anomalies.append(f"{regressions} pps_count regressions")
    return anomalies


def analyze_recovery_projection(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    anomalies: List[str] = []
    print()
    print("-" * 78)
    print("RECOVERY PROJECTION CERTIFICATION")
    print("-" * 78)

    if not recovery_boundaries:
        print("\n  No recovery gaps detected.")
        return anomalies

    for i in range(1, len(rows)):
        prev = rows[i - 1]
        curr = rows[i]
        prev_pps = _row_pps_count(prev)
        curr_pps = _row_pps_count(curr)
        if curr_pps <= prev_pps + 1:
            continue

        delta = curr_pps - prev_pps
        print(f"\n  Recovery boundary: {prev_pps} -> {curr_pps}  (delta={delta}, skipped={delta - 1})")

        prev_gnss = _gnss_ns(prev)
        curr_gnss = _gnss_ns(curr)
        if prev_gnss is None or curr_gnss is None:
            msg = f"recovery {curr_pps}: missing GNSS ledger"
            anomalies.append(msg)
            print(f"    GNSS: missing ledger — ALARM")
            continue

        expected_gnss = prev_gnss + delta * NS_PER_SECOND
        gnss_error = curr_gnss - expected_gnss
        identity_error = curr_gnss - curr_pps * NS_PER_SECOND
        print(f"    GNSS: expected={expected_gnss:,}  actual={curr_gnss:,}  "
              f"error={gnss_error:+,d}  identity_error={identity_error:+,d}")
        if gnss_error != 0 or identity_error != 0:
            anomalies.append(f"recovery {curr_pps}: GNSS projection error {gnss_error:+d}, identity {identity_error:+d}")

        for label, getter, tol, unit in [
            ("DWT", _dwt_cycles, RECOVERY_DWT_PROJECTION_ALARM_CYCLES, "cycles"),
            ("OCXO1", lambda r: _ocxo_ns(r, "ocxo1"), RECOVERY_OCXO_PROJECTION_ALARM_NS, "ns"),
            ("OCXO2", lambda r: _ocxo_ns(r, "ocxo2"), RECOVERY_OCXO_PROJECTION_ALARM_NS, "ns"),
        ]:
            prev_v = getter(prev)
            curr_v = getter(curr)
            if prev_v is None or curr_v is None or prev_gnss <= 0:
                print(f"    {label}: missing ledger")
                continue
            projected = (curr_gnss * prev_v) // prev_gnss
            error = curr_v - projected
            if label.startswith("OCXO") and curr_gnss > 0:
                implied_ppb_step = (float(error) / float(curr_gnss)) * 1.0e9
                print(f"    {label}: projected={projected:,}  actual={curr_v:,}  "
                      f"error={error:+,d} {unit}  implied_ppb_step={implied_ppb_step:+.3f}")
                if abs(implied_ppb_step) > RECOVERY_OCXO_CONTINUITY_PPB_ALARM:
                    anomalies.append(
                        f"recovery {curr_pps}: {label} visible TAU/PPB continuity step "
                        f"{implied_ppb_step:+.3f} ppb from {error:+d} ns projection error"
                    )
            else:
                print(f"    {label}: projected={projected:,}  actual={curr_v:,}  error={error:+,d} {unit}")
            if abs(error) > tol:
                anomalies.append(f"recovery {curr_pps}: {label} projection error {error:+d} {unit}")

    if not anomalies:
        print("\n  Verdict: OK — recovery gaps project cleanly from prior TIMEBASE ledgers")
    return anomalies


def analyze_recovery_cleanliness(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    """Audit the first persisted row after each recovery gap.

    core.py now discards transitional TIMEBASE pairs until OCXO science is clean.
    This check proves that the rows that *did* make it into PostgreSQL have the
    corresponding firmware evidence: no degraded hold, clean OCXO science, and
    restored Welford cardinality.
    """
    anomalies: List[str] = []
    print()
    print("-" * 78)
    print("RECOVERY CLEAN-ROW / CARRY-OVER AUDIT")
    print("-" * 78)

    if not recovery_boundaries:
        print("\n  No recovery gaps detected.")
        return anomalies

    for i in range(1, len(rows)):
        prev = rows[i - 1]
        curr = rows[i]
        prev_pps = _row_pps_count(prev)
        curr_pps = _row_pps_count(curr)
        if curr_pps <= prev_pps + 1:
            continue

        status = _row_recovery_status(curr)
        print(f"\n  Recovery public row: {prev_pps} -> {curr_pps}")
        print(f"    recover_reattach_reason:       {status.get('reason')}")
        print(f"    recover_degraded_active:       {status.get('degraded_active')}")
        print(f"    recover_degraded_science_hold: {status.get('degraded_science_hold')}")

        if status.get("degraded_active") or status.get("degraded_science_hold"):
            anomalies.append(f"recovery {curr_pps}: degraded recovery row persisted")

        rc = _recovery_continuity_forensics(curr)
        if rc:
            print(f"    recovery continuity proof:  present aligned={_to_bool(rc.get('aligned'))} reason={rc.get('reason')}")
        else:
            print("    recovery continuity proof:  not present in TIMEBASE_FORENSICS")

        for label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
            clean = _ocxo_science_clean(curr, prefix)
            science = _science(curr, prefix)
            ratio_source = science.get("total_ratio_source") or "PUBLIC_CLOCK_NS_OVER_GNSS_NS"
            continuity_correction = _recovery_continuity_lane_correction(curr, prefix)
            print(f"    {label} clean science:           {clean}")
            print(f"    {label} total ratio source:      {ratio_source}")
            if continuity_correction is not None:
                print(f"    {label} recovery correction_ns:  {continuity_correction:+d}")
            if ratio_source == "ALPHA_PHASELEDGER_AFFINE_TAU":
                anomalies.append(
                    f"recovery {curr_pps}: {label} panel TAU/PPB is AlphaTau, not public-ledger continuity"
                )
            if not clean:
                anomalies.append(
                    f"recovery {curr_pps}: {label} science not clean "
                    f"(valid={science.get('valid')} antecedents={science.get('antecedents_complete')} "
                    f"total_valid={science.get('total_valid')})"
                )

        for label, prefix in [
            ("DWT", "dwt"),
            ("VCLOCK", "vclock"),
            ("OCXO1", "ocxo1"),
            ("OCXO2", "ocxo2"),
            ("OCXO1_DAC", "ocxo1_dac"),
            ("OCXO2_DAC", "ocxo2_dac"),
        ]:
            if prefix.endswith("_dac"):
                actual_n = _dac_welford_int(curr, prefix.replace("_dac", ""), "n")
            else:
                actual_n = _welford_int(curr, prefix, "n")
            expected_n = _expected_welford_n(curr_pps, prefix, recovery_boundaries)
            if expected_n is None or actual_n is None:
                continue
            print(f"    {label:<9s} Welford N: actual={actual_n:,} expected={expected_n:,}")
            if abs(int(actual_n) - int(expected_n)) > WELFORD_N_TOLERANCE:
                anomalies.append(
                    f"recovery {curr_pps}: {label} Welford N {actual_n} != expected {expected_n}"
                )

        for label, prefix in [("DWT", "dwt"), ("VCLOCK", "vclock"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
            pub_ppb = _published_ppb(curr, prefix)
            pub_tau = _published_tau(curr, prefix)
            total_ppb = _campaign_total_ppb(curr, prefix)
            total_tau = _campaign_total_tau(curr, prefix)
            if pub_ppb is not None and total_ppb is not None:
                d = float(pub_ppb) - float(total_ppb)
                print(f"    {label:<6s} recovery PPB: published={pub_ppb:+.6f} total={total_ppb:+.6f} diff={d:+.6f}")
                if abs(d) > PPB_MISMATCH_ALARM:
                    anomalies.append(f"recovery {curr_pps}: {label} published PPB mismatch {d:+.6f}")
            if pub_tau is not None and total_tau is not None:
                d = float(pub_tau) - float(total_tau)
                if abs(d) > TAU_MISMATCH_ALARM:
                    anomalies.append(f"recovery {curr_pps}: {label} published TAU mismatch {d:+.12e}")

    if not anomalies:
        print("\n  Verdict: OK — persisted recovery rows are clean and Welford cardinality carried over")
    return anomalies


# -----------------------------------------------------------------------------
# Clock-domain integrity
# -----------------------------------------------------------------------------


def analyze_clock_domains(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    anomalies: List[str] = []
    print()
    print("-" * 78)
    print("CLOCK DOMAIN MONOTONICITY")
    print("-" * 78)

    clocks = [
        ("GNSS", _gnss_ns, "ns"),
        ("VCLOCK", _vclock_ns, "ns"),
        ("OCXO1", lambda r: _ocxo_ns(r, "ocxo1"), "ns"),
        ("OCXO2", lambda r: _ocxo_ns(r, "ocxo2"), "ns"),
        ("DWT", _dwt_cycles, "cycles"),
    ]

    for label, getter, unit in clocks:
        values: List[Tuple[int, int]] = []
        for row in rows:
            pps = _row_pps_count(row)
            v = getter(row)
            if v is not None:
                values.append((pps, v))
        if not values:
            print(f"\n  [{label}] No data")
            continue

        delta_stats = WelfordStats()
        non_monotonic = zero_deltas = bad_adjacent = 0
        for i in range(1, len(values)):
            pp, pv = values[i - 1]
            cp, cv = values[i]
            if cp in recovery_boundaries:
                continue
            delta = cv - pv
            if delta == 0:
                zero_deltas += 1
            elif delta < 0:
                non_monotonic += 1
            if cp == pp + 1:
                delta_stats.update(float(delta))
                if label in ("GNSS", "VCLOCK") and delta != NS_PER_SECOND:
                    bad_adjacent += 1
                elif label.startswith("OCXO") and abs(delta - NS_PER_SECOND) > OCXO_SECOND_ALARM_NS:
                    bad_adjacent += 1
                elif label == "DWT":
                    # DWT is a cycle-domain clock; compare only against sanity.
                    if delta <= 0:
                        bad_adjacent += 1

        print(f"\n  [{label}]")
        print(f"    Range: {values[0][1]:,} -> {values[-1][1]:,} {unit}")
        print(f"    Per-second delta: {delta_stats.summary()}")
        issues = []
        if zero_deltas:
            issues.append(f"{zero_deltas} zero")
        if non_monotonic:
            issues.append(f"{non_monotonic} non-monotonic")
        if bad_adjacent:
            issues.append(f"{bad_adjacent} bad adjacent delta")
        if issues:
            print(f"    Verdict: WARN — {', '.join(issues)}")
            anomalies.append(f"{label}: {', '.join(issues)} deltas")
        else:
            print("    Verdict: OK")
    return anomalies


def _micro_prefix(prefix: str) -> str:
    return "o1" if prefix == "ocxo1" else "o2"


def analyze_ocxo_integrity(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    anomalies: List[str] = []
    for ocxo_label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        micro = _micro_prefix(prefix)
        print()
        print("-" * 78)
        print(f"{ocxo_label} PER-SECOND INTEGRITY")
        print("-" * 78)

        clock_interval_stats = WelfordStats()
        residual_stats = WelfordStats()
        phase_step_stats = WelfordStats()
        ns_delta_error_stats = WelfordStats()

        alarm_records: List[Dict[str, Any]] = []
        second_warns: List[str] = []
        ns_delta_alarms: List[str] = []
        phase_step_alarms: List[str] = []
        ppb_step_alarms: List[str] = []

        prev_phase_offset: Optional[int] = None
        prev_ppb: Optional[float] = None
        prev_ns_count: Optional[int] = None
        valid_seconds = 0
        null_count = 0

        for row in rows:
            pps = _row_pps_count(row)
            if pps in recovery_boundaries:
                prev_phase_offset = None
                prev_ppb = None
                prev_ns_count = None
                continue

            clock_interval = _ocxo_clock_interval(row, prefix)
            gnss_interval = _ocxo_gnss_interval(row, prefix)
            residual = _ocxo_residual(row, prefix)
            phase_offset = _ocxo_phase_offset(row, prefix)
            ppb = _stat(row, prefix, "ppb")
            ns_count = _ocxo_ns(row, prefix)
            valid = _ocxo_valid(row, prefix)

            if not valid:
                prev_phase_offset = None
                prev_ppb = None
                prev_ns_count = None
                null_count += 1
                continue

            if clock_interval is not None and gnss_interval is not None and gnss_interval > 0:
                valid_seconds += 1
                clock_interval_stats.update(float(clock_interval))
                deviation = clock_interval - gnss_interval
                if residual is None:
                    residual = deviation

                if abs(deviation) > OCXO_SECOND_ALARM_NS:
                    alarm_records.append({
                        "pps": pps,
                        "deviation_ns": deviation,
                        "clock_interval": clock_interval,
                        "gnss_interval": gnss_interval,
                        "residual": residual,
                        "phase_offset": phase_offset,
                        "ppb": ppb,
                        "obs": _forensic_int(row, f"{micro}_obs"),
                        "eff": _forensic_int(row, f"{micro}_eff"),
                        "res_cycles": _forensic_int(row, f"{micro}_res"),
                        "pps_res": _forensic_int(row, f"{micro}_pps_res"),
                        "fl_err": _forensic_int(row, f"{micro}_fl_err"),
                        "fl_rej": _forensic_int(row, f"{micro}_fl_rej"),
                        "fl_acc": _forensic_int(row, f"{micro}_fl_acc"),
                    })
                elif abs(deviation) > OCXO_SECOND_WARN_NS:
                    second_warns.append(f"pps={pps:>7d}  deviation={deviation:+,d} ns")
            elif clock_interval is None:
                null_count += 1

            if residual is not None:
                residual_stats.update(float(residual))

            if ns_count is not None and prev_ns_count is not None:
                ns_delta = ns_count - prev_ns_count
                expected_delta = clock_interval if clock_interval is not None else (
                    NS_PER_SECOND + residual if residual is not None else None
                )
                if expected_delta is not None:
                    ns_delta_error = ns_delta - expected_delta
                    ns_delta_error_stats.update(float(ns_delta_error))
                    if abs(ns_delta_error) > OCXO_SECOND_WARN_NS:
                        ns_delta_alarms.append(
                            f"pps={pps:>7d}  ns_delta={ns_delta:,}  expected={expected_delta:,}  "
                            f"error={ns_delta_error:+,d}")
                elif ns_delta <= 0:
                    ns_delta_alarms.append(f"pps={pps:>7d}  non-monotonic ns_delta={ns_delta:,}")

            if phase_offset is not None and prev_phase_offset is not None:
                step = phase_offset - prev_phase_offset
                phase_step_stats.update(float(step))
                if abs(step) > PHASE_STEP_ALARM_NS:
                    phase_step_alarms.append(
                        f"pps={pps:>7d}  phase_step={step:+,d} ns  phase_offset={phase_offset:,}")

            if ppb is not None and prev_ppb is not None:
                ppb_step = ppb - prev_ppb
                if abs(ppb_step) > PPB_STEP_ALARM:
                    ppb_step_alarms.append(
                        f"pps={pps:>7d}  ppb_step={ppb_step:+.3f}  ppb={ppb:+.3f}  prev_ppb={prev_ppb:+.3f}")

            prev_phase_offset = phase_offset
            prev_ppb = ppb
            prev_ns_count = ns_count

        print(f"\n  Valid seconds:     {valid_seconds:,}")
        print(f"  Null/invalid rows: {null_count}")

        print("\n  OCXO clock interval (fragment.ocxoN.pps_residual.clock_interval_ns):")
        print(f"    {clock_interval_stats.summary()}")

        if alarm_records:
            print(f"\n    ALARM: {len(alarm_records)} seconds with |deviation| > {OCXO_SECOND_ALARM_NS:,} ns")
            anomalies.append(f"{ocxo_label}: {len(alarm_records)} ALARM seconds (|deviation| > {OCXO_SECOND_ALARM_NS:,} ns)")

            deviations = [r["deviation_ns"] for r in alarm_records]
            short_count = sum(1 for d in deviations if d < 0)
            long_count = sum(1 for d in deviations if d > 0)
            print("\n    Directionality:")
            print(f"      Short (measured second < GNSS): {short_count}")
            print(f"      Long  (measured second > GNSS): {long_count}")
            print(f"      Min deviation:  {min(deviations):+,d} ns")
            print(f"      Max deviation:  {max(deviations):+,d} ns")
            print(f"      Mean deviation: {sum(deviations) / len(deviations):+,.0f} ns")

            show_n = min(15, len(alarm_records))
            print(f"\n    Micro-raw forensic detail (first {show_n}):")
            print(f"    {'PPS':>7s}  {'DEV_NS':>10s}  {'OBS':>10s}  {'EFF':>10s}  {'RES':>6s}  {'PPS_RES':>7s}  {'FL_ERR':>7s}  {'FL_REJ':>6s}")
            print(f"    {'---':>7s}  {'---':>10s}  {'---':>10s}  {'---':>10s}  {'---':>6s}  {'---':>7s}  {'---':>7s}  {'---':>6s}")
            for r in alarm_records[:show_n]:
                print(f"    {r['pps']:>7d}  {r['deviation_ns']:>+10,d}  "
                      f"{str(r.get('obs')) if r.get('obs') is not None else '---':>10s}  "
                      f"{str(r.get('eff')) if r.get('eff') is not None else '---':>10s}  "
                      f"{str(r.get('res_cycles')) if r.get('res_cycles') is not None else '---':>6s}  "
                      f"{str(r.get('pps_res')) if r.get('pps_res') is not None else '---':>7s}  "
                      f"{str(r.get('fl_err')) if r.get('fl_err') is not None else '---':>7s}  "
                      f"{str(r.get('fl_rej')) if r.get('fl_rej') is not None else '---':>6s}")
            if len(alarm_records) > show_n:
                print(f"    ... and {len(alarm_records) - show_n} more")

        if second_warns:
            print(f"\n    WARN: {len(second_warns)} seconds with |deviation| > {OCXO_SECOND_WARN_NS} ns")
            for msg in second_warns[:10]:
                print(f"      {msg}")
            if len(second_warns) > 10:
                print(f"      ... and {len(second_warns) - 10} more")

        print("\n  Per-second residual (clock_interval_ns - gnss_interval_ns):")
        print(f"    {residual_stats.summary()}")

        print("\n  Public ledger delta error (delta(ns) - clock_interval_ns):")
        print(f"    {ns_delta_error_stats.summary()}")
        if ns_delta_alarms:
            print(f"\n    ALARM/WARN: {len(ns_delta_alarms)} public ledger delta mismatches:")
            for msg in ns_delta_alarms[:20]:
                print(f"      {msg}")
            if len(ns_delta_alarms) > 20:
                print(f"      ... and {len(ns_delta_alarms) - 20} more")
            anomalies.append(f"{ocxo_label}: {len(ns_delta_alarms)} public ledger delta mismatches")
        else:
            print("    Verdict: OK")

        print("\n  Phase offset per-second step (vclock_ns - ocxo_ns):")
        print(f"    {phase_step_stats.summary()}")
        if phase_step_alarms:
            print(f"\n    ALARM: {len(phase_step_alarms)} phase steps > {PHASE_STEP_ALARM_NS:,} ns:")
            for msg in phase_step_alarms[:20]:
                print(f"      {msg}")
            anomalies.append(f"{ocxo_label}: {len(phase_step_alarms)} phase offset step alarms")

        if ppb_step_alarms:
            print(f"\n    ALARM: {len(ppb_step_alarms)} published-ppb steps > {PPB_STEP_ALARM} ppb:")
            for msg in ppb_step_alarms[:20]:
                print(f"      {msg}")
            anomalies.append(f"{ocxo_label}: {len(ppb_step_alarms)} ppb step alarms")
        else:
            print(f"\n    Published PPB trajectory: OK — no steps > {PPB_STEP_ALARM} ppb")

        ocxo_issues = [a for a in anomalies if a.startswith(ocxo_label)]
        print(f"\n  {ocxo_label} VERDICT: {'PATHOLOGY DETECTED' if ocxo_issues else 'CLEAN'}")

    return anomalies


def analyze_ocxo_comparison(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    anomalies: List[str] = []
    print()
    print("-" * 78)
    print("OCXO1 vs OCXO2 COMPARATIVE")
    print("-" * 78)

    divergence_stats = WelfordStats()
    large_divergences: List[str] = []

    for row in rows:
        pps = _row_pps_count(row)
        if pps in recovery_boundaries:
            continue
        r1 = _ocxo_residual(row, "ocxo1")
        r2 = _ocxo_residual(row, "ocxo2")
        if r1 is None or r2 is None:
            continue
        diff = r1 - r2
        divergence_stats.update(float(diff))
        if abs(diff) > OCXO_SECOND_ALARM_NS:
            large_divergences.append(
                f"pps={pps:>7d}  ocxo1_res={r1:+,d}  ocxo2_res={r2:+,d}  diff={diff:+,d}")

    print("\n  Per-second residual difference (OCXO1 - OCXO2):")
    print(f"    {divergence_stats.summary()}")
    if large_divergences:
        print(f"\n    ALARM: {len(large_divergences)} seconds with |OCXO1 - OCXO2| > {OCXO_SECOND_ALARM_NS:,} ns:")
        for msg in large_divergences[:10]:
            print(f"      {msg}")
        if len(large_divergences) > 10:
            print(f"      ... and {len(large_divergences) - 10} more")
        anomalies.append(f"OCXO divergence: {len(large_divergences)} seconds with |OCXO1 - OCXO2| > {OCXO_SECOND_ALARM_NS:,} ns")
    else:
        print(f"\n    Verdict: OK — both OCXOs track within {OCXO_SECOND_ALARM_NS:,} ns")
    return anomalies


def analyze_welford_contamination(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    anomalies: List[str] = []
    print()
    print("-" * 78)
    print("WELFORD CONTAMINATION / CARDINALITY CHECK")
    print("-" * 78)

    lane_specs = [
        ("DWT", "dwt", "interval", 200.0, "cycles/ppb"),
        ("VCLOCK", "vclock", "interval", WELFORD_STDDEV_ALARM, "ns"),
        ("OCXO1", "ocxo1", "interval", WELFORD_STDDEV_ALARM, "ns"),
        ("OCXO2", "ocxo2", "interval", WELFORD_STDDEV_ALARM, "ns"),
        ("OCXO1_DAC", "ocxo1_dac", "dac", 1.0, "LSB"),
        ("OCXO2_DAC", "ocxo2_dac", "dac", 1.0, "LSB"),
        ("GNSS_RAW", "gnss_raw", "gnss_raw", 500.0, "ppb"),
    ]

    for label, prefix, kind, threshold, unit in lane_specs:
        points: List[Tuple[int, int, float, float, float]] = []
        n_alarms: List[str] = []

        for row_index, row in enumerate(rows, start=1):
            pps = _row_pps_count(row)
            if kind == "dac":
                lane = prefix.replace("_dac", "")
                n = _dac_welford_int(row, lane, "n")
                mean = _dac_welford(row, lane, "mean")
                sd = _dac_welford(row, lane, "stddev")
                se = _dac_welford(row, lane, "stderr")
            elif kind == "gnss_raw":
                extra = _extra(row)
                n = _first_int(extra.get("gnss_raw_welford_n"), row.get("gnss_raw_welford_n"))
                mean = _first_float(extra.get("gnss_raw_welford_mean"), row.get("gnss_raw_welford_mean"))
                sd = _first_float(extra.get("gnss_raw_welford_stddev"), row.get("gnss_raw_welford_stddev"))
                se = _first_float(extra.get("gnss_raw_welford_stderr"), row.get("gnss_raw_welford_stderr"))
            else:
                n = _welford_int(row, prefix, "n")
                mean = _welford(row, prefix, "mean")
                sd = _welford(row, prefix, "stddev")
                se = _welford(row, prefix, "stderr")

            if n is None or sd is None:
                continue
            points.append((pps, int(n), float(mean or 0.0), float(sd), float(se or 0.0)))

            if kind == "interval":
                expected = _expected_welford_n(pps, prefix, recovery_boundaries)
                if expected is not None and abs(int(n) - int(expected)) > WELFORD_N_TOLERANCE:
                    n_alarms.append(f"pps={pps:>7d}  n={n:,} expected={expected:,}")
            elif kind == "dac":
                expected = _expected_welford_n(pps, prefix, recovery_boundaries)
                if expected is not None and abs(int(n) - int(expected)) > WELFORD_N_TOLERANCE:
                    n_alarms.append(f"pps={pps:>7d}  n={n:,} expected={expected:,}")
            elif kind == "gnss_raw":
                # GNSS_RAW is Pi-owned and updates only when a TIMEBASE row is
                # persisted; unlike Teensy interval Welfords, it is not gap-
                # advanced across recovered missing PPS identities.
                expected = row_index
                if int(n) != expected:
                    n_alarms.append(f"row={row_index:>7d} pps={pps:>7d}  n={n:,} expected_persisted_rows={expected:,}")

        if not points:
            print(f"\n  [{label}] No Welford data")
            continue

        first_pps, first_n, first_mean, first_sd, _ = points[0]
        final_pps, final_n, final_mean, final_sd, final_se = points[-1]
        max_pps, max_n, _, max_sd, _ = max(points, key=lambda x: x[3])

        print(f"\n  [{label}] Welford trajectory ({unit}):")
        print(f"    First: pps={first_pps} n={first_n:,} mean={first_mean:+.6f} sd={first_sd:.6f}")
        print(f"    Final: pps={final_pps} n={final_n:,} mean={final_mean:+.6f} sd={final_sd:.6f} se={final_se:.6f}")
        print(f"    Peak SD: pps={max_pps} n={max_n:,} sd={max_sd:.6f}")

        jumps: List[str] = []
        n_regressions: List[str] = []
        for i in range(1, len(points)):
            pp, pn, _, ps, _ = points[i - 1]
            cp, cn, _, cs, _ = points[i]
            if cn < pn:
                n_regressions.append(f"pps={cp:>7d}  n {pn:,} -> {cn:,}")
            if cp in recovery_boundaries:
                continue
            if ps > 0.0 and cs > ps * 2.0 and cs > 10.0:
                jumps.append(f"pps={cp:>7d}  stddev {ps:.3f} -> {cs:.3f}  (x{cs/ps:.1f})")

        if n_alarms:
            print(f"    ALARM: {len(n_alarms)} Welford cardinality mismatch(es)")
            for msg in n_alarms[:10]:
                print(f"      {msg}")
            if len(n_alarms) > 10:
                print(f"      ... and {len(n_alarms) - 10} more")
            anomalies.append(f"{label}: {len(n_alarms)} Welford N mismatch(es)")

        if n_regressions:
            print(f"    ALARM: {len(n_regressions)} Welford N regression(s)")
            for msg in n_regressions[:10]:
                print(f"      {msg}")
            anomalies.append(f"{label}: {len(n_regressions)} Welford N regression(s)")

        if jumps:
            print(f"    WARN: {len(jumps)} sudden stddev jumps (>2x):")
            for msg in jumps[:10]:
                print(f"      {msg}")
            if len(jumps) > 10:
                print(f"      ... and {len(jumps) - 10} more")

        if final_sd > threshold:
            print(f"    ALARM: final stddev {final_sd:.6f} > {threshold} — Welford is contaminated")
            anomalies.append(f"{label}: Welford stddev contaminated ({final_sd:.6f} {unit})")
        elif not n_alarms and not n_regressions:
            print("    Verdict: OK")
    return anomalies


def analyze_ppb_sanity(rows: List[Dict[str, Any]], recovery_boundaries: Set[int]) -> List[str]:
    anomalies: List[str] = []
    print()
    print("-" * 78)
    print(f"TAU / PPB SANITY CHECK (threshold: ±{PPB_ABSOLUTE_THRESHOLD:,} ppb; recovery continuity ±{RECOVERY_OCXO_CONTINUITY_PPB_ALARM:.2f} ppb)")
    print("-" * 78)

    for label, prefix in [("DWT", "dwt"), ("VCLOCK", "vclock"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        published_ppb_stats = WelfordStats()
        total_ppb_stats = WelfordStats()
        ppb_mismatch_stats = WelfordStats()
        tau_mismatch_stats = WelfordStats()
        science_ppb_mismatch_stats = WelfordStats()
        violations: List[str] = []
        mismatches: List[str] = []

        for row in rows:
            pps = _row_pps_count(row)
            published_ppb = _published_ppb(row, prefix)
            published_tau = _published_tau(row, prefix)
            total_ppb = _campaign_total_ppb(row, prefix)
            total_tau = _campaign_total_tau(row, prefix)
            tau_from_ppb = _tau_from_ppb(published_ppb)

            if published_ppb is not None:
                published_ppb_stats.update(float(published_ppb))
                if abs(float(published_ppb)) > PPB_ABSOLUTE_THRESHOLD:
                    violations.append(f"pps={pps:>7d}  published_ppb={published_ppb:+,.3f}")
            if total_ppb is not None:
                total_ppb_stats.update(float(total_ppb))
                if abs(float(total_ppb)) > PPB_ABSOLUTE_THRESHOLD:
                    violations.append(f"pps={pps:>7d}  ledger_total_ppb={total_ppb:+,.3f}")
            if published_tau is not None:
                if abs(float(published_tau) - 1.0) > TAU_ABSOLUTE_DEVIATION_THRESHOLD:
                    violations.append(f"pps={pps:>7d}  published_tau={published_tau:.12f}")
            if total_tau is not None:
                if abs(float(total_tau) - 1.0) > TAU_ABSOLUTE_DEVIATION_THRESHOLD:
                    violations.append(f"pps={pps:>7d}  ledger_total_tau={total_tau:.12f}")

            if published_ppb is not None and total_ppb is not None:
                d = float(published_ppb) - float(total_ppb)
                ppb_mismatch_stats.update(d)
                if abs(d) > PPB_MISMATCH_ALARM:
                    mismatches.append(f"pps={pps:>7d}  published_ppb-total_ppb={d:+.6f}")
            if published_tau is not None and total_tau is not None:
                d = float(published_tau) - float(total_tau)
                tau_mismatch_stats.update(d)
                if abs(d) > TAU_MISMATCH_ALARM:
                    mismatches.append(f"pps={pps:>7d}  published_tau-total_tau={d:+.12e}")
            if published_tau is not None and tau_from_ppb is not None:
                d = float(published_tau) - float(tau_from_ppb)
                if abs(d) > TAU_MISMATCH_ALARM:
                    mismatches.append(f"pps={pps:>7d}  published_tau-(1+ppb/1e9)={d:+.12e}")

            if prefix in ("ocxo1", "ocxo2"):
                science_ppb = _science_float(row, prefix, "total_ppb")
                science_tau = _science_float(row, prefix, "total_tau")
                if published_ppb is not None and science_ppb is not None:
                    d = float(published_ppb) - float(science_ppb)
                    science_ppb_mismatch_stats.update(d)
                    if abs(d) > PPB_MISMATCH_ALARM:
                        mismatches.append(f"pps={pps:>7d}  published_ppb-science_total_ppb={d:+.6f}")
                if published_tau is not None and science_tau is not None:
                    d = float(published_tau) - float(science_tau)
                    if abs(d) > TAU_MISMATCH_ALARM:
                        mismatches.append(f"pps={pps:>7d}  published_tau-science_total_tau={d:+.12e}")

        print(f"\n  [{label}] Published PPB:")
        print(f"    {published_ppb_stats.summary()}")
        print(f"  [{label}] Campaign-total PPB computed from public ledgers:")
        print(f"    {total_ppb_stats.summary()}")
        if ppb_mismatch_stats.n:
            print(f"  [{label}] Published minus campaign-total PPB:")
            print(f"    {ppb_mismatch_stats.summary(fmt='.6f')}")
        if science_ppb_mismatch_stats.n:
            print(f"  [{label}] Published minus science.total_ppb:")
            print(f"    {science_ppb_mismatch_stats.summary(fmt='.6f')}")
        if tau_mismatch_stats.n:
            print(f"  [{label}] Published minus campaign-total TAU:")
            print(f"    {tau_mismatch_stats.summary(fmt='.12e')}")

        if violations:
            print(f"    ALARM: {len(violations)} records exceed absolute TAU/PPB sanity bounds")
            for msg in violations[:8]:
                print(f"      {msg}")
            if len(violations) > 8:
                print(f"      ... and {len(violations) - 8} more")
            anomalies.append(f"{label}: {len(violations)} records outside TAU/PPB sanity bounds")

        if mismatches:
            print(f"    ALARM: {len(mismatches)} published TAU/PPB consistency mismatch(es)")
            for msg in mismatches[:8]:
                print(f"      {msg}")
            if len(mismatches) > 8:
                print(f"      ... and {len(mismatches) - 8} more")
            anomalies.append(f"{label}: {len(mismatches)} TAU/PPB consistency mismatch(es)")

        if not violations and not mismatches:
            print("    Verdict: OK")
    return anomalies


# -----------------------------------------------------------------------------
# Operator/debug summaries
# -----------------------------------------------------------------------------


def analyze_servo(rows: List[Dict[str, Any]]) -> None:
    print()
    print("-" * 78)
    print("OCXO SERVO & DAC STATE")
    print("-" * 78)
    print(f"\n  Servo mode: {_servo_mode(rows[-1])}")
    for label, prefix in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        frag = _fragment(rows[-1])
        dac = _first_float(
            _path_get(frag, f"dac.{prefix}_dac"),
            _path_get(frag, f"dac.{prefix}.value"),
            _path_get(frag, f"dac.{prefix}.dac"),
            frag.get(f"{prefix}_dac"),
        )
        dac_mean = _dac_welford(rows[-1], prefix, "mean")
        dac_sd = _dac_welford(rows[-1], prefix, "stddev")
        dac_n = _dac_welford_int(rows[-1], prefix, "n")
        dac_se = _dac_welford(rows[-1], prefix, "stderr")
        print(f"\n  [{label}]")
        if dac is not None:
            print(f"    Current DAC:     {dac:.6f}")
        if dac_mean is not None:
            print(f"    DAC Welford:     mean={dac_mean:.6f}  sd={(dac_sd or 0.0):.6f}  n={dac_n}  se={(dac_se or 0.0):.6f}")


def analyze_micro_raw_cycles(rows: List[Dict[str, Any]]) -> None:
    print()
    print("-" * 78)
    print("TIMEBASE_FORENSICS MICRO RAW CYCLES (final record)")
    print("-" * 78)
    f = _forensics(rows[-1])
    if not f:
        print("  No forensics payload")
        return
    if not f.get("micro_raw_cycles"):
        print("  No MICRO_RAW_CYCLES_V1 payload in final forensics")
        return

    print(f"\n  Schema: {f.get('micro_schema', '?')}  temporary={f.get('temporary_safety_mode', '?')}")
    print(f"  PPS/V edge available: {f.get('pps_vclock_edge_available')}")
    print()
    print(f"  {'LANE':<7s} {'OBS':>12s} {'EFF':>12s} {'RES':>8s} {'RAW':>12s} {'PUB':>12s} {'FL_ERR':>8s} {'PPS_RES':>8s}")
    print(f"  {'-'*7} {'-'*12} {'-'*12} {'-'*8} {'-'*12} {'-'*12} {'-'*8} {'-'*8}")
    for label, prefix in [("VCLOCK", "v"), ("OCXO1", "o1"), ("OCXO2", "o2")]:
        print(f"  {label:<7s} "
              f"{str(f.get(prefix + '_obs', '---')):>12s} "
              f"{str(f.get(prefix + '_eff', '---')):>12s} "
              f"{str(f.get(prefix + '_res', '---')):>8s} "
              f"{str(f.get(prefix + '_raw', '---')):>12s} "
              f"{str(f.get(prefix + '_pub', '---')):>12s} "
              f"{str(f.get(prefix + '_fl_err', '---')):>8s} "
              f"{str(f.get(prefix + '_pps_res', '---')):>8s}")


def print_environment(rows: List[Dict[str, Any]]) -> None:
    print()
    print("-" * 78)
    print("ENVIRONMENT (final record)")
    print("-" * 78)
    env = _as_dict(rows[-1].get("environment"))
    if not env:
        print("  No environment data")
        return
    for key, label, unit in [
        ("ambient_temp_c", "Ambient", "°C"),
        ("teensy_temp_c", "Teensy", "°C"),
        ("gnss_temp_c", "GNSS", "°C"),
        ("pressure_hpa", "Pressure", "hPa"),
        ("humidity_pct", "Humidity", "%"),
    ]:
        v = env.get(key)
        if v is not None:
            print(f"  {label + ':':13s} {float(v):.1f} {unit}")
    batt = _as_dict(env.get("battery"))
    if batt:
        pct = batt.get("remaining_pct")
        tte = batt.get("tte_minutes")
        health = batt.get("health_state")
        if pct is not None:
            print(f"  {'Battery:':13s} {float(pct):.0f}%  TTE={float(tte or 0):.0f}m  {health}")


# -----------------------------------------------------------------------------
# Top-level driver
# -----------------------------------------------------------------------------


def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)
    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    recovery_boundaries = find_recovery_boundaries(rows)
    anomalies: List[str] = []

    print_campaign_overview(rows, recovery_boundaries)
    anomalies.extend(analyze_pps_continuity(rows, recovery_boundaries))
    anomalies.extend(analyze_recovery_projection(rows, recovery_boundaries))
    anomalies.extend(analyze_recovery_cleanliness(rows, recovery_boundaries))
    anomalies.extend(analyze_clock_domains(rows, recovery_boundaries))
    anomalies.extend(analyze_ocxo_integrity(rows, recovery_boundaries))
    anomalies.extend(analyze_ocxo_comparison(rows, recovery_boundaries))
    anomalies.extend(analyze_welford_contamination(rows, recovery_boundaries))
    anomalies.extend(analyze_ppb_sanity(rows, recovery_boundaries))
    analyze_servo(rows)
    analyze_micro_raw_cycles(rows)
    print_environment(rows)

    print()
    print("=" * 78)
    if anomalies:
        print(f"VERDICT: ANOMALIES FOUND ({len(anomalies)})")
        for a in anomalies:
            print(f"  * {a}")
    elif recovery_boundaries:
        print(f"VERDICT: CLEAN (with {len(recovery_boundaries)} certified recovery events)")
    else:
        print("VERDICT: CLEAN — all checks passed")
    print("=" * 78)


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: campaign_analyzer <campaign_name>")
        print()
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute("""
                    SELECT t.campaign,
                           bool_or(c.active) AS active,
                           count(*) AS tb_count,
                           min(COALESCE(
                               NULLIF(t.payload->>'teensy_pps_vclock_count', '')::bigint,
                               NULLIF(t.payload->>'pps_count', '')::bigint,
                               NULLIF(t.payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                               NULLIF(t.payload->'fragment'->>'pps_count', '')::bigint
                           )) AS pps_min,
                           max(COALESCE(
                               NULLIF(t.payload->>'teensy_pps_vclock_count', '')::bigint,
                               NULLIF(t.payload->>'pps_count', '')::bigint,
                               NULLIF(t.payload->'fragment'->>'teensy_pps_vclock_count', '')::bigint,
                               NULLIF(t.payload->'fragment'->>'pps_count', '')::bigint
                           )) AS pps_max,
                           max(t.ts) AS last_ts
                    FROM timebase t
                    LEFT JOIN campaigns c USING (campaign)
                    GROUP BY t.campaign
                    ORDER BY max(t.ts) DESC
                    LIMIT 20
                """)
                rows = cur.fetchall()
            if rows:
                print("Available campaigns:")
                print(f"  {'CAMPAIGN':<20s} {'ACTIVE':>7s} {'RECORDS':>8s} {'PPS RANGE':>20s}")
                print(f"  {'-'*20} {'-'*7} {'-'*8} {'-'*20}")
                for r in rows:
                    active = ">" if r["active"] else " "
                    pps_range = f"{r['pps_min']}-{r['pps_max']}"
                    print(f"  {r['campaign']:<20s} {active:>7s} {r['tb_count']:>8d} {pps_range:>20s}")
        except Exception:
            pass
        sys.exit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()
