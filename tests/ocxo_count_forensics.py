"""
ZPNet OCXO Count Forensics

Campaign-centric analyzer for OCXO counter / ledger / residual pathologies.

Purpose
-------
This tool inspects TIMEBASE rows for a campaign and tries to classify wild
OCXO residual spikes by comparing every representation of the same second:

  * OCXO per-edge span:
      ocxoN_gnss_ns_between_edges
      ocxoN_second_residual_ns = 1e9 - span

  * OCXO public ledger at PPS/VCLOCK:
      ocxoN_ns or ocxoN_ns_at_pps_vclock
      expected ledger delta is alignment-sensitive. In the current firmware
      publication order, public OCXO ledger deltas often align with the
      previous row's residual rather than the current row's residual. Use
      --ledger-alignment previous (default), current, or auto.

  * OCXO phase offset:
      phase_delta = phase_now - phase_prev
      expected phase_delta = -ocxoN_second_residual_ns

  * OCXO synthetic 32-bit counter identity:
      diag_counter32_at_event should advance by 10,000,000 ticks per OCXO
      one-second event, modulo 2^32

  * OCXO DWT event coordinate:
      delta(diag_dwt_at_event) should equal ocxoN_dwt_cycles_between_edges

The goal is to distinguish these broad failure classes:

  * EVENT_SPAN / EDGE_TIMING:
      the per-edge GNSS span itself is huge/small, so the event happened at
      the wrong bridge time. This suggests missed/extra OCXO cadence, electrical
      edge issues, or bridge conversion trouble.

  * LEDGER_ONLY:
      the per-edge residual is sane, but the public OCXO ledger jumps wildly.
      This suggests phase/ledger/base arithmetic rather than physical OCXO.

  * COUNTER32_JUMP:
      synthetic OCXO counter32 does not advance by exactly 10,000,000 ticks.
      This suggests missed cadence accounting or synthetic counter zero/advance
      trouble.

  * DIAG_STALE_OR_INCOHERENT:
      published diag deltas do not agree with measurement fields.

  * COMMON_MODE:
      both OCXO lanes spike together, suggesting a shared bridge/reference path.

Usage
-----
    python -m zpnet.tests.ocxo_count_forensics <campaign>
    python -m zpnet.tests.ocxo_count_forensics <campaign> --limit 200
    python -m zpnet.tests.ocxo_count_forensics <campaign> --around 1249 --radius 20
    python -m zpnet.tests.ocxo_count_forensics <campaign> --csv /tmp/ocxo_issues.csv
    python -m zpnet.tests.ocxo_count_forensics <campaign> --ledger-alignment current
    python -m zpnet.tests.ocxo_count_forensics <campaign> --ledger-alignment auto

Examples
--------
    .zt ocxo_count_forensics Converge18
    .zt ocxo_count_forensics foozy1 --around 1249 --radius 30

Notes
-----
Reads TIMEBASE.payload from Postgres. It is intentionally tolerant of schema
drift: values are read first from payload["fragment"], then from top-level
payload fields, and common legacy aliases are tried where useful.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

from zpnet.shared.db import open_db


NS_PER_SECOND = 1_000_000_000
OCXO_COUNTER32_TICKS_PER_SECOND = 10_000_000
MOD32 = 2 ** 32

DEFAULT_SPIKE_NS = 10_000
DEFAULT_JUMP_NS = 1_000
DEFAULT_LEDGER_ERR_NS = 1
DEFAULT_COUNTER_TICK_ERR = 0
DEFAULT_MAX_ISSUES = 80
LEDGER_ALIGNMENT_CHOICES = ("previous", "current", "auto")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def as_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def as_bool(value: Any) -> Optional[bool]:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("true", "1", "yes", "on"):
            return True
        if lowered in ("false", "0", "no", "off"):
            return False
    try:
        return bool(value)
    except Exception:
        return None


def fmt_int(value: Optional[int], width: int = 0, signed: bool = False) -> str:
    if value is None:
        s = "---"
    elif signed:
        s = f"{value:+,d}"
    else:
        s = f"{value:,d}"
    return f"{s:>{width}s}" if width else s


def fmt_float(value: Optional[float], width: int = 0, digits: int = 3, signed: bool = False) -> str:
    if value is None:
        s = "---"
    elif signed:
        s = f"{value:+.{digits}f}"
    else:
        s = f"{value:.{digits}f}"
    return f"{s:>{width}s}" if width else s


def mod32_delta(curr: Optional[int], prev: Optional[int]) -> Optional[int]:
    if curr is None or prev is None:
        return None
    return (int(curr) - int(prev)) & 0xFFFFFFFF


def signed_mod32_delta(curr: Optional[int], prev: Optional[int]) -> Optional[int]:
    d = mod32_delta(curr, prev)
    if d is None:
        return None
    return d - MOD32 if d >= (MOD32 // 2) else d


def first_present(mapping: Dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in mapping and mapping.get(key) is not None:
            return mapping.get(key)
    return None


def fragment(row: Dict[str, Any]) -> Dict[str, Any]:
    f = row.get("fragment")
    return f if isinstance(f, dict) else {}


def field_value(row: Dict[str, Any], *keys: str) -> Any:
    f = fragment(row)
    v = first_present(f, *keys)
    if v is not None:
        return v
    return first_present(row, *keys)


def field_int(row: Dict[str, Any], *keys: str) -> Optional[int]:
    return as_int(field_value(row, *keys))


def field_float(row: Dict[str, Any], *keys: str) -> Optional[float]:
    return as_float(field_value(row, *keys))


def field_bool(row: Dict[str, Any], *keys: str) -> Optional[bool]:
    return as_bool(field_value(row, *keys))


class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, value: Optional[float]) -> None:
        if value is None:
            return
        x = float(value)
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

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0

    def summary(self, digits: int = 3) -> str:
        if self.n == 0:
            return "n=0"
        return (
            f"n={self.n:,} mean={self.mean:+.{digits}f} "
            f"sd={self.stddev:.{digits}f} se={self.stderr:.{digits}f} "
            f"min={self.min_val:+.{digits}f} max={self.max_val:+.{digits}f}"
        )


class RunningCovariance:
    __slots__ = ("n", "mean_x", "mean_y", "c", "m2x", "m2y")

    def __init__(self) -> None:
        self.n = 0
        self.mean_x = 0.0
        self.mean_y = 0.0
        self.c = 0.0
        self.m2x = 0.0
        self.m2y = 0.0

    def update(self, x: Optional[float], y: Optional[float]) -> None:
        if x is None or y is None:
            return
        fx = float(x)
        fy = float(y)
        self.n += 1
        dx = fx - self.mean_x
        self.mean_x += dx / self.n
        dy = fy - self.mean_y
        self.mean_y += dy / self.n
        self.c += dx * (fy - self.mean_y)
        self.m2x += dx * (fx - self.mean_x)
        self.m2y += dy * (fy - self.mean_y)

    @property
    def corr(self) -> float:
        if self.n < 2 or self.m2x <= 0 or self.m2y <= 0:
            return 0.0
        return self.c / math.sqrt(self.m2x * self.m2y)


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

@dataclass
class LaneSample:
    lane: str
    pps: int
    index: int

    ocxo_ns: Optional[int]
    gnss_ns: Optional[int]
    between_edges_ns: Optional[int]
    residual_ns: Optional[int]
    phase_offset_ns: Optional[int]
    zero_established: Optional[bool]

    dwt_cycles_between_edges: Optional[int]
    diag_dwt_at_event: Optional[int]
    diag_gnss_ns_at_event: Optional[int]
    diag_counter32_at_event: Optional[int]
    window_error_ns: Optional[int]
    window_mismatches: Optional[int]
    diag_anomaly_count: Optional[int]

    # Computed against prior consecutive sample from same lane
    pps_gap: Optional[int] = None
    ledger_delta_ns: Optional[int] = None
    ledger_expected_delta_ns: Optional[int] = None
    ledger_error_ns: Optional[int] = None
    ledger_error_current_ns: Optional[int] = None
    ledger_error_previous_ns: Optional[int] = None
    ledger_alignment_used: Optional[str] = None

    phase_delta_ns: Optional[int] = None
    phase_error_ns: Optional[int] = None

    diag_gnss_delta_ns: Optional[int] = None
    diag_gnss_error_ns: Optional[int] = None

    diag_dwt_delta_cycles: Optional[int] = None
    diag_dwt_error_cycles: Optional[int] = None

    counter32_delta_ticks: Optional[int] = None
    counter32_error_ticks: Optional[int] = None

    residual_jump_ns: Optional[int] = None
    window_mismatch_increment: Optional[int] = None
    diag_anomaly_increment: Optional[int] = None

    tags: List[str] = field(default_factory=list)

    @property
    def residual_formula_error(self) -> Optional[int]:
        if self.between_edges_ns is None or self.residual_ns is None:
            return None
        return self.residual_ns - (NS_PER_SECOND - self.between_edges_ns)

    @property
    def span_deviation_ns(self) -> Optional[int]:
        if self.between_edges_ns is None:
            return None
        return self.between_edges_ns - NS_PER_SECOND


@dataclass
class RowPair:
    pps: int
    index: int
    gnss_ns: Optional[int]
    o1: LaneSample
    o2: LaneSample


@dataclass
class Thresholds:
    spike_ns: int = DEFAULT_SPIKE_NS
    jump_ns: int = DEFAULT_JUMP_NS
    ledger_err_ns: int = DEFAULT_LEDGER_ERR_NS
    counter_tick_err: int = DEFAULT_COUNTER_TICK_ERR


# ---------------------------------------------------------------------------
# DB fetch
# ---------------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY COALESCE(
                       NULLIF(payload->>'pps_count', '')::bigint,
                       NULLIF(payload->>'teensy_pps_vclock_count', '')::bigint,
                       NULLIF((payload->'fragment')->>'teensy_pps_vclock_count', '')::bigint,
                       id
                     ) ASC,
                     ts ASC,
                     id ASC
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
            payload["_db_id"] = row["id"]
            payload["_db_ts"] = str(row["ts"])
            out.append(payload)
    return out


# ---------------------------------------------------------------------------
# Extraction and analysis
# ---------------------------------------------------------------------------

def extract_lane(row: Dict[str, Any], lane: str, index: int) -> LaneSample:
    pps = field_int(row, "pps_count", "teensy_pps_vclock_count", "teensy_pps_count") or 0

    ocxo_ns = field_int(
        row,
        f"{lane}_ns",
        f"{lane}_ns_at_pps_vclock",
        f"{lane}_ns_at_pps",
        f"teensy_{lane}_ns",
    )

    return LaneSample(
        lane=lane,
        pps=pps,
        index=index,
        ocxo_ns=ocxo_ns,
        gnss_ns=field_int(row, "gnss_ns", "pps_vclock_ns", "vclock_ns", "pps_ns", "teensy_gnss_ns"),
        between_edges_ns=field_int(row, f"{lane}_gnss_ns_between_edges"),
        residual_ns=field_int(row, f"{lane}_second_residual_ns", f"{lane}_residual_ns"),
        phase_offset_ns=field_int(row, f"{lane}_phase_offset_ns"),
        zero_established=field_bool(row, f"{lane}_zero_established"),
        dwt_cycles_between_edges=field_int(row, f"{lane}_dwt_cycles_between_edges"),
        diag_dwt_at_event=field_int(row, f"{lane}_diag_dwt_at_event", f"{lane}_dwt_at_edge"),
        diag_gnss_ns_at_event=field_int(row, f"{lane}_diag_gnss_ns_at_event", f"{lane}_gnss_ns_at_edge"),
        diag_counter32_at_event=field_int(row, f"{lane}_diag_counter32_at_event"),
        window_error_ns=field_int(row, f"{lane}_window_error_ns"),
        window_mismatches=field_int(row, f"{lane}_window_mismatches"),
        diag_anomaly_count=field_int(row, f"{lane}_diag_anomaly_count"),
    )


def compute_lane_deltas(samples: List[LaneSample], ledger_alignment: str = "previous") -> None:
    if ledger_alignment not in LEDGER_ALIGNMENT_CHOICES:
        raise ValueError(f"invalid ledger_alignment={ledger_alignment!r}")

    prev: Optional[LaneSample] = None
    for s in samples:
        if prev is None:
            prev = s
            continue

        s.pps_gap = s.pps - prev.pps
        consecutive = (s.pps_gap == 1)

        if not consecutive:
            s.tags.append("PPS_GAP")
            prev = s
            continue

        if s.ocxo_ns is not None and prev.ocxo_ns is not None:
            s.ledger_delta_ns = s.ocxo_ns - prev.ocxo_ns

        # Public OCXO ledger publication is not guaranteed to align with the
        # same row's latest per-edge residual. In the current firmware stream,
        # ledger deltas often match the previous residual because the PPS/V
        # publication path and OCXO event path are distinct foreground-dispatch
        # surfaces. Compute both candidates, then select according to the
        # requested alignment mode.
        current_expected = (NS_PER_SECOND + s.residual_ns) if s.residual_ns is not None else None
        previous_expected = (NS_PER_SECOND + prev.residual_ns) if prev.residual_ns is not None else None

        if s.ledger_delta_ns is not None and current_expected is not None:
            s.ledger_error_current_ns = s.ledger_delta_ns - current_expected
        if s.ledger_delta_ns is not None and previous_expected is not None:
            s.ledger_error_previous_ns = s.ledger_delta_ns - previous_expected

        if ledger_alignment == "current":
            s.ledger_expected_delta_ns = current_expected
            s.ledger_error_ns = s.ledger_error_current_ns
            s.ledger_alignment_used = "current" if current_expected is not None else None
        elif ledger_alignment == "previous":
            s.ledger_expected_delta_ns = previous_expected
            s.ledger_error_ns = s.ledger_error_previous_ns
            s.ledger_alignment_used = "previous" if previous_expected is not None else None
        else:
            # Pick whichever model explains the public ledger better. This is a
            # diagnostic mode, not a repair path. It helps identify whether a
            # campaign changed publication ordering mid-stream.
            candidates = []
            if s.ledger_error_current_ns is not None:
                candidates.append((abs(s.ledger_error_current_ns), "current", current_expected, s.ledger_error_current_ns))
            if s.ledger_error_previous_ns is not None:
                candidates.append((abs(s.ledger_error_previous_ns), "previous", previous_expected, s.ledger_error_previous_ns))
            if candidates:
                _, label, expected, err = min(candidates, key=lambda x: x[0])
                s.ledger_expected_delta_ns = expected
                s.ledger_error_ns = err
                s.ledger_alignment_used = label

        if s.phase_offset_ns is not None and prev.phase_offset_ns is not None:
            s.phase_delta_ns = s.phase_offset_ns - prev.phase_offset_ns
            if s.residual_ns is not None:
                # Expected: phase_delta = -residual.
                s.phase_error_ns = s.phase_delta_ns + s.residual_ns

        if s.diag_gnss_ns_at_event is not None and prev.diag_gnss_ns_at_event is not None:
            if s.diag_gnss_ns_at_event > 0 and prev.diag_gnss_ns_at_event > 0:
                s.diag_gnss_delta_ns = s.diag_gnss_ns_at_event - prev.diag_gnss_ns_at_event
                if s.between_edges_ns is not None:
                    s.diag_gnss_error_ns = s.diag_gnss_delta_ns - s.between_edges_ns

        if s.diag_dwt_at_event is not None and prev.diag_dwt_at_event is not None:
            s.diag_dwt_delta_cycles = signed_mod32_delta(s.diag_dwt_at_event, prev.diag_dwt_at_event)
            if s.dwt_cycles_between_edges is not None and s.diag_dwt_delta_cycles is not None:
                s.diag_dwt_error_cycles = s.diag_dwt_delta_cycles - s.dwt_cycles_between_edges

        if s.diag_counter32_at_event is not None and prev.diag_counter32_at_event is not None:
            s.counter32_delta_ticks = mod32_delta(s.diag_counter32_at_event, prev.diag_counter32_at_event)
            if s.counter32_delta_ticks is not None:
                s.counter32_error_ticks = s.counter32_delta_ticks - OCXO_COUNTER32_TICKS_PER_SECOND

        if s.residual_ns is not None and prev.residual_ns is not None:
            s.residual_jump_ns = s.residual_ns - prev.residual_ns

        if s.window_mismatches is not None and prev.window_mismatches is not None:
            inc = s.window_mismatches - prev.window_mismatches
            if inc > 0:
                s.window_mismatch_increment = inc

        if s.diag_anomaly_count is not None and prev.diag_anomaly_count is not None:
            inc = s.diag_anomaly_count - prev.diag_anomaly_count
            if inc > 0:
                s.diag_anomaly_increment = inc

        prev = s


def classify_sample(s: LaneSample, th: Thresholds) -> None:
    formula_error = s.residual_formula_error
    if formula_error is not None and formula_error != 0:
        s.tags.append("RESIDUAL_FORMULA_MISMATCH")

    if s.between_edges_ns is None or s.between_edges_ns == 0:
        s.tags.append("NO_EDGE_SPAN")

    if s.diag_gnss_ns_at_event == 0:
        s.tags.append("DIAG_GNSS_ZERO")

    if s.zero_established is False:
        s.tags.append("ZERO_NOT_ESTABLISHED")

    if s.residual_ns is not None and abs(s.residual_ns) >= th.spike_ns:
        s.tags.append("EVENT_SPAN_SPIKE")

    if s.residual_jump_ns is not None and abs(s.residual_jump_ns) >= th.jump_ns:
        s.tags.append("RESIDUAL_JUMP")

    if s.ledger_error_ns is not None and abs(s.ledger_error_ns) >= th.ledger_err_ns:
        s.tags.append("LEDGER_INCOHERENT")

    if s.phase_error_ns is not None and s.phase_error_ns != 0:
        s.tags.append("PHASE_INCOHERENT")

    if s.diag_gnss_error_ns is not None and s.diag_gnss_error_ns != 0:
        s.tags.append("DIAG_GNSS_INCOHERENT")

    if s.diag_dwt_error_cycles is not None and s.diag_dwt_error_cycles != 0:
        s.tags.append("DIAG_DWT_INCOHERENT")

    if s.counter32_error_ticks is not None and abs(s.counter32_error_ticks) > th.counter_tick_err:
        s.tags.append("COUNTER32_JUMP")

    if s.window_mismatch_increment:
        s.tags.append("WINDOW_MISMATCH")

    if s.diag_anomaly_increment:
        s.tags.append("DIAG_ANOMALY")

    # Higher-level diagnosis hints.
    if "LEDGER_INCOHERENT" in s.tags and "EVENT_SPAN_SPIKE" not in s.tags:
        s.tags.append("LIKELY_LEDGER_ONLY")
    if "EVENT_SPAN_SPIKE" in s.tags and "COUNTER32_JUMP" in s.tags:
        s.tags.append("LIKELY_COUNTER_OR_EDGE_LOSS")
    if "EVENT_SPAN_SPIKE" in s.tags and "COUNTER32_JUMP" not in s.tags:
        s.tags.append("LIKELY_BRIDGE_OR_EVENT_TIME")
    if "DIAG_GNSS_INCOHERENT" in s.tags or "DIAG_DWT_INCOHERENT" in s.tags:
        s.tags.append("LIKELY_STALE_DIAG_OR_MEASURE")


def build_pairs(rows: List[Dict[str, Any]], th: Thresholds, ledger_alignment: str = "previous") -> List[RowPair]:
    pairs: List[RowPair] = []
    o1_samples: List[LaneSample] = []
    o2_samples: List[LaneSample] = []

    for idx, row in enumerate(rows):
        pps = field_int(row, "pps_count", "teensy_pps_vclock_count", "teensy_pps_count") or 0
        gnss_ns = field_int(row, "gnss_ns", "pps_vclock_ns", "vclock_ns", "pps_ns", "teensy_gnss_ns")
        o1 = extract_lane(row, "ocxo1", idx)
        o2 = extract_lane(row, "ocxo2", idx)
        pairs.append(RowPair(pps=pps, index=idx, gnss_ns=gnss_ns, o1=o1, o2=o2))
        o1_samples.append(o1)
        o2_samples.append(o2)

    compute_lane_deltas(o1_samples, ledger_alignment=ledger_alignment)
    compute_lane_deltas(o2_samples, ledger_alignment=ledger_alignment)

    for p in pairs:
        classify_sample(p.o1, th)
        classify_sample(p.o2, th)

        # Common-mode hint: both lanes have big residuals or jumps on the same row.
        o1_spike = "EVENT_SPAN_SPIKE" in p.o1.tags
        o2_spike = "EVENT_SPAN_SPIKE" in p.o2.tags
        o1_jump = "RESIDUAL_JUMP" in p.o1.tags
        o2_jump = "RESIDUAL_JUMP" in p.o2.tags
        if o1_spike and o2_spike:
            p.o1.tags.append("COMMON_MODE_ROW")
            p.o2.tags.append("COMMON_MODE_ROW")
        elif o1_jump and o2_jump:
            p.o1.tags.append("COMMON_MODE_JUMP")
            p.o2.tags.append("COMMON_MODE_JUMP")
        elif o1_spike != o2_spike:
            (p.o1 if o1_spike else p.o2).tags.append("SINGLE_LANE_SPIKE")

    return pairs


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_campaign_overview(campaign: str, rows: List[Dict[str, Any]], pairs: List[RowPair], ledger_alignment: str) -> None:
    pps_values = [p.pps for p in pairs if p.pps is not None]
    pps_min = min(pps_values) if pps_values else 0
    pps_max = max(pps_values) if pps_values else 0
    expected = (pps_max - pps_min + 1) if pps_values else 0
    missing = expected - len(pairs)

    first_time = rows[0].get("gnss_time_utc") or rows[0].get("system_time_utc") or "?"
    last_time = rows[-1].get("gnss_time_utc") or rows[-1].get("system_time_utc") or "?"

    gaps = 0
    for i in range(1, len(pairs)):
        if pairs[i].pps != pairs[i - 1].pps + 1:
            gaps += 1

    print("=" * 118)
    print(f"OCXO COUNT FORENSICS: {campaign}")
    print("=" * 118)
    print(f"records:            {len(pairs):,}")
    print(f"pps range:          {pps_min:,} -> {pps_max:,}")
    print(f"expected records:   {expected:,}")
    print(f"missing records:    {missing:,}")
    print(f"pps gaps:           {gaps:,}")
    print(f"time range:         {first_time} -> {last_time}")
    print(f"ledger alignment:   {ledger_alignment}")
    print()


def lane_stats(samples: Iterable[LaneSample]) -> Dict[str, Welford]:
    stats = {
        "residual_ns": Welford(),
        "span_dev_ns": Welford(),
        "residual_jump_ns": Welford(),
        "ledger_error_ns": Welford(),
        "phase_error_ns": Welford(),
        "ledger_error_current_ns": Welford(),
        "ledger_error_previous_ns": Welford(),
        "diag_gnss_error_ns": Welford(),
        "diag_dwt_error_cycles": Welford(),
        "counter32_error_ticks": Welford(),
        "dwt_cycles_between_edges": Welford(),
    }
    for s in samples:
        stats["residual_ns"].update(float(s.residual_ns) if s.residual_ns is not None else None)
        stats["span_dev_ns"].update(float(s.span_deviation_ns) if s.span_deviation_ns is not None else None)
        stats["residual_jump_ns"].update(float(s.residual_jump_ns) if s.residual_jump_ns is not None else None)
        stats["ledger_error_ns"].update(float(s.ledger_error_ns) if s.ledger_error_ns is not None else None)
        stats["ledger_error_current_ns"].update(float(s.ledger_error_current_ns) if s.ledger_error_current_ns is not None else None)
        stats["ledger_error_previous_ns"].update(float(s.ledger_error_previous_ns) if s.ledger_error_previous_ns is not None else None)
        stats["phase_error_ns"].update(float(s.phase_error_ns) if s.phase_error_ns is not None else None)
        stats["diag_gnss_error_ns"].update(float(s.diag_gnss_error_ns) if s.diag_gnss_error_ns is not None else None)
        stats["diag_dwt_error_cycles"].update(float(s.diag_dwt_error_cycles) if s.diag_dwt_error_cycles is not None else None)
        stats["counter32_error_ticks"].update(float(s.counter32_error_ticks) if s.counter32_error_ticks is not None else None)
        stats["dwt_cycles_between_edges"].update(
            float(s.dwt_cycles_between_edges) if s.dwt_cycles_between_edges is not None else None
        )
    return stats


def count_tag(samples: Iterable[LaneSample], tag: str) -> int:
    return sum(1 for s in samples if tag in s.tags)


def print_lane_summary(label: str, samples: List[LaneSample]) -> None:
    stats = lane_stats(samples)
    print("-" * 118)
    print(f"{label.upper()} SUMMARY")
    print("-" * 118)
    print(f"residual_ns:             {stats['residual_ns'].summary(3)}")
    print(f"span_dev_ns:             {stats['span_dev_ns'].summary(3)}")
    print(f"residual_jump_ns:        {stats['residual_jump_ns'].summary(3)}")
    print(f"dwt_cycles_between:      {stats['dwt_cycles_between_edges'].summary(3)}")
    print(f"ledger_error_ns:         {stats['ledger_error_ns'].summary(3)}")
    print(f"ledger_err_current:      {stats['ledger_error_current_ns'].summary(3)}")
    print(f"ledger_err_previous:     {stats['ledger_error_previous_ns'].summary(3)}")
    print(f"phase_error_ns:          {stats['phase_error_ns'].summary(3)}")
    print(f"diag_gnss_error_ns:      {stats['diag_gnss_error_ns'].summary(3)}")
    print(f"diag_dwt_error_cycles:   {stats['diag_dwt_error_cycles'].summary(3)}")
    print(f"counter32_error_ticks:   {stats['counter32_error_ticks'].summary(3)}")
    print()
    print("tag counts:")
    for tag in [
        "EVENT_SPAN_SPIKE",
        "RESIDUAL_JUMP",
        "LEDGER_INCOHERENT",
        "COUNTER32_JUMP",
        "DIAG_GNSS_ZERO",
        "DIAG_GNSS_INCOHERENT",
        "DIAG_DWT_INCOHERENT",
        "PHASE_INCOHERENT",
        "WINDOW_MISMATCH",
        "DIAG_ANOMALY",
        "LIKELY_LEDGER_ONLY",
        "LIKELY_COUNTER_OR_EDGE_LOSS",
        "LIKELY_BRIDGE_OR_EVENT_TIME",
        "LIKELY_STALE_DIAG_OR_MEASURE",
        "COMMON_MODE_ROW",
        "COMMON_MODE_JUMP",
        "SINGLE_LANE_SPIKE",
    ]:
        n = count_tag(samples, tag)
        if n:
            print(f"  {tag:<32s} {n:>8,}")
    print()


def interesting_samples(pairs: List[RowPair]) -> List[LaneSample]:
    out: List[LaneSample] = []
    for p in pairs:
        for s in (p.o1, p.o2):
            if s.tags:
                out.append(s)
    return out


def severity_score(s: LaneSample) -> Tuple[int, int]:
    abs_res = abs(s.residual_ns or 0)
    abs_led = abs(s.ledger_error_ns or 0)
    abs_c32 = abs(s.counter32_error_ticks or 0)
    tag_score = 0
    for tag, score in [
        ("COUNTER32_JUMP", 1000),
        ("LEDGER_INCOHERENT", 900),
        ("EVENT_SPAN_SPIKE", 800),
        ("DIAG_GNSS_INCOHERENT", 700),
        ("DIAG_DWT_INCOHERENT", 700),
        ("PHASE_INCOHERENT", 500),
        ("RESIDUAL_JUMP", 400),
        ("WINDOW_MISMATCH", 200),
        ("DIAG_ANOMALY", 200),
        ("DIAG_GNSS_ZERO", 150),
    ]:
        if tag in s.tags:
            tag_score += score
    magnitude = max(abs_res, abs_led, abs_c32)
    return (tag_score, magnitude)


def print_issue_table(samples: List[LaneSample], max_issues: int) -> None:
    samples = sorted(samples, key=severity_score, reverse=True)
    if max_issues > 0:
        samples = samples[:max_issues]

    print("-" * 180)
    print("ISSUE TABLE")
    print("-" * 180)
    if not samples:
        print("No tagged issues.")
        print()
        return

    print(
        f"{'pps':>7s} {'lane':>6s} {'res':>10s} {'jump':>9s} "
        f"{'span_dev':>10s} {'ledgerΔ':>14s} {'ledger_err':>11s} "
        f"{'phase_err':>10s} {'align':>8s} {'c32Δ':>12s} {'c32_err':>9s} "
        f"{'dwt_err':>9s} {'gnss_err':>9s} tags"
    )
    print("-" * 180)

    for s in samples:
        tags = ",".join(s.tags)
        print(
            f"{s.pps:>7,d} {s.lane.upper():>6s} "
            f"{fmt_int(s.residual_ns, 10, signed=True)} "
            f"{fmt_int(s.residual_jump_ns, 9, signed=True)} "
            f"{fmt_int(s.span_deviation_ns, 10, signed=True)} "
            f"{fmt_int(s.ledger_delta_ns, 14, signed=False)} "
            f"{fmt_int(s.ledger_error_ns, 11, signed=True)} "
            f"{fmt_int(s.phase_error_ns, 10, signed=True)} "
            f"{(s.ledger_alignment_used or '---'):>8s} "
            f"{fmt_int(s.counter32_delta_ticks, 12, signed=False)} "
            f"{fmt_int(s.counter32_error_ticks, 9, signed=True)} "
            f"{fmt_int(s.diag_dwt_error_cycles, 9, signed=True)} "
            f"{fmt_int(s.diag_gnss_error_ns, 9, signed=True)} "
            f"{tags}"
        )
    print()


def print_around(pairs: List[RowPair], around: int, radius: int) -> None:
    lo = around - radius
    hi = around + radius
    selected = [p for p in pairs if lo <= p.pps <= hi]
    if not selected:
        print(f"No rows around pps {around} +/- {radius}")
        return

    print("-" * 180)
    print(f"AROUND PPS {around} (+/- {radius})")
    print("-" * 180)
    print(
        f"{'pps':>7s} "
        f"{'o1_res':>9s} {'o1_ledgerΔ':>13s} {'o1_led_err':>10s} {'o1_align':>8s} {'o1_c32err':>9s} {'o1_tags':<34s} "
        f"{'o2_res':>9s} {'o2_ledgerΔ':>13s} {'o2_led_err':>10s} {'o2_align':>8s} {'o2_c32err':>9s} {'o2_tags'}"
    )
    print("-" * 180)

    for p in selected:
        print(
            f"{p.pps:>7,d} "
            f"{fmt_int(p.o1.residual_ns, 9, signed=True)} "
            f"{fmt_int(p.o1.ledger_delta_ns, 13)} "
            f"{fmt_int(p.o1.ledger_error_ns, 10, signed=True)} "
            f"{(p.o1.ledger_alignment_used or '---'):>8s} "
            f"{fmt_int(p.o1.counter32_error_ticks, 9, signed=True)} "
            f"{','.join(p.o1.tags)[:34]:<34s} "
            f"{fmt_int(p.o2.residual_ns, 9, signed=True)} "
            f"{fmt_int(p.o2.ledger_delta_ns, 13)} "
            f"{fmt_int(p.o2.ledger_error_ns, 10, signed=True)} "
            f"{(p.o2.ledger_alignment_used or '---'):>8s} "
            f"{fmt_int(p.o2.counter32_error_ticks, 9, signed=True)} "
            f"{','.join(p.o2.tags)}"
        )
    print()


def print_common_mode_summary(pairs: List[RowPair]) -> None:
    corr_res = RunningCovariance()
    corr_jump = RunningCovariance()
    common = Welford()
    diff = Welford()
    both_spike = 0
    one_spike = 0
    both_counter = 0
    one_counter = 0

    for p in pairs:
        corr_res.update(p.o1.residual_ns, p.o2.residual_ns)
        corr_jump.update(p.o1.residual_jump_ns, p.o2.residual_jump_ns)
        if p.o1.residual_ns is not None and p.o2.residual_ns is not None:
            common.update((p.o1.residual_ns + p.o2.residual_ns) / 2.0)
            diff.update(float(p.o1.residual_ns - p.o2.residual_ns))
        s1 = "EVENT_SPAN_SPIKE" in p.o1.tags
        s2 = "EVENT_SPAN_SPIKE" in p.o2.tags
        c1 = "COUNTER32_JUMP" in p.o1.tags
        c2 = "COUNTER32_JUMP" in p.o2.tags
        if s1 and s2:
            both_spike += 1
        elif s1 or s2:
            one_spike += 1
        if c1 and c2:
            both_counter += 1
        elif c1 or c2:
            one_counter += 1

    print("-" * 118)
    print("COMMON-MODE / SINGLE-LANE SUMMARY")
    print("-" * 118)
    print(f"corr(ocxo1 residual, ocxo2 residual):      {corr_res.corr:+.6f}   n={corr_res.n:,}")
    print(f"corr(ocxo1 residual jump, ocxo2 jump):     {corr_jump.corr:+.6f}   n={corr_jump.n:,}")
    print(f"common residual:                           {common.summary(3)}")
    print(f"differential residual:                     {diff.summary(3)}")
    if common.stddev > 0:
        print(f"differential/common sd ratio:              {diff.stddev / common.stddev:.3f}")
    print(f"both lanes EVENT_SPAN_SPIKE same row:       {both_spike:,}")
    print(f"single-lane EVENT_SPAN_SPIKE rows:          {one_spike:,}")
    print(f"both lanes COUNTER32_JUMP same row:         {both_counter:,}")
    print(f"single-lane COUNTER32_JUMP rows:            {one_counter:,}")
    print()


def print_verdict(pairs: List[RowPair]) -> None:
    samples = [s for p in pairs for s in (p.o1, p.o2)]
    issue_counts: Dict[str, int] = {}
    for s in samples:
        for tag in s.tags:
            issue_counts[tag] = issue_counts.get(tag, 0) + 1

    print("=" * 118)
    print("VERDICT HINTS")
    print("=" * 118)

    def n(tag: str) -> int:
        return issue_counts.get(tag, 0)

    if n("COUNTER32_JUMP"):
        print("* COUNTER32_JUMP present.")
        print("  Synthetic OCXO counter identity did not advance by exactly 10,000,000 ticks.")
        print("  This supports missed/extra cadence accounting or synthetic counter lifecycle trouble.")

    if n("LEDGER_INCOHERENT") and not n("EVENT_SPAN_SPIKE"):
        print("* Ledger incoherence without event-span spikes.")
        print("  Public OCXO ns ledger is disagreeing with residual/phase math.")
        print("  This points toward ledger/base/phase arithmetic rather than physical OCXO edges.")

    if n("EVENT_SPAN_SPIKE"):
        print("* Event-span spikes present.")
        print("  ocxo*_gnss_ns_between_edges itself went implausible.")
        print("  If DWT and diag deltas agree, the badness is upstream of reporting: the event arrived")
        print("  at a bad bridge time or the OCXO cadence/edge path emitted a bad one-second event.")

    if n("COMMON_MODE_ROW") or n("COMMON_MODE_JUMP"):
        print("* Common-mode OCXO anomalies present.")
        print("  Both lanes moved together on at least some rows, which leans toward bridge/reference path.")

    if n("SINGLE_LANE_SPIKE"):
        print("* Single-lane spikes present.")
        print("  One OCXO lane spiked without the other. That leans toward lane-specific counter/electrical behavior.")

    if n("DIAG_GNSS_INCOHERENT") or n("DIAG_DWT_INCOHERENT"):
        print("* Measurement/diag incoherence present.")
        print("  Stored diagnostic event facts disagree with measurement fields. Check publication order/staleness.")

    if not issue_counts:
        print("No tagged issues under current thresholds.")

    print("=" * 118)


def write_csv(path: str, pairs: List[RowPair]) -> None:
    fields = [
        "pps", "lane", "residual_ns", "residual_jump_ns", "span_deviation_ns",
        "between_edges_ns", "ocxo_ns", "ledger_delta_ns", "ledger_expected_delta_ns",
        "ledger_error_ns", "phase_offset_ns", "phase_delta_ns", "phase_error_ns",
        "diag_gnss_ns_at_event", "diag_gnss_delta_ns", "diag_gnss_error_ns",
        "diag_dwt_at_event", "diag_dwt_delta_cycles", "diag_dwt_error_cycles",
        "diag_counter32_at_event", "counter32_delta_ticks", "counter32_error_ticks",
        "window_error_ns", "window_mismatches", "window_mismatch_increment",
        "diag_anomaly_count", "diag_anomaly_increment", "tags",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for p in pairs:
            for s in (p.o1, p.o2):
                writer.writerow({
                    "pps": s.pps,
                    "lane": s.lane,
                    "residual_ns": s.residual_ns,
                    "residual_jump_ns": s.residual_jump_ns,
                    "span_deviation_ns": s.span_deviation_ns,
                    "between_edges_ns": s.between_edges_ns,
                    "ocxo_ns": s.ocxo_ns,
                    "ledger_delta_ns": s.ledger_delta_ns,
                    "ledger_expected_delta_ns": s.ledger_expected_delta_ns,
                    "ledger_error_ns": s.ledger_error_ns,
                    "ledger_error_current_ns": s.ledger_error_current_ns,
                    "ledger_error_previous_ns": s.ledger_error_previous_ns,
                    "ledger_alignment_used": s.ledger_alignment_used,
                    "phase_offset_ns": s.phase_offset_ns,
                    "phase_delta_ns": s.phase_delta_ns,
                    "phase_error_ns": s.phase_error_ns,
                    "diag_gnss_ns_at_event": s.diag_gnss_ns_at_event,
                    "diag_gnss_delta_ns": s.diag_gnss_delta_ns,
                    "diag_gnss_error_ns": s.diag_gnss_error_ns,
                    "diag_dwt_at_event": s.diag_dwt_at_event,
                    "diag_dwt_delta_cycles": s.diag_dwt_delta_cycles,
                    "diag_dwt_error_cycles": s.diag_dwt_error_cycles,
                    "diag_counter32_at_event": s.diag_counter32_at_event,
                    "counter32_delta_ticks": s.counter32_delta_ticks,
                    "counter32_error_ticks": s.counter32_error_ticks,
                    "window_error_ns": s.window_error_ns,
                    "window_mismatches": s.window_mismatches,
                    "window_mismatch_increment": s.window_mismatch_increment,
                    "diag_anomaly_count": s.diag_anomaly_count,
                    "diag_anomaly_increment": s.diag_anomaly_increment,
                    "tags": ",".join(s.tags),
                })


def analyze(args: argparse.Namespace) -> None:
    rows = fetch_timebase(args.campaign)
    if args.limit and args.limit > 0:
        rows = rows[:args.limit]

    if not rows:
        print(f"No TIMEBASE rows found for campaign '{args.campaign}'")
        return

    thresholds = Thresholds(
        spike_ns=args.spike_ns,
        jump_ns=args.jump_ns,
        ledger_err_ns=args.ledger_err_ns,
        counter_tick_err=args.counter_tick_err,
    )

    pairs = build_pairs(rows, thresholds, ledger_alignment=args.ledger_alignment)

    print_campaign_overview(args.campaign, rows, pairs, args.ledger_alignment)
    print_lane_summary("ocxo1", [p.o1 for p in pairs])
    print_lane_summary("ocxo2", [p.o2 for p in pairs])
    print_common_mode_summary(pairs)

    if args.around is not None:
        print_around(pairs, args.around, args.radius)

    print_issue_table(interesting_samples(pairs), args.max_issues)
    print_verdict(pairs)

    if args.csv:
        write_csv(args.csv, pairs)
        print(f"Wrote CSV: {args.csv}")


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ZPNet OCXO count/ledger/residual forensics"
    )
    parser.add_argument("campaign", help="Campaign name")
    parser.add_argument("--limit", type=int, default=0, help="Limit rows read from start of campaign")
    parser.add_argument("--around", type=int, default=None, help="Print context around this PPS count")
    parser.add_argument("--radius", type=int, default=20, help="Context radius for --around")
    parser.add_argument("--spike-ns", type=int, default=DEFAULT_SPIKE_NS, help="Residual magnitude treated as spike")
    parser.add_argument("--jump-ns", type=int, default=DEFAULT_JUMP_NS, help="Residual jump magnitude treated as issue")
    parser.add_argument("--ledger-err-ns", type=int, default=DEFAULT_LEDGER_ERR_NS, help="Ledger consistency error threshold")
    parser.add_argument(
        "--ledger-alignment",
        choices=LEDGER_ALIGNMENT_CHOICES,
        default="previous",
        help=(
            "Which residual should explain public OCXO ledger deltas: "
            "'previous' matches the current firmware publication ordering, "
            "'current' enforces same-row alignment, and 'auto' reports the better of both."
        ),
    )
    parser.add_argument("--counter-tick-err", type=int, default=DEFAULT_COUNTER_TICK_ERR, help="Counter32 tick error threshold")
    parser.add_argument("--max-issues", type=int, default=DEFAULT_MAX_ISSUES, help="Maximum issue rows to print; 0 prints all")
    parser.add_argument("--csv", default=None, help="Optional CSV output path with all lane rows")
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args(sys.argv[1:])
    analyze(args)


if __name__ == "__main__":
    main()
