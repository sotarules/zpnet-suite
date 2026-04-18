"""
OCXO Campaign Analyzer — OCXOs as Measured Peers of VCLOCK.

Context:
  VCLOCK, OCXO1, and OCXO2 are now measured with identical firmware
  machinery (alpha's *_apply_edge() functions are structurally the
  same across all three clocks).  VCLOCK is the GNSS-disciplined
  reference; OCXO1/OCXO2 are the crystals whose drift against that
  reference we care about.

  Because all three clocks publish a symmetric field set, this
  analyzer can show VCLOCK as a peer reference row directly above
  each OCXO table, making OCXO-minus-VCLOCK residuals easy to read.

What this analyzer answers:

  (1) DRIFT
      ocxoN_gnss_ns_between_edges should sit near 1e9.  The derived
      "second deviation" (1e9 - between_edges) is the drift signal.
      Compared against VCLOCK's deviation (which should be ~0), the
      difference is crystal drift — not measurement noise.

  (2) HEALTH
      ocxoN_window_mismatches should stay at 0 unless crystal drift
      exceeds CLOCK_WINDOW_TOLERANCE_NS (500 ns/sec).
      ocxoN_diag_anomaly_count should stay low.

  (3) DAC SERVO EFFORT (THE TEMPEST SIGNAL)
      ocxoN_dac_welford_mean is the servo effort required to keep
      this OCXO GNSS-locked.  Its shrinking stderr (as 1/√n) is what
      allows TEMPEST to detect a ~0.165 ppb shift between altitudes.

  (4) ANOMALIES
      Doubled-second / short-second events, residual jumps, and
      per-fragment edge issues.

Canonical row model per OCXO:
  pps_count, gnss_ns, ocxo_gnss_ns_between_edges, derived_deviation,
  ocxo_dwt_cycles_between_edges, phase_offset_ns, ppb, window_error,
  window_mismatches, dac, dac_welford_mean, dac_welford_stderr,
  anomaly_count.

Usage:
    python -m zpnet.tests.ocxo_campaign_analyzer <campaign_name>
    .zt ocxo_campaign_analyzer Opus42
"""

from __future__ import annotations

import json
import math
import sys
import traceback
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000

DOUBLED_SECOND_THRESHOLD_NS = 1_500_000_000
HALF_SECOND_THRESHOLD_NS = 500_000_000
DELTA_REASONABLE_WARN_NS = 10_000
RESIDUAL_JUMP_WARN_NS = 250
RESIDUAL_JUMP_ALARM_NS = 1_000

MAX_TABLE_ROWS = 120
MAX_ISSUES = 20


# ---------------------------------------------------------------------
# Row model
# ---------------------------------------------------------------------

@dataclass
class ClockRow:
    """
    Uniform row model for VCLOCK, OCXO1, and OCXO2.  Populated from
    whichever field prefix is passed to build_clock_rows().
    """
    pps_count: int
    gnss_ns: int

    # Per-edge measurement
    between_edges_ns: Optional[int]        # *_gnss_ns_between_edges
    deviation_ns: Optional[int]            # = 1e9 - between_edges_ns, or None if between_edges_ns is 0/None
    dwt_cycles_between_edges: Optional[int]
    ns_count_at_edge: Optional[int]        # *_ns_count_at_pps (authored cumulative)
    phase_offset_ns: Optional[int]
    ppb: Optional[float]

    # Window-tolerance accounting
    window_checks: Optional[int]
    window_mismatches: Optional[int]
    window_error_ns: Optional[int]

    # Welford (firmware-side)
    welford_n: Optional[int]
    welford_mean: Optional[float]
    welford_stddev: Optional[float]

    # Event facts (diag)
    diag_dwt_at_event: Optional[int]
    diag_gnss_ns_at_event: Optional[int]
    diag_anomaly_count: Optional[int]
    diag_enabled: Optional[bool]

    # OCXO-only (None for VCLOCK)
    dac: Optional[float]
    dac_welford_n: Optional[int]
    dac_welford_mean: Optional[float]
    dac_welford_stddev: Optional[float]
    dac_welford_stderr: Optional[float]
    servo_adjustments: Optional[int]

    zero_established: Optional[bool]


# ---------------------------------------------------------------------
# Welford
# ---------------------------------------------------------------------

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

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0

    def summary(self, fmt: str = ".3f") -> str:
        if self.n == 0:
            return "no data"
        sd = f"{self.stddev:{fmt}}" if self.n >= 2 else "---"
        se = f"{self.stderr:{fmt}}" if self.n >= 2 else "---"
        return (
            f"n={self.n}  mean={self.mean:+{fmt}}  sd={sd}  se={se}  "
            f"min={self.min_val:{fmt}}  max={self.max_val:{fmt}}"
        )


# ---------------------------------------------------------------------
# Fragment helpers
# ---------------------------------------------------------------------

def _frag(row: Dict[str, Any], key: str, default=None):
    frag = row.get("fragment")
    if not isinstance(frag, dict):
        return default
    return frag.get(key, default)


def _frag_int(row: Dict[str, Any], key: str) -> Optional[int]:
    v = _frag(row, key)
    try:
        return int(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def _frag_float(row: Dict[str, Any], key: str) -> Optional[float]:
    v = _frag(row, key)
    try:
        return float(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def _frag_bool(row: Dict[str, Any], key: str) -> Optional[bool]:
    v = _frag(row, key)
    return bool(v) if v is not None else None


# ---------------------------------------------------------------------
# Extraction
# ---------------------------------------------------------------------

def build_clock_rows(
    rows: List[Dict[str, Any]],
    prefix: str,          # "vclock", "ocxo1", or "ocxo2"
    is_ocxo: bool,
) -> List[ClockRow]:
    """
    Build per-pps rows for a clock.  prefix controls which firmware
    fields are extracted; is_ocxo controls whether DAC/servo fields
    are populated (VCLOCK has no DAC).
    """
    out: List[ClockRow] = []

    for row in rows:
        pps = int(row.get("pps_count") or _frag_int(row, "teensy_pps_count") or 0)

        between_edges_ns = _frag_int(row, f"{prefix}_gnss_ns_between_edges")

        # Derived deviation: 1e9 - between_edges.  Guard against the
        # warmup case where between_edges is 0 (first apply_edge call
        # takes the first-edge branch and publishes 0).
        deviation_ns: Optional[int] = None
        if between_edges_ns is not None and between_edges_ns != 0:
            deviation_ns = NS_PER_SECOND - between_edges_ns

        out.append(
            ClockRow(
                pps_count=pps,
                gnss_ns=_frag_int(row, "gnss_ns") or 0,
                between_edges_ns=between_edges_ns,
                deviation_ns=deviation_ns,
                dwt_cycles_between_edges=_frag_int(row, f"{prefix}_dwt_cycles_between_edges"),
                ns_count_at_edge=_frag_int(row, f"{prefix}_ns_count_at_pps"),
                phase_offset_ns=_frag_int(row, f"{prefix}_phase_offset_ns"),
                ppb=_frag_float(row, f"{prefix}_ppb"),
                window_checks=_frag_int(row, f"{prefix}_window_checks"),
                window_mismatches=_frag_int(row, f"{prefix}_window_mismatches"),
                window_error_ns=_frag_int(row, f"{prefix}_window_error_ns"),
                welford_n=_frag_int(row, f"{prefix}_welford_n"),
                welford_mean=_frag_float(row, f"{prefix}_welford_mean"),
                welford_stddev=_frag_float(row, f"{prefix}_welford_stddev"),
                diag_dwt_at_event=_frag_int(row, f"{prefix}_diag_dwt_at_event"),
                diag_gnss_ns_at_event=_frag_int(row, f"{prefix}_diag_gnss_ns_at_event"),
                diag_anomaly_count=_frag_int(row, f"{prefix}_diag_anomaly_count"),
                diag_enabled=_frag_bool(row, f"{prefix}_diag_enabled"),
                dac=_frag_float(row, f"{prefix}_dac") if is_ocxo else None,
                dac_welford_n=_frag_int(row, f"{prefix}_dac_welford_n") if is_ocxo else None,
                dac_welford_mean=_frag_float(row, f"{prefix}_dac_welford_mean") if is_ocxo else None,
                dac_welford_stddev=_frag_float(row, f"{prefix}_dac_welford_stddev") if is_ocxo else None,
                dac_welford_stderr=_frag_float(row, f"{prefix}_dac_welford_stderr") if is_ocxo else None,
                servo_adjustments=_frag_int(row, f"{prefix}_servo_adjustments") if is_ocxo else None,
                zero_established=_frag_bool(row, f"{prefix}_zero_established"),
            )
        )

    return out


# ---------------------------------------------------------------------
# DB access
# ---------------------------------------------------------------------

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, ts, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        payload["_db_id"] = row["id"]
        payload["_db_ts"] = str(row["ts"])
        result.append(payload)
    return result


def find_recovery_boundaries(rows: List[Dict[str, Any]]) -> Set[int]:
    boundaries: Set[int] = set()
    for i in range(1, len(rows)):
        prev_pps = int(rows[i - 1].get("pps_count") or 0)
        curr_pps = int(rows[i].get("pps_count") or 0)
        if curr_pps > prev_pps + 1:
            boundaries.add(curr_pps)
    return boundaries


# ---------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------

def _i(v: Optional[int], width: int) -> str:
    if v is None:
        return f"{'---':>{width}}"
    return f"{v:>{width},d}"


def _si(v: Optional[int], width: int) -> str:
    if v is None:
        return f"{'---':>{width}}"
    return f"{v:>+{width},d}"


def _f(v: Optional[float], width: int, decimals: int = 3) -> str:
    if v is None:
        return f"{'---':>{width}}"
    return f"{v:>+{width}.{decimals}f}"


def _fu(v: Optional[float], width: int, decimals: int = 3) -> str:
    """Unsigned float (for DAC code values)."""
    if v is None:
        return f"{'---':>{width}}"
    return f"{v:>{width}.{decimals}f}"


# ---------------------------------------------------------------------
# Table printing
# ---------------------------------------------------------------------

def print_vclock_reference_row(vclock_rows: List[ClockRow], at_index: int) -> None:
    """Print one VCLOCK reference row, same column layout as OCXO table (sans DAC)."""
    if at_index >= len(vclock_rows):
        return
    r = vclock_rows[at_index]
    print(
        f"{'VCLOCK':>7s}  "
        f"{_i(r.between_edges_ns, 14)}  "
        f"{_si(r.deviation_ns, 9)}  "
        f"{_i(r.dwt_cycles_between_edges, 13)}  "
        f"{_si(r.phase_offset_ns, 14)}  "
        f"{_f(r.ppb, 9, 3)}  "
        f"{_si(r.window_error_ns, 8)}  "
        f"{_i(r.window_mismatches, 6)}  "
        f"{'---':>12s}  "    # DAC
        f"{'---':>14s}  "    # DAC_MEAN
        f"{'---':>10s}  "    # DAC_SE
        f"{_i(r.diag_anomaly_count, 6)}"
    )


def print_ocxo_table(
    label: str,
    ocxo_rows: List[ClockRow],
    vclock_rows: List[ClockRow],
    max_rows: int = MAX_TABLE_ROWS,
) -> None:
    print("-" * 130)
    print(f"{label} — measured against VCLOCK reference")
    print("-" * 130)

    print(
        f"{'CLOCK':>7s}  "
        f"{'BTW_NS':>14s}  "
        f"{'DEV_NS':>9s}  "
        f"{'BTW_CYC':>13s}  "
        f"{'PH_OFF_NS':>14s}  "
        f"{'PPB':>9s}  "
        f"{'WERR':>8s}  "
        f"{'WMIS':>6s}  "
        f"{'DAC':>12s}  "
        f"{'DAC_MEAN':>14s}  "
        f"{'DAC_SE':>10s}  "
        f"{'ANOM':>6s}"
    )
    print(
        f"{'---':>7s}  "
        f"{'------':>14s}  "
        f"{'------':>9s}  "
        f"{'-------':>13s}  "
        f"{'---------':>14s}  "
        f"{'---':>9s}  "
        f"{'----':>8s}  "
        f"{'----':>6s}  "
        f"{'---':>12s}  "
        f"{'--------':>14s}  "
        f"{'------':>10s}  "
        f"{'----':>6s}"
    )

    # Build a pps_count -> index map for vclock_rows so we can pick the
    # matching reference row for each OCXO pps.
    vclock_by_pps: Dict[int, int] = {}
    for i, vr in enumerate(vclock_rows):
        vclock_by_pps[vr.pps_count] = i

    rows_shown = ocxo_rows[:max_rows]
    for row in rows_shown:
        print(
            f"{f'PPS {row.pps_count}':>7s}  "
            f"{_i(row.between_edges_ns, 14)}  "
            f"{_si(row.deviation_ns, 9)}  "
            f"{_i(row.dwt_cycles_between_edges, 13)}  "
            f"{_si(row.phase_offset_ns, 14)}  "
            f"{_f(row.ppb, 9, 3)}  "
            f"{_si(row.window_error_ns, 8)}  "
            f"{_i(row.window_mismatches, 6)}  "
            f"{_fu(row.dac, 12, 3)}  "
            f"{_f(row.dac_welford_mean, 14, 6)}  "
            f"{_f(row.dac_welford_stderr, 10, 6)}  "
            f"{_i(row.diag_anomaly_count, 6)}"
        )

    # VCLOCK reference at the bottom, from the last pps shown
    if rows_shown:
        last_pps = rows_shown[-1].pps_count
        idx = vclock_by_pps.get(last_pps)
        if idx is not None:
            print(
                f"{'-' * 130}"
            )
            print(f"VCLOCK REFERENCE @ PPS {last_pps}:")
            print_vclock_reference_row(vclock_rows, idx)

    if len(ocxo_rows) > max_rows:
        print(f"... truncated: showing first {max_rows} of {len(ocxo_rows)} rows")
    print()


# ---------------------------------------------------------------------
# Summary & anomaly detection
# ---------------------------------------------------------------------

def analyze_clock(
    label: str,
    data: List[ClockRow],
    vclock_data: Optional[List[ClockRow]],
) -> List[str]:
    anomalies: List[str] = []

    dev_stats = WelfordStats()
    btw_cyc_stats = WelfordStats()
    ppb_stats = WelfordStats()
    phase_stats = WelfordStats()

    # OCXO-only
    dac_stats = WelfordStats()

    edge_issues: List[str] = []
    delta_warns: List[str] = []
    doubled_seconds: List[str] = []
    short_seconds: List[str] = []
    residual_jump_warns: List[str] = []
    residual_jump_alarms: List[str] = []
    window_mismatch_events: List[str] = []
    anomaly_events: List[str] = []

    prev_deviation: Optional[int] = None
    prev_window_mismatches: Optional[int] = None
    prev_anomaly_count: Optional[int] = None

    for row in data:
        # Skip warmup rows where no measurement is established yet.
        if row.between_edges_ns is None or row.between_edges_ns == 0:
            continue

        dev = row.deviation_ns
        if dev is not None:
            dev_stats.update(float(dev))
        if row.dwt_cycles_between_edges is not None and row.dwt_cycles_between_edges != 0:
            btw_cyc_stats.update(float(row.dwt_cycles_between_edges))
        if row.ppb is not None:
            ppb_stats.update(row.ppb)
        if row.phase_offset_ns is not None:
            phase_stats.update(float(row.phase_offset_ns))
        if row.dac is not None:
            dac_stats.update(row.dac)

        # Anomaly detectors
        if row.between_edges_ns >= DOUBLED_SECOND_THRESHOLD_NS:
            doubled_seconds.append(
                f"pps={row.pps_count}: between_edges_ns looks doubled "
                f"({row.between_edges_ns:,})"
            )
        elif row.between_edges_ns <= HALF_SECOND_THRESHOLD_NS:
            short_seconds.append(
                f"pps={row.pps_count}: between_edges_ns implausibly short "
                f"({row.between_edges_ns:,})"
            )
        elif abs(row.between_edges_ns - NS_PER_SECOND) > DELTA_REASONABLE_WARN_NS:
            delta_warns.append(
                f"pps={row.pps_count}: between_edges deviation from 1e9 = "
                f"{row.between_edges_ns - NS_PER_SECOND:+,d} ns"
            )

        # Residual-jump detector — second-to-second change in deviation
        if dev is not None and prev_deviation is not None:
            jump = dev - prev_deviation
            if abs(jump) >= RESIDUAL_JUMP_ALARM_NS:
                residual_jump_alarms.append(
                    f"pps={row.pps_count}: deviation jump {jump:+,d} ns "
                    f"({prev_deviation:+,d} -> {dev:+,d})"
                )
            elif abs(jump) >= RESIDUAL_JUMP_WARN_NS:
                residual_jump_warns.append(
                    f"pps={row.pps_count}: deviation jump {jump:+,d} ns"
                )
        if dev is not None:
            prev_deviation = dev

        # Window-mismatch new occurrences
        if (row.window_mismatches is not None
                and prev_window_mismatches is not None
                and row.window_mismatches > prev_window_mismatches):
            new_mms = row.window_mismatches - prev_window_mismatches
            window_mismatch_events.append(
                f"pps={row.pps_count}: +{new_mms} window mismatch(es) "
                f"(werr={row.window_error_ns:+,d} ns)"
            )
        if row.window_mismatches is not None:
            prev_window_mismatches = row.window_mismatches

        # New anomaly events
        if (row.diag_anomaly_count is not None
                and prev_anomaly_count is not None
                and row.diag_anomaly_count > prev_anomaly_count):
            new_anoms = row.diag_anomaly_count - prev_anomaly_count
            anomaly_events.append(
                f"pps={row.pps_count}: +{new_anoms} diag anomaly event(s)"
            )
        if row.diag_anomaly_count is not None:
            prev_anomaly_count = row.diag_anomaly_count

    # --- Summary print ---

    print(f"{label} SUMMARY")
    print(f"  deviation_ns:      {dev_stats.summary('.3f')}")
    print(f"  btw_dwt_cyc:       {btw_cyc_stats.summary('.3f')}")
    print(f"  ppb:               {ppb_stats.summary('.3f')}")
    print(f"  phase_offset_ns:   {phase_stats.summary('.3f')}")

    # If this is an OCXO, add DAC servo summary and VCLOCK comparison
    if dac_stats.n > 0:
        print(f"  dac (per-row):     {dac_stats.summary('.6f')}")

        last_row = next(
            (r for r in reversed(data) if r.dac_welford_n is not None),
            None,
        )
        if last_row is not None:
            print(
                f"  dac_welford:       "
                f"n={last_row.dac_welford_n}  "
                f"mean={last_row.dac_welford_mean:+.6f}  "
                f"sd={last_row.dac_welford_stddev:.6f}  "
                f"se={last_row.dac_welford_stderr:.6f}"
                f"    <-- TEMPEST signal"
            )

    if vclock_data is not None:
        v_dev = WelfordStats()
        for vr in vclock_data:
            if vr.deviation_ns is not None:
                v_dev.update(float(vr.deviation_ns))
        if v_dev.n > 0:
            diff_mean = dev_stats.mean - v_dev.mean if dev_stats.n > 0 else 0
            print(
                f"  VCLOCK deviation:  {v_dev.summary('.3f')}"
            )
            print(
                f"  DRIFT vs VCLOCK:   "
                f"mean_diff={diff_mean:+.3f} ns/sec  "
                f"(OCXO drift above measurement noise)"
            )

    print()

    # --- Anomaly emission ---

    def emit(title: str, items: List[str], cap: int = MAX_ISSUES) -> None:
        if not items:
            return
        print(f"{title}: {len(items)}")
        for msg in items[:cap]:
            print(f"  {msg}")
        if len(items) > cap:
            print(f"  ... and {len(items) - cap} more")
        print()

    emit("DELTA WARNS", delta_warns)
    emit("DOUBLED-SECOND SUSPECTS", doubled_seconds)
    emit("SHORT-SECOND SUSPECTS", short_seconds)
    emit("RESIDUAL JUMP WARNS", residual_jump_warns)
    emit("RESIDUAL JUMP ALARMS", residual_jump_alarms)
    emit("WINDOW-MISMATCH EVENTS", window_mismatch_events)
    emit("DIAG ANOMALY EVENTS", anomaly_events)

    if doubled_seconds:
        anomalies.append(f"{label}: {len(doubled_seconds)} doubled-second suspects")
    if short_seconds:
        anomalies.append(f"{label}: {len(short_seconds)} short-second suspects")
    if residual_jump_alarms:
        anomalies.append(f"{label}: {len(residual_jump_alarms)} residual jump alarms")
    if window_mismatch_events:
        anomalies.append(f"{label}: {len(window_mismatch_events)} window-mismatch events")
    if anomaly_events:
        anomalies.append(f"{label}: {len(anomaly_events)} diag anomaly events")

    if anomalies:
        print(f"{label} VERDICT: NEEDS ATTENTION")
    else:
        print(f"{label} VERDICT: CLEAN")
    print()

    return anomalies


# ---------------------------------------------------------------------
# Header print
# ---------------------------------------------------------------------

def print_campaign_header(
    rows: List[Dict[str, Any]],
    recovery_boundaries: Set[int],
) -> None:
    pps_counts = [int(r.get("pps_count") or 0) for r in rows]
    pps_min, pps_max = (min(pps_counts), max(pps_counts)) if pps_counts else (0, 0)

    print("=" * 130)
    campaign_name = _frag(rows[0], "campaign") or rows[0].get("campaign", "?")
    print(f"OCXO CAMPAIGN ANALYZER: {campaign_name}")
    print("=" * 130)
    print(f"pps_count range:   {pps_min} -> {pps_max}")
    print(f"records:           {len(rows):,}")
    print(f"recoveries:        {len(recovery_boundaries)}")
    print(f"servo mode:        {_frag(rows[-1], 'calibrate_ocxo', '?')}")
    print()


# ---------------------------------------------------------------------
# Top-level analyze
# ---------------------------------------------------------------------

def analyze(campaign: str) -> None:
    rows = fetch_timebase(campaign)
    print(f"fetched rows: {len(rows)}")

    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    recovery_boundaries = find_recovery_boundaries(rows)
    print_campaign_header(rows, recovery_boundaries)

    vclock_rows = build_clock_rows(rows, "vclock", is_ocxo=False)
    ocxo1_rows = build_clock_rows(rows, "ocxo1", is_ocxo=True)
    ocxo2_rows = build_clock_rows(rows, "ocxo2", is_ocxo=True)

    print_ocxo_table("OCXO1", ocxo1_rows, vclock_rows)
    anomalies_1 = analyze_clock("OCXO1", ocxo1_rows, vclock_rows)

    print_ocxo_table("OCXO2", ocxo2_rows, vclock_rows)
    anomalies_2 = analyze_clock("OCXO2", ocxo2_rows, vclock_rows)

    anomalies = anomalies_1 + anomalies_2

    print("=" * 130)
    if anomalies:
        print(f"VERDICT: ANOMALIES FOUND ({len(anomalies)})")
        for item in anomalies:
            print(f"  * {item}")
    else:
        print("VERDICT: CLEAN")
    print("=" * 130)


def main() -> None:
    print("ocxo_campaign_analyzer starting...")

    if len(sys.argv) < 2:
        print("Usage: ocxo_campaign_analyzer <campaign_name>")
        sys.exit(1)

    campaign = sys.argv[1]
    print(f"campaign={campaign}")
    analyze(campaign)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("ocxo_campaign_analyzer crashed:")
        traceback.print_exc()
        raise