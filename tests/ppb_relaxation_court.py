"""ZPNet ppb_relaxation_court — streamed PPB relaxation causality court.

This report is designed for the repeatable CLOCKS pathology in which
STATS_RESET makes the rolling OCXO PPB populations snap less negative and the
short windows then relax downward again.

The program does not treat a published PPB bucket as its own proof.  For each
TEMPEST row it compares independent producer testimony:

* FW10   — firmware 10-minute Better-Buckets PPB.
* PH10   — 10-minute slope reconstructed from campaign PhaseLedger endpoints.
* D10    — 10-minute grand-ratio reconstructed from Delta-Raw cycle intervals
           admitted by Better-Buckets' own interval-advanced testimony.
* TOTAL  — firmware always-on TAU population for the current statistics epoch.
* DTOT   — independent current-epoch grand-ratio reconstructed from Delta-Raw
           cycle intervals.
* CAMP   — campaign-public clockface ratio, independently checked against the
           published campaign PPB.
* PH1    — one-second PhaseLedger clockface slope.
* DR1    — one-second Delta-Raw frequency ratio.
* D-P    — accumulated Delta-Cycles candidate minus PhaseLedger candidate.
* G10    — trailing candidate-gap slope in ppb; nonzero means the two candidate
           clocks are acquiring different frequency.
* DAC    — Pi-owned actuator target, with servo/dither state retained beside it.
* GDRIFT — TPS1 receiver-clock drift estimate, ppb, with adjacent-row delta.
* GFREQ  — TPS4 VCLK frequency error, ppb, with adjacent-row delta.
* PERR   — TPS4 PPS timing error from the synchronization target, ns, with delta.

The SQL follows the same campaign-centric doctrine as raw_cycles.py: use the
TEMPEST campaign keys and public-count expression, stream with a named cursor,
project only the campaign/stats/control/GNSS subtrees needed by this court, and
never run count(*), an aggregate query, OFFSET, or a whole-history materialization.
Memory is bounded by the server batch plus small 10-minute deques.

Typical use:

    python tests/ppb_relaxation_court.py PPB2
    python tests/ppb_relaxation_court.py PPB2 --every 10
    python tests/ppb_relaxation_court.py PPB2 --skip 3500 --every 1

The default display prints every row for the first 30 statistical updates after
an observed reset_count transition, then one row per minute, plus every reset,
DAC change, chronology problem, or candidate discontinuity.  Every input row is
still processed regardless of display decimation.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Iterator, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


CAMPAIGN_TYPE = "TEMPEST"
CAMPAIGN_SCHEMA = "TEMPEST_FRAGMENT_V1"
LANES: Tuple[str, ...] = ("OCXO1", "OCXO2")
LANE_KEYS = {"OCXO1": "ocxo1", "OCXO2": "ocxo2"}
NS_PER_SECOND = 1_000_000_000.0
ROLLING_SECONDS = 600
DEFAULT_BATCH_SIZE = 64
DEFAULT_DISPLAY_EVERY = 60
DEFAULT_RESET_DETAIL_ROWS = 30
DEFAULT_CANDIDATE_STEP_GATE_NS = 5.0
DEFAULT_DAC_CHANGE_EPSILON = 0.000001
CAMPAIGN_PPB_TOLERANCE = 0.002
TOTAL_RECON_TOLERANCE = 0.020
FW10_PHASE_TOLERANCE = 0.020

PPS_COUNT_SQL = """
NULLIF(
    payload #>> '{campaign,public_count}',
    ''
)::bigint
"""


# -----------------------------------------------------------------------------
# Strict canonical parsing
# -----------------------------------------------------------------------------


def req_dict(value: Any, where: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{where} must be an object; got {value!r}")
    return value


def req_int(value: Any, where: str, *, minimum: Optional[int] = None) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ValueError(f"{where} must be an integer; got {value!r}")
    try:
        out = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{where} must be an integer; got {value!r}") from exc
    if minimum is not None and out < minimum:
        raise ValueError(f"{where} must be >= {minimum}; got {out}")
    return out


def req_float(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"{where} must be numeric; got {value!r}")
    try:
        out = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{where} must be numeric; got {value!r}") from exc
    if not math.isfinite(out):
        raise ValueError(f"{where} must be finite; got {value!r}")
    return out


def req_bool(value: Any, where: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1"}:
            return True
        if normalized in {"false", "0"}:
            return False
    raise ValueError(f"{where} must be boolean; got {value!r}")


def opt_float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return out if math.isfinite(out) else None


def opt_int(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def path(obj: Dict[str, Any], dotted: str) -> Any:
    cur: Any = obj
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


# -----------------------------------------------------------------------------
# Database stream
# -----------------------------------------------------------------------------


def _assert_campaign_indexed(cur: Any, campaign: str) -> None:
    """Existence probe only; never pay a full-campaign aggregate preflight."""
    cur.execute(
        f"""
        SELECT id
        FROM campaign_detail
        WHERE campaign_type = %s
          AND campaign = %s
          AND payload #>> '{{campaign,schema}}' = %s
          AND {PPS_COUNT_SQL} IS NULL
        ORDER BY id ASC
        LIMIT 1
        """,
        (CAMPAIGN_TYPE, campaign, CAMPAIGN_SCHEMA),
    )
    row = cur.fetchone()
    if row:
        raise RuntimeError(
            f"TEMPEST campaign {campaign!r} has decorated row id={int(row['id'])} "
            "without campaign.public_count"
        )


def iter_rows(
    campaign: str,
    *,
    skip: int,
    limit: int,
    batch_size: int,
) -> Iterator[
    Tuple[int, int, Dict[str, Any], Dict[str, Any], Dict[str, Any], Dict[str, Any]]
]:
    """Stream only the canonical testimony required by the relaxation court."""
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        check = conn.cursor()
        _assert_campaign_indexed(check, campaign)
        check.close()

        sql = f"""
            SELECT
                id,
                {PPS_COUNT_SQL} AS pps_count,
                payload -> 'campaign' AS campaign_payload,
                payload #> '{{clocks,stats}}' AS stats_payload,
                payload #> '{{clocks,control}}' AS control_payload,
                payload -> 'gnss' AS gnss_payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND payload #>> '{{campaign,schema}}' = %s
              AND {PPS_COUNT_SQL} IS NOT NULL
        """
        params: list[Any] = [CAMPAIGN_TYPE, campaign, CAMPAIGN_SCHEMA]
        if skip > 0:
            sql += f" AND {PPS_COUNT_SQL} > %s"
            params.append(skip)
        sql += f" ORDER BY {PPS_COUNT_SQL} ASC, id ASC"
        if limit > 0:
            sql += " LIMIT %s"
            params.append(limit)

        cur = conn.cursor(name="ppb_relaxation_court_stream")
        cur.itersize = batch_size
        cur.execute(sql, tuple(params))

        for row in cur:
            campaign_payload = row["campaign_payload"]
            stats_payload = row["stats_payload"]
            control_payload = row["control_payload"]
            gnss_payload = row["gnss_payload"]
            if isinstance(campaign_payload, str):
                campaign_payload = json.loads(campaign_payload)
            if isinstance(stats_payload, str):
                stats_payload = json.loads(stats_payload)
            if isinstance(control_payload, str):
                control_payload = json.loads(control_payload)
            if isinstance(gnss_payload, str):
                gnss_payload = json.loads(gnss_payload)

            campaign_obj = req_dict(
                campaign_payload, f"campaign_detail id={row['id']} campaign"
            )
            stats_obj = req_dict(
                stats_payload, f"campaign_detail id={row['id']} clocks.stats"
            )
            control_obj = req_dict(
                control_payload, f"campaign_detail id={row['id']} clocks.control"
            )
            if gnss_payload is None:
                gnss_obj: Dict[str, Any] = {}
            else:
                gnss_obj = req_dict(
                    gnss_payload, f"campaign_detail id={row['id']} gnss"
                )

            if campaign_obj.get("schema") != CAMPAIGN_SCHEMA:
                raise ValueError(
                    f"campaign_detail id={row['id']} campaign schema="
                    f"{campaign_obj.get('schema')!r}, expected {CAMPAIGN_SCHEMA!r}"
                )

            db_count = req_int(row["pps_count"], f"campaign_detail id={row['id']} pps_count", minimum=1)
            payload_count = req_int(
                campaign_obj.get("public_count"),
                f"campaign_detail id={row['id']} campaign.public_count",
                minimum=1,
            )
            if payload_count != db_count:
                raise ValueError(
                    "campaign_detail relational/payload public-count mismatch: "
                    f"id={row['id']} db={db_count} payload={payload_count}"
                )

            yield int(row["id"]), db_count, campaign_obj, stats_obj, control_obj, gnss_obj


# -----------------------------------------------------------------------------
# Scientific models
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class Candidate:
    available: bool
    continuity_valid: bool
    status: str
    last_public_count: int
    ns: int
    fractional_ns: float

    @property
    def exact_ns(self) -> float:
        return float(self.ns) + self.fractional_ns


@dataclass(frozen=True)
class DeltaRaw:
    valid: bool
    science_worthy: bool
    antecedents_complete: bool
    reference_cycles: int
    clock_cycles: int

    def interval_ppb(self) -> Optional[float]:
        if not self.valid:
            return None
        if self.reference_cycles <= 0 or self.clock_cycles <= 0:
            raise ValueError(
                "valid Delta-Raw interval has non-positive reference/clock cycles"
            )
        return (
            float(self.reference_cycles) / float(self.clock_cycles) - 1.0
        ) * 1.0e9

    def clock_interval_ns_exact(self) -> Optional[float]:
        if not self.valid:
            return None
        return (
            float(self.clock_cycles) * NS_PER_SECOND / float(self.reference_cycles)
        )


@dataclass(frozen=True)
class GnssWitness:
    clock_drift_ppb: float
    drift_delta_ppb: Optional[float]
    freq_error_ppb: float
    freq_error_delta_ppb: Optional[float]
    pps_timing_error_ns: float
    pps_timing_error_delta_ns: Optional[float]
    freq_mode_name: str
    phase_skip: int
    alarm: int
    status: int


@dataclass(frozen=True)
class LaneRow:
    name: str
    key: str
    phase: Candidate
    delta: Candidate
    reported_gap_ns: int
    raw: DeltaRaw
    campaign_clock_ns: int
    campaign_ppb: float
    fw_10m_ppb: Optional[float]
    fw_60m_ppb: Optional[float]
    fw_total_ppb: Optional[float]
    welford_n: int
    dac_target: float
    dac_hw_code: int
    readback_valid: bool
    readback_code: int
    servo_last_step: float
    servo_last_residual: float
    servo_adjustments: int


@dataclass(frozen=True)
class CourtRow:
    db_id: int
    count: int
    previous_count: Optional[int]
    gap: bool
    reset_count: int
    update_count: int
    rolling_sequence: int
    bucket_10m_valid: bool
    bucket_10m_n: int
    endpoint_admitted: bool
    interval_advanced: bool
    science_eligible: bool
    timeline_valid: bool
    disposition: str
    campaign_gnss_ns: int
    servo_mode: str
    servo_active: bool
    dither_enabled: bool
    gnss: Optional[GnssWitness]
    gnss_status: str
    lanes: Dict[str, LaneRow]


@dataclass
class RawGrandRatio:
    sample_count: int = 0
    clock_ns_exact_sum: float = 0.0

    def reset(self) -> None:
        self.sample_count = 0
        self.clock_ns_exact_sum = 0.0

    def add(self, raw: DeltaRaw) -> None:
        clock_ns = raw.clock_interval_ns_exact()
        if clock_ns is None:
            raise ValueError("cannot add invalid Delta-Raw interval to grand ratio")
        self.sample_count += 1
        self.clock_ns_exact_sum += clock_ns

    def ppb(self) -> Optional[float]:
        if self.sample_count == 0 or self.clock_ns_exact_sum <= 0.0:
            return None
        reference_ns = float(self.sample_count) * NS_PER_SECOND
        return (reference_ns / self.clock_ns_exact_sum - 1.0) * 1.0e9


@dataclass(frozen=True)
class RawWindowSample:
    rolling_sequence: int
    clock_ns_exact: float


@dataclass
class RawWindow:
    samples: Deque[RawWindowSample] = field(default_factory=deque)
    clock_ns_exact_sum: float = 0.0

    def reset(self) -> None:
        self.samples.clear()
        self.clock_ns_exact_sum = 0.0

    def age_to(self, current_sequence: int) -> None:
        target = current_sequence - ROLLING_SECONDS
        while self.samples and self.samples[0].rolling_sequence <= target:
            old = self.samples.popleft()
            self.clock_ns_exact_sum -= old.clock_ns_exact

    def add(self, rolling_sequence: int, raw: DeltaRaw) -> None:
        clock_ns = raw.clock_interval_ns_exact()
        if clock_ns is None:
            raise ValueError("cannot add invalid Delta-Raw interval to rolling court")
        if self.samples and rolling_sequence <= self.samples[-1].rolling_sequence:
            raise ValueError(
                "Delta-Raw rolling sequence did not advance strictly: "
                f"previous={self.samples[-1].rolling_sequence} current={rolling_sequence}"
            )
        self.samples.append(RawWindowSample(rolling_sequence, clock_ns))
        self.clock_ns_exact_sum += clock_ns
        self.age_to(rolling_sequence)

    def ppb(self, current_sequence: int) -> Optional[float]:
        self.age_to(current_sequence)
        if not self.samples or self.clock_ns_exact_sum <= 0.0:
            return None
        reference_ns = float(len(self.samples)) * NS_PER_SECOND
        return (reference_ns / self.clock_ns_exact_sum - 1.0) * 1.0e9


@dataclass(frozen=True)
class PhaseEndpoint:
    rolling_sequence: int
    ns: int


@dataclass
class PhaseWindow:
    endpoints: Deque[PhaseEndpoint] = field(default_factory=deque)

    def reset(self) -> None:
        self.endpoints.clear()

    def add(self, rolling_sequence: int, ns: int) -> None:
        if self.endpoints and rolling_sequence <= self.endpoints[-1].rolling_sequence:
            raise ValueError(
                "PhaseLedger rolling sequence did not advance strictly: "
                f"previous={self.endpoints[-1].rolling_sequence} current={rolling_sequence}"
            )
        self.endpoints.append(PhaseEndpoint(rolling_sequence, ns))
        target = rolling_sequence - ROLLING_SECONDS
        while len(self.endpoints) >= 2 and self.endpoints[0].rolling_sequence < target:
            self.endpoints.popleft()

    def ppb(self) -> Optional[float]:
        if len(self.endpoints) < 2:
            return None
        current = self.endpoints[-1]
        target = current.rolling_sequence - ROLLING_SECONDS
        anchor: Optional[PhaseEndpoint] = None
        for endpoint in self.endpoints:
            if endpoint.rolling_sequence >= target and endpoint.rolling_sequence < current.rolling_sequence:
                anchor = endpoint
                break
        if anchor is None:
            return None
        seconds = current.rolling_sequence - anchor.rolling_sequence
        if seconds <= 0:
            raise ValueError("PhaseLedger rolling court has non-positive endpoint span")
        reference_ns = float(seconds) * NS_PER_SECOND
        clock_ns = float(current.ns - anchor.ns)
        return (clock_ns / reference_ns - 1.0) * 1.0e9


@dataclass(frozen=True)
class GapPoint:
    count: int
    gap_ns_exact: float


@dataclass
class GapWindow:
    points: Deque[GapPoint] = field(default_factory=deque)

    def reset(self) -> None:
        self.points.clear()

    def add(self, count: int, gap_ns_exact: float) -> None:
        self.points.append(GapPoint(count, gap_ns_exact))
        target = count - ROLLING_SECONDS
        while len(self.points) >= 2 and self.points[0].count < target:
            self.points.popleft()

    def slope_ppb(self) -> Optional[float]:
        if len(self.points) < 2:
            return None
        first = self.points[0]
        last = self.points[-1]
        seconds = last.count - first.count
        if seconds <= 0:
            return None
        # gap is nanoseconds and time is seconds: ns/s is numerically ppb.
        return (last.gap_ns_exact - first.gap_ns_exact) / float(seconds)


@dataclass
class LaneState:
    previous_phase: Optional[Candidate] = None
    previous_delta: Optional[Candidate] = None
    previous_gap_ns_exact: Optional[float] = None
    previous_dac_target: Optional[float] = None
    raw_total: RawGrandRatio = field(default_factory=RawGrandRatio)
    raw_window: RawWindow = field(default_factory=RawWindow)
    phase_window: PhaseWindow = field(default_factory=PhaseWindow)
    gap_window: GapWindow = field(default_factory=GapWindow)
    first_60_raw: RawGrandRatio = field(default_factory=RawGrandRatio)
    last_600_raw: Deque[float] = field(default_factory=lambda: deque(maxlen=600))

    def reset_stats_epoch(self) -> None:
        self.raw_total.reset()
        self.raw_window.reset()
        self.phase_window.reset()
        self.first_60_raw.reset()
        self.last_600_raw.clear()
        # Candidate and gap ancestry are campaign-lifetime testimony and
        # intentionally survive STATS_RESET.


@dataclass
class EpochSummary:
    reset_count: int
    start_pps: int
    end_pps: int
    complete_from_birth: bool
    rows: int = 0
    first_fw10: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    last_fw10: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    first_total: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    last_total: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    first_camp: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    last_camp: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    first_gap: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    last_gap: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    first_dac: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    last_dac: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    first_60_raw_ppb: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    last_600_raw_ppb: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )


# -----------------------------------------------------------------------------
# Row parsing and courts
# -----------------------------------------------------------------------------


def parse_gnss_witness(
    gnss: Dict[str, Any],
    previous: Optional[GnssWitness],
    *,
    adjacent: bool,
) -> Tuple[Optional[GnssWitness], str]:
    if not gnss:
        return None, "MISSING"

    clock_obj = gnss.get("clock")
    discipline_obj = gnss.get("discipline")
    if clock_obj is None or discipline_obj is None:
        missing = []
        if clock_obj is None:
            missing.append("CLOCK")
        if discipline_obj is None:
            missing.append("DISCIPLINE")
        return None, "+".join(missing) + "_MISSING"

    clock = req_dict(clock_obj, "gnss.clock")
    discipline = req_dict(discipline_obj, "gnss.discipline")

    drift = req_float(clock.get("drift_ppb"), "gnss.clock.drift_ppb")
    if "clock_drift_ppb" in gnss:
        alias = req_float(gnss.get("clock_drift_ppb"), "gnss.clock_drift_ppb")
        if abs(alias - drift) > 1.0e-9:
            raise ValueError(
                "GNSS drift alias disagreement: "
                f"gnss.clock.drift_ppb={drift} gnss.clock_drift_ppb={alias}"
            )

    freq_error = req_float(
        discipline.get("freq_error_ppb"), "gnss.discipline.freq_error_ppb"
    )
    pps_error = req_float(
        discipline.get("pps_timing_error_ns"),
        "gnss.discipline.pps_timing_error_ns",
    )
    freq_mode_name = str(discipline.get("freq_mode_name") or "").upper()
    if not freq_mode_name:
        raise ValueError("gnss.discipline.freq_mode_name is missing")

    drift_delta = None
    freq_delta = None
    pps_delta = None
    if adjacent and previous is not None:
        drift_delta = drift - previous.clock_drift_ppb
        freq_delta = freq_error - previous.freq_error_ppb
        pps_delta = pps_error - previous.pps_timing_error_ns

    return GnssWitness(
        clock_drift_ppb=drift,
        drift_delta_ppb=drift_delta,
        freq_error_ppb=freq_error,
        freq_error_delta_ppb=freq_delta,
        pps_timing_error_ns=pps_error,
        pps_timing_error_delta_ns=pps_delta,
        freq_mode_name=freq_mode_name,
        phase_skip=req_int(discipline.get("phase_skip"), "gnss.discipline.phase_skip"),
        alarm=req_int(discipline.get("alarm"), "gnss.discipline.alarm"),
        status=req_int(discipline.get("status"), "gnss.discipline.status"),
    ), "OK"


def parse_candidate(obj: Any, where: str, count: int) -> Candidate:
    src = req_dict(obj, where)
    available = req_bool(src.get("available"), f"{where}.available")
    continuity = req_bool(src.get("continuity_valid"), f"{where}.continuity_valid")
    status = str(src.get("status") or "").upper()
    if not status:
        raise ValueError(f"{where}.status is missing")
    last_count = req_int(src.get("last_public_count"), f"{where}.last_public_count", minimum=0)
    ns = req_int(src.get("ns"), f"{where}.ns", minimum=0)
    fractional = req_float(src.get("fractional_ns"), f"{where}.fractional_ns")
    if available and last_count != count:
        raise ValueError(
            f"{where} claims available at public_count={count} but last_public_count={last_count}"
        )
    return Candidate(available, continuity, status, last_count, ns, fractional)


def parse_delta_raw(obj: Any, where: str) -> DeltaRaw:
    src = req_dict(obj, where)
    valid = req_bool(src.get("delta_raw_valid"), f"{where}.delta_raw_valid")
    science_worthy = req_bool(src.get("science_worthy"), f"{where}.science_worthy")
    antecedents = req_bool(src.get("antecedents_complete"), f"{where}.antecedents_complete")
    reference = req_int(
        src.get("delta_raw_reference_interval_cycles"),
        f"{where}.delta_raw_reference_interval_cycles",
        minimum=0,
    )
    clock = req_int(
        src.get("delta_raw_clock_interval_cycles"),
        f"{where}.delta_raw_clock_interval_cycles",
        minimum=0,
    )
    if valid and (reference == 0 or clock == 0):
        raise ValueError(f"{where}: Delta-Raw valid but interval cycles are zero")
    return DeltaRaw(valid, science_worthy, antecedents, reference, clock)


def bucket_value(stats: Dict[str, Any], lane_key: str, key: str) -> Optional[float]:
    value = path(stats, f"{lane_key}.ppb_buckets.{key}")
    return opt_float(value)


def parse_lane(
    name: str,
    campaign: Dict[str, Any],
    stats: Dict[str, Any],
    control: Dict[str, Any],
    *,
    count: int,
    campaign_gnss_ns: int,
) -> LaneRow:
    key = LANE_KEYS[name]
    campaign_lane = req_dict(campaign.get(key), f"campaign.{key}")
    candidates = req_dict(campaign_lane.get("clock_candidates"), f"campaign.{key}.clock_candidates")
    phase = parse_candidate(candidates.get("phaseledger"), f"campaign.{key}.clock_candidates.phaseledger", count)
    delta = parse_candidate(candidates.get("delta_cycles"), f"campaign.{key}.clock_candidates.delta_cycles", count)
    reported_gap = req_int(
        candidates.get("delta_cycles_minus_phaseledger_ns"),
        f"campaign.{key}.clock_candidates.delta_cycles_minus_phaseledger_ns",
    )
    if phase.available and delta.available:
        computed_reported_species = delta.ns - phase.ns
        if reported_gap != computed_reported_species:
            raise ValueError(
                f"campaign.{key} candidate gap field mismatch at pps={count}: "
                f"reported={reported_gap} computed={computed_reported_species}"
            )

    raw = parse_delta_raw(campaign_lane.get("science"), f"campaign.{key}.science")

    clockfaces = req_dict(campaign.get("clockfaces"), "campaign.clockfaces")
    campaign_clock_ns = req_int(
        clockfaces.get(f"{key}_ns"), f"campaign.clockfaces.{key}_ns", minimum=1
    )
    published_source = str(candidates.get("published_source") or "").upper()
    if published_source != "COUNTERLEDGER_PHASELEDGER":
        raise ValueError(
            f"campaign.{key}.clock_candidates.published_source={published_source!r}; "
            "relaxation court requires COUNTERLEDGER_PHASELEDGER"
        )
    if phase.available and campaign_clock_ns != phase.ns:
        raise ValueError(
            f"campaign {key} published clockface != PhaseLedger at pps={count}: "
            f"clockface={campaign_clock_ns} phase={phase.ns}"
        )

    campaign_stats = req_dict(campaign.get("stats"), "campaign.stats")
    campaign_ppb_map = req_dict(campaign_stats.get("ppb"), "campaign.stats.ppb")
    campaign_ppb = req_float(campaign_ppb_map.get(key), f"campaign.stats.ppb.{key}")
    computed_campaign_ppb = (
        float(campaign_clock_ns) / float(campaign_gnss_ns) - 1.0
    ) * 1.0e9
    if abs(campaign_ppb - computed_campaign_ppb) > CAMPAIGN_PPB_TOLERANCE:
        raise ValueError(
            f"campaign {key} PPB clockface closure failed at pps={count}: "
            f"published={campaign_ppb:.9f} computed={computed_campaign_ppb:.9f}"
        )

    stats_lane = req_dict(stats.get(key), f"clocks.stats.{key}")
    welford = req_dict(stats_lane.get("welford"), f"clocks.stats.{key}.welford")
    welford_n = req_int(welford.get("n"), f"clocks.stats.{key}.welford.n", minimum=0)

    control_lane = req_dict(control.get(key), f"clocks.control.{key}")
    servo = req_dict(control_lane.get("servo"), f"clocks.control.{key}.servo")

    return LaneRow(
        name=name,
        key=key,
        phase=phase,
        delta=delta,
        reported_gap_ns=reported_gap,
        raw=raw,
        campaign_clock_ns=campaign_clock_ns,
        campaign_ppb=campaign_ppb,
        fw_10m_ppb=bucket_value(stats, key, "10_min"),
        fw_60m_ppb=bucket_value(stats, key, "60_min"),
        fw_total_ppb=bucket_value(stats, key, "total"),
        welford_n=welford_n,
        dac_target=req_float(control_lane.get("target_code"), f"clocks.control.{key}.target_code"),
        dac_hw_code=req_int(control_lane.get("hw_code"), f"clocks.control.{key}.hw_code", minimum=0),
        readback_valid=req_bool(control_lane.get("readback_valid"), f"clocks.control.{key}.readback_valid"),
        readback_code=req_int(control_lane.get("readback_code"), f"clocks.control.{key}.readback_code", minimum=0),
        servo_last_step=req_float(servo.get("last_step"), f"clocks.control.{key}.servo.last_step"),
        servo_last_residual=req_float(servo.get("last_residual"), f"clocks.control.{key}.servo.last_residual"),
        servo_adjustments=req_int(servo.get("adjustments"), f"clocks.control.{key}.servo.adjustments", minimum=0),
    )


def build_row(
    db_id: int,
    count: int,
    campaign: Dict[str, Any],
    stats: Dict[str, Any],
    control: Dict[str, Any],
    gnss_obj: Dict[str, Any],
    previous_count: Optional[int],
    previous_gnss: Optional[GnssWitness],
) -> CourtRow:
    previous_delta = None if previous_count is None else count - previous_count
    gap = previous_delta is not None and previous_delta != 1
    gnss, gnss_status = parse_gnss_witness(
        gnss_obj, previous_gnss, adjacent=not gap
    )

    status = req_dict(campaign.get("status"), "campaign.status")
    disposition = req_dict(campaign.get("disposition"), "campaign.disposition")
    timeline_valid = req_bool(status.get("timeline_valid"), "campaign.status.timeline_valid")
    science_eligible = req_bool(
        disposition.get("science_eligible"), "campaign.disposition.science_eligible"
    )
    disposition_name = str(disposition.get("status") or "").upper()
    if not disposition_name:
        raise ValueError("campaign.disposition.status is missing")

    reset_count = req_int(stats.get("reset_count"), "clocks.stats.reset_count", minimum=0)
    update_count = req_int(stats.get("update_count"), "clocks.stats.update_count", minimum=0)
    rolling_sequence = req_int(
        stats.get("rolling_ppb_current_sequence"),
        "clocks.stats.rolling_ppb_current_sequence",
        minimum=0,
    )
    checkpoint = req_dict(
        stats.get("rolling_ppb_checkpoint"),
        "clocks.stats.rolling_ppb_checkpoint",
    )
    proof_10m = req_dict(
        checkpoint.get("10_min"),
        "clocks.stats.rolling_ppb_checkpoint.10_min",
    )
    bucket_10m_valid = req_bool(
        proof_10m.get("valid"),
        "clocks.stats.rolling_ppb_checkpoint.10_min.valid",
    )
    bucket_10m_n = req_int(
        proof_10m.get("sample_count"),
        "clocks.stats.rolling_ppb_checkpoint.10_min.sample_count",
        minimum=0,
    )
    if bucket_10m_valid and bucket_10m_n == 0:
        raise ValueError(
            f"pps={count}: 10-minute proof valid with zero sample_count"
        )

    endpoint_admitted = req_bool(
        stats.get("rolling_ppb_endpoint_admitted"),
        "clocks.stats.rolling_ppb_endpoint_admitted",
    )
    interval_advanced = req_bool(
        stats.get("rolling_ppb_interval_advanced"),
        "clocks.stats.rolling_ppb_interval_advanced",
    )
    if interval_advanced and not endpoint_admitted:
        raise ValueError(
            f"pps={count}: rolling_ppb_interval_advanced without endpoint_admitted"
        )

    clockfaces = req_dict(campaign.get("clockfaces"), "campaign.clockfaces")
    campaign_gnss_ns = req_int(
        clockfaces.get("gnss_ns"), "campaign.clockfaces.gnss_ns", minimum=1
    )

    servo_mode = str(control.get("servo_mode") or "").upper()
    if servo_mode not in {"OFF", "TOTAL", "CAMP", "10-MIN"}:
        raise ValueError(f"clocks.control.servo_mode={servo_mode!r} is invalid")
    servo_active = req_bool(control.get("servo_active"), "clocks.control.servo_active")
    dither_enabled = req_bool(
        control.get("dither_operator_enabled"),
        "clocks.control.dither_operator_enabled",
    )

    lanes = {
        name: parse_lane(
            name,
            campaign,
            stats,
            control,
            count=count,
            campaign_gnss_ns=campaign_gnss_ns,
        )
        for name in LANES
    }
    for name, lane in lanes.items():
        if bucket_10m_valid != (lane.fw_10m_ppb is not None):
            raise ValueError(
                f"pps={count} {name}: 10-minute proof/value availability mismatch: "
                f"proof_valid={bucket_10m_valid} value={lane.fw_10m_ppb!r}"
            )

    return CourtRow(
        db_id=db_id,
        count=count,
        previous_count=previous_count,
        gap=gap,
        reset_count=reset_count,
        update_count=update_count,
        rolling_sequence=rolling_sequence,
        bucket_10m_valid=bucket_10m_valid,
        bucket_10m_n=bucket_10m_n,
        endpoint_admitted=endpoint_admitted,
        interval_advanced=interval_advanced,
        science_eligible=science_eligible,
        timeline_valid=timeline_valid,
        disposition=disposition_name,
        campaign_gnss_ns=campaign_gnss_ns,
        servo_mode=servo_mode,
        servo_active=servo_active,
        dither_enabled=dither_enabled,
        gnss=gnss,
        gnss_status=gnss_status,
        lanes=lanes,
    )


# -----------------------------------------------------------------------------
# Court state / derived witnesses
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class LaneDerived:
    phase_1s_ppb: Optional[float]
    delta_candidate_1s_ppb: Optional[float]
    delta_raw_1s_ppb: Optional[float]
    phase_10m_ppb: Optional[float]
    delta_raw_10m_ppb: Optional[float]
    delta_raw_total_ppb: Optional[float]
    candidate_gap_ns_exact: Optional[float]
    candidate_gap_step_ns: Optional[float]
    candidate_gap_10m_slope_ppb: Optional[float]
    dac_changed: bool
    notes: Tuple[str, ...]


@dataclass
class CourtState:
    lanes: Dict[str, LaneState] = field(
        default_factory=lambda: {name: LaneState() for name in LANES}
    )
    reset_count: Optional[int] = None
    epoch_start_pps: Optional[int] = None
    epoch_observed_floor_sequence: Optional[int] = None
    window_observation_floor_sequence: Optional[int] = None
    epoch_birth_observed: bool = False
    epoch_total_complete: bool = False
    epoch: Optional[EpochSummary] = None
    summaries: list[EpochSummary] = field(default_factory=list)

    def _finalize_epoch(self) -> None:
        if self.epoch is None:
            return
        for name, lane in self.lanes.items():
            self.epoch.first_60_raw_ppb[name] = lane.first_60_raw.ppb()
            self.epoch.last_600_raw_ppb[name] = last_window_ppb(lane)
        self.summaries.append(self.epoch)
        self.epoch = None

    def _install_epoch(self, row: CourtRow) -> None:
        self.reset_count = row.reset_count
        self.epoch_start_pps = row.count
        self.epoch_observed_floor_sequence = row.rolling_sequence
        self.window_observation_floor_sequence = row.rolling_sequence
        self.epoch_birth_observed = row.rolling_sequence <= 1 and row.update_count <= 1
        self.epoch_total_complete = self.epoch_birth_observed
        self.epoch = EpochSummary(
            row.reset_count, row.count, row.count, self.epoch_birth_observed
        )
        for lane in self.lanes.values():
            lane.reset_stats_epoch()

    def begin_epoch(self, row: CourtRow) -> Optional[str]:
        if self.reset_count is None:
            self._install_epoch(row)
            return "EPOCH_START"
        if row.reset_count == self.reset_count:
            return None
        prior = self.reset_count
        self._finalize_epoch()
        self._install_epoch(row)
        return f"STATS_RESET {prior}->{row.reset_count}"

    def note_observation_gap(self, row: CourtRow) -> None:
        # Missing durable campaign testimony cannot be replayed into an
        # independent court.  Firmware may still own exact Better-Buckets
        # custody, but this DB-stream reconstruction becomes unavailable until
        # the unseen horizon ages out.  TOTAL reconstruction remains incomplete
        # for the rest of this stats epoch because unseen accepted intervals may
        # remain in the firmware lifetime population.
        self.window_observation_floor_sequence = row.rolling_sequence
        self.epoch_total_complete = False
        for lane in self.lanes.values():
            lane.raw_window.reset()
            lane.phase_window.reset()
            lane.gap_window.reset()

    def rolling_window_complete(self, row: CourtRow) -> bool:
        if self.window_observation_floor_sequence is None:
            return False
        target = max(1, row.rolling_sequence - ROLLING_SECONDS)
        return self.window_observation_floor_sequence <= target

    def finish(self) -> None:
        self._finalize_epoch()


def candidate_interval_ppb(
    current: Candidate,
    previous: Optional[Candidate],
    *,
    adjacent: bool,
) -> Optional[float]:
    if not adjacent or previous is None:
        return None
    if not current.available or not previous.available:
        return None
    if not current.continuity_valid or not previous.continuity_valid:
        return None
    return (current.exact_ns - previous.exact_ns) - NS_PER_SECOND


def derive_lane(
    row: CourtRow,
    lane_row: LaneRow,
    state: LaneState,
    *,
    reset_event: Optional[str],
    dac_epsilon: float,
    rolling_window_complete: bool,
    total_epoch_complete: bool,
) -> LaneDerived:
    adjacent = not row.gap
    phase_1s = candidate_interval_ppb(
        lane_row.phase, state.previous_phase, adjacent=adjacent
    )
    delta_candidate_1s = candidate_interval_ppb(
        lane_row.delta, state.previous_delta, adjacent=adjacent
    )
    delta_raw_1s = lane_row.raw.interval_ppb()

    notes: list[str] = []
    if row.science_eligible and lane_row.raw.valid:
        state.raw_total.add(lane_row.raw)
        if state.raw_total.sample_count <= 60:
            state.first_60_raw.add(lane_row.raw)
        clock_ns = lane_row.raw.clock_interval_ns_exact()
        if clock_ns is None:
            raise RuntimeError("valid Delta-Raw interval produced no clock interval")
        state.last_600_raw.append(clock_ns)

    if row.interval_advanced:
        if not lane_row.raw.valid:
            raise ValueError(
                f"pps={row.count} {lane_row.name}: Better-Buckets interval advanced "
                "without Delta-Raw validity"
            )
        state.raw_window.add(row.rolling_sequence, lane_row.raw)
    else:
        state.raw_window.age_to(row.rolling_sequence)

    if row.endpoint_admitted:
        if not lane_row.phase.available:
            raise ValueError(
                f"pps={row.count} {lane_row.name}: Better-Buckets endpoint admitted "
                "without PhaseLedger candidate"
            )
        state.phase_window.add(row.rolling_sequence, lane_row.phase.ns)

    exact_gap: Optional[float] = None
    gap_step: Optional[float] = None
    if lane_row.phase.available and lane_row.delta.available:
        exact_gap = lane_row.delta.exact_ns - float(lane_row.phase.ns)
        if adjacent and state.previous_gap_ns_exact is not None:
            gap_step = exact_gap - state.previous_gap_ns_exact
        state.gap_window.add(row.count, exact_gap)
        if abs((lane_row.delta.ns - lane_row.phase.ns) - lane_row.reported_gap_ns) > 0:
            raise ValueError(
                f"pps={row.count} {lane_row.name}: candidate gap integer closure failed"
            )

    phase_10m_raw = state.phase_window.ppb()
    delta_10m_raw = state.raw_window.ppb(row.rolling_sequence)
    phase_10m = phase_10m_raw if rolling_window_complete else None
    delta_10m = delta_10m_raw if rolling_window_complete else None
    delta_total_raw = state.raw_total.ppb()
    delta_total = delta_total_raw if total_epoch_complete else None
    gap_slope = state.gap_window.slope_ppb()

    if (
        rolling_window_complete
        and row.bucket_10m_valid
        and len(state.raw_window.samples) != row.bucket_10m_n
    ):
        notes.append(
            f"B10N-D10N={row.bucket_10m_n - len(state.raw_window.samples):+d}"
        )

    if (
        total_epoch_complete
        and lane_row.fw_total_ppb is not None
        and delta_total is not None
    ):
        total_error = lane_row.fw_total_ppb - delta_total
        if abs(total_error) > TOTAL_RECON_TOLERANCE:
            notes.append(f"TOTAL-DTOT={total_error:+.3f}")

    if lane_row.fw_10m_ppb is not None and phase_10m is not None:
        phase_error = lane_row.fw_10m_ppb - phase_10m
        if abs(phase_error) > FW10_PHASE_TOLERANCE:
            notes.append(f"FW10-PH10={phase_error:+.3f}")

    if lane_row.readback_valid and lane_row.readback_code != lane_row.dac_hw_code:
        notes.append(
            f"DAC_READBACK={lane_row.readback_code}/{lane_row.dac_hw_code}"
        )

    dac_changed = (
        state.previous_dac_target is not None
        and abs(lane_row.dac_target - state.previous_dac_target) > dac_epsilon
    )
    if dac_changed:
        notes.append(
            f"DAC {state.previous_dac_target:.6f}->{lane_row.dac_target:.6f}"
        )

    state.previous_phase = lane_row.phase
    state.previous_delta = lane_row.delta
    state.previous_gap_ns_exact = exact_gap
    state.previous_dac_target = lane_row.dac_target

    return LaneDerived(
        phase_1s_ppb=phase_1s,
        delta_candidate_1s_ppb=delta_candidate_1s,
        delta_raw_1s_ppb=delta_raw_1s,
        phase_10m_ppb=phase_10m,
        delta_raw_10m_ppb=delta_10m,
        delta_raw_total_ppb=delta_total,
        candidate_gap_ns_exact=exact_gap,
        candidate_gap_step_ns=gap_step,
        candidate_gap_10m_slope_ppb=gap_slope,
        dac_changed=dac_changed,
        notes=tuple(notes),
    )


def update_epoch_summary(
    summary: EpochSummary,
    row: CourtRow,
    derived: Dict[str, LaneDerived],
) -> None:
    summary.end_pps = row.count
    summary.rows += 1
    for name in LANES:
        lane = row.lanes[name]
        derv = derived[name]
        if summary.first_fw10[name] is None and lane.fw_10m_ppb is not None:
            summary.first_fw10[name] = lane.fw_10m_ppb
        if summary.first_total[name] is None and lane.fw_total_ppb is not None:
            summary.first_total[name] = lane.fw_total_ppb
        if summary.first_camp[name] is None:
            summary.first_camp[name] = lane.campaign_ppb
        if summary.first_gap[name] is None and derv.candidate_gap_ns_exact is not None:
            summary.first_gap[name] = derv.candidate_gap_ns_exact
        if summary.first_dac[name] is None:
            summary.first_dac[name] = lane.dac_target

        summary.last_fw10[name] = lane.fw_10m_ppb
        summary.last_total[name] = lane.fw_total_ppb
        summary.last_camp[name] = lane.campaign_ppb
        summary.last_gap[name] = derv.candidate_gap_ns_exact
        summary.last_dac[name] = lane.dac_target


def last_window_ppb(state: LaneState) -> Optional[float]:
    if not state.last_600_raw:
        return None
    clock_ns = sum(state.last_600_raw)
    if clock_ns <= 0.0:
        return None
    reference_ns = float(len(state.last_600_raw)) * NS_PER_SECOND
    return (reference_ns / clock_ns - 1.0) * 1.0e9


# -----------------------------------------------------------------------------
# Presentation
# -----------------------------------------------------------------------------


def fmtf(value: Optional[float], width: int = 8, decimals: int = 3) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}.{decimals}f}"


def fmti(value: Optional[int], width: int) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}d}"


def should_display(
    row: CourtRow,
    derived: Dict[str, LaneDerived],
    *,
    reset_event: Optional[str],
    epoch_age: int,
    every: int,
    reset_detail_rows: int,
    candidate_step_gate_ns: float,
) -> bool:
    if reset_event is not None:
        return True
    if row.gap or not row.timeline_valid or row.disposition != "ACCEPT":
        return True
    if row.gnss is None:
        return True
    if epoch_age <= reset_detail_rows:
        return True
    if row.count % every == 0:
        return True
    for name in LANES:
        derv = derived[name]
        if derv.dac_changed:
            return True
        if (
            derv.candidate_gap_step_ns is not None
            and abs(derv.candidate_gap_step_ns) >= candidate_step_gate_ns
        ):
            return True
    return False


def print_header(campaign: str, batch_size: int, every: int) -> None:
    print(
        f"Campaign: {campaign}  court=PPB_RELAXATION  "
        f"server_batch={batch_size}  display_every={every}s"
    )
    print(
        "FW10=firmware bucket  PH10=PhaseLedger endpoint slope  "
        "D10=Delta-Raw admitted grand ratio  TOTAL=firmware TAU  DTOT=reconstructed TAU  CAMP=campaign clockface"
    )
    print(
        "B10N=firmware 10-minute proof sample count  PH1=PhaseLedger 1s  "
        "DR1=Delta-Raw 1s  D-P=DeltaCandidate-PhaseLedger ns  G10=trailing D-P slope ppb"
    )
    print(
        "PAIR also carries GNSS: GDRIFT=TPS1 clock drift ppb, dG=adjacent change, "
        "GFREQ=TPS4 VCLK error ppb, PERR=TPS4 PPS timing error ns, with adjacent deltas"
    )
    print()
    header = (
        f"{'PPS':>7} {'AGE':>5} {'RST':>4} {'UPD':>6} {'N':>6} {'B10N':>5} {'LANE':>5} "
        f"{'FW10':>8} {'PH10':>8} {'D10':>8} {'TOTAL':>8} {'DTOT':>8} {'CAMP':>8} "
        f"{'PH1':>8} {'DR1':>8} {'D-P':>9} {'G10':>8} {'DAC':>11}  NOTE"
    )
    print(header)
    print("─" * len(header))


def print_row(
    row: CourtRow,
    derived: Dict[str, LaneDerived],
    *,
    epoch_age: int,
    reset_event: Optional[str],
) -> None:
    row_notes: list[str] = []
    if reset_event:
        row_notes.append(reset_event)
    if row.gap:
        row_notes.append(
            f"GAP delta={row.count - row.previous_count if row.previous_count is not None else '?'}"
        )
    if not row.timeline_valid:
        row_notes.append("TIMELINE_INVALID")
    if row.disposition != "ACCEPT":
        row_notes.append(row.disposition)
    if not row.science_eligible:
        row_notes.append("SCIENCE_EXCLUDE")
    if row.servo_active:
        row_notes.append(f"SERVO={row.servo_mode}")
    elif row.servo_mode != "OFF":
        row_notes.append(f"SERVO_STATE={row.servo_mode}/INACTIVE")
    if row.dither_enabled:
        row_notes.append("DITHER")
    if row.gnss.freq_mode_name != "FINE_LOCK":
        row_notes.append(f"GNSS_MODE={row.gnss.freq_mode_name}")
    if row.gnss is None:
        row_notes.append(f"GNSS_{row.gnss_status}")
    else:
        if row.gnss.phase_skip != 0:
            row_notes.append(f"GNSS_PHASE_SKIP={row.gnss.phase_skip}")
        if row.gnss.alarm != 0:
            row_notes.append(f"GNSS_ALARM=0x{row.gnss.alarm:X}")

    for idx, name in enumerate(LANES):
        lane = row.lanes[name]
        derv = derived[name]
        notes = list(row_notes if idx == 0 else []) + list(derv.notes)
        print(
            f"{row.count:7d} {epoch_age:5d} {row.reset_count:4d} {row.update_count:6d} "
            f"{lane.welford_n:6d} {row.bucket_10m_n:5d} {name:>5} "
            f"{fmtf(lane.fw_10m_ppb)} {fmtf(derv.phase_10m_ppb)} "
            f"{fmtf(derv.delta_raw_10m_ppb)} {fmtf(lane.fw_total_ppb)} "
            f"{fmtf(derv.delta_raw_total_ppb)} {fmtf(lane.campaign_ppb)} "
            f"{fmtf(derv.phase_1s_ppb)} "
            f"{fmtf(derv.delta_raw_1s_ppb)} {fmtf(derv.candidate_gap_ns_exact, 9, 3)} "
            f"{fmtf(derv.candidate_gap_10m_slope_ppb)} {lane.dac_target:11.6f}  "
            f"{' | '.join(notes)}"
        )

    o1 = derived["OCXO1"].delta_raw_10m_ppb
    o2 = derived["OCXO2"].delta_raw_10m_ppb
    common = (o1 + o2) / 2.0 if o1 is not None and o2 is not None else None
    differential = o1 - o2 if o1 is not None and o2 is not None else None
    print(
        f"{'':7} {'':5} {'':4} {'':6} {'':6} {'':5} {'PAIR':>5} "
        f"{'':8} {'':8} {'':8} {'':8} {'':8} {'':8} {'':8} {'':8} "
        f"{'':9} {'':8} {'':11}  "
        f"D10_COMMON={fmtf(common, 8)}  D10_O1-O2={fmtf(differential, 8)}  "
        + (
            f"GDRIFT={row.gnss.clock_drift_ppb:+.3f} "
            f"dG={fmtf(row.gnss.drift_delta_ppb, 8)}  "
            f"GFREQ={row.gnss.freq_error_ppb:+.3f} "
            f"dGF={fmtf(row.gnss.freq_error_delta_ppb, 8)}  "
            f"PERR={row.gnss.pps_timing_error_ns:+.1f}ns "
            f"dPE={fmtf(row.gnss.pps_timing_error_delta_ns, 7, 1)}ns"
            if row.gnss is not None
            else f"GDRIFT=--- dG=---  GFREQ=--- dGF=---  PERR=--- dPE=---  GNSS={row.gnss_status}"
        )
    )


def print_summaries(summaries: Sequence[EpochSummary], state: CourtState) -> None:
    print("\nSTATISTICS EPOCH SUMMARY")
    print("=" * 92)
    for summary in summaries:
        span = summary.end_pps - summary.start_pps
        print(
            f"reset={summary.reset_count} pps={summary.start_pps}->{summary.end_pps} "
            f"span={span}s rows={summary.rows} "
            f"birth={'YES' if summary.complete_from_birth else 'PARTIAL'}"
        )
        for name in LANES:
            lane_state = state.lanes[name]
            gap_delta = None
            if summary.first_gap[name] is not None and summary.last_gap[name] is not None:
                gap_delta = summary.last_gap[name] - summary.first_gap[name]
            dac_delta = None
            if summary.first_dac[name] is not None and summary.last_dac[name] is not None:
                dac_delta = summary.last_dac[name] - summary.first_dac[name]
            print(
                f"  {name}: "
                f"FW10 {fmtf(summary.first_fw10[name], 8)}->{fmtf(summary.last_fw10[name], 8)}  "
                f"TOTAL {fmtf(summary.first_total[name], 8)}->{fmtf(summary.last_total[name], 8)}  "
                f"CAMP {fmtf(summary.first_camp[name], 8)}->{fmtf(summary.last_camp[name], 8)}  "
                f"D-P delta={fmtf(gap_delta, 9, 3)}ns  "
                f"DAC delta={fmtf(dac_delta, 10, 6)}  "
                f"DR first60={fmtf(summary.first_60_raw_ppb[name], 8)} "
                f"last600={fmtf(summary.last_600_raw_ppb[name], 8)}"
            )
        print()


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------


def parse(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stream one TEMPEST campaign and cross-court firmware PPB buckets, "
            "PhaseLedger clocks, Delta-Raw intervals, campaign clockfaces, and DAC control."
        )
    )
    parser.add_argument("campaign", help="TEMPEST campaign name, e.g. PPB2")
    parser.add_argument(
        "--skip",
        type=int,
        default=0,
        help="start after this campaign public_count (indexed range predicate; default 0)",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="maximum rows to process; 0 means no SQL LIMIT",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=DEFAULT_BATCH_SIZE,
        help=f"named-cursor fetch size (default {DEFAULT_BATCH_SIZE})",
    )
    parser.add_argument(
        "--every",
        type=int,
        default=DEFAULT_DISPLAY_EVERY,
        help=f"periodic display cadence in campaign seconds (default {DEFAULT_DISPLAY_EVERY})",
    )
    parser.add_argument(
        "--reset-detail",
        type=int,
        default=DEFAULT_RESET_DETAIL_ROWS,
        help=(
            "display every row for this many stats updates after each observed reset "
            f"(default {DEFAULT_RESET_DETAIL_ROWS})"
        ),
    )
    parser.add_argument(
        "--candidate-step-gate",
        type=float,
        default=DEFAULT_CANDIDATE_STEP_GATE_NS,
        help=(
            "always display a row when |Delta-Phase candidate gap step| reaches this many ns "
            f"(default {DEFAULT_CANDIDATE_STEP_GATE_NS:g})"
        ),
    )
    parser.add_argument(
        "--dac-epsilon",
        type=float,
        default=DEFAULT_DAC_CHANGE_EPSILON,
        help=f"DAC target change gate (default {DEFAULT_DAC_CHANGE_EPSILON:g})",
    )
    args = parser.parse_args(argv[1:])
    if args.skip < 0 or args.limit < 0:
        parser.error("--skip and --limit must be nonnegative")
    if args.batch_size <= 0 or args.every <= 0 or args.reset_detail < 0:
        parser.error("--batch-size/--every must be positive and --reset-detail nonnegative")
    if args.candidate_step_gate < 0.0 or args.dac_epsilon < 0.0:
        parser.error("candidate/DAC gates must be nonnegative")
    return args


def main(argv: Sequence[str]) -> None:
    args = parse(argv)
    print_header(args.campaign, args.batch_size, args.every)

    court = CourtState()
    previous_count: Optional[int] = None
    previous_gnss: Optional[GnssWitness] = None
    processed = 0
    displayed = 0

    for db_id, count, campaign, stats, control, gnss_obj in iter_rows(
        args.campaign,
        skip=args.skip,
        limit=args.limit,
        batch_size=args.batch_size,
    ):
        row = build_row(
            db_id,
            count,
            campaign,
            stats,
            control,
            gnss_obj,
            previous_count,
            previous_gnss,
        )
        reset_event = court.begin_epoch(row)
        if court.epoch_start_pps is None or court.epoch is None:
            raise RuntimeError("court did not establish a statistics epoch")
        epoch_age = row.update_count

        if row.gap:
            court.note_observation_gap(row)
        rolling_complete = court.rolling_window_complete(row)
        derived = {
            name: derive_lane(
                row,
                row.lanes[name],
                court.lanes[name],
                reset_event=reset_event,
                dac_epsilon=args.dac_epsilon,
                rolling_window_complete=rolling_complete,
                total_epoch_complete=court.epoch_total_complete,
            )
            for name in LANES
        }
        update_epoch_summary(court.epoch, row, derived)

        if should_display(
            row,
            derived,
            reset_event=reset_event,
            epoch_age=epoch_age,
            every=args.every,
            reset_detail_rows=args.reset_detail,
            candidate_step_gate_ns=args.candidate_step_gate,
        ):
            print_row(row, derived, epoch_age=epoch_age, reset_event=reset_event)
            displayed += 1

        previous_count = count
        previous_gnss = row.gnss
        processed += 1

    if processed == 0:
        raise RuntimeError(
            f"No TEMPEST campaign_detail rows found for campaign {args.campaign!r}"
        )

    court.finish()
    print_summaries(court.summaries, court)
    print(
        f"Processed {processed:,} rows through a named server cursor; "
        f"displayed {displayed:,} row groups."
    )


if __name__ == "__main__":
    main(sys.argv)
