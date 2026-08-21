"""ZPNet reference_relaxation_court — streamed common-reference causality court.

This court follows ppb_relaxation_court one layer downward.  The first court
proved that firmware Better-Buckets, PhaseLedger endpoint slopes, and Delta-Raw
cycle ratios agree while both OCXO lanes undergo a large common-mode frequency
excursion.  This program asks which shared reference or environmental witness
moves with that excursion.

For every TEMPEST row it compares the same OCXO DWT interval against two
independent references:

* VC60  — 60-second grand-ratio using the campaign Delta-Raw/selected-VCLOCK
          reference interval.  This is the primitive frequency species that
          drove D10 in ppb_relaxation_court.
* PP60  — 60-second grand-ratio using the physical PPS/GPIO raw-cycle interval
          as the reference against the same OCXO interval.

It also audits the reference geometry itself:

* PV60  — physical-PPS / observed-VCLOCK 60-second ratio, in ppb.
* REF-CPS — selected Delta-Raw reference cycles minus Alpha's smooth
            dwt_cycles_per_second ruler.
* V-P   — observed VCLOCK interval cycles minus physical PPS interval cycles.
* REF-V — selected Delta-Raw reference cycles minus the raw VCLOCK rail.

And it carries the shared context that may explain a common physical shift:
ambient / GNSS / Pi temperature, pressure, GNSS receiver drift/clock quality,
PPS timing error, and Pi-owned DAC targets.  Environment/GNSS fields are
optional testimony because not every historic CLOCKS_V4 row necessarily has
all sensors; required clock/reference testimony fails loudly.

The SQL is deliberately campaign-centric and scan-safe in the same style as
raw_cycles.py and ppb_relaxation_court.py: a named server-side cursor, campaign
keys plus public_count range predicates, no count(*), no aggregate query, no
OFFSET, and no whole-history materialization.  Only the campaign, anchor,
raw-cycle, statistics/control, environment, and GNSS subtrees needed by this
court are projected from JSONB.

Typical use:

    python tests/reference_relaxation_court.py PPB2
    python tests/reference_relaxation_court.py PPB2 --every 10
    python tests/reference_relaxation_court.py PPB2 --skip 3800 --every 1

Interpretation shortcut:

    VC60 moves, PP60 moves, PV60 stays near zero
        -> OCXO/common physical behavior relative to GNSS PPS is implicated.

    VC60 moves, PP60 stays stable, PV60 moves
        -> selected VCLOCK/reference geometry is implicated.

    VC60 and PP60 move while temperature/power witnesses co-move
        -> shared environmental forcing becomes a leading candidate.

    DAC moves before the frequency move
        -> actuator/control causality must be investigated.
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
DWT_EXPECTED_PER_PPS = 1_008_000_000
WINDOW_SECONDS = 60
DEFAULT_BATCH_SIZE = 64
DEFAULT_DISPLAY_EVERY = 10
DEFAULT_RESET_DETAIL_ROWS = 30
DAC_CHANGE_EPSILON = 0.000001
REF_VCLOCK_GATE_CYCLES = 32

PPS_COUNT_SQL = """
NULLIF(
    payload #>> '{campaign,public_count}',
    ''
)::bigint
"""


# -----------------------------------------------------------------------------
# Strict and optional parsing
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


def opt_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on", "nominal", "ready"}:
            return True
        if normalized in {"false", "0", "no", "off", "anomaly", "hold"}:
            return False
    return None


def path(obj: Dict[str, Any], dotted: str) -> Any:
    cur: Any = obj
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def first_float(obj: Dict[str, Any], *paths: str) -> Optional[float]:
    for dotted in paths:
        value = opt_float(path(obj, dotted))
        if value is not None:
            return value
    return None


def first_int(obj: Dict[str, Any], *paths: str) -> Optional[int]:
    for dotted in paths:
        value = opt_int(path(obj, dotted))
        if value is not None:
            return value
    return None


# -----------------------------------------------------------------------------
# Database stream — no aggregate scans
# -----------------------------------------------------------------------------


def _assert_campaign_indexed(cur: Any, campaign: str) -> None:
    """Existence probe only; never pay a full-campaign count/aggregate preflight."""
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
) -> Iterator[Tuple[int, int, Dict[str, Any], Dict[str, Any], Dict[str, Any], Dict[str, Any], Dict[str, Any], Dict[str, Any], Dict[str, Any]]]:
    """Stream only testimony required by the reference-relaxation court."""
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
                payload #> '{{clocks,anchor}}' AS anchor_payload,
                payload #> '{{clocks,raw_cycles}}' AS raw_cycles_payload,
                payload #> '{{clocks,stats}}' AS stats_payload,
                payload #> '{{clocks,control}}' AS control_payload,
                payload -> 'environment' AS environment_payload,
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

        cur = conn.cursor(name="reference_relaxation_court_stream")
        cur.itersize = batch_size
        cur.execute(sql, tuple(params))

        for row in cur:
            decoded: list[Any] = []
            for key in (
                "campaign_payload",
                "anchor_payload",
                "raw_cycles_payload",
                "stats_payload",
                "control_payload",
                "environment_payload",
                "gnss_payload",
            ):
                value = row[key]
                if isinstance(value, str):
                    value = json.loads(value)
                decoded.append(value)

            campaign_obj = req_dict(decoded[0], f"campaign_detail id={row['id']} campaign")
            anchor_obj = req_dict(decoded[1], f"campaign_detail id={row['id']} clocks.anchor")
            raw_obj = req_dict(decoded[2], f"campaign_detail id={row['id']} clocks.raw_cycles")
            stats_obj = req_dict(decoded[3], f"campaign_detail id={row['id']} clocks.stats")
            control_obj = req_dict(decoded[4], f"campaign_detail id={row['id']} clocks.control")
            environment_obj = decoded[5] if isinstance(decoded[5], dict) else {}
            gnss_obj = decoded[6] if isinstance(decoded[6], dict) else {}

            if campaign_obj.get("schema") != CAMPAIGN_SCHEMA:
                raise ValueError(
                    f"campaign_detail id={row['id']} campaign schema="
                    f"{campaign_obj.get('schema')!r}, expected {CAMPAIGN_SCHEMA!r}"
                )

            db_count = req_int(
                row["pps_count"],
                f"campaign_detail id={row['id']} pps_count",
                minimum=1,
            )
            payload_count = req_int(
                campaign_obj.get("public_count"),
                f"campaign_detail id={row['id']} campaign.public_count",
                minimum=1,
            )
            if db_count != payload_count:
                raise ValueError(
                    "campaign_detail relational/payload public-count mismatch: "
                    f"id={row['id']} db={db_count} payload={payload_count}"
                )

            yield (
                int(row["id"]),
                db_count,
                campaign_obj,
                anchor_obj,
                raw_obj,
                stats_obj,
                control_obj,
                environment_obj,
                gnss_obj,
            )


# -----------------------------------------------------------------------------
# Canonical row testimony
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class RawRail:
    valid: bool
    observed_cycles: int
    delay_status: str
    delay_by: str
    residual_delay_valid: bool
    residual_delay_cycles: int


@dataclass(frozen=True)
class LaneScience:
    valid: bool
    science_worthy: bool
    antecedents_complete: bool
    reference_cycles: int
    clock_cycles: int


@dataclass(frozen=True)
class Context:
    ambient_temp_c: Optional[float]
    pi_temp_c: Optional[float]
    gnss_temp_c: Optional[float]
    pressure_hpa: Optional[float]
    gnss_drift_ppb: Optional[float]
    pps_timing_error_ns: Optional[float]
    estimated_accuracy_ns: Optional[float]
    satellites: Optional[int]
    hdop: Optional[float]
    pi_power_volts: Optional[float]
    teensy_power_volts: Optional[float]
    ocxo1_power_volts: Optional[float]
    ocxo1_power_amps: Optional[float]
    ocxo1_power_watts: Optional[float]
    ocxo2_power_volts: Optional[float]
    ocxo2_power_amps: Optional[float]
    ocxo2_power_watts: Optional[float]
    battery_remaining_pct: Optional[float]


@dataclass(frozen=True)
class Row:
    db_id: int
    count: int
    previous_count: Optional[int]
    gap: bool
    reset_count: int
    update_count: int
    timeline_valid: bool
    science_eligible: bool
    disposition: str
    dwt_cps: int
    pps: RawRail
    vclock: RawRail
    lanes: Dict[str, LaneScience]
    dac: Dict[str, float]
    servo_mode: str
    dither_enabled: bool
    context: Context


def parse_raw_rail(raw: Dict[str, Any], key: str) -> RawRail:
    obj = req_dict(raw.get(key), f"clocks.raw_cycles.{key}")
    return RawRail(
        valid=req_bool(obj.get("valid"), f"clocks.raw_cycles.{key}.valid"),
        observed_cycles=req_int(
            obj.get("observed_cycles"),
            f"clocks.raw_cycles.{key}.observed_cycles",
            minimum=0,
        ),
        delay_status=str(obj.get("delay_status") or "UNKNOWN").upper(),
        delay_by=str(obj.get("delay_by") or "UNKNOWN").upper(),
        residual_delay_valid=req_bool(
            obj.get("residual_delay_valid"),
            f"clocks.raw_cycles.{key}.residual_delay_valid",
        ),
        residual_delay_cycles=req_int(
            obj.get("residual_delay_cycles"),
            f"clocks.raw_cycles.{key}.residual_delay_cycles",
        ),
    )


def parse_lane_science(campaign: Dict[str, Any], key: str) -> LaneScience:
    lane = req_dict(campaign.get(key), f"campaign.{key}")
    science = req_dict(lane.get("science"), f"campaign.{key}.science")
    valid = req_bool(science.get("delta_raw_valid"), f"campaign.{key}.science.delta_raw_valid")
    ref = req_int(
        science.get("delta_raw_reference_interval_cycles"),
        f"campaign.{key}.science.delta_raw_reference_interval_cycles",
        minimum=0,
    )
    clk = req_int(
        science.get("delta_raw_clock_interval_cycles"),
        f"campaign.{key}.science.delta_raw_clock_interval_cycles",
        minimum=0,
    )
    if valid and (ref == 0 or clk == 0):
        raise ValueError(f"campaign.{key}.science claims Delta-Raw valid with zero cycles")
    return LaneScience(
        valid=valid,
        science_worthy=req_bool(science.get("science_worthy"), f"campaign.{key}.science.science_worthy"),
        antecedents_complete=req_bool(
            science.get("antecedents_complete"),
            f"campaign.{key}.science.antecedents_complete",
        ),
        reference_cycles=ref,
        clock_cycles=clk,
    )


def parse_context(environment: Dict[str, Any], gnss: Dict[str, Any]) -> Context:
    # Current CLOCKS persistence projects these environment keys directly from
    # SYSTEM.REPORT.  Alternate GNSS paths are accepted only as aliases for the
    # same persisted testimony, never as synthesized defaults.
    return Context(
        ambient_temp_c=first_float(environment, "ambient_temp_c", "temperature_c"),
        pi_temp_c=first_float(environment, "pi_temp_c"),
        gnss_temp_c=(
            first_float(environment, "gnss_temp_c")
            if first_float(environment, "gnss_temp_c") is not None
            else first_float(gnss, "clock.temperature_c", "temperature_c")
        ),
        pressure_hpa=first_float(environment, "pressure_hpa"),
        gnss_drift_ppb=first_float(
            gnss,
            "clock.drift_ppb",
            "drift_ppb",
            "clock_drift_ppb",
        ),
        pps_timing_error_ns=first_float(
            gnss,
            "pps_timing_error_ns",
            "discipline.pps_timing_error_ns",
            "pps.timing_error_ns",
        ),
        estimated_accuracy_ns=first_float(
            gnss,
            "estimated_accuracy_ns",
            "pps.estimated_accuracy_ns",
        ),
        satellites=first_int(gnss, "satellites", "satellite_count"),
        hdop=first_float(gnss, "hdop"),
        pi_power_volts=first_float(environment, "pi_power.volts"),
        teensy_power_volts=first_float(environment, "teensy_power.volts"),
        ocxo1_power_volts=first_float(environment, "ocxo1_power.volts"),
        ocxo1_power_amps=first_float(environment, "ocxo1_power.amps"),
        ocxo1_power_watts=first_float(environment, "ocxo1_power.watts"),
        ocxo2_power_volts=first_float(environment, "ocxo2_power.volts"),
        ocxo2_power_amps=first_float(environment, "ocxo2_power.amps"),
        ocxo2_power_watts=first_float(environment, "ocxo2_power.watts"),
        battery_remaining_pct=first_float(environment, "battery.remaining_pct"),
    )


def build_row(
    db_id: int,
    count: int,
    campaign: Dict[str, Any],
    anchor: Dict[str, Any],
    raw: Dict[str, Any],
    stats: Dict[str, Any],
    control: Dict[str, Any],
    environment: Dict[str, Any],
    gnss: Dict[str, Any],
    previous_count: Optional[int],
) -> Row:
    delta = None if previous_count is None else count - previous_count
    gap = delta is not None and delta != 1

    status = req_dict(campaign.get("status"), "campaign.status")
    disposition = req_dict(campaign.get("disposition"), "campaign.disposition")
    timeline_valid = req_bool(status.get("timeline_valid"), "campaign.status.timeline_valid")
    science_eligible = req_bool(
        disposition.get("science_eligible"),
        "campaign.disposition.science_eligible",
    )
    disposition_name = str(disposition.get("status") or "").upper()
    if not disposition_name:
        raise ValueError("campaign.disposition.status is missing")

    pps = parse_raw_rail(raw, "pps")
    vclock = parse_raw_rail(raw, "vclock")
    lanes = {
        name: parse_lane_science(campaign, LANE_KEYS[name])
        for name in LANES
    }

    # Delta-Raw is defined against one selected VCLOCK reference.  Both OCXO
    # lanes on one row must therefore carry the exact same reference interval
    # whenever both are valid.
    if lanes["OCXO1"].valid and lanes["OCXO2"].valid:
        if lanes["OCXO1"].reference_cycles != lanes["OCXO2"].reference_cycles:
            raise ValueError(
                f"pps={count}: OCXO Delta-Raw reference disagreement "
                f"o1={lanes['OCXO1'].reference_cycles} "
                f"o2={lanes['OCXO2'].reference_cycles}"
            )

    control_o1 = req_dict(control.get("ocxo1"), "clocks.control.ocxo1")
    control_o2 = req_dict(control.get("ocxo2"), "clocks.control.ocxo2")

    return Row(
        db_id=db_id,
        count=count,
        previous_count=previous_count,
        gap=gap,
        reset_count=req_int(stats.get("reset_count"), "clocks.stats.reset_count", minimum=0),
        update_count=req_int(stats.get("update_count"), "clocks.stats.update_count", minimum=0),
        timeline_valid=timeline_valid,
        science_eligible=science_eligible,
        disposition=disposition_name,
        dwt_cps=req_int(
            anchor.get("dwt_cycles_per_second"),
            "clocks.anchor.dwt_cycles_per_second",
            minimum=1,
        ),
        pps=pps,
        vclock=vclock,
        lanes=lanes,
        dac={
            "OCXO1": req_float(control_o1.get("target_code"), "clocks.control.ocxo1.target_code"),
            "OCXO2": req_float(control_o2.get("target_code"), "clocks.control.ocxo2.target_code"),
        },
        servo_mode=str(control.get("servo_mode") or "OFF").upper(),
        dither_enabled=req_bool(
            control.get("dither_operator_enabled"),
            "clocks.control.dither_operator_enabled",
        ),
        context=parse_context(environment, gnss),
    )


# -----------------------------------------------------------------------------
# Rolling ratio / mean courts
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class RatioSample:
    count: int
    reference_cycles: int
    clock_cycles: int


@dataclass
class RatioWindow:
    samples: Deque[RatioSample] = field(default_factory=deque)
    reference_sum: int = 0
    clock_sum: int = 0

    def clear(self) -> None:
        self.samples.clear()
        self.reference_sum = 0
        self.clock_sum = 0

    def break_continuity(self) -> None:
        self.clear()

    def add(self, count: int, reference_cycles: int, clock_cycles: int) -> None:
        if reference_cycles <= 0 or clock_cycles <= 0:
            raise ValueError("ratio window cannot admit non-positive cycle testimony")
        if self.samples and count != self.samples[-1].count + 1:
            raise ValueError(
                f"ratio window chronology gap: previous={self.samples[-1].count} current={count}"
            )
        self.samples.append(RatioSample(count, reference_cycles, clock_cycles))
        self.reference_sum += reference_cycles
        self.clock_sum += clock_cycles
        while len(self.samples) > WINDOW_SECONDS:
            old = self.samples.popleft()
            self.reference_sum -= old.reference_cycles
            self.clock_sum -= old.clock_cycles

    def ready(self) -> bool:
        return len(self.samples) == WINDOW_SECONDS

    def ppb(self) -> Optional[float]:
        if not self.ready() or self.clock_sum <= 0:
            return None
        return (float(self.reference_sum) / float(self.clock_sum) - 1.0) * 1.0e9


@dataclass(frozen=True)
class ScalarSample:
    count: int
    value: float


@dataclass
class ScalarWindow:
    samples: Deque[ScalarSample] = field(default_factory=deque)
    total: float = 0.0

    def clear(self) -> None:
        self.samples.clear()
        self.total = 0.0

    def add(self, count: int, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError("scalar window cannot admit non-finite testimony")
        if self.samples and count != self.samples[-1].count + 1:
            raise ValueError(
                f"scalar window chronology gap: previous={self.samples[-1].count} current={count}"
            )
        self.samples.append(ScalarSample(count, value))
        self.total += value
        while len(self.samples) > WINDOW_SECONDS:
            old = self.samples.popleft()
            self.total -= old.value

    def ready(self) -> bool:
        return len(self.samples) == WINDOW_SECONDS

    def mean(self) -> Optional[float]:
        return self.total / len(self.samples) if self.ready() else None


@dataclass
class OptionalScalarWindow:
    samples: Deque[Tuple[int, Optional[float]]] = field(default_factory=deque)

    def clear(self) -> None:
        self.samples.clear()

    def add(self, count: int, value: Optional[float]) -> None:
        if self.samples and count != self.samples[-1][0] + 1:
            raise ValueError(
                f"optional window chronology gap: previous={self.samples[-1][0]} current={count}"
            )
        self.samples.append((count, value))
        while len(self.samples) > WINDOW_SECONDS:
            self.samples.popleft()

    def mean(self) -> Optional[float]:
        if len(self.samples) != WINDOW_SECONDS:
            return None
        values = [value for _count, value in self.samples if value is not None]
        if len(values) != WINDOW_SECONDS:
            return None
        return sum(values) / float(WINDOW_SECONDS)


@dataclass
class CourtState:
    vc: Dict[str, RatioWindow] = field(
        default_factory=lambda: {name: RatioWindow() for name in LANES}
    )
    pp: Dict[str, RatioWindow] = field(
        default_factory=lambda: {name: RatioWindow() for name in LANES}
    )
    pv: RatioWindow = field(default_factory=RatioWindow)
    ref_minus_cps: ScalarWindow = field(default_factory=ScalarWindow)
    v_minus_p: ScalarWindow = field(default_factory=ScalarWindow)
    ref_minus_v: ScalarWindow = field(default_factory=ScalarWindow)
    cps_delta: ScalarWindow = field(default_factory=ScalarWindow)
    pps_delta: ScalarWindow = field(default_factory=ScalarWindow)
    vclock_delta: ScalarWindow = field(default_factory=ScalarWindow)
    context_windows: Dict[str, OptionalScalarWindow] = field(
        default_factory=lambda: {
            "ambient_temp_c": OptionalScalarWindow(),
            "pi_temp_c": OptionalScalarWindow(),
            "gnss_temp_c": OptionalScalarWindow(),
            "pressure_hpa": OptionalScalarWindow(),
            "gnss_drift_ppb": OptionalScalarWindow(),
            "pps_timing_error_ns": OptionalScalarWindow(),
            "estimated_accuracy_ns": OptionalScalarWindow(),
            "pi_power_volts": OptionalScalarWindow(),
            "teensy_power_volts": OptionalScalarWindow(),
            "ocxo1_power_volts": OptionalScalarWindow(),
            "ocxo1_power_amps": OptionalScalarWindow(),
            "ocxo1_power_watts": OptionalScalarWindow(),
            "ocxo2_power_volts": OptionalScalarWindow(),
            "ocxo2_power_amps": OptionalScalarWindow(),
            "ocxo2_power_watts": OptionalScalarWindow(),
            "battery_remaining_pct": OptionalScalarWindow(),
        }
    )
    previous_dac: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )

    def break_continuity(self) -> None:
        for window in self.vc.values():
            window.break_continuity()
        for window in self.pp.values():
            window.break_continuity()
        self.pv.break_continuity()
        for window in (
            self.ref_minus_cps,
            self.v_minus_p,
            self.ref_minus_v,
            self.cps_delta,
            self.pps_delta,
            self.vclock_delta,
        ):
            window.clear()
        for window in self.context_windows.values():
            window.clear()


@dataclass(frozen=True)
class Derived:
    vc60: Dict[str, Optional[float]]
    pp60: Dict[str, Optional[float]]
    vc_common: Optional[float]
    vc_diff: Optional[float]
    pp_common: Optional[float]
    pp_diff: Optional[float]
    pv60: Optional[float]
    ref_minus_cps60: Optional[float]
    v_minus_p60: Optional[float]
    ref_minus_v60: Optional[float]
    cps_delta60: Optional[float]
    pps_delta60: Optional[float]
    vclock_delta60: Optional[float]
    context60: Dict[str, Optional[float]]


def pair_common(a: Optional[float], b: Optional[float]) -> Optional[float]:
    return None if a is None or b is None else (a + b) / 2.0


def pair_diff(a: Optional[float], b: Optional[float]) -> Optional[float]:
    return None if a is None or b is None else a - b


def update_state(state: CourtState, row: Row) -> Tuple[Derived, list[str]]:
    notes: list[str] = []
    if row.gap:
        state.break_continuity()
        notes.append(f"GAP_AFTER={row.previous_count}")

    o1 = row.lanes["OCXO1"]
    o2 = row.lanes["OCXO2"]
    both_delta = o1.valid and o2.valid and o1.science_worthy and o2.science_worthy
    refs_equal = o1.reference_cycles == o2.reference_cycles

    if both_delta and refs_equal and row.pps.valid and row.vclock.valid:
        ref = o1.reference_cycles

        # Check how the explicit Delta-Raw reference relates to the compact raw
        # VCLOCK rail.  A persistent discrepancy is useful testimony; a large
        # unexplained one is called out rather than silently equated.
        ref_minus_v = ref - row.vclock.observed_cycles
        if abs(ref_minus_v) > REF_VCLOCK_GATE_CYCLES:
            notes.append(f"REF_V={ref_minus_v:+d}cy")

        for name in LANES:
            science = row.lanes[name]
            state.vc[name].add(row.count, science.reference_cycles, science.clock_cycles)
            state.pp[name].add(row.count, row.pps.observed_cycles, science.clock_cycles)

        state.pv.add(row.count, row.pps.observed_cycles, row.vclock.observed_cycles)
        state.ref_minus_cps.add(row.count, float(ref - row.dwt_cps))
        state.v_minus_p.add(
            row.count, float(row.vclock.observed_cycles - row.pps.observed_cycles)
        )
        state.ref_minus_v.add(row.count, float(ref_minus_v))
        state.cps_delta.add(row.count, float(row.dwt_cps - DWT_EXPECTED_PER_PPS))
        state.pps_delta.add(
            row.count, float(row.pps.observed_cycles - DWT_EXPECTED_PER_PPS)
        )
        state.vclock_delta.add(
            row.count, float(row.vclock.observed_cycles - DWT_EXPECTED_PER_PPS)
        )

        for key, window in state.context_windows.items():
            window.add(row.count, getattr(row.context, key))
    else:
        # Do not bridge an excluded or incomplete physical/reference interval.
        state.break_continuity()
        missing: list[str] = []
        if not both_delta:
            missing.append("DELTA")
        if not refs_equal:
            missing.append("REF_SPLIT")
        if not row.pps.valid:
            missing.append("PPS")
        if not row.vclock.valid:
            missing.append("VCLOCK")
        notes.append("COURT_BREAK=" + "+".join(missing))

    vc = {name: state.vc[name].ppb() for name in LANES}
    pp = {name: state.pp[name].ppb() for name in LANES}
    derived = Derived(
        vc60=vc,
        pp60=pp,
        vc_common=pair_common(vc["OCXO1"], vc["OCXO2"]),
        vc_diff=pair_diff(vc["OCXO1"], vc["OCXO2"]),
        pp_common=pair_common(pp["OCXO1"], pp["OCXO2"]),
        pp_diff=pair_diff(pp["OCXO1"], pp["OCXO2"]),
        pv60=state.pv.ppb(),
        ref_minus_cps60=state.ref_minus_cps.mean(),
        v_minus_p60=state.v_minus_p.mean(),
        ref_minus_v60=state.ref_minus_v.mean(),
        cps_delta60=state.cps_delta.mean(),
        pps_delta60=state.pps_delta.mean(),
        vclock_delta60=state.vclock_delta.mean(),
        context60={key: window.mean() for key, window in state.context_windows.items()},
    )
    return derived, notes


# -----------------------------------------------------------------------------
# Epoch summaries
# -----------------------------------------------------------------------------


@dataclass
class RunningMoments:
    n: int = 0
    total: float = 0.0
    minimum: float = math.inf
    maximum: float = -math.inf

    def add(self, value: Optional[float]) -> None:
        if value is None:
            return
        self.n += 1
        self.total += value
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)

    def mean(self) -> Optional[float]:
        return None if self.n == 0 else self.total / self.n


@dataclass
class Epoch:
    reset_count: int
    start_pps: int
    end_pps: int
    rows: int = 0
    first_vc_common: Optional[float] = None
    last_vc_common: Optional[float] = None
    first_pp_common: Optional[float] = None
    last_pp_common: Optional[float] = None
    first_pv: Optional[float] = None
    last_pv: Optional[float] = None
    first_dac: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    last_dac: Dict[str, Optional[float]] = field(
        default_factory=lambda: {name: None for name in LANES}
    )
    context: Dict[str, RunningMoments] = field(
        default_factory=lambda: {
            "ambient_temp_c": RunningMoments(),
            "pi_temp_c": RunningMoments(),
            "gnss_temp_c": RunningMoments(),
            "pressure_hpa": RunningMoments(),
            "gnss_drift_ppb": RunningMoments(),
            "pps_timing_error_ns": RunningMoments(),
            "estimated_accuracy_ns": RunningMoments(),
            "pi_power_volts": RunningMoments(),
            "teensy_power_volts": RunningMoments(),
            "ocxo1_power_volts": RunningMoments(),
            "ocxo1_power_amps": RunningMoments(),
            "ocxo1_power_watts": RunningMoments(),
            "ocxo2_power_volts": RunningMoments(),
            "ocxo2_power_amps": RunningMoments(),
            "ocxo2_power_watts": RunningMoments(),
            "battery_remaining_pct": RunningMoments(),
        }
    )
    pv: RunningMoments = field(default_factory=RunningMoments)
    ref_minus_cps: RunningMoments = field(default_factory=RunningMoments)
    v_minus_p: RunningMoments = field(default_factory=RunningMoments)


def note_epoch(epoch: Epoch, row: Row, derived: Derived) -> None:
    epoch.end_pps = row.count
    epoch.rows += 1
    if derived.vc_common is not None:
        if epoch.first_vc_common is None:
            epoch.first_vc_common = derived.vc_common
        epoch.last_vc_common = derived.vc_common
    if derived.pp_common is not None:
        if epoch.first_pp_common is None:
            epoch.first_pp_common = derived.pp_common
        epoch.last_pp_common = derived.pp_common
    if derived.pv60 is not None:
        if epoch.first_pv is None:
            epoch.first_pv = derived.pv60
        epoch.last_pv = derived.pv60
        epoch.pv.add(derived.pv60)
    epoch.ref_minus_cps.add(derived.ref_minus_cps60)
    epoch.v_minus_p.add(derived.v_minus_p60)

    for name in LANES:
        if epoch.first_dac[name] is None:
            epoch.first_dac[name] = row.dac[name]
        epoch.last_dac[name] = row.dac[name]

    for key, moments in epoch.context.items():
        moments.add(derived.context60.get(key))


# -----------------------------------------------------------------------------
# Presentation
# -----------------------------------------------------------------------------


def f(value: Optional[float], width: int = 9, decimals: int = 3) -> str:
    return f"{'---':>{width}}" if value is None else f"{value:>{width}.{decimals}f}"


def fi(value: Optional[float], width: int = 9, decimals: int = 1) -> str:
    return f"{'---':>{width}}" if value is None else f"{value:>+{width}.{decimals}f}"


def context_note(row: Row, derived: Derived) -> str:
    c = derived.context60
    parts: list[str] = []
    if c.get("ambient_temp_c") is not None:
        parts.append(f"AMB={c['ambient_temp_c']:.3f}C")
    if c.get("gnss_temp_c") is not None:
        parts.append(f"GNSS_T={c['gnss_temp_c']:.3f}C")
    if c.get("pi_temp_c") is not None:
        parts.append(f"PI_T={c['pi_temp_c']:.3f}C")
    if c.get("pressure_hpa") is not None:
        parts.append(f"P={c['pressure_hpa']:.3f}hPa")
    if c.get("gnss_drift_ppb") is not None:
        parts.append(f"GNSS_D={c['gnss_drift_ppb']:+.3f}ppb")
    if c.get("pps_timing_error_ns") is not None:
        parts.append(f"PPS_E={c['pps_timing_error_ns']:+.3f}ns")
    if c.get("ocxo1_power_volts") is not None:
        parts.append(f"O1V={c['ocxo1_power_volts']:.5f}")
    if c.get("ocxo2_power_volts") is not None:
        parts.append(f"O2V={c['ocxo2_power_volts']:.5f}")
    if c.get("ocxo1_power_watts") is not None:
        parts.append(f"O1W={c['ocxo1_power_watts']:.4f}")
    if c.get("ocxo2_power_watts") is not None:
        parts.append(f"O2W={c['ocxo2_power_watts']:.4f}")
    return " ".join(parts)


def print_header(campaign: str, batch_size: int, every: int) -> None:
    print(
        f"Campaign: {campaign}  court=REFERENCE_RELAXATION  "
        f"server_batch={batch_size}  display_every={every}s",
        flush=True,
    )
    print(
        "VC60=Delta/VCLOCK common  PP60=physical-PPS/OCXO common  "
        "PV60=physical-PPS/VCLOCK ratio",
        flush=True,
    )
    print(
        "R-C=Delta reference - smooth DWT CPS cycles  V-P=VCLOCK - physical PPS cycles  "
        "R-V=Delta reference - VCLOCK cycles",
        flush=True,
    )
    print(
        f"{'PPS':>7} {'AGE':>6} {'RST':>4} {'VC60':>9} {'PP60':>9} {'PV60':>9} "
        f"{'VCDF':>9} {'PPDF':>9} {'R-C':>9} {'V-P':>9} {'R-V':>9} "
        f"{'CPSd':>9} {'PPSd':>9} {'VCLKd':>9} {'DAC1':>12} {'DAC2':>12} NOTE",
        flush=True,
    )
    print("─" * 185, flush=True)


def print_row(row: Row, derived: Derived, notes: Sequence[str]) -> None:
    age = row.update_count
    note_parts = list(notes)
    ctx = context_note(row, derived)
    if ctx:
        note_parts.append(ctx)
    if row.servo_mode != "OFF":
        note_parts.append(f"SERVO={row.servo_mode}")
    if row.dither_enabled:
        note_parts.append("DITHER=ON")
    if row.disposition != "ACCEPT":
        note_parts.append(row.disposition)
    if not row.timeline_valid:
        note_parts.append("TIMELINE_INVALID")

    print(
        f"{row.count:7d} {age:6d} {row.reset_count:4d} "
        f"{f(derived.vc_common)} {f(derived.pp_common)} {f(derived.pv60)} "
        f"{f(derived.vc_diff)} {f(derived.pp_diff)} "
        f"{fi(derived.ref_minus_cps60)} {fi(derived.v_minus_p60)} {fi(derived.ref_minus_v60)} "
        f"{fi(derived.cps_delta60)} {fi(derived.pps_delta60)} {fi(derived.vclock_delta60)} "
        f"{row.dac['OCXO1']:12.6f} {row.dac['OCXO2']:12.6f} "
        + (" | ".join(note_parts) if note_parts else ""),
        flush=True,
    )


def fmt_delta(first: Optional[float], last: Optional[float]) -> str:
    if first is None or last is None:
        return "---"
    return f"{last-first:+.3f}"


def format_range(m: RunningMoments, decimals: int = 3) -> str:
    if m.n == 0:
        return "---"
    return (
        f"mean={m.mean():.{decimals}f} "
        f"range={m.minimum:.{decimals}f}..{m.maximum:.{decimals}f} n={m.n}"
    )


def print_summaries(epochs: Sequence[Epoch]) -> None:
    print("\nREFERENCE EPOCH SUMMARY", flush=True)
    print("=" * 100, flush=True)
    for epoch in epochs:
        print(
            f"reset={epoch.reset_count} pps={epoch.start_pps}->{epoch.end_pps} "
            f"span={epoch.end_pps-epoch.start_pps}s rows={epoch.rows}",
            flush=True,
        )
        print(
            "  common: "
            f"VC60 {f(epoch.first_vc_common).strip()}->{f(epoch.last_vc_common).strip()} "
            f"delta={fmt_delta(epoch.first_vc_common, epoch.last_vc_common)} ppb; "
            f"PP60 {f(epoch.first_pp_common).strip()}->{f(epoch.last_pp_common).strip()} "
            f"delta={fmt_delta(epoch.first_pp_common, epoch.last_pp_common)} ppb; "
            f"PV60 {f(epoch.first_pv).strip()}->{f(epoch.last_pv).strip()} "
            f"delta={fmt_delta(epoch.first_pv, epoch.last_pv)} ppb",
            flush=True,
        )
        print(
            f"  DAC: O1 {epoch.first_dac['OCXO1']:.6f}->{epoch.last_dac['OCXO1']:.6f} "
            f"delta={epoch.last_dac['OCXO1']-epoch.first_dac['OCXO1']:+.6f}; "
            f"O2 {epoch.first_dac['OCXO2']:.6f}->{epoch.last_dac['OCXO2']:.6f} "
            f"delta={epoch.last_dac['OCXO2']-epoch.first_dac['OCXO2']:+.6f}",
            flush=True,
        )
        print(
            f"  reference: PV60 {format_range(epoch.pv)}; "
            f"REF-CPS cycles {format_range(epoch.ref_minus_cps)}; "
            f"V-P cycles {format_range(epoch.v_minus_p)}",
            flush=True,
        )
        for key, label, decimals in (
            ("ambient_temp_c", "ambient C", 4),
            ("gnss_temp_c", "GNSS C", 4),
            ("pi_temp_c", "Pi C", 4),
            ("pressure_hpa", "pressure hPa", 4),
            ("gnss_drift_ppb", "GNSS drift ppb", 4),
            ("pps_timing_error_ns", "PPS error ns", 4),
            ("pi_power_volts", "Pi rail V", 5),
            ("teensy_power_volts", "Teensy rail V", 5),
            ("ocxo1_power_volts", "OCXO1 rail V", 6),
            ("ocxo1_power_amps", "OCXO1 rail A", 6),
            ("ocxo1_power_watts", "OCXO1 rail W", 6),
            ("ocxo2_power_volts", "OCXO2 rail V", 6),
            ("ocxo2_power_amps", "OCXO2 rail A", 6),
            ("ocxo2_power_watts", "OCXO2 rail W", 6),
            ("battery_remaining_pct", "battery pct", 4),
        ):
            moments = epoch.context[key]
            if moments.n:
                print(f"  {label}: {format_range(moments, decimals)}", flush=True)
        print(flush=True)


# -----------------------------------------------------------------------------
# CLI / execution
# -----------------------------------------------------------------------------


def parse(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stream physical-PPS / VCLOCK / OCXO reference testimony to locate "
            "the source of common-mode PPB relaxation."
        )
    )
    parser.add_argument("campaign", help="TEMPEST campaign name, e.g. PPB2")
    parser.add_argument(
        "--skip",
        type=int,
        default=0,
        help="begin after campaign.public_count N (range predicate; not SQL OFFSET)",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="maximum rows to stream; 0 means through campaign end",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=DEFAULT_BATCH_SIZE,
        help=f"named server-cursor fetch size (default {DEFAULT_BATCH_SIZE})",
    )
    parser.add_argument(
        "--every",
        type=int,
        default=DEFAULT_DISPLAY_EVERY,
        help=f"display cadence in campaign seconds after reset detail (default {DEFAULT_DISPLAY_EVERY})",
    )
    parser.add_argument(
        "--reset-detail",
        type=int,
        default=DEFAULT_RESET_DETAIL_ROWS,
        help=f"print every row for first N updates after STATS_RESET (default {DEFAULT_RESET_DETAIL_ROWS})",
    )
    args = parser.parse_args(argv[1:])
    if args.skip < 0 or args.limit < 0:
        parser.error("--skip/--limit must be nonnegative")
    if args.batch_size <= 0 or args.every <= 0 or args.reset_detail < 0:
        parser.error("--batch-size/--every must be positive and --reset-detail nonnegative")
    return args


def main(argv: Sequence[str]) -> None:
    args = parse(argv)
    print_header(args.campaign, args.batch_size, args.every)

    state = CourtState()
    previous_count: Optional[int] = None
    previous_reset: Optional[int] = None
    epochs: list[Epoch] = []
    epoch: Optional[Epoch] = None
    processed = 0
    displayed = 0

    for (
        db_id,
        count,
        campaign,
        anchor,
        raw,
        stats,
        control,
        environment,
        gnss,
    ) in iter_rows(
        args.campaign,
        skip=args.skip,
        limit=args.limit,
        batch_size=args.batch_size,
    ):
        row = build_row(
            db_id,
            count,
            campaign,
            anchor,
            raw,
            stats,
            control,
            environment,
            gnss,
            previous_count,
        )
        processed += 1

        reset_changed = previous_reset is not None and row.reset_count != previous_reset
        if previous_reset is None or reset_changed:
            if reset_changed:
                # STATS_RESET is not a physical continuity break, so the rolling
                # reference court intentionally survives it.  Only the epoch
                # accounting changes.
                pass
            epoch = Epoch(row.reset_count, row.count, row.count)
            epochs.append(epoch)

        derived, notes = update_state(state, row)
        if reset_changed:
            notes.insert(0, f"STATS_RESET {previous_reset}->{row.reset_count}")

        if epoch is None:
            raise RuntimeError("reference court epoch was not initialized")
        note_epoch(epoch, row, derived)

        dac_changed = any(
            state.previous_dac[name] is not None
            and abs(row.dac[name] - float(state.previous_dac[name])) > DAC_CHANGE_EPSILON
            for name in LANES
        )
        if dac_changed:
            notes.append("DAC_CHANGE")

        for name in LANES:
            state.previous_dac[name] = row.dac[name]

        detail = row.update_count <= args.reset_detail
        cadence = (count % args.every) == 0
        abnormal = bool(notes) or row.gap or row.disposition != "ACCEPT" or not row.timeline_valid
        if detail or cadence or reset_changed or dac_changed or abnormal:
            print_row(row, derived, notes)
            displayed += 1

        previous_count = count
        previous_reset = row.reset_count

    if processed == 0:
        raise RuntimeError(f"No TEMPEST rows found for campaign {args.campaign!r}")

    print_summaries(epochs)
    print(
        f"Processed {processed:,} rows through a named server cursor; "
        f"displayed {displayed:,} rows.",
        flush=True,
    )


if __name__ == "__main__":
    main(sys.argv)
