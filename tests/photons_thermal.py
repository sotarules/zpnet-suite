"""Infer recorded temperature or ADC HIGH compensation for one PHOTONS campaign.

    zt photons_thermal External1
    zt photons_thermal HeatCool1 --sensor high --max-degree 1
    zt photons_thermal External1 --sensor compare --max-degree 1
    zt photons_thermal External1 --max-lag-seconds 600
    python3 photons_thermal.py External1 --input External1.jsonl

Prints the complete report to STDOUT; no files or database values are changed.
Uses recorded temperature and one-second mean lap, never cumulative means.
--sensor selects synthetic (default), bme280, bme280_receiver, bme280_out,
max31865, compare, or compare-legacy.
Current BME280 means come from environment.pi/pd/out in the recorded SYSTEM
snapshot. MAX31865 uses the deployed aggregate's recorded rtd component.
Legacy rows use the PI component and top-level receiver; OUT is not invented.
Missing selected readings are omitted; no new smoothing or weight reconstruction.
Compare uses all five predictors on common rows/blocks/splits; compare-legacy
uses synthetic, PI BME280 and MAX31865 without requiring PD or OUT.
Each sensor's degree/lag is chosen on development data, not on the final test.
New rows read the immutable raw one-second distribution directly and compare
it with the separately recorded Pi correction. Legacy rows reconstruct raw LAP
using their recorded firmware offset. New models always fit raw LAP.
Models: constant, linear, quadratic, cubic. Model selection uses forward-only
validation within the first 70%; the last 30% tests the frozen choice once.
The default uses current temperature only. Optional lag uses past temperatures
on complete, uninterrupted blocks. Positive lag means temperature BEFORE lap.

--sensor high pools exact ADC sums/counts and accepted-race-weighted raw LAP
over the SAME nonoverlapping consecutive fragments. --bin-seconds is the nominal
block width in one-second fragments; actual firmware durations are reported.
HIGH60 is not averaged again and no temperature context is required. HIGH units
are nominal mV (3300/4095), not calibrated voltage. Both physical ADC samples and
accepted races retain their own counts; they are not assumed to be paired events.
HIGH analysis uses zero lag. Direction means rising/falling voltage, not measured
warming/cooling. Full-campaign direction tables are descriptive, including fit
data; only the chronological final test measures held-out prediction.

Standard library only, plus zpnet.shared.db when reading the database.
main(argv) includes the program name, as required by the zt runner.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics as st
import sys
from collections import Counter
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


SCHEMA = "PHOTONS_V1"
MIN_FIT = 3
VERSION = "2026-10-18.1"
HIGH_REFERENCE_MV = 3300
HIGH_MV_PER_COUNT = HIGH_REFERENCE_MV / 4095
SENSORS = ("synthetic", "bme280", "bme280_receiver", "bme280_out", "max31865")
LEGACY_SENSORS = ("synthetic", "bme280", "max31865")
SENSOR_LABELS = {"high": "PD OUT HIGH", "synthetic": "Synthetic temperature",
                 "bme280": "BME280 under Pi (I2C1/0x76)",
                 "bme280_receiver": "BME280 near PD200T (I2C1/0x77)",
                 "bme280_out": "BME280 OUT (I2C2/0x77)",
                 "max31865": "MAX31865 under fiber pad"}
SENSOR_COMPONENTS = {"bme280": ("environment", "PI.I2C1.BME280.0x76"),
                     "bme280_receiver": ("receiver", "PI.I2C1.BME280.0x77"),
                     "bme280_out": ("out", "PI.I2C2.BME280.0x77"),
                     "max31865": ("rtd", "TEENSY.SPI1.MAX31865")}
BME280_KEYS = {"bme280": "pi", "bme280_receiver": "pd", "bme280_out": "out"}



def utc(value: Any) -> float:
    dt = value if isinstance(value, datetime) else datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if dt.tzinfo is None:
        raise ValueError(f"timestamp requires a timezone: {value!r}")
    return dt.timestamp()


def stamp(value: float) -> str:
    return datetime.fromtimestamp(value, timezone.utc).isoformat(timespec="seconds")


def number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"expected a finite number, got {value!r}")
    return float(value)


def integer(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"expected a nonnegative integer, got {value!r}")
    return value


def signed_integer(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not -2**31 <= value < 2**31:
        raise ValueError(f"expected a signed 32-bit integer, got {value!r}")
    return value


@dataclass(frozen=True)
class Sample:
    row_id: int
    time: float
    sequence: int
    reset: int
    pps: Optional[int]
    temperature: float  # predictor: C for thermal modes, mV for --sensor high
    lap_ns: float
    races: int
    signature: str
    update: Optional[int]
    generation: Optional[int]
    receipt: bool
    canonical_ns: float
    normalization_id: Optional[int | str] = None
    correction_ns: float = 0.0
    raw_source: str = "LEGACY"
    applied_temperature: Optional[float] = None
    correction_age_ms: Optional[int] = None
    sensor_temperatures: Tuple[Tuple[str, float], ...] = ()
    adc_count: Optional[int] = None
    adc_sum: Optional[int] = None
    adc_squares: Optional[int] = None
    fragment_elapsed_ms: Optional[int] = None
    epoch: int = 0
    run: int = 0


def sensor_reading(p: Dict[str, Any], thermal: Dict[str, Any], name: str):
    """Select the recorded layout explicitly; never fill a missing modern sensor."""
    if name == "synthetic":
        return thermal, ["temperature"]
    if name in BME280_KEYS:
        environment = p.get("environment")
        if environment is not None:
            if not isinstance(environment, dict):
                raise ValueError("environment must be an object or null")
            if "schema" not in environment:
                key = BME280_KEYS[name]
                return environment.get(key), ["environment", key]
            if environment["schema"] != "BME280_V1":
                raise ValueError("unsupported legacy environment schema")
        # Historical records predate the grouped environmental snapshot.
        if name == "bme280_receiver":
            return p.get("receiver"), ["receiver"]
        if name == "bme280_out":
            return None, ["environment", "out"]
    component = SENSOR_COMPONENTS[name][0]
    return (thermal.get("components") or {}).get(component), ["temperature", "components", component]


def parse_sample(row_id: int, p: Dict[str, Any], sensor: str = "synthetic") -> Tuple[Optional[Sample], str]:
    """Optional environmental context may be absent; science remains strict."""
    if p["schema"] != SCHEMA:
        raise ValueError(f"expected {SCHEMA}")
    ph = p["photons"]
    if ph["fragment_period_ns"] != 1_000_000_000:
        raise ValueError("thermal analysis requires one-second fragments")
    if ph["snapshot_ok"] is not True:
        return None, "snapshot_not_ok"
    adc_count = adc_sum = adc_squares = elapsed_ms = None
    if sensor == "high":
        pd = ph.get("pd_out")
        if pd is None:
            return None, "high_not_recorded"
        adc_count = integer(pd["sample_count"])
        adc_sum = integer(pd["high_sum_counts"])
        adc_squares = integer(pd["high_sum_squares_counts"])
        if integer(pd["adc_bits"]) != 12:
            raise ValueError("HIGH requires recorded 12-bit ADC codes")
        if (adc_sum > adc_count * 4095 or adc_squares > adc_sum * 4095 or
                adc_count * adc_squares < adc_sum * adc_sum):
            raise ValueError("HIGH raw moments do not describe a 12-bit population")
        if "high_window" in pd and pd["high_window"]["reference_mv"] != HIGH_REFERENCE_MV:
            raise ValueError("HIGH nominal reference differs from the published window")
        if adc_count == 0:
            return None, "high_no_samples"
        elapsed_ms = integer(ph["fragment_elapsed_ms"])
        if elapsed_ms == 0:
            raise ValueError("HIGH fragment duration must be positive")
        selected = ("high",)
        readings = {"high": adc_sum / adc_count * HIGH_MV_PER_COUNT}
        predictor_signature = {
            "path": ["photons", "pd_out"], "reference_mv": HIGH_REFERENCE_MV,
            "acquisition": {key: pd[key] for key in (
                "adc_bits", "acquisition", "samples_per_capture", "published_high_sample",
                "high_delay_target_ns", "dwt_cycles_per_second")},
        }
    else:
        # Use only this row's recorded SYSTEM snapshot. The current grouped BME280
        # readings and composite components are authored from the same observation.
        thermal = p.get("temperature")
        if not thermal:
            return None, "temperature_context_not_recorded"
        if not isinstance(thermal, dict):
            raise ValueError("temperature must be an object or null")
        if (thermal["schema"] != "SYNTHETIC_TEMPERATURE_V1"
                or thermal["source"] != "PI.SYSTEM.TEMPERATURE"):
            raise ValueError("unsupported synthetic temperature source/schema")
        model_id = integer(thermal["model_id"])
        if not 0 < model_id < 2**32:
            raise ValueError("synthetic temperature requires a nonzero uint32 model identity")
        selected = (SENSORS if sensor == "compare" else
                    LEGACY_SENSORS if sensor == "compare-legacy" else (sensor,))
        readings = {}
        selected_items = {}
        predictor_settings = {}
        for name in selected:
            item, path = sensor_reading(p, thermal, name)
            if item is None:
                return None, name + "_not_recorded"
            if not isinstance(item, dict):
                raise ValueError(f"{name} reading must be an object")
            if name == "synthetic":
                settings = {"source": thermal["source"], "path": path}
            else:
                source = SENSOR_COMPONENTS[name][1]
                if item["source"] != source:
                    raise ValueError(f"unexpected {name} source: {item['source']}")
                if path[0] == "environment" and item["schema"] != "BME280_V1":
                    raise ValueError(f"unexpected {name} schema: {item['schema']}")
                settings = {"source": source, "path": path,
                            "window_samples": item["smoothing"]["window_samples"]} if item["status"] == "OK" else {}
            if item["status"] != "OK":
                return None, name + "_unavailable"
            integer(item["age_ms"])
            readings[name] = number(item["temperature_c"])
            selected_items[name] = item
            predictor_settings[name] = settings
        predictor_signature = {"schema": thermal["schema"], "source": thermal["source"],
                               "model_id": model_id, "selected": predictor_settings}
    temperature = readings[selected[0]]
    race = ph["race"]
    flight = race["flight_ns"]
    n = integer(flight["n"])
    if n == 0:
        return None, "no_accepted_races"
    if race["active"] is not True:
        return None, "autonomous_races_inactive"
    core = ph.get("core") or {}
    if core and integer(core["sequence"]) != integer(p["sequence"]):
        raise ValueError("core diagnostic belongs to a different fragment")
    if core and integer(core["retained_count"]) != n:
        raise ValueError("core retained count does not match flight population")
    stats = ph["stats"]
    canonical_ns = number(flight["mean"])
    normalization_id = None
    correction_fs = 0
    applied_temperature = correction_age_ms = None
    policy = p.get("normalization")
    fields = ("normalization_id", "normalization_correction_fs",
              "normalization_temperature_micro_c", "normalization_update_age_ms")
    # SQL emits nulls for absent legacy fields. Partial normalization testimony
    # must fail loudly rather than silently treating corrected LAP as raw.
    measurement = p.get("measurement")
    if measurement is None and (policy is not None or any(stats.get(k) is not None for k in fields)):
        normalization_id = integer(stats["normalization_id"])
        if not 0 < normalization_id < 2**32:
            raise ValueError("normalized race population requires a nonzero uint32 calibration identity")
        correction_fs = signed_integer(stats["normalization_correction_fs"])
        applied_temperature = signed_integer(stats["normalization_temperature_micro_c"]) / 1_000_000
        correction_age_ms = integer(stats["normalization_update_age_ms"])
        if policy is not None:
            if not isinstance(policy, dict) or policy.get("normalization_id") != normalization_id:
                raise ValueError("normalization policy and firmware identity disagree")
            if (policy.get("schema") != "PHOTONS_THERMAL_NORMALIZATION_V1" or
                    policy.get("application") != "UNIFORM_OFFSET_AT_FRAGMENT_DRAIN"):
                raise ValueError("unsupported normalization reconstruction contract")
    if sensor != "high" and policy is not None and policy.get("temperature_path") == ["temperature", "temperature_c"]:
        if policy["temperature_model_id"] != model_id:
            raise ValueError("recorded synthetic temperature and normalization model disagree")
        maximum_age = integer(policy["maximum_sample_age_ms"])
        for name in selected:
            item = selected_items[name]
            if item["age_ms"] > maximum_age:
                return None, name + "_stale"
    # Undo exactly the applied offset; never recompute it from SYSTEM temperature.
    correction_ns = correction_fs / 1_000_000
    uncorrected_ns = canonical_ns + correction_ns
    raw_source = "LEGACY_RECONSTRUCTED" if normalization_id is not None else "LEGACY"
    if measurement is not None:
        # Firmware NOR1 publishes a normalized LAP plus a preserved raw witness.
        firmware = ph.get("voltage_normalization")
        identity = 0x4E4F5231 if firmware is not None else 0x52415731
        if (stats["acquisition_id"] != identity or
                measurement["acquisition_id"] != identity):
            raise ValueError("unsupported PHOTONS measurement ancestry")
        raw = measurement["raw_flight_ns"]
        normalized = measurement["normalized_flight_ns"]
        if firmware is not None:
            if raw != firmware["raw_flight_ns"] or normalized != flight:
                raise ValueError("firmware raw/normalized evidence does not match the recorded populations")
            if measurement["normalization_id"] != firmware["model_id"]:
                raise ValueError("firmware normalization model identity mismatch")
            raw_source = "DIRECT_RAW_FIRMWARE_NORMALIZED"
        else:
            if raw != flight:
                raise ValueError("Pi raw evidence differs from the firmware distribution")
            raw_source = "DIRECT_RAW"
        uncorrected_ns = number(raw["mean"])
        if normalized is not None:
            if normalized["n"] != n or normalized["m2"] != raw["m2"]:
                raise ValueError("normalization changed the raw population/dispersion")
            correction_ns = number(measurement["correction_ns"])
            canonical_ns = number(normalized["mean"])
            if abs(uncorrected_ns - correction_ns - canonical_ns) > 1e-9:
                raise ValueError("Pi correction does not close with raw and normalized means")
            normalization_id = measurement["normalization_id"]
        if measurement.get("normalization_id") is not None:
            if policy is None or policy.get("normalization_id") != measurement["normalization_id"]:
                raise ValueError("Pi normalization policy/measurement identity mismatch")
        elif policy is not None or normalized is not None or measurement["correction_ns"] is not None:
            raise ValueError("uncalibrated PHOTONS must not claim normalized science")
    # Per-second center/radius/state are data, not configuration changes.
    signature = json.dumps({
        "race": {k: race.get(k) for k in (
            "schema", "cadence_ns", "pulse_ns", "capture_gate", "capture_min_ns", "capture_max_ns",
            "launch_bracket_max_cycles", "flight_interpretation", "launch_timing_policy")},
        "core": {k: core.get(k) for k in (
            "algorithm", "minimum_count", "minimum_radius_cycles", "mad_multiplier", "minimum_retained_pct")},
        "normalization": {"id": normalization_id, "policy": policy},
        "predictor": predictor_signature,
    }, sort_keys=True)
    projection = ph.get("projection") or {}
    pps = projection.get("anchor_pps_count")
    update = ph["stats"].get("update_count")
    generation = (ph.get("recovery") or {}).get("generation")
    return Sample(
        row_id=row_id, time=utc(p["published_at_utc"]), sequence=integer(p["sequence"]),
        reset=integer(ph["stats"]["reset_count"]), pps=None if pps is None else integer(pps),
        temperature=temperature, lap_ns=uncorrected_ns, races=n,
        signature=signature,
        update=None if update is None else integer(update),
        generation=None if generation is None else integer(generation),
        receipt=p.get("recovery_receipt") is not None,
        canonical_ns=canonical_ns, normalization_id=normalization_id,
        correction_ns=correction_ns, raw_source=raw_source, applied_temperature=applied_temperature,
        correction_age_ms=correction_age_ms, sensor_temperatures=tuple(readings.items()),
        adc_count=adc_count, adc_sum=adc_sum, adc_squares=adc_squares,
        fragment_elapsed_ms=elapsed_ms,
    ), "used"


def database_rows(args: argparse.Namespace) -> Iterable[Tuple[int, Dict[str, Any]]]:
    from zpnet.shared.db import open_db

    # Project just the analysis surface; do not transfer large recovery rings
    # or cumulative science populations. Keep the exact campaign predicate.
    sql = """
        SELECT id, jsonb_build_object(
            'schema', payload -> 'schema',
            'sequence', payload -> 'sequence',
            'published_at_utc', payload -> 'published_at_utc',
            'temperature', payload -> 'temperature',
            'environment', payload -> 'environment',
            'receiver', payload -> 'receiver',
            'normalization', payload -> 'normalization',
            'measurement', payload -> 'measurement',
            'recovery_receipt', payload -> 'recovery_receipt',
            'photons', jsonb_build_object(
                'snapshot_ok', payload #> '{photons,snapshot_ok}',
                'fragment_period_ns', payload #> '{photons,fragment_period_ns}',
                'fragment_elapsed_ms', payload #> '{photons,fragment_elapsed_ms}',
                'pd_out', payload #> '{photons,pd_out}',
                'stats', jsonb_build_object(
                    'reset_count', payload #> '{photons,stats,reset_count}',
                    'update_count', payload #> '{photons,stats,update_count}',
                    'acquisition_id', payload #> '{photons,stats,acquisition_id}',
                    'normalization_id', payload #> '{photons,stats,normalization_id}',
                    'normalization_correction_fs', payload #> '{photons,stats,normalization_correction_fs}',
                    'normalization_temperature_micro_c', payload #> '{photons,stats,normalization_temperature_micro_c}',
                    'normalization_update_age_ms', payload #> '{photons,stats,normalization_update_age_ms}'),
                'recovery', jsonb_build_object('generation', payload #> '{photons,recovery,generation}'),
                'projection', jsonb_build_object('anchor_pps_count', payload #> '{photons,projection,anchor_pps_count}'),
                'core', payload #> '{photons,core}',
                'race', jsonb_build_object(
                    'active', payload #> '{photons,race,active}',
                    'flight_ns', payload #> '{photons,race,flight_ns}',
                    'schema', payload #> '{photons,race,schema}',
                    'cadence_ns', payload #> '{photons,race,cadence_ns}',
                    'pulse_ns', payload #> '{photons,race,pulse_ns}',
                    'capture_gate', payload #> '{photons,race,capture_gate}',
                    'capture_min_ns', payload #> '{photons,race,capture_min_ns}',
                    'capture_max_ns', payload #> '{photons,race,capture_max_ns}',
                    'launch_bracket_max_cycles', payload #> '{photons,race,launch_bracket_max_cycles}',
                    'flight_interpretation', payload #> '{photons,race,flight_interpretation}',
                    'launch_timing_policy', payload #> '{photons,race,launch_timing_policy}'
                )
            )
        ) AS thermal_payload
        FROM campaign_detail
        WHERE campaign_type = %s AND campaign = %s AND payload ->> 'schema' = %s
        ORDER BY id ASC
    """
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="photons_thermal")
        cur.itersize = args.batch_size
        cur.execute(sql, ("LANTERN", args.campaign, SCHEMA))
        for row in cur:
            p = row["thermal_payload"]
            yield int(row["id"]), json.loads(p) if isinstance(p, str) else p


def file_rows(path: Path, campaign: str) -> Iterable[Tuple[int, Dict[str, Any]]]:
    text = path.read_text(encoding="utf-8-sig").strip()
    if text.startswith("["):
        records = json.loads(text)
    else:
        if text.startswith("PHOTONS "):
            text = text[len("PHOTONS "):]
        try:
            records = [json.loads(text)]
        except json.JSONDecodeError:
            records = [json.loads(line.removeprefix("PHOTONS ")) for line in text.splitlines() if line.strip()]
    for i, record in enumerate(records, 1):
        p = record.get("payload", record)
        if p.get("schema") == SCHEMA and (p.get("campaign") or {}).get("campaign") == campaign:
            yield i, p


def collect(rows: Iterable[Tuple[int, Dict[str, Any]]], args: argparse.Namespace):
    samples: List[Sample] = []
    counts: Counter = Counter()
    reasons: Dict[int, str] = {}
    prev: Optional[Sample] = None
    epoch = run = 0
    origin = None
    for row_id, p in rows:
        counts["rows_read"] += 1
        try:
            t = utc(p["published_at_utc"])
            if origin is None:
                origin = t
            if ((args.start is not None and t < args.start) or
                    (args.end is not None and t >= args.end) or t < origin + args.skip_seconds):
                counts["outside_selected_time"] += 1
                continue
            sample, disposition = parse_sample(row_id, p, args.sensor)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"row {row_id}: {exc}") from exc
        if sample is None:
            counts[disposition] += 1
            if counts[disposition] == 1 and disposition.endswith(("_not_recorded", "_unavailable", "_stale")):
                print(f"  {args.campaign}: first {disposition} at row {row_id}; omitted from thermal pairs.")
            continue
        changes = []
        if prev is not None:
            if sample.receipt:
                changes.append("producer recovery receipt")
            if sample.generation != prev.generation:
                changes.append("recovery generation changed")
            if sample.reset != prev.reset:
                changes.append("statistics reset")
            # Only consecutive, same-generation republications are duplicates.
            # A reboot may reuse old counters: never globally deduplicate them.
            identity = (sample.reset, sample.sequence, sample.pps, sample.update)
            previous_identity = (prev.reset, prev.sequence, prev.pps, prev.update)
            same_generation = sample.generation == prev.generation
            repeated_receipt = not sample.receipt or prev.receipt
            if same_generation and repeated_receipt and identity == previous_identity:
                if (sample.lap_ns, sample.canonical_ns, sample.correction_ns,
                    sample.applied_temperature, sample.correction_age_ms, sample.races, sample.signature,
                    sample.adc_count, sample.adc_sum, sample.adc_squares,
                    sample.fragment_elapsed_ms) != (
                    prev.lap_ns, prev.canonical_ns, prev.correction_ns,
                    prev.applied_temperature, prev.correction_age_ms, prev.races, prev.signature,
                    prev.adc_count, prev.adc_sum, prev.adc_squares,
                    prev.fragment_elapsed_ms):
                    raise ValueError(f"row {row_id}: repeated fragment has conflicting science/settings")
                counts["duplicate_fragment"] += 1
                continue
            if sample.signature != prev.signature:
                changes.append("acquisition/core settings, normalization policy or predictor settings/path changed")
            if (sample.sequence <= prev.sequence or
                    (sample.pps is not None and prev.pps is not None and sample.pps < prev.pps) or
                    (sample.update is not None and prev.update is not None and sample.update <= prev.update)):
                changes.append("sequence/PPS/update restart")
            if sample.time <= prev.time:
                changes.append("publication time moved backward")
        if prev is None or changes:
            epoch += 1
            run += 1
            reasons[epoch] = "; ".join(changes) or "start of selected observations"
        else:
            counter_gap = (sample.sequence != prev.sequence + 1 or
                           (sample.update is not None and prev.update is not None and
                            sample.update != prev.update + 1))
            publication_gap = not 0.25 <= sample.time - prev.time <= 1.75
            if counter_gap or publication_gap:
                run += 1
                counts["continuity_breaks"] += 1
                counts["counter_gap_breaks"] += int(counter_gap)
                counts["publication_timing_breaks"] += int(publication_gap)
        sample = replace(sample, epoch=epoch, run=run)
        samples.append(sample)
        prev = sample
        counts["used"] += 1
        counts["direct_raw" if sample.raw_source == "DIRECT_RAW" else
               "reconstructed_uncorrected" if sample.normalization_id is not None else "original_uncorrected"] += 1
    return samples, counts, reasons


@dataclass(frozen=True)
class Point:
    run: int
    index: int
    time: float
    temperature: float
    lap_ns: float
    elapsed_ms: Optional[int] = None


def blocks(samples: Sequence[Sample], width: int, high: bool = False) -> Tuple[List[Point], int]:
    out = []
    bucket: List[Sample] = []
    index = 0
    used = 0
    for s in samples:
        if bucket and s.run != bucket[0].run:
            bucket = []
            index = 0
        elif out and not bucket and s.run != out[-1].run:
            index = 0
        bucket.append(s)
        if len(bucket) == width:
            if high:
                # Exact raw-code pooling, not a mean of rounded means or HIGH60.
                # Lap and HIGH cover the same fragments but different populations.
                predictor = sum(v.adc_sum for v in bucket) / sum(v.adc_count for v in bucket) * HIGH_MV_PER_COUNT
                lap = math.fsum(v.lap_ns * v.races for v in bucket) / sum(v.races for v in bucket)
                elapsed_ms = sum(v.fragment_elapsed_ms for v in bucket)
            else:
                predictor = st.fmean(v.temperature for v in bucket)
                lap = st.fmean(v.lap_ns for v in bucket)
                elapsed_ms = None
            out.append(Point(s.run, index, st.fmean(v.time for v in bucket), predictor, lap, elapsed_ms))
            index += 1
            used += width
            bucket = []
    return out, len(samples) - used


def centered(values: Sequence[float]) -> List[float]:
    avg = st.fmean(values)
    return [v - avg for v in values]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return math.fsum(x * y for x, y in zip(a, b))


def pearson(x: Sequence[float], y: Sequence[float]) -> Optional[float]:
    if len(x) < MIN_FIT:
        return None
    a, b = centered(x), centered(y)
    xx, yy = dot(a, a), dot(b, b)
    return max(-1.0, min(1.0, dot(a, b) / math.sqrt(xx * yy))) if xx > 1e-24 and yy > 1e-24 else None


def fmt(v: Optional[float], digits: int = 4) -> str:
    return "n/a" if v is None else f"{v:.{digits}f}"


@dataclass(frozen=True)
class Polynomial:
    reference: float
    coefficients: Tuple[float, ...]  # powers of (temperature - reference), ns/C^k
    low: float
    high: float

    def predict(self, temperature: float) -> float:
        value = 0.0
        for coefficient in reversed(self.coefficients):
            value = value * (temperature - self.reference) + coefficient
        return value

    def correction(self, temperature: float) -> float:
        return self.predict(temperature) - self.coefficients[0]


def polynomial(x, y, degree):
    """Small least squares fit, scaled and solved by reorthogonalized QR.

    A rank-deficient temperature population cannot identify this degree; that
    candidate is omitted explicitly rather than filled with guessed coefficients.
    """
    reference = st.fmean(x)
    scale = max(abs(t-reference) for t in x)
    if degree and scale < 1e-12:
        return None
    scale = scale or 1.0
    z = [(t-reference)/scale for t in x]
    columns = [[t**j for t in z] for j in range(degree+1)]
    q = []
    r = [[0.0]*(degree+1) for _ in range(degree+1)]
    for j, column in enumerate(columns):
        v = column[:]
        for _ in range(2):
            for i, basis in enumerate(q):
                projection = dot(basis, v)
                r[i][j] += projection
                v = [a-projection*b for a,b in zip(v,basis)]
        norm = math.sqrt(dot(v,v))
        if norm < 1e-10*math.sqrt(len(x)):
            return None
        r[j][j] = norm
        q.append([a/norm for a in v])
    y0 = st.fmean(y)
    rhs = [dot(basis, [v-y0 for v in y]) for basis in q]
    coefficients = [0.0]*(degree+1)
    for j in reversed(range(degree+1)):
        coefficients[j] = (rhs[j]-math.fsum(r[j][k]*coefficients[k]
                            for k in range(j+1,degree+1)))/r[j][j]
    coefficients[0] += y0
    coefficients = tuple(c/scale**j for j,c in enumerate(coefficients))
    return Polynomial(reference, coefficients, min(x), max(x))


def quantile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered)-1)*fraction
    i = int(position)
    return ordered[i] + (ordered[min(i+1,len(ordered)-1)]-ordered[i])*(position-i)


def rmse(errors):
    return math.sqrt(st.fmean(e*e for e in errors))


def reduction(before, after):
    return 'n/a' if before == 0 else f'{100*(1-after/before):+.2f}%'


def spread(label, values):
    sd = st.stdev(values) if len(values)>1 else 0.0
    print(f'  {label:25s} {len(values):8,d} {st.fmean(values):14.6f} '
          f'{sd*1000:12.3f} {(max(values)-min(values))*1000:12.3f} '
          f'{(quantile(values,.95)-quantile(values,.05))*1000:12.3f}')


def spread_header():
    print('  series                           N        mean_ns        SD_ps       p-p_ps     P95-P5_ps')


def name(degree, lag, width):
    return f'{("constant", "linear", "quadratic", "cubic")[degree]} / lag {lag*width}s'


def formula(model, lag_seconds, sensor="synthetic"):
    unit = "mV" if sensor == "high" else "C"
    symbol = "HIGH_mV" if sensor == "high" else "T_C"
    predictor = "HIGH" if sensor == "high" else "temperature"
    print(f'  reference = {model.reference:.9f} {unit}; x = {symbol} - reference')
    print(f'  {symbol} is the {"current" if not lag_seconds else str(lag_seconds)+" seconds earlier"} block {predictor}.')
    expression = f'{model.coefficients[0]:.12g}'
    for j,c in enumerate(model.coefficients[1:],1):
        expression += f' {c:+.12g}*x' + (f'^{j}' if j>1 else '')
    print(f'  predicted_LAP_ns = {expression}')
    correction = ' '.join(f'{c:+.12g}*x'+(f'^{j}' if j>1 else '')
                          for j,c in enumerate(model.coefficients[1:],1)) or '0'
    print(f'  candidate_normalized_LAP_ns = uncorrected_LAP_ns - ({correction})')
    print('  Apply this candidate to uncorrected LAP, never on top of an existing correction.')
    print(f'  Fitted {predictor} range: {model.low:.6f} .. {model.high:.6f} {unit}')
    for j,c in enumerate(model.coefficients):
        print(f'    coefficient c{j} = {c:.12g} ns'+(f'/{unit}^{j}' if j else ''))
    if len(model.coefficients)>1:
        print(f'  Sensitivity at reference: {model.coefficients[1]*1000:+.3f} ps/{unit}')


def shape(points, x, residual, args, scope="FINAL TEST"):
    high_mode = args.sensor == "high"
    unit = "mV" if high_mode else "C"
    column = "HIGH_mV" if high_mode else "mean_T_C"
    up, down = ("rising", "falling") if high_mode else ("warm", "cool")
    threshold = args.direction_threshold_mv_per_minute if high_mode else args.direction_threshold_c_per_minute
    minimum_width = args.high_bin_mv if high_mode else args.temperature_bin_c
    low, high = min(x), max(x)
    width = max(minimum_width, minimum_width*math.ceil((high-low)/(12*minimum_width)))
    origin = math.floor(low/width)*width
    groups = {}
    for i,t in enumerate(x):
        groups.setdefault(math.floor((t-origin)/width),[]).append(i)
    print(f'\n  {scope} PREDICTOR SHAPE ({width:g} {unit} bins; residual = actual - frozen prediction)')
    print(f'  {column:>11s}    blocks     mean_LAP_ns    residual_ps   residual_SD_ps')
    for _,indices in sorted(groups.items()):
        e = [residual[i] for i in indices]
        print(f'  {st.fmean(x[i] for i in indices):11.5f} {len(indices):9d} '
              f'{st.fmean(points[i].lap_ns for i in indices):15.6f} '
              f'{st.fmean(e)*1000:14.3f} '
              f'{fmt(st.stdev(e)*1000 if len(e)>1 else None,3):>16}')
    direction = {}
    for i in range(1,len(points)):
        a,b = points[i-1],points[i]
        if a.run == b.run and b.index == a.index+1:
            rate = (x[i]-x[i-1])*60/(b.time-a.time)
            direction[i] = up if rate>threshold else (down if rate < -threshold else 'steady')
    print(f'\n  {scope} {up.upper()} / {down.upper()} (predictor change; threshold={threshold:g} {unit}/minute)')
    if high_mode:
        print('  Falling HIGH may accompany heating; direction labels describe voltage, not temperature.')
    print(f'  Direction counts: {dict(Counter(direction.values()))}')
    print(f'  {column:>11s} {up+"_N":>8s} {down+"_N":>8s}   {up}-minus-{down} residual_ps')
    gaps = []
    for _,indices in sorted(groups.items()):
        warm = [residual[i] for i in indices if direction.get(i)==up]
        cool = [residual[i] for i in indices if direction.get(i)==down]
        if len(warm)>=2 and len(cool)>=2:
            gap = (st.fmean(warm)-st.fmean(cool))*1000
            gaps.append(gap)
            print(f'  {st.fmean(x[i] for i in indices):11.5f} {len(warm):8d} {len(cool):8d} {gap:32.3f}')
    if not gaps:
        print(f'  Insufficient overlapping {up}/{down} blocks for a direction comparison.')
    else:
        print(f'  Largest absolute direction gap: {max(map(abs,gaps)):.3f} ps.')
    print('  Residual shape or direction gaps identify structure not captured by the fitted predictor formula.')


def deployed_comparison(samples, points, canonical_points, sensor):
    """Describe the installed correction even when there is too little data to fit."""
    print('\n  DEPLOYED NORMALIZATION (same observations; no refitting)')
    print(f'  Normalization identity: {samples[0].normalization_id}')
    if samples[0].raw_source == 'DIRECT_RAW':
        print('  Raw LAP is stored directly; deployed Pi output is a separate measurement.')
    else:
        print('  Legacy raw LAP reconstructed by adding the recorded firmware offset.')
    policy = json.loads(samples[0].signature)['normalization']['policy']
    if policy is None:
        print('  Policy document absent; reconstruction uses the recorded uniform firmware offset.')
    elif samples[0].raw_source != 'DIRECT_RAW' and (sensor != 'synthetic' or policy.get('temperature_path') != ['temperature', 'temperature_c']):
        print('  Deployed predictor differs from the selected report sensor; recorded offsets still reconstruct LAP.')
    corrections = [s.correction_ns for s in samples]
    applied = [s.applied_temperature for s in samples]
    ages = [s.correction_age_ms for s in samples]
    print(f'  Recorded correction range: {min(corrections):+.6f} .. {max(corrections):+.6f} ns')
    if samples[0].raw_source != 'DIRECT_RAW':
        print(f'  Applied sensor range: {min(applied):.6f} .. {max(applied):.6f} C')
        print(f'  Correction refresh age: median={st.median(ages):.0f} ms; max={max(ages):,} ms (not physical sensor-reading age).')
    if sensor == 'synthetic' and policy is not None and policy.get('temperature_path') == ['temperature', 'temperature_c']:
        print(f'  Max |applied synthetic temperature - row SYSTEM synthetic temperature|: '
              f'{max(abs(s.applied_temperature-s.temperature) for s in samples):.6f} C')
    raw = [s.lap_ns for s in samples]
    canonical = [s.canonical_ns for s in samples]
    spread_header()
    spread('uncorrected one-second', raw)
    spread('deployed one-second', canonical)
    if len(raw)>1:
        print(f'  Deployed one-second SD reduction: {reduction(st.stdev(raw),st.stdev(canonical))}')
    print(f'  Deployed one-second peak-to-peak reduction: '
          f'{reduction(max(raw)-min(raw),max(canonical)-min(canonical))}')
    if points:
        spread('uncorrected block means', [p.lap_ns for p in points])
        spread('deployed block means', [p.lap_ns for p in canonical_points])
    temperatures = [s.temperature for s in samples]
    print(f'  Descriptive {SENSOR_LABELS[sensor]} correlation: uncorrected={fmt(pearson(temperatures,raw))}; '
          f'deployed={fmt(pearson(temperatures,canonical))}')
    print('  Descriptive comparison only; this does not establish calibration accuracy or causation.')
    print('\n  DEPLOYED NORMALIZATION IN SIX CHRONOLOGICAL WINDOWS (all usable seconds)')
    column = 'HIGH_mV' if sensor == 'high' else 'mean_T_C'
    print(f'    UTC start                    N  {column:>10s}  uncorrected_ns    deployed_ns  correction_ns')
    for i in range(6):
        window = samples[i*len(samples)//6:(i+1)*len(samples)//6]
        if window:
            print(f'    {stamp(window[0].time):25s} {len(window):5d} '
                  f'{st.fmean(s.temperature for s in window):11.5f} '
                  f'{st.fmean(s.lap_ns for s in window):15.6f} '
                  f'{st.fmean(s.canonical_ns for s in window):14.6f} '
                  f'{st.fmean(s.correction_ns for s in window):14.6f}')


def analyze(samples, reason, args):
    high = args.sensor == "high"
    unit = "mV" if high else "C"
    predictor = "HIGH" if high else "temperature"
    column = "HIGH_mV" if high else "mean_T_C"
    print(f'\nSENSOR: {SENSOR_LABELS[args.sensor]} (recorded mean)')
    print(f'\nEPOCH {samples[0].epoch}: {reason}')
    print(f'  UTC {stamp(samples[0].time)} .. {stamp(samples[-1].time)}')
    print(f'  Settings: {samples[0].signature}')
    print(f'  Usable seconds: {len(samples):,}; continuous runs: {len(set(s.run for s in samples)):,}')
    temperatures = [s.temperature for s in samples]
    print(f'  {SENSOR_LABELS[args.sensor]} range: {min(temperatures):.6f} .. {max(temperatures):.6f} {unit}')
    repeated = sum(a.run==b.run and a.temperature==b.temperature for a,b in zip(samples,samples[1:]))
    print(f'  Adjacent identical {SENSOR_LABELS[args.sensor]} readings: {repeated:,} (equal values need not be duplicate samples; overlapping means are time-correlated).')
    deployed = samples[0].normalization_id is not None
    print(f'  Raw source: {samples[0].raw_source}; deployed correction available: {deployed}.')
    spread_header()
    spread('uncorrected one-second', [s.lap_ns for s in samples])
    print(f'  Uncorrected lap range: {min(s.lap_ns for s in samples):.9f} .. {max(s.lap_ns for s in samples):.9f} ns')
    points, omitted = blocks(samples,args.bin_seconds,high)
    canonical_points, _ = blocks([replace(s,lap_ns=s.canonical_ns) for s in samples],args.bin_seconds,high)
    print(f'  Complete {args.bin_seconds}-fragment blocks: {len(points):,}; fragments in incomplete blocks: {omitted:,}')
    if high:
        print(f'  Physical ADC samples: {sum(s.adc_count for s in samples):,}; accepted races: {sum(s.races for s in samples):,}.')
        if points:
            print(f'  Actual block duration: {min(p.elapsed_ms for p in points)/1000:.3f} .. {max(p.elapsed_ms for p in points)/1000:.3f} s.')
        print('  HIGH is sample-weighted; LAP is accepted-race-weighted over the same fragments. Blocks have equal fit weight.')
    if deployed:
        deployed_comparison(samples,points,canonical_points,args.sensor)
    if len(points)<60:
        print('\n  MODEL SELECTION DEFERRED: need at least 60 complete blocks plus adequate later-test support.')
        print('  Collect longer runs across thermal cycles; smaller blocks do not create independent sensor observations.')
        return
    print(f'  Descriptive block correlations: {predictor} vs uncorrected LAP {fmt(pearson([p.temperature for p in points], [p.lap_ns for p in points]))}; '
          f'{predictor} vs time {fmt(pearson([p.temperature for p in points], [p.time for p in points]))}')
    lookup = {(p.run,p.index):p for p in points}
    steps = args.max_lag_seconds//args.bin_seconds
    targets = [p for p in points if p.index>=steps]
    print(f'  Common target blocks for every candidate lag: {len(targets):,}/{len(points):,}')
    if len(targets)<60:
        print('  Too few blocks with complete lag history; reduce --max-lag-seconds or collect longer runs.')
        return
    cut = 7*len(targets)//10
    development = targets[:cut]
    # Time embargo prevents blocks/history from touching the preceding fit.
    embargo = (steps+1)*args.bin_seconds
    test = [p for p in targets[cut:] if p.time-development[-1].time>embargo]
    folds = []
    for fraction in (.4,.6,.8):
        end = int(len(development)*fraction)
        stop = int(len(development)*(fraction+.2)+1e-8)
        train = development[:end]
        validation = [p for p in development[end:stop] if p.time-train[-1].time>embargo]
        if len(train)<10 or len(validation)<4:
            print('  Not enough forward-validation support after separation; shorten blocks or lag range.')
            return
        folds.append((train,validation))
    if len(test)<8:
        print('  Need at least 8 final test blocks after separation; reduce the block/lag duration.')
        return
    def xs(rows,lag):
        return [lookup[(p.run,p.index-lag)].temperature for p in rows]
    def ys(rows):
        return [p.lap_ns for p in rows]
    print('\n  MODEL SELECTION ON UNCORRECTED LAP (first 70% only; three expanding forward folds)')
    print(f'  Separation: >{embargo}s; final test: {len(test):,} blocks, not used to choose degree or lag.')
    for i,(train,val) in enumerate(folds,1):
        print(f'  Fold {i}: fit {len(train):,}, validate {len(val):,}; validation {stamp(val[0].time)} .. {stamp(val[-1].time)}')
    scored = []
    for degree in range(args.max_degree+1):
        for lag in (range(steps+1) if degree else [0]):
            errors = []
            fold_scores = []
            for train,val in folds:
                model = polynomial(xs(train,lag),ys(train),degree)
                if model is None:
                    break
                e = [p.lap_ns-model.predict(t) for p,t in zip(val,xs(val,lag))]
                errors.extend(e)
                fold_scores.append(rmse(e))
            if len(fold_scores)==3:
                scored.append((degree,lag,rmse(errors),fold_scores))
    print('  Best lag per degree; scores are prediction RMSE in ps:')
    print('    candidate                    pooled       fold1       fold2       fold3')
    best = []
    for degree in range(args.max_degree+1):
        entries = [entry for entry in scored if entry[0]==degree]
        if not entries:
            print(f'    degree {degree}: unidentifiable {predictor} population')
            continue
        winner = min(entries,key=lambda e:(e[2],e[1]))
        best.append(winner)
        print(f'    {name(*winner[:2],args.bin_seconds):27s} '+ ' '.join(f'{v*1000:11.3f}' for v in [winner[2],*winner[3]]))
    optimum = min(e[2] for e in scored)
    # The tolerance is a declared engineering simplicity rule, not a confidence interval.
    eligible = [e for e in scored if e[2]<=optimum*(1+args.simplicity_pct/100)+1e-12]
    chosen = min(eligible,key=lambda e:(e[0],e[1]))
    degree,lag,_,_ = chosen
    print(f'  Choose simplest degree, then shortest lag, within {args.simplicity_pct:g}% of best CV RMSE.')
    print(f'  FROZEN CHOICE: {name(degree,lag,args.bin_seconds)}')
    model = polynomial(xs(development,lag),ys(development),degree)
    assert model is not None
    print('\n  FROZEN CANDIDATE FORMULA (fitted on uncorrected development LAP only; no later-data recentering)')
    formula(model,lag*args.bin_seconds,args.sensor)
    tx, actual = xs(test,lag),ys(test)
    predictions = [model.predict(t) for t in tx]
    residual = [a-b for a,b in zip(actual,predictions)]
    normalized = [a-model.correction(t) for a,t in zip(actual,tx)]
    baseline = st.fmean(ys(development))
    baseline_errors = [y-baseline for y in actual]
    trend = polynomial([p.time for p in development],ys(development),1)
    print('\n  FINAL UNSEEN TEST (same blocks in every comparison)')
    print(f'  UTC {stamp(test[0].time)} .. {stamp(test[-1].time)}')
    outside = [i for i,t in enumerate(tx) if not model.low<=t<=model.high]
    print(f'  Predictor range: {min(tx):.6f} .. {max(tx):.6f} {unit}; outside fit range: {len(outside):,}/{len(test):,}')
    print('    prediction                       RMSE_ps        bias_ps     error_SD_ps')
    comparisons = [('constant development mean',baseline_errors),(f'selected {predictor} formula',residual)]
    if trend is not None:
        comparisons.append(('elapsed-time line (diagnostic)',[p.lap_ns-trend.predict(p.time) for p in test]))
    for label,e in comparisons:
        print(f'    {label:31s} {rmse(e)*1000:12.3f} {st.fmean(e)*1000:14.3f} {st.stdev(e)*1000:15.3f}')
    for label, indices in [('inside fit range',[i for i in range(len(tx)) if model.low<=tx[i]<=model.high]),
                           ('outside fit range',outside)]:
        if indices:
            print(f'  {label}: {len(indices):,} blocks; formula RMSE {rmse([residual[i] for i in indices])*1000:.3f} ps; '
                  f'constant RMSE {rmse([baseline_errors[i] for i in indices])*1000:.3f} ps')
    spread_header()
    spread('uncorrected test blocks',actual)
    spread('candidate test blocks',normalized)
    if deployed:
        canonical_lookup = {(p.run,p.index):p.lap_ns for p in canonical_points}
        deployed_test = [canonical_lookup[(p.run,p.index)] for p in test]
        spread('deployed test blocks',deployed_test)
        print(f'  Deployed test SD reduction vs uncorrected: {reduction(st.stdev(actual),st.stdev(deployed_test))}')
        print('  Candidate and deployed reference levels may differ; compare scatter, not absolute means.')
    print(f'  Test RMSE reduction vs constant: {reduction(rmse(baseline_errors),rmse(residual))}')
    print(f'  Test SD reduction: {reduction(st.stdev(actual),st.stdev(normalized))}')
    print(f'  Test peak-to-peak reduction: {reduction(max(actual)-min(actual),max(normalized)-min(normalized))}')
    print(f'  Residual correlation with {predictor}: {fmt(pearson(tx,residual))}; '
          f'with elapsed time: {fmt(pearson([p.time for p in test],residual))}')
    # Report actual one-second correction, separate from block-level fit performance.
    # A block contains exactly width consecutive retained fragments in a run.
    if lag==0 and not high:
        selected = {(p.run,p.index) for p in test}
        raw_seconds, corrected_seconds, deployed_seconds = [],[],[]
        ordinal = Counter()
        for s in samples:
            index = ordinal[s.run]//args.bin_seconds
            ordinal[s.run] += 1
            if (s.run,index) in selected:
                raw_seconds.append(s.lap_ns)
                corrected_seconds.append(s.lap_ns-model.correction(s.temperature))
                deployed_seconds.append(s.canonical_ns)
        print('\n  SAME FINAL TEST, AT ONE-SECOND RESOLUTION (cached recorded temperature)')
        spread_header()
        spread('uncorrected one-second',raw_seconds)
        spread('candidate one-second',corrected_seconds)
        if deployed:
            spread('deployed one-second',deployed_seconds)
        print('  Polynomial evaluated per second here; nonlinear averaging can differ from block evaluation.')
    elif high:
        print('  HIGH correction is evaluated on matched blocks only; no live one-second correction is inferred.')
    else:
        print('  Lagged correction is evaluated at block resolution only; no interpolated one-second results.')
    print('\n  FINAL TEST IN SIX CHRONOLOGICAL WINDOWS')
    print(f'    UTC start                    N  {column:>10s}  uncorrected_ns   candidate_mean_ns  residual_mean_ps')
    for i in range(6):
        indices = list(range(i*len(test)//6,(i+1)*len(test)//6))
        if indices:
            print(f'    {stamp(test[indices[0]].time):25s} {len(indices):5d} '
                  f'{st.fmean(tx[j] for j in indices):11.5f} '
                  f'{st.fmean(actual[j] for j in indices):12.6f} '
                  f'{st.fmean(normalized[j] for j in indices):19.6f} '
                  f'{st.fmean(residual[j] for j in indices)*1000:17.3f}')
    shape(test,tx,residual,args)
    if high:
        # Describe all directions, including cycles outside the final test.
        # Never refit or use this descriptive table for candidate selection.
        all_x = [p.temperature for p in points]
        all_residual = [p.lap_ns-model.predict(x) for p,x in zip(points,all_x)]
        print('\n  Full-campaign direction comparison includes development data and possible extrapolation; descriptive only.')
        shape(points,all_x,all_residual,args,scope="ALL BLOCKS (DESCRIPTIVE)")
    print('\n  INTERPRETATION')
    if degree==0:
        print(f'  Forward validation did not justify a {predictor} correction under the simplicity rule.')
    elif rmse(residual)<rmse(baseline_errors) and st.stdev(normalized)<st.stdev(actual):
        print(f'  The frozen {predictor} formula improved both later prediction and later stability.')
        print('  This is a candidate calibration for this setup and measured range, not yet a transfer test.')
    else:
        print('  The frozen formula did not improve both prediction and stability on later data.')
        print('  Do not adopt it as an established correction; the residual tables identify what remains.')
    if trend is not None and rmse(residual)>=rmse(comparisons[-1][1]):
        print(f'  {predictor} did not beat a simple elapsed-time line in this test; shared drift remains plausible.')
    if outside:
        print('  Some test predictor values are extrapolations; inspect inside-range results separately.')
    if lag and lag==steps:
        print('  Selected lag reaches the search boundary; a longer lag has not been evaluated.')
    if degree==args.max_degree and degree>1:
        print('  Selected curvature reaches the allowed degree; this alone is not a reason to increase it.')
    print('  Full-data refitting is intentionally deferred so this displayed formula retains an honest test.')


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('campaign',help='exact campaign name, e.g. External1')
    parser.add_argument('--sensor',choices=(*SENSORS, 'high', 'compare', 'compare-legacy'),default='synthetic',
                        help='temperature or high (raw ADC moments); compare remains temperature-only (default synthetic)')
    parser.add_argument('--input',type=Path,help='PHOTONS JSON/JSONL instead of PostgreSQL')
    parser.add_argument('--bin-seconds',type=int,default=60,help='complete nonoverlapping blocks (default 60)')
    parser.add_argument('--max-degree',type=int,choices=(1,2,3),default=3,help='maximum polynomial degree (default 3)')
    parser.add_argument('--max-lag-seconds',type=int,default=0,help='optional past-temperature search (default 0; multiple of block width)')
    parser.add_argument('--simplicity-pct',type=float,default=5,help='prefer simpler models within this percent of best validation RMSE (default 5)')
    parser.add_argument('--temperature-bin-c',type=float,default=.1,help='minimum report temperature-bin width (default .1)')
    parser.add_argument('--direction-threshold-c-per-minute',type=float,default=.002)
    parser.add_argument('--high-bin-mv',type=float,default=1.0,help='minimum HIGH shape-bin width in mV (default 1)')
    parser.add_argument('--direction-threshold-mv-per-minute',type=float,default=.1)
    parser.add_argument('--skip-seconds',type=int,default=0,help='omit initial campaign seconds (default 0)')
    parser.add_argument('--start',type=utc,help='inclusive UTC/offset timestamp')
    parser.add_argument('--end',type=utc,help='exclusive UTC/offset timestamp')
    parser.add_argument('--batch-size',type=int,default=512)
    args = parser.parse_args(argv[1:])
    if min(args.bin_seconds,args.batch_size)<1 or min(args.max_lag_seconds,args.skip_seconds)<0:
        parser.error('block/batch size must be positive; lag/skip must be nonnegative')
    if args.sensor == 'high' and args.max_lag_seconds:
        parser.error('HIGH mode uses matched simultaneous fragments; --max-lag-seconds must be 0')
    if args.max_lag_seconds%args.bin_seconds:
        parser.error('--max-lag-seconds must be a multiple of --bin-seconds')
    if any(not math.isfinite(v) or v<0 for v in
           (args.simplicity_pct,args.direction_threshold_c_per_minute,args.direction_threshold_mv_per_minute)) or any(
               not math.isfinite(v) or v<=0 for v in (args.temperature_bin_c,args.high_bin_mv)):
        parser.error('finite nonnegative tolerances and positive predictor-bin widths required')
    if args.start is not None and args.end is not None and args.start>=args.end:
        parser.error('--start must precede --end')
    return args


def main(argv):
    args = arguments(argv)
    print(f'PHOTONS THERMAL NORMALIZATION REPORT {VERSION}',flush=True)
    print(f'Campaign: {args.campaign}; generated UTC: {datetime.now(timezone.utc).isoformat(timespec="seconds")}')
    print(f'Options: sensor={args.sensor}, block={args.bin_seconds}s, max_degree={args.max_degree}, max_lag={args.max_lag_seconds}s, simplicity={args.simplicity_pct:g}%')
    print(f'Selection: start={args.start}, end={args.end}, skip={args.skip_seconds}s; source={args.input or "read-only database"}')
    print(f'Predictor selection: {args.sensor}; target: uncorrected one-second LAP (ns).')
    if args.sensor != 'high':
        print('Synthetic: temperature.temperature_c; BME280 PI/PD/OUT: environment.pi/pd/out.temperature_c.')
        print('MAX31865: temperature.components.rtd.temperature_c. Legacy PI/PD use the recorded component/receiver paths.')
    print('Rows missing a selected reading are omitted; missing grouped sensors never fall back to legacy aliases.')
    print('Predictor settings/path changes start separate epochs; no temperature smoothing is added.')
    if args.sensor in ('compare', 'compare-legacy'):
        print(f'Comparison requires all {len(SENSORS if args.sensor == "compare" else LEGACY_SENSORS)} predictors on every retained row; blocks and time splits are shared.')
        print('Sensor-specific degree/lag choices use development folds only. Final-test ranking is exploratory.')
    print('New rows use measurement.raw_flight_ns directly; legacy rows reconstruct raw LAP from recorded offsets.')
    if args.sensor == 'high':
        print('HIGH: exact per-fragment ADC sums/counts from photons.pd_out; nominal 3300 mV / 4095 counts.')
        print('No temperature required. HIGH60 is not used as a predictor and is not averaged again.')
        print('Pool ADC samples and accepted races separately over matching nonoverlapping fragments.')
        print('--bin-seconds counts nominal one-second fragments; actual durations are reported. No lag search.')
    else:
        print('Equal weight per usable second; no cumulative means; no rejection based on lap magnitude.')
    print('Loading campaign observations...',flush=True)
    rows = file_rows(args.input,args.campaign) if args.input else database_rows(args)
    samples,counts,reasons = collect(rows,args)
    print('\nDATA ACCOUNTING')
    for key,value in counts.items():
        print(f'  {key}: {value:,}')
    print('  Continuity-break reason counts can overlap. Partial blocks and lag-history losses are reported per epoch.')
    if not samples:
        print('No usable predictor/lap pairs. Report the accounting above before changing any filters.')
        return 1
    epochs = {}
    for sample in samples:
        epochs.setdefault(sample.epoch,[]).append(sample)
    for epoch,observations in epochs.items():
        for sensor in (SENSORS if args.sensor == 'compare' else
                       LEGACY_SENSORS if args.sensor == 'compare-legacy' else (args.sensor,)):
            sensor_args = argparse.Namespace(**vars(args))
            sensor_args.sensor = sensor
            selected = [replace(s, temperature=dict(s.sensor_temperatures)[sensor]) for s in observations]
            analyze(selected,reasons[epoch],sensor_args)
    print('\nREPORT NOTES')
    print('  Epochs are fitted separately across recorded settings/calibration/predictor changes, resets and recovery.')
    print('  Deployed = stored canonical LAP; uncorrected = original legacy LAP or reconstructed pre-correction LAP.')
    print('  Candidate formulas always fit uncorrected LAP and replace, rather than compound, the deployed correction.')
    print('  Blocks and lag history never cross continuity gaps. Unknown hardware changes need explicit time selection.')
    print('  All printed SDs describe one-second or block means, not individual-photon jitter or calibration uncertainty.')
    print('  Overlapping sensor means and cached temperatures are not independent thermal experiments.')
    print('  No p-values or independent-sample confidence claims; validation tests prediction, not causation.')
    print('  Repeatedly tuning against this final test makes it exploratory; confirm revisions on newly collected data.')
    print('  No database writes, live compensation, stored measurements, or additional output files.')
    print('END PHOTONS THERMAL REPORT')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
