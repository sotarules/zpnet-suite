"""Pi-owned PHOTONS interpretation; raw Teensy evidence never changes.

Models are frozen JSON, loaded once at PHOTONS startup. No online fitting, no
campaign recentering, and no altitude/pressure/time predictors. Thermal history
survives flash cuts; gaps/restarts warm it again. All inputs and results needed
to replay a correction are persisted in each canonical one-second record.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path


# Matches process_photons.cpp. This is NOT a thermal-model identity.
ACQUISITION_ID = 0x52415731  # RAW1: uncorrected, GNSS-projected accepted races
MODEL_SCHEMA = "PI_PHOTONS_THERMAL_MODEL_V1"
MEASUREMENT_SCHEMA = "PI_PHOTONS_MEASUREMENT_V1"
MODEL_PATH = Path(__file__).with_name("thermal_model.json")
SENSORS = {
    "pi": (("environment", "pi"), "PI.I2C1.BME280.0x76"),
    "pd": (("environment", "pd"), "PI.I2C1.BME280.0x77"),
    "out": (("environment", "out"), "PI.I2C2.BME280.0x77"),
    "rtd": (("rtd",), "TEENSY.SPI1.MAX31865"),
}
FEATURE_POLICY = {
    "schema": "PHOTONS_THERMAL_FEATURES_V1",
    "sources": {name: {"path": list(path), "source": source,
                       "value": "raw_temperature_c"}
                for name, (path, source) in SENSORS.items()},
    "maximum_sample_age_ms": 3000,
    "maximum_context_delay_ms": 1500,
    "maximum_context_round_trip_ms": 500,
    "maximum_gap_seconds": 3,
    "ema_seconds": [30, 120, 600],
    "warmup_time_constants": 5,
    "integration": "CAUSAL_ENDPOINT_EMA_ACTUAL_SAMPLE_INTERVAL",
}


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


FEATURE_ID = identity(FEATURE_POLICY)
FEATURE_NAMES = tuple(
    feature for name in SENSORS for feature in
    (f"{name}.temperature_c", f"{name}.rate_c_per_s",
     *(f"{name}.lag_{tau}_c" for tau in FEATURE_POLICY["ema_seconds"]))
)


def timestamp(value):
    value = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if value.tzinfo is None:
        raise ValueError("PHOTONS timestamps require a timezone")
    return value.timestamp()


def acquisition_signature(state):
    """Model domain excludes campaign, location, temperature values and time."""
    photons = state["photons"]
    return {
        "acquisition_id": photons["stats"]["acquisition_id"],
        "race": {key: photons["race"].get(key) for key in (
            "schema", "cadence_ns", "pulse_ns", "capture_gate", "capture_min_ns",
            "capture_max_ns", "launch_bracket_max_cycles", "flight_interpretation",
            "launch_timing_policy")},
        "core": {key: photons["core"].get(key) for key in (
            "algorithm", "minimum_count", "minimum_radius_cycles", "mad_multiplier",
            "minimum_retained_pct")},
    }


def check_model(model):
    """Reject configuration errors at startup, before any model can be used."""
    model = copy.deepcopy(model)
    recorded_id = model.pop("normalization_id", None)
    if recorded_id is not None and recorded_id != identity(model):
        raise ValueError("recorded PHOTONS model identity does not match its policy")
    if (model["schema"] != MODEL_SCHEMA or model["feature_id"] != FEATURE_ID
            or model["feature_policy"] != FEATURE_POLICY):
        raise ValueError("PHOTONS thermal model/feature schema mismatch")
    if model["application"] != "SUBTRACT_ONE_OFFSET_PER_SECOND":
        raise ValueError("unsupported PHOTONS correction application")
    needed = {name for term in model["terms"] for name in term["factors"]}
    if not needed <= set(FEATURE_NAMES):
        raise ValueError("model contains an unsupported thermal feature")
    if needed != set(model["reference"]) or needed != set(model["domain"]):
        raise ValueError("each model feature needs exactly one reference and domain")
    for name in needed:
        lo, hi = model["domain"][name]
        if not all(math.isfinite(v) for v in (lo, hi, model["reference"][name])) or lo > hi:
            raise ValueError("nonfinite or unordered thermal model domain")
    for term in model["terms"]:
        if len(term["factors"]) not in (1, 2) or not math.isfinite(term["coefficient_ns"]):
            raise ValueError("thermal model requires finite linear/quadratic terms")
    if model["terms"] and model["acquisition_signature"]["acquisition_id"] != ACQUISITION_ID:
        raise ValueError("thermal model belongs to a different raw acquisition epoch")
    return model


def term_values(model, features):
    return [math.prod(features[name] - model["reference"][name]
                      for name in term["factors"]) for term in model["terms"]]


def correction(model, features):
    return math.fsum(term["coefficient_ns"] * value for term, value in
                     zip(model["terms"], term_values(model, features)))


def corrected_distribution(raw, offset):
    """A common offset translates mean/extrema; count and within-second M2 stay raw."""
    out = copy.deepcopy(raw)
    if raw["n"]:
        for key in ("mean", "min", "max"):
            out[key] -= offset
    return out


class ThermalHistory:
    """Bounded causal sensor state; four sensors, three thermal time constants."""

    def __init__(self):
        self.previous = None
        self.sensors = {}

    def observe(self, state):
        timing = state["context_timing"]
        now = timestamp(timing["context_at_utc"])
        received = timestamp(timing["fragment_received_at_utc"])
        photons = state["photons"]
        epoch = (identity(acquisition_signature(state)), photons["stats"]["reset_count"],
                 photons["recovery"]["generation"])
        cursor = (epoch, state["sequence"], received)
        if self.previous is not None:
            previous_epoch, previous_sequence, previous_time = self.previous
            if (epoch != previous_epoch or state["sequence"] != previous_sequence + 1
                    or not 0 < received - previous_time <= FEATURE_POLICY["maximum_gap_seconds"]):
                self.sensors.clear()
        self.previous = cursor
        values, witnesses = {}, {}
        aligned = (0 <= timing["context_after_ingress_ms"] <= FEATURE_POLICY["maximum_context_delay_ms"]
                   and 0 <= timing["round_trip_ms"] <= FEATURE_POLICY["maximum_context_round_trip_ms"])
        if not aligned:
            self.sensors.clear()
            return {"feature_id": FEATURE_ID, "values": {}, "sensors": {},
                    "status": "CONTEXT_TOO_LATE"}

        for name, (path, source) in SENSORS.items():
            item = state
            for key in path:
                item = item.get(key, {})
            status = item.get("status", "NOT_RECORDED")
            if item and item["source"] != source:
                raise ValueError(f"unexpected PHOTONS temperature source: {name}")
            if status != "OK":
                self.sensors.pop(name, None)
                witnesses[name] = {"status": status}
                continue
            age = item["age_ms"]
            value = item["raw_temperature_c"]
            if age < 0 or not math.isfinite(value):
                raise ValueError(f"malformed PHOTONS temperature: {name}")
            if age + timing["round_trip_ms"] > FEATURE_POLICY["maximum_sample_age_ms"]:
                self.sensors.pop(name, None)
                witnesses[name] = {"status": "STALE", "age_ms": age}
                continue
            if name == "rtd":
                key = (item["sample_sequence"], item["sampled_at_ms32"])
                sample_time = now - age / 1000
            else:
                key = item["sampled_at_utc"]
                sample_time = timestamp(key)
            previous = self.sensors.get(name)
            if previous is not None and key == previous["key"]:
                if value != previous["value"]:
                    raise ValueError("one thermal sample identity has two temperatures")
                current = previous
                sample_time = current["time"]
            else:
                dt = sample_time - previous["time"] if previous is not None else 0
                if (previous is None or key < previous["key"]
                        or not 0 < dt <= FEATURE_POLICY["maximum_gap_seconds"]):
                    current = {"start": sample_time, "ema": {tau: value for tau in FEATURE_POLICY["ema_seconds"]}}
                else:
                    current = {"start": previous["start"], "rate": (value - previous["value"]) / dt,
                               "ema": {tau: old + (-math.expm1(-dt / tau)) * (value - old)
                                       for tau, old in previous["ema"].items()}}
                current.update(key=key, time=sample_time, value=value)
                self.sensors[name] = current
            span = sample_time - current["start"]
            values[f"{name}.temperature_c"] = value
            if "rate" in current:
                values[f"{name}.rate_c_per_s"] = current["rate"]
            for tau, average in current["ema"].items():
                if span >= FEATURE_POLICY["warmup_time_constants"] * tau:
                    values[f"{name}.lag_{tau}_c"] = value - average
            witnesses[name] = {"status": "OK", "age_ms": age, "history_seconds": span,
                               "sample_key": key, "raw_temperature_c": value}
        return {"feature_id": FEATURE_ID, "values": values, "sensors": witnesses,
                "status": "RECORDED"}


class Normalizer:
    def __init__(self, model=None):
        if model is None:
            model = json.loads(MODEL_PATH.read_text(encoding="utf-8"))
        self.model = check_model(model)
        self.model_id = identity(self.model)
        self.needed = set(self.model["reference"])
        self.history = ThermalHistory()

    def policy_snapshot(self):
        return {**copy.deepcopy(self.model), "normalization_id": self.model_id}

    def measure(self, state):
        photons = state["photons"]
        if photons["stats"]["acquisition_id"] != ACQUISITION_ID:
            raise ValueError("normalization requires raw acquisition")
        raw = copy.deepcopy(photons["race"]["flight_ns"])
        thermal = self.history.observe(state)
        out = {"schema": MEASUREMENT_SCHEMA, "authority": "PI.PHOTONS",
               "acquisition_id": ACQUISITION_ID, "normalization_id": self.model_id,
               "acquisition_signature": acquisition_signature(state),
               "raw_flight_ns": raw, "thermal": thermal,
               "normalized_flight_ns": None, "correction_ns": None}
        if not raw["n"]:
            out["status"] = "NO_ACCEPTED_RACES"
            return out
        if self.model["terms"] and self.model["acquisition_signature"] != acquisition_signature(state):
            out["status"] = "ACQUISITION_CHANGED"
            return out
        missing = sorted(self.needed - thermal["values"].keys())
        if missing:
            out.update(status="THERMAL_HISTORY_UNAVAILABLE", missing_features=missing)
            return out
        outside = [name for name in sorted(self.needed)
                   if not self.model["domain"][name][0] <= thermal["values"][name] <= self.model["domain"][name][1]]
        if outside:
            out.update(status="OUTSIDE_CALIBRATION_DOMAIN", outside_features=outside)
            return out
        offset = correction(self.model, thermal["values"])
        if not math.isfinite(offset):
            raise ValueError("nonfinite PHOTONS model correction")
        out.update(status="CORRECTED" if self.model["terms"] else "IDENTITY",
                   correction_ns=offset, normalized_flight_ns=corrected_distribution(raw, offset))
        return out
