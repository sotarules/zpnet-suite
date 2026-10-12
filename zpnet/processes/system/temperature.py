"""SYSTEM-owned temperature means. Reports only read this cache."""

import copy
import math
import threading
import time
from collections import deque
from statistics import fmean

from zpnet.shared.temperature import (
    BME280_DEVICES, TEMPERATURE_MODEL_ID, TEMPERATURE_POLICY, CALIBRATION_TEMPERATURE_POLICY,
)


_LOCK = threading.Lock()
_SOURCES = TEMPERATURE_POLICY["sources"]
_READINGS = {
    name: {"source": policy["source"], "schema": policy["schema"],
           "status": "INITIALIZING"}
    for name, policy in _SOURCES.items()
}
_OBSERVED_AT = dict.fromkeys(_SOURCES, 0.0)
_SAMPLES = {
    name: deque(maxlen=policy["window_samples"])
    for name, policy in _SOURCES.items()
}
_LAST_KEY = dict.fromkeys(_SOURCES)


def feature_status(status: str) -> str:
    if status == "INITIALIZING":
        return "INITIALIZING"
    return "NOMINAL" if status == "OK" else "HOLD"


def accept_temperature(name: str, reading: dict, sample_key: tuple,
                       observed_at: float) -> None:
    """Accept one hardware observation, identified independently of its value.

    Equal temperatures from successive conversions count; repeated RPC copies
    of one conversion do not. RTD keys contain its sequence and boot-relative
    sample timestamp, so a firmware restart starts a new mean. observed_at is
    the request start, conservatively including transport time in sample age.
    """
    policy = _SOURCES[name]
    item = copy.deepcopy(reading)
    if item["source"] != policy["source"] or item["schema"] != policy["schema"]:
        raise ValueError(f"unexpected temperature source for {name}")
    if item["status"] == "OK":
        if not math.isfinite(item["temperature_c"]) or item["age_ms"] < 0:
            raise ValueError(f"invalid temperature observation for {name}")
        if item["age_ms"] + (time.monotonic() - observed_at) * 1000 > policy["maximum_age_ms"]:
            item["status"] = "STALE"

    with _LOCK:
        samples = _SAMPLES[name]
        previous_key = _LAST_KEY[name]
        if item["status"] == "OK":
            sampled_at = observed_at - item["age_ms"] / 1000
            if previous_key is not None and sample_key < previous_key:
                samples.clear()
            if samples and (sampled_at - samples[-1][0]) * 1000 > policy["maximum_age_ms"]:
                samples.clear()
            if sample_key != previous_key or not samples:
                samples.append((sampled_at, item["temperature_c"]))
            _LAST_KEY[name] = sample_key
            item["raw_temperature_c"] = item["temperature_c"]
            # Firmware resistance/raw_code and diagnostics still describe the
            # latest raw sample; only temperature_c is the rolling mean.
            item["temperature_c"] = fmean(value for _, value in samples)
            item["smoothing"] = {
                "window_samples": policy["window_samples"],
                "sample_count": len(samples),
                "span_ms": round((samples[-1][0] - samples[0][0]) * 1000),
            }
        else:
            samples.clear()
            _LAST_KEY[name] = None
            item.pop("temperature_c", None)
            item.pop("raw_temperature_c", None)
            item.pop("smoothing", None)
            item.pop("resistance_ohms", None)
        _READINGS[name] = item
        _OBSERVED_AT[name] = observed_at


def _source_snapshot(name: str, now: float) -> dict:
    item = copy.deepcopy(_READINGS[name])
    if item["status"] == "OK":
        item["age_ms"] += max(0, int((now - _OBSERVED_AT[name]) * 1000))
        if item["age_ms"] > _SOURCES[name]["maximum_age_ms"]:
            item["status"] = "STALE"
            item.pop("temperature_c", None)
            item.pop("raw_temperature_c", None)
            item.pop("smoothing", None)
            item.pop("resistance_ohms", None)
    if name in BME280_DEVICES:
        device = BME280_DEVICES[name]
        item["label"] = device["label"]
        item["sensor_bus"] = device["bus"]
        item["sensor_address"] = f"0x{device['address']:02X}"
        item["health_state"] = feature_status(item["status"])
        if item["status"] != "OK":
            item["read_ok"] = False
            item["stale"] = item["status"] != "INITIALIZING"
            for key in ("pressure_hpa", "humidity_pct", "altitude_m"):
                item.pop(key, None)
    return item


def build_sensor_status(name: str) -> dict:
    with _LOCK:
        return _source_snapshot(name, time.monotonic())


def build_temperature_context() -> dict:
    """Take all source means and their fixed-weight composite in one snapshot.

    Sensor faults expire the authority instead of changing its composition.
    Early/restarted means use the available samples and expose their count.
    No report call adds a sample or extends the lifetime of an old observation.
    """
    with _LOCK:
        now = time.monotonic()
        context = {name: _source_snapshot(name, now) for name in _SOURCES}
    components = {}
    # Only the deployed calibration members contribute to the synthetic scalar.
    # Other configured sensors remain independently sampled and reported.
    for name, policy in CALIBRATION_TEMPERATURE_POLICY["sources"].items():
        item = context[name]
        component = {"source": policy["source"], "weight": policy["weight"],
                     "status": item["status"]}
        for key in ("temperature_c", "raw_temperature_c", "age_ms",
                    "sample_sequence", "smoothing"):
            if key in item:
                component[key] = copy.deepcopy(item[key])
        components[name] = component
    combined = {
        "schema": TEMPERATURE_POLICY["schema"],
        "source": TEMPERATURE_POLICY["source"],
        "model_id": TEMPERATURE_MODEL_ID,
        "status": "OK",
        "components": components,
    }
    statuses = [item["status"] for item in components.values()]
    if all(status == "OK" for status in statuses):
        combined["temperature_c"] = math.fsum(
            item["weight"] * item["temperature_c"] for item in components.values()
        ) / math.fsum(item["weight"] for item in components.values())
        # This is the oldest constituent's latest-sample age, not the age of a
        # report request. The trailing windows' spans are reported separately.
        combined["age_ms"] = max(item["age_ms"] for item in components.values())
    else:
        combined["status"] = "INITIALIZING" if "INITIALIZING" in statuses else "UNAVAILABLE"
    environment = {
        device["key"]: context[name] for name, device in BME280_DEVICES.items()
    }
    health = [item["health_state"] for item in environment.values()]
    environment["health_state"] = (
        "HOLD" if "HOLD" in health else
        "INITIALIZING" if "INITIALIZING" in health else "NOMINAL"
    )
    return {"environment": environment, "rtd": context["rtd"], "temperature": combined}
