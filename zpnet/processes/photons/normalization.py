"""PHOTONS calibration policy; edit source and formula here, then restart PHOTONS.

L_canonical = L_observed - sum(c[k] * (T - T_ref)**(k + 1)).
Coefficients are in ns/C**(k+1). Calibration identity changes automatically with
policy; a different identity starts a fresh measurement epoch, never a mixed one.
SYSTEM's synthetic temperature combines the three configured sensor means. Its model
identity includes each window and weight. The numerical coefficients remain the
Short2 BME280 zero-lag fit, provisionally transferred; refit against the synthetic
temperature and reconstructed uncorrected LAP in the new heating arrangement.

Firmware applies the latest supplied offset uniformly at each fragment drain.
An unavailable authority prevents refresh; firmware retains its last offset,
and each fragment records that offset, its temperature and refresh age.
Correction refresh age is distinct from constituent sample ages and mean spans.
All configured sensors are required; an outage never changes the mixture.
"""

import hashlib
import json
from decimal import Decimal, ROUND_HALF_EVEN
from zpnet.shared.temperature import TEMPERATURE_MODEL_ID, TEMPERATURE_POLICY


POLICY = {
    "schema": "PHOTONS_THERMAL_NORMALIZATION_V1",
    "temperature_path": ["temperature", "temperature_c"],
    "quality_path": ["temperature"],
    "readiness_feature": "PI.SYSTEM.TEMPERATURE",
    "temperature_source": TEMPERATURE_POLICY["source"],
    "temperature_schema": TEMPERATURE_POLICY["schema"],
    "temperature_model_id": TEMPERATURE_MODEL_ID,
    "temperature_model": TEMPERATURE_POLICY,
    "maximum_sample_age_ms": max(
        item["maximum_age_ms"] for item in TEMPERATURE_POLICY["sources"].values()
    ),
    "calibration_basis": "PROVISIONAL_SHORT2_BME280_FIT_TRANSFERRED_TO_THREE_SENSOR_MODEL",
    "witness_temperature_paths": [
        [name, "raw_temperature_c"] for name in TEMPERATURE_POLICY["sources"]
    ],
    "reference_c": "34.878255509",
    "coefficients_ns": ["2.46050130414"],
    "application": "UNIFORM_OFFSET_AT_FRAGMENT_DRAIN",
}
NORMALIZATION_ID = int(hashlib.sha256(
    json.dumps(POLICY, sort_keys=True, separators=(",", ":")).encode()
).hexdigest()[:8], 16)
assert NORMALIZATION_ID != 0  # zero is the firmware's unconfigured identity


def _at(context, path):
    for key in path:
        context = context[key]
    return context


class TemperatureUnavailable(ValueError):
    """The configured authority is temporarily waiting for fresh sensor data."""


def temperature_c(system_context):
    """Require the exact SYSTEM temperature model admitted by this calibration."""
    quality = _at(system_context, POLICY["quality_path"])
    if (quality["source"] != POLICY["temperature_source"]
            or quality["schema"] != POLICY["temperature_schema"]
            or quality["model_id"] != POLICY["temperature_model_id"]):
        raise ValueError("PHOTONS synthetic temperature authority mismatched")
    # Availability is an ordinary retry condition. Keep malformed testimony
    # and model mismatches distinct so the caller cannot silence those errors.
    if quality["status"] in ("INITIALIZING", "UNAVAILABLE"):
        raise TemperatureUnavailable(f"PHOTONS synthetic temperature {quality['status']}")
    if quality["status"] != "OK" or not 0 <= quality["age_ms"]:
        raise ValueError("PHOTONS synthetic temperature status or age malformed")
    if quality["age_ms"] > POLICY["maximum_sample_age_ms"]:
        raise TemperatureUnavailable("PHOTONS synthetic temperature stale")
    value = Decimal(str(_at(system_context, POLICY["temperature_path"])))
    if not value.is_finite():
        raise ValueError("PHOTONS normalization temperature must be finite")
    return value


def command_args(system_context):
    """Integer-only wire: microdegrees C and femtoseconds (1 fs = 1e-6 ns)."""
    temperature_micro_c = int((temperature_c(system_context)*1_000_000)
                              .to_integral_value(rounding=ROUND_HALF_EVEN))
    # Evaluate the exact temperature sent to firmware, so testimony reproduces it.
    x = Decimal(temperature_micro_c)/1_000_000 - Decimal(POLICY["reference_c"])
    correction_ns = Decimal(0)
    for coefficient in reversed(POLICY["coefficients_ns"]):
        correction_ns = (correction_ns + Decimal(coefficient))*x
    correction_fs = int((correction_ns*1_000_000)
                        .to_integral_value(rounding=ROUND_HALF_EVEN))
    if not (-2**31 <= temperature_micro_c < 2**31 and -2**31 <= correction_fs < 2**31):
        raise ValueError("PHOTONS normalization exceeds the signed 32-bit wire range")
    return {"normalization_id": NORMALIZATION_ID,
            "temperature_micro_c": temperature_micro_c, "correction_fs": correction_fs}
