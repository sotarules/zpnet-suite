"""PHOTONS calibration policy; edit source and formula here, then restart/flash.

L_canonical = L_observed - sum(c[k] * (T - T_ref)**(k + 1)).
Coefficients are in ns/C**(k+1). Calibration identity changes automatically with
policy; a different identity starts a fresh measurement epoch, never a mixed one.
The immutable BME280 Short2 fit is deliberately not refitted during acquisition.
This is the 2026-10-05 19:58 UTC zero-lag fit (1,980 blocks); the later lag
search retained only 563 common blocks and also selected a zero-lag line.

Firmware applies the latest supplied offset uniformly at each fragment drain.
An unavailable authority prevents refresh; firmware retains its last offset,
and each fragment records that offset, its temperature and refresh age. Age is
time since the Pi refresh, not the age of the BME280's cached physical reading.
"""

import hashlib
import json
from decimal import Decimal, ROUND_HALF_EVEN


POLICY = {
    "schema": "PHOTONS_THERMAL_NORMALIZATION_V1",
    "temperature_path": ["environment", "temperature_c"],
    "quality_path": ["environment"],
    "readiness_feature": "PI.SYSTEM.ENVIRONMENT",
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


def temperature_c(system_context):
    """Temperature authority adapter. Replace this adapter for a new sensor.

    BME280 quality comes from SYSTEM. Missing/unusable testimony is an input
    failure, never an instruction to apply zero correction or switch sensors.
    Change POLICY alongside any adapter semantics so the epoch identity changes.
    """
    quality = _at(system_context, POLICY["quality_path"])
    if (quality["sensor_present"] is not True or quality["read_ok"] is not True
            or quality["stale"] is not False):
        raise ValueError("PHOTONS normalization temperature authority unavailable")
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
