"""Temperature model shared by SYSTEM and PHOTONS; no hardware or process state."""

import hashlib
import json


# Acquisition identity is separate from the deployed normalization model.
BME280_DEVICES = {
    "environment": {"key": "pi", "bus": 1, "address": 0x76, "label": "BME280 PI"},
    "receiver": {"key": "pd", "bus": 1, "address": 0x77, "label": "BME280 PD"},
    "otdr": {"key": "otdr", "bus": 2, "address": 0x77, "label": "BME280 OTDR"},
}


# Add another named source here and have its sampler call accept_temperature().
# Weights apply to sensor means, never to the number of polls or samples. Keep
# the membership fixed during outages: silently dropping a warmer/cooler sensor
# would introduce a step into the PHOTONS correction. Restart SYSTEM and PHOTONS
# after changing this model; its identity is part of the calibration epoch.
TEMPERATURE_POLICY = {
    "schema": "SYNTHETIC_TEMPERATURE_V1",
    "source": "PI.SYSTEM.TEMPERATURE",
    "aggregation": "WEIGHTED_SENSOR_MEANS",
    "smoothing": "TRAILING_SAMPLE_MEAN",
    "startup": "AVAILABLE_SAMPLES",
    "availability": "ALL_CONFIGURED_SOURCES",
    "history": "RESET_ON_FAULT_GAP_OR_RESTART",
    "sources": {
        "environment": {
            "source": "PI.I2C1.BME280.0x76",
            "schema": "BME280_V1",
            "sample_period_ms": 1000,
            "window_samples": 30,
            "maximum_age_ms": 3000,
            "weight": 1,
        },
        "receiver": {
            "source": "PI.I2C1.BME280.0x77",
            "schema": "BME280_V1",
            "sample_period_ms": 1000,
            "window_samples": 30,
            "maximum_age_ms": 3000,
            "weight": 1,
        },
        "otdr": {
            "source": "PI.I2C2.BME280.0x77",
            "schema": "BME280_V1",
            "sample_period_ms": 1000,
            "window_samples": 30,
            "maximum_age_ms": 3000,
            "weight": 1,
        },
        "rtd": {
            "source": "TEENSY.SPI1.MAX31865",
            "schema": "PT1000_MAX31865_V1",
            "sample_period_ms": 1000,
            "window_samples": 30,
            "maximum_age_ms": 3000,
            "weight": 1,
        },
    },
}
# Preserve the installed two-sensor calibration identity. PD and OTDR are
# independent environmental witnesses, not new normalization members.
CALIBRATION_TEMPERATURE_POLICY = {
    **TEMPERATURE_POLICY,
    "sources": {
        name: TEMPERATURE_POLICY["sources"][name]
        for name in ("environment", "rtd")
    },
}
TEMPERATURE_MODEL_ID = int(hashlib.sha256(
    json.dumps(CALIBRATION_TEMPERATURE_POLICY, sort_keys=True, separators=(",", ":")).encode()
).hexdigest()[:8], 16)
