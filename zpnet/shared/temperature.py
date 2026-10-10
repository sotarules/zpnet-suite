"""Temperature model shared by SYSTEM and PHOTONS; no hardware or process state."""

import hashlib
import json


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
TEMPERATURE_MODEL_ID = int(hashlib.sha256(
    json.dumps(TEMPERATURE_POLICY, sort_keys=True, separators=(",", ":")).encode()
).hexdigest()[:8], 16)
