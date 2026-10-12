"""Three-source thermal aggregation regression tests (hardware-free)."""

import importlib
import time
import unittest

from zpnet.shared.temperature import TEMPERATURE_POLICY, TEMPERATURE_MODEL_ID
import zpnet.processes.system.temperature as thermal


class ThreeSensorTemperatureTests(unittest.TestCase):
    def setUp(self):
        self.thermal = importlib.reload(thermal)

    def reading(self, name, temperature, sequence=1):
        policy = TEMPERATURE_POLICY["sources"][name]
        now = time.monotonic()
        self.thermal.accept_temperature(
            name,
            {"source": policy["source"], "schema": policy["schema"],
             "status": "OK", "temperature_c": temperature,
             "age_ms": 0, "sample_sequence": sequence},
            (sequence,), now,
        )

    def test_policy_has_physical_sources(self):
        self.assertEqual(set(TEMPERATURE_POLICY["sources"]),
                         {"environment", "receiver", "rtd"})
        self.assertEqual(TEMPERATURE_POLICY["sources"]["environment"]["source"],
                         "PI.I2C1.BME280.0x76")
        self.assertEqual(TEMPERATURE_POLICY["sources"]["receiver"]["source"],
                         "PI.I2C1.BME280.0x77")
        self.assertGreater(TEMPERATURE_MODEL_ID, 0)

    def test_aggregate_keeps_individual_sources(self):
        self.reading("environment", 35.0)
        self.reading("receiver", 30.0)
        self.reading("rtd", 31.0)
        context = self.thermal.build_temperature_context()
        aggregate = context["temperature"]
        self.assertEqual(aggregate["status"], "OK")
        self.assertAlmostEqual(aggregate["temperature_c"], 32.0)
        self.assertEqual(set(aggregate["components"]),
                         {"environment", "receiver", "rtd"})
        self.assertEqual(context["receiver"]["temperature_c"], 30.0)
        self.assertEqual(aggregate["components"]["receiver"]["temperature_c"], 30.0)

    def test_receiver_unavailable_does_not_reweight_aggregate(self):
        self.reading("environment", 35.0)
        self.reading("receiver", 30.0)
        self.reading("rtd", 31.0)
        policy = TEMPERATURE_POLICY["sources"]["receiver"]
        self.thermal.accept_temperature(
            "receiver",
            {"source": policy["source"], "schema": policy["schema"],
             "status": "UNAVAILABLE", "age_ms": 0},
            (), time.monotonic(),
        )
        context = self.thermal.build_temperature_context()
        self.assertEqual(context["receiver"]["status"], "UNAVAILABLE")
        self.assertEqual(context["temperature"]["status"], "UNAVAILABLE")
        self.assertNotIn("temperature_c", context["temperature"])


if __name__ == "__main__":
    unittest.main()
