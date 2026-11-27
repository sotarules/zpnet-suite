"""NOR9 live SYSTEM temperature transport; never replay persisted context.

SYSTEM retains its two-sensor, 30-sample synthetic definition and RTD mean.
A Teensy probe
starts the input lease before SYSTEM is queried: serial/queue delay consumes
the lease instead of rejuvenating the observation. The producer latches one
pair of temperature means at the first launch of each fragment and records
those exact inputs.
An unavailable source stops new launches; publication/recovery heartbeats and
already-acquired populations continue. There is no zero-correction fallback.
"""
import logging
import math
import threading
import time

from zpnet.shared.photons_voltage import (
    NOR9_ACQUISITION_ID, NOR9_RTD_WINDOW_SAMPLES, THERMAL_TEMPERATURE_MODEL_ID,
)


class ThermalBridge:
    def __init__(self, command, system_report):
        self.command = command
        self.system_report = system_report

    def start(self):
        threading.Thread(target=self.run, name="photons-thermal-input", daemon=True).start()

    def rpc(self, command, args=None):
        reply = self.command(machine="TEENSY", subsystem="PHOTONS", command=command,
                             args=args or {}, retries=1, retry_delay_s=0.0)
        if not reply["success"]:
            raise RuntimeError(f"{command}: {reply}")
        return reply["payload"]

    def update(self):
        started = time.monotonic()
        probe = self.rpc("THERMAL_PROBE")
        if probe["status"] != "thermal_probe" or probe["acquisition_id"] != NOR9_ACQUISITION_ID:
            raise ValueError("NOR9 requires matching Teensy acquisition firmware")
        report = self.system_report()
        temperature, rtd = report["temperature"], report["rtd"]
        if (temperature["model_id"] != THERMAL_TEMPERATURE_MODEL_ID
                or temperature["schema"] != "SYNTHETIC_TEMPERATURE_V1"
                or temperature["source"] != "PI.SYSTEM.TEMPERATURE"):
            raise ValueError("NOR9 requires the frozen SYSTEM synthetic-temperature policy")
        if rtd["schema"] != "PT1000_MAX31865_V1" or rtd["source"] != "TEENSY.SPI1.MAX31865":
            raise ValueError("NOR9 requires SYSTEM MAX31865")
        value, rtd_value, lease = 0.0, 0.0, 0
        if temperature["status"] == "OK" and rtd["status"] == "OK":
            if rtd["smoothing"]["window_samples"] != NOR9_RTD_WINDOW_SAMPLES:
                raise ValueError("NOR9 requires the frozen 30-sample RTD mean")
            value = float(temperature["temperature_c"])
            rtd_value = float(rtd["temperature_c"])
            ages = (temperature["age_ms"], rtd["age_ms"])
            if (not all(math.isfinite(v) and -100 <= v <= 150 for v in (value, rtd_value))
                    or not all(math.isfinite(a) and a >= 0 for a in ages)):
                raise ValueError("malformed NOR9 temperature observation")
            age = max(ages)
            # Conservative: include all request time, even though SYSTEM's age
            # already includes part of it. Firmware separately charges transit
            # since the probe. Freshness is more important than uninterrupted N.
            lease = max(0, 3000 - math.ceil(age + 1000 * (time.monotonic() - started)))
        result = self.rpc("THERMAL_UPDATE", {
            "probe": probe["probe"], "lease_ms": lease,
            "temperature_model_id": THERMAL_TEMPERATURE_MODEL_ID,
            "temperature_c": format(value, ".15g"),
            "rtd_temperature_c": format(rtd_value, ".15g"),
        })
        if result["status"] not in ("thermal_updated", "thermal_hold"):
            raise RuntimeError(f"NOR9 temperature update rejected: {result}")
        return result["status"]

    def run(self):
        previous = None
        while True:
            started = time.monotonic()
            try:
                state = self.update()
            except Exception as exc:
                # Firmware expires its lease independently if transport or
                # SYSTEM fails. This thread never edits acquired statistics.
                state = "thermal_hold: " + str(exc)
            if state != previous:
                logging.info("[photons/nor9] %s", state)
                previous = state
            time.sleep(max(0.05, 1.0 - (time.monotonic() - started)))
