"""NOR8 live SYSTEM temperature transport; never replay persisted context.

SYSTEM retains its two-sensor, 30-sample synthetic definition. A Teensy probe
starts the input lease before SYSTEM is queried: serial/queue delay consumes
the lease instead of rejuvenating the observation. The producer latches one
temperature at the first launch of each fragment and records that exact input.
An unavailable source stops new launches; publication/recovery heartbeats and
already-acquired populations continue. There is no zero-correction fallback.
"""
import logging
import math
import threading
import time

from zpnet.shared.photons_voltage import THERMAL_TEMPERATURE_MODEL_ID


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
        if probe["status"] != "thermal_probe" or probe["acquisition_id"] != 0x4E4F5238:
            raise ValueError("NOR8 requires matching Teensy acquisition firmware")
        temperature = self.system_report()["temperature"]
        if (temperature["model_id"] != THERMAL_TEMPERATURE_MODEL_ID
                or temperature["schema"] != "SYNTHETIC_TEMPERATURE_V1"
                or temperature["source"] != "PI.SYSTEM.TEMPERATURE"):
            raise ValueError("NOR8 requires the frozen SYSTEM synthetic-temperature policy")
        value, lease = 0.0, 0
        if temperature["status"] == "OK":
            value = float(temperature["temperature_c"])
            age = temperature["age_ms"]
            if not math.isfinite(value) or not -100 <= value <= 150 or age < 0:
                raise ValueError("malformed NOR8 temperature observation")
            # Conservative: include all request time, even though SYSTEM's age
            # already includes part of it. Firmware separately charges transit
            # since the probe. Freshness is more important than uninterrupted N.
            lease = max(0, 3000 - math.ceil(age + 1000 * (time.monotonic() - started)))
        result = self.rpc("THERMAL_UPDATE", {
            "probe": probe["probe"], "lease_ms": lease,
            "temperature_model_id": THERMAL_TEMPERATURE_MODEL_ID,
            "temperature_c": format(value, ".15g"),
        })
        if result["status"] not in ("thermal_updated", "thermal_hold"):
            raise RuntimeError(f"NOR8 temperature update rejected: {result}")
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
                logging.info("[photons/nor8] %s", state)
                previous = state
            time.sleep(max(0.05, 1.0 - (time.monotonic() - started)))
