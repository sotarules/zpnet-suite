"""Teensy-owned PT1000 observations mirrored into Pi SYSTEM context.

Independent of the slow platform poller; REPORT performs no hardware I/O.
Fresh conversions enter SYSTEM's temperature mean once each. The direct Teensy
RTD_REPORT remains instantaneous, preserving the firmware's forensic evidence.
"""

import copy
import math
import time

from zpnet.processes.system.temperature import accept_temperature, build_sensor_status, feature_status
from zpnet.shared.temperature import TEMPERATURE_POLICY

_POLICY = TEMPERATURE_POLICY["sources"]["rtd"]
POLL_SECONDS = _POLICY["sample_period_ms"] / 1000


def _accept_reading(reading: dict, request_started: float) -> None:
    """Store a complete response; never merge old temperature into new faults."""
    if not isinstance(reading, dict) or reading.get("schema") != "PT1000_MAX31865_V1":
        raise ValueError("missing PT1000_MAX31865_V1 response")
    item = copy.deepcopy(reading)
    if item.get("status") == "OK":
        for key in ("temperature_c", "resistance_ohms", "age_ms"):
            value = item.get(key)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"invalid RTD {key}")
        if item["age_ms"] < 0:
            raise ValueError("negative RTD sample age")
    else:
        item.pop("temperature_c", None)
        item.pop("resistance_ohms", None)
    key = (item["sample_sequence"], item["sampled_at_ms32"]) if item["status"] == "OK" else ()
    accept_temperature("rtd", item, key, request_started)


def _unavailable(reason: str) -> None:
    accept_temperature("rtd", {
        "source": _POLICY["source"],
        "schema": _POLICY["schema"],
        "status": "UNAVAILABLE",
        "error": reason,
    }, (), time.monotonic())


def build_rtd_status() -> dict:
    return build_sensor_status("rtd")



def rtd_feature_status() -> str:
    """Derive readiness from the same aging cache used by SYSTEM.REPORT."""
    return feature_status(build_rtd_status()["status"])


def rtd_monitor() -> None:
    from zpnet.processes.processes import send_command

    while True:
        started = time.monotonic()
        try:
            response = send_command(
                machine="TEENSY", subsystem="SYSTEM", command="RTD_REPORT",
                retries=1, retry_delay_s=0.0,
            )
            if not isinstance(response, dict) or not response.get("success"):
                raise RuntimeError("Teensy SYSTEM.RTD_REPORT unavailable")
            _accept_reading(response.get("payload"), started)
        except Exception as exc:
            _unavailable(str(exc))
        time.sleep(max(0.05, POLL_SECONDS - (time.monotonic() - started)))
