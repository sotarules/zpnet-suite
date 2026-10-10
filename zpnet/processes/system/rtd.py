"""Teensy-owned PT1000 observations mirrored into Pi SYSTEM context.

Independent of the slow platform poller; REPORT performs no hardware I/O.
PHOTONS uses this fresh RTD context as its normalization authority.
The BME280 environment remains an independent witness.
"""

import copy
import math
import threading
import time

_LOCK = threading.Lock()
_READING = {"source": "TEENSY.SPI1.MAX31865", "status": "INITIALIZING"}
_OBSERVED_AT = 0.0
POLL_SECONDS = 1.0
MAX_AGE_MS = 3000


def _accept_reading(reading: dict, request_started: float) -> None:
    """Store a complete response; never merge old temperature into new faults."""
    global _READING, _OBSERVED_AT
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
    with _LOCK:
        _READING = item
        # Request start conservatively includes RPC transit in sample age.
        _OBSERVED_AT = request_started


def _unavailable(reason: str) -> None:
    global _READING, _OBSERVED_AT
    with _LOCK:
        _READING = {
            "source": "TEENSY.SPI1.MAX31865",
            "status": "UNAVAILABLE",
            "error": reason,
        }
        _OBSERVED_AT = time.monotonic()


def build_rtd_status() -> dict:
    with _LOCK:
        item = copy.deepcopy(_READING)
        observed_at = _OBSERVED_AT
    elapsed_ms = max(0, int((time.monotonic() - observed_at) * 1000))
    if item.get("status") == "OK":
        item["age_ms"] += elapsed_ms
        if item["age_ms"] > MAX_AGE_MS:
            item["status"] = "STALE"
            item.pop("temperature_c", None)
            item.pop("resistance_ohms", None)
    return item



def rtd_feature_status() -> str:
    """Derive readiness from the same aging cache used by SYSTEM.REPORT."""
    status = build_rtd_status()["status"]
    if status == "INITIALIZING":
        return "INITIALIZING"
    return "NOMINAL" if status == "OK" else "HOLD"


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
