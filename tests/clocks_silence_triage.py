"""Read-only CLOCKS silence triage: distinguish edge flow from publication flow.

    zt clocks_silence_triage
    zt clocks_silence_triage --interval 10

Use while CLOCKS is latched, before REPAIR, STATS_RESET, Teensy reboot, or
reflash. The script sends only REPORT/INFO RPCs. Two snapshots reveal whether
physical interrupts and firmware campaign/publication counters are advancing.

The full CLOCKS.REPORT_RECOVERY is intentional: it carries the retry, observer
drop, watchdog-block, and candidate counters needed to diagnose a silent
CLOCKS_FRAGMENT. It is a large dynamic Payload report, so do not use this
program to stress-test a device suspected of ongoing heap corruption.

The tool does not classify an instrument lifetime, authorize recovery, or
change any Pi, Teensy, DAC, or database state. A reset or reboot between
snapshots invalidates simple counter-delta interpretations.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import sys
import time
from typing import Any, Sequence

from zpnet.processes.processes import send_command

SCHEMA = "CLOCKS_SILENCE_TRIAGE_V1"

SAMPLE_COMMANDS = (
    ("TEENSY", "CLOCKS", "REPORT_RECOVERY"),
    ("TEENSY", "CLOCKS", "REPORT_ROW_COURT"),
    ("TEENSY", "INTERRUPT", "REPORT_STATUS"),
    ("TEENSY", "INTERRUPT", "REPORT_PPS"),
)

ONE_SHOT_COMMANDS = (
    ("PI", "CLOCKS", "CLOCKS_INFO"),
    ("PI", "CLOCKS", "DAC_INFO"),
    ("TEENSY", "SYSTEM", "PAYLOAD_FATAL_INFO"),
    ("TEENSY", "SYSTEM", "CRASH_INFO"),
    ("TEENSY", "INTERRUPT", "REPORT_SMARTZERO"),
)

COUNTER_PATHS = {
    "TEENSY.CLOCKS.REPORT_RECOVERY": (
        "campaign_seconds", "last_public_count", "candidate_count",
        "campaign_record_stage_count", "campaign_record_take_count",
        "campaign_record_observer_drop_count",
        "fragment_publication_publish_reject_count",
        "fragment_publication_retry_schedule_count",
        "fragment_publication_retry_success_count",
        "fragment_publication_active_retry_exhaustion_count",
        "fragment_publication_service_arm_failures",
    ),
    "TEENSY.CLOCKS.REPORT_ROW_COURT": (
        "court_sequence", "selector_vclock_court.sequence",
        "row.sequence", "anchor.pps_witness_sequence",
    ),
    "TEENSY.INTERRUPT.REPORT_STATUS": (
        "vclock_heartbeat_count", "vclock_one_second_count",
    ),
    "TEENSY.INTERRUPT.REPORT_PPS": (
        "sequence", "gpio_irq_count", "gpio_miss_count",
    ),
}


def emit(kind: str, **fields: Any) -> None:
    print(json.dumps({"type": kind, **fields}, separators=(",", ":"),
                     ensure_ascii=True, allow_nan=False, default=str),
          flush=True)


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def read(machine: str, subsystem: str, command: str,
         *, sample: int) -> dict[str, Any] | None:
    owner = f"{machine}.{subsystem}"
    try:
        response = send_command(
            machine=machine, subsystem=subsystem, command=command,
            retries=1, retry_delay_s=0.0,
        )
        if not isinstance(response, dict) or response.get("success") is not True:
            raise RuntimeError(f"read failed: {response!r}")
        payload = response.get("payload")
        if not isinstance(payload, dict):
            raise RuntimeError(f"missing payload: {response!r}")
        emit("rpc", sample=sample, at_utc=now_utc(), owner=owner,
             command=command, payload=payload)
        return payload
    except Exception as exc:
        emit("rpc_error", sample=sample, at_utc=now_utc(), owner=owner,
             command=command, exception_type=type(exc).__name__,
             error=str(exc))
        return None


def path_get(payload: dict[str, Any], path: str) -> Any:
    value: Any = payload
    for part in path.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(part)
    return value


def numeric_delta(before: Any, after: Any) -> int | None:
    # Reject booleans, missing values, and silently coerced strings.
    if type(before) is not int or type(after) is not int:
        return None
    if after < before:
        return None  # reboot, counter wrap, or reset: do not present as progress
    return after - before


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float, default=5.0,
                        help="seconds between firmware snapshots (1..30, default 5)")
    args = parser.parse_args(argv[1:])
    if not 1.0 <= args.interval <= 30.0:
        parser.error("--interval must be between 1 and 30 seconds")

    emit("header", schema=SCHEMA, at_utc=now_utc(),
         read_only=True, interval_seconds=args.interval,
         warning="Large read-only firmware REPORT_RECOVERY Payload requested twice")
    errors = 0
    for machine, subsystem, command in ONE_SHOT_COMMANDS:
        if read(machine, subsystem, command, sample=0) is None:
            errors += 1

    snapshots: list[dict[str, dict[str, Any]]] = []
    for sample in (1, 2):
        if sample == 2:
            time.sleep(args.interval)
        group: dict[str, dict[str, Any]] = {}
        for machine, subsystem, command in SAMPLE_COMMANDS:
            key = f"{machine}.{subsystem}.{command}"
            result = read(machine, subsystem, command, sample=sample)
            if result is None:
                errors += 1
            else:
                group[key] = result
        snapshots.append(group)

    first, second = snapshots
    deltas: dict[str, dict[str, Any]] = {}
    for key, paths in COUNTER_PATHS.items():
        old = first.get(key)
        new = second.get(key)
        if old is None or new is None:
            continue
        deltas[key] = {
            path: {"first": path_get(old, path),
                   "second": path_get(new, path),
                   "nonnegative_delta": numeric_delta(
                       path_get(old, path), path_get(new, path))}
            for path in paths
        }

    status = second.get("TEENSY.CLOCKS.REPORT_RECOVERY", {})
    emit("summary", at_utc=now_utc(), interval_seconds=args.interval,
         producer_enabled=status.get("enabled"),
         campaign=status.get("campaign"),
         campaign_state=status.get("campaign_state"),
         watchdog_publication_blocked=status.get("watchdog_publication_blocked"),
         watchdog_anomaly_active=status.get("watchdog_anomaly_active"),
         fragment_publication_retry_active=status.get("fragment_publication_retry_active"),
         fragment_publication_retry_reason=status.get("fragment_publication_retry_reason"),
         fragment_publication_last_exhausted_reason=status.get("fragment_publication_last_exhausted_reason"),
         campaign_record_last_stage_name=status.get("campaign_record_last_stage_name"),
         campaign_record_pending=status.get("campaign_record_pending"),
         recover_epoch_ready=status.get("recover_epoch_ready"),
         deltas=deltas, rpc_errors=errors,
         note="Counter deltas are descriptive, not proof of scientific continuity")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
