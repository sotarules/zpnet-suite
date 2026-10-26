"""Operator maintenance: repair CLOCKS without destructive history fallback.

    zt clocks_repair_preserve --status
    zt clocks_repair_preserve --execute

--execute requests CLOCKS.REPAIR with preserve_history_only=true. It can STOP
an active campaign, retire its restore ancestry, and reset instrument clock
statistics; it never authorizes the repair worker's destructive fallback.
The operator should retain a PostgreSQL backup before using any repair command.
No direct database writes, firmware calls, or background polling in this tool.
main(argv) receives argv[0] as required by the zt runner.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import sys
from typing import Sequence

from zpnet.processes.processes import send_command

SCHEMA = "CLOCKS_PRESERVE_REPAIR_OPERATOR_V1"


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--status", action="store_true", help="read current CLOCKS repair status")
    action.add_argument("--execute", action="store_true", help="start a preservation-only CLOCKS repair")
    args = parser.parse_args(argv[1:])

    if args.execute:
        response = send_command(
            machine="PI", subsystem="CLOCKS", command="REPAIR",
            args={"preserve_history_only": True},
        )
        result = {
            "schema": SCHEMA,
            "action": "PRESERVE_HISTORY_ONLY_REPAIR_REQUEST",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "response": response,
        }
    else:
        response = send_command(machine="PI", subsystem="CLOCKS", command="REPORT")
        payload = response.get("payload") or {}
        result = {
            "schema": SCHEMA,
            "action": "REPAIR_STATUS_READ_ONLY",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "success": response.get("success"),
            "message": response.get("message"),
            "repair": payload.get("repair"),
            "operational_state": payload.get("operational_state"),
            "campaign": payload.get("campaign"),
            "campaign_state": payload.get("campaign_state"),
        }
    print(json.dumps(result, separators=(",", ":"), ensure_ascii=True, default=str), flush=True)
    return 0 if response.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
