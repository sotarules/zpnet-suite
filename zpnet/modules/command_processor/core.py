"""
ZPNet Command Processor  —  Stellar-Compliant Revision

Polls the ZPNet server for pending commands and executes them immediately.
Each command includes a payload with `funktion` and optional `args`.

Supported functions:
  - createEvent: emit a local ZPNet event
  - executeScript: run a shell script (passive utility only)

Healing, routing, or environment regeneration is no longer handled here.

Author: The Mule
"""

import logging
import subprocess
import requests
from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
ZPNET_REMOTE_HOST = "sota.ddns.net"
COMMAND_TIMEOUT_S = 20
COMMAND_ENDPOINT = f"http://{ZPNET_REMOTE_HOST}/api"
HEADERS = {"Connection": "close"}  # ensure clean socket lifecycle

# ---------------------------------------------------------------------
# Command Handlers
# ---------------------------------------------------------------------
def create_event_cmd(event_name: str, payload: dict | None = None) -> dict:
    """Emit an event to the local event queue."""
    create_event(event_name, payload or {})
    return {"status": "event_created", "event": event_name}


def execute_script(script: str, timeout_s: int = 30) -> dict:
    """Run a shell script and return output."""
    try:
        result = subprocess.run(
            script,
            shell=True,
            timeout=timeout_s,
            capture_output=True,
            text=True,
        )
        return {
            "stdout": result.stdout,
            "stderr": result.stderr,
            "returncode": result.returncode,
        }
    except subprocess.TimeoutExpired:
        return {"error": "timeout", "returncode": -1}
    except Exception as e:
        return {"error": str(e), "returncode": -2}

# ---------------------------------------------------------------------
# Function Dispatch Map
# ---------------------------------------------------------------------
HANDLER_MAP = {
    "createEvent": create_event_cmd,
    "executeScript": execute_script,
    # Healing is no longer supported here
}

# ---------------------------------------------------------------------
# Main Logic
# ---------------------------------------------------------------------
def run():
    """
    Fetch and execute commands from the ZPNet server.

    Emits:
        COMMAND_EXECUTED (with result) for success
        COMMAND_FAILED (with traceback) for error
    """
    try:
        response = requests.get(
            COMMAND_ENDPOINT,
            headers=HEADERS,
            timeout=COMMAND_TIMEOUT_S,
        )

        if response.status_code != 200:
            logging.warning(
                f"⚠️ [command_processor] command poll HTTP {response.status_code}: {response.text}"
            )
            return

        commands = response.json()
        if not isinstance(commands, list) or not commands:
            return

        for cmd in commands:
            try:
                payload = cmd.get("payload", {}) or {}
                funktion = payload.get("funktion")
                args = payload.get("args", {}) or {}

                if funktion not in HANDLER_MAP:
                    raise ValueError(f"⚠️ [command_processor] unknown function: {funktion}")

                logging.info(f"📡 [command_processor] *invoke* {funktion}")

                result = HANDLER_MAP[funktion](**args)

                create_event(
                    "COMMAND_EXECUTED",
                    {
                        "command_type": cmd.get("command_type", "unknown"),
                        "funktion": funktion,
                        "args": args,
                        "result": result,
                    },
                )

            except Exception as e:
                logging.exception("💥 [command_processor] Command execution failed")
                create_event("COMMAND_FAILED", {"error": str(e)})

    except Exception as e:
        logging.warning(f"❌ [command_processor] Failed to poll or execute commands: {e}")

# ---------------------------------------------------------------------
# Bootstrap Entry
# ---------------------------------------------------------------------
def bootstrap():
    """Initialize logging and execute run() once."""
    setup_logging()
    run()
