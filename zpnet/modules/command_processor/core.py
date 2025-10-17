"""
ZPNet Command Processor  —  Stellar-Compliant Revision

Polls the ZPNet server for pending commands and executes them immediately.
Each command includes a payload with `funktion` and optional `args`.
Supported actions include emitting events, running shell scripts,
and triggering network recovery via choosenet.sh.

Author: The Mule
"""

import logging
import subprocess
import traceback
from pathlib import Path

import requests

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging
from zpnet.shared.recovery import invoke_choosenet

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
ENV_PATH = Path("/etc/zpnet.env")
ZPNET_REMOTE_HOST = "localhost"
COMMAND_TIMEOUT_S = 5

try:
    with ENV_PATH.open("r") as f:
        for line in f:
            if line.startswith("ZPNET_REMOTE_HOST="):
                ZPNET_REMOTE_HOST = line.strip().split("=", 1)[1]
except Exception as e:
    logging.warning(f"⚠️ Could not read {ENV_PATH}: {e}")

COMMAND_ENDPOINT = f"http://{ZPNET_REMOTE_HOST}/api"
logging.info(f"📡 Command endpoint: {COMMAND_ENDPOINT}")

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


def choose_network() -> dict:
    """Invoke the choosenet.sh recovery script via systemd-run."""
    invoked = invoke_choosenet()
    return {"invoked": invoked}


# ---------------------------------------------------------------------
# Function Dispatcher
# ---------------------------------------------------------------------
HANDLER_MAP = {
    "createEvent": create_event_cmd,
    "executeScript": execute_script,
    "chooseNetwork": choose_network,
}

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run():
    """
    Fetch and execute commands from the ZPNet server.

    Emits:
        COMMAND_EXECUTED: after successful execution.
        COMMAND_FAILED: on error or invalid command.
        SYSTEM_ERROR: if command polling fails catastrophically.
    """
    try:
        response = requests.get(COMMAND_ENDPOINT, timeout=COMMAND_TIMEOUT_S)
        if response.status_code != 200:
            logging.warning(f"⚠️ Command poll HTTP {response.status_code}: {response.text}")
            return

        commands = response.json()
        if not isinstance(commands, list) or not commands:
            return

        for cmd in commands:
            try:
                payload = cmd.get("payload", {})
                funktion = payload.get("funktion")
                args = payload.get("args", {}) or {}

                if funktion not in HANDLER_MAP:
                    raise ValueError(f"Unknown function: {funktion}")

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
                logging.exception("Command execution failed")
                create_event(
                    "COMMAND_FAILED",
                    {
                        "command_type": cmd.get("command_type", "unknown"),
                        "funktion": cmd.get("payload", {}).get("funktion"),
                        "args": cmd.get("payload", {}).get("args", {}),
                        "error": str(e),
                        "traceback": traceback.format_exc(),
                    },
                )

    except Exception as e:
        logging.warning(f"❌ Failed to poll or execute commands: {e}")
        create_event(
            "SYSTEM_ERROR",
            {
                "component": "command_processor",
                "exception": str(e),
                "traceback": traceback.format_exc(),
            },
        )


def bootstrap():
    """Initialize logging and execute run() once."""
    setup_logging()
    run()
