"""
ZPNet Command Processor

Polls the ZPNet server for pending commands and executes them immediately.
Each command includes a `payload.funktion` and `args`.
"""

import logging
import requests
import subprocess
import json
import traceback
from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging
from datetime import datetime
from pathlib import Path

# --------------------------
# Configuration
# --------------------------
TIMEOUT = 5
ENV_PATH = Path("/etc/zpnet.env")
ZPNET_REMOTE_HOST = "localhost"

# Load override if available
try:
    with open(ENV_PATH, "r") as f:
        for line in f:
            if line.startswith("ZPNET_REMOTE_HOST="):
                ZPNET_REMOTE_HOST = line.strip().split("=", 1)[1]
except Exception as e:
    logging.warning(f"Could not read {ENV_PATH}: {e}")

COMMAND_ENDPOINT = f"http://{ZPNET_REMOTE_HOST}/api"
logging.info(f"📡 Polling commands from: {COMMAND_ENDPOINT}")


# --------------------------
# Command Handlers
# --------------------------
def createEvent(eventName: str, payload: dict = None):
    """Emit an event to the local event queue."""
    create_event(eventName, payload or {})
    return {"status": "event_created", "eventName": eventName}


def executeScript(script: str, timeout: int = 30):
    """Run a shell script and return output."""
    try:
        result = subprocess.run(
            script,
            shell=True,
            timeout=timeout,
            capture_output=True,
            text=True
        )
        return {
            "stdout": result.stdout,
            "stderr": result.stderr,
            "returncode": result.returncode
        }
    except subprocess.TimeoutExpired:
        return {"error": "timeout", "returncode": -1}
    except Exception as e:
        return {"error": str(e), "returncode": -2}


# --------------------------
# Function Dispatcher
# --------------------------
HANDLER_MAP = {
    "createEvent": createEvent,
    "executeScript": executeScript
}


# --------------------------
# Main Task
# --------------------------
def run():
    """Fetch and execute commands from the ZPNet server."""
    try:
        response = requests.get(COMMAND_ENDPOINT, timeout=TIMEOUT)
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
                args = payload.get("args", {})

                if funktion not in HANDLER_MAP:
                    raise ValueError(f"Unknown function: {funktion}")

                result = HANDLER_MAP[funktion](**args)

                create_event("COMMAND_EXECUTED", {
                    "command_type": cmd.get("command_type"),
                    "funktion": funktion,
                    "args": args,
                    "result": result
                })

            except Exception as e:
                logging.exception("Command execution failed")
                create_event("COMMAND_FAILED", {
                    "command_type": cmd.get("command_type"),
                    "funktion": cmd.get("payload", {}).get("funktion"),
                    "args": cmd.get("payload", {}).get("args", {}),
                    "error": str(e),
                    "traceback": traceback.format_exc()
                })

    except Exception as e:
        logging.warning(f"❌ Failed to poll or execute commands: {e}")
        create_event("SYSTEM_ERROR", {
            "component": "command_processor",
            "exception": str(e),
            "traceback": traceback.format_exc()
        })

def bootstrap():
    setup_logging()
    run()
