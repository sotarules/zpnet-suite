"""
ZPNet Choosenet Network Healer — Stellar-Compliant + Constants-Integrated Revision
(v2025-12-29-gzip-aligned)

Performs definitive network test (ZPNet REST API at sota.ddns.net).
If test fails, invokes choosenet.sh healing script and retries until success.
When choosenet.sh reports success, emits CHOOSENET_SUCCESS event containing
the previous and new SSIDs.

Note:
  • Server-side gzip enforcement applies to requests with bodies (POST/PUT).
  • This module uses a bodyless GET for its definitive test and remains unchanged.

Author: The Mule
"""

import logging
import subprocess
import time
import requests

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.constants import (
    ZPNET_REMOTE_HOST,
    ZPNET_TEST_PATH,
    EXPECTED_TEST_STRING,
    HTTP_TIMEOUT,
    CHOOSENET_PATH,
    CHOOSENET_RETRY_INTERVAL_S,
)

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def get_ssid() -> str:
    """Return the currently associated Wi-Fi SSID, or empty string if none."""
    try:
        result = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True,
            check=True,
            text=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return ""


def zpnet_definitive_test() -> bool:
    """
    Check ZPNet API endpoint for definitive OK response.

    This is a bodyless GET request and is not subject to gzip
    request-body policy.
    """
    try:
        url = f"http://{ZPNET_REMOTE_HOST}{ZPNET_TEST_PATH}"
        response = requests.get(url, timeout=HTTP_TIMEOUT)
        return response.status_code == 200 and EXPECTED_TEST_STRING in response.text
    except requests.RequestException as e:
        logging.warning(f"[choosenet] connectivity test failed: {e}")
        return False


def run_choosenet() -> tuple[bool, str]:
    """Run choosenet.sh and return (success, stdout)."""
    try:
        result = subprocess.run(
            [CHOOSENET_PATH],
            capture_output=True,
            text=True,
            timeout=30,
        )
        stdout = (result.stdout or "").strip()
        stderr = (result.stderr or "").strip()

        logging.info(f"[choosenet] script stdout: {stdout}")
        if stderr:
            logging.info(f"[choosenet] script stderr: {stderr}")

        return result.returncode == 0, stdout

    except Exception as e:
        logging.error(f"[choosenet] failed to invoke choosenet.sh: {e}")
        return False, str(e)


# ---------------------------------------------------------------------
# Main Loop
# ---------------------------------------------------------------------
def run() -> None:
    """
    Main daemon loop for the ZPNet choosenet module.
    Never exits. Logs all retries, delays, and script invocations.

    Emits:
        CHOOSENET_SUCCESS when choosenet.sh repairs connectivity.
    """
    logging.info("🚀 ZPNet Choosenet Network Healer started")

    previous_ssid = get_ssid()

    while True:
        # Step 1 — definitive test
        if zpnet_definitive_test():
            logging.info(
                f"✅ ZPNet definitive test passed. SSID {previous_ssid} is healthy."
            )
            previous_ssid = get_ssid()
            time.sleep(CHOOSENET_RETRY_INTERVAL_S)
            continue

        logging.warning(
            f"🚨 ZPNet definitive test FAILED for SSID {previous_ssid}. Attempting recovery..."
        )

        # Step 2 — attempt healing
        success, output = run_choosenet()

        # Step 3 — post-healing evaluation
        if success:
            new_ssid = get_ssid()
            logging.info(f"🛠️ choosenet.sh reported success — SSID now '{new_ssid}'")

            payload = {
                "previous_ssid": previous_ssid or "UNKNOWN",
                "new_ssid": new_ssid or "UNKNOWN",
                "script_output": output or "",
            }

            try:
                create_event("CHOOSENET_SUCCESS", payload)
                logging.info("📡 CHOOSENET_SUCCESS event emitted.")
            except Exception as e:
                logging.warning(f"⚠️ Failed to emit CHOOSENET_SUCCESS: {e}")

            previous_ssid = new_ssid

        else:
            logging.warning("💥 choosenet.sh failed — will retry regardless")

        time.sleep(CHOOSENET_RETRY_INTERVAL_S)


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and enter daemon loop."""
    setup_logging()
    run()
