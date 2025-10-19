"""
ZPNet Choosenet Network Healer  —  Stellar-Compliant Daemon

Performs definitive network test (ZPNet REST API at sota.ddns.net).
If test fails, invokes choosenet.sh healing script and retries until success.
Never gives up. Logs everything.

Author: The Mule
"""
import logging
import subprocess
import time

import requests

from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
ZPNET_TEST_URL = "http://sota.ddns.net/api/test"
EXPECTED_STRING = "ZPNet OK"
RETRY_INTERVAL_SEC = 120
CHOOSENET_PATH = "/usr/local/bin/choosenet.sh"


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def zpnet_definitive_test() -> bool:
    """Check ZPNet API endpoint for definitive OK response."""
    try:
        response = requests.get(ZPNET_TEST_URL, timeout=3)
        return response.status_code == 200 and EXPECTED_STRING in response.text
    except requests.RequestException as e:
        logging.warning(f"[choosenet] connectivity test failed: {e}")
        return False


def run_choosenet() -> tuple[bool, str]:
    """Run choosenet.sh and return (success, stdout)."""
    try:
        result = subprocess.run(
            [CHOOSENET_PATH], capture_output=True, text=True, timeout=30
        )
        logging.info(f"[choosenet] script stdout: {result.stdout.strip()}")
        logging.info(f"[choosenet] script stderr: {result.stderr.strip()}")
        return result.returncode == 0, result.stdout.strip()
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
    """
    logging.info("🚀 ZPNet Choosenet Network Healer started")

    while True:
        if zpnet_definitive_test():
            logging.info("✅ ZPNet definitive test passed. Network is healthy.")
            time.sleep(RETRY_INTERVAL_SEC)
            continue

        logging.warning("🚨 ZPNet definitive test FAILED. Attempting recovery...")
        success, output = run_choosenet()

        if success:
            logging.info("🛠️  choosenet.sh ran without error — rechecking after delay")
        else:
            logging.warning("💥 choosenet.sh failed — will retry regardless")

        time.sleep(RETRY_INTERVAL_SEC)


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and enter daemon loop."""
    setup_logging()
    run()
