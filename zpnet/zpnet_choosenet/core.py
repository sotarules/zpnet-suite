"""
ZPNet Choosenet Network Healer
(v2026-03-16-ping-definitive)

Performs definitive network reachability test (ICMP ping to 8.8.8.8).
If test fails, invokes choosenet.sh healing script and retries until success.
When choosenet.sh reports success, emits CHOOSENET_SUCCESS event containing
the previous and new SSIDs.

The definitive test no longer depends on ZPNet Server availability.
A downed or restarting server is not a network fault and must not
trigger WPA supplicant recovery.

Author: The Mule
"""

import logging
import subprocess
import time

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.constants import (
    CHOOSENET_PATH,
    CHOOSENET_RETRY_INTERVAL_S,
    DEFINITIVE_TEST_HOST,
    DEFINITIVE_TEST_TIMEOUT_S,
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
    Verify basic internet reachability via ICMP ping to a public host.

    This tests the network itself, not the ZPNet application stack.
    A downed ZPNet Server no longer triggers false network recovery.
    """
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(DEFINITIVE_TEST_TIMEOUT_S), DEFINITIVE_TEST_HOST],
            capture_output=True,
            text=True,
        )
        return result.returncode == 0
    except Exception as e:
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