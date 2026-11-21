"""
ZPNet Choosenet Network Healer

Preserve an associated Wi-Fi network with an IPv4 default gateway, even if
there is no Internet service. The gateway can be a home router or a phone
hotspot: a local laptop-to-Pi SSH connection must not be disrupted because
an external ping fails.

If Wi-Fi loses its route, recheck once after a short settling interval before
invoking choosenet.sh to look for another configured network. Successful
script invocations retain the CHOOSENET_SUCCESS event.

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


GATEWAY_CONFIRM_DELAY_S = 10


def wifi_gateway_available() -> bool:
    """A Wi-Fi association with a default IPv4 gateway is locally usable.

    Neither a remote host nor an SSH client is required. Match the route to
    the actual associated Wi-Fi interface so Ethernet/VPN routes cannot
    falsely suppress recovery of a disconnected hotspot.
    """
    association = subprocess.run(
        ["iwgetid"], capture_output=True, text=True,
    )
    if association.returncode != 0:
        return False

    interface = association.stdout.split(maxsplit=1)[0]
    route = subprocess.run(
        ["ip", "-4", "route", "show", "default", "dev", interface],
        capture_output=True, check=True, text=True,
    )
    return any(line.startswith("default via ") for line in route.stdout.splitlines())


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
        # Step 1 — a local Wi-Fi gateway is sufficient, Internet or not.
        if wifi_gateway_available():
            previous_ssid = get_ssid()
            logging.info(f"✅ Wi-Fi gateway available on SSID '{previous_ssid}'.")
            time.sleep(CHOOSENET_RETRY_INTERVAL_S)
            continue

        # Allow association/DHCP transitions to settle before bouncing Wi-Fi.
        logging.warning(
            f"⚠️ No Wi-Fi default gateway on SSID '{previous_ssid}'; "
            f"rechecking in {GATEWAY_CONFIRM_DELAY_S}s before recovery."
        )
        time.sleep(GATEWAY_CONFIRM_DELAY_S)
        if wifi_gateway_available():
            previous_ssid = get_ssid()
            logging.info(f"✅ Wi-Fi gateway recovered on SSID '{previous_ssid}'.")
            time.sleep(CHOOSENET_RETRY_INTERVAL_S)
            continue

        logging.warning(
            f"🚨 No usable Wi-Fi gateway for SSID '{previous_ssid}'. "
            "Attempting network selection..."
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