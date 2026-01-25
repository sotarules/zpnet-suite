"""
ZPNet Network Monitor — VPN-Aware + Timeout-Hardened Revision
(v2025-12-29-gzip)

Collects network statistics and emits NETWORK_STATUS events.
All POST requests now comply with the global ZPNet policy requiring
gzip-compressed request bodies.

Author: The Mule
"""
import logging
from statistics import mean
import socket
import subprocess
import time
import random
import string

import psutil
import requests

from zpnet.shared.constants import (
    ZPNET_REMOTE_HOST,
    ZPNET_TEST_PATH,
    EXPECTED_TEST_STRING,
    HTTP_TIMEOUT,
)
from zpnet.shared.events import create_event
from zpnet.shared.http import gzip_text
from zpnet.shared.logger import setup_logging


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def get_ssid() -> str:
    """Returns the currently associated Wi-Fi SSID, or empty string if none."""
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


def zpnet_test() -> bool:
    """Definitive connectivity test against ZPNet API endpoint."""
    try:
        url = f"http://{ZPNET_REMOTE_HOST}{ZPNET_TEST_PATH}"
        resp = requests.get(url, timeout=HTTP_TIMEOUT)
        return resp.status_code == 200 and EXPECTED_TEST_STRING in resp.text
    except requests.RequestException as e:
        logging.warning(f"[network_monitor] definitive test failed: {e}")
        return False


def get_local_ip() -> str:
    """Get best-effort local IP address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip


def ping_latency_ms() -> float:
    """Measure approximate latency (ms) via TCP connect times."""
    host = "8.8.8.8"
    port = 53
    attempts = 3
    timeout_s = 2
    times = []

    for _ in range(attempts):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout_s)
        try:
            start = time.time()
            s.connect((host, port))
            end = time.time()
            times.append((end - start) * 1000.0)
        except Exception:
            continue
        finally:
            s.close()

    return round(mean(times), 2) if times else 0.0


def get_interface_stats() -> dict:
    """Return RX/TX bytes per interface."""
    stats = psutil.net_io_counters(pernic=True)
    return {
        iface: {
            "bytes_sent": s.bytes_sent,
            "bytes_recv": s.bytes_recv,
            "packets_sent": s.packets_sent,
            "packets_recv": s.packets_recv,
        }
        for iface, s in stats.items()
    }


def download_test_mbps() -> float:
    """Estimate download throughput by fetching a 1MB payload from ZPNet server."""
    url = f"http://{ZPNET_REMOTE_HOST}/api/download_test"
    headers = {"Connection": "close", "Accept-Encoding": "identity"}
    start = time.time()
    r = requests.get(url, headers=headers, timeout=HTTP_TIMEOUT)
    text = r.text
    elapsed = time.time() - start
    bits = len(text) * 8
    return round((bits / 1e6) / elapsed, 2) if elapsed > 0 else 0.0


def upload_test_mbps() -> float:
    """
    Estimate upload throughput by POSTing a 1MB ASCII payload
    using gzip-compressed request bodies.
    """
    url = f"http://{ZPNET_REMOTE_HOST}/api/upload_test"

    payload = "".join(
        random.choice(string.ascii_uppercase)
        for _ in range(1024 * 1024)
    )

    body, headers = gzip_text(payload)

    start = time.time()
    r = requests.post(
        url,
        data=body,
        headers=headers,
        timeout=HTTP_TIMEOUT,
    )
    elapsed = time.time() - start

    bits = len(payload) * 8
    return round((bits / 1e6) / elapsed, 2) if elapsed > 0 else 0.0


# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run() -> None:
    """
    Collect network info, perform definitive test,
    and emit NETWORK_STATUS event.
    """
    payload = {}

    try:
        payload["ssid"] = get_ssid()

        # definitive test
        test_ok = zpnet_test()
        payload["network_status"] = "NOMINAL" if test_ok else "DOWN"

        if not test_ok:
            return

        # Normal metrics
        payload["local_ip"] = get_local_ip()
        payload["interfaces"] = get_interface_stats()
        payload["ping_ms"] = ping_latency_ms()

        # Handle transient HTTP faults gently
        try:
            payload["download_mbps"] = download_test_mbps()
            payload["upload_mbps"] = upload_test_mbps()
        except requests.RequestException:
            pass
        except Exception as e:
            logging.warning(
                f"⚠️ [network_monitor] unexpected error during throughput test: {e}"
            )

    except Exception as e:
        # Only truly unexpected cases reach here
        logging.warning(
            f"❌ [network_monitor] unexpected network_monitor failure: {e}"
        )

    finally:
        create_event("NETWORK_STATUS", payload)


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and run once (for debugging or scheduled use)."""
    setup_logging()
    run()
