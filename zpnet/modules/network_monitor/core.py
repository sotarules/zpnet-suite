"""
ZPNet Network Monitor — VPN-Aware Revision (v2025-10-19b)

Collects network statistics and emits NETWORK_STATUS events.
This version removes the ISP field, as VPN tunneling via AirVPN
renders external IP-based ISP lookups meaningless.

Author: The Mule
"""

import json
import logging
import socket
import subprocess
import time
import requests
import random
import string
import psutil

from statistics import mean

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Constants / Config
# ---------------------------------------------------------------------
ZPNET_REMOTE_HOST = "sota.ddns.net"
ZPNET_TEST_PATH = "/api/test"
TEST_TIMEOUT_SEC = 5

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
    """Definitive connectivity test."""
    try:
        url = f"http://{ZPNET_REMOTE_HOST}{ZPNET_TEST_PATH}"
        resp = requests.get(url, timeout=TEST_TIMEOUT_SEC)
        return resp.status_code == 200 and "ZPNet OK" in resp.text
    except requests.RequestException:
        return False


def get_local_ip() -> str:
    """Get best-effort local IP address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
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
        start = time.time()
        s.connect((host, port))
        end = time.time()
        s.close()
        times.append((end - start) * 1000.0)

    return round(mean(times), 2)


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
    headers = {
        "Connection": "close",
        "Accept-Encoding": "identity"  # explicitly request no compression
    }
    start = time.time()
    r = requests.get(url, headers=headers)
    text = r.text
    elapsed = time.time() - start
    bits = len(text) * 8
    return round((bits / 1e6) / elapsed, 2)

def upload_test_mbps() -> float:
    """Estimate upload throughput by POSTing a 1MB ASCII payload to ZPNet Server."""
    url = f"http://{ZPNET_REMOTE_HOST}/api/upload_test"
    headers = {
        "Connection": "close",
        "Accept-Encoding": "identity"  # request server not to gzip response
    }
    payload = ''.join(random.choice(string.ascii_uppercase) for _ in range(1024 * 1024))
    start = time.time()
    r = requests.post(url, data=payload.encode('utf-8'), headers=headers)
    elapsed = time.time() - start
    bits = len(payload) * 8
    return round((bits / 1e6) / elapsed, 2)

# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run():
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
        payload["download_mbps"] = download_test_mbps()
        payload["upload_mbps"] = upload_test_mbps()

    except Exception as e:
        logging.exception(f"❌ [network_monitor] unexpected exception: {e}")
    finally:
        create_event("NETWORK_STATUS", payload)


def bootstrap():
    """Setup logging and run once (for debugging or scheduled use)."""
    setup_logging()
    run()
