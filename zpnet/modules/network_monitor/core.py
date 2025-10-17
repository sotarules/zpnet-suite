"""
ZPNet Network Monitor  —  Stellar-Compliant Revision

Collects network stats and emits NETWORK_STATUS events.
Performs a definitive connectivity test to the remote ZPNet host.
If the test fails, invokes choosenet.sh (throttled) for recovery.

Author: The Mule
"""

import json
import logging
import socket
import subprocess
import time
from pathlib import Path
from statistics import mean

import psutil
import requests

from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging
from zpnet.shared.recovery import invoke_choosenet

# ---------------------------------------------------------------------
# Constants / Config
# ---------------------------------------------------------------------
ENV_PATH = Path("/etc/zpnet.env")
CHOOSENET_PATH = Path("/usr/local/bin/choosenet.sh")
RECOVERY_STAMP = Path("/home/mule/.cache/zpnet/zpnet_choosenet.last")

RECOVERY_MIN_INTERVAL_SEC = 600  # 10 minutes between choosenet attempts
TEST_TIMEOUT_SEC = 5
ZPNET_TEST_PATH = "/api/test"


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def get_server_host() -> str:
    """Read current ZPNet remote host from env config."""
    with ENV_PATH.open() as f:
        lines = f.readlines()
    kv = dict(line.strip().split("=", 1) for line in lines if "=" in line)
    return kv.get("ZPNET_REMOTE_HOST", "localhost")


def zpnet_test(server_host: str) -> bool:
    """Definitive connectivity test."""
    try:
        url = f"http://{server_host}{ZPNET_TEST_PATH}"
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


def get_isp() -> str:
    """Return ISP string via ipinfo.io (rate-limited)."""
    try:
        r = requests.get("https://ipinfo.io/json", timeout=5)
        return r.json().get("org", "UNKNOWN")
    except requests.RequestException:
        return "UNKNOWN"


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
    """Estimate download throughput by fetching ~0.5 MB and timing it."""
    url = "http://speedtest.tele2.net/1MB.zip"
    timeout_s = 10
    start = time.time()
    r = requests.get(url, stream=True, timeout=timeout_s)
    total_bytes = 0
    for chunk in r.iter_content(10240):  # 10 KB chunks
        total_bytes += len(chunk)
        if total_bytes >= 500000:  # stop after ~0.5 MB
            break
    elapsed = time.time() - start
    if elapsed > 0:
        return round((total_bytes * 8 / 1e6) / elapsed, 2)
    return 0.0


def upload_test_mbps(server_host: str) -> float:
    """Measure upload throughput using iperf3 (requires server)."""
    duration_s = 5
    timeout_s = 15
    try:
        result = subprocess.run(
            ["iperf3", "-c", server_host, "-t", str(duration_s), "-f", "m", "--json"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
            check=True,
        )
        data = json.loads(result.stdout.decode())
        bps = data.get("end", {}).get("sum_sent", {}).get("bits_per_second", 0)
        return round(bps / 1e6, 2)
    except Exception:
        return 0.0


# ---------------------------------------------------------------------
# Main Routine
# ---------------------------------------------------------------------
def run():
    """
    Collect network info, perform definitive test, self-heal if needed,
    and emit NETWORK_STATUS event.
    """
    payload = {}
    try:
        server_host = get_server_host()
        payload["server_host"] = server_host

        # definitive test
        test_ok = zpnet_test(server_host)
        payload["network_status"] = "NOMINAL" if test_ok else "DOWN"

        if not test_ok:
            invoke_choosenet()
            create_event("NETWORK_STATUS", payload)
            return

        # Normal metrics
        payload["local_ip"] = get_local_ip()
        payload["interfaces"] = get_interface_stats()
        payload["isp"] = get_isp()
        payload["ping_ms"] = ping_latency_ms()
        payload["download_mbps"] = download_test_mbps()
        payload["upload_mbps"] = upload_test_mbps(server_host)

    except Exception as e:
        logging.exception(f"❌ [network_monitor] unexpected exception: {e}")
    finally:
        create_event("NETWORK_STATUS", payload)


def bootstrap():
    """Setup logging and run once (for debugging or scheduled use)."""
    setup_logging()
    run()
