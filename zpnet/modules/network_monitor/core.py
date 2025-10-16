"""
ZPNet Network Monitor (Revamped with definitive connectivity test and quiet self-heal).

Collects network stats and emits NETWORK_STATUS events.
Performs a definitive HTTP GET to http://<ZPNET_REMOTE_HOST>/api/test expecting "ZPNet OK".
If the test fails, silently invokes /usr/local/bin/choosenet.sh (throttled), without logging warnings/errors.
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

# ----------------------
# Constants / Config
# ----------------------
ENV_PATH = Path("/etc/zpnet.env")
CHOOSENET_PATH = Path("/usr/local/bin/choosenet.sh")
RECOVERY_STAMP = Path("/home/mule/.cache/zpnet/zpnet_choosenet.last")  # volatile, cleared on reboot
RECOVERY_MIN_INTERVAL_SEC = 600  # 10 minutes between choosenet attempts
TEST_TIMEOUT_SEC = 5
ZPNET_TEST_PATH = "/api/test"


# ----------------------
# Helpers
# ----------------------

def get_server_host() -> str:
    """Read current ZPNet remote host from env config."""
    with ENV_PATH.open() as f:
        lines = f.readlines()
    kv = dict(line.strip().split("=", 1) for line in lines if "=" in line)
    return kv.get("ZPNET_REMOTE_HOST")


def zpnet_test(server_host) -> bool:
    """
    Definitive connectivity test:
    HTTP GET http://<server_host>/api/test expecting 200 and body containing "ZPNet OK".

    Args: server_host str Server hostname or IP.

    Returns: True if test passes, False otherwise.
    """
    try:
        url = f"http://{server_host}{ZPNET_TEST_PATH}"
        resp = requests.get(url, timeout=TEST_TIMEOUT_SEC)
        return resp.status_code == 200 and "ZPNet OK" in resp.text
    except requests.RequestException:
        return False


def get_local_ip() -> str:
    """
    Get best-effort local IP address.

    Returns: local IP as string.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


def ping_latency():
    """
    Measure approximate latency by opening TCP socket connections.

    Returns: average latency in ms.
    """
    times = []
    host = "8.8.8.8"
    port = 53
    attempts = 3
    timeout = 2
    for _ in range(attempts):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        start = time.time()
        s.connect((host, port))
        end = time.time()
        s.close()
        times.append((end - start) * 1000.0)
    return round(mean(times), 2)


def get_isp():
    """
    Get ISP info from ipinfo.io (free, rate-limited).

    Returns: ISP string.
    """
    r = requests.get("https://ipinfo.io/json", timeout=5)
    data = r.json()
    return data.get("org")


def get_interface_stats():
    """
    Return RX/TX bytes per interface.

    Returns: dict of interface stats, or empty dict on failure.
    """
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


def download_test():
    """
    Estimate download throughput by fetching ~0.5 MB and measuring speed.

    Returns: download speed in Mbps.
    """
    url = "http://speedtest.tele2.net/1MB.zip"
    timeout = 10
    start = time.time()
    r = requests.get(url, stream=True, timeout=timeout)
    total_bytes = 0
    for chunk in r.iter_content(10240):  # 10 KB chunks
        total_bytes += len(chunk)
        if total_bytes >= 500000:  # stop after ~0.5MB
            break
    elapsed = time.time() - start
    if elapsed > 0:
        mbps = (total_bytes * 8 / 1e6) / elapsed
        return round(mbps, 2)
    return 0


def upload_test(server_host):
    """
    Measure upload throughput using iperf3. Requires iperf3 server running at `server_host`.

    Args: server_host str Server hostname or IP.

    Returns: upload speed in Mbps.
    """
    duration = 5
    timeout = 15
    result = subprocess.run(
        ["iperf3", "-c", server_host, "-t", str(duration), "-f", "m", "--json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=True
    )
    data = json.loads(result.stdout.decode())
    bps = data.get("end", {}).get("sum_sent", {}).get("bits_per_second", 0)
    return round(bps / 1e6, 2)


# ----------------------
# Main routine
# ----------------------

def run():
    """
    Collect network info, perform definitive test, self-heal if needed, and emit NETWORK_STATUS event.

    Emits: NETWORK_STATUS event with payload.
    """
    payload = {}
    try:
        server_host = get_server_host()
        payload["server_host"] = server_host
        if not zpnet_test(server_host):
            payload["network_status"] = "DOWN"
            return

        payload["network_status"] = "NOMINAL"
        payload["local_ip"] = get_local_ip()
        payload["interfaces"] = get_interface_stats()
        payload["isp"] = get_isp()
        payload["ping_ms"] = ping_latency()
        payload["download_mbps"] = download_test()
        payload["upload_mbps"] = upload_test(server_host)

    except requests.exceptions.RequestException as e:
        logging.warning("📡 [network_monitor] remote host unreachable - waiting for next poll")

    except Exception as e:
        logging.exception(f"❌[network_monitor] unexpected exception {e}")

    finally:
        create_event(event_type="NETWORK_STATUS", payload=payload)


def bootstrap():
    setup_logging()
    run()
