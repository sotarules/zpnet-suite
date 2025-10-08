"""
ZPNet Network Monitor (Self-Contained with iperf3 Upload Test)

Poll network status and emit NETWORK_STATUS events.
"""

import logging
import socket
import time
import requests  # pip install requests
import psutil
import subprocess
import json
from statistics import mean
from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event


# ----------------------
# Helpers
# ----------------------

def get_server_host():
    """Read current ZPNet remote host from env config."""
    try:
        with open("/etc/zpnet.env") as f:
            lines = f.readlines()
        kv = dict(line.strip().split("=", 1) for line in lines if "=" in line)
        return kv.get("ZPNET_REMOTE_HOST", "unknown")
    except Exception:
        return "unknown"


def get_local_ip():
    """Get best-effort local IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


def ping_latency(host="8.8.8.8", port=53, attempts=3, timeout=2):
    """Measure approximate latency by opening TCP socket connections."""
    times = []
    for _ in range(attempts):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            start = time.time()
            s.connect((host, port))
            end = time.time()
            s.close()
            times.append((end - start) * 1000.0)  # ms
        except Exception:
            continue
    return round(mean(times), 2) if times else None


def throughput_test(url="http://speedtest.tele2.net/1MB.zip", timeout=10):
    """Estimate download throughput by fetching ~0.5 MB and measuring speed."""
    try:
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
    except Exception as e:
        logging.warning(f"Throughput test failed: {e}")
    return None


def upload_test(server_host, duration=5, timeout=15):
    """
    Measure upload throughput using iperf3.
    Requires iperf3 server running at `server_host`.
    """
    try:
        result = subprocess.run(
            ["iperf3", "-c", server_host, "-t", str(duration), "-f", "m", "--json"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=True
        )
        data = json.loads(result.stdout.decode())
        # bits_per_second → convert to Mbps
        bps = data.get("end", {}).get("sum_sent", {}).get("bits_per_second", 0)
        return round(bps / 1e6, 2) if bps else None
    except Exception as e:
        logging.warning(f"Upload test failed: {e}")
        return None


def get_isp():
    """Get ISP info from ipinfo.io (free, rate-limited)."""
    try:
        r = requests.get("https://ipinfo.io/json", timeout=5)
        data = r.json()
        return data.get("org", "unknown")
    except Exception:
        return "unknown"


def get_interface_stats():
    """Return RX/TX bytes per interface."""
    try:
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
    except Exception:
        return {}


# ----------------------
# Main routine
# ----------------------

def run():
    """Collect network info and emit NETWORK_STATUS event."""
    server_host = get_server_host()

    payload = {
        "server_host": server_host,
        "local_ip": get_local_ip(),
        "interfaces": get_interface_stats(),
    }

    # Ping
    latency = ping_latency()
    if latency is not None:
        payload["ping_ms"] = latency

    # Download estimate
    payload["download_mbps"] = throughput_test()

    # Upload estimate (iperf3)
    if server_host not in ("unknown", "0.0.0.0"):
        payload["upload_mbps"] = upload_test(server_host)
    else:
        payload["upload_mbps"] = None

    # ISP info
    payload["isp"] = get_isp()

    payload["status"] = "OK" if latency is not None else "FAIL"

    create_event(event_type="NETWORK_STATUS", payload=payload)


def bootstrap():
    setup_logging()
    run()
