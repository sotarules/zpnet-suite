"""
ZPNet Shared Constants  —  Stellar-Compliant Revision (v2025-10-28a)

Centralized configuration constants shared across all ZPNet modules.
Use this file for immutable system-wide parameters (e.g., HTTP timeouts,
retry policies, key paths, or version metadata).

Author: The Mule
"""
import os
from pathlib import Path

# ---------------------------------------------------------------------
# Versioning / Metadata
# ---------------------------------------------------------------------
ZPNET_VERSION = "ZPNet 3.0.0"
ZPNET_AUTHOR = "The Mule"

# ---------------------------------------------------------------------
# HTTP / Network Defaults
# ---------------------------------------------------------------------
# Default HTTP timeouts for all REST calls (connect, read)
HTTP_TIMEOUT = (3, 10)             # seconds
# Default retry count for transient network errors
HTTP_RETRY_TOTAL = 2
HTTP_RETRY_BACKOFF = 1.0           # seconds

# ---------------------------------------------------------------------
# Filesystem / Database
# ---------------------------------------------------------------------
ZPNET_ROOT = Path("/home/mule/zpnet")
DB_PATH = ZPNET_ROOT / "zpnet.db"
CACHE_PATH = Path("/home/mule/.cache/zpnet")

# ---------------------------------------------------------------------
# Logging / System
# ---------------------------------------------------------------------
DEFAULT_LOG_LEVEL = "INFO"
LOG_DATE_FMT = "%Y-%m-%dT%H:%M:%SZ"  # UTC ISO format
SYSTEMD_UNIT_MAIN = "zpnet.service"

# ---------------------------------------------------------------------
# Scheduler / Timing
# ---------------------------------------------------------------------
TICK_INTERVAL_S = 0.1              # scheduler tick
WATCHDOG_INTERVAL_S = 60           # heartbeat ping to systemd
RESTART_DELAY_S = 5                # seconds between systemd restarts

# ---------------------------------------------------------------------
# Miscellaneous
# ---------------------------------------------------------------------
ZPNET_REMOTE_HOST = "sota.ddns.net"
ZPNET_TEST_PATH = "/api/test"
EXPECTED_TEST_STRING = "ZPNet OK"

# Emoji map for consistent logging aesthetics
EMOJI = {
    "ok": "✅",
    "fail": "❌",
    "warn": "⚠️",
    "net": "📡",
    "boot": "🚀",
    "shutdown": "🛑",
    "recovery": "🛠️",
    "heartbeat": "💓",
}

# ---------------------------------------------------------------------
# Teensy Serial Configuration
# ---------------------------------------------------------------------
TEENSY_SERIAL_PORT = os.environ.get("ZPNET_TEENSY_PORT", "/dev/zpnet-teensy")
TEENSY_BAUDRATE = 115200
TEENSY_RECONNECT_DELAY_S = 5     # seconds between reconnection attempts
TEENSY_READ_TIMEOUT_S = 1        # serial read timeout (seconds)

# ---------------------------------------------------------------------
# Choosenet Network Healer
# ---------------------------------------------------------------------
CHOOSENET_PATH = "/usr/local/bin/choosenet.sh"
CHOOSENET_RETRY_INTERVAL_S = 120  # seconds between connectivity checks

# ---------------------------------------------------------------------
# Laser / Photodiode Semantics
# ---------------------------------------------------------------------

LASER_PHOTODIODE_ON_THRESHOLD_V = 0.05

# ---------------------------------------------------------------------
# Utility Helpers
# ---------------------------------------------------------------------
def http_endpoint(path: str) -> str:
    """Return full HTTP endpoint using default host."""
    if not path.startswith("/"):
        path = "/" + path
    return f"http://{ZPNET_REMOTE_HOST}{path}"
