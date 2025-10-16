"""
ZPNet Shared Recovery Utilities

Provides reusable logic for invoking network recovery procedures,
specifically choosenet.sh and invocation throttling.
"""

import logging
import subprocess
import time
from pathlib import Path

# ---------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------
CHOOSENET_PATH = Path("/usr/local/bin/choosenet.sh")
RECOVERY_STAMP = Path("/home/mule/.cache/zpnet/zpnet_choosenet.last")
RECOVERY_MIN_INTERVAL_SEC = 600  # 10 minutes
SYSTEMD_UNIT_NAME = "choosenet-recovery"
JOURNAL_TAG = "CHOOSENET-RECOVERY"

# ---------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------
def invoke_choosenet() -> bool:
    """
    Conditionally launch choosenet.sh as a systemd transient unit.

    Returns:
        bool: True if invocation was attempted (not throttled).
    """
    if not logging.getLogger().hasHandlers():
        logging.basicConfig(level=logging.INFO)
        logging.info("⚙️ [invoke_choosenet] fallback logging enabled")

    if not should_invoke_recovery():
        logging.info("🔧 [invoke_choosenet] should invoke recovery: *false* (throttled)")
        return False

    try:
        RECOVERY_STAMP.parent.mkdir(parents=True, exist_ok=True)
        RECOVERY_STAMP.touch()
        logging.info("⏳ [invoke_choosenet] cooldown stamp written; launching via systemd-run")

        cmd = [
            "/usr/bin/systemd-run",
            f"--unit={SYSTEMD_UNIT_NAME}",
            "--property=StandardOutput=journal",
            "--property=StandardError=journal",
            f"--property=SyslogIdentifier={JOURNAL_TAG}",
            "--description=ZPNet Choosenet Recovery",
            "--quiet",
            "--collect",
            "--no-block",
            str(CHOOSENET_PATH)
        ]

        logging.debug(f"🧪 [invoke_choosenet] running: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        logging.info(f"🚀 [invoke_choosenet] systemd unit '{SYSTEMD_UNIT_NAME}' started")
        return True

    except subprocess.CalledProcessError as e:
        logging.warning(f"💥 [invoke_choosenet] systemd-run failed: {e}")
        return False

    except Exception as e:
        logging.warning(f"💥 [invoke_choosenet] unexpected error: {e}")
        return False

# ---------------------------------------------------------------------
# Throttling logic
# ---------------------------------------------------------------------
def should_invoke_recovery() -> bool:
    """
    Throttle choosenet.sh invocations to avoid repeated network resets.

    Returns:
        bool: True if enough time has passed since last invocation.
    """
    try:
        if not RECOVERY_STAMP.exists():
            return True
        last = RECOVERY_STAMP.stat().st_mtime
        now_ts = time.time()
        return (now_ts - last) >= RECOVERY_MIN_INTERVAL_SEC
    except Exception as e:
        logging.warning(f"⚠️ [should_invoke_recovery] unexpected error: {e}")
        return True  # default to "safe to run"
