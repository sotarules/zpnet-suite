"""
ZPNet Shared Recovery Utilities

Provides reusable logic for invoking network recovery procedures,
specifically choosenet.sh and invocation throttling.
"""
import logging
import subprocess
import time
from pathlib import Path

# Constants
CHOOSENET_PATH = Path("/usr/local/bin/choosenet.sh")
RECOVERY_STAMP = Path("/home/mule/.cache/zpnet/zpnet_choosenet.last")
RECOVERY_MIN_INTERVAL_SEC = 600  # 10 minutes

def invoke_choosenet() -> bool:
    """
    Conditionally launch /usr/local/bin/choosenet.sh and detach.

    Returns:
        bool: True if invocation attempted.
    """
    if not logging.getLogger().hasHandlers():
        logging.basicConfig(level=logging.INFO)
        logging.info("⚙️ [invoke_choosenet] fallback logging enabled")

    if not should_invoke_recovery():
        logging.info("🔧 [invoke_choosenet] should invoke recovery: *false* (throttled)")
        return False

    RECOVERY_STAMP.parent.mkdir(parents=True, exist_ok=True)
    RECOVERY_STAMP.touch()
    logging.info("⏳ [invoke_choosenet] cooldown stamp written; launching recovery")
    try:
        process = subprocess.Popen(
            [str(CHOOSENET_PATH)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        for line in process.stdout:
            logging.info(f"🛰️ [choosenet.sh] {line.strip()}")

        process.wait(timeout=300)
        logging.info(f"✅ [invoke_choosenet] exit code {process.returncode}")
        return process.returncode == 0

    except subprocess.TimeoutExpired:
        logging.warning("💥 [invoke_choosenet] choosenet.sh timed out!")
        return False

    except Exception as e:
        logging.warning(f"💥 [invoke_choosenet] unexpected error: {e}")
        return False

def should_invoke_recovery() -> bool:
    """
    Throttle choosenet.sh invocations to avoid calling it every poll.

    Returns:
        bool: True if enough time has passed since last invocation.
    """
    if not RECOVERY_STAMP.exists():
        return True
    last = RECOVERY_STAMP.stat().st_mtime
    now_ts = time.time()
    return (now_ts - last) >= RECOVERY_MIN_INTERVAL_SEC