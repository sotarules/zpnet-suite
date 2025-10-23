import time
import os
import logging

def setup_logging(level: int | None = None):
    """
    Minimalist logging setup for systemd-managed services.
    Keeps only level + message, removes redundant timestamp and PID.
    """
    try:
        if level is None:
            env_level = os.getenv("ZPNET_LOGLEVEL", "INFO").upper()
            level = getattr(logging, env_level, logging.INFO)

        logging.basicConfig(
            level=level,
            format="[%(levelname)s] %(message)s",
        )

        # Optional: make sure UTC conversion is still consistent if any other formatter uses asctime
        logging.Formatter.converter = time.gmtime

    except Exception:
        logging.basicConfig(level=logging.INFO)
        logging.info("⚠️ Logging fallback activated")
