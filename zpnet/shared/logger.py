import time
import os

def setup_logging(level: int | None = None):
    """
    Safe wrapper for logging setup.
    Will fall back to basicConfig if configuration fails.
    """
    import logging

    try:
        if level is None:
            env_level = os.getenv("ZPNET_LOGLEVEL", "INFO").upper()
            level = getattr(logging, env_level, logging.INFO)

        logging.basicConfig(
            level=level,
            format="%(asctime)s [%(levelname)s] %(message)s",
            datefmt="%Y-%m-%dT%H:%M:%SZ",
        )
        logging.Formatter.converter = time.gmtime
    except Exception:
        logging.basicConfig(level=logging.INFO)
        logging.info("⚠️ Logging fallback activated")
