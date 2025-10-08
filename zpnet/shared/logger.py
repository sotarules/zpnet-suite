import logging
import time
import os


def setup_logging(level: int | None = None):
    """
    Configure global logging to use UTC ISO8601 timestamps.
    Defaults to INFO unless overridden by the ZPNET_LOGLEVEL environment variable.

    Args:
        level (int | None): Explicit log level. If None, falls back to env var or INFO.
    """
    if level is None:
        # Check environment variable
        env_level = os.getenv("ZPNET_LOGLEVEL", "INFO").upper()
        level = getattr(logging, env_level, logging.INFO)

    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%SZ",
    )
    # Force UTC timestamps
    logging.Formatter.converter = time.gmtime
