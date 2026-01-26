"""
ZPNet Shared Logging Utilities — Verbose Exception Safe Revision

Guarantees full traceback visibility for logging.exception()
across threads, services, and systemd environments.
"""

import os
import time
import logging
from zpnet.shared.constants import DEFAULT_LOG_LEVEL


def setup_logging(level: int | None = None, force: bool = True) -> None:
    """
    Minimalist logging setup for systemd-managed services
    with guaranteed verbose exception output.

    Key guarantee:
      • logging.exception() ALWAYS prints full traceback
    """
    try:
        # --------------------------------------------------
        # Resolve log level
        # --------------------------------------------------
        if level is None:
            env_level = os.getenv("ZPNET_LOGLEVEL", DEFAULT_LOG_LEVEL).upper()
            level = getattr(logging, env_level, logging.INFO)

        # --------------------------------------------------
        # Configure logging
        # --------------------------------------------------
        logging.basicConfig(
            level=level,
            format="[%(levelname)s] %(message)s",
            force=force,  # CRITICAL: avoid handler reuse bugs
        )

        # --------------------------------------------------
        # Ensure UTC timestamps everywhere
        # --------------------------------------------------
        logging.Formatter.converter = time.gmtime

        # --------------------------------------------------
        # Sanity check (debug-only)
        # --------------------------------------------------
        logging.debug("[logger] initialized @ %s", logging.getLevelName(level))
        logging.debug("[logger] handlers: %s", logging.root.handlers)
        logging.debug(
            "[logger] effectiveLevel(root): %s",
            logging.getLevelName(logging.root.level),
        )

    except Exception:
        # --------------------------------------------------
        # Absolute fail-open logging
        # --------------------------------------------------
        logging.basicConfig(
            level=logging.INFO,
            format="[%(levelname)s] %(message)s",
            force=True,
        )
        logging.exception("[logger] logging initialization failed — fallback engaged")
