"""
ZPNet Shared Logging Utilities — Unified Level + Force-Setup Revision (v2025-10-28d)

Provides minimalist, systemd-friendly logging setup used across all
ZPNet modules.  Uses the global DEFAULT_LOG_LEVEL defined in
zpnet.shared.constants so that verbosity can be controlled centrally.

Features:
    • Unified log level (INFO by default, overridable via constants or env)
    • Force option ensures per-module reconfiguration when necessary
    • UTC time normalization for consistent timestamps

Author: The Mule
"""

import os
import time
import logging
from zpnet.shared.constants import DEFAULT_LOG_LEVEL  # ← unified import


def setup_logging(level: int | None = None, force: bool = False) -> None:
    """
    Minimalist logging setup for systemd-managed services.

    Args:
        level (int | None): Optional explicit log level.
            If None, uses environment variable ZPNET_LOGLEVEL or
            shared.constants.DEFAULT_LOG_LEVEL.
        force (bool): If True, forces reconfiguration even if
            handlers already exist (Python 3.8+).

    Behavior:
        • Removes redundant timestamps (systemd adds its own)
        • Emits clean `[LEVEL] message` lines
        • Sets global UTC conversion for any asctime use
    """
    try:
        # Determine desired log level
        if level is None:
            env_level = os.getenv("ZPNET_LOGLEVEL", DEFAULT_LOG_LEVEL).upper()
            level = getattr(logging, env_level, logging.INFO)

        logging.basicConfig(
            level=level,
            format="[%(levelname)s] %(message)s",
            force=force,  # ensures reset when called repeatedly
        )

        # Ensure UTC timestamps in any derived formatters
        logging.Formatter.converter = time.gmtime

        # Optional confirmation message for debugging (comment out in prod)
        logging.debug(f"[logger] initialized @ {logging.getLevelName(level)}")
        logging.debug(f"[logger] handlers: {logging.root.handlers}")
        logging.debug(f"[logger] effectiveLevel(root): {logging.getLevelName(logging.root.level)}")

    except Exception as e:
        # Fail open: ensure we still have logging available
        logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
        logging.info(f"⚠️ Logging fallback activated due to setup error: {e}")
