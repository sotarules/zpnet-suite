"""
ZPNet Shared Database Helper — PostgreSQL Backend (psycopg v3)

This module centralizes all database access policy for ZPNet.

Design principles:
  • Short-lived connections
  • Explicit transaction boundaries
  • Multi-writer safe (PostgreSQL)
  • No global locks
  • No retry loops
  • Fail fast, fail loud

Payload semantics:
  • JSON payloads are stored as TEXT
  • Interpretation happens at higher layers

Author: The Mule + GPT
"""

from contextlib import contextmanager
import os
import sys

import psycopg
from psycopg.rows import dict_row


# ---------------------------------------------------------------------
# Connection configuration
# ---------------------------------------------------------------------

# Prefer environment override, fall back to local defaults.
PG_DSN = os.environ.get(
    "ZPNET_PG_DSN",
    "host=127.0.0.1 dbname=zpnet user=zpnet password=mule",
)


def _env_nonnegative_int(name: str, default: int) -> int:
    """Read one integer PostgreSQL policy knob and fail loud if malformed."""
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return int(default)
    try:
        value = int(raw)
    except ValueError as exc:
        raise RuntimeError(f"{name} must be an integer, got {raw!r}") from exc
    if value < 0:
        raise RuntimeError(f"{name} may not be negative, got {value}")
    return value


def _default_application_name() -> str:
    """Return a compact process identity for pg_stat_activity testimony."""
    argv0 = os.path.normpath(sys.argv[0] or "python")
    parts = [part for part in argv0.split(os.sep) if part]
    process = "/".join(parts[-3:]) if parts else "python"
    return f"ZPNET:{process}:{os.getpid()}"


# Bound database waits globally.  A value of 0 disables the corresponding
# PostgreSQL timeout, but the defaults intentionally make indefinite waiting
# impossible.  Long intentional maintenance can override these through the
# environment rather than silently weakening ordinary process behavior.
PG_CONNECT_TIMEOUT_S = _env_nonnegative_int("ZPNET_PG_CONNECT_TIMEOUT_S", 5)
PG_LOCK_TIMEOUT_MS = _env_nonnegative_int("ZPNET_PG_LOCK_TIMEOUT_MS", 5_000)
PG_STATEMENT_TIMEOUT_MS = _env_nonnegative_int(
    "ZPNET_PG_STATEMENT_TIMEOUT_MS", 60_000
)
PG_IDLE_IN_TRANSACTION_TIMEOUT_MS = _env_nonnegative_int(
    "ZPNET_PG_IDLE_IN_TRANSACTION_TIMEOUT_MS", 60_000
)
PG_APPLICATION_NAME = os.environ.get(
    "ZPNET_PG_APPLICATION_NAME", _default_application_name()
).strip() or _default_application_name()

# libpq applies these session settings before the first application statement,
# so a query that is already running server-side is itself covered by the
# statement timeout.  In particular, a dead client cannot leave an unbounded
# historical rewrite running forever and blocking later ZPNet lifetimes.
PG_SESSION_OPTIONS = (
    f"-c lock_timeout={PG_LOCK_TIMEOUT_MS} "
    f"-c statement_timeout={PG_STATEMENT_TIMEOUT_MS} "
    f"-c idle_in_transaction_session_timeout={PG_IDLE_IN_TRANSACTION_TIMEOUT_MS}"
)


# ---------------------------------------------------------------------
# Connection helper
# ---------------------------------------------------------------------

@contextmanager
def open_db(*, row_dict: bool = False):
    """
    Open a PostgreSQL connection with ZPNet policy.

    Args:
        row_dict (bool): If True, rows are returned as dicts
                         (similar to sqlite3.Row).

    Yields:
        psycopg.Connection
    """
    conn = psycopg.connect(
        PG_DSN,
        row_factory=dict_row if row_dict else None,
        connect_timeout=PG_CONNECT_TIMEOUT_S,
        application_name=PG_APPLICATION_NAME,
        options=PG_SESSION_OPTIONS,
    )

    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()
