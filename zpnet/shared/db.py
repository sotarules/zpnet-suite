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
    )

    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()
