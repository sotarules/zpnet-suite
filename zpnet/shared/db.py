"""
ZPNet Shared SQLite Helper

Provides a centralized, policy-driven SQLite connection factory.
Ensures:
  • WAL mode enabled (persistent)
  • Busy timeout applied (per-connection)
  • Consistent row_factory
  • Dashboard-safe read behavior

Author: The Mule + GPT
"""

import sqlite3

from zpnet.shared.constants import DB_PATH

# ---------------------------------------------------------------------
# Configuration (tuned for ZPNet workload)
# ---------------------------------------------------------------------
BUSY_TIMEOUT_MS = 5000          # wait up to 5s for writers
CONNECT_TIMEOUT_S = 5.0         # Python-level timeout
SYNCHRONOUS_MODE = "NORMAL"     # balance durability vs performance


# ---------------------------------------------------------------------
# One-time database initialization
# ---------------------------------------------------------------------
def initialize_database() -> None:
    """
    Apply persistent SQLite settings.
    Safe to call multiple times.
    """
    with sqlite3.connect(DB_PATH) as conn:
        conn.execute("PRAGMA journal_mode = WAL;")
        conn.execute("PRAGMA synchronous = NORMAL;")


# ---------------------------------------------------------------------
# Connection factory
# ---------------------------------------------------------------------
def open_db(*, read_only: bool = False) -> sqlite3.Connection:
    """
    Open a SQLite connection with ZPNet policy applied.

    Args:
        read_only (bool): If True, open in read-only mode.
                          (Best for dashboards.)

    Returns:
        sqlite3.Connection
    """
    if read_only:
        uri = f"file:{DB_PATH}?mode=ro"
        conn = sqlite3.connect(
            uri,
            uri=True,
            timeout=CONNECT_TIMEOUT_S,
            isolation_level=None,   # autocommit
        )
    else:
        conn = sqlite3.connect(
            DB_PATH,
            timeout=CONNECT_TIMEOUT_S,
        )

    conn.row_factory = sqlite3.Row

    # Per-connection pragmas (NOT persistent)
    conn.execute(f"PRAGMA busy_timeout = {BUSY_TIMEOUT_MS};")
    conn.execute(f"PRAGMA synchronous = {SYNCHRONOUS_MODE};")

    return conn
