"""
ZPNet Main Scheduler Daemon  —  Stellar-Compliant Revision

Dynamically executes modules according to the polling schedule defined
in the local SQLite database. Each module run is logged into run_history.
All timestamps are UTC-aware.

Author: The Mule
"""

import importlib
import logging
import signal
import sqlite3
import sys
import time
from datetime import datetime, timedelta, timezone

from zpnet.shared.logger import setup_logging

DB_PATH = "/home/mule/zpnet/zpnet.db"
MODULES_PATH = "zpnet.modules"
TICK_INTERVAL_S = 0.1  # 100 ms scheduler loop


# ---------------------------------------------------------------------
# Signal Handlers
# ---------------------------------------------------------------------
def handle_sigterm(signum, frame) -> None:
    """Graceful shutdown on SIGTERM or SIGINT."""
    logging.info("🛑 SIGTERM received, shutting down...")
    sys.exit(0)


# ---------------------------------------------------------------------
# Module Runner
# ---------------------------------------------------------------------
def run_module(module_name: str) -> None:
    """
    Dynamically import and execute a module's run() function.

    Emits:
        run_history entry containing module_name, timestamps, duration, and status.
    """
    start_ts = datetime.now(timezone.utc)
    start_monotonic = time.monotonic()
    status = "OK"

    try:
        module = importlib.import_module(f"{MODULES_PATH}.{module_name}")
        module.run()
    except Exception as e:
        logging.exception(f"💥 [runner] module {module_name} failed: {e}")
        status = "FAIL"

    end_ts = datetime.now(timezone.utc)
    duration_ms = int((time.monotonic() - start_monotonic) * 1000)

    try:
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO run_history (module_name, start_ts, end_ts, duration_ms, status)
                VALUES (?, ?, ?, ?, ?)
                """,
                (
                    module_name,
                    start_ts.isoformat().replace("+00:00", "Z"),
                    end_ts.isoformat().replace("+00:00", "Z"),
                    duration_ms,
                    status,
                ),
            )
            conn.commit()
    except Exception as e:
        logging.warning(f"⚠️ [runner] failed to log run_history for {module_name}: {e}")


# ---------------------------------------------------------------------
# Scheduler Loop
# ---------------------------------------------------------------------
def scheduler_loop() -> None:
    """
    Main loop that checks the schedule table for due jobs and executes them.
    """
    logging.info("📅 ZPNet Scheduler started")

    while True:
        now_ts = datetime.now(timezone.utc)
        now_iso = now_ts.isoformat().replace("+00:00", "Z")

        try:
            with sqlite3.connect(DB_PATH) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT id, module_name, frequency_sec
                    FROM schedule
                    WHERE next_run_ts IS NULL OR next_run_ts <= ?
                    """,
                    (now_iso,),
                )
                due_jobs = cur.fetchall()

                for job_id, module_name, freq_s in due_jobs:
                    run_module(module_name)

                    next_run = datetime.now(timezone.utc) + timedelta(seconds=freq_s)
                    cur.execute(
                        """
                        UPDATE schedule
                        SET last_run_ts = ?, next_run_ts = ?
                        WHERE id = ?
                        """,
                        (
                            now_iso,
                            next_run.isoformat().replace("+00:00", "Z"),
                            job_id,
                        ),
                    )
                    conn.commit()

        except Exception as e:
            logging.warning(f"⚠️ [scheduler] database poll failed: {e}")

        time.sleep(TICK_INTERVAL_S)


# ---------------------------------------------------------------------
# Bootstrap Entry Point
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Configure logging, signal handlers, and start the scheduler."""
    setup_logging()
    signal.signal(signal.SIGTERM, handle_sigterm)
    signal.signal(signal.SIGINT, handle_sigterm)

    try:
        scheduler_loop()
    except Exception as e:
        logging.exception(f"⏰ [scheduler] unexpected exception: {e}")
        sys.exit(1)


# ---------------------------------------------------------------------
# Direct Run Guard
# ---------------------------------------------------------------------
if __name__ == "__main__":
    bootstrap()
