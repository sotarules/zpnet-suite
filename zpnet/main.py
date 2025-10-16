"""
ZPNet Main Scheduler Daemon

This module provides the main scheduler loop that dynamically executes
modules based on the polling schedule defined in the SQLite database.
"""

import importlib
import logging
import signal
import sqlite3
import sys
import time
from datetime import datetime, timedelta

from zpnet.shared.logger import setup_logging

DB_PATH = "/home/mule/zpnet/zpnet.db"
MODULES_PATH = "zpnet.modules"
TICK_INTERVAL = 0.1  # 100ms scheduler loop

# --------------------------
# Signal handler
# --------------------------
def handle_sigterm(signum, frame) -> None:
    logging.info("🛑 SIGTERM received, shutting down...")
    sys.exit(0)

# --------------------------
# Module runner
# --------------------------
def run_module(module_name: str) -> None:
    """Dynamically import and run a module, log results to run_history."""
    start_ts = datetime.utcnow()
    start = time.monotonic()
    status = "OK"

    try:
        module = importlib.import_module(f"{MODULES_PATH}.{module_name}")
        module.run()
    except Exception as e:
        logging.exception(f"💥 [runner] module {module_name} failed: {e}")
        status = "FAIL"

    end_ts = datetime.utcnow()
    end = time.monotonic()
    duration_ms = int((end - start) * 1000)

    try:
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO run_history (module_name, start_ts, end_ts, duration_ms, status)
                VALUES (?, ?, ?, ?, ?)
                """,
                (module_name, start_ts.isoformat(), end_ts.isoformat(), duration_ms, status),
            )
            conn.commit()
    except Exception as e:
        logging.warning(f"⚠️ [runner] failed to log run_history for {module_name}: {e}")

# --------------------------
# Main scheduler loop
# --------------------------
def scheduler_loop() -> None:
    """Main loop that checks schedule table and executes due jobs."""
    logging.info("📅 ZPNet Scheduler Started")

    while True:
        now = datetime.utcnow().isoformat()

        try:
            with sqlite3.connect(DB_PATH) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT id, module_name, frequency_sec FROM schedule
                    WHERE next_run_ts IS NULL OR next_run_ts <= ?
                    """,
                    (now,),
                )
                due_jobs = cur.fetchall()

                for job_id, module_name, freq in due_jobs:
                    run_module(module_name)

                    next_run = datetime.utcnow() + timedelta(seconds=freq)
                    cur.execute(
                        """
                        UPDATE schedule
                        SET last_run_ts = ?, next_run_ts = ?
                        WHERE id = ?
                        """,
                        (now, next_run.isoformat(), job_id),
                    )
                    conn.commit()

        except Exception as e:
            logging.warning(f"⚠️ [scheduler] database poll failed: {e}")

        time.sleep(TICK_INTERVAL)

# --------------------------
# Bootstrap entry point
# --------------------------
def bootstrap() -> None:
    """Configure logging, signals, and start the scheduler."""
    setup_logging()
    signal.signal(signal.SIGTERM, handle_sigterm)
    signal.signal(signal.SIGINT, handle_sigterm)

    try:
        scheduler_loop()
    except Exception as e:
        logging.exception(f"⏰ [scheduler] unexpected exception: {e}")
        sys.exit(1)

# --------------------------
# Direct run guard
# --------------------------
if __name__ == "__main__":
    bootstrap()
