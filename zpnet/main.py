"""
ZPNet Main Scheduler Daemon

This module provides the scheduler loop and helpers.
It is launched via `python3 -m zpnet` which delegates through `__main__.py`.
"""

import signal
import sys
import time
import sqlite3
import importlib
import logging
import traceback
from datetime import datetime, timedelta

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event  # <-- NEW

DB_PATH = "/home/mule/zpnet/zpnet.db"
MODULES_PATH = "zpnet.modules"
TICK_INTERVAL = 0.1  # 100ms


# --------------------------
# Signal handler
# --------------------------
def handle_sigterm(signum, frame):
    msg = "SIGTERM received, shutting down..."
    logging.info(msg)
    create_event("SYSTEM_INFO", {
        "component": "scheduler",
        "signal": signum,
        "message": msg
    })
    sys.exit(0)

# --------------------------
# Module runner
# --------------------------
def run_module(module_name: str) -> None:
    """Dynamically import and run a module, log results to run_history."""
    try:
        start_ts = datetime.utcnow()
        start = time.monotonic()

        module = importlib.import_module(f"{MODULES_PATH}.{module_name}")
        module.run()

        end_ts = datetime.utcnow()
        end = time.monotonic()
        duration_ms = int((end - start) * 1000)

        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO run_history (module_name, start_ts, end_ts, duration_ms, status)
                VALUES (?, ?, ?, ?, ?)
                """,
                (module_name, start_ts.isoformat(), end_ts.isoformat(), duration_ms, "OK"),
            )
            conn.commit()

    except Exception as e:
        err_msg = f"⚠️ Error running {module_name}: {e}"
        logging.exception(err_msg)
        create_event(
            "SYSTEM_ERROR",
            {
                "component": module_name,
                "exception": str(e),
                "traceback": traceback.format_exc(),
            },
        )
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO run_history (module_name, start_ts, end_ts, duration_ms, status)
                VALUES (?, ?, ?, ?, ?)
                """,
                (
                    module_name,
                    start_ts.isoformat() if "start_ts" in locals() else None,
                    datetime.utcnow().isoformat(),
                    0,
                    f"ERROR: {e}",
                ),
            )
            conn.commit()


# --------------------------
# Main scheduler loop
# --------------------------
def scheduler_loop() -> None:
    """Main loop that checks schedule table and executes due jobs."""
    logging.info("📅 ZPNet Scheduler Started")
    try:
        while True:
            now = datetime.utcnow().isoformat()
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

                    now_ts = datetime.utcnow()
                    next_run = now_ts + timedelta(seconds=freq)

                    cur.execute(
                        """
                        UPDATE schedule
                        SET last_run_ts = ?, next_run_ts = ?
                        WHERE id = ?
                        """,
                        (now_ts.isoformat(), next_run.isoformat(), job_id),
                    )
                    conn.commit()

            time.sleep(TICK_INTERVAL)
    except Exception as e:
        err_msg = f"💥 Scheduler loop crashed: {e}"
        logging.exception(err_msg)
        create_event(
            "SYSTEM_ERROR",
            {
                "component": "scheduler_loop",
                "exception": str(e),
                "traceback": traceback.format_exc(),
            },
        )
        raise  # bubble up so systemd can restart if configured


# --------------------------
# Bootstrap helper (optional)
# --------------------------
def bootstrap() -> None:
    """Configure logging, signals, and start the scheduler."""
    setup_logging()
    signal.signal(signal.SIGTERM, handle_sigterm)
    signal.signal(signal.SIGINT, handle_sigterm)
    try:
        scheduler_loop()
    except Exception as e:
        err_msg = f"Fatal error in bootstrap: {e}"
        logging.exception(err_msg)
        create_event(
            "SYSTEM_ERROR",
            {
                "component": "bootstrap",
                "exception": str(e),
                "traceback": traceback.format_exc(),
            },
        )
        sys.exit(1)
