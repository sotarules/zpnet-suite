"""
ZPNet Main Scheduler Daemon

This module provides the scheduler loop and helpers.
"""

import importlib
import logging
import os
import signal
import sqlite3
import sys
import time
from datetime import datetime, timedelta

from zpnet.shared.logger import setup_logging
from zpnet.shared.recovery import invoke_choosenet

DB_PATH = "/home/mule/zpnet/zpnet.db"
MODULES_PATH = "zpnet.modules"
TICK_INTERVAL = 0.1  # 100ms

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

# --------------------------
# Main scheduler loop
# --------------------------
def scheduler_loop() -> None:
    """Main loop that checks schedule table and executes due jobs."""
    logging.info("📅 ZPNet Scheduler Started")
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

# --------------------------
# Bootstrap helper (conditional debug attach)
# --------------------------
def bootstrap() -> None:
    """Configure logging, signals, and start the scheduler."""
    setup_logging()
    signal.signal(signal.SIGTERM, handle_sigterm)
    signal.signal(signal.SIGINT, handle_sigterm)

    try:
        env_host = None
        env_path = "/etc/zpnet.env"
        if os.path.exists(env_path):
            with open(env_path, "r") as f:
                for line in f:
                    if line.startswith("ZPNET_REMOTE_HOST="):
                        env_host = line.strip().split("=", 1)[1]
                        break

        # Only attach if explicitly enabled *and* host is 192.168.1.105
        if env_host == "192.168.1.105":
            import pydevd_pycharm
            pydevd_pycharm.settrace(
                host=env_host,
                port=5678,
                stdout_to_server=True,
                stderr_to_server=True,
                suspend=False
            )
            logging.info(f"🐞 Debugger attached env_host={env_host}")
        else:
            logging.info(f"🐞 Debug attach skipped env_host={env_host}")

        scheduler_loop()
    except Exception as e:
        logging.exception(f"⏰ [scheduler] unexpected exception: {e}")
        invoke_choosenet()
        sys.exit(1)
