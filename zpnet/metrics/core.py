"""
ZPNet Metrics — Full-Screen Terminal Display

Green-screen curses application for SSH-based system monitoring.
Designed for the road: run from a ThinkPad over SSH, maximized
in Putty with 3270 font.

Controls:
  L       — Lock/unlock display on current readout
  SPACE   — Advance to next readout (when locked)
  Q       — Quit

Display layout:
  Row 0   — Status header (network, battery, health, GNSS)
  Row 1   — Separator
  Row 2   — Readout name + lock indicator
  Row 3+  — Readout content

When unlocked, cycles through readouts on a fixed cadence.
When locked, holds the current readout until manually advanced.

Usage:
    python -m zpnet.metrics

Author: The Mule + GPT
"""

import curses
import logging
import time

from zpnet.shared.logger import setup_logging
from zpnet.metrics.readout_blocks import READOUTS, status_header

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

REFRESH_INTERVAL_S = 2.0         # screen repaint cadence
CYCLE_INTERVAL_S = 8.0           # auto-advance cadence (when unlocked)


# ---------------------------------------------------------------------
# Curses display engine
# ---------------------------------------------------------------------

def _main(stdscr: curses.window) -> None:
    """
    Main curses loop.

    One fault barrier wraps the entire display lifecycle.
    """
    # ---------------------------------------------------------
    # Terminal setup
    # ---------------------------------------------------------
    curses.curs_set(0)               # hide cursor
    curses.use_default_colors()
    stdscr.nodelay(True)             # non-blocking getch
    stdscr.timeout(int(REFRESH_INTERVAL_S * 1000))

    # Green on black
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_GREEN)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)

    COLOR_NORMAL = curses.color_pair(1)
    COLOR_HEADER = curses.color_pair(2) | curses.A_BOLD
    COLOR_DIM = curses.color_pair(1) | curses.A_DIM
    COLOR_BRIGHT = curses.color_pair(1) | curses.A_BOLD
    COLOR_WARN = curses.color_pair(3) | curses.A_BOLD

    # ---------------------------------------------------------
    # State
    # ---------------------------------------------------------
    readout_index = 0
    locked = False
    last_cycle_time = time.monotonic()

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------
    while True:
        # -----------------------------------------------------
        # Input handling
        # -----------------------------------------------------
        key = stdscr.getch()

        if key == ord("q") or key == ord("Q"):
            break

        elif key == ord("l") or key == ord("L"):
            locked = not locked
            last_cycle_time = time.monotonic()

        elif key == ord(" "):
            if locked:
                readout_index = (readout_index + 1) % len(READOUTS)

        # -----------------------------------------------------
        # Auto-cycle (when unlocked)
        # -----------------------------------------------------
        if not locked:
            now = time.monotonic()
            if now - last_cycle_time >= CYCLE_INTERVAL_S:
                readout_index = (readout_index + 1) % len(READOUTS)
                last_cycle_time = now

        # -----------------------------------------------------
        # Fetch data
        # -----------------------------------------------------
        readout_name, readout_fn = READOUTS[readout_index]

        try:
            header = status_header()
        except Exception:
            header = " STATUS: ERROR"

        try:
            lines = readout_fn()
        except Exception as e:
            lines = [f"ERROR: {e}"]

        # -----------------------------------------------------
        # Render
        # -----------------------------------------------------
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()

        # Row 0 — status header (inverse video)
        header_text = header[:max_x - 1].ljust(max_x - 1)
        try:
            stdscr.addstr(0, 0, header_text, COLOR_HEADER)
        except curses.error:
            pass

        # Row 1 — separator
        sep = "─" * (max_x - 1)
        try:
            stdscr.addstr(1, 0, sep, COLOR_DIM)
        except curses.error:
            pass

        # Row 2 — readout name + lock state
        lock_indicator = " [LOCKED]" if locked else ""
        nav_label = f" {readout_name}{lock_indicator}"
        try:
            stdscr.addstr(2, 0, nav_label, COLOR_BRIGHT)
        except curses.error:
            pass

        # Row 3 — separator
        try:
            stdscr.addstr(3, 0, sep, COLOR_DIM)
        except curses.error:
            pass

        # Row 4+ — readout content
        for i, line in enumerate(lines):
            row = 4 + i
            if row >= max_y - 2:
                break
            text = line[:max_x - 1]
            try:
                stdscr.addstr(row, 0, text, COLOR_NORMAL)
            except curses.error:
                pass

        # Bottom row — help bar
        help_text = " L=Lock  SPACE=Next  Q=Quit"
        try:
            stdscr.addstr(max_y - 1, 0, help_text[:max_x - 1], COLOR_DIM)
        except curses.error:
            pass

        stdscr.refresh()


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    """
    Launch the metrics terminal.

    Not a systemd service — run interactively over SSH.
    """
    setup_logging()

    try:
        curses.wrapper(_main)
    except KeyboardInterrupt:
        pass
    except Exception:
        logging.exception("💥 [metrics] unhandled exception")
        raise