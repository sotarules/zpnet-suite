"""
ZPNet Metrics — Full-Screen Generalized Campaign Display

Green-screen curses application for SSH-based system monitoring.
Consumes canonical CLOCKS_V4 and PHOTONS_V1 readouts; campaign_master/campaign_detail
schema interpretation remains isolated in readout_blocks and is not a repaint-engine
concern.
Designed for the road: run from a ThinkPad over SSH, maximized
in Putty with 3270 font.

Controls:
  L        — Lock/unlock display on current readout
  SPACE    — Advance to next readout (when locked)
  PGUP/PGDN — Scroll campaign-list readouts
  Q        — Quit

Display layout:
  Row 0   — White status title; ``* `` prefix indicates display lock
  Row 1   — Blank
  Row 2+  — Readout content; the body itself supplies section spacing

When unlocked, cycles through readouts on a fixed cadence.
When locked, holds the current readout until manually advanced.

Usage:
    python -m zpnet.metrics

Author: The Mule + GPT
"""

import curses
import logging
import time
from dataclasses import dataclass
from typing import Callable

from zpnet.shared.logger import setup_logging
from zpnet.metrics.readout_blocks import (
    PHOTONS_UPDATED, READOUTS, status_header,
)

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

REFRESH_INTERVAL_S = 1.0         # screen repaint cadence
CYCLE_INTERVAL_S = 8.0           # auto-advance cadence (when unlocked)
PHOTONS_KEY_POLL_S = 0.050       # keyboard polling; publications wake immediately


@dataclass(frozen=True)
class ReadoutSpec:
    """Core-owned readout registration and body behavior."""

    name: str
    render: Callable[[], list[str]]
    scrollable: bool = False


def _normalize_readout(entry) -> ReadoutSpec:
    """Accept the new descriptor and the legacy ``(name, render)`` tuple."""
    if isinstance(entry, ReadoutSpec):
        return entry
    if len(entry) == 3:
        name, render, scrollable = entry
        return ReadoutSpec(name=name, render=render, scrollable=bool(scrollable))
    name, render = entry
    return ReadoutSpec(name=name, render=render, scrollable=False)


def _page_step(body_height: int) -> int:
    """Move by almost one viewport so adjacent pages retain one context line."""
    return max(1, body_height - 1)


def _photons_visible_lines(lines: list[str], body_height: int) -> list[str]:
    """Keep the fixed headings and newest tail rows on shorter terminals."""
    for index, line in enumerate(lines):
        if line.split() == ["SEC", "LAP", "ACCEPT", "EXCL", "MISSED", "SD", "SE"]:
            header_end = index + 1
            row_count = body_height - header_end
            if row_count > 0:
                return lines[:header_end] + lines[header_end:][-row_count:]
            break
    return lines[:body_height]


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
    stdscr.keypad(True)              # decode navigation keys such as PGUP/PGDN

    # Redefine the ANSI green/white palette entries when the terminal permits
    # mutable colors. GNOME Terminal/VTE supports this; PuTTY commonly does not.
    # On immutable terminals, retain their existing ANSI palette definitions.
    # curses RGB components use 0..1000.
    if curses.can_change_color():
        curses.init_color(curses.COLOR_GREEN, 0, 1000, 0)          # #00FF00
        curses.init_color(curses.COLOR_WHITE, 1000, 1000, 1000)   # #FFFFFF

    # Green body and white title on the terminal's default background.
    # Using curses.COLOR_BLACK here forces ANSI palette color 0, which is not
    # necessarily the terminal's actual background (for example, GNOME Terminal
    # may render it as a dark purple/navy).  Since use_default_colors() is active,
    # -1 preserves the terminal's configured background exactly.
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_WHITE, -1)

    COLOR_NORMAL = curses.color_pair(1)
    COLOR_HEADER = curses.color_pair(2) | curses.A_BOLD

    # ---------------------------------------------------------
    # State
    # ---------------------------------------------------------
    readouts = [_normalize_readout(entry) for entry in READOUTS]
    readout_index = 0
    locked = False
    scroll_offsets = {readout.name: 0 for readout in readouts}
    readout_top_identity = {readout.name: None for readout in readouts}
    last_cycle_time = time.monotonic()
    last_repaint_time = 0.0

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------
    while True:
        # -----------------------------------------------------
        # Input handling
        # -----------------------------------------------------
        live_photons = readouts[readout_index].name == "PHOTONS"
        stdscr.timeout(0 if live_photons else int(REFRESH_INTERVAL_S * 1000))
        key = stdscr.getch()

        if (live_photons and key == -1 and not PHOTONS_UPDATED.is_set()
                and time.monotonic() - last_repaint_time < REFRESH_INTERVAL_S):
            # Sleep without holding up incoming publications. Recheck keyboard
            # input at 50 ms intervals, but repaint only on arrival/key/timer.
            PHOTONS_UPDATED.wait(PHOTONS_KEY_POLL_S)
            continue

        if key == ord("q") or key == ord("Q"):
            break

        elif key == ord("l") or key == ord("L"):
            locked = not locked
            last_cycle_time = time.monotonic()

        elif key == ord(" "):
            if locked:
                readout_index = (readout_index + 1) % len(readouts)
                scroll_offsets[readouts[readout_index].name] = 0

        elif key in (curses.KEY_PPAGE, curses.KEY_NPAGE):
            readout = readouts[readout_index]
            if readout.scrollable:
                max_y, _ = stdscr.getmaxyx()
                body_height = max(1, max_y - 2)
                step = _page_step(body_height)
                current = scroll_offsets.get(readout.name, 0)
                if key == curses.KEY_PPAGE:
                    scroll_offsets[readout.name] = max(0, current - step)
                else:
                    scroll_offsets[readout.name] = current + step

        # -----------------------------------------------------
        # Auto-cycle (when unlocked)
        # -----------------------------------------------------
        if not locked:
            now = time.monotonic()
            if now - last_cycle_time >= CYCLE_INTERVAL_S:
                readout_index = (readout_index + 1) % len(readouts)
                scroll_offsets[readouts[readout_index].name] = 0
                last_cycle_time = now

        # -----------------------------------------------------
        # Fetch data
        # -----------------------------------------------------
        readout = readouts[readout_index]
        readout_name = readout.name

        if readout_name == "PHOTONS":
            # Clear BEFORE taking the snapshot. An arrival during rendering
            # stays signaled and causes another pass instead of being lost.
            PHOTONS_UPDATED.clear()

        try:
            header = status_header()
        except Exception:
            header = " STATUS: ERROR"

        try:
            lines = readout.render()
        except Exception as e:
            lines = [f"ERROR: {e}"]

        # -----------------------------------------------------
        # Render
        # -----------------------------------------------------
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()
        body_height = max(0, max_y - 2)

        scroll_offset = 0
        max_scroll = max(0, len(lines) - body_height)
        if readout.scrollable:
            # The first body line is a stable identity token supplied by the
            # readout.  A new newest campaign resets CAMPAIGNS to the top, while
            # ordinary live value changes do not disturb the operator's position.
            top_identity = lines[0][1:] if lines and lines[0].startswith("\0") else None
            previous_identity = readout_top_identity.get(readout.name)
            if previous_identity is not None and top_identity != previous_identity:
                scroll_offsets[readout.name] = 0
            readout_top_identity[readout.name] = top_identity

            if lines and lines[0].startswith("\0"):
                lines = lines[1:]
                max_scroll = max(0, len(lines) - body_height)
            scroll_offset = min(scroll_offsets.get(readout.name, 0), max_scroll)
            scroll_offsets[readout.name] = scroll_offset

        # Row 0 — white status title. Match the dashboard convention: the
        # title itself carries the lock state as a leading ``* `` marker.
        title_prefix = "* " if locked else ""
        header_text = title_prefix + header.lstrip()
        try:
            stdscr.addstr(0, 0, header_text[:max_x - 1], COLOR_HEADER)
        except curses.error:
            pass

        # Row 1 remains blank. Page names and horizontal rules are deliberately
        # absent; the body content identifies the active readout.
        visible_lines = lines[scroll_offset:scroll_offset + body_height]
        if readout_name == "PHOTONS":
            visible_lines = _photons_visible_lines(lines, body_height)
        for i, line in enumerate(visible_lines):
            row = 2 + i
            if row >= max_y:
                break
            text = line[:max_x - 1]
            try:
                stdscr.addstr(row, 0, text, COLOR_NORMAL)
            except curses.error:
                pass

        stdscr.refresh()
        last_repaint_time = time.monotonic()


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
