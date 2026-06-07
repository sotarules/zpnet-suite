"""
ZPNet Metrics — Full-Screen Terminal Display

Green-screen curses application for SSH-based system monitoring.
Designed for the road: run from a ThinkPad over SSH, maximized
in Putty with 3270 font.

Controls:
  L        — Lock/unlock display on current readout
  SPACE    — Advance to next readout (when locked)
  F9/F12   — OCXO1 DAC coarse down/up
  F10/F11  — OCXO1 DAC fine down/up
  F5/F8    — OCXO2 DAC coarse down/up
  F6/F7    — OCXO2 DAC fine down/up
  Q        — Quit

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
from zpnet.metrics.readout_blocks import READOUTS, adjust_ocxo_dac, status_header

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

REFRESH_INTERVAL_S = 1.0         # screen repaint cadence
CYCLE_INTERVAL_S = 8.0           # auto-advance cadence (when unlocked)

DAC_KEY_HELP = (
    "O1 F9/F10/F11/F12=coarse-/fine-/fine+/coarse+  "
    "O2 F5/F6/F7/F8=coarse-/fine-/fine+/coarse+"
)


# ---------------------------------------------------------------------
# DAC keyboard controls
# ---------------------------------------------------------------------

# Most terminals let curses translate function keys into KEY_F(n).  Treat
# escape-sequence decoding as part of metrics' operator input layer so DAC
# control does not depend on a perfect terminal DB.

ESCAPE_SEQUENCE_READ_WINDOW_S = 0.050


def _read_key(stdscr: curses.window):
    """Read one operator key, decoding raw terminal function-key sequences."""
    key = stdscr.getch()
    if key != 27:  # ESC
        return key

    seq = [27]
    deadline = time.monotonic() + ESCAPE_SEQUENCE_READ_WINDOW_S

    # Keep the outer loop's effective cadence intact: after ESC, drain any
    # already-buffered sequence bytes for a tiny bounded window, then restore
    # normal timeout behavior.
    while time.monotonic() < deadline:
        nxt = stdscr.getch()
        if nxt == -1:
            time.sleep(0.001)
            continue
        seq.append(nxt)

        decoded = _decode_function_key_escape(seq)
        if decoded is not None:
            return decoded

        if len(seq) >= 8:
            break

    decoded = _decode_function_key_escape(seq)
    return decoded if decoded is not None else key


def _decode_function_key_escape(seq: list[int]):
    """Map common raw terminal F-key escape sequences to curses KEY_F(n)."""
    try:
        s = bytes(seq).decode("ascii", errors="ignore")
    except Exception:
        return None

    escape_map = {
        # Common xterm/VT220 encodings for the DAC-control function-key range.
        "\x1b[15~": curses.KEY_F5,
        "\x1b[17~": curses.KEY_F6,
        "\x1b[18~": curses.KEY_F7,
        "\x1b[19~": curses.KEY_F8,
        "\x1b[20~": curses.KEY_F9,
        "\x1b[21~": curses.KEY_F10,
        "\x1b[23~": curses.KEY_F11,
        "\x1b[24~": curses.KEY_F12,
    }
    return escape_map.get(s)


def _dac_key_bindings() -> dict[int, tuple[str, int, str]]:
    """Return DAC key bindings after curses has initialized KEY_F values."""
    return {
        curses.KEY_F9:  ("ocxo1", -1, "coarse down"),
        curses.KEY_F10: ("ocxo1", -1, "fine down"),
        curses.KEY_F11: ("ocxo1", +1, "fine up"),
        curses.KEY_F12: ("ocxo1", +1, "coarse up"),
        curses.KEY_F5:  ("ocxo2", -1, "coarse down"),
        curses.KEY_F6:  ("ocxo2", -1, "fine down"),
        curses.KEY_F7:  ("ocxo2", +1, "fine up"),
        curses.KEY_F8:  ("ocxo2", +1, "coarse up"),
    }


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
    stdscr.keypad(True)               # decode function keys as curses.KEY_F(n)
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
    last_control_message = ""
    last_control_time = 0.0

    dac_key_bindings = _dac_key_bindings()

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------
    while True:
        # -----------------------------------------------------
        # Input handling
        # -----------------------------------------------------
        key = _read_key(stdscr)

        if key == ord("q") or key == ord("Q"):
            break

        elif key == ord("l") or key == ord("L"):
            locked = not locked
            last_cycle_time = time.monotonic()

        elif key == ord(" "):
            if locked:
                readout_index = (readout_index + 1) % len(READOUTS)

        elif key in dac_key_bindings:
            lane, direction, step_kind = dac_key_bindings[key]
            try:
                result = adjust_ocxo_dac(lane=lane, direction=direction, step_kind=step_kind)
                last_control_message = result.get("message", "DAC update complete")
            except Exception as e:
                last_control_message = f"DAC update failed: {e}"
            last_control_time = time.monotonic()

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

        # Operator feedback row — recent DAC control result
        if last_control_message and (time.monotonic() - last_control_time) <= 6.0:
            try:
                stdscr.addstr(max_y - 2, 0, last_control_message[:max_x - 1], COLOR_WARN)
            except curses.error:
                pass

        # Bottom row — help bar
        help_text = " L=Lock  SPACE=Next  " + DAC_KEY_HELP + "  Q=Quit"
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