"""
ZPNet Dashboard — PostgreSQL-Compliant Edition (v2025-12-19-psql)

Terminal-style real-time dashboard using pygame with dynamic readouts.

Features:
  • Locked/unlocked mode (L to toggle, SPACE to cycle in locked)
  • Displays full system status from aggregated Postgres records
  • Emits DASHBOARD_READOUT events for telemetry auditing

Author: The Mule + GPT
"""

import json
import signal
import sys
import time
from collections.abc import Generator
from typing import Callable

import pygame

from zpnet.shared.db import open_db
from zpnet.shared.events import create_event


# ---------------------------------------------------------------------
# Signal Handling
# ---------------------------------------------------------------------
def handle_exit(signum: int | None = None, frame: object | None = None) -> None:
    pygame.quit()
    sys.exit(0)


signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)


# ---------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 480
FONT_NAME = "IBM 3270"
FONT_SIZE = 30
READOUT_PADDING = 2
BG_COLOR = (0, 0, 0)
HEADER_COLOR = (255, 255, 255)
TEXT_COLOR = (0, 255, 0)
SCROLL_SPEED = 120
LINE_DELAY = 0.05
READOUT_DELAY = 4
LOCKED_REFRESH_S = 1.0


# ---------------------------------------------------------------------
# DB Helper
# ---------------------------------------------------------------------
def fetch_aggregate(name: str) -> dict:
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                "SELECT payload FROM aggregates WHERE aggregate_type = %s",
                (name,),
            )
            row = cur.fetchone()

        if not row or row["payload"] is None:
            return {}

        payload = row["payload"]

        if isinstance(payload, dict):
            return payload

        if isinstance(payload, (str, bytes, bytearray)):
            return json.loads(payload)

        return {}

    except Exception as e:
        # Optional: log once while stabilizing
        # logging.debug(f"[dashboard] fetch_aggregate({name}) failed: {e}")
        return {}


# ---------------------------------------------------------------------
# Health Combiner
# ---------------------------------------------------------------------
def combine_health(states: list[str]) -> str:
    if not states:
        return "DOWN"
    if all(s == "NOMINAL" for s in states):
        return "NOMINAL"
    if any(s == "DOWN" for s in states):
        return "DOWN"
    return "HOLD"


# ---------------------------------------------------------------------
# Header (with optional prefix for LOCKED mode)
# ---------------------------------------------------------------------
def header_readout(prefix: str = "") -> Generator[str, None, None]:
    ag_net = fetch_aggregate("NETWORK_STATUS")
    ag_bat = fetch_aggregate("BATTERY_STATE_OF_CHARGE")
    ag_env = fetch_aggregate("ENVIRONMENT_STATUS")
    ag_gnss = fetch_aggregate("GNSS_DATA")
    ag_las = fetch_aggregate("LASER_STATUS")
    ag_pow = fetch_aggregate("POWER_STATUS")
    ag_sen = fetch_aggregate("SENSOR_SCAN")
    ag_tee = fetch_aggregate("TEENSY_STATUS")
    ag_pi  = fetch_aggregate("RASPBERRY_PI_STATUS")

    ssid = ag_net.get("ssid", "UNKNOWN")
    batt = ag_bat.get("remaining_pct")
    batt_str = f"{batt:.1f}%" if batt is not None else "N/A"

    healths = [
        ag_net.get("health_state"),
        ag_bat.get("health_state"),
        ag_env.get("health_state"),
        ag_gnss.get("health_state"),
        ag_las.get("health_state"),
        ag_pow.get("health_state"),
        ag_sen.get("health_state"),
        ag_tee.get("health_state"),
        ag_pi.get("health_state"),
    ]

    overall = combine_health([h for h in healths if h])
    yield f"{prefix}NET: {ssid}  BAT: {batt_str}  SYS: {overall}"
    yield ""


# ---------------------------------------------------------------------
# Typing
# ---------------------------------------------------------------------
Readout = Generator[str, None, None]


# ---------------------------------------------------------------------
# Import your individual readouts here
# ---------------------------------------------------------------------
from zpnet.dashboard.readout_blocks import (
    gnss_data_readout,
    laser_status_readout,
    environment_status_readout,
    sensor_scan_readout,
    battery_status_readout,
    power_status_readout,
    network_status_readout,
    teensy_status_readout,
    raspberry_pi_status_readout,
)


# ---------------------------------------------------------------------
# Registry (order defines display sequence)
# ---------------------------------------------------------------------
READOUTS: list[Callable[[], Readout]] = [
    gnss_data_readout,
    laser_status_readout,
    environment_status_readout,
    sensor_scan_readout,
    battery_status_readout,
    power_status_readout,
    network_status_readout,
    teensy_status_readout,
    raspberry_pi_status_readout,
]

# ---------------------------------------------------------------------
# SDL Event Pump
# ---------------------------------------------------------------------
def pump_events(locked: bool, readout_idx: int, readout_count: int) -> tuple[bool, int]:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            handle_exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                handle_exit()
            elif event.key == pygame.K_l:
                locked = not locked
            elif event.key == pygame.K_SPACE and locked:
                readout_idx = (readout_idx + 1) % readout_count
    return locked, readout_idx


# ---------------------------------------------------------------------
# Rendering Helpers
# ---------------------------------------------------------------------
def render_locked_lines(screen, font, lines: list[str]) -> int:
    screen.fill(BG_COLOR)
    y = 20
    line_h = FONT_SIZE + READOUT_PADDING * 2
    for line in lines:
        surf = font.render(line.upper(), True, HEADER_COLOR)
        screen.blit(surf, (20, y))
        y += line_h
    pygame.display.flip()
    return y


def clear_scroll_area(screen, baseline: int) -> None:
    rect = pygame.Rect(0, baseline, SCREEN_WIDTH, SCREEN_HEIGHT - baseline)
    pygame.draw.rect(screen, BG_COLOR, rect)
    pygame.display.update(rect)


def scroll_text(screen, font, lines: list[str], start_y: int, clock) -> None:
    line_h = FONT_SIZE + READOUT_PADDING * 2
    y = start_y
    for line in lines:
        text_so_far = ""
        for char in line.upper():
            text_so_far += char
            clear_rect = pygame.Rect(20, y, SCREEN_WIDTH - 40, FONT_SIZE + READOUT_PADDING * 2)
            pygame.draw.rect(screen, BG_COLOR, clear_rect)
            surf = font.render(text_so_far, True, TEXT_COLOR)
            screen.blit(surf, (20, y))
            pygame.display.flip()
            clock.tick(SCROLL_SPEED)
        y += line_h
        if y + line_h > SCREEN_HEIGHT:
            screen.scroll(dy=-line_h)
            y -= line_h
        time.sleep(LINE_DELAY)


# ---------------------------------------------------------------------
# Main Loop
# ---------------------------------------------------------------------
def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ZPNET TERMINAL DASHBOARD")
    font = pygame.font.SysFont(FONT_NAME, FONT_SIZE, bold=False)
    clock = pygame.time.Clock()

    locked = False
    readout_idx = 0

    while True:
        locked, readout_idx = pump_events(locked, readout_idx, len(READOUTS))
        prefix = "* " if locked else ""
        header_lines = list(header_readout(prefix=prefix))
        baseline = render_locked_lines(screen, font, header_lines)
        clear_scroll_area(screen, baseline)

        if locked:
            lines = list(READOUTS[readout_idx]())
            render_locked_lines(screen, font, header_lines + lines)
            time.sleep(LOCKED_REFRESH_S)
            continue

        for readout_fn in READOUTS:
            locked, readout_idx = pump_events(locked, readout_idx, len(READOUTS))
            prefix = "* " if locked else ""
            header_lines = list(header_readout(prefix=prefix))
            baseline = render_locked_lines(screen, font, header_lines)
            clear_scroll_area(screen, baseline)

            lines = list(readout_fn())
            scroll_text(screen, font, lines, baseline, clock)

            payload = {
                "header": header_lines[0].upper(),
                "body": [line.upper() for line in lines],
            }
            create_event("DASHBOARD_READOUT", payload)
            time.sleep(READOUT_DELAY)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------
if __name__ == "__main__":
    main()
