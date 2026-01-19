"""
ZPNet Dashboard — Stable Render + WebSocket Edition (v2026-01-09)

Terminal-style real-time dashboard using pygame.

Key invariants:
  • Rendering must NEVER throw
  • No imperative hardware queries in core
  • WebSocket failures must never affect UI
  • Dashboard is an observer, not an instrument

Author: The Mule + GPT
"""

import json
import signal
import sys
import time
import threading
import asyncio
from collections.abc import Generator
from typing import Callable

import pygame
import websockets

from zpnet.shared.db import open_db


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

READOUT_DELAY = 2.0
LOCKED_REFRESH_S = 0.5

WS_HOST = "0.0.0.0"
WS_PORT = 8765


# ---------------------------------------------------------------------
# Database Access (read-only, defensive)
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

    except Exception:
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
# Header
# ---------------------------------------------------------------------
def header_readout(prefix: str = "") -> list[str]:
    ag_net = fetch_aggregate("NETWORK_STATUS")
    ag_bat = fetch_aggregate("BATTERY_STATE_OF_CHARGE")
    ag_env = fetch_aggregate("ENVIRONMENT_STATUS")
    ag_gns = fetch_aggregate("GNSS_DATA")
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
        ag_gns.get("health_state"),
        ag_las.get("health_state"),
        ag_pow.get("health_state"),
        ag_sen.get("health_state"),
        ag_tee.get("health_state"),
        ag_pi.get("health_state"),
    ]

    overall = combine_health([h for h in healths if h])
    return [f"{prefix}NET: {ssid}  BAT: {batt_str}  SYS: {overall}", ""]


# ---------------------------------------------------------------------
# Readout imports
# ---------------------------------------------------------------------
from zpnet.dashboard.readout_blocks import (
    gnss_report_readout,
    laser_status_readout,
    environment_status_readout,
    sensor_scan_readout,
    battery_status_readout,
    power_status_readout,
    network_status_readout,
    teensy_status_readout,
    photodiode_status_readout,
    raspberry_pi_status_readout,
)

Readout = Generator[str, None, None]

READOUTS: list[Callable[[], Readout]] = [
    gnss_report_readout,
    laser_status_readout,
    environment_status_readout,
    sensor_scan_readout,
    battery_status_readout,
    power_status_readout,
    network_status_readout,
    teensy_status_readout,
    photodiode_status_readout,
    raspberry_pi_status_readout,
]


# ---------------------------------------------------------------------
# Event Pump (never blocks)
# ---------------------------------------------------------------------
def pump_events(locked: bool, idx: int, count: int) -> tuple[bool, int]:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            handle_exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                handle_exit()
            elif event.key == pygame.K_l:
                locked = not locked
            elif event.key == pygame.K_SPACE and locked:
                idx = (idx + 1) % count
    return locked, idx


# ---------------------------------------------------------------------
# Renderers
# ---------------------------------------------------------------------
def render_tty(screen, font, header: list[str], body: list[str], clock) -> None:
    screen.fill(BG_COLOR)
    y = 20
    line_h = FONT_SIZE + READOUT_PADDING * 2

    for line in header:
        surf = font.render(line.upper(), True, HEADER_COLOR)
        screen.blit(surf, (20, y))
        y += line_h

    pygame.display.flip()

    for line in body:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                handle_exit()

        surf = font.render(line.upper(), True, TEXT_COLOR)
        screen.blit(surf, (20, y))
        y += line_h

        pygame.display.flip()
        clock.tick(15)


def render_panel(screen, font, header: list[str], body: list[str]) -> None:
    screen.fill(BG_COLOR)
    y = 20
    line_h = FONT_SIZE + READOUT_PADDING * 2

    for line in header:
        surf = font.render(line.upper(), True, HEADER_COLOR)
        screen.blit(surf, (20, y))
        y += line_h

    for line in body:
        surf = font.render(line.upper(), True, TEXT_COLOR)
        screen.blit(surf, (20, y))
        y += line_h

    pygame.display.flip()


# ---------------------------------------------------------------------
# WebSocket Surface (hardened)
# ---------------------------------------------------------------------
_ws_clients: set = set()
_ws_loop: asyncio.AbstractEventLoop | None = None


async def _ws_handler(ws):
    _ws_clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        _ws_clients.discard(ws)


async def _ws_server():
    async with websockets.serve(_ws_handler, WS_HOST, WS_PORT):
        await asyncio.Future()  # run forever


def start_ws_server():
    global _ws_loop
    _ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_ws_loop)
    _ws_loop.run_until_complete(_ws_server())


def ws_broadcast(payload: dict) -> None:
    if not _ws_clients or _ws_loop is None:
        return

    msg = json.dumps(payload, separators=(",", ":"))

    for ws in list(_ws_clients):
        try:
            asyncio.run_coroutine_threadsafe(ws.send(msg), _ws_loop)
        except Exception:
            _ws_clients.discard(ws)


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
    last_rendered: list[str] | None = None

    # Start WebSocket server
    threading.Thread(target=start_ws_server, daemon=True).start()

    while True:
        try:
            locked, readout_idx = pump_events(locked, readout_idx, len(READOUTS))
            prefix = "* " if locked else ""

            header = header_readout(prefix=prefix)
            readout_fn = READOUTS[readout_idx]

            try:
                body = list(readout_fn())
            except Exception:
                body = ["READOUT ERROR"]

            combined = header + body
            payload = {
                "header": header[0].upper(),
                "body": [line.upper() for line in body],
            }

            if locked:
                if combined != last_rendered:
                    render_panel(screen, font, header, body)
                    ws_broadcast(payload)
                    last_rendered = combined

                time.sleep(LOCKED_REFRESH_S)
                continue

            render_tty(screen, font, header, body, clock)
            ws_broadcast(payload)

            time.sleep(READOUT_DELAY)
            readout_idx = (readout_idx + 1) % len(READOUTS)

        except Exception:
            # Absolute last-resort containment
            time.sleep(0.2)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------
if __name__ == "__main__":
    main()
