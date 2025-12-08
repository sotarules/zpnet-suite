"""
ZPNet Dashboard — X-Safe Event-Pump Hardened Revision (v2025-12-02a)

This version fixes the rare but fatal condition where Xorg terminates the
entire X session due to the dashboard not pumping X events during long-render
animations or sleeps. All time.sleep() calls have been replaced with
event-pumped waits that keep the SDL/X11 connection alive.

Author: The Mule + GPT6-Preview
"""

import json
import signal
import sqlite3
import sys
import time
from collections.abc import Generator
from pathlib import Path

import pygame

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
DB_PATH = Path("/home/mule/zpnet/zpnet.db")
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 480
FONT_NAME = "IBM 3270"
FONT_SIZE = 30
READOUT_PADDING = 2
BG_COLOR = (0, 0, 0)
HEADER_COLOR = (255, 255, 255)
TEXT_COLOR = (0, 255, 0)
SCROLL_SPEED = 120
LINE_DELAY = 0.05
READOUT_DELAY = 10

# ---------------------------------------------------------------------
# Event Pump Helpers
# ---------------------------------------------------------------------
def pump_events():
    """Keep SDL/X11 connection alive."""
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            handle_exit()
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            handle_exit()


def wait_with_pumping(seconds: float, clock: pygame.time.Clock, fps: int = 60):
    """Replace time.sleep with a responsive wait."""
    end = time.time() + seconds
    while time.time() < end:
        pump_events()
        clock.tick(fps)

# ---------------------------------------------------------------------
# Database Access
# ---------------------------------------------------------------------
def fetch_aggregate(name: str) -> dict:
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute("SELECT payload FROM aggregates WHERE aggregate_type=?", (name,))
        row = cur.fetchone()
        return json.loads(row["payload"]) if row else {}

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
def header_readout() -> Generator[str, None, None]:
    ag_net = fetch_aggregate("NETWORK_STATUS")
    ssid = ag_net.get("ssid", "UNKNOWN")

    ag_bat = fetch_aggregate("BATTERY_STATE_OF_CHARGE")
    ag_sen = fetch_aggregate("SENSOR_SCAN")
    ag_pow = fetch_aggregate("POWER_STATUS")
    ag_tee = fetch_aggregate("TEENSY_STATUS")
    ag_pi = fetch_aggregate("RASPBERRY_PI_STATUS")

    batt = ag_bat.get("remaining_pct")
    batt_str = f"{batt:.1f}%" if batt is not None else "N/A"

    healths = [
        ag_net.get("health_state"),
        ag_bat.get("health_state"),
        ag_pow.get("health_state"),
        ag_sen.get("health_state"),
        ag_tee.get("health_state"),
        ag_pi.get("health_state"),
    ]
    overall = combine_health([h for h in healths if h])

    yield f"NET: {ssid}  BAT: {batt_str}  SYS: {overall}"
    yield ""

# ---------------------------------------------------------------------
# Readouts
# ---------------------------------------------------------------------
def sensor_scan_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("SENSOR_SCAN")
    yield f"SENSOR SCAN: {ag.get('health_state', 'DOWN')}"
    for k, v in ag.items():
        if isinstance(v, str) and k != "health_state":
            yield f"{k.upper()}: {v.upper()}"


def battery_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("BATTERY_STATE_OF_CHARGE")
    yield f"BATTERY STATUS: {ag.get('health_state', 'DOWN')}"
    if not ag:
        yield "BATTERY DATA UNAVAILABLE."
        return

    tte_val = ag.get("tte_minutes", 0)
    if isinstance(tte_val, float) and (tte_val == float("inf") or tte_val > 1e8):
        yield "TIME-TO-EMPTY: ∞ (BATTERY FULL)"
    else:
        tte = int(tte_val)
        h, m = divmod(tte, 60)
        yield f"TIME-TO-EMPTY: {h}H {m}M."

    yield f"REMAINING PERCENT: {ag.get('remaining_pct', 0):.1f}%"
    yield f"WH USED SINCE RECHARGE: {ag.get('wh_used_since_recharge', 0):.2f}"
    yield f"WH REMAINING EST: {ag.get('wh_remaining_estimate', 0):.2f}"


def power_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("POWER_STATUS")
    yield f"POWER STATUS: {ag.get('health_state', 'DOWN')}"
    if not ag or "sensors" not in ag:
        yield "POWER DATA UNAVAILABLE."
        return

    load_total_w = 0.0
    battery_power_w = None
    for s in ag["sensors"]:
        label = s.get("label", "UNKNOWN")
        v = s.get("voltage_v", 0.0)
        i = s.get("current_ma", 0.0)
        p = s.get("power_w", 0.0)
        yield f"{label.upper():<14}  {v:>6.3f} V  {i:>7.1f} mA  {p:>6.3f} W"
        if label.lower() == "battery":
            battery_power_w = p
        else:
            load_total_w += p

    yield f"TOTAL LOAD POWER: {load_total_w:.3f} W"

    if battery_power_w and battery_power_w > 0:
        efficiency = (load_total_w / battery_power_w) * 100.0
        yield f"EFFICIENCY: {efficiency:.1f} %"


def network_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("NETWORK_STATUS")
    yield f"NETWORK STATUS: {ag.get('health_state', 'DOWN')}"
    if not ag:
        yield "NETWORK DATA UNAVAILABLE."
        return
    yield f"SERVER HOST: sota.ddns.net"
    yield f"LOCAL IP: {ag.get('local_ip', '0.0.0.0')}"
    yield f"SSID: {ag.get('ssid', 'UNKNOWN')}"
    yield f"PING: {ag.get('ping_ms', 0):.1f} MS"
    yield f"DOWNLOAD: {ag.get('download_mbps', 0):.2f} MBPS"
    yield f"UPLOAD: {ag.get('upload_mbps', 0):.2f} MBPS"


def teensy_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("TEENSY_STATUS")
    yield f"TEENSY STATUS: {ag.get('health_state', 'DOWN')}"
    if not ag:
        yield "TEENSY DATA UNAVAILABLE."
        return
    yield f"FW: {ag.get('fw_version', 'UNKNOWN')}"
    yield f"CPU TEMP: {ag.get('cpu_temp_c', 0):.1f} °C"
    yield f"VCC: {ag.get('vcc_v', 0):.2f} V"
    yield f"FREE HEAP: {ag.get('free_heap_bytes', 0) / 1024:.1f} KB"


def raspberry_pi_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("RASPBERRY_PI_STATUS")
    yield f"RASPBERRY PI STATUS: {ag.get('health_state', 'DOWN')}"

    if not ag:
        yield "RASPBERRY PI DATA UNAVAILABLE."
        return

    yield f"DEVICE: {ag.get('device_name', 'UNKNOWN')}"
    yield f"CPU TEMP: {ag.get('cpu_temp_c', 0):.1f} °C"
    yield f"LOAD (1/5/15): {ag.get('load_1m', 0):.2f} / {ag.get('load_5m', 0):.2f} / {ag.get('load_15m', 0):.2f}"
    yield f"UPTIME: {ag.get('uptime_s', 0) / 3600:.2f} H"

    mem = ag.get("memory", {})
    yield f"MEM USED: {mem.get('used_mb', 0):.0f} / {mem.get('total_mb', 0):.0f} MB ({mem.get('percent', 0):.1f}%)"

    disk = ag.get("disk", {})
    yield f"DISK USED: {disk.get('used_gb', 0):.2f} / {disk.get('total_gb', 0):.2f} GB ({disk.get('percent', 0):.1f}%)"

    uv = ag.get("undervoltage_flags", {})
    now_uv = uv.get("currently_undervolted")
    past_uv = uv.get("previously_undervolted")

    if now_uv is True:
        yield "⚠️ UNDERVOLTAGE: ACTIVE"
    elif past_uv is True:
        yield "⚠️ UNDERVOLTAGE: RECOVERED"
    elif now_uv is False and past_uv is False:
        yield "UNDERVOLTAGE: None detected"
    else:
        yield "UNDERVOLTAGE: Unknown"

# ---------------------------------------------------------------------
# Readout Registry
# ---------------------------------------------------------------------
READOUTS = [
    sensor_scan_readout,
    battery_status_readout,
    power_status_readout,
    network_status_readout,
    teensy_status_readout,
    raspberry_pi_status_readout,
]

# ---------------------------------------------------------------------
# Rendering
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
            pump_events()
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

        wait_with_pumping(LINE_DELAY, clock)

# ---------------------------------------------------------------------
# Main Loop
# ---------------------------------------------------------------------
def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ZPNET TERMINAL DASHBOARD")
    font = pygame.font.SysFont(FONT_NAME, FONT_SIZE, bold=False)
    clock = pygame.time.Clock()

    while True:
        pump_events()

        header_lines = list(header_readout())
        baseline = render_locked_lines(screen, font, header_lines)
        clear_scroll_area(screen, baseline)

        for readout_fn in READOUTS:
            pump_events()
            clear_scroll_area(screen, baseline)

            lines = list(readout_fn())
            scroll_text(screen, font, lines, baseline, clock)

            payload = {
                "header": header_lines[0].upper(),
                "body": [line.upper() for line in lines],
            }
            create_event("DASHBOARD_READOUT", payload)

            # Hardened delay
            wait_with_pumping(READOUT_DELAY, clock)


if __name__ == "__main__":
    main()
