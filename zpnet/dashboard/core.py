"""
ZPNet Dashboard — POWER_STATUS Efficiency Revision (v2025-10-25c)

Displays real-time system aggregates with a minimalist header
showing the current Wi-Fi network (SSID), battery percentage,
and overall system status.

NEW:
    • Added EFFICIENCY line to POWER_STATUS readout.
      Computed as (TOTAL LOAD POWER / BATTERY POWER) × 100.

Author: The Mule
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
    """Terminate dashboard cleanly on SIGINT/SIGTERM."""
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
# Database Access
# ---------------------------------------------------------------------
def fetch_aggregate(name: str) -> dict:
    """Return the most recent aggregate payload as a dict."""
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
    """Combine multiple health states into a single overall value."""
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
    """Top header: network SSID, battery %, and SYS overall health."""
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
    health = ag.get("health_state", "DOWN")
    yield f"SENSOR SCAN: {health}"
    for k, v in ag.items():
        if isinstance(v, str) and k != "health_state":
            yield f"{k.upper()}: {v.upper()}"


def battery_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("BATTERY_STATE_OF_CHARGE")
    health = ag.get("health_state", "DOWN")
    yield f"BATTERY STATUS: {health}"
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
    """
    POWER STATUS readout showing each INA260 device’s voltage, current, and power.
    Computes TOTAL LOAD POWER as sum of all rails excluding the Battery.
    Also displays converter efficiency (load / battery × 100) if calculable.
    """
    ag = fetch_aggregate("POWER_STATUS")
    health = ag.get("health_state", "DOWN")
    yield f"POWER STATUS: {health}"
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
    health = ag.get("health_state", "DOWN")
    yield f"NETWORK STATUS: {health}"
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
    health = ag.get("health_state", "DOWN")
    yield f"TEENSY STATUS: {health}"
    if not ag:
        yield "TEENSY DATA UNAVAILABLE."
        return
    yield f"FW: {ag.get('fw_version', 'UNKNOWN')}"
    yield f"CPU TEMP: {ag.get('cpu_temp_c', 0):.1f} °C"
    yield f"VCC: {ag.get('vcc_v', 0):.2f} V"
    yield f"FREE HEAP: {ag.get('free_heap_bytes', 0) / 1024:.1f} KB"


def raspberry_pi_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("RASPBERRY_PI_STATUS")
    health = ag.get("health_state", "DOWN")
    yield f"RASPBERRY PI STATUS: {health}"

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
    uv_now = uv.get("currently_undervolted")
    uv_past = uv.get("previously_undervolted")

    if uv_now is True:
        yield "⚠️ UNDERVOLTAGE: ACTIVE"
    elif uv_past is True:
        yield "⚠️ UNDERVOLTAGE: RECOVERED (previous event)"
    elif uv_now is False and uv_past is False:
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
# Rendering Utilities
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
        color = TEXT_COLOR
        text_so_far = ""
        for char in line.upper():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    handle_exit()
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    handle_exit()
            text_so_far += char
            clear_rect = pygame.Rect(20, y, SCREEN_WIDTH - 40, FONT_SIZE + READOUT_PADDING * 2)
            pygame.draw.rect(screen, BG_COLOR, clear_rect)
            surf = font.render(text_so_far, True, color)
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

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
            ):
                handle_exit()

        header_lines = list(header_readout())
        baseline = render_locked_lines(screen, font, header_lines)
        clear_scroll_area(screen, baseline)

        for readout_fn in READOUTS:
            clear_scroll_area(screen, baseline)
            lines = list(readout_fn())
            scroll_text(screen, font, lines, baseline, clock)

            payload = {
                "header": header_lines[0].upper(),
                "body": [line.upper() for line in lines],
            }
            create_event("DASHBOARD_READOUT", payload)

            for _ in range(int(READOUT_DELAY * 10)):
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (
                        event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
                    ):
                        handle_exit()
                clock.tick(10)


if __name__ == "__main__":
    main()
