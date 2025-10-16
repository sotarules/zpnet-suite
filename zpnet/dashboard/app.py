import json
import sqlite3
import pygame
import time
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import signal
import sys

# ------------------------------
# Signal Handling for Safe Exit
# ------------------------------
def handle_exit(signum=None, frame=None):
    pygame.quit()
    sys.exit(0)

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

# ------------------------------
# Config Constants
# ------------------------------
DB_PATH = Path("/home/mule/zpnet/zpnet.db")
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 480
FONT_NAME = "IBM 3270"
FONT_SIZE = 24
SCROLL_SPEED = 56600  # characters per second
READOUT_PADDING = 2  # blank lines between readouts
TEXT_COLOR = (0, 255, 0)
HEADER_COLOR = (255, 255, 255)
ERROR_COLOR = (255, 0, 0)  # red for errors
BG_COLOR = (0, 0, 0)
LINE_DELAY = 0.05
READOUT_DELAY = 10
LOCKED_LINES = 2  # number of lines reserved at top

# System error display parameters
MAX_ERRORS_DISPLAYED = 3
ERROR_DISPLAY_WINDOW = 3600  # seconds (1 hour)

# ------------------------------
# Utility: Database Access
# ------------------------------
def fetch_aggregate(name):
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute("SELECT payload FROM aggregates WHERE aggregate_type=?", (name,))
        row = cur.fetchone()
        return json.loads(row["payload"]) if row else {}

# ------------------------------
# Header (Locked) Definitions
# ------------------------------
def header_readout():
    """Locked header lines: datetime (PST/PDT), host, battery %, ISP, network status."""
    now = datetime.now(ZoneInfo("America/Los_Angeles")).strftime("%Y-%m-%d %H:%M:%S")
    host = fetch_aggregate("NETWORK_STATUS").get("server_host", "UNKNOWN")
    batt = fetch_aggregate("BATTERY_STATE_OF_CHARGE").get("remaining_pct", None)
    batt_str = f"{batt:.1f}%" if batt is not None else "N/A"

    yield f"{now}  {host}  {batt_str}"

    ag_net = fetch_aggregate("NETWORK_STATUS")
    isp = ag_net.get("isp", "UNKNOWN")
    netstat = ag_net.get("network_status", "UNKNOWN")

    yield f"{isp}  {netstat}"

# ------------------------------
# Error Lines Definitions
# ------------------------------
def system_error_lines():
    ag = fetch_aggregate("SYSTEM_ERROR")
    if not ag:
        return []

    events = []
    now = datetime.utcnow()
    for ev in ag.get("events", []):
        ts = datetime.fromisoformat(ev["timestamp"].replace("Z", "+00:00"))
        age = (now - ts).total_seconds()
        if age <= ERROR_DISPLAY_WINDOW:
            component = ev.get("context", {}).get("component", "UNKNOWN")
            msg = ev.get("message", "UNKNOWN ERROR")
            events.append(f"[{component.upper()}] {msg}")
    return events[:MAX_ERRORS_DISPLAYED]

# ------------------------------
# Readout Definitions
# ------------------------------
def sensor_scan_readout():
    yield "SENSOR SCAN INITIALIZATION SEQUENCE."
    ag = fetch_aggregate("SENSOR_SCAN")
    if not ag:
        yield "SENSOR SCAN STATUS UNAVAILABLE."
        return

    # 1. Raspberry Pi (always show first if present)
    if "Raspberry Pi" in ag:
        yield f"RASPBERRY PI: {ag['Raspberry Pi'].upper()}"

    # 2. Teensy Controller
    if "teensy_status" in ag:
        yield f"TEENSY CONTROLLER: {ag['teensy_status'].upper()}"

    # 3. INA260 sensor lines (all other str fields except known ones)
    for k, v in ag.items():
        if (
            isinstance(v, str)
            and k not in ("Raspberry Pi", "teensy_status", "i2c_count", "i2c_devices")
        ):
            yield f"{k.upper()}: {v.upper()}"

def battery_status_readout():
    ag = fetch_aggregate("BATTERY_STATE_OF_CHARGE")
    if not ag:
        yield "BATTERY STATUS UNAVAILABLE."
        return

    yield "BATTERY STATUS."
    tte = int(ag['tte_minutes'])
    hours, minutes = divmod(tte, 60)
    yield f"TIME-TO-EMPTY: {hours}H {minutes}M."
    yield f"REMAINING PERCENTAGE: {ag['remaining_pct']:.1f}%"
    yield f"WATT-HOURS USED SINCE LAST RECHARGE: {ag['wh_used_since_recharge']:.2f}"
    yield f"WATT-HOURS REMAINING ESTIMATE: {ag['wh_remaining_estimate']:.2f}"

def network_status_readout():
    ag = fetch_aggregate("NETWORK_STATUS")
    if not ag:
        yield "NETWORK STATUS UNAVAILABLE."
        return

    yield "NETWORK STATUS."
    yield f"SERVER HOST: {ag.get('server_host', 'UNKNOWN')}"
    yield f"LOCAL IP: {ag.get('local_ip', '0.0.0.0')}"
    yield f"ISP: {ag.get('isp', 'UNKNOWN')}"

    dl = ag.get("download_mbps")
    ul = ag.get("upload_mbps")
    ping = ag.get("ping_ms")
    if dl is not None:
        yield f"DOWNLOAD: {dl:.2f} MBPS"
    if ul is not None:
        yield f"UPLOAD: {ul:.2f} MBPS"
    if ping is not None:
        yield f"PING: {ping:.2f} MS"

    interfaces = ag.get("interfaces", {})
    for iface, stats in interfaces.items():
        sent_mb = stats.get("bytes_sent", 0) / 1e6
        recv_mb = stats.get("bytes_recv", 0) / 1e6
        yield f"IFACE {iface.upper()}: SENT={sent_mb:.2f} MB RECV={recv_mb:.2f} MB"

READOUTS = [
    sensor_scan_readout,
    battery_status_readout,
    network_status_readout,
]

# ------------------------------
# Text Rendering
# ------------------------------
def render_locked_lines(screen, font, lines):
    screen.fill(BG_COLOR)
    y = 20
    line_height = FONT_SIZE + READOUT_PADDING * 2
    for line in lines:
        surf = font.render(line.upper(), True, HEADER_COLOR)
        screen.blit(surf, (20, y))
        y += line_height
    pygame.display.flip()
    return y

def clear_scroll_area(screen, baseline):
    rect = pygame.Rect(0, baseline, SCREEN_WIDTH, SCREEN_HEIGHT - baseline)
    pygame.draw.rect(screen, BG_COLOR, rect)
    pygame.display.update(rect)

def scroll_text(screen, font, lines, start_y, clock):
    """Render scrolling lines starting below locked header or errors."""
    line_height = FONT_SIZE + READOUT_PADDING * 2
    y = start_y
    for line in lines:
        text_so_far = ""
        for char in line.upper():
            # process quit / key events every frame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    handle_exit()
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    handle_exit()

            text_so_far += char
            clear_rect = pygame.Rect(20, y, SCREEN_WIDTH - 40, FONT_SIZE + READOUT_PADDING * 2)
            pygame.draw.rect(screen, BG_COLOR, clear_rect)
            surf = font.render(text_so_far, True, TEXT_COLOR)
            screen.blit(surf, (20, y))
            pygame.display.flip()

            # use clock.tick instead of time.sleep so events still flow
            clock.tick(SCROLL_SPEED)

        y += line_height
        if y + line_height > SCREEN_HEIGHT:
            screen.scroll(dy=-line_height)
            y -= line_height
        time.sleep(LINE_DELAY)

# ------------------------------
# Main Loop
# ------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ZPNET TERMINAL DASHBOARD")
    font = pygame.font.SysFont(FONT_NAME, FONT_SIZE, bold=False)
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                handle_exit()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                handle_exit()

        header_lines = list(header_readout())
        baseline = render_locked_lines(screen, font, header_lines)
        clear_scroll_area(screen, baseline)

        for readout_fn in READOUTS:
            clear_scroll_area(screen, baseline)
            lines = list(readout_fn())
            scroll_text(screen, font, lines, baseline, clock)
            # during the delay, also stay responsive
            for i in range(int(READOUT_DELAY * 10)):
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        handle_exit()
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        handle_exit()
                clock.tick(10)

if __name__ == "__main__":
    main()
