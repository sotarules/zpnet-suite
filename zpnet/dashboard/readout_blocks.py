"""
ZPNet Dashboard Readout Blocks — SYSTEM Snapshot Edition

All readouts are derived directly from the authoritative SYSTEM snapshot.

Invariants:
  • No database access
  • No aggregates
  • No events
  • No inference beyond explicit tests
  • Always fresh
  • Defenseless (exceptions propagate to outer fault barrier)

Author: The Mule + GPT
"""
import logging
from collections.abc import Generator

from zpnet.processes.processes import send_command

log = logging.getLogger("zpnet.dashboard")

# ---------------------------------------------------------------------
# Shared SYSTEM snapshot fetcher (AUTHORITATIVE)
# ---------------------------------------------------------------------

def get_system_snapshot() -> dict:
    return send_command(machine="PI", subsystem="SYSTEM", command="REPORT")["payload"]


def get_clocks_report() -> dict:
    return send_command(machine="TEENSY", subsystem="CLOCKS", command="REPORT")["payload"]

# ---------------------------------------------------------------------
# GNSS REPORT (snapshot)
# ---------------------------------------------------------------------

def gnss_report_readout() -> Generator[str, None, None]:
    g = get_system_snapshot()["gnss"]

    # Headline
    yield f"GNSS REPORT: {g.get('lock_quality', 'UNKNOWN')}"

    # Time validity & PPS
    time_valid = g.get("time_valid")
    pps_valid  = g.get("pps_valid")

    yield (
        f"TIME VALID: {'YES' if time_valid else 'NO'}"
        f" | PPS: {'YES' if pps_valid else 'NO'}"
    )

    # UTC date/time
    if "date" in g or "time" in g:
        yield (
            f"UTC: "
            f"{g.get('date', '----')} "
            f"{g.get('time', '--:--:--')}"
        )

    # Discipline & constellations
    if "discipline" in g or "position_mode" in g:
        yield (
            f"DISCIPLINE: {g.get('discipline', 'N/A')}"
            f" | MODE: {g.get('position_mode', '---')}"
        )

    # Satellites & geometry
    if "satellites" in g or "hdop" in g:
        yield (
            f"SATELLITES: {g.get('satellites', '?')}"
            f" | HDOP: {g.get('hdop', float('nan')):.2f}"
        )

    # Position
    if "latitude_deg" in g and "longitude_deg" in g:
        yield f"LATITUDE:  {g['latitude_deg']:.6f}°"
        yield f"LONGITUDE: {g['longitude_deg']:.6f}°"

    # Altitude stack
    if "altitude_m" in g:
        yield f"ALTITUDE (MSL): {g['altitude_m']:.1f} m"

    if "ellipsoid_height_m" in g or "geoid_sep_m" in g:
        yield (
            f"ELLIPSOID: {g.get('ellipsoid_height_m', float('nan')):.1f} m"
            f" | GEOID Δ: {g.get('geoid_sep_m', float('nan')):.1f} m"
        )

    # Motion (optional but compact)
    if "speed_knots" in g or "course_deg" in g:
        yield (
            f"SPEED / COURSE: "
            f"{g.get('speed_knots', 0.0):.2f} kn"
            f" / {g.get('course_deg', 0.0):.1f}°"
        )


# ---------------------------------------------------------------------
# CLOCKS (synthetic clock substrate)
# ---------------------------------------------------------------------

def seconds_to_hms(seconds: int) -> str:
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def clocks_status_readout() -> Generator[str, None, None]:
    c = get_clocks_report()

    state = c.get("campaign_state")

    # ------------------------------------------------------------
    # Status line
    # ------------------------------------------------------------

    if state == "STARTED":
        campaign = c.get("campaign")
        secs = c.get("campaign_seconds", 0)
        elapsed_hms = seconds_to_hms(secs)
        yield f"CLOCKS: STARTED {elapsed_hms} {campaign}"
    else:
        yield f"CLOCKS: {state}"

    yield ""

    # ------------------------------------------------------------
    # Clock table
    # ------------------------------------------------------------

    yield f"{'CLOCK':<8} {'TAU':>14} {'PPB':>14}"

    # GNSS baseline
    yield f"{'GNSS':<8} {'1.0000000000':>14} {'0.00':>14}"

    # DWT
    yield f"{'DWT':<8} {c['tau_dwt']:>14.10f} {c['dwt_ppb']:>14.2f}"

    # OCXO (optional / diagnostic)
    # yield f"{'OCXO':<8} {c['tau_ocxo']:>14.10f} {c['ocxo_ppb']:>14.2f}"





# ---------------------------------------------------------------------
# SENSOR SCAN
# ---------------------------------------------------------------------

def sensor_scan_readout() -> Generator[str, None, None]:
    sensors = get_system_snapshot()["sensors"]

    health = sensors.get("health_state", "UNKNOWN")
    yield f"SENSOR SCAN: {health}"

    for bus_key, devices in sensors.items():
        # Skip roll-up key
        if bus_key == "health_state":
            continue

        # bus_key is "i2c-1", "i2c-2", etc.
        bus_num = bus_key.split("-")[1]

        for name, state in devices.items():
            yield f"{bus_num} {name.upper()}: {state}"


# ---------------------------------------------------------------------
# ENVIRONMENT
# ---------------------------------------------------------------------

def environment_status_readout() -> Generator[str, None, None]:
    e = get_system_snapshot()["environment"]

    yield f"ENVIRONMENT: {e['health_state']}"
    yield f"TEMPERATURE: {e['temperature_c']:.2f} C"
    yield f"HUMIDITY: {e['humidity_pct']:.2f} %"
    yield f"PRESSURE: {e['pressure_hpa']:.2f} HPA"
    yield f"ALTITUDE: {e['altitude_m']:.1f} M"


# ---------------------------------------------------------------------
# LASER
# ---------------------------------------------------------------------

def laser_status_readout() -> Generator[str, None, None]:
    l = get_system_snapshot()["laser"]

    yield f"LASER STATUS: {l['health_state']}"
    yield f"I2C ADDRESS: {l['i2c_address']}"
    yield f"SYSTEM ENABLED: {'YES' if l['sys_enabled'] else 'NO'}"
    yield f"ID1 ENABLED: {'YES' if l['id1_enabled'] else 'NO'}"
    yield f"ID1 CURRENT: {l['id1_current_ma']:.2f} MA"
    yield f"LASER EMITTING: {'YES' if l['laser_emitting'] else 'NO'}"
    yield f"PD VOLTAGE: {l['pd_voltage']:.4f} V"


# ---------------------------------------------------------------------
# NETWORK
# ---------------------------------------------------------------------

def network_status_readout() -> Generator[str, None, None]:
    n = get_system_snapshot()["network"]

    yield f"NETWORK STATUS: {n['network_status']}"
    yield f"SSID: {n['ssid']}"
    yield f"LOCAL IP: {n['local_ip']}"
    yield f"PING: {n['ping_ms']:.2f} MS"
    yield f"DOWNLOAD: {n['download_mbps']:.2f} MBPS"
    yield f"UPLOAD: {n['upload_mbps']:.2f} MBPS"


# ---------------------------------------------------------------------
# POWER STATUS
# ---------------------------------------------------------------------

def power_status_readout() -> Generator[str, None, None]:
    p = get_system_snapshot()["power"]

    yield f"POWER STATUS: {p['health_state']}"

    load_total = 0.0
    battery_power = None

    for bus_key, devices in p.items():
        if bus_key == "health_state":
            continue

        # bus_key is "i2c-1", "i2c-2", etc.
        bus_num = bus_key.split("-")[1]

        for _, r in devices.items():
            label = r.get("label", "UNKNOWN")
            v  = r["volts"]
            i  = r["amps"]
            pw = r["watts"]

            yield f"{label.upper():<14} {v:>6.3f} V {i:>7.1f} MA {pw:>6.3f} W"

            if label.lower() == "battery":
                battery_power = pw
            else:
                load_total += pw

    yield f"TOTAL LOAD POWER: {load_total:.3f} W"

    if battery_power and battery_power > 0:
        yield f"EFFICIENCY: {(load_total / battery_power) * 100.0:.1f} %"




# ---------------------------------------------------------------------
# BATTERY STATUS
# ---------------------------------------------------------------------

def battery_status_readout() -> Generator[str, None, None]:
    p = get_system_snapshot()["power"]

    yield f"BATTERY STATUS: {p['health_state']}"

    # Flatten all rails across all I2C buses
    rails = []
    for bus_name, bus in p.items():
        if bus_name == "health_state":
            continue
        if isinstance(bus, dict):
            rails.extend(bus.values())

    # Find the battery rail
    battery = next(
        r for r in rails
        if r.get("label", "").lower() == "battery"
    )

    yield f"VOLTAGE: {battery['volts']:.3f} V"
    yield f"CURRENT: {battery['amps']:.1f} MA"
    yield f"POWER:   {battery['watts']:.3f} W"

    if battery.get("ideal_voltage_v") is not None:
        yield f"IDEAL VOLTAGE: {battery['ideal_voltage_v']:.1f} V"


# ---------------------------------------------------------------------
# TEENSY
# ---------------------------------------------------------------------

def teensy_status_readout() -> Generator[str, None, None]:
    t = get_system_snapshot()["teensy"]

    yield f"TEENSY STATUS: {t['health_state']}"
    yield f"FW VERSION: {t['fw_version']}"
    yield f"CPU TEMP: {t['cpu_temp_c']:.2f} C"
    yield f"CPU USAGE: {t['cpu_usage_pct']:.4f} %"
    yield f"FREE HEAP: {t['free_heap_bytes']} BYTES"


# ---------------------------------------------------------------------
# PHOTODIODE STATUS (laser-owned ground truth)
# ---------------------------------------------------------------------

def photodiode_status_readout() -> Generator[str, None, None]:
    l = get_system_snapshot()["laser"]

    yield f"PHOTODIODE STATUS: {l['health_state']}"
    yield f"ANALOG VOLTAGE: {l['pd_voltage']:.5f} V"
    yield f"LIGHT PRESENT: {'YES' if l['laser_emitting'] else 'NO'}"


# ---------------------------------------------------------------------
# RASPBERRY PI
# ---------------------------------------------------------------------

def raspberry_pi_status_readout() -> Generator[str, None, None]:
    p = get_system_snapshot()["pi"]
    mem = p["memory"]
    disk = p["disk"]
    uv = p["undervoltage_flags"]

    yield f"RASPBERRY PI STATUS: {p['health_state']}"
    yield f"DEVICE: {p['device_name']}"
    yield f"CPU TEMP: {p['cpu_temp_c']:.1f} C"
    yield f"LOAD (1/5/15): {p['load_1m']:.2f} / {p['load_5m']:.2f} / {p['load_15m']:.2f}"
    yield f"UPTIME: {p['uptime_s'] / 3600:.2f} H"
    yield f"MEM USED: {mem['used_mb']:.0f} / {mem['total_mb']:.0f} MB ({mem['percent']:.1f}%)"
    yield f"DISK USED: {disk['used_gb']:.2f} / {disk['total_gb']:.2f} GB ({disk['percent']:.1f}%)"

    if uv["currently_undervolted"]:
        yield "UNDERVOLTAGE: ACTIVE"
    elif uv["previously_undervolted"]:
        yield "UNDERVOLTAGE: RECOVERED"
    else:
        yield "UNDERVOLTAGE: NONE"
