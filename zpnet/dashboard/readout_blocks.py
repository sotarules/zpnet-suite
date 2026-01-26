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

from collections.abc import Generator

from zpnet.processes.processes import send_command


# ---------------------------------------------------------------------
# Shared SYSTEM snapshot fetcher (AUTHORITATIVE)
# ---------------------------------------------------------------------

def get_system_snapshot() -> dict:
    return send_command(machine="PI", subsystem="SYSTEM", command="REPORT")["payload"]


# ---------------------------------------------------------------------
# GNSS REPORT (snapshot)
# ---------------------------------------------------------------------

def gnss_report_readout() -> Generator[str, None, None]:
    g = get_system_snapshot()["gnss"]

    yield f"GNSS REPORT: {g['health_state']}"

    if "date" in g:
        yield f"UTC DATE: {g['date']}"
    if "time" in g:
        yield f"UTC TIME: {g['time']}"

    if "latitude_deg" in g and "longitude_deg" in g:
        yield f"LATITUDE:  {g['latitude_deg']:.6f}°"
        yield f"LONGITUDE: {g['longitude_deg']:.6f}°"

    if "altitude_m" in g:
        yield f"ALTITUDE: {g['altitude_m']:.1f} M"

    if "discipline" in g:
        yield f"DISCIPLINE MODE: {g['discipline']}"


# ---------------------------------------------------------------------
# SENSOR SCAN
# ---------------------------------------------------------------------

def sensor_scan_readout() -> Generator[str, None, None]:
    sensors = get_system_snapshot()["sensors"]
    yield "SENSOR SCAN:"
    for name, state in sensors.items():
        yield f"{name.upper()}: {state}"


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
# POWER
# ---------------------------------------------------------------------

def power_status_readout() -> Generator[str, None, None]:
    p = get_system_snapshot()["power"]
    rails = p["rails"]

    yield f"POWER STATUS: {p['health_state']}"

    load_total = 0.0
    battery_power = None

    for r in rails:
        label = r["label"]
        v = r["voltage_v"]
        i = r["current_ma"]
        pw = r["power_w"]

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
    rails = p["rails"]

    yield f"BATTERY STATUS: {p['health_state']}"

    battery = next(r for r in rails if r["label"].lower() == "battery")

    yield f"VOLTAGE: {battery['voltage_v']:.3f} V"
    yield f"CURRENT: {battery['current_ma']:.1f} MA"
    yield f"POWER:   {battery['power_w']:.3f} W"

    if "ideal_voltage_v" in battery and battery["ideal_voltage_v"] is not None:
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
