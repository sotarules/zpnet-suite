"""
ZPNet Dashboard Readout Blocks — SYSTEM Snapshot Edition

All readouts are derived directly from the authoritative SYSTEM
snapshot delivered via WebSocket.

Invariants:
  • No database access
  • No aggregates
  • No events
  • No inference
  • Always fresh
  • Readouts NEVER throw

Author: The Mule + GPT
"""

from collections.abc import Generator

from zpnet.processes.processes import send_command

# ---------------------------------------------------------------------
# Shared SYSTEM snapshot fetcher (AUTHORITATIVE)
# ---------------------------------------------------------------------

def get_system_snapshot() -> dict:
    """
    Retrieve the authoritative SYSTEM snapshot.

    Semantics:
      • SYSTEM is a logical authority, not a transport detail
      • Under the hood, this resolves to the systemd-managed
        ZPNet SYSTEM service via PROCESS.COMMAND
      • Best-effort, snapshot-only
      • NEVER throws
      • NEVER blocks indefinitely
      • NEVER returns partial protocol state

    Returns:
        dict: SYSTEM payload on success, or {} on failure.
    """

    try:
        resp = send_command(
            "PROCESS.COMMAND",
            {
                "type": "SYSTEM",
                "proc_cmd": "REPORT",
            }
        )
    except Exception:
        # Transport failure, timeout, socket absence, etc.
        return {}

    # Defensive validation of response shape
    if not isinstance(resp, dict):
        return {}

    if not resp.get("success"):
        return {}

    payload = resp.get("payload")

    if not isinstance(payload, dict):
        return {}

    return payload


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _safe(d: dict, *keys, default=None):
    for k in keys:
        if not isinstance(d, dict):
            return default
        d = d.get(k)
    return d if d is not None else default

# ---------------------------------------------------------------------
# GNSS REPORT (authoritative, snapshot)
# ---------------------------------------------------------------------

def gnss_report_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    gnss = sys.get("gnss", {})

    yield "GNSS REPORT: NOMINAL" if gnss else "GNSS REPORT: DOWN"

    if not gnss:
        yield "NO GNSS DATA"
        return

    # GNSS time/discipline may not always be present
    # Report what exists, omit what does not
    time_str = gnss.get("time")
    date_str = gnss.get("date")

    if date_str:
        yield f"UTC DATE: {date_str}"
    if time_str:
        yield f"UTC TIME: {time_str}"

    lat = gnss.get("latitude_deg")
    lon = gnss.get("longitude_deg")

    if lat is not None and lon is not None:
        yield f"LATITUDE:  {lat:.6f}°"
        yield f"LONGITUDE: {lon:.6f}°"

    alt = gnss.get("altitude_m")
    if alt is not None:
        yield f"ALTITUDE: {alt:.1f} M"

    discipline = gnss.get("discipline")
    if discipline:
        yield f"DISCIPLINE MODE: {discipline}"

# ---------------------------------------------------------------------
# SENSOR SCAN (authoritative presence snapshot)
# ---------------------------------------------------------------------

def sensor_scan_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    sensors = sys.get("sensors", {})

    yield "SENSOR SCAN:"

    if not sensors:
        yield "NO SENSOR DATA"
        return

    for name, state in sensors.items():
        yield f"{name.upper()}: {state}"


# ---------------------------------------------------------------------
# ENVIRONMENT
# ---------------------------------------------------------------------

def environment_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    env = sys.get("environment", {})

    yield f"ENVIRONMENT: {env.get('health_state', 'DOWN')}"

    if not env:
        yield "ENVIRONMENT DATA UNAVAILABLE."
        return

    yield f"TEMPERATURE: {env.get('temperature_c', 0):.2f} C"
    yield f"HUMIDITY: {env.get('humidity_pct', 0):.2f} %"
    yield f"PRESSURE: {env.get('pressure_hpa', 0):.2f} HPA"
    yield f"ALTITUDE: {env.get('altitude_m', 0):.1f} M"


# ---------------------------------------------------------------------
# LASER
# ---------------------------------------------------------------------

def laser_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    las = sys.get("laser", {})

    yield f"LASER STATUS: {las.get('health_state', 'DOWN')}"

    if not las or not las.get("device_present"):
        yield "LASER CONTROLLER UNAVAILABLE."
        return

    yield f"I2C ADDRESS: {las.get('i2c_address', 'UNKNOWN')}"
    yield f"SYSTEM ENABLED: {'YES' if las.get('sys_enabled') else 'NO'}"
    yield f"ID1 ENABLED: {'YES' if las.get('id1_enabled') else 'NO'}"
    yield f"ID1 CURRENT: {las.get('id1_current_ma', 0):.2f} MA"
    yield f"LASER EMITTING: {'YES' if las.get('laser_emitting') else 'NO'}"
    yield f"PD VOLTAGE: {las.get('pd_voltage', 0):.4f} V"


# ---------------------------------------------------------------------
# NETWORK
# ---------------------------------------------------------------------

def network_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    net = sys.get("network", {})

    yield f"NETWORK STATUS: {net.get('network_status', 'DOWN')}"

    if not net:
        yield "NETWORK DATA UNAVAILABLE."
        return

    yield f"SSID: {net.get('ssid', 'UNKNOWN')}"
    yield f"LOCAL IP: {net.get('local_ip', '0.0.0.0')}"
    yield f"PING: {net.get('ping_ms', 0):.2f} MS"
    yield f"DOWNLOAD: {net.get('download_mbps', 0):.2f} MBPS"
    yield f"UPLOAD: {net.get('upload_mbps', 0):.2f} MBPS"


# ---------------------------------------------------------------------
# POWER
# ---------------------------------------------------------------------

def power_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    pwr = sys.get("power", {})

    yield f"POWER STATUS: {pwr.get('health_state', 'DOWN')}"

    rails = pwr.get("rails")
    if not rails:
        yield "POWER DATA UNAVAILABLE."
        return

    load_total = 0.0
    battery_power = None

    for r in rails:
        label = r.get("label", "UNKNOWN")
        v = r.get("voltage_v", 0.0)
        i = r.get("current_ma", 0.0)
        p = r.get("power_w", 0.0)

        yield f"{label.upper():<14} {v:>6.3f} V {i:>7.1f} MA {p:>6.3f} W"

        if label.lower() == "battery":
            battery_power = p
        else:
            load_total += p

    yield f"TOTAL LOAD POWER: {load_total:.3f} W"

    if battery_power and battery_power > 0:
        eff = (load_total / battery_power) * 100.0
        yield f"EFFICIENCY: {eff:.1f} %"

# ---------------------------------------------------------------------
# BATTERY STATUS (instantaneous, snapshot-only)
# ---------------------------------------------------------------------

def battery_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    pwr = sys.get("power", {})

    yield f"BATTERY STATUS: {pwr.get('health_state', 'DOWN')}"

    rails = pwr.get("rails")
    if not rails:
        yield "BATTERY DATA UNAVAILABLE."
        return

    battery = next(
        (r for r in rails if r.get("label", "").lower() == "battery"),
        None
    )

    if not battery:
        yield "BATTERY RAIL NOT FOUND."
        return

    v = battery.get("voltage_v", 0.0)
    i = battery.get("current_ma", 0.0)
    p = battery.get("power_w", 0.0)

    yield f"VOLTAGE: {v:.3f} V"
    yield f"CURRENT: {i:.1f} MA"
    yield f"POWER:   {p:.3f} W"

    ideal = battery.get("ideal_voltage_v")
    if ideal:
        yield f"IDEAL VOLTAGE: {ideal:.1f} V"


# ---------------------------------------------------------------------
# SENSORS
# ---------------------------------------------------------------------

def sensor_scan_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    sensors = sys.get("sensors", {})

    yield "SENSOR SCAN:"

    if not sensors:
        yield "NO SENSOR DATA."
        return

    for name, state in sensors.items():
        yield f"{name.upper()}: {state}"


# ---------------------------------------------------------------------
# TEENSY
# ---------------------------------------------------------------------

def teensy_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    tee = sys.get("teensy", {})

    yield f"TEENSY STATUS: {tee.get('health_state', 'NOMINAL')}"

    if not tee:
        yield "TEENSY DATA UNAVAILABLE."
        return

    yield f"FW VERSION: {tee.get('fw_version', 'UNKNOWN')}"
    yield f"CPU TEMP: {tee.get('cpu_temp_c', 0):.2f} C"
    yield f"CPU USAGE: {tee.get('cpu_usage_pct', 0):.4f} %"
    yield f"FREE HEAP: {tee.get('free_heap_bytes', 0)} BYTES"


# ---------------------------------------------------------------------
# PHOTODIODE STATUS (physical ground truth)
# ---------------------------------------------------------------------

def photodiode_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    las = sys.get("laser", {})

    # Health is inherited from laser observation path
    yield f"PHOTODIODE STATUS: {las.get('health_state', 'DOWN')}"

    if not las:
        yield "NO PHOTODIODE DATA"
        return

    pd_v = las.get("pd_voltage")
    emitting = las.get("laser_emitting")

    if pd_v is not None:
        yield f"ANALOG VOLTAGE: {pd_v:.5f} V"
    else:
        yield "ANALOG VOLTAGE: UNKNOWN"

    if emitting is True:
        yield "LIGHT PRESENT: YES"
    elif emitting is False:
        yield "LIGHT PRESENT: NO"
    else:
        yield "LIGHT PRESENT: UNKNOWN"


# ---------------------------------------------------------------------
# RASPBERRY PI
# ---------------------------------------------------------------------

def raspberry_pi_status_readout() -> Generator[str, None, None]:
    sys = get_system_snapshot()
    pi = sys.get("pi", {})

    yield f"RASPBERRY PI STATUS: {pi.get('health_state', 'DOWN')}"

    if not pi:
        yield "PI DATA UNAVAILABLE."
        return

    yield f"DEVICE: {pi.get('device_name', 'UNKNOWN')}"
    yield f"CPU TEMP: {pi.get('cpu_temp_c', 0):.1f} C"
    yield (
        f"LOAD (1/5/15): "
        f"{pi.get('load_1m', 0):.2f} / "
        f"{pi.get('load_5m', 0):.2f} / "
        f"{pi.get('load_15m', 0):.2f}"
    )
    yield f"UPTIME: {pi.get('uptime_s', 0) / 3600:.2f} H"

    mem = pi.get("memory", {})
    yield (
        f"MEM USED: {mem.get('used_mb', 0):.0f} / "
        f"{mem.get('total_mb', 0):.0f} MB "
        f"({mem.get('percent', 0):.1f}%)"
    )

    disk = pi.get("disk", {})
    yield (
        f"DISK USED: {disk.get('used_gb', 0):.2f} / "
        f"{disk.get('total_gb', 0):.2f} GB "
        f"({disk.get('percent', 0):.1f}%)"
    )

    uv = pi.get("undervoltage_flags", {})
    if uv.get("currently_undervolted"):
        yield "UNDERVOLTAGE: ACTIVE"
    elif uv.get("previously_undervolted"):
        yield "UNDERVOLTAGE: RECOVERED"
    else:
        yield "UNDERVOLTAGE: NONE"
