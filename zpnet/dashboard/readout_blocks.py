"""
ZPNet Dashboard Readout Blocks

Each readout is a generator that emits lines of text representing
a system status snapshot. These are used in the terminal UI.

Author: The Mule + GPT
"""

from collections.abc import Generator
from zpnet.dashboard.core import fetch_aggregate


# ---------------------------------------------------------------------
# GNSS
# ---------------------------------------------------------------------
def gnss_data_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("GNSS_DATA")
    yield f"GNSS DATA: {ag.get('health_state', 'DOWN')}"

    if not ag:
        yield "GNSS DATA UNAVAILABLE."
        return

    if "utc_datetime" in ag:
        yield f"UTC: {ag.get('utc_datetime')}"
    if "time_status" in ag:
        yield f"TIME STATUS: {ag.get('time_status')}"
    if "leap_seconds" in ag:
        yield f"LEAP SECONDS: {ag.get('leap_seconds')}"
    if "freq_mode" in ag:
        yield f"FREQ MODE: {ag.get('freq_mode')}"

    if "pps_accuracy_ns" in ag:
        yield f"PPS ACCURACY: {ag['pps_accuracy_ns']} NS"
    if "pps_timing_error_ns" in ag:
        yield f"PPS ERROR: {ag['pps_timing_error_ns']} NS"
    if "clock_drift_ppb" in ag:
        yield f"CLOCK DRIFT: {ag['clock_drift_ppb']} PPB"
    if "temperature_c" in ag:
        yield f"OSC TEMP: {ag['temperature_c']:.2f} C"

    if "latitude_deg" in ag and "longitude_deg" in ag:
        yield f"LAT/LON: {ag['latitude_deg']:.6f}, {ag['longitude_deg']:.6f}"

    for key in ["raw_zda", "raw_rmc", "raw_crw", "raw_crx", "raw_crz"]:
        if key in ag:
            yield f"{key.upper()}: {str(ag[key])[:38]}"


# ---------------------------------------------------------------------
# Laser
# ---------------------------------------------------------------------
def laser_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("LASER_STATUS")
    yield f"LASER STATUS: {ag.get('health_state', 'DOWN')}"

    if not ag or not ag.get("device_present"):
        yield "LASER CONTROLLER UNAVAILABLE."
        return

    yield f"I2C ADDRESS: {ag.get('i2c_address', 'UNKNOWN')}"
    yield f"SYSTEM ENABLED: {'YES' if ag.get('sys_enabled') else 'NO'}"
    yield f"ID1 ENABLED: {'YES' if ag.get('id1_enabled') else 'NO'}"
    yield f"MODE: {ag.get('id1_mode', 'UNKNOWN')}"
    yield f"CURRENT CODE: {ag.get('id1_current_code', 0)}"
    yield f"LASER ENABLED: {'YES' if ag.get('laser_enabled') else 'NO'}"


# ---------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------
def environment_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("ENVIRONMENT_STATUS")
    yield f"ENVIRONMENT: {ag.get('health_state', 'DOWN')}"

    if not ag or not ag.get("sensor_present"):
        yield "ENV SENSOR UNAVAILABLE."
        return

    yield f"TEMPERATURE: {ag.get('temperature_c', 0):.2f} C"
    yield f"HUMIDITY: {ag.get('humidity_pct', 0):.2f} %"
    yield f"PRESSURE: {ag.get('pressure_hpa', 0):.2f} HPA"
    yield f"ALTITUDE: {ag.get('altitude_m', 0):.1f} M"


# ---------------------------------------------------------------------
# Sensor Scan
# ---------------------------------------------------------------------
def sensor_scan_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("SENSOR_SCAN")
    yield f"SENSOR SCAN: {ag.get('health_state', 'DOWN')}"
    for k, v in ag.items():
        if isinstance(v, str) and k != "health_state":
            yield f"{k.upper()}: {v.upper()}"


# ---------------------------------------------------------------------
# Battery
# ---------------------------------------------------------------------
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
        yield f"TIME-TO-EMPTY: {h}H {m}M"

    yield f"REMAINING PERCENT: {ag.get('remaining_pct', 0):.1f}%"
    yield f"WH USED SINCE RECHARGE: {ag.get('wh_used_since_recharge', 0):.2f}"
    yield f"WH REMAINING EST: {ag.get('wh_remaining_estimate', 0):.2f}"


# ---------------------------------------------------------------------
# Power
# ---------------------------------------------------------------------
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
        eff = (load_total_w / battery_power_w) * 100.0
        yield f"EFFICIENCY: {eff:.1f} %"


# ---------------------------------------------------------------------
# Network
# ---------------------------------------------------------------------
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


# ---------------------------------------------------------------------
# Teensy
# ---------------------------------------------------------------------
def teensy_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("TEENSY_STATUS")
    yield f"TEENSY STATUS: {ag.get('health_state', 'DOWN')}"

    if not ag:
        yield "TEENSY DATA UNAVAILABLE."
        return

    yield f"FW: {ag.get('fw_version', 'UNKNOWN')}"
    yield f"CPU TEMP: {ag.get('cpu_temp_c', 0):.1f} C"
    yield f"VREF: {ag.get('vref_v', 0):.2f} V"
    yield f"FREE HEAP: {ag.get('free_heap_bytes', 0) / 1024:.1f} KB"


# ---------------------------------------------------------------------
# Raspberry Pi
# ---------------------------------------------------------------------
def raspberry_pi_status_readout() -> Generator[str, None, None]:
    ag = fetch_aggregate("RASPBERRY_PI_STATUS")
    yield f"RASPBERRY PI STATUS: {ag.get('health_state', 'DOWN')}"

    if not ag:
        yield "RASPBERRY PI DATA UNAVAILABLE."
        return

    yield f"DEVICE: {ag.get('device_name', 'UNKNOWN')}"
    yield f"CPU TEMP: {ag.get('cpu_temp_c', 0):.1f} C"
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
        yield "⚠️ UNDERVOLTAGE: RECOVERED"
    elif uv_now is False and uv_past is False:
        yield "UNDERVOLTAGE: NONE"
    else:
        yield "UNDERVOLTAGE: UNKNOWN"
