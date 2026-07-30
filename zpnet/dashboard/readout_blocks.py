"""
ZPNet Dashboard Readout Blocks — TIMEBASE_V3 / SYSTEM 2026 Edition

The rotating pygame dashboard is an observer.  Both platform state and live
clock science are read from the unified MONITOR heartbeat.  MONITOR.clocks is
the always-on instrument surface; campaign state merely decorates it.  No
dashboard panel reads TIMEBASE or polls Teensy CLOCKS imperatively.
"""
from __future__ import annotations

import math
from collections.abc import Generator

from zpnet.processes.processes import create_pubsub_cache, send_command

VREF = 5.0
DAC_CODE_SCALE = 65536.0

_MONITOR_TOPIC = "MONITOR"
_LIVE_CACHE = create_pubsub_cache(_MONITOR_TOPIC)


def get_system_snapshot() -> dict:
    """Return the latest unified MONITOR snapshot without issuing a command."""
    return _LIVE_CACHE.get(_MONITOR_TOPIC) or {}


def _monitor_root() -> dict:
    """Return the MONITOR publication itself.

    PubSubTapCache already removes the transport envelope and returns the
    published payload.  MONITOR also legitimately contains a top-level
    ``payload`` block for Teensy Payload allocator/serialization metrics; that
    block is data, not another message envelope.  Unwrapping it hides sibling
    surfaces such as ``clocks``, ``features``, and ``gnss``.
    """
    return get_system_snapshot()


def get_pi_clocks_report() -> dict:
    """Return the live MONITOR.clocks surface without issuing a command."""
    root = _monitor_root()
    clocks = root.get("clocks")
    if not isinstance(clocks, dict):
        return {}
    report = dict(clocks)
    for key in (
        "campaign", "campaign_state", "campaign_present", "campaign_elapsed",
        "instrument_elapsed", "instrument_always_on", "gnss_time_utc",
        "system_time_utc", "published_at_utc",
    ):
        if report.get(key) is None and root.get(key) is not None:
            report[key] = root.get(key)
    return report


def _get_clocks_baseline() -> dict | None:
    baseline = get_pi_clocks_report().get("baseline")
    return baseline if isinstance(baseline, dict) and baseline.get("baseline_set") else None


def _dict(value) -> dict:
    return value if isinstance(value, dict) else {}


def _path_get(obj, path: str, default=None):
    cur = obj
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur.get(part)
    return default if cur is None else cur


def _payload_root(report: dict) -> dict:
    return _dict(report.get("payload")) or _dict(report)


def _fragment_root(report: dict) -> dict:
    root = _payload_root(report)
    return _dict(root.get("fragment")) or root


def _field(report: dict, *paths, default=None):
    root = _payload_root(report)
    fragment = _fragment_root(report)
    for source in (fragment, root):
        for path in paths:
            value = _path_get(source, path, None)
            if value is not None:
                return value
    return default


def _extra(report: dict, path: str, default=None):
    root = _payload_root(report)
    value = _path_get(_dict(root.get("extra_clocks")), path, None)
    if value is not None:
        return value
    gnss_raw = _dict(root.get("gnss_raw"))
    aliases = {
        "gnss_raw_ns": ("presentation.ns", "instrument.ns", "ns"),
        "gnss_raw_ref_ns": ("presentation.ref_ns", "instrument.ref_ns", "ref_ns"),
        "gnss_raw_tau": ("presentation.tau", "instrument.tau", "tau"),
        "gnss_raw_ppb": ("presentation.ppb", "instrument.ppb", "ppb"),
        "gnss_raw_drift_ppb": ("drift_ppb",),
        "gnss_raw_welford_n": ("welford.n",),
        "gnss_raw_welford_mean": ("welford.mean",),
        "gnss_raw_welford_stddev": ("welford.stddev",),
        "gnss_raw_welford_stderr": ("welford.stderr",),
        "gnss_raw_welford_min": ("welford.min",),
        "gnss_raw_welford_max": ("welford.max",),
    }
    for alias in aliases.get(path, (path,)):
        value = _path_get(gnss_raw, alias, None)
        if value is not None:
            return value
    return default


def _to_int(value):
    try:
        return None if value is None else int(value)
    except (TypeError, ValueError):
        return None


def _to_float(value):
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None


def _to_bool(value):
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "on"}:
        return True
    if text in {"false", "0", "no", "off"}:
        return False
    return None


def _num(value, decimals=3, signed=False, suffix="") -> str:
    number = _to_float(value)
    if number is None or not math.isfinite(number):
        return "---"
    sign = "+" if signed else ""
    return f"{number:{sign}.{decimals}f}{suffix}"


def _integer(value, comma=False, signed=False) -> str:
    number = _to_int(value)
    if number is None:
        return "---"
    if comma and signed:
        return f"{number:+,d}"
    if comma:
        return f"{number:,d}"
    return f"{number:+d}" if signed else str(number)


def _yes_no(value) -> str:
    flag = _to_bool(value)
    return "YES" if flag is True else "NO" if flag is False else "---"


def _clock_report() -> tuple[dict | None, str]:
    report = get_pi_clocks_report()
    if not report:
        return None, "CLOCKS: MONITOR UNAVAILABLE"

    state = str(report.get("campaign_state") or ("STARTED" if report.get("campaign_present") else "STOPPED")).upper()
    count = _to_int(_field(
        report, "instrument_count", "instrument_seconds",
        "stats.ocxo1.welford.n", "stats.ocxo1.n",
        "teensy_pps_vclock_count", "pps_count",
    )) or 0
    if state == "STARTED" or report.get("campaign_present"):
        campaign = report.get("campaign", "?")
        elapsed = report.get("campaign_elapsed", "00:00:00")
        header = f"CLOCKS: {campaign} {elapsed} n={count}"
    else:
        elapsed = report.get("instrument_elapsed", "00:00:00")
        header = f"CLOCKS: INSTRUMENT {elapsed} n={count}"
    return report, header


def _frequency(report: dict, lane: str) -> tuple[float | None, float | None]:
    if lane == "gnss":
        return 1.0, 0.0
    if lane == "gnss_raw":
        tau = _to_float(_extra(report, "gnss_raw_tau"))
        ppb = _to_float(_extra(report, "gnss_raw_ppb"))
        return tau, ppb
    tau = _to_float(_field(report, f"stats.{lane}.tau", f"{lane}_tau"))
    ppb = _to_float(_field(report, f"stats.{lane}.ppb", f"{lane}_ppb"))
    return tau, ppb


def _welford(report: dict, lane: str, field: str):
    if lane == "gnss_raw":
        return _extra(report, f"gnss_raw_welford_{field}")
    paths = [f"stats.{lane}.welford.{field}", f"stats.{lane}.{field}", f"{lane}_welford_{field}"]
    if lane.endswith("_dac"):
        clock = lane[:-4]
        paths.insert(0, f"stats.dac.{clock}.{field}")
    return _field(report, *paths)


def _prediction(report: dict, lane: str, field: str):
    # TIMEBASE_FRAGMENT_V5 folds the former prediction object into raw_cycles.
    # The previous completed interval is the prediction for the current row.
    raw_field = {
        "prediction_cycles": "previous_observed_cycles",
        "actual_cycles": "observed_cycles",
        "residual_cycles": "residual_cycles",
        "completed_interval_count": "completed_interval_count",
        "valid": "valid",
    }.get(field)
    if raw_field is not None:
        value = _field(report, f"raw_cycles.{lane}.{raw_field}")
        if value is not None:
            return value

    aliases = {
        "prediction_cycles": ("prediction_cycles", "static_prediction_cycles"),
        "actual_cycles": ("actual_cycles",),
        "residual_cycles": ("residual_cycles", "static_residual_cycles"),
        "completed_interval_count": ("completed_interval_count",),
        "valid": ("valid", "prediction_valid", "static_prediction_valid"),
    }.get(field, (field,))
    paths = []
    for alias in aliases:
        paths.extend((f"prediction.{lane}.{alias}", f"prediction.{lane}_{alias}", f"{lane}_{alias}"))
    return _field(report, *paths)


def _ocxo_residual(report: dict, lane: str):
    return _field(report,
                  f"{lane}.pps_residual.fast_residual_ns",
                  f"{lane}.science.fast_residual_ns",
                  f"{lane}_second_residual_ns")


def _dac_value(report: dict, lane: str):
    return _to_float(_field(report,
                            f"dac.{lane}_dac",
                            f"dac.{lane}.value",
                            f"dac.{lane}.dac",
                            f"stats.dac.{lane}.value",
                            f"stats.dac.{lane}.mean",
                            f"{lane}_dac"))


def _dac_voltage(code) -> float | None:
    value = _to_float(code)
    return None if value is None else value * VREF / DAC_CODE_SCALE


def _servo_mode(report: dict) -> str:
    for path in ("dac.servo_mode", "dac.calibrate_ocxo", "servo.mode", "servo_mode", "calibrate_ocxo"):
        value = _field(report, path)
        if value is not None and str(value).strip():
            text = str(value).strip().upper()
            return "IDLE" if text in {"OFF", "NONE", "IDLE"} else text
    return "IDLE"


def _dither_label(code) -> str:
    value = _to_float(code)
    if value is None:
        return "---"
    low = int(value)
    high = min(65535, low + 1)
    high_ms = max(0, min(1000, int(round((value - low) * 1000.0))))
    return f"{low}:{1000-high_ms:03d}/{high}:{high_ms:03d}"


def gnss_report_readout() -> Generator[str, None, None]:
    g = _dict(get_system_snapshot().get("gnss"))
    discipline = _dict(g.get("discipline"))
    clock = _dict(g.get("clock"))
    integrity = _dict(g.get("integrity"))
    survey = _dict(g.get("survey_mode"))
    pps = _dict(g.get("pps"))

    lock = g.get("lock_quality", "UNKNOWN")
    yield f"GNSS REPORT: {lock}"
    yield f"MODE: {integrity.get('pos_mode') or survey.get('receiver_mode') or g.get('pos_mode') or '?'}  DISC: {discipline.get('freq_mode_name') or g.get('freq_mode_name') or '?'}"
    yield f"TIME: {clock.get('time_status_name') or g.get('time_status') or '?'}  PPS: {_yes_no(g.get('pps_valid'))}  TRAIM: {integrity.get('traim') or g.get('traim') or '?'}"
    yield f"SATS: {_integer(g.get('satellites'))}  HDOP: {_num(g.get('hdop'), 2)}  PPS ERR: {_num(discipline.get('pps_timing_error_ns') or g.get('pps_timing_error_ns'), 1, True, ' ns')}"
    if g.get("latitude_deg") is not None and g.get("longitude_deg") is not None:
        yield f"LAT: {_num(g.get('latitude_deg'), 6)}  LON: {_num(g.get('longitude_deg'), 6)}"
    yield f"ALT MSL: {_num(g.get('altitude_m'), 1, False, ' m')}  ELLIP: {_num(g.get('ellipsoid_height_m'), 1, False, ' m')}"
    yield f"GEOID: {_num(g.get('geoid_sep_m'), 1, False, ' m')}  TEMP: {_num(clock.get('temperature_c') or g.get('temperature_c'), 1, False, ' C')}"


def clocks_tau_readout() -> Generator[str, None, None]:
    report, header = _clock_report()
    yield header
    if report is None:
        return
    yield f"{'CLK':<7}{'TAU':>17}{'PPB':>12}{'N':>8}"
    for label, lane in (("GNSS", "gnss"), ("VCLOCK", "vclock"), ("OCXO1", "ocxo1"),
                        ("OCXO2", "ocxo2"), ("GN_RAW", "gnss_raw"), ("DWT", "dwt")):
        tau, ppb = _frequency(report, lane)
        tau_text = "---" if tau is None else f"{tau:.12f}"
        ppb_text = "---" if ppb is None else f"{ppb:+.3f}"
        sample_n = _to_int(_welford(report, lane, "n"))
        yield f"{label:<7}{tau_text:>17}{ppb_text:>12}{_integer(sample_n):>8}"


def clocks_prediction_readout() -> Generator[str, None, None]:
    report, header = _clock_report()
    yield header
    if report is None:
        return
    yield "STATIC PREDICTION (prior interval -> current)"
    yield f"{'LANE':<6}{'PRED':>16}{'ACTUAL':>16}"
    for label, lane in (("PPS", "pps"), ("VCLK", "vclock"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
        valid = _to_bool(_prediction(report, lane, "valid"))
        pred = _to_int(_prediction(report, lane, "prediction_cycles"))
        actual = _to_int(_prediction(report, lane, "actual_cycles"))
        if valid is not True:
            pred = actual = None
        yield f"{label:<6}{_integer(pred, True):>16}{_integer(actual, True):>16}"


def clocks_comparison_readout() -> Generator[str, None, None]:
    report, header = _clock_report()
    yield header
    if report is None:
        return
    baseline = _get_clocks_baseline()
    if not baseline:
        yield "NO BASELINE SET"
        yield "USE: .pc clocks set_baseline id=<N>"
        return
    yield f"BASELINE: {baseline.get('baseline_campaign', '?')} (#{baseline.get('baseline_id', '?')})"
    yield f"{'CLK':<7}{'BASE':>10}{'NOW':>10}{'DELTA':>10}"
    baseline_ppb = _dict(baseline.get("baseline_ppb"))
    for label, lane in (("GNSS", "gnss"), ("VCLK", "vclock"), ("OCXO1", "ocxo1"),
                        ("OCXO2", "ocxo2"), ("GN_RAW", "gnss_raw"), ("DWT", "dwt")):
        _, now = _frequency(report, lane)
        base = _to_float(baseline_ppb.get(lane))
        delta = None if base is None or now is None else now - base
        yield f"{label:<7}{_num(base):>10}{_num(now):>10}{_num(delta, 3, True):>10}"


def clocks_servo_readout() -> Generator[str, None, None]:
    report, header = _clock_report()
    yield header
    if report is None:
        return
    yield f"SERVO: {_servo_mode(report)}"
    yield f"{'OCXO':<6}{'DAC':>10}{'VOUT':>9}{'RES NS':>9}{'PPB':>9}"
    for label, lane in (("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")):
        code = _dac_value(report, lane)
        volts = _dac_voltage(code)
        _, ppb = _frequency(report, lane)
        residual = _to_int(_ocxo_residual(report, lane))
        yield f"{label:<6}{_num(code):>10}{_num(volts, 6):>9}{_integer(residual, False, True):>9}{_num(ppb, 3, True):>9}"
        yield f"  DITHER: {_dither_label(code)}"


def clocks_dac_welford_readout() -> Generator[str, None, None]:
    report, header = _clock_report()
    yield header
    if report is None:
        return
    yield "DAC WELFORD (always-on cumulative)"
    yield f"{'OCXO':<6}{'MEAN':>10}{'SD':>8}{'SE':>8}{'N':>7}"
    for label, lane in (("OCXO1", "ocxo1_dac"), ("OCXO2", "ocxo2_dac")):
        yield f"{label:<6}{_num(_welford(report, lane, 'mean')):>10}{_num(_welford(report, lane, 'stddev')):>8}{_num(_welford(report, lane, 'stderr')):>8}{_integer(_welford(report, lane, 'n')):>7}"
        yield f"  MIN {_num(_welford(report, lane, 'min'))}  MAX {_num(_welford(report, lane, 'max'))}"


_FEATURE_CELLS = (
    ("T SYS", "TEENSY.SYSTEM.FEATURE_STATUS"),
    ("OBS EDGE", "TEENSY.INTERRUPT.OBSERVED_EDGE_AUTHORITY"),
    ("PPS/V", "TEENSY.INTERRUPT.PPS_VCLOCK_AUTHORITY"),
    ("QTIMER", "TEENSY.INTERRUPT.QTIMER_COUNTER_CUSTODY"),
    ("CTR32", "TEENSY.INTERRUPT.COUNTER32_LINEAGE"),
    ("DWT CAL", "TEENSY.CLOCKS.DWT_CALIBRATION"),
    ("PRED", "TEENSY.CLOCKS.STATIC_PREDICTION"),
    ("SMARTZERO", "TEENSY.CLOCKS.SMARTZERO"),
    ("EPOCH", "TEENSY.CLOCKS.ALPHA_EPOCH"),
    ("ORIGIN", "TEENSY.CLOCKS.OCXO_PUBLIC_ORIGIN"),
    ("SCIENCE", "TEENSY.CLOCKS.SCIENCE_RESIDUALS"),
    ("TIMEBASE", "TEENSY.CLOCKS.TIMEBASE_PUBLICATION"),
    ("PI NET", "PI.SYSTEM.NETWORK"),
    ("PI GNSS", "PI.GNSS.REPORT"),
    ("PI POWER", "PI.SYSTEM.POWER"),
    ("PI HOST", "PI.SYSTEM.HOST"),
)


def feature_status_readout() -> Generator[str, None, None]:
    features = _dict(get_system_snapshot().get("features"))
    yield "FEATURE READINESS"
    cells = []
    for label, path in _FEATURE_CELLS:
        value = _path_get(features, path, "---")
        cells.append(f"{label}:{str(value).upper()}")
    for index in range(0, len(cells), 2):
        left = cells[index]
        right = cells[index + 1] if index + 1 < len(cells) else ""
        yield f"{left:<21}{right}"


def sensor_scan_readout() -> Generator[str, None, None]:
    sensors = _dict(get_system_snapshot().get("sensors"))
    yield f"SENSOR SCAN: {sensors.get('health_state', 'UNKNOWN')}"
    found = False
    for bus_name, devices in sensors.items():
        if not str(bus_name).startswith("i2c-") or not isinstance(devices, dict):
            continue
        for address, state in devices.items():
            found = True
            yield f"{str(bus_name).upper():<7}{str(address).upper():<7}{str(state).upper()}"
    if not found:
        yield "NO SENSOR INVENTORY"


def environment_status_readout() -> Generator[str, None, None]:
    env = _dict(get_system_snapshot().get("environment"))
    yield f"ENVIRONMENT: {env.get('health_state', 'UNKNOWN')}"
    yield f"TEMP: {_num(env.get('temperature_c'), 2, False, ' C')}"
    yield f"HUMIDITY: {_num(env.get('humidity_pct'), 2, False, ' %')}"
    yield f"PRESSURE: {_num(env.get('pressure_hpa'), 2, False, ' HPA')}"
    yield f"BARO ALT: {_num(env.get('altitude_m'), 1, False, ' M')}"
    yield f"READ OK: {_yes_no(env.get('read_ok'))}  STALE: {_yes_no(env.get('stale'))}"
    yield f"FAILURES: {_integer(env.get('read_fail_count'))}  RECOVERIES: {_integer(env.get('recovery_count'))}"


def network_status_readout() -> Generator[str, None, None]:
    net = _dict(get_system_snapshot().get("network"))
    yield f"NETWORK STATUS: {net.get('network_status', 'UNKNOWN')}"
    yield f"SSID: {net.get('ssid') or '---'}"
    yield f"LOCAL IP: {net.get('local_ip') or '---'}"
    yield f"PING: {_num(net.get('ping_ms'), 2, False, ' MS')}"
    yield f"DOWNLOAD: {_num(net.get('download_mbps'), 2, False, ' MBPS')}"
    yield f"UPLOAD: {_num(net.get('upload_mbps'), 2, False, ' MBPS')}"


def _power_rails(snapshot: dict) -> list[dict]:
    rails = []
    for bus_name, devices in _dict(snapshot.get("power")).items():
        if str(bus_name).startswith("i2c-") and isinstance(devices, dict):
            for value in devices.values():
                if isinstance(value, dict):
                    rails.append(value)
    return rails


def power_status_readout() -> Generator[str, None, None]:
    snapshot = get_system_snapshot()
    power = _dict(snapshot.get("power"))
    yield f"POWER STATUS: {power.get('health_state', 'UNKNOWN')}"
    load = 0.0
    for rail in _power_rails(snapshot):
        label = str(rail.get("label", "UNKNOWN"))[:13].upper()
        volts = _to_float(rail.get("volts"))
        amps = _to_float(rail.get("amps"))
        watts = _to_float(rail.get("watts"))
        stale = " *" if rail.get("stale") else ""
        current = "---" if amps is None else f"{int(round(amps))}mA"
        yield f"{label:<13}{_num(volts,3):>7}V {current:>8} {_num(watts,3):>7}W{stale}"
        if label != "BATTERY" and watts is not None:
            load += watts
    yield f"TOTAL LOAD: {load:.3f} W   *=STALE"


def battery_status_readout() -> Generator[str, None, None]:
    snapshot = get_system_snapshot()
    battery = _dict(snapshot.get("battery"))
    rail = next((r for r in _power_rails(snapshot) if str(r.get("label", "")).lower() == "battery"), {})
    yield f"BATTERY STATUS: {battery.get('health_state', 'UNKNOWN')}"
    yield f"REMAINING: {_num(battery.get('remaining_pct'), 1, False, ' %')}"
    tte = _to_float(battery.get("tte_minutes"))
    yield f"TIME TO EMPTY: {'---' if tte is None or not math.isfinite(tte) else f'{tte:.1f} MIN'}"
    yield f"VOLTAGE: {_num(rail.get('volts'), 3, False, ' V')}"
    yield f"CURRENT: {_num(rail.get('amps'), 3, False, ' A')}"
    yield f"POWER: {_num(rail.get('watts'), 3, False, ' W')}"
    yield f"USED: {_num(battery.get('wh_used_since_recharge'), 2, False, ' WH')}  LEFT: {_num(battery.get('wh_remaining_estimate'), 2, False, ' WH')}"
    yield f"SAMPLES: {_integer(battery.get('samples_used'))}"


def teensy_status_readout() -> Generator[str, None, None]:
    teensy = _dict(get_system_snapshot().get("teensy"))
    usage_milli = _to_float(teensy.get("cpu_usage_pct_milli"))
    idle_milli = _to_float(teensy.get("cpu_idle_spin_pct_milli"))
    yield f"TEENSY STATUS: {teensy.get('health_state', 'UNKNOWN')}"
    yield f"FW VERSION: {teensy.get('fw_version', '---')}"
    yield f"CPU: {_integer(teensy.get('cpu_freq_mhz'))} MHZ"
    yield f"WORK: {'---' if usage_milli is None else f'{usage_milli/1000.0:.3f}%'}  IDLE: {'---' if idle_milli is None else f'{idle_milli/1000.0:.3f}%'}"
    yield f"WALL CYCLES: {_integer(teensy.get('cpu_wall_cycles'), True)}"
    yield f"CRASH REPORT: {'PRESENT' if teensy.get('crash_report_present') else 'NONE'}"
    yield f"FEATURE COURT: {'OK' if teensy.get('feature_status_foreground_court_ok') else 'ANOMALY'}"


def raspberry_pi_status_readout() -> Generator[str, None, None]:
    pi = _dict(get_system_snapshot().get("pi"))
    memory = _dict(pi.get("memory"))
    disk = _dict(pi.get("disk"))
    uv = _dict(pi.get("undervoltage_flags"))
    yield f"RASPBERRY PI STATUS: {pi.get('health_state', 'UNKNOWN')}"
    yield f"DEVICE: {pi.get('device_name', '---')}"
    yield f"CPU TEMP: {_num(pi.get('cpu_temp_c'), 1, False, ' C')}"
    yield f"LOAD 1/5/15: {_num(pi.get('load_1m'),2)} / {_num(pi.get('load_5m'),2)} / {_num(pi.get('load_15m'),2)}"
    uptime = _to_float(pi.get("uptime_s"))
    yield f"UPTIME: {'---' if uptime is None else f'{uptime/3600.0:.2f} H'}"
    yield f"MEM: {_num(memory.get('used_mb'),0)} / {_num(memory.get('total_mb'),0)} MB ({_num(memory.get('percent'),1)}%)"
    yield f"DISK: {_num(disk.get('used_gb'),2)} / {_num(disk.get('total_gb'),2)} GB ({_num(disk.get('percent'),1)}%)"
    if uv.get("currently_undervolted"):
        yield "UNDERVOLTAGE: ACTIVE"
    elif uv.get("previously_undervolted"):
        yield "UNDERVOLTAGE: RECOVERED"
    else:
        yield "UNDERVOLTAGE: NONE"


def _process_health(proc: dict) -> str:
    checks = ("rpc_counter_order_ok", "invariant_received_ok", "invariant_routed_ok",
              "invariant_handler_ok", "invariant_response_ok")
    return "NOMINAL" if all(proc.get(key) is True for key in checks) else "ANOMALY"


def _transport_health(tx: dict) -> str:
    failures = sum(_to_int(tx.get(key)) or 0 for key in (
        "tx_alloc_fail", "tx_budget_fail", "tx_queue_full", "tx_rr_drop_count",
        "rx_bad_stx", "rx_bad_etx", "rx_len_overflow", "rx_guard_failure_count"))
    return "NOMINAL" if failures == 0 and tx.get("rx_buffer_alignment_ok") is not False else "ANOMALY"


def _payload_health(payload: dict) -> str:
    failures = sum(_to_int(payload.get(key)) or 0 for key in (
        "payload_entry_alloc_fail", "payload_entry_overflow", "payload_arena_alloc_fail",
        "payload_serialize_overflow", "payload_to_json_fail", "payload_parse_error"))
    expected_alive = (_to_int(payload.get("payload_instances_constructed")) or 0) - (_to_int(payload.get("payload_instances_destroyed")) or 0)
    alive = _to_int(payload.get("payload_alive_now")) or 0
    return "NOMINAL" if failures == 0 and expected_alive == alive else "ANOMALY"


def teensy_metrics_readout() -> Generator[str, None, None]:
    snapshot = get_system_snapshot()
    proc = _dict(snapshot.get("process"))
    tx = _dict(snapshot.get("transport"))
    payload = _dict(snapshot.get("payload"))
    p_health = _process_health(proc)
    t_health = _transport_health(tx)
    a_health = _payload_health(payload)
    overall = "NOMINAL" if p_health == t_health == a_health == "NOMINAL" else "ANOMALY"

    yield f"TEENSY PIPELINE: {overall}"
    yield f"RPC {p_health}: RX {_integer(proc.get('rpc_received'))} ROUTED {_integer(proc.get('rpc_routed'))} SENT {_integer(proc.get('rpc_response_sent'))}"
    yield f"RPC INFLIGHT: {_integer(proc.get('rpc_handler_inflight'))}  PENDING: {_integer(proc.get('rpc_response_pending'))}  PS: {_integer(proc.get('ps_dispatched'))}"
    rpc_errors = sum(_to_int(proc.get(key)) or 0 for key in ("rpc_err_missing_fields", "rpc_err_unknown_subsys", "rpc_err_unknown_command"))
    yield f"RPC ERRORS: {rpc_errors}"
    yield f"TX {t_health}: JOBS {_integer(tx.get('tx_jobs_sent'))}/{_integer(tx.get('tx_jobs_enqueued'))}  QUEUE {_integer(tx.get('tx_job_count'))}"
    yield f"RX: FRAMES {_integer(tx.get('rx_frames_dispatched'))}/{_integer(tx.get('rx_frames_complete'))}  OVERLAP {_integer(tx.get('rx_overlap'))}"
    framing = sum(_to_int(tx.get(key)) or 0 for key in ("rx_bad_stx", "rx_bad_etx", "rx_len_overflow"))
    yield f"RX ERRORS: {framing}  GUARD: {_integer(tx.get('rx_guard_failure_count'))}  DMAMEM: {_yes_no(tx.get('rx_buffer_in_dmamem'))}"
    yield f"PAYLOAD {a_health}: ALIVE {_integer(payload.get('payload_alive_now'))}  HWM {_integer(payload.get('payload_alive_high_water'))}"
    yield f"HEAP: {_integer(payload.get('payload_heap_bytes_alive'), True)} / HWM {_integer(payload.get('payload_heap_bytes_high_water'), True)}"
    payload_faults = sum(_to_int(payload.get(key)) or 0 for key in ("payload_entry_alloc_fail", "payload_entry_overflow", "payload_arena_alloc_fail", "payload_serialize_overflow", "payload_to_json_fail", "payload_parse_error"))
    yield f"PAYLOAD FAULTS: {payload_faults}"
