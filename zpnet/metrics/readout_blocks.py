"""
ZPNet Metrics Readout Blocks — Dense Clocks Panel (v12 Current TIMEBASE Model)

Designed for full-screen terminal display over SSH at 1080p.

Current data model:

  • Pi CLOCKS REPORT → campaign payload with report dict
  • report contains cumulative campaign clock blocks:
      gnss, gnss_raw, dwt, ocxo1, ocxo2
  • report['fragment'] contains the latest TIMEBASE_FRAGMENT fields from Teensy
  • fragment carries the operational / per-second / diagnostic detail:
      DWT prediction, OCXO edge timing, servo state, interrupt diag, etc.

Display rules:

  • TAU = cumulative nanoseconds of a given clock / cumulative GNSS nanoseconds
  • PPB = cumulative campaign PPB, not instantaneous and not statistical
  • MEAN / SD / N = Welford / prediction-stat columns only
  • EXTRA = operational facts from the latest fragment
"""

from zpnet.processes.processes import send_command


# ---------------------------------------------------------------------
# Data fetchers
# ---------------------------------------------------------------------

def _get_system_snapshot() -> dict:
    return send_command(machine="PI", subsystem="SYSTEM", command="REPORT")["payload"]


def _get_teensy_clocks_report() -> dict:
    return send_command(machine="TEENSY", subsystem="CLOCKS", command="REPORT")["payload"]


def _get_pi_clocks_report() -> dict:
    return send_command(machine="PI", subsystem="CLOCKS", command="REPORT")["payload"]


def _get_clocks_baseline() -> dict | None:
    try:
        resp = send_command(machine="PI", subsystem="CLOCKS", command="BASELINE_INFO")
        if resp.get("success"):
            p = resp.get("payload", {})
            if p.get("baseline_set"):
                return p
    except Exception:
        pass
    return None


# ---------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------

def _fmt(value, fmt_str, width, fallback="---"):
    if value is None:
        return f"{fallback:>{width}}"
    return f"{value:{fmt_str}}"


def _sign_int(value, width):
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>+{width}d}"


def _comma_int(value, width):
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width},d}"


def _safe_lines(fn):
    try:
        return fn()
    except Exception as e:
        return [f"ERROR: {e}"]


def _bool_str(value):
    if value is None:
        return "---"
    return "YES" if value else "NO"


# ---------------------------------------------------------------------
# Fragment-aware field access
# ---------------------------------------------------------------------

def _frag(r: dict, key: str, default=None):
    frag = r.get("fragment")
    if isinstance(frag, dict):
        val = frag.get(key)
        if val is not None:
            return val
    val = r.get(key)
    return val if val is not None else default


# ---------------------------------------------------------------------
# Status header
# ---------------------------------------------------------------------

def status_header() -> str:
    try:
        s = _get_system_snapshot()

        net = s.get("network", {}).get("network_status", "?")
        pi_health = s.get("pi", {}).get("health_state", "?")
        teensy_health = s.get("teensy", {}).get("health_state", "?")
        gnss_lock = s.get("gnss", {}).get("lock_quality", "?")

        bat_v = "?"
        power = s.get("power", {})
        for bus_key, devices in power.items():
            if not bus_key.startswith("i2c-"):
                continue
            if isinstance(devices, dict):
                for rail in devices.values():
                    if isinstance(rail, dict) and rail.get("label", "").lower() == "battery":
                        bat_v = f"{rail['volts']:.2f}V"

        battery = s.get("battery", {})
        pct = battery.get("remaining_pct")
        bat_pct = f"{pct:.0f}%" if pct is not None else "?"

        return (
            f" NET: {net}"
            f"  BAT: {bat_v} {bat_pct}"
            f"  PI: {pi_health}"
            f"  TEENSY: {teensy_health}"
            f"  GNSS: {gnss_lock}"
        )

    except Exception:
        return " STATUS: UNAVAILABLE"


# ---------------------------------------------------------------------
# Helpers for cumulative tau / ppb
# ---------------------------------------------------------------------

def _cum_tau(block: dict):
    return block.get("tau")


def _cum_ppb(block: dict):
    return block.get("ppb")


# ---------------------------------------------------------------------
# Combined clocks readout
# ---------------------------------------------------------------------

def clocks_combined_readout() -> list[str]:
    lines = []

    try:
        p = _get_pi_clocks_report()
    except Exception:
        return ["CLOCKS: UNAVAILABLE"]

    report = p.get("report", {})
    state = report.get("campaign_state") or p.get("campaign_state", "IDLE")
    if state != "STARTED":
        return [f"CLOCKS: {state}"]

    r = report
    campaign = r.get("campaign", "?")
    elapsed = r.get("campaign_elapsed", "00:00:00")
    n = r.get("pps_count", 0)

    try:
        t = _get_teensy_clocks_report()
    except Exception:
        t = {}

    cal = _frag(r, "calibrate_ocxo", t.get("calibrate_ocxo", "OFF"))

    baseline = _get_clocks_baseline()
    baseline_ppb = baseline.get("baseline_ppb", {}) if baseline else {}
    baseline_id = baseline.get("baseline_id", "?") if baseline else None
    baseline_campaign = baseline.get("baseline_campaign", "?") if baseline else None

    servo_str = cal if cal and cal != "OFF" else "IDLE"
    baseline_str = f"BASELINE: #{baseline_id} ({baseline_campaign})" if baseline_id else "BASELINE: NONE"

    lines.append(
        f"CAMPAIGN: {campaign}  ELAPSED: {elapsed}  n={n}"
        f"    SERVO: {servo_str}"
        f"    {baseline_str}"
    )
    lines.append("")

    lines.append(
        f"{'CLK':<6}"
        f"{'TAU':>18}"
        f"{'PPB':>10}"
        f"{'RAW':>14}"
        f"{'RES':>6}"
        f"{'MEAN':>8}"
        f"{'SD':>8}"
        f"{'N':>6}"
        f"  "
        f"{'BASE':>10}"
        f"{'NOW':>10}"
        f"{'DELTA':>10}"
        f"   EXTRA"
    )

    # -----------------------------------------------------------------
    # GNSS
    # -----------------------------------------------------------------
    gnss_blk = r.get("gnss", {})
    gnss_tau = _cum_tau(gnss_blk)
    gnss_ppb = _cum_ppb(gnss_blk)
    gnss_res = gnss_blk.get("residual")
    gnss_mean = None
    gnss_sd = None
    gnss_n = None
    gnss_base = baseline_ppb.get("gnss")
    gnss_now = gnss_ppb

    if gnss_base is not None and gnss_now is not None:
        gnss_delta = gnss_now - gnss_base
        gnss_comp = (
            f"{_fmt(gnss_base, '>10.3f', 10)}"
            f"{_fmt(gnss_now, '>10.3f', 10)}"
            f"{_fmt(gnss_delta, '>+10.3f', 10)}"
        )
    else:
        gnss_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'GNSS':<6}"
        f"{_fmt(gnss_tau, '>18.12f', 18)}"
        f"{_fmt(gnss_ppb, '>10.3f', 10)}"
        f"{'---':>14}"
        f"{_fmt(gnss_res, '>6d', 6)}"
        f"{_fmt(gnss_mean, '>8.3f', 8)}"
        f"{_fmt(gnss_sd, '>8.3f', 8)}"
        f"{_fmt(gnss_n, '>6d', 6)}"
        f"  "
        f"{gnss_comp}"
        f"   stream={'OK' if gnss_blk.get('stream_valid', True) else 'BAD'}"
    )

    # -----------------------------------------------------------------
    # GNSS_RAW
    # -----------------------------------------------------------------
    gnss_raw_blk = r.get("gnss_raw", {})
    gnss_raw_tau = _cum_tau(gnss_raw_blk)
    gnss_raw_ppb = _cum_ppb(gnss_raw_blk)
    gnss_raw_res = gnss_raw_blk.get("pred_residual")
    gnss_raw_mean = gnss_raw_blk.get("pred_mean")
    gnss_raw_sd = gnss_raw_blk.get("pred_stddev")
    gnss_raw_n = gnss_raw_blk.get("pred_n")
    gnss_raw_base = baseline_ppb.get("gnss_raw")
    gnss_raw_now = gnss_raw_ppb

    if gnss_raw_base is not None and gnss_raw_now is not None:
        gnss_raw_delta = gnss_raw_now - gnss_raw_base
        gnss_raw_comp = (
            f"{_fmt(gnss_raw_base, '>10.3f', 10)}"
            f"{_fmt(gnss_raw_now, '>10.3f', 10)}"
            f"{_fmt(gnss_raw_delta, '>+10.3f', 10)}"
        )
    else:
        gnss_raw_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'GN_RAW':<6}"
        f"{_fmt(gnss_raw_tau, '>18.12f', 18)}"
        f"{_fmt(gnss_raw_ppb, '>10.3f', 10)}"
        f"{'---':>14}"
        f"{_fmt(gnss_raw_res, '>6.1f', 6)}"
        f"{_fmt(gnss_raw_mean, '>8.3f', 8)}"
        f"{_fmt(gnss_raw_sd, '>8.3f', 8)}"
        f"{_fmt(gnss_raw_n, '>6d', 6)}"
        f"  "
        f"{gnss_raw_comp}"
    )

    # -----------------------------------------------------------------
    # DWT
    # -----------------------------------------------------------------
    dwt_blk = r.get("dwt", {})
    dwt_tau = _cum_tau(dwt_blk)
    dwt_ppb = _cum_ppb(dwt_blk)
    dwt_raw = _frag(r, "dwt_cycle_count_between_pps")
    dwt_res = None
    dwt_mean = None
    dwt_sd = None
    dwt_n = None
    dwt_base = baseline_ppb.get("dwt")
    dwt_now = dwt_ppb

    if dwt_base is not None and dwt_now is not None:
        dwt_delta = dwt_now - dwt_base
        dwt_comp = (
            f"{_fmt(dwt_base, '>10.3f', 10)}"
            f"{_fmt(dwt_now, '>10.3f', 10)}"
            f"{_fmt(dwt_delta, '>+10.3f', 10)}"
        )
    else:
        dwt_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'DWT':<6}"
        f"{_fmt(dwt_tau, '>18.12f', 18)}"
        f"{_fmt(dwt_ppb, '>10.3f', 10)}"
        f"{_comma_int(dwt_raw, 14)}"
        f"{_fmt(dwt_res, '>6.1f', 6)}"
        f"{_fmt(dwt_mean, '>8.3f', 8)}"
        f"{_fmt(dwt_sd, '>8.3f', 8)}"
        f"{_fmt(dwt_n, '>6d', 6)}"
        f"  "
        f"{dwt_comp}"
    )

    # -----------------------------------------------------------------
    # OCXO rows
    # -----------------------------------------------------------------
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        blk = r.get(key, {})
        tau = _cum_tau(blk)
        ppb = _cum_ppb(blk)

        raw = blk.get("dwt_cycles_between_edges", _frag(r, f"{key}_dwt_cycles_between_edges"))
        res = blk.get("second_residual_ns", _frag(r, f"{key}_second_residual_ns"))

        # Current data model does not provide Pi-side Welford prediction stats
        # for OCXO domains in the report block, so leave blank unless present.
        mean = blk.get("pred_mean")
        sd = blk.get("pred_stddev")
        pred_n = blk.get("pred_n")

        base_ppb = baseline_ppb.get(key)
        now_ppb = ppb

        if base_ppb is not None and now_ppb is not None:
            delta = now_ppb - base_ppb
            comp = (
                f"{_fmt(base_ppb, '>10.3f', 10)}"
                f"{_fmt(now_ppb, '>10.3f', 10)}"
                f"{_fmt(delta, '>+10.3f', 10)}"
            )
        else:
            comp = f"{'---':>10}{'---':>10}{'---':>10}"

        dac_now = blk.get("dac", _frag(r, f"{key}_dac"))
        gnss_ns_at_edge = blk.get("gnss_ns_at_edge", _frag(r, f"{key}_gnss_ns_at_edge"))
        ns_count_at_edge = blk.get("ns_count_at_edge", _frag(r, f"{key}_ns_count_at_edge"))
        ns_count_at_pps = blk.get("ns_count_at_pps", _frag(r, f"{key}_ns_count_at_pps"))
        edge_dwt = blk.get("dwt_at_edge", _frag(r, f"{key}_dwt_at_edge"))
        sec_res = blk.get("second_residual_ns", _frag(r, f"{key}_second_residual_ns"))

        extra_parts = []
        if dac_now is not None:
            dac_now_f = float(dac_now)
            dac_volts = dac_now_f * 3.002 / 65535.0
            extra_parts.append(f"DAC={dac_now_f:>9.3f} {dac_volts:.5f}V")
        if edge_dwt is not None:
            extra_parts.append(f"EDGE={int(edge_dwt)}")
        if gnss_ns_at_edge is not None:
            extra_parts.append(f"EDGE_GNSS={int(gnss_ns_at_edge)}")
        if ns_count_at_edge is not None:
            extra_parts.append(f"EDGE_NS={int(ns_count_at_edge)}")
        if sec_res is not None:
            extra_parts.append(f"SEC_RES={int(sec_res):+d}")
        if ns_count_at_pps is not None:
            extra_parts.append(f"PPS_NS={int(ns_count_at_pps)}")

        extra = "   " + " ".join(extra_parts)

        lines.append(
            f"{name:<6}"
            f"{_fmt(tau, '>18.12f', 18)}"
            f"{_fmt(ppb, '>10.3f', 10)}"
            f"{_comma_int(raw, 14)}"
            f"{_sign_int(res, 6)}"
            f"{_fmt(mean, '>8.3f', 8)}"
            f"{_fmt(sd, '>8.3f', 8)}"
            f"{_fmt(pred_n, '>6d', 6)}"
            f"  "
            f"{comp}"
            f"{extra}"
        )

    # -----------------------------------------------------------------
    # DAC Welford
    # -----------------------------------------------------------------
    baseline_dac_mean = baseline.get("baseline_dac_mean", {}) if baseline else {}

    lines.append("")
    lines.append(
        f"{'DAC':<6}"
        f"{'':>18}"
        f"{'':>10}"
        f"{'':>14}"
        f"{'':>6}"
        f"{'MEAN':>8}"
        f"{'SD':>8}"
        f"{'N':>6}"
        f"  "
        f"{'BASE':>10}"
        f"{'NOW':>10}"
        f"{'DELTA':>10}"
    )

    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        blk = r.get(key, {})
        dac_n = blk.get("dac_n")
        dac_mean = blk.get("dac_mean")
        dac_sd = blk.get("dac_stddev")

        # Fallback: if no Welford DAC block exists, show current DAC in NOW column.
        current_dac = blk.get("dac", _frag(r, f"{key}_dac"))
        base_dac = baseline_dac_mean.get(key)

        if base_dac is not None and current_dac is not None:
            dac_delta = float(current_dac) - float(base_dac)
            dac_comp = (
                f"{_fmt(base_dac, '>10.3f', 10)}"
                f"{_fmt(current_dac, '>10.3f', 10)}"
                f"{_fmt(dac_delta, '>+10.3f', 10)}"
            )
        else:
            dac_comp = f"{'---':>10}{'---':>10}{'---':>10}"

        lines.append(
            f"{name:<6}"
            f"{'':>18}"
            f"{'':>10}"
            f"{'':>14}"
            f"{'':>6}"
            f"{_fmt(dac_mean, '>8.3f', 8)}"
            f"{_fmt(dac_sd, '>8.3f', 8)}"
            f"{_fmt(dac_n, '>6d', 6)}"
            f"  "
            f"{dac_comp}"
        )

    lines.append("")

    # -----------------------------------------------------------------
    # NOW servo detail
    # -----------------------------------------------------------------
    if cal == "NOW":
        vref = 3.002

        lines.append(
            f"{'CAL':<6}"
            f"{'SEC_RES':>12}"
            f"{'PRED_NS':>14}"
            f"{'EDGE_GNSS':>20}"
            f"  "
            f"{'DAC':>10}"
            f"{'V_OUT':>10}"
            f"{'PPS_NS':>16}"
        )

        for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
            residual = _frag(r, f"{key}_second_residual_ns")
            pred_ns = _frag(r, f"{key}_ns_count_next_second_prediction")
            edge_gnss_ns = _frag(r, f"{key}_gnss_ns_at_edge")
            dac_now = _frag(r, f"{key}_dac")
            pps_ns = _frag(r, f"{key}_ns_count_at_pps")

            dac_now_f = float(dac_now) if dac_now is not None else None
            v_now = (dac_now_f * vref / 65535.0) if dac_now_f is not None else None

            lines.append(
                f"{name:<6}"
                f"{_sign_int(residual, 12)}"
                f"{_comma_int(pred_ns, 14)}"
                f"{_comma_int(edge_gnss_ns, 20)}"
                f"  "
                f"{_fmt(dac_now_f, '>10.3f', 10)}"
                f"{_fmt(v_now, '>10.5f', 10)}"
                f"{_comma_int(pps_ns, 16)}"
            )

        lines.append("")

    # -----------------------------------------------------------------
    # TIME
    # -----------------------------------------------------------------
    gnss_time = r.get("gnss_time_utc", "---")
    system_time = r.get("system_time_utc", "---")

    lines.append(f"TIME  GNSS: {gnss_time}    SYSTEM: {system_time}")
    lines.append("")

    # -----------------------------------------------------------------
    # DWT detail
    # -----------------------------------------------------------------
    dwt_pred = _frag(r, "dwt_cycle_count_next_second_prediction")
    dwt_actual = _frag(r, "dwt_cycle_count_between_pps")
    dwt_pps_residual = _frag(r, "dwt_pps_residual")
    dwt_cyccnt_at_pps = _frag(r, "dwt_cycle_count_at_pps")
    dwt_cycles_64 = r.get("teensy_dwt_cycles")
    gnss_ns_64 = r.get("teensy_gnss_ns")

    dwt_label_w = 12
    dwt_num_w = 20

    lines.append(
        f"DWT   "
        f"{'PREDICTED:':>{dwt_label_w}} {_comma_int(dwt_pred, dwt_num_w)}"
        f"    {'ACTUAL:':<{dwt_label_w}} {_comma_int(dwt_actual, dwt_num_w)}"
        f"    {'PPS_RESIDUAL:':>13} {_sign_int(dwt_pps_residual, 6)}"
        f"    {'CYCCNT@PPS:':>11} {_comma_int(dwt_cyccnt_at_pps, 14)}"
    )
    lines.append(
        f"      "
        f"{'DWT_CYCLES:':>{dwt_label_w}} {_comma_int(dwt_cycles_64, dwt_num_w)}"
        f"    {'GNSS_NS:':<{dwt_label_w}} {_comma_int(gnss_ns_64, dwt_num_w)}"
    )
    lines.append("")

    # -----------------------------------------------------------------
    # Interrupt diagnostics
    # -----------------------------------------------------------------
    lines.append(
        f"DIAG  "
        f"PPS app={_comma_int(_frag(r, 'pps_diag_approach_cycles'), 6)}"
        f" sh={_comma_int(_frag(r, 'pps_diag_shadow_dwt'), 10)}"
        f" an={_comma_int(_frag(r, 'pps_diag_anomaly_count'), 4)}"
        f"    "
        f"OX1 app={_comma_int(_frag(r, 'ocxo1_diag_approach_cycles'), 6)}"
        f" sh={_comma_int(_frag(r, 'ocxo1_diag_shadow_dwt'), 10)}"
        f" an={_comma_int(_frag(r, 'ocxo1_diag_anomaly_count'), 4)}"
        f"    "
        f"OX2 app={_comma_int(_frag(r, 'ocxo2_diag_approach_cycles'), 6)}"
        f" sh={_comma_int(_frag(r, 'ocxo2_diag_shadow_dwt'), 10)}"
        f" an={_comma_int(_frag(r, 'ocxo2_diag_anomaly_count'), 4)}"
    )
    lines.append(
        f"PRE   "
        f"PPS arm={_comma_int(_frag(r, 'pps_diag_prespin_arm_count'), 6)}"
        f" to={_comma_int(_frag(r, 'pps_diag_prespin_timeout_count'), 4)}"
        f"    "
        f"OX1 arm={_comma_int(_frag(r, 'ocxo1_diag_prespin_arm_count'), 6)}"
        f" to={_comma_int(_frag(r, 'ocxo1_diag_prespin_timeout_count'), 4)}"
        f"    "
        f"OX2 arm={_comma_int(_frag(r, 'ocxo2_diag_prespin_arm_count'), 6)}"
        f" to={_comma_int(_frag(r, 'ocxo2_diag_prespin_timeout_count'), 4)}"
    )
    lines.append("")

    # -----------------------------------------------------------------
    # GNSS status + position
    # -----------------------------------------------------------------
    try:
        snapshot = _get_system_snapshot()
    except Exception:
        snapshot = {}

    gnss = snapshot.get("gnss", {})

    discipline = gnss.get("discipline", {})
    freq_mode_name = discipline.get("freq_mode_name", "?") if isinstance(discipline, dict) else str(discipline)

    survey = gnss.get("survey_mode", {})
    pos_mode = survey.get("receiver_mode", "?") if isinstance(survey, dict) else "?"

    integrity = gnss.get("integrity", {})
    traim = integrity.get("traim", "?") if isinstance(integrity, dict) else "?"

    sats = gnss.get("satellites", "?")
    hdop = gnss.get("hdop")
    hdop_str = f"{hdop:.2f}" if hdop is not None else "---"

    lat = gnss.get("latitude_deg")
    lon = gnss.get("longitude_deg")
    alt_gnss = gnss.get("altitude_m")
    ellipsoid = gnss.get("ellipsoid_height_m")
    geoid = gnss.get("geoid_sep_m")

    gnss_line = (
        f"GNSS  MODE: {pos_mode}"
        f"  DISC: {freq_mode_name}"
        f"  SATS: {sats}"
        f"  HDOP: {hdop_str}"
        f"  TRAIM: {traim}"
    )

    pos_parts = []
    if lat is not None and lon is not None:
        pos_parts.append(f"LAT: {lat:.6f}")
        pos_parts.append(f"LON: {lon:.6f}")
    if alt_gnss is not None:
        pos_parts.append(f"ALT(MSL): {alt_gnss:.1f}m")
    if ellipsoid is not None:
        pos_parts.append(f"ELLIP: {ellipsoid:.1f}m")
    if geoid is not None:
        pos_parts.append(f"GEOID: {geoid:.1f}m")

    if pos_parts:
        gnss_line += "  " + "  ".join(pos_parts)

    lines.append(gnss_line)
    lines.append("")

    # -----------------------------------------------------------------
    # Environment
    # -----------------------------------------------------------------
    env = snapshot.get("environment", {})
    pi_data = snapshot.get("pi", {})
    teensy_data = snapshot.get("teensy", {})

    temp_c = env.get("temperature_c")
    humidity = env.get("humidity_pct")
    pressure = env.get("pressure_hpa")
    baro_alt = env.get("altitude_m")
    pi_temp = pi_data.get("cpu_temp_c")
    teensy_temp = teensy_data.get("cpu_temp_c")

    env_parts = []
    if temp_c is not None:
        env_parts.append(f"AMBIENT: {temp_c:.1f}°C")
    if humidity is not None:
        env_parts.append(f"RH: {humidity:.0f}%")
    if pressure is not None:
        env_parts.append(f"BARO: {pressure:.1f}hPa")
    if baro_alt is not None:
        env_parts.append(f"BARO_ALT: {baro_alt:.1f}m")
    if pi_temp is not None:
        env_parts.append(f"PI: {pi_temp:.1f}°C")
    if teensy_temp is not None:
        env_parts.append(f"TEENSY: {teensy_temp:.1f}°C")

    lines.append("ENV   " + "  ".join(env_parts))
    lines.append("")

    # -----------------------------------------------------------------
    # Power
    # -----------------------------------------------------------------
    power = snapshot.get("power", {})

    def _find_rail(label_match: str) -> dict | None:
        for bus_key, devices in power.items():
            if not bus_key.startswith("i2c-"):
                continue
            if isinstance(devices, dict):
                for rail in devices.values():
                    if isinstance(rail, dict) and label_match.lower() in rail.get("label", "").lower():
                        return rail
        return None

    power_rails = [
        ("PI DOMAIN",     _find_rail("pi domain")),
        ("TEENSY DOMAIN", _find_rail("teensy")),
        ("OCXO1 DOMAIN",  _find_rail("ocxo1")),
        ("OCXO2 DOMAIN",  _find_rail("ocxo2")),
    ]

    lines.append(f"{'POWER':<18}{'V':>10}{'MA':>10}{'W':>10}")
    for label, rail in power_rails:
        if rail:
            lines.append(
                f"{label:<18}"
                f"{rail['volts']:>10.3f}"
                f"{rail['amps']:>10.1f}"
                f"{rail['watts']:>10.3f}"
            )
        else:
            lines.append(f"{label:<18}{'---':>10}{'---':>10}{'---':>10}")

    return lines


# ---------------------------------------------------------------------
# Readout registry
# ---------------------------------------------------------------------

READOUTS = [
    ("CLOCKS", lambda: _safe_lines(clocks_combined_readout)),
]