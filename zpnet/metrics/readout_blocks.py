"""
ZPNet Metrics Readout Blocks — Dense Clocks Panel

Adjusted rules:
  • TAU = cumulative total for the campaign
  • PPB = cumulative total for the campaign
  • RES = latest single-second residual
  • DWT TAU = total DWT ns / total nominal GNSS ns
    where nominal DWT second is 1,008,000,000 cycles/sec

Current report contract used by this panel:
  • PI report.<clock>.ppb is authoritative cumulative PPB for campaign clocks
  • PI report.<clock>.tau is ignored for clocks known to be wrong upstream
  • TAU is derived from cumulative PPB as: 1.0 + ppb / 1e9
"""

from zpnet.processes.processes import send_command

DWT_EXPECTED_PER_PPS = 1_008_000_000


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


def _frag(r: dict, key: str, default=None):
    frag = r.get("fragment")
    if isinstance(frag, dict):
        val = frag.get(key)
        if val is not None:
            return val
    val = r.get(key)
    return val if val is not None else default


def _to_float(v):
    if v is None:
        return None
    try:
        return float(v)
    except Exception:
        return None


def _to_int(v):
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        return None


def _tau_to_ppb(tau):
    if tau is None:
        return None
    return (float(tau) - 1.0) * 1_000_000_000.0


def _ppb_to_tau(ppb):
    if ppb is None:
        return None
    return 1.0 + (float(ppb) / 1_000_000_000.0)


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
# Cumulative tau helpers
# ---------------------------------------------------------------------

def _cum_tau_gnss():
    return 1.0


def _cum_tau_from_report_block(block: dict):
    return _to_float(block.get("tau"))


def _cum_tau_dwt(r: dict):
    total_cycles = _to_int(r.get("teensy_dwt_cycles")) or _to_int(_frag(r, "dwt_cycle_count_total"))
    gnss_ns = _to_int(r.get("teensy_gnss_ns")) or _to_int(_frag(r, "gnss_ns"))
    if total_cycles is None or gnss_ns in (None, 0):
        return None
    dwt_ns_total = total_cycles * 1_000_000_000.0 / DWT_EXPECTED_PER_PPS
    return dwt_ns_total / float(gnss_ns)


def _cum_tau_ocxo(r: dict, key: str):
    ocxo_ns = _to_int(r.get(f"teensy_{key}_ns")) or _to_int(_frag(r, f"{key}_ns_count_at_pps"))
    gnss_ns = _to_int(r.get("teensy_gnss_ns")) or _to_int(_frag(r, "gnss_ns"))
    if ocxo_ns is None or gnss_ns in (None, 0):
        return None
    return float(ocxo_ns) / float(gnss_ns)


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
        f"{'RES':>8}"
        f"{'MEAN':>10}"
        f"{'SD':>8}"
        f"{'N':>6}"
        f"  "
        f"{'BASE':>10}"
        f"{'NOW':>10}"
        f"{'DELTA':>10}"
        f"   EXTRA"
    )

    # GNSS
    gnss_blk = r.get("gnss", {})
    gnss_tau = _cum_tau_gnss()
    gnss_ppb = _tau_to_ppb(gnss_tau)
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
        f"{_fmt(gnss_res, '>8d', 8)}"
        f"{_fmt(gnss_mean, '>10.3f', 10)}"
        f"{_fmt(gnss_sd, '>8.3f', 8)}"
        f"{_fmt(gnss_n, '>6d', 6)}"
        f"  {gnss_comp}"
        f"   stream={'OK' if gnss_blk.get('stream_valid', True) else 'BAD'}"
    )

    # GNSS_RAW
    gnss_raw_blk = r.get("gnss_raw", {})
    gnss_raw_ppb = _to_float(gnss_raw_blk.get('ppb'))
    gnss_raw_tau = _ppb_to_tau(gnss_raw_ppb) if gnss_raw_ppb is not None else _cum_tau_from_report_block(gnss_raw_blk)
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
        f"{_fmt(gnss_raw_res, '>8.1f', 8)}"
        f"{_fmt(gnss_raw_mean, '>10.3f', 10)}"
        f"{_fmt(gnss_raw_sd, '>8.3f', 8)}"
        f"{_fmt(gnss_raw_n, '>6d', 6)}"
        f"  {gnss_raw_comp}"
    )

    # DWT
    dwt_blk = r.get('dwt', {})
    dwt_ppb = _to_float(dwt_blk.get('ppb'))
    dwt_tau = _ppb_to_tau(dwt_ppb) if dwt_ppb is not None else _cum_tau_dwt(r)
    dwt_raw = _to_int(_frag(r, "dwt_cycle_count_between_pps"))
    dwt_res = dwt_raw - DWT_EXPECTED_PER_PPS if dwt_raw is not None else None
    dwt_mean = _to_float(_frag(r, "dwt_residual_mean"))
    dwt_sd = _to_float(_frag(r, "dwt_residual_stddev"))
    dwt_n = _to_int(_frag(r, "dwt_residual_n"))
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
        f"{_sign_int(dwt_res, 8)}"
        f"{_fmt(dwt_mean, '>10.3f', 10)}"
        f"{_fmt(dwt_sd, '>8.3f', 8)}"
        f"{_fmt(dwt_n, '>6d', 6)}"
        f"  {dwt_comp}"
    )

    # OCXO rows
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        blk = r.get(key, {})
        ppb = _to_float(blk.get('ppb'))
        tau = _ppb_to_tau(ppb) if ppb is not None else _cum_tau_ocxo(r, key)
        raw = blk.get("dwt_cycles_between_edges", _frag(r, f"{key}_dwt_cycles_between_edges"))
        res = blk.get("second_residual_ns", _frag(r, f"{key}_second_residual_ns"))
        mean_frag = _frag(r, f"{key}_residual_mean")
        sd_frag = _frag(r, f"{key}_residual_stddev")
        n_frag = _frag(r, f"{key}_residual_n")
        mean = float(mean_frag) if mean_frag is not None else blk.get("pred_mean")
        sd = float(sd_frag) if sd_frag is not None else blk.get("pred_stddev")
        pred_n = int(n_frag) if n_frag is not None else blk.get("pred_n")
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
        if dac_now is not None:
            dac_now_f = float(dac_now)
            dac_volts = dac_now_f * 3.002 / 65535.0
            extra = f"   DAC={dac_now_f:>9.3f} {dac_volts:.5f}V"
        else:
            extra = ""
        lines.append(
            f"{name:<6}"
            f"{_fmt(tau, '>18.12f', 18)}"
            f"{_fmt(ppb, '>10.3f', 10)}"
            f"{_comma_int(raw, 14)}"
            f"{_sign_int(res, 8)}"
            f"{_fmt(mean, '>10.3f', 10)}"
            f"{_fmt(sd, '>8.3f', 8)}"
            f"{_fmt(pred_n, '>6d', 6)}"
            f"  {comp}{extra}"
        )

    # DAC Welford
    baseline_dac_mean = baseline.get("baseline_dac_mean", {}) if baseline else {}
    lines.append("")
    lines.append(
        f"{'DAC':<6}"
        f"{'':>18}"
        f"{'':>10}"
        f"{'':>14}"
        f"{'':>8}"
        f"{'MEAN':>10}"
        f"{'SD':>8}"
        f"{'N':>6}"
        f"  "
        f"{'BASE':>10}"
        f"{'NOW':>10}"
        f"{'DELTA':>10}"
    )
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        blk = r.get(key, {})
        dac_mean_frag = _frag(r, f"dac_{key}_mean")
        dac_sd_frag = _frag(r, f"dac_{key}_stddev")
        dac_n_frag = _frag(r, f"dac_{key}_n")
        dac_mean = float(dac_mean_frag) if dac_mean_frag is not None else blk.get("dac_mean")
        dac_sd = float(dac_sd_frag) if dac_sd_frag is not None else blk.get("dac_stddev")
        dac_n = int(dac_n_frag) if dac_n_frag is not None else blk.get("dac_n")
        current_dac = dac_mean if dac_mean is not None else blk.get("dac", _frag(r, f"{key}_dac"))
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
            f"{'':>8}"
            f"{_fmt(dac_mean, '>10.3f', 10)}"
            f"{_fmt(dac_sd, '>8.3f', 8)}"
            f"{_fmt(dac_n, '>6d', 6)}"
            f"  {dac_comp}"
        )

    lines.append("")

    # NOW servo detail
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

    gnss_time = r.get("gnss_time_utc", "---")
    system_time = r.get("system_time_utc", "---")
    lines.append(f"TIME  GNSS: {gnss_time}    SYSTEM: {system_time}")
    lines.append("")

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

    power = snapshot.get("power", {})
    def _find_rail(label_match: str):
        for bus_key, devices in power.items():
            if not bus_key.startswith("i2c-"):
                continue
            if isinstance(devices, dict):
                for rail in devices.values():
                    if isinstance(rail, dict) and label_match.lower() in rail.get("label", "").lower():
                        return rail
        return None

    power_rails = [
        ("PI DOMAIN", _find_rail("pi domain")),
        ("TEENSY DOMAIN", _find_rail("teensy")),
        ("OCXO1 DOMAIN", _find_rail("ocxo1")),
        ("OCXO2 DOMAIN", _find_rail("ocxo2")),
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


READOUTS = [
    ("CLOCKS", lambda: _safe_lines(clocks_combined_readout)),
]
