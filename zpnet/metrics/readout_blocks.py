"""
ZPNet Metrics Readout Blocks — Dense Clocks Panel

Data source:
  The Pi CLOCKS report IS the most recent TIMEBASE record plus
  campaign_state.  All clock data lives in the fragment sub-dict
  (Teensy-authoritative).  Pi-side synthetic clocks live in
  extra_clocks.

Column layout (CLK rows):
  NAME   VALUE (ns or cycles)   TAU   PPB   RAW   RES   MEAN   SD   N   BASE   NOW   DELTA

Column layout (DAC rows):
  NAME   DAC_VALUE VOLTAGE   (blanks)   MEAN   SD   N   BASE   NOW   DELTA

Column layout (INT rows):
  NAME   START_GNSS_NS   END_GNSS_NS   DELTA_NS
"""

from zpnet.processes.processes import send_command

DWT_EXPECTED_PER_PPS = 1_008_000_000
VREF = 3.002

# Persist last-seen PPS edge so the readout can show a start→end
# interval for PPS without upstream state.
_PREV_EDGE_NS = {
    "pps": None,
}


# ---------------------------------------------------------------------
# Data fetchers
# ---------------------------------------------------------------------

def _get_system_snapshot() -> dict:
    return send_command(machine="PI", subsystem="SYSTEM", command="REPORT")["payload"]


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


def _frag_first(r: dict, *keys, default=None):
    for key in keys:
        val = _frag(r, key, None)
        if val is not None:
            return val
    return default


def _extra(r: dict, key: str, default=None):
    ec = r.get("extra_clocks")
    if isinstance(ec, dict):
        val = ec.get(key)
        if val is not None:
            return val
    return default


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


def _has_any(r: dict, *keys) -> bool:
    return any(_frag(r, k, None) is not None for k in keys)


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
# Baseline comparison helper
# ---------------------------------------------------------------------

def _baseline_comp(base_val, now_val):
    if base_val is not None and now_val is not None:
        delta = float(now_val) - float(base_val)
        return (
            f"{_fmt(base_val, '>10.3f', 10)}"
            f"{_fmt(now_val, '>10.3f', 10)}"
            f"{_fmt(delta, '>+10.3f', 10)}"
        )
    return f"{'---':>10}{'---':>10}{'---':>10}"


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

    cal = _frag(r, "calibrate_ocxo", "OFF")

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

    # ── Column widths ──
    W_NAME  = 6
    W_VALUE = 20
    W_TAU   = 18
    W_PPB   = 10
    W_RAW   = 14
    W_RES   = 8
    W_MEAN  = 10
    W_SD    = 8
    W_N     = 6
    W_BASE  = 10
    W_NOW   = 10
    W_DELTA = 10

    # ── CLK header ──
    lines.append(
        f"{'CLK':<{W_NAME}}"
        f"{'VALUE':>{W_VALUE}}"
        f"{'TAU':>{W_TAU}}"
        f"{'PPB':>{W_PPB}}"
        f"{'RAW':>{W_RAW}}"
        f"{'RES':>{W_RES}}"
        f"{'MEAN':>{W_MEAN}}"
        f"{'SD':>{W_SD}}"
        f"{'N':>{W_N}}"
        f"  "
        f"{'BASE':>{W_BASE}}"
        f"{'NOW':>{W_NOW}}"
        f"{'DELTA':>{W_DELTA}}"
    )

    # ── GNSS ──
    gnss_ns = _to_int(_frag(r, "gnss_ns"))
    gnss_tau = 1.0
    gnss_ppb = 0.0
    lines.append(
        f"{'GNSS':<{W_NAME}}"
        f"{_comma_int(gnss_ns, W_VALUE)}"
        f"{_fmt(gnss_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(gnss_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{'---':>{W_RAW}}"
        f"{'---':>{W_RES}}"
        f"{'---':>{W_MEAN}}"
        f"{'---':>{W_SD}}"
        f"{'---':>{W_N}}"
        f"  {_baseline_comp(baseline_ppb.get('gnss'), gnss_ppb)}"
    )

    # ── VCLOCK (measured peer of OCXO1/OCXO2; GNSS-disciplined 10 MHz reference) ──
    vclock_ns = _to_int(_frag(r, "vclock_ns_count_at_pps"))
    vclock_tau = _to_float(_frag(r, "vclock_tau"))
    vclock_ppb = _to_float(_frag(r, "vclock_ppb"))
    vclock_raw = _to_int(_frag(r, "vclock_gnss_ns_between_edges"))
    vclock_res = _to_int(_frag(r, "vclock_second_residual_ns"))
    vclock_mean = _to_float(_frag(r, "vclock_welford_mean"))
    vclock_sd = _to_float(_frag(r, "vclock_welford_stddev"))
    vclock_n = _to_int(_frag(r, "vclock_welford_n"))
    lines.append(
        f"{'VCLOCK':<{W_NAME}}"
        f"{_comma_int(vclock_ns, W_VALUE)}"
        f"{_fmt(vclock_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(vclock_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{_comma_int(vclock_raw, W_RAW)}"
        f"{_sign_int(vclock_res, W_RES)}"
        f"{_fmt(vclock_mean, f'>{W_MEAN}.3f', W_MEAN)}"
        f"{_fmt(vclock_sd, f'>{W_SD}.3f', W_SD)}"
        f"{_fmt(vclock_n, f'>{W_N}d', W_N)}"
        f"  {_baseline_comp(baseline_ppb.get('vclock'), vclock_ppb)}"
    )

    # ── GNSS_RAW ──
    gnss_raw_ns = _to_int(_extra(r, "gnss_raw_ns"))
    gnss_raw_tau = _to_float(_extra(r, "gnss_raw_tau"))
    gnss_raw_ppb = _to_float(_extra(r, "gnss_raw_ppb"))
    gnss_raw_res = _to_float(_extra(r, "gnss_raw_drift_ppb"))
    gnss_raw_mean = _to_float(_extra(r, "gnss_raw_welford_mean"))
    gnss_raw_sd = _to_float(_extra(r, "gnss_raw_welford_stddev"))
    gnss_raw_n = _to_int(_extra(r, "gnss_raw_welford_n"))
    lines.append(
        f"{'GN_RAW':<{W_NAME}}"
        f"{_comma_int(gnss_raw_ns, W_VALUE)}"
        f"{_fmt(gnss_raw_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(gnss_raw_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{'---':>{W_RAW}}"
        f"{_fmt(gnss_raw_res, f'>{W_RES}.1f', W_RES)}"
        f"{_fmt(gnss_raw_mean, f'>{W_MEAN}.3f', W_MEAN)}"
        f"{_fmt(gnss_raw_sd, f'>{W_SD}.3f', W_SD)}"
        f"{_fmt(gnss_raw_n, f'>{W_N}d', W_N)}"
        f"  {_baseline_comp(baseline_ppb.get('gnss_raw'), gnss_raw_ppb)}"
    )

    # ── DWT ──
    dwt_present = _has_any(
        r,
        "dwt_cycle_count_total",
        "dwt_tau",
        "dwt_ppb",
        "dwt_cycle_count_between_pps",
        "dwt_welford_mean",
    )
    if dwt_present:
        dwt_total = _to_int(_frag(r, "dwt_cycle_count_total"))
        dwt_tau = _to_float(_frag(r, "dwt_tau"))
        dwt_ppb = _to_float(_frag(r, "dwt_ppb"))
        dwt_raw = _to_int(_frag(r, "dwt_cycle_count_between_pps"))
        dwt_res = dwt_raw - DWT_EXPECTED_PER_PPS if dwt_raw is not None else None
        dwt_mean = _to_float(_frag(r, "dwt_welford_mean"))
        dwt_sd = _to_float(_frag(r, "dwt_welford_stddev"))
        dwt_n = _to_int(_frag(r, "dwt_welford_n"))
        lines.append(
            f"{'DWT':<{W_NAME}}"
            f"{_comma_int(dwt_total, W_VALUE)}"
            f"{_fmt(dwt_tau, f'>{W_TAU}.12f', W_TAU)}"
            f"{_fmt(dwt_ppb, f'>{W_PPB}.3f', W_PPB)}"
            f"{_comma_int(dwt_raw, W_RAW)}"
            f"{_sign_int(dwt_res, W_RES)}"
            f"{_fmt(dwt_mean, f'>{W_MEAN}.3f', W_MEAN)}"
            f"{_fmt(dwt_sd, f'>{W_SD}.3f', W_SD)}"
            f"{_fmt(dwt_n, f'>{W_N}d', W_N)}"
            f"  {_baseline_comp(baseline_ppb.get('dwt'), dwt_ppb)}"
        )

    # ── OCXO1, OCXO2 ──
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        ocxo_ns = _to_int(_frag_first(r, f"{key}_ns_count_at_pps", f"{key}_ns"))
        tau = _to_float(_frag(r, f"{key}_tau"))
        ppb = _to_float(_frag(r, f"{key}_ppb"))
        raw = _to_int(_frag(r, f"{key}_gnss_ns_between_edges"))
        res = _to_int(_frag_first(r, f"{key}_diag_ocxo_second_residual_ns", f"{key}_second_residual_ns"))
        mean = _to_float(_frag(r, f"{key}_welford_mean"))
        sd = _to_float(_frag(r, f"{key}_welford_stddev"))
        wn = _to_int(_frag(r, f"{key}_welford_n"))
        lines.append(
            f"{name:<{W_NAME}}"
            f"{_comma_int(ocxo_ns, W_VALUE)}"
            f"{_fmt(tau, f'>{W_TAU}.12f', W_TAU)}"
            f"{_fmt(ppb, f'>{W_PPB}.3f', W_PPB)}"
            f"{_comma_int(raw, W_RAW)}"
            f"{_sign_int(res, W_RES)}"
            f"{_fmt(mean, f'>{W_MEAN}.3f', W_MEAN)}"
            f"{_fmt(sd, f'>{W_SD}.3f', W_SD)}"
            f"{_fmt(wn, f'>{W_N}d', W_N)}"
            f"  {_baseline_comp(baseline_ppb.get(key), ppb)}"
        )

    # ── DAC detail ──
    lines.append("")
    lines.append(
        f"{'DAC':<6}"
        f"{'':>{W_VALUE}}"
        f"{'':>{W_TAU}}"
        f"{'':>{W_PPB}}"
        f"{'':>{W_RAW}}"
        f"{'':>{W_RES}}"
        f"{'':>{W_MEAN}}"
        f"{'':>{W_SD}}"
        f"{'':>{W_N}}"
        f"  "
        f"{'BASE':>{W_BASE}}"
        f"{'NOW':>{W_NOW}}"
        f"{'DELTA':>{W_DELTA}}"
    )
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        dac_now = _to_float(_frag(r, f"{key}_dac"))
        dac_volts = (dac_now * VREF / 65535.0) if dac_now is not None else None
        if dac_now is not None:
            dac_label = f"{dac_now:>.3f} {dac_volts:.5f}V"
        else:
            dac_label = "---"

        # The curtailed payload may omit DAC Welford detail. Use the live DAC as NOW.
        base_dac = None
        if baseline:
            base_dac = baseline.get("baseline_dac_mean", {}).get(key)
        now_dac = dac_now

        lines.append(
            f"{name:<6}"
            f"{dac_label:>{W_VALUE}}"
            f"{'':>{W_TAU}}"
            f"{'':>{W_PPB}}"
            f"{'':>{W_RAW}}"
            f"{'':>{W_RES}}"
            f"{'---':>{W_MEAN}}"
            f"{'---':>{W_SD}}"
            f"{'---':>{W_N}}"
            f"  {_baseline_comp(base_dac, now_dac)}"
        )

    # ── Interrupt edge intervals ──
    lines.append("")
    lines.append(
        f"{'INT':<6}"
        f"{'START_GNSS_NS':>22}"
        f"{'END_GNSS_NS':>22}"
        f"{'DELTA_NS':>14}"
    )

    # PPS: use the final PPS event GNSS and a persisted previous edge.
    pps_end = _to_int(_frag_first(r, "pps_diag_gnss_ns_at_event_final", "pps_diag_gnss_ns_at_event"))
    pps_start = _PREV_EDGE_NS.get("pps")
    pps_delta = None
    if pps_start is not None and pps_end is not None:
        pps_delta = pps_end - pps_start
    lines.append(
        f"{'PPS':<6}"
        f"{_comma_int(pps_start, 22)}"
        f"{_comma_int(pps_end, 22)}"
        f"{_comma_int(pps_delta, 14)}"
    )
    if pps_end is not None:
        _PREV_EDGE_NS["pps"] = pps_end

    # OCXO1/OCXO2: use the canonical bucket start/end/delta fields.
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        start_edge = _to_int(_frag(r, f"{key}_diag_ocxo_second_start_gnss_ns_raw"))
        end_edge = _to_int(_frag(r, f"{key}_diag_ocxo_second_end_gnss_ns_raw"))
        delta_edge = _to_int(_frag_first(r, f"{key}_diag_ocxo_second_gnss_ns_observed", f"{key}_gnss_ns_between_edges"))
        lines.append(
            f"{name:<6}"
            f"{_comma_int(start_edge, 22)}"
            f"{_comma_int(end_edge, 22)}"
            f"{_comma_int(delta_edge, 14)}"
        )

    lines.append("")

    # ── NOW servo detail ──
    if cal == "NOW":
        lines.append(
            f"{'CAL':<6}"
            f"{'SEC_RES':>12}"
            f"{'EDGE_GNSS':>20}"
            f"  "
            f"{'DAC':>10}"
            f"{'V_OUT':>10}"
            f"{'PPS_NS':>16}"
        )
        for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
            residual = _to_int(_frag_first(r, f"{key}_diag_ocxo_second_residual_ns", f"{key}_second_residual_ns"))
            edge_gnss_ns = _to_int(_frag_first(r, f"{key}_diag_gnss_ns_at_event_final", f"{key}_gnss_ns_at_edge"))
            dac_now = _to_float(_frag(r, f"{key}_dac"))
            pps_ns = _to_int(_frag_first(r, f"{key}_ns_count_at_pps", f"{key}_ns"))
            v_now = (dac_now * VREF / 65535.0) if dac_now is not None else None
            lines.append(
                f"{name:<6}"
                f"{_sign_int(residual, 12)}"
                f"{_comma_int(edge_gnss_ns, 20)}"
                f"  "
                f"{_fmt(dac_now, '>10.3f', 10)}"
                f"{_fmt(v_now, '>10.5f', 10)}"
                f"{_comma_int(pps_ns, 16)}"
            )
        lines.append("")

    # ── Time ──
    gnss_time = r.get("gnss_time_utc", "---")
    system_time = r.get("system_time_utc", "---")
    lines.append(f"TIME  GNSS: {gnss_time}    SYSTEM: {system_time}")
    lines.append("")

    # ── DWT detail ──
    dwt_pred = _frag(r, "dwt_cycle_count_next_second_prediction")
    dwt_actual = _frag(r, "dwt_cycle_count_between_pps")
    dwt_cyccnt_at_pps = _frag(r, "dwt_cycle_count_at_pps")
    dwt_cycles_64 = _frag(r, "dwt_cycle_count_total")
    gnss_ns_64 = _frag(r, "gnss_ns")
    if any(v is not None for v in [dwt_pred, dwt_actual, dwt_cyccnt_at_pps, dwt_cycles_64]):
        dwt_label_w = 12
        dwt_num_w = 20
        lines.append(
            f"DWT   "
            f"{'PREDICTED:':>{dwt_label_w}} {_comma_int(dwt_pred, dwt_num_w)}"
            f"    {'ACTUAL:':<{dwt_label_w}} {_comma_int(dwt_actual, dwt_num_w)}"
            f"    {'CYCCNT@PPS:':>11} {_comma_int(dwt_cyccnt_at_pps, 14)}"
        )
        lines.append(
            f"      "
            f"{'DWT_CYCLES:':>{dwt_label_w}} {_comma_int(dwt_cycles_64, dwt_num_w)}"
            f"    {'GNSS_NS:':<{dwt_label_w}} {_comma_int(gnss_ns_64, dwt_num_w)}"
        )
        lines.append("")

    # ── GNSS status ──
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

    # ── Environment ──
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

    return lines


READOUTS = [
    ("CLOCKS", lambda: _safe_lines(clocks_combined_readout)),
]