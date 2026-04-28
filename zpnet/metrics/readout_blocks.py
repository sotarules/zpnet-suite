"""
ZPNet Metrics Readout Blocks — Dense Clocks Panel

Data source:
  The Pi CLOCKS report IS the most recent TIMEBASE record plus
  campaign_state.  All clock data lives in the fragment sub-dict
  (Teensy-authoritative).  Pi-side synthetic clocks live in
  extra_clocks.

Stats policy (Pi is a stenographer):
  Every statistical quantity shown in this panel is read verbatim from
  the Teensy-authored TIMEBASE_FRAGMENT.  No Pi-side means, stddevs,
  stderrs, or residuals are computed here.  The only Pi-side arithmetic
  is presentation-layer unit conversion (DAC code → voltage) and
  baseline delta (NOW - BASE), neither of which is a statistic.

Column layout (CLK rows):
  NAME   VALUE   TAU   PPB   RAW   RES   MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (DAC rows):
  NAME   DAC_VALUE_VOLTAGE   (blanks)   MEAN   SD   SE   N   BASE   NOW   DELTA

Column layout (INT rows):
  NAME   END_GNSS_NS   DELTA_NS

Witness reports:
  BRIDGE / PPS_PHASE / ROUND_TRIP / ENTRY_LATENCY are not rendered from
  TIMEBASE.  They should be queried directly from process_witness in a
  later metrics pass.
"""

from zpnet.processes.processes import send_command

VREF = 3.002


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


def _extra(r: dict, key: str, default=None):
    ec = r.get("extra_clocks")
    if isinstance(ec, dict):
        val = ec.get(key)
        if val is not None:
            return val
    return default


def _field(r: dict, *keys, default=None):
    """Return the first present TIMEBASE/TIMEBASE_FRAGMENT field.

    The CLOCKS firmware now prefers PPS/VCLOCK names but keeps some legacy
    aliases.  Metrics is a display client, so it tolerates both while
    preferring the canonical PPS/VCLOCK names.
    """
    for key in keys:
        val = _frag(r, key, None)
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
# Welford row renderer — reads 4 firmware-owned statistics verbatim
# ---------------------------------------------------------------------
#
# Returns the formatted MEAN | SD | SE | N column group.  All four values
# come from firmware-published fields:
#     <prefix>_welford_mean
#     <prefix>_welford_stddev
#     <prefix>_welford_stderr
#     <prefix>_welford_n
#
# source_fn(key) is a lookup function returning the value or None.
# Typically _frag or _extra.
#
# mean_decimals overrides the default precision (3) for the MEAN column.
# Useful for large-magnitude signals where sub-ns precision is false
# precision and the extra digits overflow the column.
#
def _welford_cols(source_fn, prefix, w_mean, w_sd, w_se, w_n, mean_decimals=3):
    mean = _to_float(source_fn(f"{prefix}_welford_mean"))
    sd   = _to_float(source_fn(f"{prefix}_welford_stddev"))
    se   = _to_float(source_fn(f"{prefix}_welford_stderr"))
    wn   = _to_int  (source_fn(f"{prefix}_welford_n"))
    return (
        f"{_fmt(mean, f'>{w_mean}.{mean_decimals}f', w_mean)}"
        f"{_fmt(sd,   f'>{w_sd}.3f',   w_sd)}"
        f"{_fmt(se,   f'>{w_se}.3f',   w_se)}"
        f"{_fmt(wn,   f'>{w_n}d',      w_n)}"
    )


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
    n = _to_int(_field(r, "teensy_pps_vclock_count", "teensy_pps_count", "pps_count")) or 0

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
    W_SE    = 8
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
        f"{'SE':>{W_SE}}"
        f"{'N':>{W_N}}"
        f"  "
        f"{'BASE':>{W_BASE}}"
        f"{'NOW':>{W_NOW}}"
        f"{'DELTA':>{W_DELTA}}"
    )

    # ── GNSS (the reference; tau/ppb are tautologically 1.0 / 0.0) ──
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
        f"{'---':>{W_SE}}"
        f"{'---':>{W_N}}"
        f"  {_baseline_comp(baseline_ppb.get('gnss'), gnss_ppb)}"
    )

    # ── VCLOCK (measured peer of OCXO1/OCXO2; the GNSS-disciplined reference) ──
    vclock_ns  = _to_int(_field(r, "vclock_ns", "vclock_gnss_ns_at_pps_vclock", "pps_vclock_ns", "gnss_ns"))
    vclock_tau = _to_float(_frag(r, "vclock_tau"))
    vclock_ppb = _to_float(_frag(r, "vclock_ppb"))
    vclock_raw = _to_int(_frag(r, "vclock_gnss_ns_between_edges"))
    vclock_res = _to_int(_frag(r, "vclock_second_residual_ns"))
    lines.append(
        f"{'VCLOCK':<{W_NAME}}"
        f"{_comma_int(vclock_ns, W_VALUE)}"
        f"{_fmt(vclock_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(vclock_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{_comma_int(vclock_raw, W_RAW)}"
        f"{_sign_int(vclock_res, W_RES)}"
        f"{_welford_cols(lambda k: _frag(r, k), 'vclock', W_MEAN, W_SD, W_SE, W_N)}"
        f"  {_baseline_comp(baseline_ppb.get('vclock'), vclock_ppb)}"
    )

    # ── GN_RAW (GNSS receiver's self-reported drift; Pi-side synthetic) ──
    gnss_raw_ns  = _to_int(_extra(r, "gnss_raw_ns"))
    gnss_raw_tau = _to_float(_extra(r, "gnss_raw_tau"))
    gnss_raw_ppb = _to_float(_extra(r, "gnss_raw_ppb"))
    gnss_raw_res = _to_float(_extra(r, "gnss_raw_drift_ppb"))
    lines.append(
        f"{'GN_RAW':<{W_NAME}}"
        f"{_comma_int(gnss_raw_ns, W_VALUE)}"
        f"{_fmt(gnss_raw_tau, f'>{W_TAU}.12f', W_TAU)}"
        f"{_fmt(gnss_raw_ppb, f'>{W_PPB}.3f', W_PPB)}"
        f"{'---':>{W_RAW}}"
        f"{_fmt(gnss_raw_res, f'>{W_RES}.1f', W_RES)}"
        f"{_welford_cols(lambda k: _extra(r, k), 'gnss_raw', W_MEAN, W_SD, W_SE, W_N)}"
        f"  {_baseline_comp(baseline_ppb.get('gnss_raw'), gnss_raw_ppb)}"
    )

    # ── DWT ──
    dwt_present = _has_any(
        r,
        "dwt_ns",
        "dwt_cycle_count_total",
        "dwt_tau",
        "dwt_ppb",
        "dwt_cycles_between_pps_vclock",
        "dwt_cycle_count_between_pps",
        "dwt_welford_mean",
    )
    if dwt_present:
        dwt_total = _to_int(_field(r, "dwt_ns", "dwt_cycle_count_total"))
        dwt_tau   = _to_float(_frag(r, "dwt_tau"))
        dwt_ppb   = _to_float(_frag(r, "dwt_ppb"))
        dwt_raw   = _to_int(_field(r, "dwt_cycles_between_pps_vclock", "dwt_cycle_count_between_pps"))
        # RES comes from firmware (dwt_second_residual_cycles), not
        # computed Pi-side.
        dwt_res   = _to_int(_frag(r, "dwt_second_residual_cycles"))
        lines.append(
            f"{'DWT':<{W_NAME}}"
            f"{_comma_int(dwt_total, W_VALUE)}"
            f"{_fmt(dwt_tau, f'>{W_TAU}.12f', W_TAU)}"
            f"{_fmt(dwt_ppb, f'>{W_PPB}.3f', W_PPB)}"
            f"{_comma_int(dwt_raw, W_RAW)}"
            f"{_sign_int(dwt_res, W_RES)}"
            f"{_welford_cols(lambda k: _frag(r, k), 'dwt', W_MEAN, W_SD, W_SE, W_N)}"
            f"  {_baseline_comp(baseline_ppb.get('dwt'), dwt_ppb)}"
        )

    # ── OCXO1, OCXO2 ──
    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        ocxo_ns = _to_int(_field(r, f"{key}_ns", f"{key}_ns_at_pps_vclock", f"{key}_ns_count_at_pps"))
        tau     = _to_float(_frag(r, f"{key}_tau"))
        ppb     = _to_float(_frag(r, f"{key}_ppb"))
        raw     = _to_int(_frag(r, f"{key}_gnss_ns_between_edges"))
        res     = _to_int(_frag(r, f"{key}_second_residual_ns"))
        lines.append(
            f"{name:<{W_NAME}}"
            f"{_comma_int(ocxo_ns, W_VALUE)}"
            f"{_fmt(tau, f'>{W_TAU}.12f', W_TAU)}"
            f"{_fmt(ppb, f'>{W_PPB}.3f', W_PPB)}"
            f"{_comma_int(raw, W_RAW)}"
            f"{_sign_int(res, W_RES)}"
            f"{_welford_cols(lambda k: _frag(r, k), key, W_MEAN, W_SD, W_SE, W_N)}"
            f"  {_baseline_comp(baseline_ppb.get(key), ppb)}"
        )

    # ── DAC detail ──
    #
    # Each DAC row shows the current DAC code + its voltage (label column),
    # plus the firmware-owned welford_ocxoN_dac statistics (mean/sd/se/n),
    # aligned to the CLK block's statistic columns so scanning is easy.
    #
    lines.append("")
    lines.append(
        f"{'DAC':<{W_NAME}}"
        f"{'':>{W_VALUE}}"
        f"{'':>{W_TAU}}"
        f"{'':>{W_PPB}}"
        f"{'':>{W_RAW}}"
        f"{'':>{W_RES}}"
        f"{'MEAN':>{W_MEAN}}"
        f"{'SD':>{W_SD}}"
        f"{'SE':>{W_SE}}"
        f"{'N':>{W_N}}"
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

        base_dac = None
        if baseline:
            base_dac = baseline.get("baseline_dac_mean", {}).get(key)

        lines.append(
            f"{name:<{W_NAME}}"
            f"{dac_label:>{W_VALUE}}"
            f"{'':>{W_TAU}}"
            f"{'':>{W_PPB}}"
            f"{'':>{W_RAW}}"
            f"{'':>{W_RES}}"
            f"{_welford_cols(lambda k: _frag(r, k), f'{key}_dac', W_MEAN, W_SD, W_SE, W_N)}"
            f"  {_baseline_comp(base_dac, dac_now)}"
        )

    # ── Witness reports retired from TIMEBASE panel ───────────────
    #
    # BRIDGE / PPS_PHASE / ROUND_TRIP / ENTRY_LATENCY now belong to
    # process_witness.  This dense panel stays focused on the sacred
    # TIMEBASE heartbeat and direct clock ledgers.  A later metrics pass
    # can query process_witness directly and render those reports without
    # shoving them into TIMEBASE.

    # ── Interrupt edge intervals ──
    #
    # END_GNSS_NS is the firmware-reported GNSS moment of the most recent
    # event; DELTA_NS is the firmware-computed between-edges interval.
    # Both come verbatim from the fragment.  No Pi-side state.
    #
    lines.append("")
    lines.append(
        f"{'INT':<{W_NAME}}"
        f"{'END_GNSS_NS':>22}"
        f"{'DELTA_NS':>14}"
    )

    # PPS/VCLOCK — the canonical one-second advancer.  This is the selected
    # VCLOCK edge, not the physical PPS witness edge.
    pps_vclock_end   = _to_int(_field(r, "pps_vclock_ns", "vclock_ns", "vclock_gnss_ns_at_pps_vclock", "gnss_ns"))
    pps_vclock_delta = _to_int(_frag(r, "vclock_gnss_ns_between_edges"))
    lines.append(
        f"{'PPS/V':<{W_NAME}}"
        f"{_comma_int(pps_vclock_end, 22)}"
        f"{_comma_int(pps_vclock_delta, 14)}"
    )

    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        ocxo_end   = _to_int(_frag(r, f"{key}_diag_gnss_ns_at_event"))
        ocxo_delta = _to_int(_frag(r, f"{key}_gnss_ns_between_edges"))
        lines.append(
            f"{name:<{W_NAME}}"
            f"{_comma_int(ocxo_end, 22)}"
            f"{_comma_int(ocxo_delta, 14)}"
        )

    lines.append("")

    # ── NOW servo detail (only shown when servo mode is NOW) ──
    if cal == "NOW":
        lines.append(
            f"{'CAL':<{W_NAME}}"
            f"{'SEC_RES':>12}"
            f"{'EDGE_GNSS':>20}"
            f"  "
            f"{'DAC':>10}"
            f"{'V_OUT':>10}"
            f"{'PPS/V_NS':>16}"
        )
        for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
            residual     = _to_int(_frag(r, f"{key}_second_residual_ns"))
            edge_gnss_ns = _to_int(_frag(r, f"{key}_diag_gnss_ns_at_event"))
            dac_now      = _to_float(_frag(r, f"{key}_dac"))
            pps_ns       = _to_int(_field(r, f"{key}_ns", f"{key}_ns_at_pps_vclock", f"{key}_ns_count_at_pps"))
            v_now        = (dac_now * VREF / 65535.0) if dac_now is not None else None
            lines.append(
                f"{name:<{W_NAME}}"
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
    #
    # ACTUAL = dwt_cycles_between_pps_vclock (firmware).
    # EXPECTED = dwt_expected_per_pps_vclock (firmware).
    # The canonical DWT anchor is dwt_at_pps_vclock: the selected VCLOCK
    # edge chosen by PPS, not the raw physical PPS GPIO edge.
    #
    dwt_actual        = _field(r, "dwt_cycles_between_pps_vclock", "dwt_cycle_count_between_pps")
    dwt_expected      = _field(r, "dwt_expected_per_pps_vclock", "dwt_expected_per_pps")
    dwt_at_anchor     = _field(r, "dwt_at_pps_vclock", "dwt_cycle_count_at_pps")
    dwt_cycles_64     = _frag(r, "dwt_cycle_count_total")
    gnss_ns_64        = _frag(r, "gnss_ns")
    if any(v is not None for v in [dwt_actual, dwt_expected, dwt_at_anchor, dwt_cycles_64]):
        dwt_label_w = 12
        dwt_num_w = 20
        lines.append(
            f"DWT   "
            f"{'EXPECTED:':>{dwt_label_w}} {_comma_int(dwt_expected, dwt_num_w)}"
            f"    {'ACTUAL:':<{dwt_label_w}} {_comma_int(dwt_actual, dwt_num_w)}"
            f"    {'DWT@PPS/V:':>11} {_comma_int(dwt_at_anchor, 14)}"
        )
        dwt_ns_64 = _frag(r, "dwt_ns")
        counter32 = _field(r, "counter32_at_pps_vclock", "qtimer_at_pps")
        lines.append(
            f"      "
            f"{'DWT_CYCLES:':>{dwt_label_w}} {_comma_int(dwt_cycles_64, dwt_num_w)}"
            f"    {'DWT_NS:':<{dwt_label_w}} {_comma_int(dwt_ns_64, dwt_num_w)}"
            f"    {'CTR32:':>11} {_comma_int(counter32, 14)}"
        )
        lines.append(
            f"      "
            f"{'GNSS_NS:':>{dwt_label_w}} {_comma_int(gnss_ns_64, dwt_num_w)}"
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