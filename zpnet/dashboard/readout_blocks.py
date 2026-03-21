"""
ZPNet Dashboard Readout Blocks — v10 GNSS_RAW Edition

All readouts are derived directly from the authoritative SYSTEM snapshot
or from direct process queries.

v10 changes:
  • GNSS_RAW synthetic clock domain added to tau, comparison, and
    prediction readouts.  GNSS_RAW is a Pi-side clock synthesized
    from the GF-8802's clock_drift_ppb (TPS1), representing the
    environmental forcing function on an undisciplined crystal.
  • _CLOCK_DOMAINS now includes five domains: GNSS, DWT, OCXO1,
    OCXO2, GNSS_RAW.

v9 changes:
  • clocks_prediction_readout() displays Teensy prediction statistics
    (pred_stddev, pred_n, pred_residual) for DWT, OCXO1, and OCXO2.
  • Dual OCXO.  _CLOCK_DOMAINS includes OCXO1 and OCXO2.
    All tau, prediction, and comparison readouts show four domains.
    Baseline comparison handles ocxo1 and ocxo2 PPB keys.

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

def get_teensy_clocks_report() -> dict:
    return send_command(machine="TEENSY", subsystem="CLOCKS", command="REPORT")["payload"]

def get_pi_clocks_report() -> dict:
    return send_command(machine="PI", subsystem="CLOCKS", command="REPORT")["payload"]

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
# CLOCKS — shared helpers
# ---------------------------------------------------------------------

# Five clock domains: GNSS (reference), GNSS_RAW (synthetic), DWT, OCXO1, OCXO2
_CLOCK_DOMAINS = [
    ("GNSS", "gnss"),
    ("GN_RAW", "gnss_raw"),
    ("DWT", "dwt"),
    ("OCXO1", "ocxo1"),
    ("OCXO2", "ocxo2"),
]

# Teensy-owned clock domains (have prediction stats)
_TEENSY_CLOCK_DOMAINS = [
    ("DWT", "dwt"),
    ("OCXO1", "ocxo1"),
    ("OCXO2", "ocxo2"),
]


def _get_clocks_baseline() -> dict | None:
    """Fetch baseline info from CLOCKS subsystem. Returns payload or None."""
    try:
        resp = send_command(machine="PI", subsystem="CLOCKS", command="BASELINE_INFO")
        if resp.get("success"):
            p = resp.get("payload", {})
            if p.get("baseline_set"):
                return p
    except Exception:
        pass
    return None


def _clocks_header() -> tuple[dict | None, str]:
    """Fetch clocks report; return (report_dict, header_line) or (None, status)."""
    p = get_pi_clocks_report()
    state = p.get("report", {}).get("campaign_state") or p.get("campaign_state", "IDLE")
    if state != "STARTED":
        return None, f"CLOCKS: {state}"
    r = p["report"]
    campaign = r.get("campaign", "?")
    elapsed = r.get("campaign_elapsed", "00:00:00")
    n = r.get("pps_count", 0)
    return r, f"CLOCKS: {campaign} {elapsed} n={n}"


# ---------------------------------------------------------------------
# CLOCKS 1/5 — Tau
# ---------------------------------------------------------------------

def clocks_tau_readout() -> Generator[str, None, None]:
    r, hdr = _clocks_header()
    yield hdr
    if r is None:
        return

    yield f"{'CLK':<6} {'TAU':>16} {'PPB':>10}"
    yield f"{'GNSS':<6} {'1.0000000000':>16} {'0.000':>10}"

    for name, key in _CLOCK_DOMAINS[1:]:
        blk = r.get(key, {})
        tau = blk.get("tau")
        ppb = blk.get("ppb")
        if tau is not None and ppb is not None:
            yield f"{name:<6} {tau:>16.10f} {ppb:>10.3f}"
        else:
            yield f"{name:<6} {'---':>16} {'---':>10}"


# ---------------------------------------------------------------------
# CLOCKS 2/5 — Prediction Statistics (v8, replaces Welford readout)
# ---------------------------------------------------------------------

def clocks_prediction_readout() -> Generator[str, None, None]:
    """
    Display Teensy prediction statistics for DWT, OCXO1, and OCXO2.

    The prediction residual stddev is the authoritative interpolation
    uncertainty metric — "how wrong is my best estimate of this
    second's crystal rate?"

    For DWT this is typically ~3-4 cycles (3-4 ns).
    For OCXO1/OCXO2 this is typically ~1 tick (100 ns) when servo-locked.

    GNSS is phase-coherent by definition (residual always 0), so
    we show the stream health canary instead.

    GNSS_RAW shows drift_ppb (the instantaneous GF-8802 clock drift)
    instead of prediction stats (it has no Teensy predictor).
    """
    r, hdr = _clocks_header()
    yield hdr
    if r is None:
        return

    yield f"{'CLK':<6} {'RES':>6} {'MEAN':>8} {'SD':>8} {'N':>6}"

    # GNSS — stream health canary (no prediction needed)
    gnss_blk = r.get("gnss", {})
    gnss_res = gnss_blk.get("pps_residual", 0)
    yield f"{'GNSS':<6} {gnss_res:>6} {'---':>8} {'---':>8} {'---':>6}"

    # GNSS_RAW — Pi-side Welford on drift_ppb
    gnss_raw_blk = r.get("gnss_raw", {})
    gnss_raw_n = gnss_raw_blk.get("pred_n")
    if gnss_raw_n is not None and gnss_raw_n > 0:
        gr_res = gnss_raw_blk.get("pred_residual", 0.0)
        gr_mean = gnss_raw_blk.get("pred_mean", 0.0)
        gr_sd = gnss_raw_blk.get("pred_stddev", 0.0)
        yield f"{'GN_RAW':<6} {gr_res:>6.1f} {gr_mean:>8.3f} {gr_sd:>8.3f} {gnss_raw_n:>6}"
    else:
        yield f"{'GN_RAW':<6} {'---':>6} {'---':>8} {'---':>8} {'---':>6}"

    # DWT, OCXO1, OCXO2 — Teensy prediction stats
    for name, key in _TEENSY_CLOCK_DOMAINS:
        blk = r.get(key, {})
        pred_n = blk.get("pred_n")

        if pred_n is not None and pred_n > 0:
            res    = blk.get("pred_residual", 0)
            mean   = blk.get("pred_mean", 0.0)
            stddev = blk.get("pred_stddev", 0.0)
            yield f"{name:<6} {res:>6} {mean:>8.3f} {stddev:>8.3f} {pred_n:>6}"
        else:
            # Not enough history yet (need 3 deltas for first scored prediction)
            yield f"{name:<6} {'---':>6} {'---':>8} {'---':>8} {'---':>6}"


# ---------------------------------------------------------------------
# CLOCKS 3/5 — Comparison vs baseline
# ---------------------------------------------------------------------

def clocks_comparison_readout() -> Generator[str, None, None]:
    r, hdr = _clocks_header()
    yield hdr
    if r is None:
        return

    baseline = _get_clocks_baseline()
    if baseline is None:
        yield "NO BASELINE SET"
        yield "USE: .pc clocks SET_BASELINE id=<N>"
        return

    baseline_ppb = baseline.get("baseline_ppb", {})
    baseline_id = baseline.get("baseline_id", "?")
    baseline_campaign = baseline.get("baseline_campaign", "?")

    yield f"BASELINE: {baseline_campaign} (#{baseline_id})"
    yield f"{'CLK':<6} {'BASE':>10} {'NOW':>10} {'DELTA':>10}"

    for name, key in _CLOCK_DOMAINS:
        blk = r.get(key, {})
        base_ppb = baseline_ppb.get(key)
        now_ppb = blk.get("ppb")

        if base_ppb is not None and now_ppb is not None:
            delta = now_ppb - base_ppb
            yield f"{name:<6} {base_ppb:>10.3f} {now_ppb:>10.3f} {delta:>+10.3f}"
        else:
            yield f"{name:<6} {'---':>10} {'---':>10} {'---':>10}"


# ---------------------------------------------------------------------
# CLOCKS 4/5 — OCXO Servo Status
# ---------------------------------------------------------------------

def clocks_servo_readout() -> Generator[str, None, None]:
    """
    Display OCXO1 and OCXO2 servo/DAC state from the Teensy CLOCKS report.
    """
    try:
        t = get_teensy_clocks_report()
    except Exception:
        yield "SERVO: UNAVAILABLE"
        return

    cal = t.get("calibrate_ocxo", "OFF")
    yield f"SERVO: {cal if cal and cal != 'OFF' else 'IDLE'}"

    for label, dac_key, adj_key, res_key in [
        ("OCXO1", "ocxo1_dac", "ocxo1_servo_adjustments", "isr_residual_ocxo1"),
        ("OCXO2", "ocxo2_dac", "ocxo2_servo_adjustments", "isr_residual_ocxo2"),
    ]:
        dac = t.get(dac_key)
        adj = t.get(adj_key, 0)
        res = t.get(res_key)

        if dac is not None:
            res_str = f"{res:+d}" if res is not None else "---"
            yield f"{label:<6} DAC={dac:>10.3f}  ADJ={adj:>4}  RES={res_str}"
        else:
            yield f"{label:<6} ---"


# ---------------------------------------------------------------------
# CLOCKS 5/5 — GNSS_RAW detail
# ---------------------------------------------------------------------

def clocks_gnss_raw_readout() -> Generator[str, None, None]:
    """
    Display GNSS_RAW synthetic clock detail from the Pi CLOCKS report.
    Shows the GF-8802 internal TCXO behavior as a clock domain.
    """
    r, hdr = _clocks_header()
    yield hdr
    if r is None:
        return

    yield "GNSS RAW CLOCK (GF-8802 TCXO drift)"

    blk = r.get("gnss_raw", {})
    tau = blk.get("tau")
    ppb = blk.get("ppb")
    drift = blk.get("drift_ppb")

    if tau is not None:
        yield f"  TAU:       {tau:.12f}"
    else:
        yield f"  TAU:       ---"

    if ppb is not None:
        yield f"  PPB:       {ppb:+.3f}"
    else:
        yield f"  PPB:       ---"

    if drift is not None:
        yield f"  DRIFT NOW: {drift:+.3f} ppb"
    else:
        yield f"  DRIFT NOW: ---"


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

"""
Teensy Subsystem Health Readout

Renders independent Pi-side health judgment for:
  • Process (RPC pipeline invariants)
  • Transport (TX/RX accounting, framing errors)
  • Payload (allocator health, leak detection)
  • Memory (heap growth, stack usage, fragmentation)

Health levels:
  NOMINAL  — all invariants hold, counters are healthy
  ANOMALY  — something unexpected but still livable
  FAIL     — something is broken, investigate immediately

These judgments are INDEPENDENT of Teensy-reported health_state.
The Pi does its own math on the raw numbers.
"""

from typing import Generator

# ============================================================================
# Health classification
# ============================================================================

NOMINAL = "NOMINAL"
ANOMALY = "ANOMALY"
FAIL = "FAIL"

def _worst(*levels: str) -> str:
    """Return the worst health level from a set."""
    if FAIL in levels:
        return FAIL
    if ANOMALY in levels:
        return ANOMALY
    return NOMINAL

# ============================================================================
# Process health analysis
# ============================================================================

def _assess_process(proc: dict) -> tuple[str, list[str]]:
    """
    Analyze RPC pipeline invariants.

    Invariants:
      received == routed + err_missing + err_unknown_subsys + err_unknown_command
      handler_invoked == handler_completed  (±1 for in-flight query)
      handler_completed == response_sent    (±1 for in-flight query)
    """
    findings = []

    received = proc.get("rpc_received", 0)
    routed = proc.get("rpc_routed", 0)
    invoked = proc.get("rpc_handler_invoked", 0)
    completed = proc.get("rpc_handler_completed", 0)
    sent = proc.get("rpc_response_sent", 0)

    err_missing = proc.get("rpc_err_missing_fields", 0)
    err_subsys = proc.get("rpc_err_unknown_subsys", 0)
    err_cmd = proc.get("rpc_err_unknown_command", 0)

    # Accounting invariant
    expected_received = routed + err_missing + err_subsys + err_cmd
    if received != expected_received:
        findings.append(f"RECEIVED MISMATCH: {received} != {expected_received}")

    # Handler invariant (allow ±1 for in-flight query)
    if abs(invoked - completed) > 1:
        findings.append(f"STUCK HANDLER: invoked={invoked} completed={completed}")

    # Response invariant (allow ±1 for in-flight query)
    if abs(completed - sent) > 1:
        findings.append(f"LOST RESPONSE: completed={completed} sent={sent}")

    # Error rates — anomaly if any errors exist
    total_errors = err_missing + err_subsys + err_cmd
    if total_errors > 0:
        findings.append(f"RPC ERRORS: {total_errors} total")

    if any("MISMATCH" in f or "STUCK" in f or "LOST" in f for f in findings):
        return FAIL, findings
    elif findings:
        return ANOMALY, findings

    return NOMINAL, [f"RPC {received} OK, PS {proc.get('ps_dispatched', 0)} dispatched"]

# ============================================================================
# Transport health analysis
# ============================================================================

def _assess_transport(tx: dict) -> tuple[str, list[str]]:
    """
    Analyze transport TX/RX accounting and framing health.
    """
    findings = []

    enqueued = tx.get("tx_jobs_enqueued", 0)
    sent = tx.get("tx_jobs_sent", 0)
    bytes_enq = tx.get("tx_bytes_enqueued", 0)
    bytes_sent = tx.get("tx_bytes_sent", 0)
    rr_drops = tx.get("tx_rr_drop_count", 0)
    alloc_fail = tx.get("tx_arena_alloc_fail", 0)
    job_count = tx.get("tx_job_count", 0)

    bad_stx = tx.get("rx_bad_stx", 0)
    bad_etx = tx.get("rx_bad_etx", 0)
    overflow = tx.get("rx_len_overflow", 0)
    overlap = tx.get("rx_overlap", 0)

    # TX job accounting
    pending = enqueued - sent
    if pending > 10:
        findings.append(f"TX BACKLOG: {pending} jobs pending")

    # Byte-perfect accounting
    if bytes_enq != bytes_sent and job_count == 0:
        findings.append(f"TX BYTE MISMATCH: enqueued={bytes_enq} sent={bytes_sent}")

    # RR drops
    if rr_drops > 0:
        findings.append(f"RR DROPS: {rr_drops}")

    # Arena exhaustion
    if alloc_fail > 0:
        findings.append(f"ARENA ALLOC FAIL: {alloc_fail}")

    # Framing errors
    framing_errors = bad_stx + bad_etx + overflow
    if framing_errors > 0:
        findings.append(f"FRAMING ERRORS: stx={bad_stx} etx={bad_etx} overflow={overflow}")

    # Overlap (new frame before previous completed)
    if overlap > 5:
        findings.append(f"RX OVERLAP: {overlap}")

    if any("MISMATCH" in f or "DROPS" in f or "ALLOC FAIL" in f for f in findings):
        return FAIL, findings
    elif findings:
        return ANOMALY, findings

    return NOMINAL, [f"TX {sent} jobs, RX clean"]

# ============================================================================
# Payload health analysis
# ============================================================================

def _assess_payload(pl: dict) -> tuple[str, list[str]]:
    """
    Analyze payload allocator health.
    """
    findings = []

    constructed = pl.get("payload_instances_constructed", 0)
    destroyed = pl.get("payload_instances_destroyed", 0)
    alive = pl.get("payload_alive_now", 0)
    alive_hwm = pl.get("payload_alive_high_water", 0)
    alloc_fail = pl.get("payload_arena_alloc_fail", 0)
    overflow = pl.get("payload_entry_overflow", 0)

    # Leak detection
    expected_alive = constructed - destroyed
    if expected_alive != alive:
        findings.append(f"LEAK: constructed-destroyed={expected_alive} but alive={alive}")

    # Alive count sanity
    if alive > 20:
        findings.append(f"HIGH ALIVE: {alive} payloads (hwm={alive_hwm})")

    # Allocation failures
    if alloc_fail > 0:
        findings.append(f"ARENA ALLOC FAIL: {alloc_fail}")

    # Entry overflow
    if overflow > 0:
        findings.append(f"ENTRY OVERFLOW: {overflow}")

    if any("LEAK" in f or "ALLOC FAIL" in f for f in findings):
        return FAIL, findings
    elif findings:
        return ANOMALY, findings

    return NOMINAL, [f"{constructed} constructed, {alive} alive"]

# ============================================================================
# Memory health analysis
# ============================================================================

def _assess_memory(mem: dict) -> tuple[str, list[str]]:
    """
    Analyze Teensy memory health.
    """
    findings = []

    heap_growing = mem.get("heap_growing", False)
    stack_pct = mem.get("stack_usage_pct", 0)
    heap_free = mem.get("heap_free_above", 0)
    heap_total = mem.get("heap_total", 1)
    heap_frag = mem.get("heap_fragmentation_pct", 0)
    heap_arena = mem.get("heap_arena", 0)
    heap_used = mem.get("heap_used", 0)
    heap_free_above = mem.get("heap_free_above", 0)

    # Heap growth (leak indicator)
    if heap_growing:
        findings.append("HEAP GROWING (possible leak)")

    # Stack usage
    if stack_pct > 75:
        findings.append(f"STACK CRITICAL: {stack_pct}% used")
    elif stack_pct > 50:
        findings.append(f"STACK ELEVATED: {stack_pct}% used")

    # Heap headroom
    heap_used_pct = (heap_arena * 100) // heap_total if heap_total > 0 else 0
    if heap_used_pct > 80:
        findings.append(f"HEAP CRITICAL: {heap_used_pct}% committed")
    elif heap_used_pct > 50:
        findings.append(f"HEAP ELEVATED: {heap_used_pct}% committed")

    # Fragmentation (only matters if heap is large)
    if heap_frag > 80 and heap_arena > 16384 and heap_free_above < 32768:
        findings.append(f"FRAGMENTATION: {heap_frag}% of {heap_arena} bytes")

    if any("CRITICAL" in f or "GROWING" in f for f in findings):
        return FAIL, findings
    elif findings:
        return ANOMALY, findings

    stack_hw = mem.get("stack_high_water", 0)
    return NOMINAL, [f"stack={stack_hw}B heap={heap_used}/{heap_arena}B"]

# ============================================================================
# Public readout
# ============================================================================

def teensy_metrics_readout() -> Generator[str, None, None]:

    snapshot = get_system_snapshot()

    proc_data = snapshot.get("process", {})
    tx_data = snapshot.get("transport", {})
    pl_data = snapshot.get("payload", {})
    mem_data = snapshot.get("memory", {})

    proc_health, proc_notes = _assess_process(proc_data)
    tx_health, tx_notes = _assess_transport(tx_data)
    pl_health, pl_notes = _assess_payload(pl_data)
    mem_health, mem_notes = _assess_memory(mem_data)

    overall = _worst(proc_health, tx_health, pl_health, mem_health)

    yield f"TEENSY METRICS: {overall}"
    yield ""

    yield f"  PROCESS:   {proc_health}"
    for note in proc_notes:
        yield f"    {note}"

    yield f"  TRANSPORT: {tx_health}"
    for note in tx_notes:
        yield f"    {note}"

    yield f"  PAYLOAD:   {pl_health}"
    for note in pl_notes:
        yield f"    {note}"

    yield f"  MEMORY:    {mem_health}"
    for note in mem_notes:
        yield f"    {note}"