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

"""
readout_teensy_metrics.py — Teensy Subsystem Health Readout

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

    Key checks:
      tx_jobs_enqueued == tx_jobs_sent  (no stuck jobs)
      tx_bytes_enqueued == tx_bytes_sent (byte-perfect accounting)
      tx_rr_drop_count == 0 (no dropped request/responses)
      tx_arena_alloc_fail == 0 (no arena exhaustion)
      rx framing errors are zero or frozen
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
    elif pending > 0 and job_count == 0:
        # jobs_enqueued > jobs_sent but queue is empty = counter mismatch
        pass  # Transient, ignore

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

    Key checks:
      alive_now should be small and stable (5 is normal)
      arena_alloc_fail == 0
      entry_overflow == 0
      constructed - destroyed == alive_now (no leaks)
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

    Key checks:
      heap_growing == false (no monotonic sbrk advance)
      stack_usage_pct < 50% (plenty of headroom)
      heap_free_above is large (not approaching ceiling)
      heap_fragmentation_pct is reasonable
    """
    findings = []

    heap_growing = mem.get("heap_growing", False)
    stack_pct = mem.get("stack_usage_pct", 0)
    heap_free = mem.get("heap_free_above", 0)
    heap_total = mem.get("heap_total", 1)
    heap_frag = mem.get("heap_fragmentation_pct", 0)
    heap_arena = mem.get("heap_arena", 0)
    heap_used = mem.get("heap_used", 0)

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
    if heap_frag > 80 and heap_arena > 16384:
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
