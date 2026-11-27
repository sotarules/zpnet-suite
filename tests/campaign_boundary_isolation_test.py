"""Regress campaign-only-bookend boundaries without operating field hardware.

Run on the Pi through the standard test launcher:
    zt campaign_boundary_isolation_test

This test reads repository source and compiles a handful of isolated Python
functions into a fake read-only environment. It never sends RPCs, writes DB
rows, or opens an I2C device. All reporting goes to STDOUT.
"""
from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def source(path):
    return (ROOT / path).read_text()


def fn(tree, name):
    return next(node for node in tree.body
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name == name)


def calls(node):
    return {item.func.id if isinstance(item.func, ast.Name)
            else item.func.attr if isinstance(item.func, ast.Attribute)
            else "<dynamic>"
            for item in ast.walk(node) if isinstance(item, ast.Call)}


def literal_strings(node):
    return {item.value for item in ast.walk(node)
            if isinstance(item, ast.Constant) and isinstance(item.value, str)}


def isolated_function(tree, name, namespace):
    node = fn(tree, name)
    future = ast.ImportFrom(module='__future__',
                            names=[ast.alias(name='annotations')], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, node], type_ignores=[]))
    exec(compile(module, '<campaign-isolation-test>', 'exec'), namespace)
    return namespace[name]


def cpp_body(text, signature):
    start = text.index(signature)
    opening = text.index('{', start)
    nesting = 1
    for index in range(opening + 1, len(text)):
        nesting += (text[index] == '{') - (text[index] == '}')
        if nesting == 0:
            return text[opening + 1:index]
    raise AssertionError(f'unterminated firmware function: {signature}')


def test_boundaries():
    core = ast.parse(source('zpnet/processes/clocks/core.py'))
    start = fn(core, 'cmd_start')
    stop = fn(core, 'cmd_stop')
    for name, node in (('CLOCKS.START', start), ('CLOCKS.STOP', stop)):
        invoked = calls(node)
        assert '_ensure_system_location' not in invoked, name
        assert '_request_teensy_recover_abort_best_effort' not in invoked, name
        assert 'cmd_servos' not in invoked, name
        assert '_dac_set_target_explicit' not in invoked, name
        assert '_dac_reset_statistics' not in invoked, name
    assert '_request_teensy_stop_best_effort' not in calls(start)
    assert '_request_teensy_stop_best_effort' not in calls(stop)
    assert '_confirm_teensy_stopped_for_cold_start' in calls(start)

    loc = fn(core, '_campaign_location_snapshot')
    assert calls(loc) == {'_fetch_system_report', 'get'}, 'campaign location must be read-only'
    example = {'location': {'current_location': 'Pinos', 'receiver_mode': 'TO'},
               'gnss': {'freq_mode_name': 'LOCKED'}}
    read = isolated_function(core, '_campaign_location_snapshot',
                             {'_fetch_system_report': lambda: example})
    assert read() == {'current_location': 'Pinos',
                      'verified_pos_mode_name': 'TO',
                      'freq_mode_name': 'LOCKED'}

    beta = source('pnc/firmware/teensy/process_clocks_beta.cpp')
    stopped = cpp_body(beta, 'static FLASHMEM Payload cmd_stop(')
    for forbidden in ('clocks_ppb_restore_protocol_clear(',
                      'interrupt_smartzero_abort(',
                      'recover_lifecycle_abort(',
                      'clocks_fragment_recover_reset_publication_custody(',
                      'clocks_alpha_instrument_stats_reset(',
                      'clocks_alpha_smartzero_pending_clear('):
        assert forbidden not in stopped, f'firmware STOP invokes {forbidden}'
    assert 'request_stop = true;' in stopped


def test_cold_start_gate():
    core = ast.parse(source('zpnet/processes/clocks/core.py'))
    class Clock:
        def __init__(self):
            self.now = 0.0
        def monotonic(self):
            self.now += 0.05
            return self.now
        def sleep(self, seconds):
            self.now += seconds
    clock = Clock()
    reports = []
    def read_recovery(**kwargs):
        assert kwargs == {'machine': 'TEENSY', 'subsystem': 'CLOCKS',
                          'command': 'REPORT_RECOVERY'}
        reports.append(1)
        finished = len(reports) == 3
        return {'success': True, 'payload': {
            'campaign_state': 'STOPPED' if finished else 'STARTED',
            'restore_court_ready': finished,
        }}
    gate = isolated_function(core, '_confirm_teensy_stopped_for_cold_start',
                             {'time': clock, 'send_command': read_recovery})
    gate()
    assert len(reports) == 3, 'cold START must wait for actual STOP commit'


def test_servo():
    core = ast.parse(source('zpnet/processes/clocks/core.py'))
    inputs = fn(core, '_dac_servo_inputs')
    assert [arg.arg for arg in inputs.args.args] == ['clocks', 'lane_name']
    assert not any('camp' in s.lower() for s in literal_strings(inputs))
    process = fn(core, '_dac_process_completed_row')
    assert [arg.arg for arg in process.args.args] == ['teensy_clocks', 'sequence']
    assert '"CAMP"' not in ast.get_source_segment(
        source('zpnet/processes/clocks/core.py'), process)
    command = ast.get_source_segment(source('zpnet/processes/clocks/core.py'),
                                     fn(core, 'cmd_servos'))
    assert 'CAMP servo was retired' in command

    ns = {'math': math, 'SERVO_TOTAL_MIN_SAMPLES': 10}
    calculate = isolated_function(core, '_dac_servo_inputs', ns)
    data = {'stats': {'ocxo1': {'ppb_buckets': {'10_min': 1.25, 'total': 1.1}},
                      'ocxo1_tau_state': {'sample_count': 700}},
            'raw_cycles': {'vclock': {'valid': True, 'observed_cycles': 10000000},
                           'ocxo1': {'valid': True, 'observed_cycles': 9999999}}}
    result = calculate(data, 'ocxo1')
    assert result['10_min_valid'] and result['total_valid']
    assert result['10_min_ppb'] == 1.25
    assert 'camp_ppb' not in result


def test_dac_resume():
    core = ast.parse(source('zpnet/processes/clocks/core.py'))
    choose = isolated_function(core, '_dac_resume_target', {})
    assert choose(20223.75, 20224, True) == 20223.75  # keep fractional dither
    assert choose(20223.75, 20223, False) == 20223.0  # powered static hardware wins
    assert choose(20223.75, 21400, True) == 21400.0  # stale saved target loses
    assert choose(20223.75, 0, False) == 20223.75  # truly reset device
    restore = fn(core, '_dac_restore_control_from_clocks')
    text = ast.get_source_segment(source('zpnet/processes/clocks/core.py'), restore)
    assert '_ad5693r_read_input(lane.address)' in text
    assert 'elif not dither and _dac_rounded_hw_code(saved_target) != 0' in text
    assert 'if observed_code:' in text


def test_photons():
    core = ast.parse(source('zpnet/processes/photons/core.py'))
    for name in ('cmd_start', 'cmd_stop', 'cmd_flash_cut'):
        node = fn(core, name)
        invoked = calls(node)
        assert not (invoked & {'set_code', 'manual', 'start', 'stop',
                               'cmd_stats_reset', 'cmd_set_dac',
                               'cmd_servo_start', 'cmd_servo_stop'}), name
    servo = fn(core, '_laser_servo_hold_reason')
    assert '_active_campaign' not in ast.get_source_segment(
        source('zpnet/processes/photons/core.py'), servo)
    firmware = source('pnc/firmware/teensy/process_photons.cpp')
    for signature in ('static FLASHMEM Payload cmd_start(',
                      'static FLASHMEM Payload cmd_stop(',
                      'static FLASHMEM Payload photons_flash_cut_command_body('):
        body = cpp_body(firmware, signature)
        for forbidden in ('photons_instrument_statistics_reset_commit(',
                          'photons_stats_reset_commit_after_publish(',
                          'photons_laser_initialize_hardware('):
            assert forbidden not in body, signature


def main(argv):
    assert len(argv) == 1, 'usage: zt campaign_boundary_isolation_test'
    test_boundaries()
    test_cold_start_gate()
    test_servo()
    test_dac_resume()
    test_photons()
    print('CAMPAIGN BOUNDARY ISOLATION: GNSS read-only; CLOCKS STOP '
          'no recovery abort; always-on servos; DAC resume; PHOTONS '
          'bookends PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
