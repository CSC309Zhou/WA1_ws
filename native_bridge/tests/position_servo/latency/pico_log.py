#!/usr/bin/env python3
"""Read existing Pico ServoJ logs and compare them with a PC latency report.

The remote program only opens and reads an existing log file. It never changes
Pico files, ROS parameters, services, or topics. No robot command is sent.
"""

import argparse
from bisect import bisect_left
import getpass
import json
import math
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys

from probe import distribution, first_crossing


DEFAULT_REPORT = Path(__file__).resolve().parent / 'results/Wrist_Z_L/latest.json'
DEFAULT_REMOTE_LOG = '/home/nav01/zj_humanoid/logs/run.log.1'
REMOTE_READER = r'''
import json, re, sys
start_ns, end_ns, path = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
pattern = re.compile(rb'\[(\d{10})\.(\d{1,9})\].*subscrib_arm_servoj \| (arm_type|joints):\s*([^\r\n\x1b]*)')
last_arm, last_arm_stamp = None, None
with open(path, 'rb') as log:
    for line in log:
        match = pattern.search(line)
        if not match:
            continue
        stamp = int(match.group(1)) * 1000000000 + int(match.group(2).ljust(9, b'0'))
        if match.group(3) == b'arm_type':
            last_arm = match.group(4).strip().decode(errors='replace')
            last_arm_stamp = stamp
            continue
        if not start_ns <= stamp <= end_ns:
            continue
        if last_arm != 'LeftArm' or last_arm_stamp is None or not 0 <= stamp - last_arm_stamp < 1000000:
            continue
        try:
            values = [float(item) for item in match.group(4).split()]
        except ValueError:
            continue
        if len(values) == 7:
            print(json.dumps({'pico_log_stamp_ns': stamp, 'arm_type': last_arm,
                              'joint': values}), flush=False)
'''


def validate_remote(value, label):
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', value):
        raise ValueError(f'{label} contains unsupported characters')
    return value


def fetch_pico_log(report, host, user, remote_log, ask_password):
    """Run a fixed, read-only Python filter on Pico and receive JSONL on PC."""
    host = validate_remote(host, 'host')
    user = validate_remote(user, 'user')
    if not re.fullmatch(r'/[A-Za-z0-9_./-]+', remote_log):
        raise ValueError('remote log path must be a plain absolute path')
    stamps = [row['robot_stamp_ns'] for row in report['feedback_samples']]
    if not stamps:
        raise ValueError('report has no Pico JointState stamps')
    start_ns = min(stamps) - 500_000_000
    end_ns = max(stamps) + 500_000_000
    remote_args = ['python3', '-', str(start_ns), str(end_ns), remote_log]
    base = ['ssh', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=5']
    target = f'{user}@{host}'
    if not ask_password:
        attempt = subprocess.run(base + ['-o', 'BatchMode=yes', target] + remote_args,
                                 input=REMOTE_READER, text=True, capture_output=True)
        if attempt.returncode == 0:
            return attempt.stdout
    if not shutil.which('sshpass'):
        raise RuntimeError('SSH key login failed; install sshpass or configure a Pico SSH key')
    password = getpass.getpass(f'Pico SSH password for {target}: ')
    import os
    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, password.encode() + b'\n')
        os.close(write_fd)
        write_fd = -1
        result = subprocess.run(
            ['sshpass', '-d', str(read_fd)] + base + [target] + remote_args,
            input=REMOTE_READER, text=True, capture_output=True, pass_fds=(read_fd,))
    finally:
        os.close(read_fd)
        if write_fd >= 0:
            os.close(write_fd)
    if result.returncode:
        raise RuntimeError(f'Pico read-only log query failed: {result.stderr.strip()}')
    return result.stdout


def parse_jsonl(source):
    rows = []
    for lineno, line in enumerate(source.splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
            if (not isinstance(row['pico_log_stamp_ns'], int) or
                    len(row['joint']) != 7 or
                    any(not math.isfinite(float(x)) for x in row['joint'])):
                raise ValueError('invalid row')
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'invalid Pico log JSONL row {lineno}: {exc}') from exc
        rows.append(row)
    return sorted(rows, key=lambda row: row['pico_log_stamp_ns'])


def moving_pairs(report, rows):
    """Use rounded log targets only in the unique middle 20–80% of motion."""
    start = report['start_rad']
    delta = report['delta_rad']
    moving = [row for row in report['published']
              if row['phase'] == 'move' and
              0.2 * report['duration_s'] <= row['scheduled_t_s'] <=
              0.8 * report['duration_s']]
    # Pico's debug log prints roughly six significant digits, so 2 µrad is a
    # formatting tolerance, not a timing or tracking tolerance.
    candidates = [(row['joint'][4], row) for row in rows]
    candidates.sort(key=lambda item: item[0])
    values = [item[0] for item in candidates]
    pairs = []
    used = set()
    for command in moving:
        value = command['target_rad']
        index = bisect_left(values, value)
        nearby = candidates[max(0, index - 2):min(len(candidates), index + 3)]
        if not nearby:
            continue
        logged_value, logged = min(nearby, key=lambda item: abs(item[0] - value))
        if abs(logged_value - value) > 2e-6 or logged['pico_log_stamp_ns'] in used:
            continue
        used.add(logged['pico_log_stamp_ns'])
        pairs.append((command, logged))
    pairs.sort(key=lambda pair: pair[0]['pc_publish_call_start_ns'])
    return moving, pairs


def analyze(report, rows):
    if report.get('mode') != 'EXECUTE' or report.get('joint_name') != 'Wrist_Z_L':
        raise ValueError('requires an executed Wrist_Z_L latency report')
    if not rows:
        raise ValueError('no matching Pico ServoJ log rows; check remote log rotation/path')
    start = report['start_rad']
    delta = report['delta_rad']
    pico_command = [{'pico_log_stamp_ns': row['pico_log_stamp_ns'],
                     'target_rad': row['joint'][4]} for row in rows]
    state = [{'robot_stamp_ns': row['robot_stamp_ns'],
              'position_rad': row['position_rad']} for row in report['feedback_samples']]
    pc_command = [{'pc_publish_call_start_ns': row['pc_publish_call_start_ns'],
                   'target_rad': row['target_rad']} for row in report['published']]
    pc_state = [{'pc_monotonic_ns': row['pc_monotonic_ns'],
                 'position_rad': row['position_rad']} for row in report['feedback_samples']]
    crossings = {}
    for step in range(1, 10):
        level = step / 10
        command_ns = first_crossing(pico_command, 'pico_log_stamp_ns',
                                    'target_rad', start, delta, level)
        state_ns = first_crossing(state, 'robot_stamp_ns',
                                  'position_rad', start, delta, level)
        if command_ns is None or state_ns is None:
            continue
        inside_ms = (state_ns - command_ns) / 1e6
        pc_command_ns = first_crossing(pc_command, 'pc_publish_call_start_ns',
                                       'target_rad', start, delta, level)
        pc_state_ns = first_crossing(pc_state, 'pc_monotonic_ns',
                                     'position_rad', start, delta, level)
        pc_ms = ((pc_state_ns - pc_command_ns) / 1e6
                 if pc_command_ns is not None and pc_state_ns is not None else None)
        crossings[str(level)] = {
            'pico_command_log_to_joint_state_stamp_ms': inside_ms,
            'pc_command_to_pc_feedback_apparent_ms': pc_ms,
            'outside_pico_combined_ms': pc_ms - inside_ms if pc_ms is not None else None,
        }
    moving, pairs = moving_pairs(report, rows)
    if any(b['pico_log_stamp_ns'] <= a['pico_log_stamp_ns']
           for (_, a), (_, b) in zip(pairs, pairs[1:])):
        raise ValueError('Pico log command order disagrees with PC command order')
    spacings = [(b['pico_log_stamp_ns'] - a['pico_log_stamp_ns']) / 1e6
                for a, b in zip(rows, rows[1:])]
    # This offset contains the unsynchronized PC/Pico clock difference AND
    # network delay. Only its variation across paired commands is meaningful.
    diagnostics = {row['target_bits']: row for row in report['command_timing']}
    raw_offsets = [(pico['pico_log_stamp_ns'] -
                    diagnostics[pc['target_bits']]['dds_source_timestamp_ns']) / 1e6
                   for pc, pico in pairs if pc['target_bits'] in diagnostics]
    offset_median = statistics.median(raw_offsets) if raw_offsets else None
    outside = [row['outside_pico_combined_ms'] for row in crossings.values()
               if row['outside_pico_combined_ms'] is not None]
    return {
        'report_move_start_pc_monotonic_ns': report['move_start_pc_monotonic_ns'],
        'report_first_robot_stamp_ns': report['feedback_samples'][0]['robot_stamp_ns'],
        'clock_assumption': ('Pico ROS log timestamps and JointState.header.stamp share '
                             'the Pico ROS wall clock; PC and Pico clocks are not synchronized.'),
        'pico_log_rows': len(rows),
        'pc_moving_middle_commands': len(moving),
        'uniquely_paired_middle_commands': len(pairs),
        'pico_log_interval_ms': distribution(spacings),
        'pico_vs_pc_clock_plus_transport_offset_ms': offset_median,
        'paired_offset_jitter_about_median_ms': distribution(
            [offset - offset_median for offset in raw_offsets]) if raw_offsets else None,
        'threshold_crossings': crossings,
        'outside_pico_combined_across_thresholds_ms': distribution(outside),
        'interpretation': ('Pico callback log → Pico JointState stamp includes internal control '
                           'and feedback sampling, not physical ground truth. The outside-Pico '
                           'value combines PC processing and both transport directions; it '
                           'does not separately measure either one-way network delay.'),
    }


def plot(report, rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    origin = rows[0]['pico_log_stamp_ns']
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot([(row['pico_log_stamp_ns'] - origin) / 1e9 for row in rows],
            [row['joint'][4] for row in rows], label='Pico ServoJ callback log target')
    states = [row for row in report['feedback_samples']
              if origin - 300_000_000 <= row['robot_stamp_ns'] <=
              rows[-1]['pico_log_stamp_ns'] + 500_000_000]
    ax.plot([(row['robot_stamp_ns'] - origin) / 1e9 for row in states],
            [row['position_rad'] for row in states], label='Pico JointState Header.stamp')
    ax.set(title='Wrist_Z_L on the Pico ROS time axis',
           xlabel='Pico ROS time relative to first logged ServoJ command (s)',
           ylabel='Joint position (rad)')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, default=DEFAULT_REPORT)
    parser.add_argument('--fetch', action='store_true',
                        help='Read existing Pico log over SSH; prompts for password if needed')
    parser.add_argument('--local-log', type=Path,
                        help='Previously fetched Pico JSONL; used when --fetch is omitted')
    parser.add_argument('--host', default='192.168.217.66')
    parser.add_argument('--user', default='nav01')
    parser.add_argument('--remote-log', default=DEFAULT_REMOTE_LOG)
    args = parser.parse_args()
    if args.fetch == bool(args.local_log):
        parser.error('choose exactly one of --fetch or --local-log')
    report = json.loads(args.report.read_text())
    stem = args.report.with_suffix('')
    log_path = Path(str(stem) + '_pico_log.jsonl')
    summary_path = Path(str(stem) + '_pico_analysis.json')
    plot_path = Path(str(stem) + '_pico.png')
    if args.fetch:
        text = fetch_pico_log(report, args.host, args.user, args.remote_log, False)
        rows = parse_jsonl(text)
        if rows:
            log_path.write_text(text)
    else:
        rows = parse_jsonl(args.local_log.read_text())
    summary = analyze(report, rows)
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    plot(report, rows, plot_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f'Pico analysis: {summary_path}\nPico plot: {plot_path}')
    if args.fetch:
        print(f'Filtered Pico log copy on PC: {log_path}')


if __name__ == '__main__':
    raise SystemExit(main())
