"""PC-side timestamp capture and analysis for the Wrist_Z_L ServoJ probe."""

import json
from pathlib import Path
import statistics
import struct
import threading
import time


COMMAND_TOPIC = '/upperlimb/bridge/servoj_left_arm_timing'
RECEIVE_TOPIC = '/upperlimb/bridge/joint_receive_time'


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def bits(value):
    """Exact IEEE-754 double bits used to match moving command samples."""
    return f'{struct.unpack("=Q", struct.pack("=d", value))[0]:016x}'


def distribution(values):
    if not values:
        return None
    values = sorted(values)

    def percentile(q):
        x = (len(values) - 1) * q
        lo = int(x)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (x - lo)

    return {'n': len(values), 'median': statistics.median(values),
            'p95': percentile(0.95), 'p99': percentile(0.99),
            'min': values[0], 'max': values[-1]}


def first_crossing(rows, time_key, value_key, start, delta, fraction):
    """Find the first measured crossing, interpolating between adjacent samples."""
    level = abs(delta) * fraction
    direction = 1 if delta > 0 else -1
    ordered = sorted(rows, key=lambda row: row[time_key])
    for before, after in zip(ordered, ordered[1:]):
        a = direction * (before[value_key] - start)
        b = direction * (after[value_key] - start)
        if a < level <= b and b > a:
            weight = (level - a) / (b - a)
            return before[time_key] + weight * (after[time_key] - before[time_key])
    return None


class LatencyCapture:
    COMMAND_TOPIC = COMMAND_TOPIC
    RECEIVE_TOPIC = RECEIVE_TOPIC
    bits = staticmethod(bits)

    def __init__(self):
        self.lock = threading.Lock()
        self.commands = []
        self.receives = []
        self.bad_messages = 0

    def on_command_timing(self, msg):
        callback_ns = time.monotonic_ns()
        try:
            row = json.loads(msg.data)
            if (not isinstance(row['target_bits'], str) or
                    len(row['target_bits']) != 16 or
                    any(c not in '0123456789abcdef' for c in row['target_bits']) or
                    any(not isinstance(row[key], int) for key in (
                        'callback_entry_ns', 'ros1_publish_start_ns',
                        'ros1_publish_return_ns'))):
                raise ValueError('invalid timing record')
            row['pc_diagnostic_callback_ns'] = callback_ns
        except (KeyError, TypeError, ValueError):
            with self.lock:
                self.bad_messages += 1
            return
        with self.lock:
            self.commands.append(row)

    def on_joint_receive(self, msg):
        row = {'robot_stamp_ns': stamp_ns(msg.header.stamp),
               'bridge_ros1_callback_ns': stamp_ns(msg.time_ref),
               'pc_timing_callback_ns': time.monotonic_ns()}
        with self.lock:
            self.receives.append(row)

    def snapshot(self):
        with self.lock:
            return {'command_timing': list(self.commands),
                    'joint_receive_timing': list(self.receives),
                    'bad_timing_messages': self.bad_messages}

    @staticmethod
    def analyze(report, spec):
        target_key = spec.key('target')
        position_key = spec.key('position')
        start = report[spec.key('start')]
        delta = report[spec.key('delta')]
        duration = report['duration_s']
        move_start_ns = report.get('move_start_pc_monotonic_ns')
        commands = [row for row in report['published']
                    if row['phase'] == 'move' and 0 < row['scheduled_t_s'] < duration]
        by_bits = {}
        for row in report['command_timing']:
            by_bits.setdefault(row['target_bits'], []).append(row)
        pairs = []
        ambiguous = 0
        for sent in commands:
            matches = by_bits.get(sent['target_bits'], [])
            if len(matches) == 1:
                bridge = matches[0]
                if bridge['callback_entry_ns'] >= sent['pc_publish_call_start_ns']:
                    pairs.append((sent, bridge))
            elif len(matches) > 1:
                ambiguous += 1

        by_stamp = {}
        for row in report['joint_receive_timing']:
            by_stamp.setdefault(row['robot_stamp_ns'], []).append(row)
        state_pairs = []
        for state in report['feedback_samples']:
            matches = by_stamp.get(state['robot_stamp_ns'], [])
            if len(matches) == 1 and state['pc_monotonic_ns'] >= matches[0]['bridge_ros1_callback_ns']:
                state_pairs.append((state, matches[0]))

        segments = {
            'ros2_publish_api_call_ms': distribution([
                (row['pc_monotonic_ns'] - row['pc_publish_call_start_ns']) / 1e6
                for row in report['published']]),
            'pc_ros2_publish_start_to_bridge_callback_ms': distribution([
                (bridge['callback_entry_ns'] - sent['pc_publish_call_start_ns']) / 1e6
                for sent, bridge in pairs]),
            'bridge_callback_to_ros1_publish_start_ms': distribution([
                (bridge['ros1_publish_start_ns'] - bridge['callback_entry_ns']) / 1e6
                for _, bridge in pairs]),
            'ros1_publish_api_call_ms': distribution([
                (bridge['ros1_publish_return_ns'] - bridge['ros1_publish_start_ns']) / 1e6
                for _, bridge in pairs]),
            'bridge_ros1_feedback_callback_to_pc_ros2_joint_callback_ms': distribution([
                (state['pc_monotonic_ns'] - bridge['bridge_ros1_callback_ns']) / 1e6
                for state, bridge in state_pairs]),
        }

        crossings = {}
        if move_start_ns is not None:
            sent_move = [row for row in report['published']
                         if row['phase'] == 'move' and row['scheduled_t_s'] >= 0]
            visible = [row for row in report['feedback_samples']
                       if move_start_ns - 100_000_000 <= row['pc_monotonic_ns'] <=
                       move_start_ns + int((duration + 0.8) * 1e9)]
            for level in (0.1, 0.5, 0.9):
                command_time = first_crossing(sent_move, 'pc_publish_call_start_ns',
                                              target_key, start, delta, level)
                state_time = first_crossing(visible, 'pc_monotonic_ns',
                                            position_key, start, delta, level)
                if command_time is not None and state_time is not None:
                    crossings[str(level)] = {
                        'command_pc_monotonic_ns': command_time,
                        'feedback_pc_monotonic_ns': state_time,
                        'apparent_lag_ms': (state_time - command_time) / 1e6,
                    }

        return {
            'clock': ('All reported intervals use the same PC Linux CLOCK_MONOTONIC. '
                      'Robot Header.stamp is only a matching key.'),
            'moving_commands': len(commands),
            'paired_moving_commands': len(pairs),
            'ambiguous_moving_commands': ambiguous,
            'joint_feedback_samples': len(report['feedback_samples']),
            'paired_joint_feedback_samples': len(state_pairs),
            'bad_timing_messages': report['bad_timing_messages'],
            'segments': segments,
            'trajectory_threshold_crossings': crossings,
            'apparent_lag_median_ms': (statistics.median(
                row['apparent_lag_ms'] for row in crossings.values()) if crossings else None),
            'interpretation': ('Trajectory phase lag includes transport to Pico, control response, '
                               'sampling and return transport. ROS1 publish() returning is not '
                               'Pico receipt or physical joint execution.'),
        }

    @staticmethod
    def save_plot(report, stem, spec):
        """Plot individually matched PC intervals; the main script plots motion."""
        import matplotlib.pyplot as plt

        origin = report.get('move_start_pc_monotonic_ns')
        fig, ax = plt.subplots(figsize=(11, 4))
        if origin is not None:
            sent_by_bits = {row['target_bits']: row for row in report['published']
                            if row['phase'] == 'move' and
                            0 < row['scheduled_t_s'] < report['duration_s']}
            downlink = []
            for row in report['command_timing']:
                sent = sent_by_bits.get(row['target_bits'])
                if sent and row['callback_entry_ns'] >= sent['pc_publish_call_start_ns']:
                    downlink.append(((row['callback_entry_ns'] - origin) / 1e9,
                                     (row['callback_entry_ns'] - sent['pc_publish_call_start_ns']) / 1e6))
            receive_by_stamp = {row['robot_stamp_ns']: row
                                for row in report['joint_receive_timing']}
            uplink = []
            for state in report['feedback_samples']:
                bridge = receive_by_stamp.get(state['robot_stamp_ns'])
                if bridge and state['pc_monotonic_ns'] >= bridge['bridge_ros1_callback_ns']:
                    uplink.append(((state['pc_monotonic_ns'] - origin) / 1e9,
                                   (state['pc_monotonic_ns'] - bridge['bridge_ros1_callback_ns']) / 1e6))
            if downlink:
                ax.scatter(*zip(*downlink), s=5, color='tab:blue',
                           label='PC publish start → bridge ROS2 callback')
            if uplink:
                ax.scatter(*zip(*uplink), s=5, color='tab:orange',
                           label='Bridge ROS1 feedback → PC ROS2 callback')
            if downlink or uplink:
                ax.legend()
        else:
            ax.text(0.5, 0.5, 'Preview: no commands sent', ha='center', va='center',
                    transform=ax.transAxes)
        ax.set(title=f'{spec.name}: individually timed PC segments',
               xlabel='PC monotonic time relative to motion start (s)',
               ylabel='Interval (ms)')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path = Path(str(stem) + '_timing.png')
        fig.savefig(path, dpi=160)
        plt.close(fig)
        return path
