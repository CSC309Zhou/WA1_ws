#!/usr/bin/env python3
"""Shared WA1 single-joint ServoJ trajectory test. Preview by default."""

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import time


MASTER = os.environ.get('WA1_ROS1_MASTER_URI', 'http://192.168.217.1:11311').rstrip('/')
RATE_HZ = 200
RAMP = 0.1
PRE_HOLD_S = 0.2
POST_HOLD_S = 0.3

# Keep ServoJ vectors in the same part order as MoveJ; never use JointState indices.
PARTS = {
    'left_arm': (('Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L',
                  'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L'), 1),
    'right_arm': (('Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R',
                   'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R'), 2),
    'neck': (('Neck_Z', 'Neck_Y'), 4),
    'waist': (('Waist_Z', 'Waist_Y'), 8),
    'lifting': (('Lifting_Z',), 16),
}

# Supplied URDF and teleoperation-guide example bounds intersection; controller
# limits have not been verified. Lifting_Z uses metres; all other axes use rad.
TARGET_LIMITS = {
    'Shoulder_Y_L': (-3.197, 1.145),
    'Shoulder_X_L': (0.0, 3.026),
    'Shoulder_Z_L': (-2.83, 2.83),
    'Elbow_L': (-2.0, 0.07),
    'Wrist_Z_L': (-2.855, 2.855),
    'Wrist_Y_L': (-1.487, 1.2566),
    'Wrist_X_L': (-1.487, 1.487),
    'Shoulder_Y_R': (-3.197, 1.145),
    'Shoulder_X_R': (-3.026, 0.0),
    'Shoulder_Z_R': (-2.83, 2.83),
    'Elbow_R': (-2.0, 0.07),
    'Wrist_Z_R': (-2.855, 2.855),
    'Wrist_Y_R': (-1.487, 1.2566),
    'Wrist_X_R': (-1.487, 1.487),
    'Neck_Z': (-0.733, 0.733),
    'Neck_Y': (-0.1693, 0.6),
    'Waist_Z': (-2.018, 3.0892),
    'Waist_Y': (-0.478, 0.733),
    'Lifting_Z': (0.002, 0.206),
}


@dataclass(frozen=True)
class JointSpec:
    name: str
    part: str
    group: tuple
    arm_type: int
    limits: tuple
    unit: str
    max_peak_speed: float

    @property
    def index(self):
        return self.group.index(self.name)

    @property
    def command(self):
        return f'/cmd/upperlimb/servoj/{self.part}'

    @property
    def ros1_command(self):
        return f'/zj_humanoid/upperlimb/servoj/{self.part}'

    def key(self, kind):
        return f'{kind}_{self.unit}'


def spec_for(joint_name):
    for part, (group, arm_type) in PARTS.items():
        if joint_name in group:
            is_lift = joint_name == 'Lifting_Z'
            return JointSpec(joint_name, part, group, arm_type,
                             TARGET_LIMITS[joint_name], 'm' if is_lift else 'rad',
                             0.05 if is_lift else 5.0)
    raise ValueError(f'Unknown WA1 upper-limb joint: {joint_name}')


def fraction(t, duration):
    """Short acceleration/deceleration; constant velocity in the middle 80%."""
    s = max(0.0, min(1.0, t / duration))
    if s < RAMP:
        return s * s / (2 * RAMP * (1 - RAMP))
    if s > 1 - RAMP:
        return 1 - (1 - s) ** 2 / (2 * RAMP * (1 - RAMP))
    return (s - RAMP / 2) / (1 - RAMP)


def plan_for(start, delta, duration, spec):
    steps = round(duration * RATE_HZ)
    period = duration / steps
    pre = round(PRE_HOLD_S / period)
    post = round(POST_HOLD_S / period)
    plan = []
    for k in range(-pre, steps + post + 1):
        t = k * period
        plan.append({'t_s': t, spec.key('target'): start + delta * fraction(t, duration),
                     'phase': 'prehold' if k < 0 else ('move' if k <= steps else 'posthold')})
    return plan, period, pre


def command_vector(start_group, target, spec):
    """Replace only the selected joint in its ROS1 part-ordered command vector."""
    if len(start_group) != len(spec.group):
        raise ValueError(f'{spec.part} command requires {len(spec.group)} joints')
    goal = list(start_group)
    goal[spec.index] = target
    return goal


class Feedback:
    def __init__(self, spec):
        self.spec = spec
        self.lock = threading.Lock()
        self.joint = None
        self.occupancy = None
        self.status = None
        self.mode = None
        self.samples = []

    def on_joint(self, msg):
        now = time.monotonic_ns()
        if len(msg.name) != 19 or len(msg.position) != 19 or len(set(msg.name)) != 19:
            return
        if not all(math.isfinite(x) for x in msg.position):
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.spec.group):
            return
        group = [positions[name] for name in self.spec.group]
        robot_stamp_ns = (msg.header.stamp.sec * 1_000_000_000 +
                          msg.header.stamp.nanosec)
        with self.lock:
            self.joint = (group, now)
            self.samples.append({'pc_monotonic_ns': now,
                                 self.spec.key('position'): group[self.spec.index],
                                 self.spec.key('group'): group,
                                 'robot_stamp_ns': robot_stamp_ns})

    def on_occupancy(self, msg):
        with self.lock:
            self.occupancy = (int(msg.data), time.monotonic_ns())

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.status = (value, time.monotonic_ns())

    def on_mode(self, msg):
        with self.lock:
            self.mode = (int(msg.cmd_num), str(msg.cmd_name), time.monotonic_ns())

    def snapshot(self, allow_stream_fault=False):
        now = time.monotonic_ns()
        with self.lock:
            joint, occupancy, status = self.joint, self.occupancy, self.status
        if not joint or not occupancy or not status:
            raise RuntimeError('Joint, occupancy, or bridge status feedback is missing')
        if now - joint[1] > 100_000_000 or now - occupancy[1] > 250_000_000:
            raise RuntimeError('Joint or occupancy feedback is stale')
        if now - status[1] > 2_000_000_000:
            raise RuntimeError('Bridge status is stale')
        bridge = status[0]
        if bridge.get('ros1_master_uri', '').rstrip('/') != MASTER:
            raise RuntimeError('Bridge is not connected to the expected real ROS1 Master')
        if bridge.get('feedback_ready') is not True or occupancy[0] != 0:
            raise RuntimeError('Upper limb feedback is not ready or occupied')
        if bridge.get('stream_fault') and not allow_stream_fault:
            raise RuntimeError('Bridge stream fault is latched')
        return list(joint[0]), dict(bridge)

    def stopped_snapshot(self):
        """Require fresh, ready feedback and a fresh STOPPED state."""
        _, bridge = self.snapshot(allow_stream_fault=True)
        now = time.monotonic_ns()
        with self.lock:
            mode = self.mode
        if mode is None or now - mode[2] > 500_000_000:
            raise RuntimeError('UplimbState STOPPED feedback is missing or stale')
        if mode[:2] != (0, 'STOPPED'):
            raise RuntimeError(f'Upper limb is not STOPPED: {mode[:2]}')
        return bridge


def call_service(client, request, label, timeout=5.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    if not future.done() or future.result() is None:
        raise RuntimeError(f'{label} timed out; do not retry automatically')
    response = future.result()
    if not response.success:
        raise RuntimeError(f'{label} rejected: {response.message}')
    return str(response.message)


def reset_latched_fault(feedback, client, request, report, phase, timeout=4.0):
    """Clear only an observed bridge latch after fresh STOPPED feedback."""
    _, bridge = feedback.snapshot(allow_stream_fault=True)
    if not bridge.get('stream_fault'):
        return False
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            bridge = feedback.stopped_snapshot()
            break
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.02)
    else:
        raise RuntimeError(f'{phase} reset refused: STOPPED state not confirmed; '
                           f'last check: {last_error}')
    report[f'{phase}_bridge_before_reset'] = bridge
    report[f'{phase}_reset_message'] = call_service(
        client, request, 'bridge/reset_fault')
    deadline = time.monotonic() + 2.5
    last_error = None
    while time.monotonic() < deadline:
        try:
            _, bridge = feedback.snapshot(allow_stream_fault=True)
            if bridge.get('stream_fault') is False:
                report[f'{phase}_bridge_after_reset'] = bridge
                return True
        except RuntimeError as exc:
            last_error = exc
        time.sleep(0.02)
    raise RuntimeError(f'{phase} reset service succeeded but ready bridge status did not clear; '
                       f'last check: {last_error}')


def save_result(report, stem, spec):
    """Save original times and values before plotting; no plotting in the control loop."""
    start_ns = report.get('move_start_pc_monotonic_ns')
    samples = report['feedback_samples']
    start_key = spec.key('start')
    delta_key = spec.key('delta')
    target_key = spec.key('target')
    position_key = spec.key('position')
    if start_ns is not None and samples:
        during = [row for row in samples
                  if 0 <= (row['pc_monotonic_ns'] - start_ns) / 1e9 <= report['duration_s']]
        if during:
            errors = [row[position_key] -
                      (report[start_key] + report[delta_key] *
                       fraction((row['pc_monotonic_ns'] - start_ns) / 1e9,
                                report['duration_s'])) for row in during]
            report['pc_observed_tracking'] = {
                'samples': len(errors),
                spec.key('rms_error'): math.sqrt(sum(x * x for x in errors) / len(errors)),
                spec.key('max_abs_error'): max(map(abs, errors)),
                'clock_note': 'PC receipt time includes robot sampling and transport delay',
            }
            if len(during) > 1:
                report['pc_observed_tracking']['feedback_hz'] = (
                    (len(during) - 1) * 1e9 /
                    (during[-1]['pc_monotonic_ns'] - during[0]['pc_monotonic_ns']))
        report[spec.key('last_observed')] = samples[-1][position_key]
        report[spec.key('last_error')] = samples[-1][position_key] - report[target_key]
        if report['published']:
            report[spec.key('last_published_error')] = (
                samples[-1][position_key] - report['published'][-1][target_key])
    sent = report['published']
    if len(sent) > 1:
        intervals = [(b['pc_monotonic_ns'] - a['pc_monotonic_ns']) / 1e6
                     for a, b in zip(sent, sent[1:])]
        report['publish_interval_ms'] = {'median': statistics.median(intervals),
                                         'max': max(intervals), 'min': min(intervals)}
    before = report.get('bridge_status_before') or {}
    after = report.get('bridge_status_after') or {}
    if all(isinstance(before.get(key), int) and isinstance(after.get(key), int)
           for key in ('sent', 'dropped')):
        report['bridge_counter_delta'] = {
            key: after[key] - before[key] for key in ('sent', 'dropped')}
    data_path = stem.with_suffix('.json')
    data_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 5))
    plan = report['plan']
    if plan:
        ax.plot([x['t_s'] for x in plan], [x[target_key] for x in plan],
                '--', label='Ideal target')
    if start_ns is not None:
        if sent:
            ax.plot([(x['pc_monotonic_ns'] - start_ns) / 1e9 for x in sent],
                    [x[target_key] for x in sent], label='Published target')
        visible = [x for x in samples
                   if -PRE_HOLD_S - 0.1 <= (x['pc_monotonic_ns'] - start_ns) / 1e9
                   <= report['duration_s'] + POST_HOLD_S + 0.6]
        if visible:
            ax.plot([(x['pc_monotonic_ns'] - start_ns) / 1e9 for x in visible],
                    [x[position_key] for x in visible], label='JointState on PC')
        ax.set_xlim(-PRE_HOLD_S - 0.1, report['duration_s'] + POST_HOLD_S + 0.6)
    ax.axvline(0, color='0.5', linewidth=0.8)
    ax.axvline(report['duration_s'], color='0.5', linewidth=0.8)
    ax.set(title=f"{spec.name}: delta={report[delta_key]:+.4f} {spec.unit}, "
                 f"duration={report['duration_s']:.3f} s, {RATE_HZ} Hz ({report['mode']})",
           xlabel='PC monotonic time relative to movement start (s)',
           ylabel=f'Joint position ({spec.unit})')
    ax.grid(True, alpha=0.3)
    handles, _ = ax.get_legend_handles_labels()
    if handles:
        ax.legend()
    else:
        ax.text(0.5, 0.5, 'No trajectory generated', ha='center', va='center',
                transform=ax.transAxes)
    fig.tight_layout()
    image_path = stem.with_suffix('.png')
    fig.savefig(image_path, dpi=160)
    plt.close(fig)
    return data_path, image_path


def main(joint_name, *, latency_probe=False):
    if latency_probe and joint_name != 'Wrist_Z_L':
        raise ValueError('The current latency probe is for Wrist_Z_L only')
    if latency_probe:
        from latency.probe import LatencyCapture
    spec = spec_for(joint_name)
    start_key = spec.key('start')
    target_key = spec.key('target')
    delta_key = spec.key('delta')
    speed_key = f'planned_peak_speed_{spec.unit}_s'
    accel_key = f'planned_peak_accel_{spec.unit}_s2'
    parser = argparse.ArgumentParser(
        description=f'{spec.name} ServoJ trajectory test; preview by default.')
    parser.add_argument('--duration', required=True, type=float,
                        help='Movement duration in seconds, strictly greater than 0.1')
    parser.add_argument('--delta', required=True, type=float,
                        help=f'Finite signed relative {spec.name} displacement in {spec.unit}; '
                             f'planned peak speed <= {spec.max_peak_speed:g} {spec.unit}/s')
    parser.add_argument('--execute', action='store_true',
                        help='Actually configure ServoJ and publish to the real robot')
    parser.add_argument('--output-dir', type=Path,
                        default=(Path(__file__).resolve().parent /
                                 ('latency/results' if latency_probe else 'results')),
                        help=f'PC results root; latest files are overwritten in its {spec.name} subfolder')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.1:
        parser.error('--duration must be finite and greater than 0.1 s')
    if not math.isfinite(args.delta):
        parser.error('--delta must be finite')
    if latency_probe and args.delta == 0:
        parser.error('A latency trajectory needs a nonzero --delta')
    planned_peak_speed = abs(args.delta) / (args.duration * (1 - RAMP))
    planned_peak_accel = planned_peak_speed / (args.duration * RAMP)
    if planned_peak_speed > spec.max_peak_speed:
        parser.error(f'Constant-section speed would exceed '
                     f'{spec.max_peak_speed:g} {spec.unit}/s')
    if args.execute and os.environ.get('ROS_DOMAIN_ID', '0') != '0':
        parser.error('--execute requires the real PC DDS domain 0')

    # Plotting and local write access must work before any robot-side operation.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot  # noqa: F401
    joint_dir = args.output_dir / spec.name
    joint_dir.mkdir(parents=True, exist_ok=True)
    stem = joint_dir / 'latest'
    # Check write access without erasing the previous result before this run completes.
    with tempfile.TemporaryFile(dir=joint_dir) as check_file:
        check_file.write(b'write check')

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState, TimeReference
    from std_msgs.msg import Int8, String
    from std_srvs.srv import Trigger
    from upperlimb.msg import Joints, UplimbState
    from upperlimb.srv import Servo

    rclpy.init()
    node = rclpy.create_node(f'wa1_{spec.name.lower()}_servoj_operator')
    feedback = Feedback(spec)
    latency = LatencyCapture() if latency_probe else None
    keep = []
    keep.append(node.create_subscription(
        JointState, '/upperlimb/joint_states', feedback.on_joint,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    keep.append(node.create_subscription(
        Int8, '/upperlimb/occupancy', feedback.on_occupancy,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    keep.append(node.create_subscription(
        String, '/upperlimb/bridge/status', feedback.on_status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    keep.append(node.create_subscription(
        UplimbState, '/upperlimb/uplimb_state', feedback.on_mode,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    if latency is not None:
        keep.append(node.create_subscription(
            TimeReference, latency.RECEIVE_TOPIC, latency.on_joint_receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
        keep.append(node.create_subscription(
            String, latency.COMMAND_TOPIC, latency.on_command_timing,
            QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT)))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    report = {'mode': 'EXECUTE' if args.execute else 'PREVIEW_ONLY',
              'joint_name': spec.name, 'part': spec.part, 'unit': spec.unit,
              'arm_type': spec.arm_type,
              'duration_s': args.duration, delta_key: args.delta,
              speed_key: planned_peak_speed, accel_key: planned_peak_accel,
              'rate_hz': RATE_HZ, 'ramp_fraction': RAMP,
              'pre_hold_s': PRE_HOLD_S, 'post_hold_s': POST_HOLD_S,
              'command_topic': spec.command, 'plan': [], 'published': [],
              'feedback_samples': [], 'skipped_slots': 0,
              'stop_called': False, 'servo_params_cleared': False,
              'completion': 'not_started'}
    if latency is not None:
        report['latency_probe'] = True
    set_client = clear_client = stop_client = reset_client = None
    configured = False
    exit_code = 0
    try:
        deadline = time.monotonic() + 8.0
        last_feedback_error = None
        while time.monotonic() < deadline:
            try:
                current, status = feedback.snapshot(allow_stream_fault=True)
                break
            except RuntimeError as exc:
                last_feedback_error = exc
                time.sleep(0.02)
        else:
            raise RuntimeError(f'No ready joint, occupancy and bridge feedback within 8 s; '
                               f'last check: {last_feedback_error}')
        if node.count_publishers('/upperlimb/bridge/status') != 1:
            raise RuntimeError('Expected exactly one bridge status publisher')
        if not spec.limits[0] <= current[spec.index] + args.delta <= spec.limits[1]:
            raise RuntimeError(f'{spec.name} target outside configured position limits')
        report[spec.key('preview_start')] = current[spec.index]
        report['bridge_status_before'] = status
        plan, period, pre_steps = plan_for(current[spec.index], args.delta,
                                           args.duration, spec)
        report['plan'] = plan
        report['planned_rate_hz'] = 1.0 / period
        report[start_key] = current[spec.index]
        report[target_key] = current[spec.index] + args.delta
        params = {'v': 0.0, 'acc': 0.0, 'time': period,
                  'lookahead_time': 0.2, 'gain': 100, 'arm_type': spec.arm_type}
        report['servo_params'] = params
        print(json.dumps({'mode': report['mode'], 'joint': spec.name,
                          'part': spec.part, 'unit': spec.unit,
                          start_key: current[spec.index], target_key: report[target_key],
                          'duration_s': args.duration, delta_key: args.delta,
                          speed_key: planned_peak_speed, accel_key: planned_peak_accel,
                          'planned_samples': len(plan), 'servo_params': params,
                          'result_prefix': str(stem)}, indent=2), flush=True)
        if not args.execute:
            report['completion'] = 'preview_only'
            print('Preview complete. No servo service, fault reset or command publisher was created.')
        else:
            if status.get('read_only') is not False:
                raise RuntimeError('Control bridge is not running')
            if node.count_publishers(spec.command) != 0:
                raise RuntimeError('Another ROS2 ServoJ publisher exists')
            if latency is not None:
                probe_deadline = time.monotonic() + 5.0
                while time.monotonic() < probe_deadline and (
                        node.count_publishers(latency.COMMAND_TOPIC) != 1 or
                        node.count_publishers(latency.RECEIVE_TOPIC) != 1):
                    time.sleep(0.02)
                if node.count_publishers(latency.COMMAND_TOPIC) != 1:
                    raise RuntimeError('PC bridge latency diagnostics missing; start with '
                                       '--latency-probe')
                if node.count_publishers(latency.RECEIVE_TOPIC) != 1:
                    raise RuntimeError('PC bridge joint receive timing topic missing')
            set_client = node.create_client(Servo, '/upperlimb/set_servo_params')
            clear_client = node.create_client(Servo, '/upperlimb/clear_servo_params')
            stop_client = node.create_client(Trigger, '/upperlimb/stop')
            reset_client = node.create_client(Trigger, '/upperlimb/bridge/reset_fault')
            for label, client in (('set_servo_params', set_client),
                                  ('clear_servo_params', clear_client), ('stop', stop_client),
                                  ('bridge/reset_fault', reset_client)):
                if not client.wait_for_service(timeout_sec=5.0):
                    raise RuntimeError(f'{label} service is unavailable')
            report['pre_run_reset_performed'] = reset_latched_fault(
                feedback, reset_client, Trigger.Request(), report, 'pre_run')
            publisher = node.create_publisher(
                Joints, spec.command,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            keep.append(publisher)
            until = time.monotonic() + 5.0
            while time.monotonic() < until and node.count_subscribers(spec.command) != 1:
                time.sleep(0.02)
            if node.count_subscribers(spec.command) != 1:
                raise RuntimeError('Expected exactly one PC bridge command subscriber')
            current, status = feedback.snapshot()
            report['bridge_status_before'] = status
            if status.get('ros1_command_connections', {}).get(spec.ros1_command) != 1:
                raise RuntimeError('Expected exactly one real ROS1 ServoJ connection')
            if status.get('read_only') is not False:
                raise RuntimeError('Control bridge changed mode')
            report['set_servo_params_message'] = call_service(
                set_client, Servo.Request(**params), 'set_servo_params')
            configured = True
            current, status = feedback.snapshot()
            if not spec.limits[0] <= current[spec.index] + args.delta <= spec.limits[1]:
                raise RuntimeError(f'{spec.name} target moved outside limits')
            start_group = current
            report[start_key] = start_group[spec.index]
            report[target_key] = start_group[spec.index] + args.delta
            plan, period, pre_steps = plan_for(report[start_key], args.delta,
                                               args.duration, spec)
            report['plan'] = plan
            period_ns = round(period * 1e9)
            stream_start_ns = time.monotonic_ns()
            move_start_ns = stream_start_ns + pre_steps * period_ns
            report['move_start_pc_monotonic_ns'] = move_start_ns
            for slot, row in enumerate(plan):
                due_ns = stream_start_ns + slot * period_ns
                remaining_ns = due_ns - time.monotonic_ns()
                if remaining_ns > 0:
                    time.sleep(remaining_ns / 1e9)
                now_ns = time.monotonic_ns()
                if now_ns - due_ns > period_ns:
                    report['skipped_slots'] += 1
                    continue
                _, status = feedback.snapshot()
                if status.get('read_only') is not False:
                    raise RuntimeError('Control bridge changed mode during stream')
                goal = command_vector(start_group, row[target_key], spec)
                publish_start_ns = time.monotonic_ns() if latency is not None else None
                publisher.publish(Joints(joint=goal))
                sent_ns = time.monotonic_ns()
                published = {'pc_monotonic_ns': sent_ns,
                             'scheduled_t_s': row['t_s'],
                             target_key: row[target_key],
                             'phase': row['phase']}
                if latency is not None:
                    published['pc_publish_call_start_ns'] = publish_start_ns
                    published['target_bits'] = latency.bits(row[target_key])
                report['published'].append(published)
            if not report['published'] or report['published'][-1]['phase'] != 'posthold':
                raise RuntimeError('Final hold was not published')
            report['completion'] = 'stream_completed'
    except KeyboardInterrupt:
        report['completion'] = 'interrupted'
        report['error'] = 'KeyboardInterrupt; physical stop is not confirmed'
        exit_code = 130
    except Exception as exc:
        report['completion'] = 'failed'
        report['error'] = str(exc)
        exit_code = 1
    finally:
        if report['published'] and stop_client is not None:
            try:
                report['stop_message'] = call_service(stop_client, Trigger.Request(), 'stop')
                report['stop_called'] = True
            except Exception as exc:
                report['stop_error'] = str(exc)
                exit_code = 1
        if configured and clear_client is not None:
            try:
                report['clear_servo_params_message'] = call_service(
                    clear_client, Servo.Request(**report['servo_params']), 'clear_servo_params')
                report['servo_params_cleared'] = True
            except Exception as exc:
                report['clear_servo_params_error'] = str(exc)
                exit_code = 1
        if report['published']:
            # Current bridge watchdog still latches after a finite stream ends.
            time.sleep(1.3)
        with feedback.lock:
            report['feedback_samples'] = list(feedback.samples)
            report['bridge_status_after'] = feedback.status[0] if feedback.status else None
            report['uplimb_state_after'] = feedback.mode[:2] if feedback.mode else None
        if latency is not None:
            report.update(latency.snapshot())
        if exit_code and report['completion'] == 'stream_completed':
            report['completion'] = 'cleanup_failed'
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        if latency is not None:
            try:
                report['latency_analysis'] = latency.analyze(report, spec)
            except Exception as exc:
                report['latency_analysis_error'] = str(exc)
                exit_code = 1
        report['exit_code'] = exit_code
        try:
            data_path, image_path = save_result(report, stem, spec)
            print(f'Data: {data_path}\nPlot: {image_path}')
            if latency is not None and 'latency_analysis' in report:
                timing_path = latency.save_plot(report, stem, spec)
                print(f'Timing plot: {timing_path}')
                print(json.dumps(report['latency_analysis'], ensure_ascii=False, indent=2))
        except Exception as exc:
            print(f'Result saving failed: {exc}; JSON path: {stem.with_suffix(".json")}')
            exit_code = 1
        if report.get('error'):
            print('Test error:', report['error'])
        if report.get('pre_run_reset_performed'):
            print('Previous bridge stream fault reset after fresh STOPPED feedback check.')
        if report['published']:
            print('Inspect bridge/status and cleanup fields; the next --execute will check and reset a latched stream fault.')
    return exit_code
