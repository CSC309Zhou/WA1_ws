#!/usr/bin/env python3
"""Preview or run one finite right-arm ServoL TCP trajectory in BASE.

Inputs are xyz metres and fixed-axis XYZ RPY radians. Only --execute calls
servo services or publishes commands. Preview reads feedback and writes PC files.
"""

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import time


MASTER = os.environ.get('WA1_ROS1_MASTER_URI', 'http://192.168.217.1:11311').rstrip('/')
COMMAND = '/cmd/upperlimb/servol/right_arm'
ROS1_COMMAND = '/zj_humanoid/upperlimb/servol/right_arm'
RATE_HZ = 200
RAMP = 0.1
PRE_HOLD_S = 0.2
POST_HOLD_S = 0.3
LEFT_JOINTS = ('Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L',
               'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L')
RIGHT_JOINTS = ('Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R',
                'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R')


def quat_unit(q):
    norm = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid quaternion')
    return [x / norm for x in q]


def quat_from_rpy(roll, pitch, yaw):
    """Fixed-axis XYZ rotation, Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return quat_unit([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_multiply(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return quat_unit([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def quat_slerp(start, end, fraction):
    dot = sum(a*b for a, b in zip(start, end))
    if dot < 0:
        end = [-x for x in end]
        dot = -dot
    if dot > 0.9995:
        return quat_unit([a + fraction * (b-a) for a, b in zip(start, end)])
    angle = math.acos(min(1.0, dot))
    scale = math.sin(angle)
    a = math.sin((1-fraction) * angle) / scale
    b = math.sin(fraction * angle) / scale
    return quat_unit([a*x + b*y for x, y in zip(start, end)])


def quat_angle_deg(a, b):
    dot = min(1.0, abs(sum(x*y for x, y in zip(a, b))))
    return 2 * math.degrees(math.acos(dot))


def tcp_pose(msg):
    xyz = [msg.position.x, msg.position.y, msg.position.z]
    q = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
    if not all(math.isfinite(x) for x in xyz + q):
        raise ValueError('Non-finite TCP position or quaternion')
    norm = math.sqrt(sum(x*x for x in q))
    if not 0.98 <= norm <= 1.02:
        raise ValueError(f'Invalid TCP quaternion norm {norm}')
    return {'position_m': xyz, 'quaternion_xyzw': quat_unit(q)}


def write_pose(destination, pose):
    (destination.position.x, destination.position.y,
     destination.position.z) = (float(x) for x in pose['position_m'])
    (destination.orientation.x, destination.orientation.y,
     destination.orientation.z, destination.orientation.w) = (
         float(x) for x in pose['quaternion_xyzw'])


def fraction_at(t, duration):
    """10% acceleration and deceleration ramps; constant speed in between."""
    s = max(0.0, min(1.0, t / duration))
    if s < RAMP:
        return s*s / (2*RAMP*(1-RAMP))
    if s > 1-RAMP:
        return 1 - (1-s)**2 / (2*RAMP*(1-RAMP))
    return (s-RAMP/2) / (1-RAMP)


def plan_for(start, delta, duration):
    steps = round(duration * RATE_HZ)
    period = duration / steps
    pre = round(PRE_HOLD_S / period)
    post = round(POST_HOLD_S / period)
    start_q = start['quaternion_xyzw']
    target_q = quat_multiply(quat_from_rpy(*delta[3:]), start_q)
    result = []
    for k in range(-pre, steps + post + 1):
        t = k * period
        u = fraction_at(t, duration)
        result.append({
            't_s': t,
            'phase': 'prehold' if k < 0 else ('move' if k <= steps else 'posthold'),
            'position_m': [a + u*b for a, b in zip(start['position_m'], delta[:3])],
            'quaternion_xyzw': quat_slerp(start_q, target_q, u),
        })
    return result, period, pre


class Feedback:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.tcp_samples = []
        self.joint_samples = []
        self.invalid_tcp = 0

    def update(self, key, value, now=None):
        now = time.monotonic_ns() if now is None else now
        with self.lock:
            self.latest[key] = (value, now)

    def on_tcp(self, key, msg):
        now = time.monotonic_ns()
        try:
            pose = tcp_pose(msg)
        except ValueError:
            with self.lock:
                self.invalid_tcp += 1
            return
        with self.lock:
            self.latest[key] = (pose, now)
            if key == 'right':
                self.tcp_samples.append({'pc_monotonic_ns': now, **pose})

    def on_joint(self, msg):
        if len(msg.name) != 19 or len(msg.position) != 19 or len(set(msg.name)) != 19:
            return
        if not all(math.isfinite(x) for x in msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if any(name not in values for name in LEFT_JOINTS + RIGHT_JOINTS):
            return
        now = time.monotonic_ns()
        with self.lock:
            self.latest['joint'] = (values, now)
            self.joint_samples.append({
                'pc_monotonic_ns': now,
                'left_rad': [values[name] for name in LEFT_JOINTS],
                'right_rad': [values[name] for name in RIGHT_JOINTS],
            })

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.update('status', value)

    def on_mode(self, msg):
        self.update('mode', (int(msg.cmd_num), str(msg.cmd_name)))

    def snapshot(self, allow_stream_fault=False):
        now = time.monotonic_ns()
        with self.lock:
            latest = dict(self.latest)
        maximum_age = {'left': 250_000_000, 'right': 250_000_000,
                       'joint': 250_000_000, 'occupancy': 250_000_000,
                       'mode': 500_000_000, 'status': 2_000_000_000}
        for key, age in maximum_age.items():
            if key not in latest:
                raise RuntimeError(f'{key} feedback is missing')
            if now - latest[key][1] > age:
                raise RuntimeError(f'{key} feedback is stale')
        values = {key: pair[0] for key, pair in latest.items()}
        status = values['status']
        if status.get('ros1_master_uri', '').rstrip('/') != MASTER:
            raise RuntimeError('Bridge is not connected to the expected real ROS1 Master')
        if status.get('feedback_ready') is not True or values['occupancy'] != 0:
            raise RuntimeError(f"Feedback not ready or occupancy={values['occupancy']}")
        if status.get('stream_fault') and not allow_stream_fault:
            raise RuntimeError('Bridge stream fault is latched')
        return values

    def stopped_snapshot(self):
        values = self.snapshot(allow_stream_fault=True)
        if values['mode'] != (0, 'STOPPED'):
            raise RuntimeError(f"Upper limb is not STOPPED: {values['mode']}")
        return values


def wait_for_feedback(feedback, timeout=8.0, stopped=False):
    end = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < end:
        try:
            return feedback.stopped_snapshot() if stopped else feedback.snapshot(True)
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.02)
    raise RuntimeError(f'No ready feedback within {timeout:g} s; last check: {last_error}')


def call_service(client, request, label, timeout=5.0):
    future = client.call_async(request)
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        time.sleep(0.01)
    if not future.done() or future.result() is None:
        raise RuntimeError(f'{label} timed out; do not retry automatically')
    response = future.result()
    if not response.success:
        raise RuntimeError(f'{label} rejected: {response.message}')
    return str(response.message)


def reset_latched_fault(feedback, client, request):
    if not feedback.snapshot(allow_stream_fault=True)['status'].get('stream_fault'):
        return False
    wait_for_feedback(feedback, timeout=4.0, stopped=True)
    call_service(client, request, 'bridge/reset_fault')
    end = time.monotonic() + 2.5
    last_error = None
    while time.monotonic() < end:
        try:
            if feedback.snapshot(allow_stream_fault=True)['status'].get('stream_fault') is False:
                return True
        except RuntimeError as exc:
            last_error = exc
        time.sleep(0.02)
    raise RuntimeError('Bridge reset response succeeded, but the stream fault did not clear; '
                       f'last feedback error: {last_error}')


def save_result(report, stem):
    data_path = stem.with_suffix('.json')
    data_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    plan = report['plan']
    published = report['published']
    start_ns = report.get('move_start_pc_monotonic_ns')
    feedback = report['tcp_feedback_samples']
    first_q = report.get('start_right_tcp', {}).get('quaternion_xyzw')
    for axis, label in enumerate(('X (m)', 'Y (m)', 'Z (m)', 'rotation from start (deg)')):
        ax = axes[axis]
        if axis < 3:
            extract = lambda row, i=axis: row['position_m'][i]
        else:
            extract = lambda row: quat_angle_deg(first_q, row['quaternion_xyzw'])
        if plan:
            ax.plot([row['t_s'] for row in plan], [extract(row) for row in plan],
                    '--', label='Planned target')
        if start_ns is not None:
            if published:
                ax.plot([(row['pc_monotonic_ns'] - start_ns) / 1e9 for row in published],
                        [extract(row) for row in published], label='Published target')
            visible = [row for row in feedback
                       if -PRE_HOLD_S - 0.1 <= (row['pc_monotonic_ns'] - start_ns) / 1e9
                       <= report['duration_s'] + POST_HOLD_S + 0.6]
            if visible:
                ax.plot([(row['pc_monotonic_ns'] - start_ns) / 1e9 for row in visible],
                        [extract(row) for row in visible], label='TCP feedback on PC')
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.axvline(0, color='0.6', linewidth=0.8)
        ax.axvline(report['duration_s'], color='0.6', linewidth=0.8)
    axes[0].legend(loc='best')
    axes[-1].set_xlabel('PC monotonic time relative to movement start (s)')
    fig.suptitle(f"Right ServoL: delta={report['delta_xyz_m_rpy_rad']}, "
                 f"duration={report['duration_s']:g} s, {RATE_HZ} Hz ({report['mode']})")
    fig.tight_layout()
    plot_path = stem.with_suffix('.png')
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    return data_path, plot_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', required=True, type=float,
                        help='Movement duration in seconds, strictly greater than 0.1')
    parser.add_argument('--delta', required=True, nargs=6, type=float,
                        metavar=('DX', 'DY', 'DZ', 'DROLL', 'DPITCH', 'DYAW'),
                        help='BASE-frame relative TCP pose; metres then radians')
    parser.add_argument('--execute', action='store_true',
                        help='Configure ServoL and publish the finite target stream')
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).resolve().parent / 'results' / 'Right_TCP',
                        help='PC result directory; preview and latest files are separate')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.1:
        parser.error('--duration must be finite and greater than 0.1 s')
    if not all(math.isfinite(x) for x in args.delta) or not any(x != 0 for x in args.delta):
        parser.error('--delta must be finite and nonzero')
    if args.execute and os.environ.get('ROS_DOMAIN_ID', '0') != '0':
        parser.error('--execute requires the real PC DDS domain 0')

    # Verify all PC output dependencies before any robot-facing operation.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot  # noqa: F401
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryFile(dir=args.output_dir) as probe:
        probe.write(b'write check')
    stem = args.output_dir / ('latest' if args.execute else 'preview')

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8, String
    from std_srvs.srv import Trigger
    from upperlimb.msg import DualPose, Pose as TcpPose, UplimbState
    from upperlimb.srv import Servo

    rclpy.init()
    node = rclpy.create_node('wa1_right_arm_servol_operator')
    feedback = Feedback()
    keep = []
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    for cls, topic, callback in (
        (TcpPose, '/upperlimb/tcp_pose/left', lambda msg: feedback.on_tcp('left', msg)),
        (TcpPose, '/upperlimb/tcp_pose/right', lambda msg: feedback.on_tcp('right', msg)),
        (JointState, '/upperlimb/joint_states', feedback.on_joint),
        (Int8, '/upperlimb/occupancy', lambda msg: feedback.update('occupancy', int(msg.data))),
        (UplimbState, '/upperlimb/uplimb_state', feedback.on_mode),
    ):
        keep.append(node.create_subscription(cls, topic, callback, sensor_qos))
    keep.append(node.create_subscription(
        String, '/upperlimb/bridge/status', feedback.on_status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    report = {
        'mode': 'EXECUTE' if args.execute else 'PREVIEW_ONLY',
        'command_topic': COMMAND, 'ros1_command_topic': ROS1_COMMAND,
        'duration_s': args.duration, 'delta_xyz_m_rpy_rad': args.delta,
        'rate_hz': RATE_HZ, 'ramp_fraction': RAMP,
        'pre_hold_s': PRE_HOLD_S, 'post_hold_s': POST_HOLD_S,
        'plan': [], 'published': [], 'tcp_feedback_samples': [], 'joint_feedback_samples': [],
        'skipped_slots': 0, 'stop_called': False, 'servo_params_cleared': False,
        'completion': 'not_started',
    }
    set_client = clear_client = stop_client = reset_client = None
    configured = False
    exit_code = 0
    try:
        initial = wait_for_feedback(feedback)
        if node.count_publishers('/upperlimb/bridge/status') != 1:
            raise RuntimeError('Expected exactly one PC bridge status publisher')
        start_right = initial['right']
        plan, period, pre_steps = plan_for(start_right, args.delta, args.duration)
        target = {'position_m': plan[-1]['position_m'],
                  'quaternion_xyzw': plan[-1]['quaternion_xyzw']}
        params = {'v': 0.0, 'acc': 0.0, 'time': period,
                  'lookahead_time': 0.2, 'gain': 100, 'arm_type': 2}
        report.update({'start_right_tcp': start_right, 'start_left_tcp': initial['left'],
                       'target_right_tcp': target, 'plan': plan,
                       'servo_params': params, 'planned_samples': len(plan),
                       'planned_linear_peak_m_s':
                           math.dist([0, 0, 0], args.delta[:3]) / (args.duration * (1-RAMP)),
                       'planned_angular_peak_rad_s':
                           math.radians(quat_angle_deg(start_right['quaternion_xyzw'],
                                                       target['quaternion_xyzw'])) /
                           (args.duration * (1-RAMP)),
                       'bridge_status_before': initial['status']})
        report['planned_linear_peak_accel_m_s2'] = (
            report['planned_linear_peak_m_s'] / (args.duration * RAMP))
        report['planned_angular_peak_accel_rad_s2'] = (
            report['planned_angular_peak_rad_s'] / (args.duration * RAMP))
        print(json.dumps({key: report[key] for key in (
            'mode', 'command_topic', 'duration_s', 'delta_xyz_m_rpy_rad',
            'start_right_tcp', 'target_right_tcp', 'start_left_tcp',
            'planned_samples', 'planned_linear_peak_m_s',
            'planned_linear_peak_accel_m_s2', 'planned_angular_peak_rad_s',
            'planned_angular_peak_accel_rad_s2', 'servo_params')},
            ensure_ascii=False, indent=2), flush=True)
        if not args.execute:
            report['completion'] = 'preview_only'
            print('Preview complete. No servo service, fault reset, or command publisher was created.')
        else:
            if initial['status'].get('read_only') is not False:
                raise RuntimeError('Control bridge is not running')
            if node.count_publishers(COMMAND) != 0:
                raise RuntimeError('Another ROS2 ServoL publisher exists')
            set_client = node.create_client(Servo, '/upperlimb/set_servo_params')
            clear_client = node.create_client(Servo, '/upperlimb/clear_servo_params')
            stop_client = node.create_client(Trigger, '/upperlimb/stop')
            reset_client = node.create_client(Trigger, '/upperlimb/bridge/reset_fault')
            for name, client in (('set_servo_params', set_client),
                                 ('clear_servo_params', clear_client),
                                 ('stop', stop_client), ('bridge/reset_fault', reset_client)):
                if not client.wait_for_service(timeout_sec=5.0):
                    raise RuntimeError(f'{name} service is unavailable')
            report['pre_run_reset_performed'] = reset_latched_fault(
                feedback, reset_client, Trigger.Request())
            feedback.stopped_snapshot()
            publisher = node.create_publisher(
                DualPose, COMMAND,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
            keep.append(publisher)
            end = time.monotonic() + 5.0
            while time.monotonic() < end and node.count_subscribers(COMMAND) != 1:
                time.sleep(0.02)
            if node.count_subscribers(COMMAND) != 1:
                raise RuntimeError('Expected exactly one PC bridge ServoL subscriber')
            ready = feedback.stopped_snapshot()
            status = ready['status']
            if status.get('read_only') is not False:
                raise RuntimeError('Control bridge changed mode')
            if status.get('ros1_command_connections', {}).get(ROS1_COMMAND) != 1:
                raise RuntimeError('Expected exactly one real ROS1 ServoL connection')
            report['set_servo_params_message'] = call_service(
                set_client, Servo.Request(**params), 'set_servo_params')
            configured = True
            ready = feedback.snapshot()
            start_right = ready['right']
            plan, period, pre_steps = plan_for(start_right, args.delta, args.duration)
            report['start_right_tcp'] = start_right
            report['start_left_tcp'] = ready['left']
            report['target_right_tcp'] = {'position_m': plan[-1]['position_m'],
                                         'quaternion_xyzw': plan[-1]['quaternion_xyzw']}
            report['plan'] = plan
            report['bridge_status_before'] = ready['status']
            period_ns = round(period * 1e9)
            stream_start_ns = time.monotonic_ns()
            move_start_ns = stream_start_ns + pre_steps * period_ns
            report['move_start_pc_monotonic_ns'] = move_start_ns
            print(json.dumps({'publishing_to': COMMAND,
                              'start_right_tcp': start_right,
                              'target_right_tcp': report['target_right_tcp'],
                              'planned_samples': len(plan)}, indent=2), flush=True)
            for slot, row in enumerate(plan):
                due_ns = stream_start_ns + slot * period_ns
                remaining_ns = due_ns - time.monotonic_ns()
                if remaining_ns > 0:
                    time.sleep(remaining_ns / 1e9)
                now_ns = time.monotonic_ns()
                if now_ns - due_ns > period_ns:
                    report['skipped_slots'] += 1
                    continue
                current = feedback.snapshot()
                if current['status'].get('read_only') is not False:
                    raise RuntimeError('Control bridge changed mode during ServoL stream')
                if current['mode'][0] == 15 or current['mode'][1] == 'PROTECTED':
                    raise RuntimeError('Upper limb entered PROTECTED during ServoL stream')
                message = DualPose()
                write_pose(message.right_arm_pose, row)
                write_pose(message.left_arm_pose, current['left'])
                publisher.publish(message)
                report['published'].append({
                    'pc_monotonic_ns': time.monotonic_ns(),
                    'scheduled_t_s': row['t_s'], 'phase': row['phase'],
                    'position_m': row['position_m'],
                    'quaternion_xyzw': row['quaternion_xyzw'],
                })
            if not report['published'] or report['published'][-1]['phase'] != 'posthold':
                raise RuntimeError('Final target hold was not published')
            report['completion'] = 'stream_completed'
    except KeyboardInterrupt:
        report['completion'] = 'interrupted'
        report['error'] = 'KeyboardInterrupt; robot stop is not confirmed'
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
            # A finite stream still causes the current PC bridge watchdog to latch.
            time.sleep(1.3)
        with feedback.lock:
            report['tcp_feedback_samples'] = list(feedback.tcp_samples)
            report['joint_feedback_samples'] = list(feedback.joint_samples)
            report['invalid_tcp_samples'] = feedback.invalid_tcp
            report['bridge_status_after'] = (feedback.latest['status'][0]
                                             if 'status' in feedback.latest else None)
            report['uplimb_state_after'] = (feedback.latest['mode'][0]
                                            if 'mode' in feedback.latest else None)
            last_right = feedback.latest['right'][0] if 'right' in feedback.latest else None
        if report['published'] and report['uplimb_state_after'] is not None and (
                report['uplimb_state_after'][0] == 15 or
                report['uplimb_state_after'][1] == 'PROTECTED'):
            report['post_run_error'] = 'Upper limb ended in PROTECTED'
            exit_code = 1
        if report.get('target_right_tcp') and last_right and report['published']:
            target = report['target_right_tcp']
            report['observed_right_tcp'] = last_right
            report['final_tcp_error'] = {
                'position_error_m': math.dist(target['position_m'], last_right['position_m']),
                'orientation_error_deg': quat_angle_deg(
                    target['quaternion_xyzw'], last_right['quaternion_xyzw']),
            }
        if len(report['published']) > 1:
            intervals = [(b['pc_monotonic_ns']-a['pc_monotonic_ns'])/1e6
                         for a, b in zip(report['published'], report['published'][1:])]
            report['publish_interval_ms'] = {
                'median': statistics.median(intervals), 'max': max(intervals),
                'min': min(intervals),
            }
        before = report.get('bridge_status_before') or {}
        after = report.get('bridge_status_after') or {}
        if all(isinstance(before.get(key), int) and isinstance(after.get(key), int)
               for key in ('sent', 'dropped')):
            report['bridge_counter_delta'] = {
                key: after[key]-before[key] for key in ('sent', 'dropped')}
        if exit_code and report['completion'] == 'stream_completed':
            report['completion'] = 'cleanup_failed'
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        try:
            data_path, plot_path = save_result(report, stem)
            print(f'Data: {data_path}\nPlot: {plot_path}')
        except Exception as exc:
            print(f'Result saving failed: {exc}; JSON path: {stem.with_suffix(".json")}')
            exit_code = 1
        if report.get('error'):
            print('Test error:', report['error'])
        if report.get('pre_run_reset_performed'):
            print('Previous bridge stream fault reset after STOPPED feedback check.')
        if report['published']:
            print('Inspect bridge/status; the next --execute will reset a latched stream fault only after STOPPED.')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
