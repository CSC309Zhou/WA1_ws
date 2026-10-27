#!/usr/bin/env python3
"""Preview or send one right-arm MoveL target in the BASE Cartesian frame.

Input is six-dimensional: xyz in metres and fixed-axis XYZ RPY in radians.
The wire request uses geometry_msgs/Pose (xyz plus quaternion). Only --execute
sends a MoveL request.
"""

import argparse
import json
import math
import os
import time


RIGHT_JOINTS = (
    'Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R',
    'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R',
)
MASTER = os.environ.get('WA1_ROS1_MASTER_URI', 'http://192.168.217.1:11311').rstrip('/')


def quat_unit(q):
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid quaternion')
    return [value / norm for value in q]


def quat_from_rpy(roll, pitch, yaw):
    """Fixed-axis XYZ rotation: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return quat_unit([
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    ])


def quat_multiply(a, b):
    """Hamilton product a ⊗ b, both in xyzw order."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return quat_unit([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def pose_values(pose, feedback=False):
    rotation = pose.quaternion if feedback else pose.orientation
    xyz = [pose.position.x, pose.position.y, pose.position.z]
    q = [rotation.x, rotation.y, rotation.z, rotation.w]
    if not all(math.isfinite(v) for v in xyz + q):
        raise RuntimeError('Non-finite TCP position or quaternion')
    norm = math.sqrt(sum(v*v for v in q))
    if not 0.98 <= norm <= 1.02:
        raise RuntimeError(f'Invalid TCP quaternion norm: {norm}')
    return xyz, quat_unit(q)


def write_pose(pose, xyz, q):
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q


def pose_json(pose, feedback=False):
    xyz, q = pose_values(pose, feedback)
    return {'position_m': xyz, 'quaternion_xyzw': q}


def error_to(target, observed, observed_is_feedback=False):
    p1, q1 = pose_values(target)
    p2, q2 = pose_values(observed, observed_is_feedback)
    cosine = min(1.0, abs(sum(a*b for a, b in zip(q1, q2))))
    return {'position_error_m': math.dist(p1, p2),
            'orientation_error_deg': 2*math.degrees(math.acos(cosine))}


def make_target(current_tcp, args, pose_type):
    current_xyz, current_q = pose_values(current_tcp, feedback=True)
    if args.delta is not None:
        dx, dy, dz, droll, dpitch, dyaw = args.delta
        xyz = [value + change for value, change in zip(current_xyz, (dx, dy, dz))]
        # BASE-frame rotation: q_delta is applied before current orientation.
        q = quat_multiply(quat_from_rpy(droll, dpitch, dyaw), current_q)
    else:
        x, y, z, roll, pitch, yaw = args.absolute
        xyz = [x, y, z]
        q = quat_from_rpy(roll, pitch, yaw)
    result = pose_type()
    write_pose(result, xyz, q)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--delta', nargs=6, type=float,
                        metavar=('DX', 'DY', 'DZ', 'DROLL', 'DPITCH', 'DYAW'),
                        help='BASE-frame TCP increment; metres then radians')
    target.add_argument('--absolute', nargs=6, type=float,
                        metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                        help='Absolute BASE-frame TCP pose; metres then radians')
    parser.add_argument('--v', type=float, default=0.05)
    parser.add_argument('--acc', type=float, default=0.1)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true',
                        help='Send exactly one synchronous MoveL request')
    action.add_argument('--check-only', action='store_true',
                        help='Check live execution prerequisites without sending a MoveL request')
    args = parser.parse_args()
    six = args.delta if args.delta is not None else args.absolute
    if not all(math.isfinite(v) for v in six):
        parser.error('All six Cartesian inputs must be finite')
    if args.delta is not None and not any(v != 0 for v in six):
        parser.error('--delta must change at least one component')
    if not math.isfinite(args.v) or args.v <= 0 or not math.isfinite(args.acc) or args.acc <= 0:
        parser.error('--v and --acc must be finite and positive')

    # No FK, joint-target calculation, or MoveL client in preview mode.
    import rclpy
    from geometry_msgs.msg import Pose as GeometryPose
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8, String
    from upperlimb.msg import Pose as TcpPose, UplimbState
    from upperlimb.srv import MoveL

    rclpy.init()
    node = rclpy.create_node('wa1_right_arm_movel_operator')
    latest = {}
    subscriptions = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def update(key, message):
        latest[key] = (message, time.monotonic_ns())

    for cls, name, key in (
        (JointState, '/upperlimb/joint_states', 'joint'),
        (TcpPose, '/upperlimb/tcp_pose/left', 'left'),
        (TcpPose, '/upperlimb/tcp_pose/right', 'right'),
        (UplimbState, '/upperlimb/uplimb_state', 'mode'),
        (Int8, '/upperlimb/occupancy', 'occupancy'),
    ):
        subscriptions.append(node.create_subscription(
            cls, name, lambda msg, which=key: update(which, msg), qos))
    subscriptions.append(node.create_subscription(
        String, '/upperlimb/bridge/status',
        lambda msg: update('status', json.loads(msg.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def snapshot():
        needed = ('joint', 'left', 'right', 'mode', 'occupancy', 'status')
        if any(key not in latest for key in needed):
            raise RuntimeError('Joint, TCP, mode, occupancy, or bridge feedback is missing')
        now = time.monotonic_ns()
        for key in needed:
            maximum_age = 2_000_000_000 if key == 'status' else 500_000_000
            if now - latest[key][1] > maximum_age:
                raise RuntimeError(f'{key} feedback is stale')
        values = {key: latest[key][0] for key in needed}
        if values['status'].get('ros1_master_uri', '').rstrip('/') != MASTER:
            raise RuntimeError('Bridge is not connected to the real ROS1 Master')
        state = values['joint']
        if len(state.name) != 19 or len(state.position) != 19 or len(set(state.name)) != 19:
            raise RuntimeError('Expected 19 unique joint names and positions')
        values['joints'] = dict(zip(state.name, state.position))
        if any(name not in values['joints'] for name in RIGHT_JOINTS):
            raise RuntimeError('Right-arm feedback is incomplete')
        if not all(math.isfinite(v) for v in state.position):
            raise RuntimeError('JointState contains a non-finite position')
        return values

    def build_request(feedback, target_pose):
        request = MoveL.Request()
        write_pose(request.pose[0], *pose_values(target_pose))
        # The right_arm endpoint uses pose[0]. Keep the other fixed array entry
        # at the current left TCP instead of leaving a zero quaternion.
        write_pose(request.pose[1], *pose_values(feedback['left'], feedback=True))
        request.v, request.acc, request.is_async = args.v, args.acc, False
        return request

    def report(mode, feedback, request):
        return {
            'mode': mode, 'service': '/upperlimb/movel/right_arm',
            'input_kind': 'BASE_frame_delta' if args.delta is not None else 'absolute_BASE_pose',
            'input_xyz_m_and_rpy_rad': six,
            'current_right_tcp': pose_json(feedback['right'], feedback=True),
            'target_right_tcp_pose_0': pose_json(request.pose[0]),
            'left_current_pose_1_unused': pose_json(request.pose[1]),
            'target_change': error_to(request.pose[0], feedback['right'], True),
            'v': args.v, 'acc': args.acc, 'is_async': False,
            'upperlimb_mode': [int(feedback['mode'].cmd_num), feedback['mode'].cmd_name],
            'occupancy': int(feedback['occupancy'].data),
            'bridge_read_only': feedback['status'].get('read_only'),
            'bridge_feedback_ready': feedback['status'].get('feedback_ready'),
            'bridge_stream_fault': feedback['status'].get('stream_fault'),
        }

    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and len(latest) < 6:
            rclpy.spin_once(node, timeout_sec=0.05)
        feedback = snapshot()
        target_pose = make_target(feedback['right'], args, GeometryPose)
        request = build_request(feedback, target_pose)
        mode = 'EXECUTE_ONCE' if args.execute else 'CHECK_ONLY' if args.check_only else 'PREVIEW_ONLY'
        print(json.dumps(report(mode,
                                feedback, request), ensure_ascii=False, indent=2), flush=True)
        if not (args.execute or args.check_only):
            print('Preview complete. No MoveL request was sent.')
            return 0

        if os.environ.get('ROS_DOMAIN_ID', '0') != '0':
            raise RuntimeError('--execute and --check-only require the real PC DDS domain 0')
        client = node.create_client(MoveL, '/upperlimb/movel/right_arm')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Right-arm MoveL service is unavailable')
        rclpy.spin_once(node, timeout_sec=0.05)
        feedback = snapshot()
        status = feedback['status']
        problems = []
        if status.get('read_only') is not False:
            problems.append(f"bridge read_only={status.get('read_only')!r}")
        if status.get('feedback_ready') is not True:
            problems.append(f"bridge feedback_ready={status.get('feedback_ready')!r}")
        if status.get('stream_fault'):
            problems.append(f"bridge stream_fault={status.get('stream_fault')!r}")
        if feedback['occupancy'].data != 0:
            problems.append(f"upper limb occupancy={feedback['occupancy'].data!r}")
        # The ROS2 Python binding represents the IDL `byte STOPPED=0` constant
        # as b'\x00', while the `int8 cmd_num` feedback field is an integer.
        stopped_code = UplimbState.STOPPED
        if isinstance(stopped_code, bytes):
            stopped_code = stopped_code[0]
        if feedback['mode'].cmd_num != stopped_code:
            problems.append(f"upper limb mode={feedback['mode'].cmd_num!r} "
                            f"({feedback['mode'].cmd_name})")
        if problems:
            raise RuntimeError('Preflight failed: ' + '; '.join(problems))
        if node.count_publishers('/upperlimb/bridge/status') != 1:
            raise RuntimeError('Expected exactly one PC bridge')
        # Recompute relative target from the latest TCP immediately before use.
        target_pose = make_target(feedback['right'], args, GeometryPose)
        request = build_request(feedback, target_pose)
        if args.check_only:
            print(json.dumps({'preflight': 'PASSED', 'request_sent': False,
                              'service': '/upperlimb/movel/right_arm',
                              'pose_0_right_target': pose_json(request.pose[0]),
                              'pose_1_left_current_unused': pose_json(request.pose[1]),
                              'v': request.v, 'acc': request.acc,
                              'is_async': request.is_async}, indent=2), flush=True)
            return 0
        print(json.dumps({'sending_once_to': '/upperlimb/movel/right_arm',
                          'pose_0_right_target': pose_json(request.pose[0]),
                          'pose_1_left_current_unused': pose_json(request.pose[1]),
                          'v': request.v, 'acc': request.acc,
                          'is_async': request.is_async}, indent=2), flush=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('MoveL response timed out; do not retry automatically')
        response = future.result()
        print(json.dumps({'service_success': response.success,
                          'service_message': response.message}, ensure_ascii=False, indent=2))
        if not response.success:
            raise RuntimeError('MoveL was rejected')
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        observed = snapshot()
        print(json.dumps({
            'observed_right_tcp': pose_json(observed['right'], feedback=True),
            'target_right_tcp': pose_json(target_pose),
            'tcp_error': error_to(target_pose, observed['right'], True),
            'right_joint_delta_rad': {
                name: observed['joints'][name] - feedback['joints'][name]
                for name in RIGHT_JOINTS},
        }, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        print('Interrupted locally; this does not confirm that robot motion stopped.')
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
