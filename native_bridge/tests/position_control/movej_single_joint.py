"""Shared operator-run MoveJ test for one WA1 upper-limb joint.

Each manual_*.py entry point selects a fixed joint. Without --execute this
module only reads feedback; a real MoveJ request is sent at most once.
"""

import argparse
import json
import math
import os
import time


# The order within each part is the WA1 MoveJ request order, not an assumed
# index into JointState. Feedback is always mapped by joint name.
PARTS = {
    'left_arm': (('Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L',
                  'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L'), 1),
    'right_arm': (('Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R',
                   'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R'), 2),
    'neck': (('Neck_Z', 'Neck_Y'), 4),
    'waist': (('Waist_Z', 'Waist_Y'), 8),
    'lifting': (('Lifting_Z',), 16),
}

# Intersections of the supplied WA1 URDF bounds and the teleoperation guide's
# example middleware bounds. These are NOT verified controller safety limits.
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
MASTER = os.environ.get('WA1_ROS1_MASTER_URI', 'http://192.168.217.1:11311').rstrip('/')


def part_for(joint_name):
    for part, (names, arm_type) in PARTS.items():
        if joint_name in names:
            return part, names, arm_type
    raise ValueError(f'Unknown WA1 upper-limb joint: {joint_name}')


def target_from(current, names, joint_name, absolute_target):
    if len(current) != len(names):
        raise ValueError('Current joint vector does not match the MoveJ group')
    target = list(current)
    target[names.index(joint_name)] = absolute_target
    return target


def main(joint_name):
    part, names, arm_type = part_for(joint_name)
    unit = 'm' if joint_name == 'Lifting_Z' else 'rad'
    lower, upper = TARGET_LIMITS[joint_name]
    service = f'/upperlimb/movej/{part}'
    # Lifting_Z is prismatic. Its supplied URDF speed limit is 0.05 m/s.
    v, acc = (0.02, 0.02) if joint_name == 'Lifting_Z' else (0.2, 0.2)
    t = 0.0
    tolerance = 0.002 if joint_name == 'Lifting_Z' else 0.02
    min_delta = 0.0005 if joint_name == 'Lifting_Z' else 0.001

    parser = argparse.ArgumentParser(
        description=f'One-shot absolute MoveJ test for {joint_name}; preview by default.')
    parser.add_argument('--target', type=float, required=True,
                        help=f'Absolute {joint_name} position in {unit}')
    parser.add_argument('--execute', action='store_true',
                        help='Send one MoveJ request to the real robot')
    args = parser.parse_args()
    if not math.isfinite(args.target) or not lower <= args.target <= upper:
        parser.error(f'--target must be finite and within [{lower}, {upper}] {unit}')

    # Keep ROS imports after argument validation so --help and rejected input
    # never initialize ROS or contact the robot.
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8, String
    from upperlimb.srv import MoveJ

    rclpy.init()
    node = rclpy.create_node(f'wa1_{joint_name.lower()}_movej_operator')
    latest = {'state': None, 'occupancy': None, 'status': None}
    subscriptions = []

    def update(key, value):
        latest[key] = (value, time.monotonic())

    subscriptions.append(node.create_subscription(
        JointState, '/upperlimb/joint_states', lambda msg: update('state', msg),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    subscriptions.append(node.create_subscription(
        Int8, '/upperlimb/occupancy', lambda msg: update('occupancy', msg),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    subscriptions.append(node.create_subscription(
        String, '/upperlimb/bridge/status', lambda msg: update('status', json.loads(msg.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def fresh():
        now = time.monotonic()
        return all(latest.values()) and all(
            now - latest[key][1] <= limit for key, limit in
            (('state', .25), ('occupancy', .25), ('status', 2.0)))

    def snapshot():
        if not fresh():
            raise RuntimeError('Joint, occupancy, or bridge status feedback is stale')
        if node.count_publishers('/upperlimb/bridge/status') != 1:
            raise RuntimeError('Expected exactly one bridge status publisher')
        state, occupancy, status = (latest[key][0] for key in ('state', 'occupancy', 'status'))
        if status.get('ros1_master_uri', '').rstrip('/') != MASTER:
            raise RuntimeError('Bridge is not connected to the expected real ROS1 Master')
        if not status.get('feedback_ready') or occupancy.data != 0 or status.get('stream_fault'):
            raise RuntimeError('Upper limb is occupied, faulted, or feedback is not ready')
        if len(state.name) != 19 or len(state.position) != 19 or len(set(state.name)) != 19:
            raise RuntimeError('Expected 19 unique joint names and positions')
        if not all(math.isfinite(value) for value in state.position):
            raise RuntimeError('Non-finite joint feedback')
        positions = dict(zip(state.name, state.position))
        if not all(name in positions for name in names):
            raise RuntimeError(f'Missing {part} joint in feedback')
        return [positions[name] for name in names], status

    def send_once(client, current, goal):
        request = MoveJ.Request(joints=goal, v=v, acc=acc, t=t,
                                is_async=False, arm_type=arm_type)
        print(json.dumps({'sending_once_to': service, 'joint_name': joint_name,
                          'current': current, 'joints': list(request.joints),
                          'v': request.v, 'acc': request.acc, 't': request.t,
                          'is_async': request.is_async, 'arm_type': request.arm_type},
                         ensure_ascii=False, indent=2), flush=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('MoveJ response timed out; do not retry automatically')
        response = future.result()
        print(json.dumps({'service_success': response.success,
                          'service_message': response.message}, ensure_ascii=False, indent=2))
        if not response.success:
            raise RuntimeError('MoveJ was rejected')

    def observe_goal(goal):
        # Service success alone does not prove physical arrival.
        until = time.monotonic() + 2.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        observed, _ = snapshot()
        errors = [abs(actual - expected) for actual, expected in zip(observed, goal)]
        reached = all(error <= tolerance for error in errors)
        print(json.dumps({'observed': observed, 'goal': goal,
                          'absolute_errors': errors, 'unit': unit,
                          'within_tolerance': reached, 'tolerance': tolerance},
                         ensure_ascii=False, indent=2))
        return reached

    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not fresh():
            rclpy.spin_once(node, timeout_sec=.05)
        current, status = snapshot()
        goal = target_from(current, names, joint_name, args.target)
        print(json.dumps({'mode': 'EXECUTE' if args.execute else 'PREVIEW_ONLY',
                          'service': service, 'part': part, 'arm_type': arm_type,
                          'joint_name': joint_name, 'joint_names': names,
                          'unit': unit, 'target_limits': [lower, upper],
                          'current': current, 'target': goal,
                          'delta': args.target - current[names.index(joint_name)],
                          'v': v, 'acc': acc, 't': t, 'is_async': False,
                          'bridge_read_only': status.get('read_only')},
                         ensure_ascii=False, indent=2), flush=True)
        if not args.execute:
            print('Preview complete. No MoveJ service request was sent.')
            return 0
        if status.get('read_only') is not False:
            raise RuntimeError('Control bridge is not running')

        client = node.create_client(MoveJ, service)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveJ service is unavailable')
        # Rebuild from current named feedback; the other group axes stay current.
        refresh_deadline = time.monotonic() + 2.0
        while time.monotonic() < refresh_deadline and not fresh():
            rclpy.spin_once(node, timeout_sec=.02)
        current, status = snapshot()
        if status.get('read_only') is not False:
            raise RuntimeError('Control bridge changed mode before request')
        if abs(args.target - current[names.index(joint_name)]) < min_delta:
            raise RuntimeError(f'Target is already within {min_delta} {unit} of current {joint_name}')
        goal = target_from(current, names, joint_name, args.target)
        send_once(client, current, goal)
        return 0 if observe_goal(goal) else 2
    except KeyboardInterrupt:
        print('Interrupted locally; this does not confirm robot motion has stopped.')
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
