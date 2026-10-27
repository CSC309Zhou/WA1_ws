#!/usr/bin/env python3
"""Operator-run Shoulder_Y_L absolute MoveJ test; preview by default.

Preview mode never creates a service client.
"""

import argparse
import json
import math
import os
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from upperlimb.srv import MoveJ


JOINTS = ('Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L',
          'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L')
DEFAULT_TARGET = 0.1  # absolute radian position, not an increment
TARGET_LOWER = -3.197  # conservative limit from the supplied WA1 teleoperation guide
TARGET_UPPER = 1.145
POSITION_TOLERANCE = 0.02
V = 0.2
ACC = 0.2
T = 0.0
SERVICE = '/upperlimb/movej/left_arm'
MASTER = os.environ.get('WA1_ROS1_MASTER_URI', 'http://192.168.217.1:11311').rstrip('/')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=float, default=DEFAULT_TARGET,
                        help='Absolute Shoulder_Y_L position in rad (default: 0.1)')
    parser.add_argument('--execute', action='store_true',
                        help='Send one MoveJ request to the target')
    args = parser.parse_args()
    if not math.isfinite(args.target) or not TARGET_LOWER <= args.target <= TARGET_UPPER:
        parser.error(f'--target must be finite and within [{TARGET_LOWER}, {TARGET_UPPER}] rad')

    rclpy.init()
    node = rclpy.create_node('wa1_shoulder_y_l_movej_operator')
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
        if not all(name in positions for name in JOINTS):
            raise RuntimeError('Missing left-arm joint in feedback')
        return [positions[name] for name in JOINTS], status

    def target_from(current):
        target = current.copy()
        target[0] = args.target
        return target

    def send_once(client, current, goal):
        request = MoveJ.Request(joints=goal, v=V, acc=ACC, t=T,
                                is_async=False, arm_type=1)
        print(json.dumps({'sending_once_to': SERVICE,
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
        # A successful service response alone is not proof of arrival.
        until = time.monotonic() + 2.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        observed, _ = snapshot()
        errors = [abs(actual - expected) for actual, expected in zip(observed, goal)]
        reached = all(error <= POSITION_TOLERANCE for error in errors)
        print(json.dumps({'observed': observed,
                          'goal': goal, 'absolute_errors_rad': errors,
                          'within_0p02_rad': reached}, ensure_ascii=False, indent=2))
        return reached

    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not fresh():
            rclpy.spin_once(node, timeout_sec=.05)
        current, status = snapshot()
        target = target_from(current)
        request_data = dict(mode='EXECUTE' if args.execute else 'PREVIEW_ONLY',
                            service=SERVICE, joint_names=JOINTS, current=current,
                            target=target, delta=args.target-current[0],
                            v=V, acc=ACC, t=T, is_async=False, arm_type=1,
                            bridge_read_only=status.get('read_only'))
        print(json.dumps(request_data, ensure_ascii=False, indent=2), flush=True)
        if not args.execute:
            print('Preview complete. No MoveJ service request was sent.')
            return 0
        if status.get('read_only') is not False:
            raise RuntimeError('Control bridge is not running')

        client = node.create_client(MoveJ, SERVICE)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveJ service is unavailable')
        # Rebuild from fresh feedback so all six untouched joints remain current.
        refresh_deadline = time.monotonic() + 2.0
        while time.monotonic() < refresh_deadline and not fresh():
            rclpy.spin_once(node, timeout_sec=.02)
        current, status = snapshot()
        if status.get('read_only') is not False:
            raise RuntimeError('Control bridge changed mode before request')
        if abs(args.target - current[0]) < .001:
            raise RuntimeError('Target is already within 0.001 rad of current Shoulder_Y_L')
        target = target_from(current)
        send_once(client, current, target)
        reached = observe_goal(target)
        return 0 if reached else 2
    except KeyboardInterrupt:
        print('Interrupted locally; this does not confirm robot motion has stopped.')
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
