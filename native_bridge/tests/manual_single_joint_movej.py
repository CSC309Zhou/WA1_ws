#!/usr/bin/env python3
"""Operator-run first physical MoveJ test. Default is preview, no motion request.

Only --execute can call the robot-facing MoveJ service. This script has not been
used to command the real WA1; it intentionally excludes whole_body and unlock.
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

MASTER = os.environ.get('WA1_ROS1_MASTER_URI', 'http://192.168.217.1:11311').rstrip('/')

PARTS = {
    'left_arm': (['Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L',
                  'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L'], 1),
    'right_arm': (['Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R',
                   'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R'], 2),
    'neck': (['Neck_Z', 'Neck_Y'], 4),
    'waist': (['Waist_Z', 'Waist_Y'], 8),
    'lifting': (['Lifting_Z'], 16),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint-name', required=True,
                        choices=[name for names, _ in PARTS.values() for name in names])
    parser.add_argument('--delta', required=True, type=float,
                        help='Signed change: radians for rotation, meters for Lifting_Z')
    parser.add_argument('--execute', action='store_true',
                        help='Actually call the real MoveJ service; omitted means preview')
    parser.add_argument('--v', type=float, help='Robot MoveJ velocity parameter; required with --execute')
    parser.add_argument('--acc', type=float, help='Robot MoveJ acceleration parameter; required with --execute')
    parser.add_argument('--t', type=float, help='Robot MoveJ time parameter; required with --execute')
    args = parser.parse_args()
    limit = .002 if args.joint_name == 'Lifting_Z' else .01
    if not math.isfinite(args.delta) or args.delta == 0 or abs(args.delta) > limit:
        parser.error(f'nonzero finite |delta| must be at most {limit} for this first-step test')
    if args.execute and any(x is None or not math.isfinite(x) or x <= 0
                            for x in (args.v, args.acc, args.t)):
        parser.error('--execute requires positive finite --v, --acc and --t')

    rclpy.init()
    node = rclpy.create_node('wa1_manual_single_joint_preview')
    latest = {'state': None, 'occupancy': None, 'status': None}
    subs = []

    def update(key, value):
        latest[key] = (value, time.monotonic())

    def feedback_fresh():
        now = time.monotonic()
        return all(latest.values()) and now - latest['state'][1] <= .25 and \
            now - latest['occupancy'][1] <= .25 and now - latest['status'][1] <= 2

    subs.append(node.create_subscription(
        JointState, '/upperlimb/joint_states', lambda m: update('state', m),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    subs.append(node.create_subscription(
        Int8, '/upperlimb/occupancy', lambda m: update('occupancy', m),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    subs.append(node.create_subscription(
        String, '/upperlimb/bridge/status', lambda m: update('status', json.loads(m.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not feedback_fresh():
            rclpy.spin_once(node, timeout_sec=.05)
        if not feedback_fresh():
            raise RuntimeError('Missing or stale joint, occupancy or bridge status feedback')
        if node.count_publishers('/upperlimb/bridge/status') != 1:
            raise RuntimeError('Expected exactly one bridge status publisher')
        client = None
        if args.execute:
            client = node.create_client(MoveJ, f'/upperlimb/movej/{next(part for part, (names, _) in PARTS.items() if args.joint_name in names)}')
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError('MoveJ service unavailable')
            refresh_deadline = time.monotonic() + 2
            while time.monotonic() < refresh_deadline and not feedback_fresh():
                rclpy.spin_once(node, timeout_sec=.02)
        if not feedback_fresh():
            raise RuntimeError('Feedback/status is stale; do not use this target')
        state, occupancy, status = (latest[k][0] for k in ('state', 'occupancy', 'status'))
        if status.get('ros1_master_uri', '').rstrip('/') != MASTER:
            raise RuntimeError('Bridge is not connected to the expected real ROS1 Master')
        if not status.get('feedback_ready') or occupancy.data != 0:
            raise RuntimeError('Bridge feedback is not ready or upperlimb is occupied')
        if len(state.name) != 19 or len(state.position) != 19 or len(set(state.name)) != 19:
            raise RuntimeError('Expected 19 unique joint names and positions')
        if not all(math.isfinite(x) for x in state.position):
            raise RuntimeError('Nonfinite joint feedback')
        name_to_position = dict(zip(state.name, state.position))
        part, names, mask = next((part, names, mask) for part, (names, mask) in PARTS.items()
                                 if args.joint_name in names)
        if not all(name in name_to_position for name in names):
            raise RuntimeError('Missing target group joint name in live feedback')
        target = [name_to_position[name] for name in names]
        target[names.index(args.joint_name)] += args.delta
        print(json.dumps({'mode': 'execute' if args.execute else 'PREVIEW_ONLY',
                          'part': part, 'arm_type': mask, 'joint_name': args.joint_name,
                          'delta': args.delta, 'current': [name_to_position[n] for n in names],
                          'target': target, 'service': f'/upperlimb/movej/{part}',
                          'robot_stamp': {'sec': state.header.stamp.sec,
                                          'nanosec': state.header.stamp.nanosec}}, indent=2))
        if not args.execute:
            print('Preview complete. No MoveJ service request was sent.')
            return
        if status.get('read_only') or status.get('stream_fault'):
            raise RuntimeError('Control bridge is not ready')
        # Recheck live feedback immediately before issuing the one request.
        rclpy.spin_once(node, timeout_sec=.01)
        if not feedback_fresh() or latest['occupancy'][0].data != 0:
            raise RuntimeError('Feedback became stale/occupied before request')
        fresh_positions = dict(zip(latest['state'][0].name, latest['state'][0].position))
        if any(abs(fresh_positions[name] - name_to_position[name]) > .005 for name in names):
            raise RuntimeError('Joint feedback moved since target calculation; rerun preview')
        request = MoveJ.Request(joints=target, v=args.v, acc=args.acc, t=args.t,
                                is_async=False, arm_type=mask)
        response_future = client.call_async(request)
        rclpy.spin_until_future_complete(node, response_future, timeout_sec=max(10., args.t + 10.))
        if not response_future.done() or response_future.result() is None:
            raise RuntimeError('MoveJ response timed out; inspect robot state before doing anything else')
        response = response_future.result()
        print(json.dumps({'service_success': response.success, 'service_message': response.message},
                         ensure_ascii=False, indent=2))
        if not response.success:
            raise RuntimeError('Robot rejected MoveJ; inspect state before retrying')
        print('Service accepted the request; confirm actual motion and final position separately.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
