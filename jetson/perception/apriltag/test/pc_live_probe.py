#!/usr/bin/env python3
"""Read-only PC ROS2 check for the live WA1 AprilTag pose pipeline."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


CALIBRATION = (Path(__file__).resolve().parents[1] /
               'csc309_apriltag_pose/config/calibration_vendor_run01.json')


def quaternion(pose):
    q = pose.orientation
    values = [q.x, q.y, q.z, q.w]
    norm = math.sqrt(sum(x * x for x in values))
    return [x / norm for x in values]


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return [vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx]


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


class Probe(Node):
    def __init__(self, calibration):
        super().__init__('wa1_apriltag_live_probe')
        self.cal = calibration
        self.pending_camera = {}
        self.pending_base = {}
        self.arrivals = {'camera': [], 'base': []}
        self.position_errors = []
        self.orientation_errors = []
        self.detected = []
        self.base_valid = []
        self.status = None
        self.joints = None
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, '/perception/apriltag/tag0/pose_camera',
                                 self.on_camera, qos)
        self.create_subscription(PoseStamped, '/perception/apriltag/tag0/pose_base',
                                 self.on_base, qos)
        self.create_subscription(Bool, '/perception/apriltag/tag0/detected',
                                 lambda m: self.detected.append(m.data), qos)
        self.create_subscription(Bool, '/perception/apriltag/tag0/base_valid',
                                 lambda m: self.base_valid.append(m.data), qos)
        self.create_subscription(String, '/perception/apriltag/tag0/status',
                                 self.on_status, qos)
        self.create_subscription(JointState, '/upperlimb/joint_states',
                                 self.on_joints, qos)

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

    def on_camera(self, message):
        self.arrivals['camera'].append(time.monotonic())
        key = self.stamp(message)
        self.pending_camera[key] = message
        self.match(key)

    def on_base(self, message):
        self.arrivals['base'].append(time.monotonic())
        key = self.stamp(message)
        self.pending_base[key] = message
        self.match(key)

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_joints(self, message):
        self.joints = dict(zip(message.name, message.position))

    def match(self, key):
        if key not in self.pending_camera or key not in self.pending_base:
            return
        camera = self.pending_camera.pop(key)
        base = self.pending_base.pop(key)
        if camera.header.frame_id != self.cal['camera_frame']:
            raise ValueError('Unexpected camera frame')
        if base.header.frame_id != self.cal['base_frame']:
            raise ValueError('Unexpected BASE frame')
        extrinsic = self.cal['base_to_camera']
        parent_q = extrinsic['quaternion_xyzw']
        child_xyz = [camera.pose.position.x, camera.pose.position.y, camera.pose.position.z]
        expected_xyz = [a + b for a, b in zip(
            extrinsic['translation_m'], rotate(parent_q, child_xyz))]
        actual_xyz = [base.pose.position.x, base.pose.position.y, base.pose.position.z]
        self.position_errors.append(math.dist(expected_xyz, actual_xyz))
        expected_q = multiply(parent_q, quaternion(camera.pose))
        actual_q = quaternion(base.pose)
        dot = min(1.0, abs(sum(a * b for a, b in zip(expected_q, actual_q))))
        self.orientation_errors.append(2.0 * math.acos(dot))
        for pending in (self.pending_camera, self.pending_base):
            if len(pending) > 100:
                pending.pop(next(iter(pending)))


def rate(times):
    return None if len(times) < 2 else round((len(times) - 1) / (times[-1] - times[0]), 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    calibration = json.loads(CALIBRATION.read_text(encoding='utf-8'))
    rclpy.init()
    probe = Probe(calibration)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.1)
        deviations = {}
        if probe.joints is not None:
            deviations = {name: round(probe.joints[name] - target, 7)
                          for name, target in calibration['fixed_joints'].items()
                          if name in probe.joints}
        result = {
            'seconds': args.seconds,
            'camera_samples': len(probe.arrivals['camera']),
            'base_samples': len(probe.arrivals['base']),
            'matched_stamps': len(probe.position_errors),
            'camera_hz': rate(probe.arrivals['camera']),
            'base_hz': rate(probe.arrivals['base']),
            'detected_true_fraction': (sum(probe.detected) / len(probe.detected)
                                       if probe.detected else None),
            'base_valid_true_fraction': (sum(probe.base_valid) / len(probe.base_valid)
                                         if probe.base_valid else None),
            'position_transform_error_max_m': (max(probe.position_errors)
                                               if probe.position_errors else None),
            'position_transform_error_median_m': (statistics.median(probe.position_errors)
                                                  if probe.position_errors else None),
            'orientation_transform_error_max_rad': (max(probe.orientation_errors)
                                                    if probe.orientation_errors else None),
            'fixed_joint_deviation': deviations,
            'latest_status': probe.status,
        }
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if (len(probe.position_errors) < 10 or not probe.status or
                not probe.status['camera_info_ok'] or probe.status['fixed_joint_reason'] or
                max(probe.position_errors) > 1e-5 or
                max(probe.orientation_errors) > 1e-4):
            return 1
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
