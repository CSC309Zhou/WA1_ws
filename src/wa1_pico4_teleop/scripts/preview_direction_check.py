#!/usr/bin/env python3
"""Read-only XR hand and TCP target deltas while a grip is held; publishes nothing."""

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from pico4_ros2.msg import XrFrame
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class DirectionCheck(Node):
    def __init__(self, side):
        super().__init__('wa1_teleop_preview_direction_check')
        self.side = side
        self.grip = 0.0
        self.baseline = None
        self.raw_baseline = None
        self.raw_now = None
        self.target = None
        self.target_at = 0.0
        self.create_subscription(XrFrame, '/pico4/frame', self.on_frame, SENSOR_QOS)
        self.create_subscription(PoseStamped, f'/wa1/teleop/target/{side}',
                                 self.on_target, SENSOR_QOS)
        self.create_timer(0.25, self.print_delta)

    def on_frame(self, frame):
        controller = frame.left_controller if self.side == 'left' else frame.right_controller
        self.grip = controller.gripper
        self.raw_now = tuple(controller.tracked_pose.pose[:3])
        if self.grip < 0.6:
            self.baseline = None
            self.raw_baseline = None
        elif self.grip >= 0.8 and self.raw_baseline is None:
            self.raw_baseline = self.raw_now

    def on_target(self, msg):
        position = msg.pose.position
        self.target = (position.x, position.y, position.z)
        self.target_at = time.monotonic()
        if self.grip >= 0.8 and self.baseline is None:
            self.baseline = self.target

    def print_delta(self):
        if self.baseline is None or self.raw_baseline is None or \
                self.target is None or self.grip < 0.8 or \
                time.monotonic() - self.target_at > 0.5:
            print(f'{self.side}: waiting for a held grip and fresh target', flush=True)
            return
        delta = tuple(now - start for now, start in zip(self.target, self.baseline))
        raw = tuple(now - start for now, start in zip(self.raw_now, self.raw_baseline))
        expected = (-raw[2], -raw[0], raw[1])
        print(f'{self.side} XR raw: x={raw[0]:+.4f} y={raw[1]:+.4f} z={raw[2]:+.4f} m | '
              f'official R: X={expected[0]:+.4f} Y={expected[1]:+.4f} '
              f'Z={expected[2]:+.4f} m | '
              f'TCP target: X前={delta[0]:+.4f} Y左={delta[1]:+.4f} '
              f'Z上={delta[2]:+.4f} m', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    args = parser.parse_args()
    rclpy.init()
    node = DirectionCheck(args.side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
