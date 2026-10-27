#!/usr/bin/env python3
"""Read-only PICO4/WA1 teleoperation status monitor; creates no control entities."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from pico4_ros2.msg import XrFrame


SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
STATUS_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StatusMonitor(Node):
    def __init__(self):
        super().__init__('wa1_teleop_read_only_monitor')
        self.frame = None
        self.frame_at = 0.0
        self.teleop = None
        self.teleop_at = 0.0
        self.pico = None
        self.create_subscription(XrFrame, '/pico4/frame', self.on_frame, SENSOR_QOS)
        self.create_subscription(String, '/wa1/teleop/status', self.on_teleop, STATUS_QOS)
        self.create_subscription(String, '/pico4/status', self.on_pico, STATUS_QOS)
        self.create_timer(0.5, self.print_line)

    def on_frame(self, msg):
        self.frame = msg
        self.frame_at = time.monotonic()

    def on_teleop(self, msg):
        try:
            self.teleop = json.loads(msg.data)
            self.teleop_at = time.monotonic()
        except (TypeError, ValueError):
            pass

    def on_pico(self, msg):
        try:
            self.pico = json.loads(msg.data)
        except (TypeError, ValueError):
            pass

    def print_line(self):
        now = time.monotonic()
        if self.frame is None:
            input_text = 'XR: no /pico4/frame'
        else:
            left = self.frame.left_controller
            right = self.frame.right_controller
            input_text = (
                f'XR age={1000*(now-self.frame_at):.0f}ms '
                f'A={int(right.primary_button)} B={int(right.secondary_button)} '
                f'left_grip={left.gripper:.2f} right_grip={right.gripper:.2f}')
        if self.teleop is None:
            control_text = 'teleop: no /wa1/teleop/status'
        else:
            s = self.teleop
            control_text = (
                f'status age={1000*(now-self.teleop_at):.0f}ms '
                f'scale={s.get("translation_scale", "?")} '
                f'sensitivity={s.get("sensitivity_level", "?")} '
                f'unlocked={int(s.get("software_unlocked", False))} '
                f'Aseen={int(s.get("a_button_seen", False))} '
                f'Apresses={s.get("a_button_press_count", "?")} '
                f'Aaction={s.get("a_button_last_action", "?")} '
                f'release_required={int(s.get("release_required", False))} '
                f'initial_done={int(s.get("initial_done", False))} '
                f'Bhold={s.get("b_hold_ms", "?")}ms '
                f'Brequests={s.get("b_return_movej_requests", "?")} '
                f'left_active={int(s.get("left_grip_active", False))} '
                f'right_active={int(s.get("right_grip_active", False))} '
                f'target={int(s.get("left_target_valid", False))} '
                f'xr_fresh={int(s.get("xr_fresh", False))} '
                f'state={s.get("control_state", "?")} '
                f'cmd={s.get("commands_published", "?")} '
                f'robot_mode={s.get("robot_mode", "?")} '
                f'bridge_ready={int(s.get("bridge_ready", False))} '
                f'bridge_fault={int(s.get("bridge_stream_fault", False))} '
                f'recovery_ready={int(s.get("recovery_ready", False))} '
                f'xr_timeouts={s.get("xr_timeout_count", "?")} '
                f'XRstampRepeats={s.get("xr_device_timestamp_nonmonotonic_count", "?")} '
                f'recovery_block={s.get("recovery_block_reason", "")!r} '
                f'control_block={s.get("control_block_reason", "")!r} '
                f'reason={s.get("reason", "")!r}')
        if self.pico is not None:
            input_text += (
                f' SDK_frames={self.pico.get("frames", "?")} '
                f'SDK_age={self.pico.get("last_frame_age_ms", "?")}ms '
                f'SDK_connected={int(self.pico.get("server_connected", False))} '
                f'device_online={int(self.pico.get("device_online", False))}')
        print(input_text + ' | ' + control_text, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=0.0,
                        help='Stop after this many seconds; default is until Ctrl-C')
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error('--seconds must be nonnegative')
    rclpy.init()
    monitor = StatusMonitor()
    deadline = time.monotonic() + args.seconds if args.seconds else None
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(monitor, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
