#!/usr/bin/env python3
"""Read-only check of ROS2 hand feedback and service availability."""

import argparse
import json
import time

import rclpy
from hand.msg import PressureSensor
from hand.srv import Gesture, HandJoint
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

PREFIX = "/hand/"
BASE_NAMES = (
    "THUMB_MP", "THUMB_CMC", "INDEX_MCP",
    "MIDDLE_MCP", "RING_MCP", "LITTLE_MCP")
EXPECTED_NAMES = [
    name + "_" + side for side in ("LEFT", "RIGHT") for name in BASE_NAMES
]


class Probe(Node):
    def __init__(self):
        super().__init__("wa1_hand_probe")
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joints = None
        self.pressures = {}
        self.counts = {"joint_states": 0, "left": 0, "right": 0}
        self.create_subscription(
            JointState, PREFIX + "joint_states", self.on_joints, sensor_qos)
        for side in ("left", "right"):
            self.create_subscription(
                PressureSensor, PREFIX + "finger_pressures/" + side,
                lambda message, side=side: self.on_pressure(side, message), sensor_qos)
        self.service_clients = {}
        for side in ("left", "right", "dual"):
            for group, service_type in (
                ("joint_switch", HandJoint), ("gesture_switch", Gesture)
            ):
                suffix = group + "/" + side
                self.service_clients[suffix] = self.create_client(service_type, PREFIX + suffix)
        for side in ("left", "right"):
            suffix = "finger_pressures/" + side + "/zero"
            self.service_clients[suffix] = self.create_client(Trigger, PREFIX + suffix)

    def on_joints(self, message):
        self.joints = message
        self.counts["joint_states"] += 1

    def on_pressure(self, side, message):
        self.pressures[side] = message
        self.counts[side] += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.joints is None or list(node.joints.name) != EXPECTED_NAMES:
            raise RuntimeError("Missing or unexpected 12-joint hand feedback")
        if len(node.joints.position) != 12:
            raise RuntimeError("Hand joint position count is not 12")
        for side in ("left", "right"):
            message = node.pressures.get(side)
            if message is None or len(message.pressure) != 6:
                raise RuntimeError("Missing six-value pressure feedback for " + side)
        unavailable = [
            suffix for suffix, client in node.service_clients.items()
            if not client.service_is_ready()
        ]
        if unavailable:
            raise RuntimeError("Hand services unavailable: " + ", ".join(unavailable))
        report = {
            "mode": "read_only",
            "joint_names": list(node.joints.name),
            "joint_position_count": len(node.joints.position),
            "joint_velocity_count": len(node.joints.velocity),
            "pressure_lengths": {
                side: len(node.pressures[side].pressure) for side in ("left", "right")
            },
            "sample_counts": node.counts,
            "services_ready": sorted(node.service_clients),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
