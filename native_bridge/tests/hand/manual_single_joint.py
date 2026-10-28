#!/usr/bin/env python3
"""Preview one absolute finger target, or send one HandJoint request with --execute."""

import argparse
import json
import math
import time

import rclpy
from hand.srv import HandJoint
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

JOINTS = (
    "THUMB_MP", "THUMB_CMC", "INDEX_MCP",
    "MIDDLE_MCP", "RING_MCP", "LITTLE_MCP")
LIMITS = {
    "THUMB_MP": (-0.7854, 0.7854),
    "THUMB_CMC": (-0.3491, 1.5708),
    **{name: (0.0, 1.3963) for name in JOINTS[2:]},
}


class HandClient(Node):
    def __init__(self, side):
        super().__init__("wa1_hand_single_joint_test")
        self.side = side
        self.latest = None
        self.received_at = 0.0
        self.create_subscription(
            JointState, "/hand/joint_states", self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.client = self.create_client(HandJoint, "/hand/joint_switch/" + side)

    def on_state(self, message):
        self.latest = message
        self.received_at = time.monotonic()

    def wait_state(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None and time.monotonic() - self.received_at < 0.5:
                return self.latest
        raise RuntimeError("No fresh /hand/joint_states feedback")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint", choices=JOINTS, required=True)
    parser.add_argument("--target", type=float, required=True, help="Absolute target in radians")
    parser.add_argument("--execute", action="store_true", help="Send one real HandJoint service request")
    args = parser.parse_args()
    if not math.isfinite(args.target):
        parser.error("--target must be finite")
    lower, upper = LIMITS[args.joint]
    if not lower <= args.target <= upper:
        parser.error(f"{args.joint} target must be in [{lower}, {upper}] rad")

    rclpy.init()
    node = HandClient(args.side)
    try:
        state = node.wait_state(5.0)
        by_name = dict(zip(state.name, state.position))
        suffix = args.side.upper()
        names = [name + "_" + suffix for name in JOINTS]
        if len(state.name) != len(state.position) or any(
            name not in by_name or not math.isfinite(by_name[name]) for name in names
        ):
            raise RuntimeError("Hand feedback lacks six finite positions for " + args.side)
        current = [float(by_name[name]) for name in names]
        target = current.copy()
        index = JOINTS.index(args.joint)
        target[index] = args.target
        preview = {
            "mode": "EXECUTE_ONCE" if args.execute else "PREVIEW_ONLY",
            "service": "/hand/joint_switch/" + args.side,
            "joint_names": names,
            "current_rad": current,
            "target_rad": target,
            "selected_joint": names[index],
        }
        print(json.dumps(preview, ensure_ascii=False, indent=2))
        if not args.execute:
            print("Preview complete. No hand service request was sent.")
            return
        if not node.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("HandJoint service is unavailable")
        request = HandJoint.Request()
        request.q = target
        future = node.client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("HandJoint service timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("HandJoint service returned no response")
        print(json.dumps({
            "service_success": response.success,
            "service_message": response.message,
        }, ensure_ascii=False, indent=2))
        if not response.success:
            raise RuntimeError("HandJoint service rejected the request")
        observation_deadline = time.monotonic() + 2.0
        while time.monotonic() < observation_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        observed = dict(zip(node.latest.name, node.latest.position))
        actual = float(observed[names[index]])
        print(json.dumps({
            "observed_joint_rad": actual,
            "target_joint_rad": args.target,
            "absolute_error_rad": abs(actual - args.target),
        }, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
