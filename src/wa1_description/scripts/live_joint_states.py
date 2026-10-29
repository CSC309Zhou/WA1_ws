#!/usr/bin/env python3
"""Map bridge feedback onto the static visualization model."""

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class LiveJointStates(Node):
    def __init__(self):
        super().__init__('live_joint_states')
        description = self.declare_parameter('robot_description', '').value
        root = ET.fromstring(description)
        self.positions = {
            joint.attrib['name']: 0.0
            for joint in root.findall('joint')
            if joint.attrib['type'] != 'fixed'
        }
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            JointState, '/upperlimb/joint_states', self.feedback,
            qos_profile_sensor_data,
        )

    def feedback(self, message):
        import math

        if len(message.name) != len(message.position):
            return
        matched = False
        for name, position in zip(message.name, message.position):
            if name in self.positions and math.isfinite(position):
                self.positions[name] = position
                matched = True
        if not matched:
            return
        output = JointState()
        # Use the PC clock: robot and PC clocks may not be synchronized.
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = list(self.positions)
        output.position = list(self.positions.values())
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = LiveJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
