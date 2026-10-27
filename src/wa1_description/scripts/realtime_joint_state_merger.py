#!/usr/bin/env python3
"""Module-local real robot JointState merger for URDF visualization."""

import threading
import xml.etree.ElementTree as ElementTree

import rospy
from sensor_msgs.msg import JointState


ARM_JOINT_MAP = {
    "lift": "Lifting_Z",
    "waistPitch": "Waist_Y",
    "waistYaw": "Waist_Z",
    "leftShoulderRoll": "Shoulder_X_L",
    "leftShoulderYaw": "Shoulder_Z_L",
    "leftShoulderPitch": "Shoulder_Y_L",
    "leftElbow": "Elbow_L",
    "leftWristRoll": "Wrist_X_L",
    "leftWristYaw": "Wrist_Z_L",
    "leftWristPitch": "Wrist_Y_L",
    "rightShoulderRoll": "Shoulder_X_R",
    "rightShoulderYaw": "Shoulder_Z_R",
    "rightShoulderPitch": "Shoulder_Y_R",
    "rightElbow": "Elbow_R",
    "rightWristRoll": "Wrist_X_R",
    "rightWristYaw": "Wrist_Z_R",
    "rightWristPitch": "Wrist_Y_R",
    "neckPitch": "Neck_Y",
    "neckYaw": "Neck_Z"
}

HAND_JOINT_MAP = {
    'THUMB_MP_LEFT': 'thumb_MP_L', 'THUMB_CMC_LEFT': 'thumb_CMC_L',
    'INDEX_MCP_LEFT': 'index_MCP_L', 'MIDDLE_MCP_LEFT': 'middle_MCP_L',
    'RING_MCP_LEFT': 'ring_MCP_L', 'LITTLE_MCP_LEFT': 'little_MCP_L',
    'THUMB_MP_RIGHT': 'thumb_MP_R', 'THUMB_CMC_RIGHT': 'thumb_CMC_R',
    'INDEX_MCP_RIGHT': 'index_MCP_R', 'MIDDLE_MCP_RIGHT': 'middle_MCP_R',
    'RING_MCP_RIGHT': 'ring_MCP_R', 'LITTLE_MCP_RIGHT': 'little_MCP_R',
}

DERIVED_JOINTS = {
    'thumb_IP_L': ('thumb_MP_L', -1.0), 'thumb_IP_R': ('thumb_MP_R', -1.0),
    'index_PIP_L': ('index_MCP_L', 1.0), 'index_DIP_L': ('index_MCP_L', 0.8),
    'middle_PIP_L': ('middle_MCP_L', 1.0), 'middle_DIP_L': ('middle_MCP_L', 0.8),
    'ring_PIP_L': ('ring_MCP_L', 1.0), 'ring_DIP_L': ('ring_MCP_L', 0.8),
    'little_PIP_L': ('little_MCP_L', 1.0), 'little_DIP_L': ('little_MCP_L', 0.8),
    'index_PIP_R': ('index_MCP_R', 1.0), 'index_DIP_R': ('index_MCP_R', 0.8),
    'middle_PIP_R': ('middle_MCP_R', 1.0), 'middle_DIP_R': ('middle_MCP_R', 0.8),
    'ring_PIP_R': ('ring_MCP_R', 1.0), 'ring_DIP_R': ('ring_MCP_R', 0.8),
    'little_PIP_R': ('little_MCP_R', 1.0), 'little_DIP_R': ('little_MCP_R', 0.8),
}


class JointStateMerger:
    def __init__(self):
        self.positions = {}
        self.lock = threading.Lock()
        arm_topic = rospy.get_param('~arm_joint_states_topic')
        hand_topic = rospy.get_param('~hand_joint_states_topic')
        output_topic = rospy.get_param('~urdf_joint_states_topic')
        self.initialize_defaults(rospy.get_param('~robot_description_param',
                                                  '/robot_description'))
        self.publisher = rospy.Publisher(output_topic, JointState, queue_size=10)
        rospy.Subscriber(arm_topic, JointState, self.arm_callback, queue_size=10)
        rospy.Subscriber(hand_topic, JointState, self.hand_callback, queue_size=10)

    def initialize_defaults(self, description_param):
        try:
            root = ElementTree.fromstring(rospy.get_param(description_param))
        except Exception as error:
            rospy.logwarn('Cannot read %s: %s', description_param, error)
            return
        for joint in root.findall('joint'):
            name = joint.get('name')
            if name and joint.get('type') != 'fixed':
                self.positions.setdefault(name, 0.0)

    def update(self, message, name_map):
        if len(message.name) != len(message.position):
            rospy.logwarn_throttle(5.0, 'Ignoring malformed JointState')
            return False
        for name, position in zip(message.name, message.position):
            self.positions[name_map.get(name, name)] = position
        return True

    def publish(self):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = sorted(self.positions)
        message.position = [self.positions[name] for name in message.name]
        self.publisher.publish(message)

    def arm_callback(self, message):
        with self.lock:
            if self.update(message, ARM_JOINT_MAP):
                self.publish()

    def hand_callback(self, message):
        with self.lock:
            if not self.update(message, HAND_JOINT_MAP):
                return
            for target, (source, multiplier) in DERIVED_JOINTS.items():
                if source in self.positions:
                    value = self.positions[source] * multiplier
                    self.positions[target] = (
                        max(0.0, value) if target.startswith('thumb_IP') else value)
            self.publish()


if __name__ == '__main__':
    rospy.init_node('joint_state_merger')
    JointStateMerger()
    rospy.spin()

