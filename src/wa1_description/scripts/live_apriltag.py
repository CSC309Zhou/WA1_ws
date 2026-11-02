#!/usr/bin/env python3
"""Show the live AprilTag pose in the WA1 model's visualization TF tree."""

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


SOURCE_FRAME = 'wa1_upperlimb_base'
MODEL_FRAME = 'BASE'
TAG_FRAME = 'apriltag_36h11_0'
AXIS_LENGTH_M = 0.12
MARKER_LIFETIME_NS = 300_000_000


def rotate_vector(quaternion, vector):
    """Rotate a vector by a unit quaternion in xyzw order."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def make_visualization(pose, stamp):
    """Return a model-frame TF and short-lived markers for a BASE pose."""
    if pose.header.frame_id.lstrip('/') != SOURCE_FRAME:
        raise ValueError(f'Expected {SOURCE_FRAME}, got {pose.header.frame_id!r}')
    position = pose.pose.position
    orientation = pose.pose.orientation
    xyz = (position.x, position.y, position.z)
    xyzw = (orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(value) for value in (*xyz, *xyzw)):
        raise ValueError('AprilTag pose has a non-finite coordinate')
    norm = math.sqrt(sum(value * value for value in xyzw))
    if norm < 1e-12:
        raise ValueError('AprilTag quaternion has zero norm')
    quaternion = tuple(value / norm for value in xyzw)

    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = MODEL_FRAME
    transform.child_frame_id = TAG_FRAME
    transform.transform.translation.x = position.x
    transform.transform.translation.y = position.y
    transform.transform.translation.z = position.z
    (transform.transform.rotation.x, transform.transform.rotation.y,
     transform.transform.rotation.z, transform.transform.rotation.w) = quaternion

    markers = MarkerArray()
    for index, (axis, color) in enumerate((
        ((AXIS_LENGTH_M, 0.0, 0.0), (1.0, 0.1, 0.1)),
        ((0.0, AXIS_LENGTH_M, 0.0), (0.1, 1.0, 0.1)),
        ((0.0, 0.0, AXIS_LENGTH_M), (0.2, 0.5, 1.0)),
    )):
        direction = rotate_vector(quaternion, axis)
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = MODEL_FRAME
        marker.ns = TAG_FRAME
        marker.id = index
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                         Point(x=xyz[0] + direction[0],
                               y=xyz[1] + direction[1],
                               z=xyz[2] + direction[2])]
        marker.scale.x = 0.004
        marker.scale.y = 0.009
        marker.scale.z = 0.013
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.lifetime.nanosec = MARKER_LIFETIME_NS
        markers.markers.append(marker)

    label = Marker()
    label.header.stamp = stamp
    label.header.frame_id = MODEL_FRAME
    label.ns = TAG_FRAME
    label.id = 3
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    label.pose.position.x = position.x
    label.pose.position.y = position.y
    label.pose.position.z = position.z + 0.14
    label.pose.orientation.w = 1.0
    label.scale.z = 0.05
    label.color.r = label.color.g = label.color.b = 0.92
    label.color.a = 1.0
    label.text = 'Tag36h11:0'
    label.lifetime.nanosec = MARKER_LIFETIME_NS
    markers.markers.append(label)
    return transform, markers


class LiveAprilTag(Node):
    def __init__(self):
        super().__init__('live_apriltag')
        self.broadcaster = TransformBroadcaster(self)
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'apriltag_markers', 10)
        self.subscription = self.create_subscription(
            PoseStamped, '/perception/apriltag/tag0/pose_base',
            self.on_pose, qos_profile_sensor_data)
        self.reported_errors = set()

    def on_pose(self, pose):
        # Match live_joint_states.py: the PC visualization tree uses PC time.
        stamp = self.get_clock().now().to_msg()
        try:
            transform, markers = make_visualization(pose, stamp)
        except ValueError as exc:
            message = str(exc)
            if message not in self.reported_errors:
                self.get_logger().warning(message)
                self.reported_errors.add(message)
            return
        self.broadcaster.sendTransform(transform)
        self.marker_publisher.publish(markers)


def main():
    rclpy.init()
    node = LiveAprilTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
