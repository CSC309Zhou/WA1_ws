#!/usr/bin/env python3
"""Detect Tag36h11 ID 0 on Jetson and publish camera / upper-limb BASE poses."""

import json
import math
import time

import cv2
import numpy as np
import rospy
from apriltag import apriltag
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String


def normalize_quaternion(values):
    q = np.asarray(values, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError('Invalid quaternion')
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError('Zero quaternion')
    return q / norm


def compose_pose(parent_xyz, parent_q, child_xyz, child_q):
    """Compose B_T_C with C_T_Tag; quaternions are xyzw."""
    parent_rotation = Rotation.from_quat(normalize_quaternion(parent_q))
    child_rotation = Rotation.from_quat(normalize_quaternion(child_q))
    xyz = np.asarray(parent_xyz, dtype=np.float64) + parent_rotation.apply(child_xyz)
    q = (parent_rotation * child_rotation).as_quat()
    return xyz, q


def pose_message(stamp, frame, xyz, q):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [float(x) for x in xyz]
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = [float(x) for x in q]
    return msg


def grayscale(msg):
    if msg.is_bigendian or msg.height <= 0 or msg.width <= 0:
        raise ValueError('Unsupported image layout')
    if len(msg.data) != msg.height * msg.step:
        raise ValueError('Image payload size does not match step and height')
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    if msg.encoding == 'mono8':
        return np.ascontiguousarray(raw[:, :msg.width])
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('Expected mono8, rgb8 or bgr8 image')
    color = np.ascontiguousarray(raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3))
    code = cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(color, code)


class PosePipeline:
    def __init__(self):
        with open(rospy.get_param('~calibration_file'), encoding='utf-8') as stream:
            self.cal = json.load(stream)
        if self.cal['model'] != 'vendor_camera_info':
            raise ValueError('This hand-eye transform is paired with vendor_camera_info')
        self.base_xyz = np.asarray(self.cal['base_to_camera']['translation_m'], dtype=np.float64)
        self.base_q = normalize_quaternion(self.cal['base_to_camera']['quaternion_xyzw'])
        if self.base_xyz.shape != (3,) or not np.isfinite(self.base_xyz).all():
            raise ValueError('Invalid BASE to camera translation')
        self.tag_id = int(rospy.get_param('~tag_id', 0))
        self.tag_size = float(rospy.get_param('~tag_size_m', 0.04))
        if self.tag_id != 0 or not math.isclose(self.tag_size, 0.04, abs_tol=1e-9):
            raise ValueError('Expected Tag36h11 ID 0, black-border edge 0.04 m')
        self.max_reprojection_px = float(rospy.get_param('~max_reprojection_error_px', 5.0))
        self.detector = apriltag(
            rospy.get_param('~tag_family', 'tag36h11'),
            threads=int(rospy.get_param('~threads', 4)),
            maxhamming=int(rospy.get_param('~max_hamming', 0)),
            decimate=float(rospy.get_param('~decimate', 1.0)),
            blur=float(rospy.get_param('~blur', 0.0)),
            refine_edges=bool(rospy.get_param('~refine_edges', True)))
        self.camera_ok = False
        self.camera_reason = 'Waiting for CameraInfo'
        self.joint_positions = {}
        self.joint_receive_monotonic = None
        self.last_image_monotonic = None
        self.frames = 0
        self.tag_frames = 0
        self.camera_pose_frames = 0
        self.base_pose_frames = 0
        self.last_processing_ms = None
        self.last_reprojection_px = None
        self.reason = 'Waiting for camera images'
        self.camera_pub = rospy.Publisher('tag0/pose_camera', PoseStamped, queue_size=1)
        self.base_pub = rospy.Publisher('tag0/pose_base', PoseStamped, queue_size=1)
        self.detected_pub = rospy.Publisher('tag0/detected', Bool, queue_size=1)
        self.base_valid_pub = rospy.Publisher('tag0/base_valid', Bool, queue_size=1)
        self.status_pub = rospy.Publisher('tag0/status', String, queue_size=1)
        rospy.Subscriber(rospy.get_param('~camera_info_topic'), CameraInfo,
                         self.on_camera_info, queue_size=1)
        rospy.Subscriber(rospy.get_param('~joint_state_topic'), JointState,
                         self.on_joint_state, queue_size=1)
        rospy.Subscriber(rospy.get_param('~image_topic'), Image,
                         self.on_image, queue_size=1, buff_size=16 * 1024 * 1024,
                         tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0), self.on_status)
        rospy.loginfo('Tag36h11 ID 0: %s -> %s, size %.3f m',
                      self.cal['camera_frame'], self.cal['base_frame'], self.tag_size)

    def on_camera_info(self, msg):
        expected = self.cal['camera_matrix']
        frame_ok = msg.header.frame_id.lstrip('/') == self.cal['camera_frame'].lstrip('/')
        shape_ok = msg.width == self.cal['image_width'] and msg.height == self.cal['image_height']
        k_ok = len(msg.K) == 9 and all(abs(a - b) <= 0.05 for a, b in zip(msg.K, expected))
        # The native AprilTag pose solver uses fx, fy, cx, cy from K. The current
        # RealSense raw image also has P=K and D=0, so no rectification is needed.
        projection = [expected[0], expected[1], expected[2], 0.0,
                      expected[3], expected[4], expected[5], 0.0,
                      expected[6], expected[7], expected[8], 0.0]
        p_ok = len(msg.P) == 12 and all(abs(a - b) <= 0.05 for a, b in zip(msg.P, projection))
        r_ok = len(msg.R) == 9 and all(abs(a - b) <= 1e-6 for a, b in zip(
            msg.R, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]))
        model_ok = msg.distortion_model == self.cal['distortion_model']
        d_ok = len(msg.D) == len(self.cal['distortion_coefficients']) and all(
            abs(a - b) <= 1e-5 for a, b in zip(msg.D, self.cal['distortion_coefficients']))
        self.camera_ok = frame_ok and shape_ok and k_ok and p_ok and r_ok and d_ok and model_ok
        self.camera_reason = ('CameraInfo matches pinned calibration' if self.camera_ok else
                              'CameraInfo frame, size, model, K, D, R or P differs from calibration')
        if not self.camera_ok:
            rospy.logwarn_throttle(5.0, self.camera_reason)

    def on_joint_state(self, msg):
        if len(msg.name) == len(msg.position):
            self.joint_positions = dict(zip(msg.name, msg.position))
            self.joint_receive_monotonic = time.monotonic()

    def fixed_joint_reason(self):
        if self.joint_receive_monotonic is None:
            return 'Waiting for upper-limb joint states'
        if time.monotonic() - self.joint_receive_monotonic > 0.3:
            return 'Upper-limb joint states are stale'
        for name, target in self.cal['fixed_joints'].items():
            value = self.joint_positions.get(name)
            tolerance = (self.cal['lift_tolerance_m'] if name == 'Lifting_Z' else
                         self.cal['angle_tolerance_rad'])
            if value is None or not math.isfinite(value):
                return 'Missing or invalid fixed joint: ' + name
            if abs(value - target) > tolerance:
                return 'Fixed joint differs from calibration: ' + name
        return ''

    def reprojection_error(self, detection, xyz, rotation_matrix):
        half = self.tag_size / 2.0
        points = np.array([[-half, half, 0], [half, half, 0],
                           [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
        rvec, _ = cv2.Rodrigues(rotation_matrix)
        projected, _ = cv2.projectPoints(points, rvec, xyz,
                                          np.asarray(self.cal['camera_matrix']).reshape(3, 3),
                                          np.asarray(self.cal['distortion_coefficients']))
        observed = np.asarray(detection['lb-rb-rt-lt'], dtype=np.float64)
        return float(np.mean(np.linalg.norm(projected.reshape(4, 2) - observed, axis=1)))

    def on_image(self, msg):
        start = time.monotonic()
        self.last_image_monotonic = start
        self.frames += 1
        detected = False
        base_valid = False
        self.last_reprojection_px = None
        try:
            if msg.header.frame_id.lstrip('/') != self.cal['camera_frame'].lstrip('/'):
                raise ValueError('Image frame is not calibrated camera optical frame')
            if msg.width != self.cal['image_width'] or msg.height != self.cal['image_height']:
                raise ValueError('Image resolution differs from calibration')
            hits = [d for d in self.detector.detect(grayscale(msg))
                    if int(d['id']) == self.tag_id and int(d['hamming']) == 0]
            detected = bool(hits)
            if not detected:
                self.reason = 'Tag ID 0 absent'
                return
            self.tag_frames += 1
            if not self.camera_ok:
                self.reason = self.camera_reason
                return
            detection = max(hits, key=lambda d: float(d['margin']))
            k = self.cal['camera_matrix']
            pose = self.detector.estimate_tag_pose(
                detection, self.tag_size, k[0], k[4], k[2], k[5])
            xyz = np.asarray(pose['t'], dtype=np.float64).reshape(3)
            matrix = np.asarray(pose['R'], dtype=np.float64).reshape(3, 3)
            if not np.isfinite(xyz).all() or xyz[2] <= 0 or not np.isfinite(matrix).all():
                raise ValueError('AprilTag pose is invalid or behind camera')
            reprojection_px = self.reprojection_error(detection, xyz, matrix)
            self.last_reprojection_px = round(reprojection_px, 3)
            if not math.isfinite(reprojection_px) or reprojection_px > self.max_reprojection_px:
                raise ValueError('Tag corner reprojection exceeds configured threshold')
            q = Rotation.from_matrix(matrix).as_quat()
            self.camera_pub.publish(pose_message(msg.header.stamp,
                                                 self.cal['camera_frame'], xyz, q))
            self.camera_pose_frames += 1
            joint_reason = self.fixed_joint_reason()
            if joint_reason:
                self.reason = joint_reason
                return
            base_xyz, base_q = compose_pose(self.base_xyz, self.base_q, xyz, q)
            self.base_pub.publish(pose_message(msg.header.stamp,
                                               self.cal['base_frame'], base_xyz, base_q))
            self.base_pose_frames += 1
            base_valid = True
            self.reason = 'BASE pose published'
        except (ValueError, RuntimeError, cv2.error) as exc:
            self.reason = str(exc)
            rospy.logwarn_throttle(5.0, self.reason)
        finally:
            self.detected_pub.publish(Bool(data=detected))
            self.base_valid_pub.publish(Bool(data=base_valid))
            self.last_processing_ms = round((time.monotonic() - start) * 1000.0, 2)

    def on_status(self, _event):
        age_ms = (None if self.last_image_monotonic is None else
                  round((time.monotonic() - self.last_image_monotonic) * 1000.0, 1))
        self.status_pub.publish(String(data=json.dumps({
            'frames': self.frames, 'tag_frames': self.tag_frames,
            'camera_pose_frames': self.camera_pose_frames,
            'base_pose_frames': self.base_pose_frames,
            'frame_age_ms': age_ms, 'processing_ms': self.last_processing_ms,
            'reprojection_error_px': self.last_reprojection_px,
            'camera_info_ok': self.camera_ok,
            'fixed_joint_reason': self.fixed_joint_reason(),
            'reason': self.reason,
        }, separators=(',', ':'))))


if __name__ == '__main__':
    rospy.init_node('csc309_apriltag_pose')
    PosePipeline()
    rospy.spin()
