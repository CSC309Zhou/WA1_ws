#!/usr/bin/env python3
"""Exercise the ROS1 image-to-pose node on a private local ROS Master."""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'csc309_apriltag_pose'
if not PACKAGE.exists():
    PACKAGE = ROOT / 'catkin_ws/src/csc309_apriltag_pose'
CAL = json.loads((PACKAGE / 'config/calibration_vendor_run01.json').read_text(encoding='utf-8'))


def local_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    port = local_port()
    env = dict(os.environ, ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
               HOME='/tmp', ROS_LOG_DIR='/tmp/csc309-apriltag-test-log')
    Path(env['ROS_LOG_DIR']).mkdir(parents=True, exist_ok=True)
    log = open('/tmp/csc309-apriltag-isolated-test.log', 'w', encoding='utf-8')
    master = subprocess.Popen(['roscore', '-p', str(port)], env=env, stdout=log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    launch = None
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=0.2):
                    break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError('Private roscore did not start')
        launch = subprocess.Popen(['roslaunch', 'csc309_apriltag_pose', 'pipeline.launch'],
                                  env=env, stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'], ROS_IP='127.0.0.1')
        import rospy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from std_msgs.msg import Bool

        rospy.init_node('csc309_apriltag_isolated_test', anonymous=True, disable_signals=True)
        camera_poses = []
        base_poses = []
        detected = []
        base_valid = []
        rospy.Subscriber('/csc309/apriltag/tag0/pose_camera', PoseStamped,
                         camera_poses.append, queue_size=1)
        rospy.Subscriber('/csc309/apriltag/tag0/pose_base', PoseStamped,
                         base_poses.append, queue_size=1)
        rospy.Subscriber('/csc309/apriltag/tag0/detected', Bool,
                         lambda m: detected.append(m.data), queue_size=1)
        rospy.Subscriber('/csc309/apriltag/tag0/base_valid', Bool,
                         lambda m: base_valid.append(m.data), queue_size=1)
        image_pub = rospy.Publisher('/zj_humanoid/sensor/realsense_head/color/image_raw',
                                    Image, queue_size=1)
        info_pub = rospy.Publisher('/zj_humanoid/sensor/realsense_head/color/camera_info',
                                   CameraInfo, queue_size=1)
        joint_pub = rospy.Publisher('/zj_humanoid/upperlimb/joint_states',
                                    JointState, queue_size=1)
        image = np.full((CAL['image_height'], CAL['image_width']), 255, dtype=np.uint8)
        marker = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 320)
        image[380:700, 800:1120] = marker

        def publish_frame(gray, lift):
            stamp = rospy.Time.now()
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = CAL['camera_frame']
            info.width, info.height = CAL['image_width'], CAL['image_height']
            info.distortion_model = 'plumb_bob'
            info.D = CAL['distortion_coefficients']
            info.K = CAL['camera_matrix']
            info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            k = CAL['camera_matrix']
            info.P = [k[0], 0, k[2], 0, 0, k[4], k[5], 0, 0, 0, 1, 0]
            joints = JointState()
            joints.header.stamp = stamp
            joints.name = list(CAL['fixed_joints'])
            joints.position = [lift if name == 'Lifting_Z' else CAL['fixed_joints'][name]
                               for name in joints.name]
            msg = Image()
            msg.header = info.header
            msg.width, msg.height = info.width, info.height
            msg.encoding = 'mono8'
            msg.step = msg.width
            msg.data = gray.tobytes()
            info_pub.publish(info)
            joint_pub.publish(joints)
            image_pub.publish(msg)

        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not base_poses:
            publish_frame(image, CAL['fixed_joints']['Lifting_Z'])
            time.sleep(0.1)
        assert camera_poses and base_poses, 'No camera / BASE pose from synthetic tag'
        assert True in detected and True in base_valid
        cam = camera_poses[-1]
        base = base_poses[-1]
        assert cam.header.frame_id == CAL['camera_frame']
        assert base.header.frame_id == CAL['base_frame']
        cam_xyz = np.array([cam.pose.position.x, cam.pose.position.y, cam.pose.position.z])
        expected_base = (np.array(CAL['base_to_camera']['translation_m']) +
                         Rotation.from_quat(CAL['base_to_camera']['quaternion_xyzw']).apply(cam_xyz))
        base_xyz = np.array([base.pose.position.x, base.pose.position.y, base.pose.position.z])
        assert np.allclose(base_xyz, expected_base, atol=1e-6), (base_xyz, expected_base)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            publish_frame(image, CAL['fixed_joints']['Lifting_Z'] + 0.05)
            time.sleep(0.1)
        prior_base_count = len(base_poses)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            publish_frame(image, CAL['fixed_joints']['Lifting_Z'] + 0.05)
            time.sleep(0.1)
        assert False in base_valid and len(base_poses) == prior_base_count
        assert len(camera_poses) > prior_base_count

        blank = np.full_like(image, 255)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            publish_frame(blank, CAL['fixed_joints']['Lifting_Z'])
            time.sleep(0.1)
        assert False in detected
        print(json.dumps({'camera_pose_count': len(camera_poses),
                          'base_pose_count': len(base_poses),
                          'camera_xyz_m': cam_xyz.tolist(),
                          'base_xyz_m': base_xyz.tolist(),
                          'fixed_joint_gate': 'passed', 'tag_absence': 'passed'}, indent=2))
    finally:
        if launch is not None:
            stop(launch)
        stop(master)
        log.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'Isolated test failed: {exc}; log: /tmp/csc309-apriltag-isolated-test.log',
              file=sys.stderr)
        raise
