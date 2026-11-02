#!/usr/bin/env python3
"""Run the native detector and pose solver on a generated ID 0 image."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
from apriltag import apriltag

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'csc309_apriltag_pose'
if not PACKAGE.exists():
    PACKAGE = ROOT / 'catkin_ws/src/csc309_apriltag_pose'
sys.path.insert(0, str(PACKAGE / 'scripts'))
from pose_pipeline import PosePipeline, compose_pose


CALIBRATION = PACKAGE / 'config/calibration_vendor_run01.json'


def main():
    cal = json.loads(CALIBRATION.read_text(encoding='utf-8'))
    image = np.full((cal['image_height'], cal['image_width']), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 320)
    image[380:700, 800:1120] = marker
    detector = apriltag('tag36h11', threads=4, maxhamming=0, decimate=2.0)
    detections = [d for d in detector.detect(image) if d['id'] == 0]
    assert len(detections) == 1, f'Expected one ID 0 detection, got {len(detections)}'
    k = cal['camera_matrix']
    pose = detector.estimate_tag_pose(detections[0], 0.04, k[0], k[4], k[2], k[5])
    xyz = np.asarray(pose['t']).reshape(3)
    assert np.isfinite(xyz).all() and 0.10 < xyz[2] < 0.25, xyz
    error_px = PosePipeline.reprojection_error(
        SimpleNamespace(tag_size=0.04, cal=cal), detections[0], xyz, pose['R'])
    assert error_px < 1.0, error_px
    base_xyz, _ = compose_pose([1, 2, 3], [0, 0, np.sqrt(0.5), np.sqrt(0.5)],
                               [1, 0, 0], [0, 0, 0, 1])
    assert np.allclose(base_xyz, [1, 3, 3]), base_xyz
    print(json.dumps({'id': detections[0]['id'], 'hamming': detections[0]['hamming'],
                      'camera_xyz_m': xyz.tolist(), 'pose_error': float(pose['error']),
                      'reprojection_error_px': error_px}, indent=2))


if __name__ == '__main__':
    main()
