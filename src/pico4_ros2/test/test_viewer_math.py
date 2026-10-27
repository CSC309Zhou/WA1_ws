"""Verify the viewer's axis convention and orientation arrows."""

from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import pytest
import numpy as np

from viewer_math import DEFAULT_AXIS_MAP, axis_matrix, display_pose, map_vector


def test_default_axis_map_is_forward_left_up_and_right_handed():
    matrix = axis_matrix(DEFAULT_AXIS_MAP)
    assert map_vector(matrix, [0.0, 0.0, -1.0]) == [1.0, 0.0, 0.0]
    assert map_vector(matrix, [-1.0, 0.0, 0.0]) == [0.0, 1.0, 0.0]
    assert map_vector(matrix, [0.0, 1.0, 0.0]) == [0.0, 0.0, 1.0]
    with pytest.raises(ValueError, match='right-handed'):
        axis_matrix(('-z', '+x', '+y'))


def test_position_origin_and_rotated_local_axis():
    matrix = axis_matrix(DEFAULT_AXIS_MAP)
    origin = [0.1, 1.0, 0.3]
    identity = display_pose([0.0, 1.2, 0.1, 0.0, 0.0, 0.0, 1.0], origin, matrix)
    assert identity['position'] == pytest.approx([0.2, 0.1, 0.2])
    assert identity['basis'][0] == pytest.approx([0.0, -1.0, 0.0])

    # 180 degrees about XR +Y reverses the controller's local +X direction.
    turned = display_pose([0.0, 1.2, 0.1, 0.0, 1.0, 0.0, 0.0], origin, matrix)
    assert turned['basis'][0] == pytest.approx([0.0, 1.0, 0.0])


def test_bad_quaternion_is_not_displayed():
    with pytest.raises(ValueError, match='quaternion'):
        display_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     [0.0, 0.0, 0.0], axis_matrix(DEFAULT_AXIS_MAP))


def test_ros_float32_pose_can_be_sent_as_json():
    raw = np.array([0, 1.6, -0.3, 0, 0, 0, 1], dtype=np.float32)
    origin = np.array([0, 1.6, 0], dtype=np.float32)
    pose = display_pose(raw, origin, axis_matrix(DEFAULT_AXIS_MAP))
    assert pose['position'] == pytest.approx([0.3, 0.0, 0.0])
    json.dumps(pose, allow_nan=False)
