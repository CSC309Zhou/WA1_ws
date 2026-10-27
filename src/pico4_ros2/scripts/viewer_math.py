"""Pure geometry for the PICO4 pose viewer."""

import math


AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}
DEFAULT_AXIS_MAP = ('-z', '-x', '+y')  # XR right/up/back -> display forward/left/up


def axis_matrix(tokens):
    """Return a proper 3x3 rotation from signed XR axis names."""
    if len(tokens) != 3:
        raise ValueError('world_from_xr_axes needs exactly three entries')
    rows = []
    used = set()
    for token in tokens:
        if not isinstance(token, str) or len(token) != 2 or token[0] not in '+-' or token[1] not in AXIS_INDEX:
            raise ValueError('Each axis must look like +x or -z')
        axis = AXIS_INDEX[token[1]]
        if axis in used:
            raise ValueError('Each XR axis must occur exactly once')
        used.add(axis)
        row = [0.0, 0.0, 0.0]
        row[axis] = 1.0 if token[0] == '+' else -1.0
        rows.append(row)
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if determinant != 1.0:
        raise ValueError('Display axes must remain right-handed (determinant +1)')
    return rows


def map_vector(matrix, vector):
    return [sum(row[i] * vector[i] for i in range(3)) for row in matrix]


def normalized_quaternion(raw):
    if len(raw) != 4 or not all(math.isfinite(value) for value in raw):
        raise ValueError('Invalid quaternion components')
    length = math.sqrt(sum(value * value for value in raw))
    if not 0.5 < length < 1.5:
        raise ValueError('Invalid quaternion norm')
    return [value / length for value in raw]


def rotate_vector(quaternion, vector):
    """Rotate a vector with an XYZW unit quaternion."""
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return [
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    ]


def display_pose(raw_pose, origin_raw, matrix):
    """Map an SDK XYZ/XYZW pose to display position and orientation arrows."""
    if len(raw_pose) != 7 or not all(math.isfinite(value) for value in raw_pose):
        raise ValueError('Pose must contain seven finite numbers')
    if len(origin_raw) != 3 or not all(math.isfinite(value) for value in origin_raw):
        raise ValueError('Invalid display origin')
    # ROS-generated float32 arrays can contain NumPy scalars; the HTTP API needs
    # ordinary Python floats so json.dumps does not reject a valid frame.
    raw_pose = [float(value) for value in raw_pose]
    origin_raw = [float(value) for value in origin_raw]
    orientation = normalized_quaternion(raw_pose[3:7])
    translation = [raw_pose[i] - origin_raw[i] for i in range(3)]
    basis = [
        map_vector(matrix, rotate_vector(orientation, axis))
        for axis in ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
    ]
    return {
        'position': map_vector(matrix, translation),
        'basis': basis,
        'raw_pose': list(raw_pose),
    }
