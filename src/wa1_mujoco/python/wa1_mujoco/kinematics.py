"""Local-only interface to the official WA1 upper-body FK/IK library.

All public poses are TCP_L or TCP_R with respect to BASE, in metres and
quaternion xyzw. The endoscope and endoscope_tcp are deliberately not IK frames.
"""

from __future__ import annotations

import ctypes
import math
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import get_package_prefix, get_package_share_directory


LEFT = ('Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L',
        'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L')
RIGHT = ('Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R',
         'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R')
# The official C++ API orders the last three differently from joint_states.
JOINT_ORDER = LEFT + RIGHT + ('Neck_Z', 'Neck_Y', 'Lifting_Z', 'Waist_Z', 'Waist_Y')
TORSO = ('Lifting_Z', 'Waist_Z', 'Waist_Y')


def load_home() -> tuple[mujoco.MjModel, mujoco.MjData]:
    scene = Path(get_package_share_directory('wa1_mujoco')) / 'assets' / 'wa1_assets' / 'WA1_400L' / 'scene.xml'
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if key < 0:
        raise RuntimeError('MuJoCo home keyframe is missing')
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    return model, data


def joint_positions(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    return {name: float(data.qpos[model.joint(name).qposadr[0]]) for name in JOINT_ORDER}


def set_joint_positions(model: mujoco.MjModel, data: mujoco.MjData,
                        joints: dict[str, float], *, set_control: bool = True) -> None:
    for name, value in joints.items():
        if name not in JOINT_ORDER:
            raise ValueError(f'unsupported WA1 kinematic joint: {name}')
        data.qpos[model.joint(name).qposadr[0]] = value
        if set_control:
            data.ctrl[model.actuator(name).id] = value
    mujoco.mj_forward(model, data)


def _body_transform(data: mujoco.MjData, name: str) -> np.ndarray:
    body = data.body(name)
    result = np.eye(4)
    result[:3, :3] = np.asarray(body.xmat).reshape(3, 3)
    result[:3, 3] = body.xpos
    return result


def _matrix_to_pose(transform: np.ndarray) -> np.ndarray:
    quat_wxyz = np.empty(4)
    mujoco.mju_mat2Quat(quat_wxyz, np.ascontiguousarray(transform[:3, :3]).reshape(9))
    return np.array([*transform[:3, 3], *quat_wxyz[1:], quat_wxyz[0]], dtype=np.float64)


def _pose_to_matrix(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=np.float64)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError('pose must be finite [x,y,z,qx,qy,qz,qw]')
    q = np.array([pose[6], pose[3], pose[4], pose[5]], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError('quaternion norm is zero')
    q /= norm
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, q)
    result = np.eye(4)
    result[:3, :3] = rotation.reshape(3, 3)
    result[:3, 3] = pose[:3]
    return result


def pose_in_base(data: mujoco.MjData, side: str) -> np.ndarray:
    """MuJoCo FK: TCP relative to BASE, position in m and quaternion xyzw."""
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    frame = 'TCP_L' if side == 'left' else 'TCP_R'
    transform = np.linalg.inv(_body_transform(data, 'BASE')) @ _body_transform(data, frame)
    return _matrix_to_pose(transform)


class OfflineKinematics:
    """Official WA1 FK and fixed-q7 IK, evaluated entirely on the PC.

    ``solve`` returns every official solution and its arm angle. When torso
    joints move, the target is converted to the library's zero-torso IK frame
    using the MuJoCo model's BASE and WAIST_PITCH transforms.
    """

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self._scratch = mujoco.MjData(model)
        self._tmp = tempfile.TemporaryDirectory(prefix='wa1_offline_ik_')
        share = Path(get_package_share_directory('wa1_mujoco'))
        lib_dir = Path(get_package_prefix('wa1_mujoco')) / 'lib' / 'wa1_mujoco'
        lib = lib_dir / 'libwa1_official_kinematics.so'
        # The vendor library was built with a fixed /opt/openrobots RUNPATH.
        # Preload its bundled dependencies by absolute path so a ROS 2 host
        # needs no global LD_LIBRARY_PATH or system Focal libraries.
        dependencies = ('libminizip.so.1', 'libassimp.so.5',
                        'libboost_filesystem.so.1.71.0',
                        'libboost_serialization.so.1.71.0',
                        'liboctomath.so.1.9', 'liboctomap.so.1.9',
                        'libconsole_bridge.so.0.4', 'libtinyxml.so.2.6.2',
                        'liburdfdom_model.so.1.0',
                        'libcoal.so.3.0.1', 'libpinocchio_default.so.3.4.0',
                        'libpinocchio_parsers.so.3.4.0', 'libyaml-cpp.so.0.6',
                        'libuplimb_library.so.1')
        self._preloaded = [ctypes.CDLL(str(lib_dir / name), mode=ctypes.RTLD_GLOBAL)
                           for name in dependencies]
        self._native = ctypes.CDLL(str(lib))
        self._native.wa1_last_error.restype = ctypes.c_char_p
        self._native.wa1_init.argtypes = [ctypes.c_char_p]
        self._native.wa1_init.restype = ctypes.c_int
        double_ptr = ctypes.POINTER(ctypes.c_double)
        self._native.wa1_fk.argtypes = [double_ptr, ctypes.c_int, double_ptr]
        self._native.wa1_fk.restype = ctypes.c_int
        self._native.wa1_ik_zero_torso.argtypes = [double_ptr, ctypes.c_int, ctypes.c_double,
                                                   double_ptr, double_ptr, ctypes.c_int]
        self._native.wa1_ik_zero_torso.restype = ctypes.c_int
        template = (share / 'vendor' / 'model' / 'robot_define_WA1.yaml').read_text()
        original = '/opt/ros/noetic/share/pico_runtime/uplimb/'
        template = template.replace(original + 'models/WA1_400L.urdf',
                                    str(share / 'vendor' / 'model' / 'WA1_400L.urdf'))
        template = template.replace(original + 'config/controller_paras_wa1.yaml',
                                    str(share / 'vendor' / 'model' / 'controller_paras_wa1.yaml'))
        cfg = Path(self._tmp.name) / 'robot_define_WA1.yaml'
        cfg.write_text(template)
        if self._native.wa1_init(str(cfg).encode()) != 0:
            raise RuntimeError(self._error())

    def _error(self) -> str:
        return self._native.wa1_last_error().decode(errors='replace')

    def forward(self, joints: dict[str, float], side: str) -> np.ndarray:
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        q = np.ascontiguousarray([joints[name] for name in JOINT_ORDER], dtype=np.float64)
        result = np.empty(7, dtype=np.float64)
        rc = self._native.wa1_fk(q.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                                1 if side == 'left' else 2,
                                result.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
        if rc != 0:
            raise RuntimeError(self._error())
        return result

    def solve(self, target_base_tcp: np.ndarray, side: str,
              reference_joints: dict[str, float]) -> list[dict]:
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        # Both transforms are BASE -> WAIST_PITCH. This matches the official
        # library's IK solver, which assumes a zero-valued torso state.
        self._scratch.qpos[:] = self.model.key_qpos[self.model.key('home').id]
        for name in TORSO:
            self._scratch.qpos[self.model.joint(name).qposadr[0]] = reference_joints[name]
        mujoco.mj_forward(self.model, self._scratch)
        base_inv = np.linalg.inv(_body_transform(self._scratch, 'BASE'))
        torso_current = base_inv @ _body_transform(self._scratch, 'WAIST_PITCH')
        for name in TORSO:
            self._scratch.qpos[self.model.joint(name).qposadr[0]] = 0.0
        mujoco.mj_forward(self.model, self._scratch)
        torso_zero = base_inv @ _body_transform(self._scratch, 'WAIST_PITCH')
        normalized = torso_zero @ np.linalg.inv(torso_current) @ _pose_to_matrix(target_base_tcp)
        target = np.ascontiguousarray(_matrix_to_pose(normalized))
        solution_array = np.empty((16, 7), dtype=np.float64)
        phi_array = np.empty(16, dtype=np.float64)
        ptr = ctypes.POINTER(ctypes.c_double)
        arm_joints = LEFT if side == 'left' else RIGHT
        count = self._native.wa1_ik_zero_torso(
            target.ctypes.data_as(ptr), 1 if side == 'left' else 2,
            reference_joints[arm_joints[6]],
            solution_array.ctypes.data_as(ptr), phi_array.ctypes.data_as(ptr), 16)
        if count < 0:
            raise RuntimeError(self._error())
        return [{'joints': dict(zip(arm_joints, solution_array[i].tolist())),
                 'phi_rad': float(phi_array[i])} for i in range(count)]

    def nearest(self, solutions: list[dict], reference_joints: dict[str, float]) -> dict | None:
        if not solutions:
            return None
        return min(solutions, key=lambda item: sum(
            (value - reference_joints[name]) ** 2 for name, value in item['joints'].items()))


def pose_error(actual: np.ndarray, target: np.ndarray) -> tuple[float, float]:
    position = float(np.linalg.norm(actual[:3] - target[:3]))
    dot = min(1.0, abs(float(np.dot(actual[3:], target[3:]))))
    rotation_deg = math.degrees(2 * math.acos(dot))
    return position, rotation_deg
