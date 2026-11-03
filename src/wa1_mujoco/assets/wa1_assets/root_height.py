#!/usr/bin/env python3

from __future__ import annotations

import math
import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
Transform = tuple[Matrix3, Vector3]

IDENTITY_MATRIX: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
IDENTITY_TRANSFORM: Transform = (IDENTITY_MATRIX, (0.0, 0.0, 0.0))


def local_name(tag: str) -> str:
    return tag.split("}", 1)[-1] if "}" in tag else tag


def robot_children(robot_elem: ET.Element, tag_name: str) -> list[ET.Element]:
    return [elem for elem in robot_elem if local_name(elem.tag) == tag_name]


def find_direct_child(elem: ET.Element, tag_name: str) -> ET.Element | None:
    for child in elem:
        if local_name(child.tag) == tag_name:
            return child
    return None


def joint_links(joint_elem: ET.Element) -> tuple[str | None, str | None]:
    parent_elem = find_direct_child(joint_elem, "parent")
    child_elem = find_direct_child(joint_elem, "child")
    parent_link = parent_elem.attrib.get("link") if parent_elem is not None else None
    child_link = child_elem.attrib.get("link") if child_elem is not None else None
    return parent_link, child_link


def parse_float_values(raw_value: str | None, *, expected_len: int, default: Vector3) -> Vector3:
    if raw_value is None or not raw_value.strip():
        if expected_len != 3:
            raise RuntimeError("internal error: only 3-item defaults are supported")
        return default
    values = [float(item) for item in raw_value.split()]
    if len(values) != expected_len:
        raise RuntimeError(f"expected {expected_len} numeric values, got {raw_value!r}")
    return (values[0], values[1], values[2])


def rotation_matrix_from_rpy(rpy: Vector3) -> Matrix3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotation_matrix_from_axis_angle(axis: Vector3, angle: float) -> Matrix3:
    axis_len = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    if axis_len == 0:
        raise RuntimeError("joint axis cannot be zero length")
    x, y, z = axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return (
        (c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s),
        (y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s),
        (z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c),
    )


def matrix_multiply(lhs: Matrix3, rhs: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(lhs[row][k] * rhs[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def matrix_vector_multiply(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)
    )  # type: ignore[return-value]


def compose_transform(lhs: Transform, rhs: Transform) -> Transform:
    lhs_rot, lhs_pos = lhs
    rhs_rot, rhs_pos = rhs
    rotated_pos = matrix_vector_multiply(lhs_rot, rhs_pos)
    return (
        matrix_multiply(lhs_rot, rhs_rot),
        (
            lhs_pos[0] + rotated_pos[0],
            lhs_pos[1] + rotated_pos[1],
            lhs_pos[2] + rotated_pos[2],
        ),
    )


def transform_point(transform: Transform, point: Vector3) -> Vector3:
    rotation, position = transform
    rotated = matrix_vector_multiply(rotation, point)
    return (
        position[0] + rotated[0],
        position[1] + rotated[1],
        position[2] + rotated[2],
    )


def parse_origin_transform(elem: ET.Element) -> Transform:
    origin_elem = find_direct_child(elem, "origin")
    if origin_elem is None:
        return IDENTITY_TRANSFORM
    xyz = parse_float_values(origin_elem.attrib.get("xyz"), expected_len=3, default=(0.0, 0.0, 0.0))
    rpy = parse_float_values(origin_elem.attrib.get("rpy"), expected_len=3, default=(0.0, 0.0, 0.0))
    return rotation_matrix_from_rpy(rpy), xyz


def resolve_initial_joint_position(config: dict[str, Any], joint_name: str) -> float:
    joint_pos_cfg = ((config.get("init_state") or {}).get("joint_pos") or {})
    resolved = 0.0
    if not isinstance(joint_pos_cfg, dict):
        return resolved
    for pattern, raw_value in joint_pos_cfg.items():
        try:
            matched = re.fullmatch(str(pattern), joint_name) is not None
        except re.error:
            matched = str(pattern) == joint_name
        if matched:
            resolved = float(raw_value)
    return resolved


def joint_motion_transform(joint_elem: ET.Element, config: dict[str, Any]) -> Transform:
    joint_type = joint_elem.attrib.get("type", "").strip().lower()
    if joint_type not in {"revolute", "continuous", "prismatic"}:
        return IDENTITY_TRANSFORM

    axis_elem = find_direct_child(joint_elem, "axis")
    axis = parse_float_values(
        axis_elem.attrib.get("xyz") if axis_elem is not None else None,
        expected_len=3,
        default=(1.0, 0.0, 0.0),
    )
    joint_name = joint_elem.attrib.get("name") or ""
    joint_pos = resolve_initial_joint_position(config, joint_name)
    if joint_type == "prismatic":
        return (
            IDENTITY_MATRIX,
            (axis[0] * joint_pos, axis[1] * joint_pos, axis[2] * joint_pos),
        )
    return rotation_matrix_from_axis_angle(axis, joint_pos), (0.0, 0.0, 0.0)


def build_nominal_link_transforms(robot_elem: ET.Element, config: dict[str, Any]) -> dict[str, Transform]:
    root_link = str(config.get("root_link") or "")
    transforms: dict[str, Transform] = {root_link: IDENTITY_TRANSFORM}
    outgoing: dict[str, list[ET.Element]] = {}
    for joint_elem in robot_children(robot_elem, "joint"):
        parent_link, child_link = joint_links(joint_elem)
        if parent_link and child_link:
            outgoing.setdefault(parent_link, []).append(joint_elem)

    queue = [root_link]
    while queue:
        parent_link = queue.pop(0)
        parent_transform = transforms[parent_link]
        for joint_elem in outgoing.get(parent_link, []):
            _, child_link = joint_links(joint_elem)
            if not child_link or child_link in transforms:
                continue
            joint_transform = compose_transform(
                parse_origin_transform(joint_elem),
                joint_motion_transform(joint_elem, config),
            )
            transforms[child_link] = compose_transform(parent_transform, joint_transform)
            queue.append(child_link)
    return transforms


def mesh_path_for_urdf_filename(asset_dir: Path, filename: str) -> Path | None:
    raw = filename.strip()
    if not raw:
        return None
    if raw.startswith("package://"):
        suffix = raw.removeprefix("package://").split("/", 1)
        raw = suffix[1] if len(suffix) == 2 else suffix[0]
    path = Path(raw)
    if path.is_absolute():
        return path if path.is_file() else None

    candidates = [
        asset_dir / path,
        asset_dir / path.name,
        asset_dir / "meshes" / path.name,
        asset_dir / "mesh" / path.name,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def load_stl_vertices(path: Path) -> list[Vector3]:
    raw = path.read_bytes()
    if len(raw) >= 84:
        triangle_count = struct.unpack("<I", raw[80:84])[0]
        expected_len = 84 + triangle_count * 50
        if expected_len == len(raw):
            vertices: list[Vector3] = []
            offset = 84
            for _ in range(triangle_count):
                offset += 12
                for _ in range(3):
                    vertices.append(struct.unpack("<fff", raw[offset : offset + 12]))
                    offset += 12
                offset += 2
            return vertices

    vertices = []
    text = raw.decode("utf-8", errors="ignore")
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return vertices


def load_obj_vertices(path: Path) -> list[Vector3]:
    vertices = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) >= 4 and parts[0] == "v":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return vertices


def load_mesh_vertices(path: Path) -> list[Vector3]:
    suffix = path.suffix.lower()
    if suffix == ".stl":
        return load_stl_vertices(path)
    if suffix == ".obj":
        return load_obj_vertices(path)
    return []


def mesh_geometry_min_z(
    mesh_elem: ET.Element,
    asset_dir: Path,
    collision_transform: Transform,
) -> float | None:
    filename = mesh_elem.attrib.get("filename") or mesh_elem.attrib.get("file")
    if not filename:
        return None
    mesh_path = mesh_path_for_urdf_filename(asset_dir, filename)
    if mesh_path is None:
        return None
    vertices = load_mesh_vertices(mesh_path)
    if not vertices:
        return None

    scale = parse_float_values(mesh_elem.attrib.get("scale"), expected_len=3, default=(1.0, 1.0, 1.0))
    return min(
        transform_point(
            collision_transform,
            (vertex[0] * scale[0], vertex[1] * scale[1], vertex[2] * scale[2]),
        )[2]
        for vertex in vertices
    )


def collision_geometry_min_z(
    collision_elem: ET.Element,
    asset_dir: Path,
    collision_transform: Transform,
) -> float | None:
    geometry_elem = find_direct_child(collision_elem, "geometry")
    if geometry_elem is None:
        return None

    shape_elem = next(iter(geometry_elem), None)
    if shape_elem is None:
        return None

    shape_name = local_name(shape_elem.tag)
    rotation, position = collision_transform
    if shape_name == "sphere":
        radius = float(shape_elem.attrib["radius"])
        return position[2] - radius

    if shape_name == "cylinder":
        radius = float(shape_elem.attrib["radius"])
        half_length = float(shape_elem.attrib["length"]) / 2.0
        radial_z = math.sqrt(rotation[2][0] ** 2 + rotation[2][1] ** 2) * radius
        axial_z = abs(rotation[2][2]) * half_length
        return position[2] - radial_z - axial_z

    if shape_name == "box":
        size = parse_float_values(shape_elem.attrib.get("size"), expected_len=3, default=(0.0, 0.0, 0.0))
        half_sizes = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
        corners = [
            (x_sign * half_sizes[0], y_sign * half_sizes[1], z_sign * half_sizes[2])
            for x_sign in (-1.0, 1.0)
            for y_sign in (-1.0, 1.0)
            for z_sign in (-1.0, 1.0)
        ]
        return min(transform_point(collision_transform, corner)[2] for corner in corners)

    if shape_name == "mesh":
        return mesh_geometry_min_z(shape_elem, asset_dir, collision_transform)

    return None


def infer_urdf_geom_root_height_reference(
    robot_elem: ET.Element,
    asset_dir: Path,
    config: dict[str, Any],
) -> float | None:
    link_lookup = {link_elem.attrib.get("name"): link_elem for link_elem in robot_children(robot_elem, "link")}
    link_transforms = build_nominal_link_transforms(robot_elem, config)
    lowest_z: float | None = None

    for link_name, link_elem in link_lookup.items():
        if not isinstance(link_name, str):
            continue
        link_transform = link_transforms.get(link_name)
        if link_transform is None:
            continue
        for child_elem in link_elem:
            if local_name(child_elem.tag) != "collision":
                continue
            collision_transform = compose_transform(link_transform, parse_origin_transform(child_elem))
            min_z = collision_geometry_min_z(child_elem, asset_dir, collision_transform)
            if min_z is None:
                continue
            lowest_z = min_z if lowest_z is None else min(lowest_z, min_z)

    if lowest_z is None:
        return None
    root_height = -lowest_z
    return root_height if root_height > 0 else None


def _apply_initial_joint_positions(model, data, config: dict[str, Any]) -> None:
    import mujoco

    for joint_id in range(model.njnt):
        joint_type = model.jnt_type[joint_id]
        if joint_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not joint_name:
            continue
        qpos_id = int(model.jnt_qposadr[joint_id])
        if qpos_id < 0:
            continue
        data.qpos[qpos_id] = resolve_initial_joint_position(config, joint_name)


def _is_collision_geom(model, geom_id: int) -> bool:
    return (
        int(model.geom_group[geom_id]) == 3
        or int(model.geom_contype[geom_id]) != 0
        or int(model.geom_conaffinity[geom_id]) != 0
    )


def _mjcf_geom_min_z(model, data, geom_id: int) -> float | None:
    import mujoco

    geom_type = model.geom_type[geom_id]
    center_z = float(data.geom_xpos[geom_id][2])
    z_axis = data.geom_xmat[geom_id][6:9]
    size = model.geom_size[geom_id]

    if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        return center_z - float(size[0])

    if geom_type in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        z_extent = sum(abs(float(z_axis[idx])) * float(size[idx]) for idx in range(3))
        return center_z - z_extent

    if geom_type in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        radius = float(size[0])
        half_length = float(size[1])
        radial_z = math.sqrt(float(z_axis[0]) ** 2 + float(z_axis[1]) ** 2) * radius
        axial_z = abs(float(z_axis[2])) * half_length
        return center_z - radial_z - axial_z

    if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id < 0:
            return None
        start = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        if count <= 0:
            return None
        xpos = data.geom_xpos[geom_id]
        xmat = data.geom_xmat[geom_id]
        min_z: float | None = None
        for vertex in model.mesh_vert[start : start + count]:
            world_z = (
                float(xpos[2])
                + float(xmat[6]) * float(vertex[0])
                + float(xmat[7]) * float(vertex[1])
                + float(xmat[8]) * float(vertex[2])
            )
            min_z = world_z if min_z is None else min(min_z, world_z)
        return min_z

    return None


def infer_mjcf_root_height(xml_path: Path, config: dict[str, Any]) -> float | None:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    _apply_initial_joint_positions(model, data, config)
    mujoco.mj_forward(model, data)

    root_link = str(config.get("root_link") or "")
    root_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root_link)
    if root_body_id < 0:
        raise RuntimeError(f"root body {root_link!r} not found in MJCF: {xml_path}")

    lowest_z: float | None = None
    for geom_id in range(model.ngeom):
        if not _is_collision_geom(model, geom_id):
            continue
        min_z = _mjcf_geom_min_z(model, data, geom_id)
        if min_z is None:
            continue
        lowest_z = min_z if lowest_z is None else min(lowest_z, min_z)

    if lowest_z is None:
        return None

    root_height = float(data.xpos[root_body_id][2]) - lowest_z
    return root_height if root_height > 0 else None


def compare_root_height_estimators(
    xml_path: Path,
    robot_elem: ET.Element,
    asset_dir: Path,
    config: dict[str, Any],
) -> dict[str, float | None]:
    mjcf_height = infer_mjcf_root_height(xml_path, config)
    reference_height = infer_urdf_geom_root_height_reference(robot_elem, asset_dir, config)
    delta = None if mjcf_height is None or reference_height is None else mjcf_height - reference_height
    return {
        "mjcf": mjcf_height,
        "urdf_geom_reference": reference_height,
        "delta": delta,
    }
