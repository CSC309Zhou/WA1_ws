#!/usr/bin/env python3
"""Convert URDF to MuJoCo MJCF with visual meshes, actuators, and equality constraints."""

from __future__ import annotations

import argparse
import sys
import tempfile
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
from pathlib import Path

from main import (
    DEFAULT_INIT_STATE,
    load_default_cfg,
    load_robot_config,
    parse_joint_config,
    remove_fixed_joint_axes,
    resolve_asset_dir,
    robot_children,
)

DEFAULT_CFG = load_default_cfg()
MUJOCO_DEFAULTS = DEFAULT_CFG["mujoco"]


def _meshdir_for_asset(asset_dir: Path) -> str:
    return "meshes/" if (asset_dir / "meshes").exists() else "mesh/"


def _normalize_mesh_filenames_for_mujoco(root: ET.Element, meshdir: str) -> None:
    prefix = meshdir.rstrip("/") + "/"
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        normalized = filename.replace("\\", "/")
        if normalized.startswith(prefix):
            mesh.set("filename", normalized[len(prefix) :])


def _format_attr_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return " ".join(_format_attr_value(item) for item in value)
    return str(value)


def _stringify_attrs(attrs: dict[str, object]) -> dict[str, str]:
    return {key: _format_attr_value(value) for key, value in attrs.items()}


def _write_import_urdf(
    urdf_path: Path,
    asset_dir: Path,
    config: dict,
    keep_fixed_body_inertials: bool,
) -> Path:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    remove_fixed_joint_axes(root)
    meshdir = _meshdir_for_asset(asset_dir)
    _normalize_mesh_filenames_for_mujoco(root, meshdir)

    mujoco_elem = ET.Element("mujoco")
    compiler_attrs = {
        "meshdir": meshdir,
        **_stringify_attrs(MUJOCO_DEFAULTS["compiler"]),
    }
    if keep_fixed_body_inertials:
        compiler_attrs["fusestatic"] = "false"
    ET.SubElement(mujoco_elem, "compiler", compiler_attrs)
    root.insert(0, mujoco_elem)

    root_link = config.get("root_link", "base_link")
    init_state = config.get("init_state", DEFAULT_INIT_STATE)
    init_pos = init_state.get("root_pos", DEFAULT_INIT_STATE["root_pos"])
    insert_idx = 1
    root.insert(insert_idx, ET.Element("link", {"name": "world"}))
    # Anchor BASE at its configured world pose instead of giving it a free joint.
    base_joint = ET.Element("joint", {"name": "base_joint", "type": "fixed"})
    ET.SubElement(base_joint, "parent", {"link": "world"})
    ET.SubElement(base_joint, "child", {"link": root_link})
    ET.SubElement(
        base_joint,
        "origin",
        {
            "xyz": f"{init_pos[0]} {init_pos[1]} {init_pos[2]}",
            "rpy": "0 0 0",
        },
    )
    root.insert(insert_idx + 1, base_joint)

    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        suffix=urdf_path.suffix,
        prefix=f"{urdf_path.stem}_mujoco_",
        dir=urdf_path.parent,
        delete=False,
    ) as tmp_file:
        tree.write(tmp_file, encoding="unicode", xml_declaration=True)
        return Path(tmp_file.name)


def _write_pretty_xml(root: ET.Element, output_path: Path) -> None:
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    lines = [line for line in xml_str.split("\n") if line.strip()]
    output_path.write_text("\n".join(lines), encoding="utf-8")


def _joint_dof_range(model, joint_id: int) -> range:
    dof_start = model.jnt_dofadr[joint_id]
    if dof_start < 0:
        return range(0)

    dof_end = model.nv
    for next_joint_id in range(joint_id + 1, model.njnt):
        next_dof_start = model.jnt_dofadr[next_joint_id]
        if next_dof_start >= 0:
            dof_end = next_dof_start
            break
    return range(dof_start, dof_end)


def _joint_params(config: dict, urdf_path: Path) -> dict[str, dict[str, float]]:
    """Read actuator parameters from robot_cfg.yaml's actuated_joints mapping."""
    actuated_joints = config.get("actuated_joints")
    if not isinstance(actuated_joints, dict):
        raise RuntimeError("robot config must define actuated_joints as a mapping")

    urdf_root = ET.parse(urdf_path).getroot()
    urdf_joints = {
        joint.get("name")
        for joint in robot_children(urdf_root, "joint")
        if joint.get("name") and joint.get("type") not in {"fixed", "floating"}
    }
    unknown_joints = sorted(set(actuated_joints) - urdf_joints)
    if unknown_joints:
        raise RuntimeError(f"actuated_joints contains joints missing from URDF: {unknown_joints}")

    params: dict[str, dict[str, float]] = {}
    for name, values in actuated_joints.items():
        parsed = parse_joint_config(values, f"actuated_joints.{name}")
        params[name] = {
            "type": parsed["type"],
            "Tm": parsed["Tm"],
            "kp": parsed["kp"],
            "kd": parsed["kd"],
            "arma": parsed["arma"],
            "static_friction": parsed["fric"],
            "viscous_friction": parsed["damp"],
        }
    return params


def _attach_endoscope(root: ET.Element, asset_dir: Path) -> None:
    """Attach the endoscope frame to WA1's right TCP with an identity transform."""
    if asset_dir.name != "WA1_400L":
        return

    mesh_path = asset_dir / "meshes" / "endoscope.STL"
    if not mesh_path.is_file() or mesh_path.read_bytes().startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise FileNotFoundError(f"endoscope mesh is missing or is an LFS pointer: {mesh_path}")

    assets = root.find("asset")
    tcp_body = root.find(".//body[@name='TCP_R']")
    if assets is None or tcp_body is None:
        raise RuntimeError("WA1 MJCF must contain an asset section and TCP_R body")

    ET.SubElement(assets, "mesh", {"name": "endoscope_mesh", "file": "endoscope.STL"})
    endoscope = ET.SubElement(tcp_body, "body", {"name": "endoscope", "pos": "0.01 0 0", "quat": "0.7071068 0 0.7071068 0"})
    # Mass, center of mass, and inertia come from endoscope/urdf/endoscope.urdf.
    ET.SubElement(
        endoscope,
        "inertial",
        {
            "pos": "-2.8843e-08 1.2151e-06 -0.086544",
            "mass": "0.071614",
            "fullinertia": "0.00057368 0.00057368 5.5728e-06 -8.002e-12 -5.5665e-10 2.7205e-08",
        },
    )
    ET.SubElement(endoscope, "geom", {"name": "endoscope_visual", "type": "mesh", "mesh": "endoscope_mesh", "class": "visual"})
    ET.SubElement(endoscope, "geom", {"name": "endoscope_collision", "type": "mesh", "mesh": "endoscope_mesh", "class": "collision"})
    # The original endoscope_tcp link is fixed 0.4 m along the tool's -Z axis.
    ET.SubElement(endoscope, "site", {"name": "endoscope_tcp", "pos": "0 0 -0.4", "size": "0.005", "rgba": "1 0.2 0.2 1"})


def _apply_robot_gravity_compensation(root: ET.Element, asset_dir: Path) -> None:
    ratio = MUJOCO_DEFAULTS.get("robot_gravcomp", {}).get(asset_dir.name)
    if ratio is None:
        return
    base = root.find("./worldbody/body[@name='BASE']")
    if base is None:
        raise RuntimeError("WA1 MJCF must contain the BASE body")
    for body in base.iter("body"):
        body.set("gravcomp", _format_attr_value(ratio))


def _add_home_keyframe(root: ET.Element, xml_path: Path, config: dict, asset_dir: Path) -> None:
    """Store the configured pose and matching servo targets in the MJCF."""
    if asset_dir.name not in MUJOCO_DEFAULTS["position_servo_joints"]:
        return

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    joint_positions = config["init_state"]["joint_pos"]
    qpos = model.qpos0.tolist()
    for name, value in joint_positions.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError(f"initial joint {name!r} is missing from the MJCF")
        if model.jnt_type[joint_id] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            raise RuntimeError(f"initial joint {name!r} is not a one-DoF joint")
        qpos[model.jnt_qposadr[joint_id]] = float(value)

    ctrl = [0.0] * model.nu
    for actuator in root.find("actuator"):
        if actuator.tag != "position":
            continue
        name = actuator.get("name")
        joint_name = actuator.get("joint")
        if joint_name not in joint_positions:
            raise RuntimeError(f"missing initial position for servo joint {joint_name!r}")
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id < 0:
            raise RuntimeError(f"position actuator {name!r} is missing from the MJCF")
        ctrl[actuator_id] = float(joint_positions[joint_name])

    keyframes = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframes, "key", {"name": "home", "qpos": _format_attr_value(qpos), "ctrl": _format_attr_value(ctrl)})


def convert_urdf_to_mjcf(
    urdf_path: Path,
    asset_dir: Path,
    config: dict | None = None,
    keep_fixed_body_inertials: bool = True,
) -> Path:
    import mujoco

    if config is None:
        config = load_robot_config(asset_dir)
    joint_params = _joint_params(config, urdf_path)

    xml_path = asset_dir / f"{urdf_path.stem}.xml"
    import_urdf_path = _write_import_urdf(
        urdf_path,
        asset_dir,
        config,
        keep_fixed_body_inertials,
    )

    # Step 1: MuJoCo base conversion and direct DOF parameter injection.
    try:
        model = mujoco.MjModel.from_xml_path(str(import_urdf_path))
        for joint_name, params in joint_params.items():
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise RuntimeError(f"actuated joint {joint_name!r} is missing from imported MuJoCo model")
            for dof_id in _joint_dof_range(model, joint_id):
                model.dof_armature[dof_id] = params["arma"]
                # MuJoCo represents velocity-proportional friction as damping
                # and a single dry-friction magnitude as frictionloss.
                model.dof_damping[dof_id] = params["viscous_friction"]
                model.dof_frictionloss[dof_id] = params["static_friction"]
        mujoco.mj_saveLastXML(str(xml_path), model)
    finally:
        import_urdf_path.unlink(missing_ok=True)

    # Step 2: XML post-process for defaults, collision classes, actuators, and loop constraints.
    tree = ET.parse(xml_path)
    root = tree.getroot()
    for mesh in root.findall("./asset/mesh"):
        # MuJoCo 3.4 adds this inferred attribute when saving imported STL.
        mesh.attrib.pop("content_type", None)

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", _meshdir_for_asset(asset_dir))

    default_section = root.find("default")
    if default_section is None:
        default_section = ET.Element("default")
        idx = list(root).index(compiler) + 1 if compiler in root else 0
        root.insert(idx, default_section)
    ET.SubElement(default_section, "joint", _stringify_attrs(MUJOCO_DEFAULTS["default_joint"]))
    def_visual = ET.SubElement(default_section, "default", {"class": "visual"})
    ET.SubElement(def_visual, "geom", _stringify_attrs(MUJOCO_DEFAULTS["visual_geom"]))
    def_collision = ET.SubElement(default_section, "default", {"class": "collision"})
    ET.SubElement(def_collision, "geom", _stringify_attrs(MUJOCO_DEFAULTS["collision_geom"]))

    worldbody = root.find("worldbody")
    if worldbody is not None:
        for geom in worldbody.iter("geom"):
            is_visual = geom.get("contype") == "0" and geom.get("conaffinity") == "0" and geom.get("density") == "0"
            geom.set("class", "visual" if is_visual else "collision")
            if geom.get("class") == "collision" and geom.get("type", "sphere") == "cylinder":
                geom.set("type", "capsule")

    actuator_section = root.find("actuator")
    if actuator_section is None:
        actuator_section = ET.SubElement(root, "actuator")
    position_servo_names = set(MUJOCO_DEFAULTS["position_servo_joints"].get(asset_dir.name, []))
    servo_gains = MUJOCO_DEFAULTS["position_servo_gains"].get(asset_dir.name, {})
    missing_servo_joints = position_servo_names - joint_params.keys()
    if missing_servo_joints:
        raise RuntimeError(f"position servo joints missing from robot config: {sorted(missing_servo_joints)}")
    for joint in root.iter("joint"):
        if joint.get("type") == "free":
            continue
        name = joint.get("name")
        if not name or name not in joint_params:
            continue
        params = joint_params[name]
        if params["type"] != "generic":
            raise RuntimeError(
                f"unsupported actuator type {params['type']!r} for joint {name!r}; expected 'generic'"
            )
        joint.set("armature", _format_attr_value(params["arma"]))
        joint.set("damping", _format_attr_value(params["viscous_friction"]))
        joint.set("frictionloss", _format_attr_value(params["static_friction"]))
        if params["Tm"] == 0:
            continue
        if name in position_servo_names:
            joint_range = joint.get("range")
            if not joint_range:
                raise RuntimeError(f"position servo joint {name!r} must have a joint range")
            gains = {**servo_gains.get("default", {}), **servo_gains.get("joint_overrides", {}).get(name, {})}
            ET.SubElement(
                actuator_section,
                "position",
                {
                    "name": name,
                    "joint": name,
                    "gear": "1",
                    "kp": _format_attr_value(gains.get("kp", params["kp"])),
                    "kv": _format_attr_value(gains.get("kv", params["kd"])),
                    "ctrllimited": "true",
                    "ctrlrange": joint_range,
                    "forcelimited": "true",
                    "forcerange": f"-{params['Tm']} {params['Tm']}",
                },
            )
            continue
        ET.SubElement(
            actuator_section,
            "motor",
            {
                "name": name,
                "joint": name,
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": f"-{params['Tm']} {params['Tm']}",
            },
        )

    urdf_root = ET.parse(urdf_path).getroot()
    loop_joints = robot_children(urdf_root, "loop_joint")
    if loop_joints:
        equality_defaults = _stringify_attrs(MUJOCO_DEFAULTS["equality"]["connect"])
        equality_section = ET.SubElement(root, "equality")
        for loop_elem in loop_joints:
            link1_elem = next((child for child in loop_elem if child.tag == "link1"), None)
            link2_elem = next((child for child in loop_elem if child.tag == "link2"), None)
            if link1_elem is None or link2_elem is None:
                continue
            ET.SubElement(
                equality_section,
                "connect",
                {
                    "name": loop_elem.get("name", ""),
                    "body1": link1_elem.get("link", ""),
                    "body2": link2_elem.get("link", ""),
                    "anchor": link1_elem.get("xyz", "0 0 0"),
                    **equality_defaults,
                },
            )
    _attach_endoscope(root, asset_dir)
    _apply_robot_gravity_compensation(root, asset_dir)
    _write_pretty_xml(root, xml_path)
    _add_home_keyframe(root, xml_path, config, asset_dir)
    _write_pretty_xml(root, xml_path)

    # Step 3: Emit the companion scene.xml.
    scene_path = asset_dir / "scene.xml"
    scene_cfg = MUJOCO_DEFAULTS["scene"]
    scene_root = ET.Element("mujoco", {"model": f"{urdf_path.stem}_scene"})
    ET.SubElement(scene_root, "include", {"file": f"{urdf_path.stem}.xml"})
    ET.SubElement(scene_root, "option", _stringify_attrs(scene_cfg["option"]))

    visual = ET.SubElement(scene_root, "visual")
    ET.SubElement(visual, "headlight", _stringify_attrs(scene_cfg["headlight"]))
    ET.SubElement(visual, "rgba", _stringify_attrs({"haze": scene_cfg["haze"]}))

    asset = ET.SubElement(scene_root, "asset")
    ET.SubElement(asset, "texture", _stringify_attrs(scene_cfg["grid_texture"]))
    ET.SubElement(asset, "material", _stringify_attrs(scene_cfg["grid_material"]))

    worldbody = ET.SubElement(scene_root, "worldbody")
    ET.SubElement(worldbody, "light", _stringify_attrs(scene_cfg["light"]))
    ET.SubElement(worldbody, "geom", _stringify_attrs(scene_cfg["floor"]))
    _write_pretty_xml(scene_root, scene_path)

    print(f"[convert_mjcf] wrote: {xml_path}")
    return xml_path


def main() -> int:
    try:
        parser = argparse.ArgumentParser(description="Convert URDF to MuJoCo MJCF.")
        parser.add_argument("asset", help="Asset folder name or path.")
        args = parser.parse_args()

        asset_dir = resolve_asset_dir(args.asset)
        urdf_path = next(path for path in asset_dir.glob("*.urdf"))
        convert_urdf_to_mjcf(urdf_path, asset_dir, keep_fixed_body_inertials=True)

        return 0
    except (RuntimeError, FileNotFoundError, ImportError) as exc:
        print(f"[convert_mjcf][error] {exc}", flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
