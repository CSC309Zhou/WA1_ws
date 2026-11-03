#!/usr/bin/env python3
"""Generate WA1 robot_cfg.yaml and convert it to MJCF."""

from __future__ import annotations

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
DEFAULT_CFG_PATH = ROOT / "default_cfg.yaml"
DEFAULT_CFG = yaml.safe_load(DEFAULT_CFG_PATH.read_text(encoding="utf-8"))
DEFAULT_INIT_STATE = copy.deepcopy(DEFAULT_CFG["init_state"])
ACTUATED_JOINT_DEFAULTS = list(DEFAULT_CFG["joint_defaults"]["normal"])
JOINT_CONFIG_FIELDS = ("type", "Tm", "Vm", "kp", "kd", "arma", "fric", "damp")
JOINT_CONFIG_INDEX = {field: index for index, field in enumerate(JOINT_CONFIG_FIELDS)}


def load_default_cfg() -> dict:
    return copy.deepcopy(DEFAULT_CFG)


def resolve_asset_dir(asset: str) -> Path:
    candidate = (ROOT / asset).resolve()
    if not candidate.is_dir():
        raise FileNotFoundError(f"asset folder not found: {asset}")
    return candidate


def find_single_urdf(asset_dir: Path) -> Path:
    files = sorted(asset_dir.glob("*.urdf"))
    if len(files) != 1:
        raise RuntimeError(f"expected exactly one URDF under {asset_dir}, found {len(files)}")
    return files[0]


def load_robot_config(asset_dir: Path) -> dict:
    path = asset_dir / "robot_cfg.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"robot_cfg.yaml not found under {asset_dir}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"invalid robot_cfg.yaml under {asset_dir}")
    return data


def robot_children(root: ET.Element, tag: str) -> list[ET.Element]:
    return [child for child in root if child.tag.rsplit("}", 1)[-1] == tag]


def remove_fixed_joint_axes(root: ET.Element) -> bool:
    changed = False
    for joint in robot_children(root, "joint"):
        if joint.attrib.get("type") != "fixed":
            continue
        for child in list(joint):
            if child.tag.rsplit("}", 1)[-1] == "axis":
                joint.remove(child)
                changed = True
    return changed


def _joint_links(joint: ET.Element) -> tuple[str | None, str | None]:
    parent = next((x for x in joint if x.tag.rsplit("}", 1)[-1] == "parent"), None)
    child = next((x for x in joint if x.tag.rsplit("}", 1)[-1] == "child"), None)
    return (parent.get("link") if parent is not None else None, child.get("link") if child is not None else None)


def _float(value: str | None, default: float = 0.0) -> float:
    try:
        return float(value) if value is not None else default
    except ValueError:
        return default


def _joint_limit(joint: ET.Element, name: str, default: float = 0.0) -> float:
    limit = next((child for child in joint if child.tag.rsplit("}", 1)[-1] == "limit"), None)
    if limit is None:
        return default
    return _float(limit.get(name), default)


def parse_joint_config(values: object, context: str = "joint config") -> dict[str, object]:
    if not isinstance(values, (list, tuple)) or len(values) != len(JOINT_CONFIG_FIELDS):
        raise RuntimeError(f"{context} must contain {len(JOINT_CONFIG_FIELDS)} values")
    if not isinstance(values[JOINT_CONFIG_INDEX["type"]], str):
        raise RuntimeError(f"{context}.type must be a string")
    parsed: dict[str, object] = {"type": values[JOINT_CONFIG_INDEX["type"]]}
    try:
        for field in JOINT_CONFIG_FIELDS[1:]:
            parsed[field] = float(values[JOINT_CONFIG_INDEX[field]])
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{context} contains non-numeric parameters") from exc
    return parsed


def format_scalar(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        precision = 6 if 0 < abs(value) < 1e-3 else 4
        rendered = f"{value:.{precision}f}".rstrip("0").rstrip(".")
        return "0" if rendered in {"", "-0"} else rendered
    return str(value)


def build_joint_config(joint: ET.Element) -> list[object]:
    config = list(ACTUATED_JOINT_DEFAULTS)
    if config[JOINT_CONFIG_INDEX["Tm"]] != 0:
        config[JOINT_CONFIG_INDEX["Tm"]] = _joint_limit(joint, "effort", float(config[JOINT_CONFIG_INDEX["Tm"]]))
    if config[JOINT_CONFIG_INDEX["Vm"]] != 0:
        config[JOINT_CONFIG_INDEX["Vm"]] = _joint_limit(joint, "velocity", float(config[JOINT_CONFIG_INDEX["Vm"]]))
    return config


def parse_robot_config(root: ET.Element, asset_name: str) -> dict:
    links = robot_children(root, "link")
    link_names = [link.get("name", "") for link in links]
    child_names = {_joint_links(joint)[1] for joint in robot_children(root, "joint")}
    root_link = next((name for name in link_names if name and name not in child_names), link_names[0])
    movable = [joint for joint in robot_children(root, "joint") if joint.get("type") not in {"fixed", "floating"}]
    ref_joints = [joint.get("name") for joint in movable if joint.get("name")]
    ref_links = [_joint_links(joint)[1] for joint in movable if _joint_links(joint)[1]]
    total_mass = sum(
        _float(next((x.get("value") for x in link.iter() if x.tag.rsplit("}", 1)[-1] == "mass"), None))
        for link in links
    )
    init_state = copy.deepcopy(DEFAULT_INIT_STATE)
    initial_overrides = DEFAULT_CFG.get("initial_joint_positions", {}).get(asset_name, {})
    init_state["joint_pos"] = {name: initial_overrides.get(name, 0) for name in ref_joints}
    actuated_joints = {}
    for joint in movable:
        name = joint.get("name")
        if not name or _joint_limit(joint, "effort") == 0:
            continue
        actuated_joints[name] = build_joint_config(joint)
    return {
        "robot_name": asset_name,
        "total_mass": round(total_mass, 4),
        "root_link": root_link,
        "root_height": 0,
        "init_state": init_state,
        "actuated_joints": actuated_joints,
        "policy_joint_names": list(actuated_joints),
        "ref_joint_order": ref_joints,
        "ref_link_order": ref_links,
    }


def build_yaml(config: dict) -> str:
    def flow(values: list[object]) -> str:
        return "[" + ",  ".join(format_scalar(value) for value in values) + "]"

    init_state = config["init_state"]
    lines = [
        "# robot_name: must match the asset directory and URDF robot name",
        f"robot_name: {config['robot_name']}",
        "# total_mass: sum of URDF link inertial mass values in kg",
        f"total_mass: {config['total_mass']}",
        "# root_link: robot body attached to the MuJoCo world",
        f"root_link: {config['root_link']}",
        "# root_height: root_link z distance to the lowest collision geometry at init pose",
        f"root_height: {config['root_height']}",
        "# Numeric formatting: values use decimal text for readability; lengths and masses are in SI units.",
        "# init_state: nominal pose used for the MuJoCo home keyframe",
        "init_state:",
        "  # root_pos.z: root_height + 20mm nominal clearance",
        f"  root_pos: {flow(init_state['root_pos'])}",
        "  # root_rot quaternion format: [x, y, z, w]",
        f"  root_rot: {flow(init_state['root_rot'])}",
        "  # joint_pos: initial position for every movable URDF joint, in joint order",
        "  joint_pos:",
    ]
    lines.extend(f"    {name}: {value}" for name, value in init_state["joint_pos"].items())
    lines.extend(
        [
            "actuated_joints:",
            "  #          [   type,   Tm,   Vm, kp, kd, arma, fric, damp]",
        ]
    )
    for name, values in config["actuated_joints"].items():
        lines.append(f"  {name}: {flow(values)}")
    lines.extend(["# WARNING: DO NOT MODIFY", "policy_joint_names:"])
    lines.extend(f"  - {name}" for name in config["policy_joint_names"])
    lines.extend(["# WARNING: DO NOT MODIFY", "ref_joint_order:"])
    lines.extend(f"  - {name}" for name in config["ref_joint_order"])
    lines.extend(["# WARNING: DO NOT MODIFY", "ref_link_order:"])
    lines.extend(f"  - {name}" for name in config["ref_link_order"])
    return "\n".join(lines) + "\n"


def update_root_height_from_mjcf(asset_dir: Path, xml_path: Path) -> float:
    from root_height import infer_mjcf_root_height

    config = load_robot_config(asset_dir)
    root_height = infer_mjcf_root_height(xml_path, config)
    if root_height is None:
        raise RuntimeError("could not infer root_height from robot collision geometry")
    root_height = round(root_height, 4)
    config["root_height"] = root_height
    init_state = config.setdefault("init_state", copy.deepcopy(DEFAULT_INIT_STATE))
    root_pos = list(init_state.get("root_pos", DEFAULT_INIT_STATE["root_pos"]))
    root_pos[2] = round(root_height + 0.02, 4)
    init_state["root_pos"] = root_pos
    (asset_dir / "robot_cfg.yaml").write_text(build_yaml(config), encoding="utf-8")
    return root_height


def generate_robot_cfg(asset_dir: Path, *, override: bool = False) -> Path:
    path = asset_dir / "robot_cfg.yaml"
    urdf_path = find_single_urdf(asset_dir)
    root = ET.parse(urdf_path).getroot()
    expected_name = asset_dir.name
    if root.get("name") != expected_name or urdf_path.stem != expected_name:
        raise RuntimeError("asset directory, URDF filename, and robot name must match")
    if path.exists() and not override:
        load_robot_config(asset_dir)
        return path
    config = parse_robot_config(root, expected_name)
    path.write_text(build_yaml(config), encoding="utf-8")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate WA1 robot_cfg.yaml and MJCF.")
    parser.add_argument("asset", help="asset folder, e.g. WA1_400L")
    parser.add_argument("--override", "-o", action="store_true", help="regenerate robot_cfg.yaml")
    parser.add_argument("--convert", action="store_true", help="generate MJCF after config generation")
    args = parser.parse_args()
    try:
        asset_dir = resolve_asset_dir(args.asset)
        cfg_path = generate_robot_cfg(asset_dir, override=args.override)
        print(f"[main] robot config: {cfg_path}")
        if args.convert:
            from convert_mjcf import convert_urdf_to_mjcf

            urdf_path = find_single_urdf(asset_dir)
            xml_path = convert_urdf_to_mjcf(urdf_path, asset_dir)
            root_height = update_root_height_from_mjcf(asset_dir, xml_path)
            xml_path = convert_urdf_to_mjcf(urdf_path, asset_dir)
            print(f"[main] root height: {root_height}")
            print(f"[main] MJCF: {xml_path}")
        return 0
    except (FileNotFoundError, RuntimeError, ImportError, ValueError) as exc:
        print(f"[main][error] {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
