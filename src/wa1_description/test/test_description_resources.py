from pathlib import Path
import importlib.util
import re
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MESH_URI_PREFIX = "package://wa1_description/meshes/"


def test_manifest_declares_build_and_runtime_dependencies():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    declared_dependencies = {
        (element.tag, (element.text or "").strip())
        for element in root
        if element.tag.endswith("depend")
    }

    expected_dependencies = {
        ("buildtool_depend", "ament_cmake"),
        ("exec_depend", "ament_index_python"),
        ("exec_depend", "joint_state_publisher_gui"),
        ("exec_depend", "launch"),
        ("exec_depend", "launch_ros"),
        ("exec_depend", "robot_state_publisher"),
        ("exec_depend", "rviz2"),
        ("test_depend", "python3-yaml"),
    }

    assert expected_dependencies <= declared_dependencies


def test_cmake_installs_all_runtime_resources_and_registers_tests():
    cmake_source = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    install_rule = re.search(
        r"install\s*\(\s*DIRECTORY\s+(.*?)\s+DESTINATION\s+share/\$\{PROJECT_NAME\}",
        cmake_source,
        flags=re.DOTALL,
    )

    assert install_rule is not None
    assert {"launch", "meshes", "rviz", "urdf"} <= set(install_rule.group(1).split())
    assert "if(BUILD_TESTING)" in cmake_source
    assert "ament_add_pytest_test" in cmake_source


def test_cmake_excludes_generated_python_cache_from_installed_resources():
    cmake_source = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert re.search(r'PATTERN\s+"__pycache__"\s+EXCLUDE', cmake_source)


def test_every_urdf_mesh_uses_an_existing_package_resource():
    urdf_root = ET.parse(PACKAGE_ROOT / "urdf" / "WA1_400L.urdf").getroot()
    mesh_filenames = [mesh.attrib["filename"] for mesh in urdf_root.iter("mesh")]

    assert mesh_filenames
    for filename in mesh_filenames:
        assert filename.startswith(MESH_URI_PREFIX)
        mesh_path = PACKAGE_ROOT / "meshes" / filename.removeprefix(MESH_URI_PREFIX)
        assert mesh_path.is_file(), f"Missing mesh resource: {mesh_path.name}"


def _load_display_launch(monkeypatch, tmp_path):
    import ament_index_python.packages

    def source_package_share(package_name):
        assert package_name == "wa1_description"
        return str(PACKAGE_ROOT)

    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    monkeypatch.setattr(
        ament_index_python.packages,
        "get_package_share_directory",
        source_package_share,
    )
    launch_path = PACKAGE_ROOT / "launch" / "display.launch.py"
    spec = importlib.util.spec_from_file_location("wa1_display_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _node_parameters(node):
    parameters = {}
    for parameter_group in evaluate_parameters(
        LaunchContext(), node._Node__parameters
    ):
        if isinstance(parameter_group, dict):
            parameters.update(parameter_group)
    return parameters


def test_display_launch_starts_publishers_and_rviz_with_shared_description(
    monkeypatch, tmp_path
):
    module = _load_display_launch(monkeypatch, tmp_path)
    launch_description = module.generate_launch_description()
    nodes = {
        action.node_package: action
        for action in launch_description.entities
        if isinstance(action, Node)
    }

    assert set(nodes) == {
        "joint_state_publisher_gui",
        "robot_state_publisher",
        "rviz2",
    }
    expected_description = (PACKAGE_ROOT / "urdf" / "WA1_400L.urdf").read_text(
        encoding="utf-8"
    )
    robot_parameters = _node_parameters(nodes["robot_state_publisher"])
    joint_parameters = _node_parameters(nodes["joint_state_publisher_gui"])
    assert robot_parameters["robot_description"] == expected_description
    assert joint_parameters["robot_description"] == expected_description
    assert robot_parameters["use_sim_time"] is False
    assert nodes["rviz2"]._Node__arguments == [
        "-d",
        str(PACKAGE_ROOT / "rviz" / "wa1_400l.rviz"),
    ]


def test_rviz_config_shows_robot_in_base_frame():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / "wa1_400l.rviz").read_text(encoding="utf-8")
    )
    manager = config["Visualization Manager"]

    assert manager["Global Options"]["Fixed Frame"] == "BASE"
    robot_models = [
        display
        for display in manager["Displays"]
        if display.get("Class") == "rviz_default_plugins/RobotModel"
    ]
    assert len(robot_models) == 1
    assert robot_models[0]["Enabled"] is True
