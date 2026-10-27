import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("wa1_description")
    xacro_file = os.path.join(pkg_share, "urdf", "assembly.xacro")
    config_file = LaunchConfiguration("config")
    robot_type = LaunchConfiguration("robot_type")
    robot_name = LaunchConfiguration("robot_name")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " config:=",
            config_file,
            " robot_type:=",
            robot_type,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value=EnvironmentVariable("ROBOT_NAME", default_value="zj_humanoid"),
            description="Robot namespace, must match the joint_states publisher",
        ),
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(pkg_share, "config", "options.yaml"),
            description="Robot configuration file",
        ),
        DeclareLaunchArgument(
            "robot_type",
            default_value=EnvironmentVariable("ROBOT_TYPE", default_value="WA1-S"),
            description="Robot model type",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            namespace=robot_name,
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=robot_name,
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=robot_name,
            output="screen",
        ),
    ])
