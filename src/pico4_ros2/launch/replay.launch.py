"""Replay recorded SDK JSON lines without headset, vendor service, or robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('jsonl', description='SDK JSONL file to replay'),
        DeclareLaunchArgument('rate_hz', default_value='60.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        Node(package='pico4_ros2', executable='xr_receiver', output='screen',
             parameters=[{'replay_jsonl': LaunchConfiguration('jsonl'),
                          'replay_rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
                          'replay_loop': ParameterValue(LaunchConfiguration('loop'), value_type=bool)}]),
    ])
