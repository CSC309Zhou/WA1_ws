"""Start only the local, read-only PICO4 browser pose viewer."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('pico4_ros2'),
                          'config', 'pose_viewer.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        Node(package='pico4_ros2', executable='pose_viewer', output='screen',
             parameters=[config, {'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}]),
    ])
