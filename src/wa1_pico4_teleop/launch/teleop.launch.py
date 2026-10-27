"""Start the complete WA1 dual-arm PICO4 teleoperation node."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wa1_pico4_teleop'),
                          'config', 'wa1_teleop.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='true',
                              description='Enable complete dual-arm control and startup MoveJ'),
        DeclareLaunchArgument('translation_scale', default_value='1.0',
                              description='Hand translation to TCP translation ratio [0.5, 1.5]'),
        Node(package='wa1_pico4_teleop', executable='wa1_teleop', output='screen',
             parameters=[config, {
                 'enable_control': LaunchConfiguration('enable_control'),
                 'translation_scale': ParameterValue(
                     LaunchConfiguration('translation_scale'), value_type=float),
             }]),
    ])
