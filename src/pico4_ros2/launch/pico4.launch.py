"""Run the copied XRoboToolkit PC service and its robot-independent ROS 2 adapter."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    root = os.path.join(get_package_share_directory('pico4_ros2'), 'vendor', 'roboticsservice')
    library_dirs = [root, os.path.join(root, 'lib'), os.path.join(root, 'SDK', 'x64')]
    ld_path = ':'.join(library_dirs + ([os.environ['LD_LIBRARY_PATH']]
                                     if os.environ.get('LD_LIBRARY_PATH') else []))
    env = {
        'LD_LIBRARY_PATH': ld_path,
        'QT_PLUGIN_PATH': os.path.join(root, 'plugins'),
        'QT_QML_PATH': os.path.join(root, 'qml'),
    }
    service = ExecuteProcess(
        cmd=[os.path.join(root, 'RoboticsServiceProcess')],
        cwd=root,
        output='screen',
        additional_env=env,
        condition=IfCondition(LaunchConfiguration('start_vendor_service')),
    )
    receiver = Node(
        package='pico4_ros2', executable='xr_receiver', output='screen',
        additional_env=env,
        parameters=[{'publish_raw_json': LaunchConfiguration('publish_raw_json')}],
    )
    return [service, receiver]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_vendor_service', default_value='true'),
        DeclareLaunchArgument('publish_raw_json', default_value='false'),
        OpaqueFunction(function=_start),
    ])
