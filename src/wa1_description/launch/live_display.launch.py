from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('wa1_description'))
    description = (share / 'urdf' / 'WA1_400L.urdf').read_text(encoding='utf-8')
    parameters = [{'robot_description': description, 'use_sim_time': False}]
    tf_remappings = [('/tf', '/wa1_visualization/tf'),
                     ('/tf_static', '/wa1_visualization/tf_static')]
    return LaunchDescription([
        Node(package='wa1_description', executable='live_joint_states.py',
             namespace='wa1_visualization', parameters=parameters, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='wa1_visualization', parameters=parameters,
             remappings=tf_remappings + [
                 ('robot_description', '/robot_description')], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'wa1_400l.rviz')],
             remappings=tf_remappings, output='screen'),
    ])
