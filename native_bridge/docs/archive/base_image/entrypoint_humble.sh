#!/bin/bash
set -e
unset ROS_DISTRO
source /opt/ros/humble/setup.bash
unset ROS_DISTRO
source /ros1_ws/install_isolated/setup.bash
unset ROS_DISTRO
source /upperlimb_ws/devel/setup.bash
unset ROS_DISTRO
source /bridge_ws/install/setup.bash
exec ros2 run ros1_bridge dynamic_bridge "$@"
