#!/bin/bash
# This script runs INSIDE wa1-upperlimb-bridge:humble. Absolute paths below are
# in the container filesystem, not under the PC's ~/WA1 directory.
set -e
# ROS1 Noetic was installed in the PC's existing base image.
unset ROS_DISTRO
source /ros1_ws/install_isolated/setup.bash
# Dockerfile builds the WA1 ROS1 message/service package here.
source /wa1/ros1_interfaces/install/local_setup.bash
# Clear ROS1's distro marker before loading the ROS2 Humble underlay.
unset ROS_DISTRO
source /opt/ros/humble/setup.bash
# Dockerfile builds the matching WA1 ROS2 interfaces, conversion factories,
# and native C++ bridge into these three overlays, in dependency order.
source /wa1/ros2_interfaces/install/local_setup.bash
source /wa1/bridge_ws/install/local_setup.bash
source /wa1/native_ws/install/local_setup.bash
# Replace this setup shell with the command given by bridge.py / Docker CMD.
exec "$@"
