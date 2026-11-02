#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${WA1_SENSOR_IMAGE:-10.51.33.201:30002/navi_project/sensor:22CUDA_v1}"
MASTER="${WA1_ROS1_MASTER_URI:-http://192.168.217.1:11311}"
JETSON_IP="${WA1_JETSON_WIRED_IP:-192.168.217.100}"
case "${1:-}" in
  '') DOCKER_ARGS=() ;;
  --detach) DOCKER_ARGS=(-d) ;;
  *) echo 'Usage: bash run.sh [--detach]' >&2; exit 2 ;;
esac
test -f "$ROOT/catkin_ws/install_isolated/setup.bash"
docker run --rm "${DOCKER_ARGS[@]}" --name csc309_apriltag --network host \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONUNBUFFERED=1 \
  -e "ROS_MASTER_URI=$MASTER" -e "ROS_IP=$JETSON_IP" \
  -v "$ROOT/catkin_ws:/work:ro" -w /work \
  --entrypoint /bin/bash "$IMAGE" -lc \
  'mkdir -p /tmp/.ros/log && source /opt/ros/noetic/setup.bash && source /work/install_isolated/setup.bash && export PYTHONPATH="/work/install_isolated/lib/python3.10/site-packages:${PYTHONPATH:-}" && export LD_LIBRARY_PATH="/work/install_isolated/lib:${LD_LIBRARY_PATH:-}" && exec roslaunch csc309_apriltag_pose pipeline.launch'
