#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${WA1_SENSOR_IMAGE:-10.51.33.201:30002/navi_project/sensor:22CUDA_v1}"
APRILTAG_COMMIT=b7c0ebe9aa20f82ec7a828579004f9e706bfecd9
mkdir -p "$ROOT/catkin_ws/src"
if [[ ! -d "$ROOT/catkin_ws/src/apriltag/.git" ]]; then
  git clone https://github.com/AprilRobotics/apriltag.git "$ROOT/catkin_ws/src/apriltag"
fi
if [[ "$(git -C "$ROOT/catkin_ws/src/apriltag" rev-parse HEAD)" != "$APRILTAG_COMMIT" ]]; then
  git -C "$ROOT/catkin_ws/src/apriltag" fetch origin "$APRILTAG_COMMIT"
  git -C "$ROOT/catkin_ws/src/apriltag" checkout --detach "$APRILTAG_COMMIT"
fi
test -f "$ROOT/catkin_ws/src/csc309_apriltag_pose/package.xml"
docker run --rm --network host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -v "$ROOT/catkin_ws:/work" -w /work \
  --entrypoint /bin/bash "$IMAGE" -lc \
  'source /opt/ros/noetic/setup.bash && catkin_make_isolated --install -j4 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF'
