# Frozen linux/amd64 Noetic + Humble foundation published in GHCR.
ARG BASE_IMAGE=ghcr.io/csc309zhou/wa1-ros1-humble-base@sha256:cc808811ffd85e3b0f431a345e395394ab8bdefb079fd1a31737f7746f6c7963
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-c"]
ENV CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2

# Reuse the PC's compiled Noetic/Humble foundation, but rebuild both interface
# packages and all conversion factories. The old image has empty upperlimb factories.
COPY native_bridge/ros1_interfaces/upperlimb/CMakeLists.txt native_bridge/ros1_interfaces/upperlimb/package.xml /wa1/ros1_interfaces/src/upperlimb/
COPY src/upperlimb/msg /wa1/ros1_interfaces/src/upperlimb/msg
COPY src/upperlimb/srv /wa1/ros1_interfaces/src/upperlimb/srv
# Keep COLCON_IGNORE on the host, but do not copy it into the catkin workspace:
# catkin_pkg also treats COLCON_IGNORE as a package exclusion marker.
COPY native_bridge/ros1_interfaces/tf2_msgs/CMakeLists.txt native_bridge/ros1_interfaces/tf2_msgs/package.xml /wa1/ros1_interfaces/src/tf2_msgs/
COPY native_bridge/ros1_interfaces/tf2_msgs/msg /wa1/ros1_interfaces/src/tf2_msgs/msg
RUN source /ros1_ws/install_isolated/setup.bash && \
    cd /wa1/ros1_interfaces && catkin_make install -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF

COPY src/upperlimb /wa1/ros2_interfaces/src/upperlimb
RUN source /opt/ros/humble/setup.bash && cd /wa1/ros2_interfaces && \
    colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN test "$(git -C /bridge_ws/src/ros1_bridge rev-parse HEAD)" = 611755fd917285316051cbea80507e8b2f6b7ec1 && \
    mkdir -p /wa1/bridge_ws/src && cp -a /bridge_ws/src/ros1_bridge /wa1/bridge_ws/src/ros1_bridge
COPY native_bridge/patch_transport.py /wa1/patch_transport.py
RUN --mount=type=cache,target=/wa1/bridge_ws/build \
    python3 /wa1/patch_transport.py && \
    export MAKEFLAGS=-j4 CMAKE_BUILD_PARALLEL_LEVEL=4 && \
    source /wa1/ros1_interfaces/install/setup.bash && unset ROS_DISTRO && \
    source /opt/ros/humble/setup.bash && source /wa1/ros2_interfaces/install/setup.bash && \
    cd /wa1/bridge_ws && colcon build --executor sequential --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

COPY native_bridge/src/wa1_native_bridge /wa1/native_ws/src/wa1_native_bridge
RUN source /wa1/ros1_interfaces/install/setup.bash && unset ROS_DISTRO && \
    source /opt/ros/humble/setup.bash && source /wa1/ros2_interfaces/install/setup.bash && \
    source /wa1/bridge_ws/install/setup.bash && cd /wa1/native_ws && \
    colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

COPY native_bridge/entrypoint.sh /wa1/entrypoint.sh
COPY native_bridge/config/fastdds_pc.xml /wa1/fastdds_pc.xml
ENV ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/wa1/fastdds_pc.xml
ENTRYPOINT ["/bin/bash", "/wa1/entrypoint.sh"]
CMD ["ros2", "run", "wa1_native_bridge", "upperlimb_bridge"]
