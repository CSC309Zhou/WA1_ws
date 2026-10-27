# 增量构建所需的 ROS1 接口包材料

`upperlimb/` 只放 ROS1 catkin 包的构建元数据；WA1 消息和服务字段统一取自工作空间 [`src/upperlimb`](../../src/upperlimb) 的 `msg/`、`srv/`。`tf2_msgs/` 提供增量镜像中所需的 ROS1 `TFMessage` 定义。

[`../Dockerfile`](../Dockerfile) 将这些文件复制到镜像里的 `/wa1/ros1_interfaces/src/` 并编译。各目录的 `COLCON_IGNORE` 仅用于避免宿主 PC 的 Humble `colcon` 将它们识别为 ROS2 包；Dockerfile 不复制该标记到 catkin 工作区。
