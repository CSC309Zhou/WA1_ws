# WA1 PC ROS 2 Humble 工作空间

本仓库正在 `v1-work` 分支整理发布版。已完成的本机系统由 ROS 2 接口、PICO4 数据接入、WA1 双臂遥操作和 PC 上的 ROS1↔ROS2 上肢桥组成。用户已完成关节、笛卡尔和双臂遥操作真机验收；桥的原理、接口和使用方式见 [native_bridge README](native_bridge/README.md)及各软件包说明。第 0 版保存在本地标签 `v0.0.0-experimental`。

## 目录

| 路径 | 用途 |
|---|---|
| [`src/upperlimb`](src/upperlimb) | 桥、遥操作和其他 PC 节点共用的 WA1 ROS 2 上肢消息与服务定义 |
| [`src/pico4_ros2`](src/pico4_ros2) | 与机器人无关的 PICO4 SDK 接入、消息和查看器 |
| [`src/wa1_pico4_teleop`](src/wa1_pico4_teleop) | PICO4 到 WA1 双臂的映射和操作状态机 |
| [`src/wa1_description`](src/wa1_description) | WA1 模型与可视化资源；完整装配模型有可选手部依赖 |
| [`native_bridge`](native_bridge) | ROS1↔ROS2 原生桥、基础镜像上的增量 Dockerfile、PC Docker 管理器和验收脚本 |

`native_bridge` 有 `COLCON_IGNORE`，因为其内部 C++ 包需要在同时具备 ROS1 和 ROS2 的桥接容器中编译；宿主机纯 Humble 工作空间只发现 `src/` 下的 ROS 2 包。正式增量构建使用 `native_bridge/Dockerfile` 和 `native_bridge/ros1_interfaces/`；基础镜像的历史配方只在 `native_bridge/docs/archive/` 保存。Docker 调用包装器是 `native_bridge/pc_docker.py`。`native_bridge/evidence` 保存第 0 版实验记录，新实验输出写入被 Git 忽略的 `native_bridge/results`。

目录去向、整理步骤和验收边界见 [代码整理计划](CODE_ORGANIZATION_PLAN.md)。计划发布冻结的基础镜像，其他用户拉取后在本地增量构建完整桥镜像；目前线上地址与 digest 尚未确定，见 [基础镜像说明](native_bridge/docs/BASE_IMAGE.md)。供应商文件分发和云端发布仍待后续处理。
