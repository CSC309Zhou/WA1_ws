# 桥接基础镜像与增量构建

本项目发布冻结的 ROS1 Noetic + ROS2 Humble **基础镜像**。使用者拉取该镜像后，在本地用 [`../Dockerfile`](../Dockerfile) 编译 WA1 接口、转换库和上肢专用桥，得到 `wa1-upperlimb-bridge:humble`。

## 已发布镜像

| 项目 | 值 |
|---|---|
| 仓库 | `ghcr.io/csc309zhou/wa1-ros1-humble-base`，私有 GHCR 包，与 `CSC309Zhou/WA1_ws` 关联并继承仓库访问权限 |
| 版本标签 | `v1.0.0-amd64` |
| 固定引用 | `ghcr.io/csc309zhou/wa1-ros1-humble-base@sha256:cc808811ffd85e3b0f431a345e395394ab8bdefb079fd1a31737f7746f6c7963` |
| 平台 | `linux/amd64` |
| 本机原始标签 | `wa1_ros1_bridge_humble:latest` |

固定引用来自 GHCR 的推送结果，并已通过按 digest 拉取核对。当前 Docker 镜像存储中的本机 ID 恰好与远端 OCI image index digest 相同；构建配置以远端 digest 为准。`native_bridge/Dockerfile` 与 `bridge.py` 都默认使用这个固定引用。

增量 Dockerfile 要求基础镜像提供：

- `/ros1_ws/install_isolated/setup.bash`：在 Ubuntu 22.04 上构建的 ROS1 Noetic 环境；
- `/opt/ros/humble/setup.bash`：ROS2 Humble 环境；
- `/bridge_ws/src/ros1_bridge`：本机基础镜像内的提交已只读核对为 `611755fd917285316051cbea80507e8b2f6b7ec1`；Dockerfile 在构建中再次检查该值；
- `catkin_make`、`colcon`、C++ 编译器及构建 `upperlimb`、`tf2_msgs`、`ros1_bridge` 所需依赖。

当前镜像只覆盖 `linux/amd64`。历史 [`Dockerfile.humble`](archive/base_image/Dockerfile.humble) 作为开发记录保存；正式增量构建使用 [`../Dockerfile`](../Dockerfile)。

## 拉取与增量构建

使用有该私有 GHCR 包读取权限的 GitHub 账号完成 `docker login ghcr.io` 后，在 `WA1_ws` 根目录执行：

```bash
python3 native_bridge/pc_docker.py pull \
  ghcr.io/csc309zhou/wa1-ros1-humble-base@sha256:cc808811ffd85e3b0f431a345e395394ab8bdefb079fd1a31737f7746f6c7963
python3 native_bridge/bridge.py build
python3 native_bridge/tests/run_mock_test.py --seconds 3
```

Docker 的构建上下文是 `WA1_ws` 根目录，因为 [`src/upperlimb`](../../src/upperlimb) 的 ROS2 消息和服务定义也要复制到镜像里。`native_bridge/ros1_interfaces/` 只维护 ROS1 包构建元数据和额外的 TF 消息定义；不要复制出第二份 WA1 消息字段。

`build --base-image <镜像引用>` 或环境变量 `WA1_BASE_IMAGE` 可以覆盖默认基础镜像。`run_mock_test.py` 仅连接 PC 本机的模拟 ROS1 Master，不接入机器人。
