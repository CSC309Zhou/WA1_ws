# 桥接基础镜像与增量构建

本项目计划只发布冻结的 ROS1 Noetic + ROS2 Humble **基础镜像**。使用者拉取该镜像后，在本地用 [`../Dockerfile`](../Dockerfile) 编译 WA1 接口、转换库和上肢专用桥，得到 `wa1-upperlimb-bridge:humble`。基础镜像并不是可直接运行 WA1 上肢桥的最终镜像。

## 当前状态

基础镜像**尚未上传到镜像仓库**。本机已用于增量构建的标签是 `wa1_ros1_bridge_humble:latest`，本机 `linux/amd64` 镜像 ID 为 `sha256:cc808811ffd85e3b0f431a345e395394ab8bdefb079fd1a31737f7746f6c7963`。这个本地 image ID **不是**上传后供他人拉取的仓库 digest，不能填入公开的 `FROM ...@sha256:`。正式仓库地址、版本标签和 digest 须在上传并从仓库重新拉取验证后补充。

增量 Dockerfile 要求基础镜像提供：

- `/ros1_ws/install_isolated/setup.bash`：在 Ubuntu 22.04 上构建的 ROS1 Noetic 环境；
- `/opt/ros/humble/setup.bash`：ROS2 Humble 环境；
- `/bridge_ws/src/ros1_bridge`：本机基础镜像内的提交已只读核对为 `611755fd917285316051cbea80507e8b2f6b7ec1`；Dockerfile 在构建中再次检查该值；
- `catkin_make`、`colcon`、C++ 编译器及构建 `upperlimb`、`tf2_msgs`、`ros1_bridge` 所需依赖。

目前只验证了 `linux/amd64`，没有 ARM64 基础镜像。历史 [`Dockerfile.humble`](archive/base_image/Dockerfile.humble) 仅作为开发记录保存；它未在全新机器上验证，且含旧路径与未锁定的上游版本，**不是当前冻结镜像的可复现配方**。

## 本机增量构建

当前机器保留本地基础镜像时，在 `~/WA1` 执行：

```bash
python3 WA1_ws/native_bridge/bridge.py build
python3 WA1_ws/native_bridge/tests/run_mock_test.py --seconds 3
```

Docker 的构建上下文是 `WA1_ws` 根目录，因为 [`src/upperlimb`](../../src/upperlimb) 的 ROS2 消息和服务定义也要复制到镜像里。`native_bridge/ros1_interfaces/` 只维护 ROS1 包构建元数据和额外的 TF 消息定义；不要复制出第二份 WA1 消息字段。

## 发布基础镜像时补全

1. 给基础镜像一个明确的仓库名称和版本标签，例如 `ghcr.io/<用户名>/wa1-ros1-humble-base:<版本>`；记录 `linux/amd64` 平台。
2. 上传后取得**仓库 manifest digest**，记录完整引用 `仓库名@sha256:<digest>`；同时保存本机镜像 ID 供核对，两者不要混用。
3. 从仓库按 digest 拉取，在与本机旧标签无关的环境中执行增量构建和 PC 模拟测试。例如：

   ```bash
   docker pull 'ghcr.io/<用户名>/wa1-ros1-humble-base@sha256:<仓库digest>'
   python3 WA1_ws/native_bridge/bridge.py build \
     --base-image 'ghcr.io/<用户名>/wa1-ros1-humble-base@sha256:<仓库digest>'
   python3 WA1_ws/native_bridge/tests/run_mock_test.py --seconds 3
   ```

4. 验证通过后，把正式 digest 设为 Dockerfile 和 `bridge.py` 的默认基础镜像，并在本文、根 README 中替换占位信息。`latest` 只用于当前本机过渡，不能作为公开 V1 的固定依赖。

这些步骤不需要修改 Jetson 或机器人 Pico，也不会发送关节控制指令。镜像仓库尚未选定、digest 尚未产生，因此本节的占位命令现在不能直接运行。
