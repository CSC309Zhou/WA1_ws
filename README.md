# WA1 PC ROS2 Humble 工作空间

本工作空间在 Ubuntu 22.04 `x86_64` PC 上运行。它包含 WA1 上肢的 ROS1↔ROS2 桥、独立的 PICO4 数据接入和 PICO4 双臂遥操作。桥的冻结基础镜像存放在私有 GHCR；克隆源码后在本机增量构建桥镜像。机器人 Jetson 和机器人 Pico 运行原有 ROS1 系统。

## 目录

| 路径 | 用途 |
|---|---|
| [`src/upperlimb`](src/upperlimb) | 桥、遥操作和其他 PC 节点共用的 WA1 ROS2 上肢消息与服务定义 |
| [`src/pico4_ros2`](src/pico4_ros2) | 与机器人无关的 PICO4 SDK 接入、消息和查看器 |
| [`src/wa1_pico4_teleop`](src/wa1_pico4_teleop) | PICO4 到 WA1 双臂的映射和操作状态机 |
| [`src/wa1_description`](src/wa1_description) | WA1 模型与可视化资源 |
| [`native_bridge`](native_bridge) | ROS1↔ROS2 原生桥、增量 Dockerfile、PC Docker 管理器和验收脚本 |

`native_bridge/COLCON_IGNORE` 让宿主机纯 Humble 构建只处理 `src/` 下的软件包；桥内的 C++ 包在同时具备 ROS1 和 ROS2 的容器里构建。历史基础镜像配方保存在 `native_bridge/docs/archive/`，正式构建使用 [`native_bridge/Dockerfile`](native_bridge/Dockerfile)。

## 获取源码与编译 PC 软件包

PC 需已安装 ROS2 Humble、`colcon`、`rosdep`、Git LFS 和 Docker，当前用户需能访问 Docker daemon。账号需有 `CSC309Zhou/WA1_ws` 私有仓库的读取权限。

```bash
git lfs install
git clone https://github.com/CSC309Zhou/WA1_ws.git
cd WA1_ws
git lfs pull
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/upperlimb src/pico4_ros2 src/wa1_pico4_teleop \
  --ignore-src --rosdistro humble -r -y
colcon build --packages-up-to wa1_pico4_teleop
source install/setup.bash
```

PICO4 PC Service 安装包由 Git LFS 保存；版本与 SHA-256 见 [`pico4_ros2` 说明](src/pico4_ros2/README.md)。`wa1_description` 可按需另行构建。

## 构建 PC 桥镜像

使用具有 `CSC309Zhou` 私有 GHCR 包读取权限的账号登录，在密码提示处输入具有 `read:packages` 权限的 GitHub PAT classic，然后在工作空间根目录构建：

```bash
docker login ghcr.io -u YOUR_GITHUB_USERNAME
python3 native_bridge/bridge.py build
```

`build` 默认从固定的 [`wa1-ros1-humble-base` 基础镜像](native_bridge/docs/BASE_IMAGE.md)增量生成 `wa1-upperlimb-bridge:humble`。下载权限、镜像版本和 digest 见基础镜像说明。Docker 调用包装器是 `native_bridge/pc_docker.py`。

## 运行与验证

桥接容器的启动、查看与关闭：

```bash
python3 native_bridge/bridge.py start
python3 native_bridge/bridge.py logs
python3 native_bridge/bridge.py stop
```

在另一个终端运行 PC ROS2 客户端前，先执行 `source native_bridge/setup_pc.bash`。机器人网络地址、桥接接口和测试入口见 [`native_bridge` 说明](native_bridge/README.md)。仅在 PC 上验证桥的消息转换与控制路径，可运行 `python3 native_bridge/tests/run_mock_test.py --seconds 3`。

PICO4 独立接入和浏览器可视化见 [`pico4_ros2` 说明](src/pico4_ros2/README.md)；完整双臂遥操作的启动顺序、按键与状态查看见 [`wa1_pico4_teleop` 说明](src/wa1_pico4_teleop/README.md)。本地实验版与目录调整的记录见 [代码整理计划](CODE_ORGANIZATION_PLAN.md)。
