# WA1 PC ROS2 Humble 工作空间

本工作空间在 Ubuntu 22.04 `x86_64` PC 上运行。它包含 WA1 上肢与灵巧手的 ROS1↔ROS2 桥、独立的 PICO4 数据接入和 PICO4 双臂遥操作。桥的冻结基础镜像存放在私有 GHCR；克隆源码后在本机增量构建桥镜像。机器人 Jetson 和机器人 Pico 运行原有 ROS1 系统。

## 目录

| 路径 | 用途 |
|---|---|
| [`src/upperlimb`](src/upperlimb) | 桥、遥操作和其他 PC 节点共用的 WA1 ROS2 上肢消息与服务定义 |
| [`src/hand`](src/hand) | WA1 灵巧手 ROS2 压力消息、关节与手势服务定义 |
| [`src/wa1_robot_state`](src/wa1_robot_state) | WA1 上肢状态、安全锁查询与显式解锁命令 |
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
rosdep install --from-paths src/upperlimb src/hand src/wa1_robot_state src/pico4_ros2 src/wa1_pico4_teleop \
  --ignore-src --rosdistro humble -r -y
colcon build --packages-up-to wa1_pico4_teleop wa1_robot_state hand
```

PICO4 PC Service 安装包由 Git LFS 保存；版本与 SHA-256 见 [`pico4_ros2` 说明](src/pico4_ros2/README.md)。`wa1_description` 可按需另行构建。

## 命令运行前的环境

以下命令均在 `WA1_ws` 根目录执行。`source` 只影响当前终端；每开一个新终端，都要在运行命令前加载对应环境。已经启动的进程不会因后来执行 `source` 而改变环境。

| 要运行的命令 | 该终端需要先执行 |
|---|---|
| `git`、`docker login`、`native_bridge/pc_docker.py`、`native_bridge/bridge.py build/start/stop/logs` | 无需加载 ROS2 环境。 |
| `colcon build`、`colcon test` | `source /opt/ros/humble/setup.bash`。 |
| `native_bridge/tests/run_mock_test.py` | 无需手动加载 ROS2 环境；脚本会为其 ROS2 子进程设置模拟 domain 和运行环境。 |
| 真机 PICO4 接收、查看器、桥接话题查询、遥操作、监视器及手动验收脚本 | 先设 `ROS_DOMAIN_ID=0`，再加载 `native_bridge/setup_pc.bash`。 |
| PICO4 JSONL 回放及对应查看器 | 两个终端均先设 `ROS_DOMAIN_ID=85`，再加载 `native_bridge/setup_pc.bash`。 |

真机运行时，在**每个执行 `ros2` 命令或 ROS2 Python 测试脚本的终端**运行：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
```

这个脚本已包含 ROS2 Humble、工作空间接口和 PC Fast DDS 配置，无需再单独执行 `source install/setup.bash`。它保留事先设置的 `ROS_DOMAIN_ID`；真机与桥容器的默认 domain 都是 0。PICO4 回放使用独立的 domain 85。

## 构建 PC 桥镜像

首次构建时，先完成 GHCR 登录：

1. 用有权读取 `CSC309Zhou` 私有镜像包的 GitHub 账号，在 `Settings → Developer settings → Personal access tokens → Tokens (classic)` 创建 PAT classic；操作入口见 [GitHub 令牌文档](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-personal-access-token-classic)。
2. 为令牌选择 `read:packages` 权限；如组织启用 SSO，还需给该令牌授权。[GitHub Container registry 文档](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry) 列出了镜像下载权限。
3. 运行下列登录命令，在 `Username:` 提示输入实际 GitHub 用户名，在 `Password:` 提示粘贴 PAT。GitHub 账号密码不能用作这里的令牌。

```bash
docker login ghcr.io
python3 native_bridge/bridge.py build
```

`build` 默认从固定的 [`wa1-ros1-humble-base` 基础镜像](native_bridge/docs/BASE_IMAGE.md)增量生成 `wa1-upperlimb-bridge:humble`。下载权限、镜像版本和 digest 见基础镜像说明。Docker 调用包装器是 `native_bridge/pc_docker.py`。

## 运行与验证

启动桥前先核对有线连接。`ip route get` 的 `dev` 应为连接机器人的有线网卡，`src` 应与桥使用的 PC 地址一致；桥默认地址为 `192.168.217.10`。机器人 Jetson 和机器人 Pico 的默认有线地址分别为 `192.168.217.1` 和 `192.168.217.66`。

```bash
ip -br address
ip route get 192.168.217.1
ip route get 192.168.217.66
```

桥接容器的启动、查看与关闭无需 `source`：

```bash
python3 native_bridge/bridge.py start
python3 native_bridge/bridge.py logs
python3 native_bridge/bridge.py stop
```

在另一个终端按上面的真机运行环境命令加载 `native_bridge/setup_pc.bash`，再运行 ROS2 客户端。机器人网络地址、桥接接口和测试入口见 [`native_bridge` 说明](native_bridge/README.md)。仅在 PC 上验证桥的消息转换与控制路径，可直接运行 `python3 native_bridge/tests/run_mock_test.py --seconds 3`。

PICO4 独立接入和浏览器可视化见 [`pico4_ros2` 说明](src/pico4_ros2/README.md)；完整双臂遥操作的启动顺序、按键与状态查看见 [`wa1_pico4_teleop` 说明](src/wa1_pico4_teleop/README.md)。上肢保护模式与安全锁的查询、显式解锁见 [`wa1_robot_state` 说明](src/wa1_robot_state/README.md)。本地实验版与目录调整的记录见 [代码整理计划](CODE_ORGANIZATION_PLAN.md)。
