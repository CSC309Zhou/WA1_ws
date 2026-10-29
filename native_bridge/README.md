# WA1 上肢与灵巧手 ROS1 ↔ ROS2 Humble 桥

## 目的与范围

`native_bridge` 让 PC 上的 ROS2 Humble 节点直接使用 WA1 原有的 ROS1 上肢与灵巧手接口。桥运行在 PC 的 Docker 容器中，连接机器人 ROS1 Master、上肢控制节点和手部节点，并在 PC 上提供 ROS2 状态话题、控制话题和服务。

桥覆盖左臂 7 轴、右臂 7 轴、颈部 2 轴、腰部 2 轴和升降 1 轴，共 19 轴；同时转发双臂 TCP 位姿、速度和上肢 TF。灵巧手另有 12 轴反馈、左右压力反馈以及关节、手势和压力标零服务。镜像、桥节点和开发测试均位于 PC。

## 数据链路与工作原理

```mermaid
flowchart LR
  APP[PC ROS2 Humble 节点] <-->|本机 DDS| BRIDGE[PC Docker：Noetic + Humble + WA1 C++ 桥]
  BRIDGE -.->|ROS1 注册与发现| MASTER[Jetson ROS1 Master]
  BRIDGE <-->|有线 TCPROS 与服务| PICO[机器人 Pico 上肢与灵巧手节点]
```

1. 容器内的 C++ 节点从 ROS1 接收上肢与手部关节状态、手部压力、TCP 状态与 TF，转换消息类型后发布到 PC 的 ROS2 网络。上肢 TF 按机器人原有发布节点筛选。
2. PC ROS2 节点通过桥发布 ServoJ、ServoL 等流指令，或调用 MoveJ、MoveL、FK、IK 等服务；手部通过 HandJoint 和 Gesture 服务发送一次性目标。桥转换请求并转发至对应的 ROS1 端点。
3. 状态话题使用 BestEffort、KeepLast(1)；下行流命令使用 Reliable、KeepLast(1)。桥保留机器人消息时间戳，并发布链路状态 `/upperlimb/bridge/status`。
4. 流命令检查维度、数值、命令时间、反馈新鲜度和占用状态。已发送的流在 150 ms 内没有新有效指令时，桥设置 `stream_fault` 并调用上肢停止服务。`/upperlimb/bridge/reset_fault` 用于清除桥内的流故障锁存。

当前现场有线配置为：PC `192.168.217.10`，ROS1 Master `192.168.217.1:11311`，机器人 Pico `192.168.217.66`。容器使用 host 网络与 IPC；PC 上的 ROS2 节点通过本机 DDS 与桥通信。

## 源码与镜像

| 路径 | 作用 |
|---|---|
| [bridge.py](bridge.py) | 构建镜像、启动和关闭容器、查看日志。 |
| [Dockerfile](Dockerfile) | 在 Noetic + Humble 基础镜像上编译 ROS1/ROS2 接口、`ros1_bridge` 转换库和专用 C++ 桥。 |
| [src/wa1_native_bridge](src/wa1_native_bridge/) | ROS1 ↔ ROS2 上肢与手部桥节点实现。 |
| [ros1_interfaces](ros1_interfaces/README.md) | ROS1 `upperlimb`、`hand` 包元数据与 TF 消息定义。 |
| [entrypoint.sh](entrypoint.sh) | 在容器内按顺序加载 ROS1、Humble、接口和桥工作区。 |
| [patch_transport.py](patch_transport.py) | 在镜像构建时准备 `ros1_bridge` 的消息转换与传输实现。 |
| [config/fastdds_pc.xml](config/fastdds_pc.xml) | PC 侧 Fast DDS 配置。 |
| [setup_pc.bash](setup_pc.bash) | 为 PC 上的 ROS2 客户端加载工作区接口与 DDS 配置。 |
| [tests](tests/) | PC 模拟回归与各控制接口的验收脚本。 |

ROS2 消息和服务定义来自工作空间的 [`src/upperlimb`](../src/upperlimb) 与 [`src/hand`](../src/hand)。`native_bridge/COLCON_IGNORE` 将双 ROS 构建留在容器内。默认基础镜像固定到私有 GHCR 的 `v1.0.0-amd64` 版本 digest；增量构建产物为 `wa1-upperlimb-bridge:humble`。镜像地址与内容见 [基础镜像说明](docs/BASE_IMAGE.md)。

## 编译

以下命令均在 `WA1_ws` 根目录运行。先加载 Humble，编译 PC 使用的 ROS2 接口包：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select upperlimb hand
```

GHCR 登录使用有镜像读取权限的 GitHub 账号；在 `Password:` 提示输入具有 `read:packages` 权限的 PAT classic，具体流程见[工作空间 README](../README.md)。下面的 Docker 登录与桥镜像构建均无需 ROS2 `source`：

```bash
docker login ghcr.io
python3 native_bridge/bridge.py build
```

`build --base-image 镜像引用` 可选择另一份基础镜像；也可设置 `WA1_BASE_IMAGE` 环境变量。Docker 构建上下文为 `WA1_ws` 根目录，镜像内的 ROS1 和 ROS2 接口共用 `src/upperlimb` 的消息字段。

## 有线网络检查

启动桥前，在运行桥的 PC 上检查实际有线地址和路由：

```bash
ip -br address
ip route get 192.168.217.1
ip route get 192.168.217.66
```

路由输出中的 `dev` 应是连接机器人的有线网卡，`src` 应是机器人能访问的 PC 有线地址。`bridge.py start` 默认将 `ROS_IP` 设为 `192.168.217.10`；如果这台 PC 的实际地址不同，启动时用 `--pc-ip` 指定实际地址。`ROS_MASTER_URI` 默认是 `http://192.168.217.1:11311`，机器人 Pico 的默认地址为 `192.168.217.66`。PICO4 的 Wi-Fi 数据正常不代表这条有线链路正常。

## 启动、查看与关闭

以下 Docker 管理命令无需加载 ROS2 环境：

```bash
python3 native_bridge/bridge.py start
python3 native_bridge/bridge.py logs
python3 native_bridge/pc_docker.py ps -a --filter name=wa1-upperlimb-bridge
python3 native_bridge/bridge.py stop
```

`start` 运行名为 `wa1-upperlimb-bridge` 的容器，开放上肢与手部状态和全部桥接控制端点。相同配置的容器已运行时，`start` 直接返回，不会重建连接；`stop` 停止并移除该容器，镜像保留在本机。机器人重启或需要改变 `--pc-ip` 等容器参数后，先执行 `stop`，再执行 `start`。

| 参数 | 默认值与用途 |
|---|---|
| `--pc-ip` | `192.168.217.10`；PC 有线地址，对应 `WA1_PC_IP`。 |
| `--master-uri` | `http://192.168.217.1:11311`；ROS1 Master，对应 `WA1_ROS1_MASTER_URI`。 |
| `--pico-ip` | `192.168.217.66`；机器人 Pico 主机别名，对应 `WA1_PICO_IP`。 |
| `--jetson-ip` | `192.168.217.1`；Jetson 有线主机别名，对应 `WA1_JETSON_IP`。 |
| `--domain` | `0`；桥和 PC 客户端使用相同的 ROS2 domain。 |
| `--latency-probe` | 发布左臂 ServoJ 的 PC 侧时序诊断。 |
| `--mock` | 使用本机 ROS1 Master `127.0.0.1:11321` 和 DDS domain 83。 |

`pc_docker.py` 在当前用户可访问 Docker socket 时直接调用 Docker；现有 `docker` 组尚未在 shell 生效时使用 `sg docker` 执行命令。

## 查看 ROS2 数据

在每个运行 ROS2 CLI 或 Python 客户端的新终端，先设真机 domain 0，再加载工作空间及 PC DDS 环境。`setup_pc.bash` 保留事先设置的 `ROS_DOMAIN_ID`；在已启动的进程之后执行 `source` 不会改变该进程的环境。

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
ros2 topic list -t
ros2 service list -t
ros2 topic echo --once /upperlimb/bridge/status std_msgs/msg/String
ros2 topic echo --once --qos-reliability best_effort /upperlimb/joint_states sensor_msgs/msg/JointState
ros2 topic echo --once --qos-reliability best_effort /upperlimb/tcp_pose/left upperlimb/msg/Pose
ros2 topic echo --once --qos-reliability best_effort /hand/joint_states sensor_msgs/msg/JointState
ros2 topic echo --once --qos-reliability best_effort /hand/finger_pressures/left hand/msg/PressureSensor
ros2 topic echo --once /tf tf2_msgs/msg/TFMessage
ros2 topic echo --once /tf_static tf2_msgs/msg/TFMessage --qos-durability transient_local
```

`/upperlimb/bridge/status` 以 JSON 提供 `feedback_ready`、`stream_fault`、`state_count`、`sent`、`dropped` 和 ROS1 命令连接数。19 轴状态约以 200 Hz 更新；旋转关节以弧度表示，`Lifting_Z` 以米表示。PC 客户端按 `JointState.name` 查找关节。TCP 位姿使用位置与四元数；位置坐标基于机器人 BASE。

`setup_pc.bash` 加载工作区消息定义，设置 Fast DDS 回环传输和 `rmw_fastrtps_cpp`。PC 客户端的 domain 与桥启动时的 `--domain` 相同；`topic list` 只查看话题发现信息，`topic echo` 才在当前终端建立订阅并接收消息。高频状态话题采用 BestEffort，因此上面的 `echo` 显式选择相同可靠性。切换 ROS2 环境后需要即时重新发现话题时，可加 `--no-daemon`。

## 故障排查：桥在运行，但本机看不到话题

`bridge.py logs` 中的 `Passing message ... (showing msg only once per type)` 只在首次转换该消息类型时打印，不能用它判断此刻是否仍有数据。先在查询话题的**同一个终端**设置真机 domain 并加载 PC 环境：

```bash
cd ~/WA1/WA1_ws
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
ros2 topic list --no-daemon -t
ros2 topic list -t
```

`setup_pc.bash` 保留已有的 `ROS_DOMAIN_ID`，所以先前为 PICO4 回放设置的 domain 85 不会因 `source` 自动变回 0。`--no-daemon` 让本次命令按当前终端环境直接发现话题；普通 `topic list` 使用已经运行的 ROS2 daemon。daemon 是独立进程，之后在终端执行 `source` 不会更新它的环境或发现结果。

如果 `--no-daemon` 能看到 `/upperlimb/joint_states`，普通列表却看不到，重启**本机 ROS2 daemon** 后复查：

```bash
ros2 daemon stop
ros2 topic list -t
ros2 topic echo --no-daemon --once --qos-reliability best_effort \
  /upperlimb/joint_states sensor_msgs/msg/JointState --field name
```

这次现场故障正是这种情况：`--no-daemon` 能发现约 30 个上肢与手部话题并收到关节状态，普通列表只显示系统话题；停止 daemon 后普通列表恢复。旧 daemon 为何失去发现结果尚未确定。

如果两个列表都看不到桥的话题，继续检查桥容器配置、PC 有线地址与机器人 ROS1 Master 路由：

```bash
python3 native_bridge/pc_docker.py ps -a --filter name=wa1-upperlimb-bridge
python3 native_bridge/bridge.py logs
ip route get 192.168.217.1
```

桥默认使用 domain 0、PC 有线地址 `192.168.217.10` 和 ROS1 Master `192.168.217.1:11311`；这些值应与当前网络和查询终端一致。话题出现在列表中仅表示发现成功，是否持续传输仍需用 `topic echo` 验证。

## 主要接口

| 类别 | PC ROS2 端点 | 类型与内容 |
|---|---|---|
| 关节反馈 | `/upperlimb/joint_states` | `sensor_msgs/msg/JointState`，19 轴位置、速度、力矩。 |
| TCP 反馈 | `/upperlimb/tcp_pose/{left,right}`、`/upperlimb/tcp_speed/dual` | `upperlimb/msg/Pose`、`upperlimb/msg/TcpSpeed`。 |
| 状态与 TF | `/upperlimb/uplimb_state`、`/upperlimb/occupancy`、`/tf`、`/tf_static` | 上肢模式、占用状态和上肢坐标变换。 |
| 安全锁查询 | `/upperlimb/safety_lock` | `std_srvs/srv/Trigger`；`success=true` 表示锁定，`false` 表示未锁定。只读。 |
| 解除机器人保护 | `/upperlimb/unlock` | `std_srvs/srv/Trigger`；显式请求机器人解锁。 |
| 关节位置 | `/upperlimb/movej/{left_arm,right_arm,dual_arm,neck,waist,lifting,whole_body}` | `upperlimb/srv/MoveJ`。 |
| 关节伺服 | `/cmd/upperlimb/servoj/{left_arm,right_arm,dual_arm,neck,waist,lifting,whole_body}` | `upperlimb/msg/Joints`。 |
| TCP 位置 | `/upperlimb/movel/{left_arm,right_arm,dual_arm}` | `upperlimb/srv/MoveL`，使用 `Pose[2]`。 |
| TCP 伺服 | `/cmd/upperlimb/servol/{left_arm,right_arm,dual_arm}` | `upperlimb/msg/DualPose`。 |
| 运动学 | `/upperlimb/FK/{left_arm,right_arm}`、`/upperlimb/IK/{left_arm,right_arm}` | `upperlimb/srv/FK`、`upperlimb/srv/IK`；FK 使用完整 19 轴数组。 |

接口还包含 SpeedJ、SpeedL、路径控制、伺服参数服务和 `is_singular` 查询。完整字段可在 PC 执行 `ros2 interface show upperlimb/srv/MoveJ`、`ros2 interface show upperlimb/srv/MoveL` 等命令查看。

可使用 [`wa1_robot_state`](../src/wa1_robot_state/README.md) 的 `wa1_state status`、`wa1_state lock` 查询上肢保护状态与安全锁，或在需要时显式运行 `wa1_state unlock`。机器人安全锁与桥的 `stream_fault` 是两个独立状态。

## 灵巧手接口

| 类别 | PC ROS2 端点 | 类型 |
|---|---|---|
| 12 轴反馈 | `/hand/joint_states` | `sensor_msgs/msg/JointState`；左手六轴后接右手六轴，位置单位弧度。 |
| 指尖压力 | `/hand/finger_pressures/{left,right}` | `hand/msg/PressureSensor`；各六个原始值。 |
| 关节位置 | `/hand/joint_switch/{left,right,dual}` | `hand/srv/HandJoint`；请求 `float32[] q`，返回 `success/message`。 |
| 手势 | `/hand/gesture_switch/{left,right,dual}` | `hand/srv/Gesture`；请求 `string[] gesture_name`。 |
| 压力标零 | `/hand/finger_pressures/{left,right}/zero` | `std_srvs/srv/Trigger`。 |

手关节和左右压力在真机约以 200 Hz 发布。`joint_states.velocity` 当前为空；`effort` 的物理含义尚未从官方接口确认。教程与机器人安装的消息注释对六个压力通道的顺序描述不同，桥保留原始数组，不重排或命名。手部服务响应表示调用结果；是否到达目标位置应查看后续关节反馈。当前 PICO 关闭腕部六维力传感器，因此本版未桥接腕力话题和标零服务。

手部桥与上肢桥在同一个容器、同一个启动入口中运行。上肢流命令看门狗只管理上肢流命令，不会因手部服务调用而启动。

## 测试入口与验收状态

```bash
python3 native_bridge/tests/run_mock_test.py --seconds 3
```

该脚本自动运行 PC 回环 ROS1 模拟器和桥，测试 200 Hz 状态流、消息转换、`safety_lock` 只读查询、命令端点、服务以及流看门狗，结果保存在 `native_bridge/results/`。`tests/mock_ros1.py` 和 `tests/ros2_probe.py` 由这条入口自动调用；入口脚本为其 ROS2 子进程加载 domain 83 和运行环境，宿主终端无需手动 `source`。

灵巧手真机只读检查与单手位置控制入口见 [手部测试说明](tests/hand/README.md)。机器人上肢状态和安全锁查询见 [`wa1_robot_state`](../src/wa1_robot_state/README.md)。

真机测试先在终端 A 启动桥：

```bash
python3 native_bridge/bridge.py start
```

终端 B 加载 PC ROS2 环境：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
```

在终端 B 选择一类测试，先运行下面的预览命令；在同一条命令末尾加 `--execute` 即执行对应控制请求：

| 测试 | 预览命令 | 输出 |
|---|---|---|
| [单关节 MoveJ](tests/position_control/README.md) | `python3 native_bridge/tests/position_control/manual_shoulder_y_l.py --target 0.1` | 终端打印当前与目标关节位置；`--target` 是绝对位置。 |
| [单关节 ServoJ](tests/position_servo/README.md) | `python3 native_bridge/tests/position_servo/manual_wrist_z_l.py --duration 1 --delta 0.01` | `tests/position_servo/results/<关节名>/` 中保存轨迹 JSON 和 PNG；`--delta` 是相对位移。 |
| [左臂 MoveL](tests/cartesian_position/README.md) | `python3 native_bridge/tests/cartesian_position/manual_left_arm_movel.py --delta 0.005 0 0 0 0 0` | 终端打印 BASE 下的六维 TCP 目标与执行结果。 |
| [左臂 ServoL](tests/cartesian_servo/README.md) | `python3 native_bridge/tests/cartesian_servo/manual_left_arm_servol.py --duration 1 --delta 0.005 0 0 0 0 0` | `tests/cartesian_servo/results/Left_TCP/` 中保存轨迹 JSON 和 PNG。 |

MoveL/ServoL 的六维增量依次为 `dx dy dz droll dpitch dyaw`，位置用米，角度用弧度。右臂测试使用同目录的 `manual_right_arm_movel.py`、`manual_right_arm_servol.py`；其他关节使用对应的 `manual_<joint>.py`。各目录 README 列出全部入口、参数和结果路径。`tests/manual_single_joint_movej.py` 是早期通用 MoveJ 入口；左腕延迟测量见 [延迟测试说明](tests/position_servo/latency/README.md)。

第一版已完成 19 轴 MoveJ 和 ServoJ、左右臂 MoveL 和 ServoL 的真机测试，并完成左臂 FK/IK 查询。历史验收数据位于 [evidence](evidence/)。
