# WA1 PICO4 双臂遥操作

## 目的与范围

`wa1_pico4_teleop` 在 PC 的 ROS2 Humble 工作空间中，将 PICO4 双手柄的相对位姿映射为 WA1 左右臂 TCP 目标，并通过 `native_bridge` 调用机器人原有的 MoveJ 与 ServoL 接口。控制范围为左右臂各 7 轴。PICO4 是 VR 头显；机器人内部负责嵌入式设备的 Pico 是另一台设备。

## 数据链路与工作原理

```mermaid
flowchart LR
  XR["PICO4 双手柄"] -->|WiFi| ADAPTER["PC pico4_ros2"]
  ADAPTER -->|XR 帧| TELEOP["PC wa1_pico4_teleop"]
  TELEOP -->|MoveJ 与 ServoL| BRIDGE["PC native_bridge"]
  BRIDGE -->|ROS1| PICO["WA1 Pico 上肢控制节点"]
  BRIDGE -->|关节、TCP 与模式反馈| TELEOP
```

1. `pico4_ros2` 从官方 PC Service 接收 PICO4 数据，发布 `/pico4/frame`。本包读取左右手柄的七维位姿、A/B 键、握键与前扳机；头显位姿不参与双臂目标计算。
2. 握键接管时记录手柄与机器人 TCP 的参考位姿，之后用手柄相对位姿计算 TCP 目标。左右握键各自控制对应手臂；两侧共用 `/cmd/upperlimb/servol/dual_arm`，未接管的一侧保持已记录的 TCP 位姿。
3. 节点从桥接话题接收关节、TCP、上肢模式、占用和桥状态；启动及 B 键返回使用 `/upperlimb/movej/dual_arm`。目标预览、遥操作状态和扳机值另行发布为 PC ROS2 话题。

## 源码与配置

| 路径 | 作用 |
|---|---|
| [src/wa1_teleop.cpp](src/wa1_teleop.cpp) | A/B 键、握键接管、初始位 MoveJ、ServoL、状态与恢复流程。 |
| [src/teleop_math.cpp](src/teleop_math.cpp)、[teleop_math.hpp](include/wa1_pico4_teleop/teleop_math.hpp) | 手柄位姿变换及平移、旋转跟踪限速。 |
| [config/wa1_teleop.yaml](config/wa1_teleop.yaml) | 14 轴初始位、比例、反馈时限和 MoveJ 参数。 |
| [launch/teleop.launch.py](launch/teleop.launch.py) | 正式双臂遥操作入口；无附加参数即启用完整流程。 |
| [scripts/monitor_status.py](scripts/monitor_status.py) | 只订阅 PICO4 与遥操作状态的终端监视工具。 |
| [scripts/preview_direction_check.py](scripts/preview_direction_check.py) | 对照手柄原始位移与 TCP 目标位移。 |
| [test/test_teleop_math.cpp](test/test_teleop_math.cpp) | 坐标映射、比例和跟踪限速的本机单元测试。 |
| [reference/](reference/) | 官方遥操作启动与配置文件的对照资料。 |

## 遥操作初始位置与按键

启动后，节点先保持 A 软件锁锁定。收到新鲜的 PICO4 帧且 A 已松开、机器人上肢处于 STOPPED、反馈与桥连接就绪时，节点自动发送一次双臂 MoveJ 到遥操作初始位置。解锁状态下长按右手 B 键 1 秒，机器人返回**同一位置**。

每侧关节顺序为 `Shoulder_Y, Shoulder_X, Shoulder_Z, Elbow, Wrist_Z, Wrist_Y, Wrist_X`，单位 rad：

| 左臂 | 右臂 |
|---|---|
| `[0.84, 0.49, 1.00, -1.96, 0.09, -0.08, 0.02]` | `[0.84, -0.49, -1.00, -1.96, -0.09, -0.08, -0.02]` |

启动和 B 返回使用 `v=0.3, acc=0.2, t=0, is_async=false, arm_type=3`。这些值保存在 [wa1_teleop.yaml](config/wa1_teleop.yaml)。

| PICO4 输入 | 遥操作动作 |
|---|---|
| 右手 A 主键 | 每次按下沿切换一次 PC 软件锁；松开时保持当前状态。初始位到达后按一次 A 解锁，再按一次上锁。 |
| 左右握键 | 握住一侧即从该侧当前手柄与 TCP 位姿建立参考并接管对应手臂；双侧可同时操作。松开后停止伺服并保持当前位置，再握住时重建参考。 |
| 右手 B 副键 | 解锁状态下长按至少 1 秒，结束伺服流并返回遥操作初始位置；一次长按触发一次。 |
| 左右前扳机 | 发布 `/wa1/teleop/trigger/left`、`/right` 的 `Float32` 原值，供末端执行器开发使用。 |

PICO4 的 `xrrobotics` 插件需打开 **Send**，并关闭 **Switch w/ A Button**，让 A 键只由 PC 遥操作节点处理。

## 位姿映射与跟踪速度

握键时的手柄与 TCP 位姿作为参考。手柄平移通过官方固定轴变换 `(x, y, z) → (-z, -x, +y)` 映射到机器人 BASE，再乘平移比例；旋转由手柄相对四元数映射。发布的 `/wa1/teleop/target/{left,right}` 是限速前的目标，`header.frame_id=BASE`。

`translation_scale` 默认 1.0，可在启动时设为 0.5～1.5；旋转保持 1:1。跟踪灵敏度固定为已验证的 4 档：最大平移速度 0.40 m/s、最大旋转速度 2.00 rad/s、最大平移加速度 1.6 m/s²、最大旋转加速度 8 rad/s²。ServoL 以 100 Hz 发布，参数为 `time=0.01 s, lookahead_time=0.2, gain=100`。

## 编译

以下命令在 `WA1_ws` 根目录执行，`colcon` 会一并构建本包依赖的 `upperlimb` 与 `pico4_ros2`：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to wa1_pico4_teleop
```

## 启动、查看与关闭

桥、PICO4 接收器和遥操作节点使用同一个 ROS2 domain。真机流程使用 domain 0；以下每个终端都从 `WA1_ws` 根目录运行。`source` 只影响执行它的终端，须在启动对应 ROS2 节点前完成。`native_bridge/setup_pc.bash` 已包含 Humble、工作空间和 PC DDS 配置，无需另行加载 `install/setup.bash`。

终端 A 先按[桥接说明](../../native_bridge/README.md)核对机器人有线地址和路由，再启动已构建的桥容器；Docker 管理命令无需 ROS2 `source`：

```bash
python3 native_bridge/bridge.py start
```

终端 B 加载真机 ROS2 环境，启动 PICO4 PC Service 与接收器；在 PICO4 插件中填写 PC 的 Wi-Fi IP、连接并开启 Send：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
ros2 launch pico4_ros2 pico4.launch.py
```

终端 C 独立加载相同环境，启动完整双臂遥操作：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
ros2 launch wa1_pico4_teleop teleop.launch.py
```

这条命令默认创建双臂 ServoL 发布者及 MoveJ、伺服配置、停止和桥故障复位客户端。满足前述条件后会自动发送一次初始位 MoveJ；到位后按 A 解锁，分别握住左右握键操作，长按右 B 返回初始位。平移比例可在启动时选择，例如 `ros2 launch wa1_pico4_teleop teleop.launch.py translation_scale:=0.8`。

终端 D 独立加载相同环境，查看状态：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
ros2 run wa1_pico4_teleop teleop_monitor
```

监视器显示 PICO4 帧龄、A/B 与握键状态、`initial_done`、`unlocked`、`control_block`、机器人模式和桥状态。只看位姿映射时，在已加载真机 ROS2 环境的终端 C 用 `ros2 launch wa1_pico4_teleop teleop.launch.py enable_control:=false` 启动预览模式；此模式不创建运动指令发布者或控制服务客户端。另一个已加载相同环境的终端可运行 `ros2 run wa1_pico4_teleop preview_direction_check --side left`，也可将 `left` 改为 `right`。

结束时先松开双握键，在终端 C、B 依次按 `Ctrl+C`，然后在终端 A 执行 `python3 native_bridge/bridge.py stop`。

## 主要 ROS2 接口

| 类别 | 端点 | 内容 |
|---|---|---|
| PICO4 输入 | `/pico4/frame` | `pico4_ros2/msg/XrFrame`，左右手柄位姿与按键。 |
| 机器人反馈 | `/upperlimb/joint_states`、`/upperlimb/tcp_pose/{left,right}`、`/upperlimb/occupancy`、`/upperlimb/uplimb_state`、`/upperlimb/bridge/status` | 当前关节、TCP、占用、模式和桥状态。 |
| 目标与状态 | `/wa1/teleop/target/{left,right}`、`/wa1/teleop/status` | BASE 下的原始 TCP 目标和 JSON 运行状态。 |
| 前扳机 | `/wa1/teleop/trigger/{left,right}` | 两侧扳机的 `std_msgs/msg/Float32` 原值。 |
| 双臂位置 | `/upperlimb/movej/dual_arm` | 初始化及 B 返回的 `upperlimb/srv/MoveJ`。 |
| 双臂伺服 | `/cmd/upperlimb/servol/dual_arm` | 左右 TCP 合成的 `upperlimb/msg/DualPose` 流。 |
| 配置与恢复 | `/upperlimb/{set_servo_params,clear_servo_params,stop}`、`/upperlimb/bridge/reset_fault` | 伺服参数、停止和桥流故障复位服务。 |

机器人进入 PROTECTED 后，遥操作状态转为 `FAULT` 并锁定。人工解除保护、上肢回到 STOPPED、双握键松开且监视器显示 `recovery_ready=1` 后，按一次 A 发起恢复；恢复完成时状态回到 `IDLE`，重新握键即可继续。

## 本机测试

位姿数学测试验证固定坐标映射、平移比例、旋转变换和 4 档速度与加速度限幅；在新终端运行时先加载 Humble：

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select wa1_pico4_teleop
colcon test-result --test-result-base build/wa1_pico4_teleop --verbose
```

此前的现场验证已覆盖初始位 MoveJ、A 锁、B 返回、左右及双臂握键遥操作、坐标映射和 4 档跟踪。本轮整理完成了本机编译与测试，尚未重新运行真机遥操作。
