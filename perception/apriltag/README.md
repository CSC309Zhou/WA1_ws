# 通过桥接获取 AprilTag 位姿

本目录保留 PC 端通过 `native_bridge` 订阅 AprilTag Tag36h11 / ID 0 位姿的代码与说明。运行前需要机器人端检测服务已发布 ROS1 位姿与状态；桥将这些结果转发到 PC ROS2。

## 文件

- [`test/pc_live_probe.py`](test/pc_live_probe.py)：只订阅 ROS2，统计位姿频率、配对时间戳并校验相机系到 BASE 系的变换。
- [`config/calibration_vendor_run01.json`](config/calibration_vendor_run01.json)：上述脚本所需的参考标定与固定关节位置，仅用于校验接收到的数据。机器人端标定改变时需同步更新。
- 桥接转发实现在 [`upperlimb_bridge.cpp`](../../native_bridge/src/wa1_native_bridge/src/upperlimb_bridge.cpp)。

## 启动桥接

以下命令均在 PC 的 `WA1_ws` 根目录执行。首次使用或桥代码更新后构建镜像：

```bash
python3 native_bridge/bridge.py build
python3 native_bridge/bridge.py start
python3 native_bridge/bridge.py logs
```

构建依赖见 [`native_bridge` 说明](../../native_bridge/README.md)；网络默认值、各主机角色及地址覆盖参数统一见[桥参数说明](../../native_bridge/README.md#启动查看与关闭)。

## 读取位姿

在查询终端加载 ROS2 环境：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
ros2 topic echo --once --qos-reliability best_effort \
  /perception/apriltag/tag0/status std_msgs/msg/String --full-length
ros2 topic echo --once --qos-reliability best_effort \
  /perception/apriltag/tag0/detected std_msgs/msg/Bool
ros2 topic echo --once --qos-reliability best_effort \
  /perception/apriltag/tag0/base_valid std_msgs/msg/Bool
ros2 topic echo --once --qos-reliability best_effort \
  /perception/apriltag/tag0/pose_camera geometry_msgs/msg/PoseStamped
ros2 topic echo --once --qos-reliability best_effort \
  /perception/apriltag/tag0/pose_base geometry_msgs/msg/PoseStamped
```

| 机器人 ROS1 | PC ROS2 | ROS2 类型 |
|---|---|---|
| `/csc309/apriltag/tag0/pose_camera` | `/perception/apriltag/tag0/pose_camera` | `geometry_msgs/msg/PoseStamped` |
| `/csc309/apriltag/tag0/pose_base` | `/perception/apriltag/tag0/pose_base` | `geometry_msgs/msg/PoseStamped` |
| `/csc309/apriltag/tag0/detected` | `/perception/apriltag/tag0/detected` | `std_msgs/msg/Bool` |
| `/csc309/apriltag/tag0/base_valid` | `/perception/apriltag/tag0/base_valid` | `std_msgs/msg/Bool` |
| `/csc309/apriltag/tag0/status` | `/perception/apriltag/tag0/status` | `std_msgs/msg/String`（JSON） |

`PoseStamped.pose.position` 的单位为米，`orientation` 是四元数 `x, y, z, w`。位姿表示标签坐标系相对 `header.frame_id` 的变换，时间戳沿用原始图像。

- `pose_camera`：相对 `realsense_head_color_optical_frame`，x 向图像右、y 向图像下、z 向镜头前方。
- `pose_base`：相对上肢 API 的 `wa1_upperlimb_base`，仅在固定标定姿态有效时发布。
- `detected` / `base_valid`：当前帧是否检测到标签 / 是否发布 BASE 位姿。
- `status`：约 1 Hz，包含 `camera_info_ok`、`frame_age_ms`、`fixed_joint_reason` 等诊断字段。

标签离开画面时位姿停止更新；颈部、腰部、升降偏离标定位置时 BASE 位姿停止更新。接收端应检查消息是否持续更新及有效状态，避免使用上一次接收到的旧位姿。

## Python 订阅与校验

加载上述 ROS2 环境后运行：

```bash
python3 perception/apriltag/test/pc_live_probe.py --seconds 6
```

脚本订阅五个感知话题及 `/upperlimb/joint_states`，输出 JSON：两路位姿数量与频率、按图像时间戳匹配的数量、检测与 BASE 有效比例、坐标变换误差、固定关节偏差及最新状态。

退出码 `0` 表示至少匹配 10 对位姿、相机参数检查通过、固定关节无暂停原因，且最大位置变换误差不超过 `1e-5 m`、最大姿态变换误差不超过 `1e-4 rad`；否则返回 `1`。该校验依赖标签持续可见且机器人满足标定姿态，验证的是接收与坐标变换一致性。

未收到位姿时先检查 `status` 和 `detected`；只有 BASE 位姿缺失时，检查 `base_valid`、`fixed_joint_reason` 及关节偏差。`--once` 在没有消息时会等待，可用 `Ctrl+C` 退出。

停止 PC 桥接：

```bash
python3 native_bridge/bridge.py stop
```
