# 双臂单独 ServoL 测试

`manual_left_arm_servol.py` 读取左臂 TCP 的位置和四元数，以 `BASE` 坐标系中的六维增量生成有限时长的位姿目标流。`--delta` 依次为 `dx dy dz droll dpitch dyaw`，前三项单位为 m，后三项单位为 rad。姿态使用当前四元数和目标旋转插值，不读取 TCP 消息的 `rpy_rad`。

先预览已确认空间中的小位移，例如 X 方向 +5 mm、持续 1 s：

```bash
source WA1_ws/native_bridge/setup_pc.bash
python3 WA1_ws/native_bridge/tests/cartesian_servo/manual_left_arm_servol.py \
  --duration 1 --delta 0.005 0 0 0 0 0
```

预览只订阅反馈并在 PC 上保存 `results/Left_TCP/preview.json` 和 `preview.png`。预览不创建指令发布器，不调用伺服参数、停止或桥故障复位服务。

由现场操作者确认目标后，加 `--execute` 才运行真机实验：

```bash
python3 WA1_ws/native_bridge/tests/cartesian_servo/manual_left_arm_servol.py \
  --duration 1 --delta 0.005 0 0 0 0 0 --execute
```

脚本向 ROS2 `/cmd/upperlimb/servol/left_arm` 发布 `upperlimb/msg/DualPose`：`left_arm_pose` 是插值后的目标，`right_arm_pose` 填实时右臂位姿。运行前调用 `/upperlimb/set_servo_params`，使用 `time=1/200 s`、`lookahead_time=0.2`、`gain=100`、`arm_type=1`；按 200 Hz 发布起点保持、平滑移动和终点保持；完成或异常时调用 `stop` 并清除伺服参数。有限流结束后，当前桥的看门狗可能锁存故障；下次执行仅在新鲜反馈确认 `STOPPED` 后复位，不自动解锁 `PROTECTED`。

实测覆盖 `results/Left_TCP/latest.json` 和 `latest.png`，记录发布目标、TCP/关节反馈、发布间隔、桥计数及终点误差。曲线用于查看 ServoL 跟随情况，不评价 MoveL 的直线轨迹。

## 右臂

`manual_right_arm_servol.py` 使用与左臂脚本相同的六维 `--delta`、`--duration` 和 200 Hz 插值。目标从 `/upperlimb/tcp_pose/right` 的位置与四元数计算，发布到 `/cmd/upperlimb/servol/right_arm` 的 `right_arm_pose`；`left_arm_pose` 填实时左臂位姿。`set_servo_params` 的 `arm_type` 为 2。

先预览右臂目标，例如 BASE 的 +X 方向 5 mm、持续 1 s：

```bash
python3 WA1_ws/native_bridge/tests/cartesian_servo/manual_right_arm_servol.py \
  --duration 1 --delta 0.005 0 0 0 0 0
```

由现场操作者确认目标后，才在同一命令末尾加 `--execute`。右臂预览保存到 `results/Right_TCP/preview.json`、`preview.png`；实测保存到 `results/Right_TCP/latest.json`、`latest.png`。脚本自动处理伺服参数、停止及下一次运行前的已锁存桥故障复位；`PROTECTED` 状态不会自动解锁。只有 `--execute` 会向机器人发布控制命令。
