# 双臂单独 ServoL 测试

本目录仅保留通用入口 [servol_single_arm.py](servol_single_arm.py)，必填 `--side left` 或 `--side right` 选择手臂。脚本读取所选手臂 TCP 的位置和四元数，以 `BASE` 坐标系中的六维增量生成有限时长的位姿目标流。`--delta` 依次为 `dx dy dz droll dpitch dyaw`，前三项单位为 m，后三项单位为 rad。姿态使用当前四元数和目标旋转插值，不读取 TCP 消息的 `rpy_rad`。

以下命令在工作空间根目录执行；先按[桥接说明](../../README.md#启动查看与关闭)启动桥，再预览已确认空间中的小位移，例如 X 方向 +5 mm、持续 1 s：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
python3 native_bridge/tests/cartesian_servo/servol_single_arm.py \
  --side left --duration 1 --delta 0.005 0 0 0 0 0
```

右臂使用同一入口和参数，将 `--side left` 改为 `--side right`。预览只订阅反馈并保存 JSON/PNG，不创建指令发布器，不调用伺服参数、停止或桥故障复位服务。由现场操作者确认目标后，只有追加 `--execute` 才发送控制：

```bash
python3 native_bridge/tests/cartesian_servo/servol_single_arm.py \
  --side right --duration 1 --delta 0.005 0 0 0 0 0 --execute
```

| `--side` | TCP 输入 | ROS2 发布话题 | 目标字段 | `arm_type` | 默认结果目录（相对本目录） |
|---|---|---|---|---:|---|
| left | `/upperlimb/tcp_pose/left` | `/cmd/upperlimb/servol/left_arm` | `left_arm_pose` | 1 | `results/Left_TCP/` |
| right | `/upperlimb/tcp_pose/right` | `/cmd/upperlimb/servol/right_arm` | `right_arm_pose` | 2 | `results/Right_TCP/` |

发布类型为 `upperlimb/msg/DualPose`；所选手臂字段填插值目标，另一臂字段填该臂实时位姿。运行前调用 `/upperlimb/set_servo_params`，使用 `time=1/200 s`、`lookahead_time=0.2`、`gain=100` 和上表的 `arm_type`；按 200 Hz 发布起点保持、平滑移动和终点保持。`--duration` 必须有限且严格大于 0.1 s，六维 `--delta` 必须有限且非零。

预览覆盖保存 `preview.json`、`preview.png`；执行覆盖保存 `latest.json`、`latest.png`。`--output-dir` 可覆盖上表的默认目录。结果记录发布目标、TCP/关节反馈、发布间隔、桥计数及终点误差。曲线用于查看 ServoL 跟随情况，不评价 MoveL 的直线轨迹；左右臂各自保留原有 JSON 字段名称。

已发布指令后完成或异常时调用 `stop`，已配置伺服参数后清除参数。有限流结束后，当前桥的看门狗可能锁存故障；下次执行仅在新鲜反馈确认 `STOPPED` 后复位，不自动解锁 `PROTECTED`。
