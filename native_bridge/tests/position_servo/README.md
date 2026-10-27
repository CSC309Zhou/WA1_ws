# WA1 单关节 ServoJ 轨迹测试

`latency/` 中另有基于同一轨迹和收尾流程的 `Wrist_Z_L` 延迟探针，使用前先阅读 [延迟探针说明](latency/README.md)。

本目录有 19 个 `manual_<joint>.py` 入口，覆盖双臂各 7 轴、头部 2 轴、腰部 2 轴和升降 1 轴。它们共用 `servoj_single_joint.py`，每次只改变所选关节，所属部位的其余关节保持开始时的实测位置；不使用 `whole_body`。原有 `manual_wrist_z_l.py` 的参数和结果格式保持兼容。

| 部位 | 入口文件（均位于本目录） | ServoJ 话题 | `arm_type` |
| --- | --- | --- | ---: |
| 左臂 | `manual_shoulder_y_l.py`, `manual_shoulder_x_l.py`, `manual_shoulder_z_l.py`, `manual_elbow_l.py`, `manual_wrist_z_l.py`, `manual_wrist_y_l.py`, `manual_wrist_x_l.py` | `/cmd/upperlimb/servoj/left_arm` | 1 |
| 右臂 | `manual_shoulder_y_r.py`, `manual_shoulder_x_r.py`, `manual_shoulder_z_r.py`, `manual_elbow_r.py`, `manual_wrist_z_r.py`, `manual_wrist_y_r.py`, `manual_wrist_x_r.py` | `/cmd/upperlimb/servoj/right_arm` | 2 |
| 头部 | `manual_neck_z.py`, `manual_neck_y.py` | `/cmd/upperlimb/servoj/neck` | 4 |
| 腰部 | `manual_waist_z.py`, `manual_waist_y.py` | `/cmd/upperlimb/servoj/waist` | 8 |
| 升降 | `manual_lifting_z.py` | `/cmd/upperlimb/servoj/lifting` | 16 |

`--duration` 必须是有限且严格大于 0.1 秒的运动时长；`--delta` 是有限的有符号相对位移。旋转轴使用 rad、rad/s，规划匀速段的峰值速度上限是 5 rad/s；`Lifting_Z` 使用 m、m/s，峰值速度上限是 0.05 m/s。目标位置范围沿用所附 WA1 URDF 与遥操作教程示例范围的交集，不代表当前控制器真实保护阈值。所有入口按约 200 Hz 发布，运动前后各有 0.2/0.3 秒定点段，两端各占运动时长的 10% 用于加减速，中间 80% 匀速。理论峰值速度为 `|delta| / [duration × 0.9]`，理论峰值加速度为 `峰值速度 / [duration × 0.1]`；提高频率不会降低这两个理论值。

在 PC 的 WA1 根目录预览一个关节，例如：

```bash
source WA1_ws/native_bridge/setup_pc.bash
python3 WA1_ws/native_bridge/tests/position_servo/manual_neck_y.py \
  --duration 1 --delta 0.01
```

预览只读取真机反馈、规划轨迹并保存 JSON/PNG；不会创建 ServoJ 发布者，也不会调用设置伺服参数、stop 或故障复位服务。只有现场操作者显式追加 `--execute` 才会执行。执行模式要求 DDS domain 0、控制桥、空闲且新鲜的反馈、单一桥发布者和对应部位的 ROS1 ServoJ 接收者。脚本通过 `/upperlimb/set_servo_params` 设置 `time≈0.005 s`、`gain=100`、`lookahead_time=0.2` 和该部位的 `arm_type`。官方课程将此版本的 `v/acc` 标为无效，脚本传 0；这些值不限制真实速度或加速度。

每次运行将数据和图像覆盖写入 `results/<关节名>/latest.json` 与 `latest.png`；`--output-dir` 可改结果根目录。JSON 记录规划轨迹、实际发布时间、关节反馈和误差；旋转轴字段以 `_rad` 结尾，升降轴以 `_m` 结尾，速度/加速度分别以 `_rad_s`/`_rad_s2` 或 `_m_s`/`_m_s2` 结尾。PC 接收时间上的误差包含采样和传输延迟，不直接等于电机闭环误差。执行后脚本请求一次针对整个上肢的 `/upperlimb/stop` 并调用 `/upperlimb/clear_servo_params`；不会自动返回起点。

PC 桥在有限流的最后一条指令后约 150 ms 会锁存 `stream_fault` 并请求 stop。下一次 `--execute` 若发现锁存，会在确认新鲜反馈、`occupancy=0` 和上肢 `STOPPED` 后自动调用本地 `/upperlimb/bridge/reset_fault`。如果机器人仍处于 `PROTECTED`，此复位被拒绝，轨迹不会发布。桥复位只清除 PC 锁存，不解除机器人自身的安全保护；运行后可检查 JSON 中的 `pre_run_reset_performed`、`bridge_status_after`、`stop_called` 和 `servo_params_cleared`。

根据现场操作者反馈，19 个关节的位置伺服测试均已完成真机验证。新增的 `latency/` 探针仍需单独运行和解读；此前的运动验收不能替代延迟分解。
