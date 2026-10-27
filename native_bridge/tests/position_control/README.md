# WA1 单关节绝对位置 MoveJ 测试

本目录有 19 个 `manual_<joint>.py` 入口，分别对应双臂各 7 轴、颈部 2 轴、腰部 2 轴和升降 1 轴。每次只选中一个关节；发送给所属 MoveJ 服务的是该部位的完整关节数组，其余轴采用刚收到的同名关节位置。不会调用 `whole_body`，也不会自动返回、重试、解锁或使能。

先在 PC 端加载 ROS2 环境并预览，例如：

```bash
source WA1_ws/native_bridge/setup_pc.bash
python3 WA1_ws/native_bridge/tests/position_control/manual_neck_y.py --target 0.05
```

除已单独实测的 `manual_shoulder_y_l.py` 仍保留默认目标 `0.1 rad` 外，其余 18 个入口都必须显式给出 `--target`。`--target` 是绝对位置：旋转轴用 rad，`Lifting_Z` 用 m。仅增加 `--execute` 才发送一次真实 MoveJ；运行前应检查预览中的 `current`、`target`、`delta`、部位和机器人的实际空间。脚本退出后不保存起点。

| 部位 | 入口文件 | MoveJ 服务 | `arm_type` |
|---|---|---|---:|
| 左臂 | `manual_shoulder_y_l.py`, `manual_shoulder_x_l.py`, `manual_shoulder_z_l.py`, `manual_elbow_l.py`, `manual_wrist_z_l.py`, `manual_wrist_y_l.py`, `manual_wrist_x_l.py` | `/upperlimb/movej/left_arm` | 1 |
| 右臂 | `manual_shoulder_y_r.py`, `manual_shoulder_x_r.py`, `manual_shoulder_z_r.py`, `manual_elbow_r.py`, `manual_wrist_z_r.py`, `manual_wrist_y_r.py`, `manual_wrist_x_r.py` | `/upperlimb/movej/right_arm` | 2 |
| 颈部 | `manual_neck_z.py`, `manual_neck_y.py` | `/upperlimb/movej/neck` | 4 |
| 腰部 | `manual_waist_z.py`, `manual_waist_y.py` | `/upperlimb/movej/waist` | 8 |
| 升降 | `manual_lifting_z.py` | `/upperlimb/movej/lifting` | 16 |

新增入口共用 `movej_single_joint.py`，默认 `v=0.2`、`acc=0.2`、`t=0`；升降轴使用 `v=0.02`、`acc=0.02`、`t=0`，到位容差为 `0.002 m`，旋转轴为 `0.02 rad`。这些是脚本参数，不代表已验证的真机速度、碰撞或有效限位。目标范围采用所附 WA1 URDF 和遥操作教程示例范围的交集；两份资料对个别关节并不一致，现场控制器的实际范围仍需单独确认。所有新增执行分支目前只完成离线检查，未在真机上发送关节指令。
