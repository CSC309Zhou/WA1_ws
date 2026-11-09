# WA1 单关节绝对位置 MoveJ 测试

本目录仅保留通用入口 [movej_single_joint.py](movej_single_joint.py)，使用 `--joint-name` 选择 19 个上肢关节之一。每次只选中一个关节；发送给所属 MoveJ 服务的是该部位的完整关节数组，其余轴采用刚收到的同名关节位置。不会调用 `whole_body`，也不会自动返回、重试、解锁或使能。

以下命令在工作空间根目录执行；先按[桥接说明](../../README.md#启动查看与关闭)启动桥，再加载环境并预览：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
python3 native_bridge/tests/position_control/movej_single_joint.py --joint-name Neck_Y --target 0.05
```

所有关节均须显式给出 `--joint-name` 和 `--target`，左肩也不再使用特殊默认目标。`--target` 是绝对位置：旋转轴用 rad，`Lifting_Z` 用 m。仅增加 `--execute` 才发送一次真实 MoveJ；运行前应检查预览中的 `current`、`target`、`delta`、部位和机器人的实际空间。脚本退出后不保存起点。

| 部位 | `--joint-name` 可选值 | MoveJ 服务 | `arm_type` |
|---|---|---|---:|
| 左臂 | `Shoulder_Y_L, Shoulder_X_L, Shoulder_Z_L, Elbow_L, Wrist_Z_L, Wrist_Y_L, Wrist_X_L` | `/upperlimb/movej/left_arm` | 1 |
| 右臂 | `Shoulder_Y_R, Shoulder_X_R, Shoulder_Z_R, Elbow_R, Wrist_Z_R, Wrist_Y_R, Wrist_X_R` | `/upperlimb/movej/right_arm` | 2 |
| 颈部 | `Neck_Z, Neck_Y` | `/upperlimb/movej/neck` | 4 |
| 腰部 | `Waist_Z, Waist_Y` | `/upperlimb/movej/waist` | 8 |
| 升降 | `Lifting_Z` | `/upperlimb/movej/lifting` | 16 |

关节分组和保守测试范围由 [joint_config.py](../joint_config.py) 统一维护。默认 `v=0.2`、`acc=0.2`、`t=0`；升降轴使用 `v=0.02`、`acc=0.02`、`t=0`，到位容差为 `0.002 m`，旋转轴为 `0.02 rad`。这些是脚本参数，不代表已验证的真机速度、碰撞或有效限位。目标范围采用所附 WA1 URDF 和遥操作教程示例范围的交集；两份资料对个别关节并不一致，现场控制器的实际范围仍需单独确认。所有新增执行分支目前只完成离线检查，未在真机上发送关节指令。
