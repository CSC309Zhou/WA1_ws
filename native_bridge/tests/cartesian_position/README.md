# 双臂笛卡尔 MoveL 测试

本目录仅保留通用入口 [movel_single_arm.py](movel_single_arm.py)，必填 `--side left` 或 `--side right` 选择手臂。

该入口接收所选手臂 TCP 的六维目标：`x, y, z` 是 `BASE` 下的位置（m），`roll, pitch, yaw` 是固定轴 XYZ 姿态角（rad）。脚本只读取反馈的 `position + quaternion`，不使用 TCP 消息中的 `rpy_rad`。原厂 MoveL 服务接收 `geometry_msgs/Pose[2]`，脚本把六维输入转换为三维位置和四元数。官方教程规定，**单独控制左臂或右臂时都把目标放在 `pose[0]`**；只有双臂控制时 `pose[0]` 是左臂、`pose[1]` 是右臂。

以下命令在工作空间根目录执行；先按[桥接说明](../../README.md#启动查看与关闭)启动桥，再只预览相对位移。例如让目标沿 BASE 的 +X 方向变化 5 mm：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
python3 native_bridge/tests/cartesian_position/movel_single_arm.py --side left \
  --delta 0.005 0 0 0 0 0
```

`--delta` 的六项依次是 `dx dy dz droll dpitch dyaw`。前三项直接加到当前 BASE 位置；后三项构造旋转四元数，并按 `q_target = q_delta ⊗ q_current` 计算目标。也可以输入绝对位姿：

```bash
python3 native_bridge/tests/cartesian_position/movel_single_arm.py --side left \
  --absolute X Y Z ROLL PITCH YAW
```

右臂采用同一组参数，例如先只预览沿 BASE 的 +X 方向移动 5 mm：

```bash
python3 native_bridge/tests/cartesian_position/movel_single_arm.py --side right \
  --delta 0.005 0 0 0 0 0
```

预览只订阅反馈，不创建 MoveL 客户端。添加 `--check-only` 可检查实时反馈、桥状态、STOPPED 状态以及对应 MoveL 服务是否可用，且不发送请求。现场操作者确认目标后，可自行添加 `--execute`，向对应的 `/upperlimb/movel/left_arm` 或 `/upperlimb/movel/right_arm` 发送**一次**同步请求；`--v`、`--acc` 默认分别是 `0.05`、`0.1`，可在命令行修改。`--check-only` 和 `--execute` 不能同时使用。脚本在发送前重新读取对应的 TCP，并重算相对目标；运动返回后显示 TCP 误差和该臂七个关节的实际变化。

左右臂均使用 `pose[0]`；固定数组的 `pose[1]` 填入另一臂的当前位姿。MoveL 的逆解可能同时改变同一臂的多个关节。
