# 灵巧手接口检查

以下命令均在 `WA1_ws` 根目录运行。桥接容器需要连接机器人的有线 ROS1 网络。

## 只读检查

终端 A 启动桥：

```bash
python3 native_bridge/bridge.py start
```

终端 B 加载与桥相同的 ROS2 环境，检查双手 12 关节、左右各 6 个压力值和 8 个服务端点：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
python3 native_bridge/tests/hand/probe.py --seconds 3
```

`probe.py` 只订阅反馈并检查服务可发现性，不调用服务。也可单独查看消息：

```bash
ros2 topic echo --once --qos-reliability best_effort /hand/joint_states sensor_msgs/msg/JointState
ros2 topic echo --once --qos-reliability best_effort /hand/finger_pressures/left hand/msg/PressureSensor
ros2 topic echo --once --qos-reliability best_effort /hand/finger_pressures/right hand/msg/PressureSensor
ros2 service list -t
```

## 单关节位置测试

继续使用终端 B。以下命令预览左手食指的绝对目标角度：

```bash
python3 native_bridge/tests/hand/manual_single_joint.py --side left --joint INDEX_MCP --target 0.4
```

确认目标后，在命令末尾添加 `--execute`，脚本会读取该手另外五个关节的当前角度，只调用一次 `/hand/joint_switch/left`，并打印服务响应及之后的关节反馈。右手使用 `--side right`。`--target` 的单位是弧度，范围采用官方教程的关节参考范围。

手势和压力标零服务的接口列表见 [桥接说明](../../README.md#灵巧手接口)。
