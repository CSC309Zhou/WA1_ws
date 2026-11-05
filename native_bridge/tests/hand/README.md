# 灵巧手接口检查

以下命令均在 `WA1_ws` 根目录运行。桥接容器需要连接机器人的有线 ROS1 网络。

## 只读检查

终端 A 启动桥：

```bash
python3 native_bridge/bridge.py start
```

终端 B 加载与桥相同的 ROS2 环境，检查双手 12 关节、左右各 6 个指尖力反馈值和 8 个服务端点：

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

力值单位、换算及通道顺序见 [指尖力反馈说明](../../../src/hand/README.md#指尖力反馈)。

## 指尖力空载标零

保持手指静止、指尖不接触物体，在已加载上述环境的终端调用：

```bash
ros2 service call /hand/finger_pressures/left/zero std_srvs/srv/Trigger '{}'
ros2 service call /hand/finger_pressures/right/zero std_srvs/srv/Trigger '{}'
```

检查返回的 `success` 和 `message`，再订阅反馈确认空载读数回到零附近。标零用于校正空载偏置；当前仓库未说明零偏是否掉电保存。

## 单关节位置测试

继续使用终端 B。以下命令预览左手食指的绝对目标角度：

```bash
python3 native_bridge/tests/hand/manual_single_joint.py --side left --joint INDEX_MCP --target 0.4
```

确认目标后，在命令末尾添加 `--execute`，脚本会读取该手另外五个关节的当前角度，只调用一次 `/hand/joint_switch/left`，并打印服务响应及之后的关节反馈。右手使用 `--side right`。`--target` 的单位是弧度，F5D6-P 关节位置范围见 [关节顺序与位置范围](../../../src/hand/README.md#关节顺序与位置范围)。

手势和压力标零服务的接口列表见 [桥接说明](../../README.md#灵巧手接口)。

## 四指持续力闭环

力控、键盘开合及退出复位说明见 [force_control](force_control/README.md)。
