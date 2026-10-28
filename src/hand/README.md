# WA1 灵巧手 ROS2 接口

本包镜像机器人 ROS1 `hand` 包的三个接口，供 PC 上的 ROS2 Humble 节点使用：

| 接口 | 字段 |
|---|---|
| `hand/msg/PressureSensor` | `std_msgs/Header header`、`float64[] pressure` |
| `hand/srv/HandJoint` | 请求 `float32[] q`；响应 `bool success`、`string message` |
| `hand/srv/Gesture` | 请求 `string[] gesture_name`；响应 `bool success`、`string message` |

本包只定义接口，没有常驻节点。PC 的 `native_bridge` 镜像使用同一组字段构建 ROS1 接口和转换工厂，再把机器人数据接入 ROS2；话题与服务列表见 [桥接说明](../../native_bridge/README.md#灵巧手接口)。

每只手的关节数组按 `THUMB_MP, THUMB_CMC, INDEX_MCP, MIDDLE_MCP, RING_MCP, LITTLE_MCP` 排列。压力数组保留机器人原始下标；教程和机器人安装的消息注释对压力通道顺序描述不同。

在 `WA1_ws` 根目录编译和查看接口：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hand
source native_bridge/setup_pc.bash
ros2 interface show hand/srv/HandJoint
ros2 interface show hand/srv/Gesture
```

真机反馈只读检查与单关节位置测试见 [灵巧手接口检查](../../native_bridge/tests/hand/README.md)。
