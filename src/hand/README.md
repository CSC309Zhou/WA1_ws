# WA1 灵巧手 ROS2 接口

本包镜像机器人 ROS1 `hand` 包的三个接口，供 PC 上的 ROS2 Humble 节点使用：

| 接口 | 字段 |
|---|---|
| `hand/msg/PressureSensor` | `std_msgs/Header header`、`float64[] pressure` |
| `hand/srv/HandJoint` | 请求 `float32[] q`；响应 `bool success`、`string message` |
| `hand/srv/Gesture` | 请求 `string[] gesture_name`；响应 `bool success`、`string message` |

本包只定义接口，没有常驻节点。PC 的 `native_bridge` 镜像使用同一组字段构建 ROS1 接口和转换工厂，再把机器人数据接入 ROS2；话题与服务列表见 [桥接说明](../../native_bridge/README.md#灵巧手接口)。

## 指尖力反馈

`PressureSensor.pressure` 表示指尖力，单位为 **0.1 N**：

```text
力值（N）= pressure 中的读数 × 0.1
```

例如，读数 `25` 对应 `2.5 N`。根据 F5D6-P 接口资料，反馈按**大拇指、食指、中指、无名指、小指**排列；桥按收到的数组顺序转发，不进行重排或单位换算。

现有真机检查脚本要求每手数组长度为 6，而上述资料只明确了五指顺序，未说明六个元素的完整对应关系。第六个元素的含义及完整下标映射仍需确认，不能套用下方的六关节顺序。

## 关节顺序与位置范围

每只手的关节数组顺序及位置范围如下，范围依据 F5D6-P 接口资料；资料中的 `INDEX`、`MIDDLE`、`RING`、`LITTLE` 对应下表的 `*_MCP` 名称。

| 下标 | 代码中的关节名 | 含义 | 最小值（°） | 最大值（°） | 最小值（rad） | 最大值（rad） |
|---|---|---|---|---|---|---|
| 0 | `THUMB_MP` | 拇指掌指关节 | -45.0 | 45.0 | -0.7854 | 0.7854 |
| 1 | `THUMB_CMC` | 拇指腕掌关节 | -20.0 | 90.0 | -0.3491 | 1.5708 |
| 2 | `INDEX_MCP` | 食指掌指关节 | 0.0 | 80.0 | 0 | 1.3963 |
| 3 | `MIDDLE_MCP` | 中指掌指关节 | 0.0 | 80.0 | 0 | 1.3963 |
| 4 | `RING_MCP` | 无名指掌指关节 | 0.0 | 80.0 | 0 | 1.3963 |
| 5 | `LITTLE_MCP` | 小指掌指关节 | 0.0 | 80.0 | 0 | 1.3963 |

`hand/srv/HandJoint` 请求的 `q` 和 `/hand/joint_states` 的关节位置均使用**弧度**。表中弧度值为近似值。

## 编译与接口查看

在 `WA1_ws` 根目录编译和查看接口：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hand
source native_bridge/setup_pc.bash
ros2 interface show hand/srv/HandJoint
ros2 interface show hand/srv/Gesture
```

真机反馈只读检查与单关节位置测试见 [灵巧手接口检查](../../native_bridge/tests/hand/README.md)。
