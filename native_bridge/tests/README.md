# 控制测试索引

以下命令均在工作空间根目录运行。真机测试先按[桥接说明](../README.md#启动查看与关闭)启动桥，再按[环境约定](../../README.md#命令运行前的环境)加载客户端环境。各控制入口默认预览，显式追加 `--execute` 才发送控制请求；具体前置检查和收尾行为见各模块说明。

| 测试 | 模块说明 |
|---|---|
| 单关节绝对位置 MoveJ | [position_control](position_control/README.md) |
| 单关节轨迹 ServoJ | [position_servo](position_servo/README.md) |
| 单臂笛卡尔位置 MoveL | [cartesian_position](cartesian_position/README.md) |
| 单臂笛卡尔轨迹 ServoL | [cartesian_servo](cartesian_servo/README.md) |
| 左腕 ServoJ 延迟分析 | [position_servo/latency](position_servo/latency/README.md) |
| 灵巧手只读检查与单关节位置 | [hand](hand/README.md) |
| 四指力控与键盘开合 | [hand/force_control](hand/force_control/README.md) |

完整桥链路的本机模拟回归见[桥接测试说明](../README.md#测试入口与验收状态)。

## 关节分组

MoveJ、ServoJ 的 `--joint-name` 使用下表中的关节名。部位后缀用于组成各自的服务或话题端点，`arm_type` 是对应的部位掩码；两个单关节入口均不使用 `whole_body`。

| 部位 | 部位后缀 | `--joint-name` 可选值（按数组顺序） | `arm_type` |
|---|---|---|---:|
| 左臂 | `left_arm` | `Shoulder_Y_L, Shoulder_X_L, Shoulder_Z_L, Elbow_L, Wrist_Z_L, Wrist_Y_L, Wrist_X_L` | 1 |
| 右臂 | `right_arm` | `Shoulder_Y_R, Shoulder_X_R, Shoulder_Z_R, Elbow_R, Wrist_Z_R, Wrist_Y_R, Wrist_X_R` | 2 |
| 颈部 | `neck` | `Neck_Z, Neck_Y` | 4 |
| 腰部 | `waist` | `Waist_Z, Waist_Y` | 8 |
| 升降 | `lifting` | `Lifting_Z` | 16 |

关节顺序、掩码与保守测试范围由 [joint_config.py](joint_config.py) 统一维护。旋转轴单位为 rad，升降轴单位为 m；范围取所附 URDF 与遥操作教程示例的交集，尚未确认为现场控制器的实际限位。
