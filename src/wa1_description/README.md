# WA1 机器人描述与 RViz2 展示

`wa1_description` 是 WA1 400L 的最小 ROS 2 Humble 描述包。它安装静态 URDF、STL mesh、RViz2 配置和展示 launch，不依赖机器人、ROS1 桥或可选手部模型。

## 包内容

| 路径 | 用途 |
|---|---|
| `urdf/WA1_400L.urdf` | WA1 400L 的 link、joint、惯性参数和 mesh 引用 |
| `meshes/` | URDF 使用的 STL 外观与碰撞模型 |
| `launch/display.launch.py` | 同时启动关节 GUI、状态发布器和 RViz2 |
| `rviz/wa1_400l.rviz` | 使用 `BASE` 固定坐标系的默认 RViz2 视图 |

## 安装依赖与构建

以下命令在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/wa1_description --ignore-src --rosdistro humble -r -y
colcon build --packages-select wa1_description
source install/setup.bash
```

每个新终端都需要重新加载 `/opt/ros/humble/setup.bash` 和工作空间的 `install/setup.bash`。修改 URDF、mesh、launch 或 RViz 配置后，需要重新构建并重新加载工作空间环境。

## 在 RViz2 中查看模型

```bash
ros2 launch wa1_description display.launch.py
```

该命令会启动三个进程：

- `robot_state_publisher`：读取 URDF，并根据 `/joint_states` 发布 TF；
- `joint_state_publisher_gui`：显示可动关节滑块并发布 `/joint_states`；
- `rviz2`：加载包内配置，以 `BASE` 为固定坐标系显示 RobotModel。

在关节状态窗口拖动滑块即可检查升降、腰部、双臂、头部和轮组关节。关闭 launch 所在终端或按 `Ctrl+C` 会结束整组进程。

## 真机数据实时映射到 RViz2

先按 [桥接说明](../../native_bridge/README.md) 完成 PC 软件包和桥镜像构建，确保有线网络能够访问机器人。PC 默认地址为 `192.168.217.10`。

在工作空间根目录启动桥：

```bash
python3 native_bridge/bridge.py start
```

另开终端构建描述包并启动实时展示：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select wa1_description
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
source install/local_setup.bash
ros2 topic echo --once --qos-reliability best_effort /upperlimb/joint_states
ros2 launch wa1_description live_display.launch.py
```

实时入口按 `JointState.name` 将 `/upperlimb/joint_states` 的位置映射到 URDF，旋转关节单位为弧度，升降为米。它使用 BestEffort 订阅和 PC 时间戳，收到反馈时发布完整模型关节状态；未收到反馈的关节保持零位，反馈中断后模型停留在最后姿态。本模型不含灵巧手，轮组零位也不代表实时底盘里程计。

映射后的状态在 `/wa1_visualization/joint_states`，模型 TF 位于 `/wa1_visualization/tf` 和 `/wa1_visualization/tf_static`，RViz 已自动订阅对应 TF。运行时关闭手动展示入口 `display.launch.py`，避免多个 RobotModel 描述发布器同时运行。

本入口只订阅机器人反馈，不发送运动命令。排查收不到数据时，先检查桥日志及 `/upperlimb/bridge/status`，再检查 `/upperlimb/joint_states` 和 `/wa1_visualization/joint_states`。

## 常见问题

- `Package 'wa1_description' not found`：确认构建成功，并在当前终端执行 `source install/setup.bash`。
- RViz2 中模型为空或 mesh 报错：确认当前使用的是最新安装空间；重新执行构建和 `source install/setup.bash`。URDF 中的 mesh 应使用 `package://wa1_description/meshes/...` URI。
- `joint_state_publisher_gui` 或 RViz2 无法打开窗口：确认当前会话具有可用的图形显示环境和 `DISPLAY`；SSH 使用时需要配置图形转发。
- 模型出现但部分关节不动：检查 `/joint_states` 是否存在，可运行 `ros2 topic echo /joint_states --once`。

当前模型资源的上游许可字段仍为 `TODO`；公开再分发前需要完成授权确认。
