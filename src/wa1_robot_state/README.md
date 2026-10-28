# WA1 机器人上肢状态与安全锁

`wa1_robot_state` 是 PC ROS2 Humble 软件包，提供一个命令行入口 `wa1_state`。它通过 `native_bridge` 读取机器人上肢模式与安全锁状态，也可以显式调用机器人原有的解锁服务。该包不连接机器人 ROS1 Master。

## 接口

| 用途 | ROS2 接口 | 类型 |
|---|---|---|
| 上肢运行模式 | `/upperlimb/uplimb_state` | `upperlimb/msg/UplimbState` |
| 查询安全锁 | `/upperlimb/safety_lock` | `std_srvs/srv/Trigger` |
| 请求解锁 | `/upperlimb/unlock` | `std_srvs/srv/Trigger` |

上肢模式 `cmd_num=15` 表示 `PROTECTED`。`safety_lock` 是查询服务：其响应 `success=true` 表示安全锁处于锁定状态，`success=false` 表示未锁定；这里的 `false` 不是查询失败。`unlock` 响应的 `success` 则表示解锁请求是否成功。

## 编译与运行

以下命令均在 `WA1_ws` 根目录执行。先编译 PC 软件包与上肢接口：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to wa1_robot_state
```

桥镜像需要包含 `safety_lock` 转发代码；更新镜像后重启容器：

```bash
python3 native_bridge/bridge.py build
python3 native_bridge/bridge.py stop
python3 native_bridge/bridge.py start
```

在另一个终端加载与桥相同的 ROS2 domain 和 PC 工作空间环境：

```bash
export ROS_DOMAIN_ID=0
source native_bridge/setup_pc.bash
ros2 run wa1_robot_state wa1_state status
ros2 run wa1_robot_state wa1_state lock
```

`status` 同时返回上肢模式和安全锁状态；`lock` 只查询安全锁。两条命令均不发控制请求。需要向机器人请求解除保护时，显式运行：

```bash
ros2 run wa1_robot_state wa1_state unlock
```

`unlock` 先读取新鲜的上肢模式和安全锁状态；如果已解锁且模式不是 `PROTECTED`，输出 `already_unlocked`。否则调用一次 `/upperlimb/unlock`，再查询状态并等待退出保护模式。可在子命令前使用 `--timeout SECONDS` 调整等待时间，例如 `wa1_state --timeout 8 status`。桥自己的流故障锁存仍由 `/upperlimb/bridge/reset_fault` 单独管理。
