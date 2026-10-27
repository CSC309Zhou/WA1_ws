# Wrist_Z_L 位置伺服延迟探针

本目录仅针对左腕 `Wrist_Z_L`。它复用上一层 `servoj_single_joint.py` 的 200 Hz 轨迹、ServoJ 参数、反馈就绪检查、终止时 `stop`/清除参数以及下一次运行前的桥故障复位逻辑。默认只预览，`--execute` 才会发出真机指令。脚本、镜像和结果均在 PC；不修改 Jetson 或 Pico。

## 链路与测点

当前 ROS1 Master 的注册信息显示，左臂 ServoJ 接收节点以及 `/zj_humanoid/upperlimb/joint_states` 发布节点位于 `192.168.217.66`。PC 到该地址的路由走 `eno1` 有线网卡，源地址为 `192.168.217.10`。Master `192.168.217.1` 用于 ROS1 发现；Jetson 的 Wi-Fi SSH 地址不是这条 ServoJ 数据路径中的测点。

所有可减算的时刻均取自同一台 PC 的 `CLOCK_MONOTONIC`，容器与宿主共享内核时钟：

1. PC ROS2 Python 节点进入及返回 `publisher.publish()`；
2. PC 桥进入对应 ROS2 回调；
3. PC 桥调用及返回 ROS1 `publish()`；
4. PC 桥进入 ROS1 JointState 回调，写入已有 `/upperlimb/bridge/joint_receive_time`；
5. PC ROS2 Python 节点收到 JointState。

新增的 `/upperlimb/bridge/servoj_left_arm_timing` 只在 PC 桥启动时指定 `--latency-probe` 后出现。它报告每条已接受左臂 ServoJ 指令的桥回调时间和 ROS1 发布调用时间。通过移动段关节目标值的精确 double 位模式匹配到发布样本；起终点重复的定点指令不参与逐条匹配。反馈侧通过原有 `JointState.header.stamp` 匹配两条上行话题的同一帧，**不将机器人的壁钟与 PC 时钟直接相减**。

`latest.json` 的 `latency_analysis.segments` 给出已测 PC 环节的样本数、中位数、P95、P99、最小值和最大值。`trajectory_threshold_crossings` 给出目标与反馈到达 10%、50%、90% 位移时的 PC 接收时间差。这是轨迹**表观滞后**，包含 ROS1 数据传输、Pico 控制和采样、关节响应、反馈返回；它不等于单向网络延迟。ROS1 `publish()` 返回只表示本地调用完成，**不证明 Pico 已收到、更不证明电机已执行**。在没有 Pico 进程内时戳或独立物理测量前，不能把未测部分细分，也不能据此断定最大延迟来自哪个子环节。

阈值法与此前采用的轨迹移位拟合不是同一个估计量，数值无需完全相同；位移太小、接近反馈抖动时，阈值交点尤其容易偏移。应结合原始曲线和三处阈值逐项检查。

## PC 侧准备

从 `~/WA1` 执行，重新构建 PC 桥镜像后启用诊断开关：

```bash
python3 WA1_ws/native_bridge/bridge.py build
python3 WA1_ws/native_bridge/bridge.py start --latency-probe
source WA1_ws/native_bridge/setup_pc.bash
ros2 topic list --no-daemon -t | rg 'servoj_left_arm_timing|joint_receive_time|joint_states'
```

如果 `start` 提示项目容器已经存在，先检查当前任务与容器状态；若现有控制任务正在运行，先结束任务并确认机器人停止，然后运行 `python3 WA1_ws/native_bridge/bridge.py stop` 移除项目容器，再执行上面的 `start`。`build`、`stop`、`start` 本身不发布关节目标。诊断开关固定在容器启动参数中；普通控制模式依旧默认关闭。

先预览指定相对位移和运动时长：

```bash
python3 WA1_ws/native_bridge/tests/position_servo/latency/manual_wrist_z_l_latency.py \
  --duration 1 --delta 0.1
```

预览只订阅实时反馈、生成轨迹和结果文件；不创建 ServoJ 发布者，也不调用控制服务。与原测试相同，`--duration` 必须严格大于 0.1 s，计划峰值速度不得超过 5 rad/s，目标须处于当前配置的位置范围。测试者现场确认方向、范围和机器人状态后，才手动追加 `--execute`：

```bash
python3 WA1_ws/native_bridge/tests/position_servo/latency/manual_wrist_z_l_latency.py \
  --duration 1 --delta 0.1 --execute
```

每次覆盖 `latency/results/Wrist_Z_L/latest.json`、`latest.png` 和 `latest_timing.png`；`--output-dir` 可以修改结果根目录。JSON 保留每条发布、桥回调和反馈原始时戳，以便复核配对。诊断话题采用 best effort；先检查 `paired_moving_commands / moving_commands` 与 `paired_joint_feedback_samples / joint_feedback_samples`，若匹配率低，应先排查丢样或 CPU 负载，再解读分位数。测试结束后桥的有限指令流 watchdog 可能锁存；下一次执行会沿用已有的 STOPPED 检查及本地桥复位流程。

## Pico 已有日志：同机时钟测量

只读 SSH 核查发现 Pico `naviai_wa_h_robot_node` 已在 `/home/nav01/zj_humanoid/logs/run.log.1` 中为每条左臂 ServoJ 回调写入带 Pico ROS 时间的目标值。Pico 的 `NTPSynchronized=no`，其壁钟与 PC 约有十秒偏移；**不能直接把 PC 时戳减去 Pico 时戳**。`pico_log.py` 只读提取实验窗口内的左臂日志，按目标值配对，再用同属 Pico 时间域的日志时戳与 `JointState.header.stamp` 求差：

```bash
python3 WA1_ws/native_bridge/tests/position_servo/latency/pico_log.py --fetch
```

若 SSH key 不可用，会提示输入 Pico 密码；密码经文件描述符交给 `sshpass`，不会写入脚本、命令行参数或结果。此机器官方文档标注 Pico 密码为单个空格。日志轮转后可用 `--remote-log /home/nav01/zj_humanoid/logs/run.log.2` 指定包含该次实验的文件；默认读取 `.1`。这一步**不需要再次运动**，可以对现有 `latest.json` 使用。结果保存在 `latest_pico_log.jsonl`、`latest_pico_analysis.json` 与 `latest_pico.png`。可用 `--local-log <已下载的 JSONL>` 完全离线重算。

2026-10-23 的单次 `+0.1 rad / 1 s` 实验，Pico 日志 301 条；中段 121 条指令与 PC 目标一一配对。50%/90% 位移处，Pico 回调日志→Pico JointState 时戳分别为 42.77/43.42 ms；PC 两端对应为 43.89/44.57 ms。10%–90% 九个交点的两种差值再相减，范围约 1.04–1.26 ms。这表明大部分**反馈轨迹表观滞后**位于 Pico 回调与关节状态时戳之间；该区间仍包含控制计算、驱动响应和采样，JointState 也不是独立物理测量。

## PC 有线抓包：界定网络与传输环节

`wire_capture.py` 只抓 PC `eno1` 上与 `192.168.217.66` 之间的 TCP 包，不启动桥、不发布 ROS 消息。它用 `tcpdump` 的 PC 内核软件时戳，持续记录 PC 实时时钟与单调时钟的映射。抓包需要 PC 的 sudo 权限。终端 A 在同一个终端中先取得 sudo 缓存，再启动抓包：

```bash
sudo -v
python3 WA1_ws/native_bridge/tests/position_servo/latency/wire_capture.py --seconds 20
```

看到 `Capture ready` 后，现场操作者在终端 B 运行**已经确认安全**的延迟实验；如果只想检查抓包程序，可不运行运动测试，届时离线分析会报告没有配对的指令。抓包输出为 `latest_wire.pcap` 和 `latest_wire_clock.json`，均在 PC 的 `results/Wrist_Z_L` 中。脚本以普通用户保存文件；`sudo` 只用于启动 PC 的 `tcpdump`。抓包窗口要覆盖整段运动及其前后状态。

`tcpdump` 在当前 PC 上可能把输出文件改属主为 `nobody`；抓包脚本结束时现会用已缓存的本机 sudo 权限改回当前用户，供普通用户运行 `wire_analyze.py`。抓包与 `latest.json` 必须来自**同一次运动**：若先完成运动、隔了数分钟才开始抓包，现有 pcap 无法追溯那些已发出的包。离线脚本会在读完整 pcap 前先检查两个时间窗是否重叠。

同一次实验结束后，先运行 `pico_log.py --fetch` 更新该次 Pico 分析，再运行：

```bash
python3 WA1_ws/native_bridge/tests/position_servo/latency/wire_analyze.py
```

`wire_analyze.py` 离线重组 TCP 流，用移动段目标值的精确 double 位模式匹配 ROS1 `Joints` 帧，并用 `JointState.header.stamp` 匹配入站状态帧。它报告桥 ROS1 `publish()` 返回→PC 出站抓包、PC 入站抓包→桥 ROS1 回调，以及 PC 出站→入站的表观轨迹间隔。结合**同一次** Pico 日志，脚本给出两次网络传输及时间戳落点的合并余量；它不能精确分开两个单向网络延迟。输出为 `latest_wire_analysis.json` 与 `latest_wire.png`。分析会检查 Pico 结果是否属于同一次实验，并列出帧配对率、TCP 重组缺口及抓包时钟校准波动；配对不足时不要解释分位数。

抓包时戳来自 PC 内核软件路径，不是网卡硬件时戳；Pico 日志发生在回调内部，`Header.stamp` 也未证明电机物理响应时刻。建议在重复实验中比较正反方向和不同速度，并对照无诊断开关时的轨迹，确认测量本身的影响。不能根据当前一组数据直接调整控制器参数。
