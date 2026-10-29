# head_camera_viewer：PC 本地头部相机查看器

## 作用

`head_camera_viewer` 在 PC 浏览器中同时显示 WA1 头部 RealSense D435i 的彩色图像和对齐到彩色视角的深度图。深度图在 PC 上按固定距离范围着色；侧边栏展示 PC 实际接收帧率、相机原始分辨率、最新帧时间、彩色帧大小和深度有效像素比例。

查看器从 Jetson 的 ROSBridge 订阅 ROS1 压缩图像，在 PC 的 `127.0.0.1:8767` 提供网页。它独立于 ROS2 工作空间和上肢桥运行，启动前无需 `source` 任何 ROS 脚本。

## 准备

当前机器人的 Jetson 有线地址实测为 `192.168.217.100`，ROSBridge 监听 `9090`；PC 通过直连网线访问该地址。Jetson 的 `naviai_sensor` 和 `naviai_rosbridge` 容器需运行，头部相机 ROS1 图像话题需持续发布。

在 `WA1_ws` 根目录使用已有 `.venv`。首次部署且尚无 `.venv` 时，可先安装依赖：

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r head_camera_viewer/requirements.txt
```

已有 `.venv` 但缺少 `cv2`、`numpy` 或 `websocket-client` 时，只运行第二行安装命令。

## 启动与关闭

在 `WA1_ws` 根目录执行：

```bash
.venv/bin/python head_camera_viewer/head_camera_viewer.py
```

浏览器打开 `http://127.0.0.1:8767`。终端按 `Ctrl+C` 关闭查看器。

默认显示彩色和对齐深度两个画面。彩色图直接使用 ROS1 的 JPEG 压缩帧；深度图从 `16UC1; compressedDepth png` 解码，按 `0.3～5.0 m` 的固定范围着色，无效像素显示为深灰色。深度图仅在显示时缩至宽 `800` 像素；侧边栏的“分辨率”仍表示相机发布的原始分辨率，“显示尺寸”表示处理后的画面尺寸。网页布局会随窗口宽度调整。

参数示例：

```bash
.venv/bin/python head_camera_viewer/head_camera_viewer.py \
  --jetson-host 192.168.217.100 \
  --rosbridge-port 9090 \
  --ui-port 8767 \
  --depth-min-m 0.3 --depth-max-m 5.0 \
  --depth-preview-width 800
```

`--jetson-host` 也可以设为当前 Jetson 的 Wi-Fi 地址，例如 `192.168.0.104`。双路图像通过有线网络传输时更稳定。`WA1_CAMERA_HOST` 环境变量可设置默认 Jetson 地址，命令行 `--jetson-host` 优先。

## 数据链

| 画面 | ROS1 订阅话题 | PC 处理 |
|---|---|---|
| 彩色 | `/zj_humanoid/sensor/realsense_head/color/image_raw/compressed` | 解码验证 JPEG 后，将压缩帧交给本地网页显示。 |
| 对齐深度 | `/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth` | 解码 16 位深度 PNG，按固定范围生成伪彩色预览。 |

查看器为两路图像各保留最新帧；网页显示跟不上时会跳过旧帧。侧边栏的 FPS 是 **PC 接收并处理后的帧率**，不是相机驱动的设定频率；“最新帧”使用 PC 单调时钟计算，不依赖 PC 与 Jetson 的时钟同步。

当前真机通过 ROSBridge 双路订阅时，PC 实测彩色约 30 FPS、深度约 5～7 FPS，最新帧处于百毫秒量级。深度显示帧率受 ROSBridge 双路压缩图传输吞吐影响；需要完整相机帧率供 ROS2 节点处理时，应另建专用的图像传输链路。

## 状态与故障排查

查看器运行时，在另一终端读取两路连接与帧状态：

```bash
curl -s http://127.0.0.1:8767/api/status | python3 -m json.tool
```

如果 `connected=false` 或网页显示“连接中断”，检查 `robot_host` 是否为当前 Jetson 地址、PC 到 Jetson 的有线路由，以及 ROSBridge 端口：

```bash
ip route get 192.168.217.100
nc -vz -w 3 192.168.217.100 9090
```

端口能连通只表示 ROSBridge 在监听，不表示相机正在发送图像。若状态中 `connected=true`，但彩色和深度的 `frames` 长时间为 `0`、`state=waiting`，先看 `decode_errors` 和 `error`：解码错误持续增长时，检查图像消息格式；解码错误仍为 0 时，确认订阅的是上表中的两个话题，再检查 Jetson 的 RealSense 进程及日志。ROS1 中能列出相机话题，也只表示发布者已注册；要确认图像流，应查看帧计数是否增长。

这次现场故障中，ROSBridge 已订阅两路图像，但相机 ROS1 节点的发送计数连续数秒为 0；`naviai_sensor` 容器和 RealSense 进程仍显示运行，D435i 也仍能被 USB 识别。通过 SSH 登录 Jetson 后，可检查进程与日志：

```bash
ssh naviai@192.168.0.104
docker exec naviai_sensor supervisorctl -c /etc/naviai/supervisor.conf status
docker exec naviai_sensor tail -n 60 /var/log/realsense.out.log
```

若 RealSense 进程仍运行而两路帧持续为 0，可只重启该进程，再回到 PC 用 `/api/status` 检查 `frames` 是否增长：

```bash
docker exec naviai_sensor supervisorctl -c /etc/naviai/supervisor.conf restart realsense
```

本次重启后彩色与对齐深度均恢复出图。日志中出现过 USB 控制传输告警，但恢复后也出现过同类告警，因此目前不能据此确定停发帧的底层原因。若 `frames` 在增长而网页仍没有画面，再检查状态中的 `decode_errors`、`error` 和浏览器是否连接到本机的 `127.0.0.1:8767`。
