# WA1_ws 代码整理计划

初版日期：2026-10-26；2026-10-27 按“发布冻结基础镜像，其他用户本地增量构建桥”方案更新。工作分支：`v1-work`。本地第 0 版提交 `7f5fb0f`，标签 `v0.0.0-experimental` 保持不变。桥专用的根目录 `docker/` 与 `tools/` 已并入 `native_bridge/`。

## 1. 代码与构建边界

| 模块 | 作用与调用链 | 本轮去向 |
|---|---|---|
| `src/upperlimb` | PC 的 ROS 2 `msg/srv`；同一接口定义也被桥接 Dockerfile 复制到 ROS1/ROS2 容器内编译 | 保留 |
| `src/pico4_ros2` | PICO4 官方 PC Service → C++ 接收器 → `/pico4/frame`；含独立网页查看器、回放和测试 | 保留独立软件包；SDK `.deb` 的 CMake 路径可配置 |
| `src/wa1_pico4_teleop` | 订阅 `/pico4/frame` 和桥反馈，经锁定、握键及速率限制后给桥发双臂命令；保留数学单元测试与只读监视工具 | 默认启用完整双臂流程，移除隔离 mock 与分阶段实验入口 |
| `wa1_description` | 静态 URDF、可配置 xacro、mesh 和 RViz 启动文件 | 移入 `src/wa1_description`；可选手部模型不再写作基础编译依赖 |
| `native_bridge` | Dockerfile、启动管理器、容器入口、C++ 桥包、基础镜像材料、现场测试和 PC 模拟 | 整体留在根目录；`COLCON_IGNORE` 隔离宿主编译 |
| `native_bridge/src/wa1_native_bridge` | C++ 专用桥，依赖同时存在的 Noetic、Humble 和 `ros1_bridge` | 留在桥目录内，只由容器构建 |
| 历史基础镜像配方与 ROS1 接口材料 | 配方仅供追溯；ROS1 元数据仍参与增量构建 | 历史配方归档到 `native_bridge/docs/archive/base_image/`；活动接口移到 `native_bridge/ros1_interfaces/` |
| `tools/pc_docker.py` | `bridge.py`、`run_mock_test.py` 共用的本机 Docker 调用包装器 | 移为 `native_bridge/pc_docker.py` |

`native_bridge` 放在 `src/` 外是构建边界：它不是宿主 Humble 包集合的一部分。把它整体移入 `src/` 会让双 ROS 构建材料与宿主软件包混杂。测试、配置、`patch_transport.py`、`entrypoint.sh`、`pc_docker.py` 和 ROS1 构建元数据统一归桥目录管理。`src/upperlimb` 是共享 ROS 2 接口包：遥操作包和以后任何 PC 上位机节点都直接依赖它，因此仍在 `src/`，并非只属于桥内部。

## 2. 旧代码和实验资料

| 原路径 | 处理 |
|---|---|
| `src/wa1_rosbridge_uplink` | 旧 WebSocket 上行包已被原生桥取代，从 v1 工作树移除；第 0 版可恢复 |
| `scripts/servol_hold_test.py` | 旧直连控制试验入口，当前链路无调用者，从 v1 工作树移除；根目录 `scripts/` 随之不再需要 |
| `docker/Dockerfile`、`docker/entrypoint.sh`、`docker/params/upperlimb_bridge_uplink.yaml` | 旧 Foxy 动态桥专属文件，从 v1 工作树移除；历史 Humble 基础配方现归档在 `native_bridge/docs/archive/base_image/`，不再参与增量构建 |
| `native_bridge/evidence/` | 保存已提交的第 0 版实验基线；新模拟和零指令握手结果写到被 Git 忽略的 `native_bridge/results/` |
| `native_bridge/tests/**/results/` | 本机测量数据，继续被 Git 忽略；测试脚本、README 和合成样本保留 |
| `build/`、`install/`、`log/`、`.venv/` | 本机编译与运行产物，继续留在磁盘但不进 Git；`.dockerignore` 也不传给镜像构建 |

外层 `官方教程/`、`report/`、`gpt_report/` 不属于本仓库，本轮不移动。`src/pico4_ros2/vendor/original_*.msg` 和 `src/wa1_pico4_teleop/reference/` 是供应商接口对照材料，暂留；是否随第 1 版公开须做授权核查。

## 3. 实施与验收顺序

1. 从 `v0.0.0-experimental` 建立 `v1-work`；第 0 版分支与标签不改动。
2. 移除无调用者的旧入口，迁移模型包，修正 ROS1 容器专用目录的发现边界。
3. 让模拟结果与历史证据分开，加入 `.dockerignore`，并允许使用外部路径提供 PICO4 SDK 安装包。
4. 将桥专用 `docker/`、`tools/` 并入 `native_bridge/`，更新启动管理脚本、两份 Dockerfile 的 `COPY` 路径以及受路径变动影响的说明。
5. 用 `colcon list` 核对宿主只发现四个 ROS 2 包；编译并运行软件包测试与 PC 隔离模拟回归。所有控制测试只连接本机模拟 ROS1 Master，不连接机器人。
6. 通过后在本地 `v1-work` 提交；不配置 GitHub 远程，也不推送。

## 4. 整理过程中的本机验收记录

- `colcon list --names-only` 仅列出 `pico4_ros2`、`upperlimb`、`wa1_description`、`wa1_pico4_teleop`。
- 四个包在独立的 `build/v1_work`、`install/v1_work` 中编译通过；`colcon test-result` 汇总为 19 项测试、0 错误、0 失败。
- 整理前的 `mock_operator_flow.py`、`mock_motion_levels.py` 在 PC 隔离 DDS domain 87 通过；本轮遥操作包清理后，两份脚本已从工作树移除。`run_mock_test.py --seconds 3` 在 PC 回环 ROS1 Master 与 DDS domain 83 通过，其结果位于 `native_bridge/results/`。
- 将 Docker 包装器和基础镜像材料收拢后，`run_mock_test.py --seconds 3` 再次通过；桥管理器的命令行入口正常。
- 使用 `FROM scratch`、`--network none` 的本机 Docker 构建上下文探针，实际执行了两份 Dockerfile 所需源目录的 `COPY`，全部通过。基础镜像从零构建和真机回归属于后续镜像、现场验收阶段。

## 5. 第 1 版发布前仍需处理

- 上传当前经验证的 `linux/amd64` 基础镜像，记录镜像仓库 digest；用这个 digest 替换当前仅用于本机过渡的 `wa1_ros1_bridge_humble:latest` 默认值，在干净环境中从线上基础镜像增量构建并运行 PC 模拟回归。历史基础 Dockerfile 不作为公开配方。
- 确认 PICO4 官方 SDK `.deb`、WA1 URDF/mesh 和供应商参考代码的再分发许可；决定二进制随仓库、镜像或由使用者另行获取的方式。
- 清理硬编码的本机路径、网络参数和旧手册描述，写出从空白机器到状态检查、隔离模拟和人工真机验收的完整步骤。
- 发布整理后的源码与增量 Dockerfile 到 GitHub，只把冻结的基础镜像发布到容器仓库；使用者下载基础镜像并在本地构建 `wa1-upperlimb-bridge:humble`。基础镜像地址与 digest 记录在 `native_bridge/docs/BASE_IMAGE.md`。第 0 版历史不随 `main` 上传。
