# WA1_ws 代码整理计划

更新日期：2026-11-09。检查依据和分阶段实施记录见[整理报告](REDUNDANCY_REVIEW_2026-11-09.md)。本计划按最新用户要求更新；提交状态以 Git 记录为准。

## 已完成

- [x] 合并左右臂 MoveL、ServoL 的重复实现，各保留一个通用入口，使用必填 `--side left|right` 选择手臂。
- [x] MoveJ、ServoJ 各保留一个通用入口，通过必填 `--joint-name` 选择 19 轴之一，删除 38 个逐关节包装文件。
- [x] MoveJ 统一要求显式 `--target`，移除左肩特殊默认目标、专用输出兼容分支和废弃的重复入口。
- [x] 测试内部共用 `joint_config.py` 的关节顺序、掩码和保守测试限位，保持原值及单位。
- [x] 删除早期 `native_bridge/evidence/` 材料和相关索引、失效配置与链接。
- [x] 修正不存在的目录、示例命令的工作目录，以及旧验收材料的适用范围。
- [x] 删除根目录旧整理计划，将现行计划和报告统一放在 `docs/` 并允许 Git 跟踪。

## 当前结构与边界

以下路径均相对仓库根目录；运行和环境准备见[根 README](../README.md)与[桥接说明](../native_bridge/README.md)。

| 路径 | 职责 |
|---|---|
| `src/upperlimb`、`src/hand` | 共用 ROS2 接口字段；桥镜像复用同一份定义构建 ROS1 接口 |
| `native_bridge/` | 双 ROS 桥、Docker 构建与管理；由 `COLCON_IGNORE` 隔离宿主构建 |
| `native_bridge/tests/joint_config.py` | 测试使用的关节分组、掩码和保守范围 |
| `native_bridge/tests/position_control/movej_single_joint.py` | 通用绝对位置 MoveJ：`--joint-name`、`--target` |
| `native_bridge/tests/position_servo/servoj_single_joint.py` | 通用轨迹 ServoJ：`--joint-name`、`--duration`、`--delta` |
| `native_bridge/tests/cartesian_position/movel_single_arm.py` | 通用单臂 MoveL：`--side`，以及 `--delta` 或 `--absolute` |
| `native_bridge/tests/cartesian_servo/servol_single_arm.py` | 通用单臂 ServoL：`--side`、`--duration`、`--delta` |
| `native_bridge/tests/position_servo/latency/` | 左腕延迟采集、Pico 日志和抓包分析，复用通用 ServoJ |

上述通用控制入口默认预览，显式 `--execute` 才发送控制。手部硬件范围与保守测试范围继续区分；不因目录整理修改现场地址、控制参数或退出复位策略。

## 验证记录

- 操作者依赖模拟测试已按用户要求移除；当前 `python3 -m pytest -q`：12 项通过。
- 最新文档引用、从其他当前目录执行 CLI 帮助及 `git diff --check` 检查通过。
- 没有重新构建桥镜像或进行真机运动验收；上述结果不能替代现场验证。

## 后续事项

- [ ] 补齐可复核的真机运动验收记录：日期、提交版本、镜像引用、入口参数与实际结果。
- [ ] 现场确认控制器实际限位；当前测试范围不能作为硬件保护阈值。
- [ ] 根据来源方资料确认 SDK、模型和供应商参考材料的再分发许可。

本机构建目录、Python 环境、运行结果、供应商资源及历史基础镜像配方不属于此次删除范围。旧实施计划仍在本机 `docs/superpowers/` 中作为被忽略的过程材料，现行安排以本文为准。
