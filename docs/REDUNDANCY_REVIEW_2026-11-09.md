# 当前代码与文档冗余整理报告

> 第 1～7 节保留首次静态检查的结论与当时状态；第一轮实施结果见第 8 节。用户随后要求移除逐关节入口，当前关节接口见第 9 节，笛卡尔接口见第 11 节及模块 README；第 10 节记录早期证据清理。

检查日期：2026-11-09。基准提交：`b123f50`。检查开始时 Git 工作树干净。

本轮仅检查并新增本报告，不删除、移动或重构现有文件，不运行连接机器人的脚本，不启动桥或执行运动测试。

## 1. 结论

当前主要问题是控制测试中的成对实现重复、关节元数据重复维护，以及现行说明与历史记录混在一起。没有发现可直接按文件内容去重的一组非空文本文件；不能把名称相似的入口、ROS1/ROS2 包元数据、历史证据直接作为垃圾删除。

最值得整理的是：

1. 左右臂 MoveL、ServoL 各抽取一份公共实现，保留原命令入口。
2. MoveJ、ServoJ 共用关节分组与测试限位；单独处理左肩旧实现的兼容性。
3. 修正文档中的目录、命令路径和验收证据归属，再收拢重复操作说明。
4. 将旧整理计划明确标为历史记录，区分已完成事项与仍待确认事项。

## 2. 检查范围与方法

| 项目 | 检查结果 |
|---|---|
| Git 跟踪文件 | 237 个；本报告新增前统计 |
| Git 跟踪 Markdown | 20 份 |
| Git 跟踪 Python | 81 份 |
| `src/*/package.xml` | 6 个 ROS2 包清单；未据此声称实际 colcon 发现或构建成功 |
| 额外本地文档 | `docs/superpowers/plans/` 中 2 份计划，被 Git 忽略 |
| 完全重复文件检查 | 对非空 `.py/.cpp/.hpp/.md/.msg/.srv/.yaml/.xml/.txt` 文件做 SHA-256 分组，未发现相同内容 |
| 代码结构检查 | Python AST 比较跨文件的至少 10 行函数；结合左右文件 diff、常量和调用入口核对 |
| 文档引用检查 | 20 份跟踪 Markdown 的标准内联相对链接均能找到本地目标；未验证 URL、锚点及反引号中的路径 |

重点阅读了桥构建与启动入口、四类运动测试、手部公共代码、模块 README、旧整理计划及证据类型；对其他模块做文件结构与相关引用检查。本报告是静态冗余审查，不是全仓运行正确性、安全性或死代码审计。没有构建、运行 ROS/Docker 回归或复核真机验收。

## 3. 代码冗余

### C1．左右臂笛卡尔测试复制了大部分实现【高优先级，已确认】

| 文件组 | 每份行数 | 顺序对齐的相同行数 | 相似率 |
|---|---:|---:|---:|
| `native_bridge/tests/cartesian_position/manual_{left,right}_arm_movel.py` | 296 | 266 | 89.9% |
| `native_bridge/tests/cartesian_servo/manual_{left,right}_arm_servol.py` | 570 | 540 | 94.7% |

口径：使用 `difflib.SequenceMatcher(autojunk=False)` 对原始行序列比较，比例为 `2×匹配行数/(两文件总行数)`，包含注释和空行；它衡量文本相似性，不等于可删除代码比例。

MoveL 的差异主要是关节名、端点、选取的 TCP、打印字段和另一臂反馈。ServoL 的差异主要是左右臂选择、话题、`arm_type`、结果目录和结果字段。AST 还确认了以下重复：

- MoveL 两份文件的 `quat_from_rpy`、`quat_multiply`、`pose_values`、`make_target`。
- ServoL 两份文件的 `quat_from_rpy`、`quat_slerp`、`plan_for`、`wait_for_feedback`、`call_service`、`reset_latched_fault` 及部分反馈方法。

建议每类保留一个公共实现，由原有左右臂文件传入固定侧别。提取时保留服务请求中 `pose[0]` 的现有语义、另一臂姿态填充、故障复位、stop/clear 收尾和 JSON 字段兼容性。不要仅通过字符串替换左右名称完成迁移。

收益：四份实现共 1,732 行，可消除约一份 MoveL 和一份 ServoL 规模的重复维护；实际净减行数取决于侧别配置与兼容处理，不给出确定删减承诺。

### C2．MoveJ 左肩仍保留独立旧实现【中优先级，已确认】

证据：

- `native_bridge/tests/position_control/manual_shoulder_y_l.py:1` 是 165 行独立实现。
- 同目录 `movej_single_joint.py:77` 已提供通用逻辑，其余 18 个单关节入口调用该模块。
- `position_control/README.md:12` 明确记录左肩入口默认目标为 `0.1 rad`，其他入口要求显式 `--target`。

两份逻辑重复了状态订阅、新鲜度、Master 校验、目标构造、服务调用和到位观察。直接换成通用入口会改变默认参数和输出字段。

建议把左肩默认值作为显式兼容配置，确认输出依赖后再接入公共实现。该文件有文档引用，不能当作无调用者旧文件直接删掉。

### C3．上肢关节分组与限位多处维护【高优先级，已确认】

`position_control/movej_single_joint.py:15` 与 `position_servo/servoj_single_joint.py:23` 重复维护 `PARTS`；前者 `:29` 与后者 `:37` 的 `TARGET_LIMITS` 逐项一致，覆盖 19 轴。笛卡尔测试与遥操作 C++ 另有左右臂关节名称列表。

风险是调整顺序、名称或测试范围时只修改一处。建议先让桥测试内部共用一份关节元数据；跨 Python/C++ 的运行模块是否进一步统一，应单独评估，避免为了去重引入运行时文件加载和部署依赖。

这些限位来自 URDF 与教程示例范围的交集，不能不加区分地替换为 URDF 原值或宣称为控制器硬件限位。

### C5．手部共同概念存在不同限位策略【中优先级，不能机械合并】

`tests/hand/manual_single_joint.py:15` 与 `tests/hand/force_control/hand_common.py:14` 都定义六关节名称和范围，但数值有意不同：

| 关节 | 单关节脚本／接口文档范围（rad） | 力控公共测试范围（rad） |
|---|---|---|
| THUMB_MP | -0.7854～0.7854 | -0.75～0.75 |
| THUMB_CMC | -0.3491～1.5708 | -0.31～1.54 |
| 四指 MCP | 0～1.3963 | 0～1.36 |

力控 README 已说明测试范围可比硬件范围更保守。这是策略差异，不是已证实的参数错误。可以共用名称和单位定义，同时保留有名称和来源的不同限位策略。单次位置测试与持续控制的退出复位流程也不应无条件合并。

`force_control.py` 与 `keyboard_four_fingers.py` 已共用 `hand_common.py`；当前共享方向合理。

## 4. 文档冗余与过时内容

### D1．根 README 引用了不存在的目录【高优先级，已确认】

`README.md:3` 称 `jetson/` 存放头部相机本地感知代码，但当前工作树和 Git 文件清单均无该目录。实际 `perception/apriltag/README.md` 描述的是 PC 订阅与校验代码，机器人端检测是外部前提。

建议将根说明改成准确的 PC 端范围，并说明机器人端代码所在位置或外部服务要求；不能仅为满足旧文档创建空目录。

### D2．运行命令的目录约定不一致【高优先级，已确认】

- 根 README 规定在 `WA1_ws` 根目录运行。
- `position_servo/README.md:17` 同样要求在根目录预览，但 `:20–22` 使用 `source WA1_ws/native_bridge/...` 和 `python3 WA1_ws/native_bridge/...`，实际多了一层 `WA1_ws/`。
- `position_control/README.md:8–9` 也使用上述前缀，未明确切换到仓库父目录。
- `native_bridge/README.md:126` 使用本机路径 `cd ~/WA1/WA1_ws`，与本次实际路径 `/home/kai/WA1_ws` 不符，也不适用于任意克隆位置。

建议统一为“以下命令在仓库根目录执行”，脚本路径均从 `native_bridge/` 开始；用户自行进入实际克隆目录。标准 Markdown 链接检查不会发现代码块里的这些问题。

### D3．旧整理计划仍被用作当前入口【中优先级，已确认】

`CODE_ORGANIZATION_PLAN.md` 有历史价值，但根 README 仍直接链接它，没有充分区分当前约定和历史过程：

- `:12` 提到可配置 xacro，当前描述包只有静态 URDF，CMake 安装目录也不包含 xacro。
- `:39` 的“四个 ROS2 包”是旧实施目标，当前 `src/` 已有六份包清单。
- `:52` 将发布基础镜像、用 digest 替换本机 latest 写成待办；当前 Dockerfile、`bridge.py` 和 `BASE_IMAGE.md` 已配置固定 GHCR digest。
- 文件开头的工作分支、旧提交，及“不配置远程、不推送”等是当时流程记录，不应被读作当前操作要求。

建议保留历史验收记录，增加明确历史标识；将当前构建边界归入根 README／桥 README，将待办拆成“已由现行代码体现”“仍待外部确认”。不能把旧测试结果改写成这次重新验证通过。

### D4．多处重复环境、网络和启动步骤【中优先级，已确认】

根 README、桥 README、AprilTag README、描述包 README 及测试说明多次解释 `ROS_DOMAIN_ID=0`、`setup_pc.bash`、默认 IP、构建和启停命令。重复入口有助于独立使用，但详细解释与现场记录重复后容易漂移。

建议保留每个模块可复制的最短启动段；完整 ROS 环境约定由根 README 维护，桥参数与故障排查由桥 README 维护，固定镜像资料由 `BASE_IMAGE.md` 维护。模块文档链接这些来源，保留自身独有参数、数据语义和结果说明。

特别注意：根 README 记录 Jetson `eno1` 实测地址为 `.100`，桥 `--jetson-ip` 默认值为 `.1`，代码说明该参数用于 `jzrobot-a` 主机别名。静态内容不足以判断它们是否必须相同，应补充网络角色说明，不能直接认定默认值错误并替换。

### D5．验收结论与指向的证据不匹配【高优先级，已确认引用不足】

`native_bridge/README.md:230` 称 19 轴 MoveJ/ServoJ、双臂 MoveL/ServoL 已完成真机测试，并将历史验收数据指向 `evidence/`。

但该目录相关 JSON 的类型是：

- `real_robot_readonly.json`：`real_robot_read_only`。
- `real_robot_handshake.json`：`real_robot_handshake_no_commands`。
- `mock_control_results*.json`：`private_mock_control`。
- 上述四份均为 `motion_on_real_robot: false`。

这不能证明文档中的运动测试没有做过，但当前指向的材料无法支撑该结论。建议区分“操作者反馈已完成”“可复核的测量记录”“旧模拟／只读基线”，为运动验收补充实际证据索引、日期与版本。首次检查建议保留早期证据；用户随后要求删除，处理结果见第 10 节。早期模拟与只读记录不能替代运动验收记录。

### D6．本地计划被忽略，且存在过时行为描述【中优先级，已确认】

`.gitignore:32` 的 `docs/` 规则忽略未跟踪的文档产物；本地有两份 2026-11-05 手部计划：

- `docs/superpowers/plans/2026-11-05-hand-force-control.md` 记录的早期默认参数、限时和退出行为已与当前代码及 README 不同。
- `docs/superpowers/plans/2026-11-05-hand-exit-reset.md` 以启动快照作为退出复位目标描述主流程，当前右手使用固定 `RIGHT_RESET_RAD`，仅左手使用启动位置。

它们应作为历史实施材料，不能作为现行使用说明。若需共享，建议选择性纳入并标注历史状态；不建议直接解除整个 `docs/` 忽略规则，把本机输出一并加入 Git。

注意：已有跟踪的 `native_bridge/docs/BASE_IMAGE.md` 等文件仍会正常记录变更；不能把忽略规则误解为这些文件已经退出版本控制。本报告放在根目录，确保可被 Git 正常识别。

## 5. 建议保留的结构与资料

| 对象 | 保留理由 |
|---|---|
| 19 个 MoveJ、19 个 ServoJ 单关节命令入口 | MoveJ 中 18 个、ServoJ 中 19 个已是约 8 行薄入口；方便固定选择关节，继续去重收益很小 |
| 延迟探针入口与分析工具 | 探针已复用 `servoj_single_joint.main(..., latency_probe=True)`；抓包、Pico 日志、线缆分析各有独立测点 |
| ROS1 与 ROS2 的 `package.xml`／CMakeLists | 分别服务 catkin 与 ament 构建；Dockerfile 已复用 `src/upperlimb`、`src/hand` 的字段定义，没有第二份 WA1 消息字段需要删除 |
| `native_bridge/docs/archive/base_image/` | 历史基础镜像材料，已与正式增量 Dockerfile 区分 |
| 早期 evidence 基线 | 首次检查建议保留不同阶段记录；随后已按用户要求删除，见第 10 节 |
| PICO4 `vendor/`、遥操作 `reference/` | SDK 和供应商对照材料，是否保留涉及构建与来源，不宜按冗余直接删掉 |
| 两个网页查看器 | 一个显示相机图像，一个显示 PICO4 位姿；当前没有足够依据为它们抽取统一框架 |

## 6. 本机产物与仓库体积

本次 `du -sh` 快照：`build/` 492 MB、`install/` 324 MB、`.venv/` 156 MB、`log/` 648 KB；均是已忽略的本地目录。它们占空间，但不等于版本库源码冗余。尤其不能在代码整理中顺手删除仍在使用的安装环境。

`src/pico4_ros2/vendor/` 约 93 MB，`src/wa1_description/meshes/` 约 36 MB；属于依赖和模型资源，需结合实际引用与分发方式单独评估。本次没有对 STL 几何等价性或 SDK 内部内容做去重判断。

## 7. 后续整理顺序与验收边界

以下是建议批次，本轮尚未实施。

| 批次 | 内容 | 完成时应提供的验证 |
|---|---|---|
| 1．文档校正 | D1、D2、D3、D5；补文档职责与历史标识 | 相对链接、命令路径静态核对；验收声明逐项对应来源 |
| 2．笛卡尔代码收拢 | C1；保留四个原入口和结果字段 | 离线验证左右侧端点、掩码、目标填充、预览无发送及异常收尾；PC 隔离模拟验证服务／消息行为 |
| 3．关节元数据收拢 | C2、C3；明确左肩默认值兼容 | 覆盖 19 轴顺序、单位、范围、绝对／相对目标差异和 CLI 兼容 |
| 4．历史资料治理 | D6；评估资料索引和归档 | 核对文档、脚本与构建引用 |

不建议把所有控制逻辑合成一个通用大脚本，也不建议仅因 `wa1_teleop.cpp` 达到 956 行就认定它冗余。应先消除已确认的复制实现，再根据职责与测试边界决定是否拆分。

代码整理后可先做离线和 PC 隔离模拟验证；这些结果不能替代真机运动行为的现场确认。本次报告未执行上述后续验证。

## 8. 按报告实施的整理结果（2026-11-09）

本次实施使用 `refactor/redundancy-cleanup-20261109` 分支；以下记录各轮整理过程，提交状态以 Git 记录为准。

| 报告项 | 本次处理 |
|---|---|
| C1 | 新增 `cartesian_position/movel_single_arm.py`、`cartesian_servo/servol_single_arm.py`；四个原脚本改为固定侧别的薄入口，保留端点、请求数组语义、结果字段及默认输出目录 |
| C2 | 左肩入口接入共享 MoveJ；保留默认 `0.1 rad`、原有 JSON 字段及执行参数 |
| C3 | 新增 `native_bridge/tests/joint_config.py`，统一 MoveJ、ServoJ 和笛卡尔测试使用的关节分组；MoveJ/ServoJ 统一引用原值不变的 19 轴测试限位 |
| C5 | 手部的硬件范围、保守测试范围和退出策略保持区分 |
| D1、D2 | 修正不存在的 `jetson/` 描述；四类运动测试及延迟测量文档统一从仓库根目录执行，移除本机克隆路径依赖 |
| D3、D4 | 旧整理计划明确标为历史并补现状对照；环境约定、桥参数、镜像资料按既有主文档链接，保留模块最短启动命令 |
| D5 | 新增 `native_bridge/evidence/README.md`，区分模拟、只读、无指令握手基线；桥 README 明确运动验收尚缺可复核记录 |
| D6 | 两份本地旧手部计划增加历史提示，继续保持被忽略状态；没有整体解除 `docs/` 忽略规则 |

受影响的操作者实现从 2,875 行收拢为 1,880 行，净减少 995 行，统计包括新增公共模块，不包括新增回归测试。没有删除历史证据、SDK、模型、参考材料或本机构建环境；机器人地址和手部控制参数未调整。

验证结果：

- `python3 native_bridge/tests/run_mock_test.py --seconds 3`：PC 回环 ROS1 Master、DDS domain 83 的完整桥模拟通过；输出在 `native_bridge/results/mock_control_results.json`。
- 在同一隔离配置下，通过真实 ROS1/ROS2 模拟链路分别运行左右臂 MoveL、左右臂 ServoL、左肩 MoveJ 共 5 个原入口的预览：均通过；ROS1 模拟器记录 0 条控制命令、0 次服务调用。未绕过入口对真机执行 domain 的限制。
- 43 个原入口从其他当前目录运行 `--help` 通过；文档内 77 处仓库相对命令路径和全部标准内联相对链接检查通过。
- 原关节分组、掩码、限位与基线 AST 对照一致；Python 语法与 `git diff --check` 通过。
- 独立审查未发现 Critical、Important 或 Minor 缺陷。

真实 ROS 链路验证了当时的预览入口，完整桥模拟验证了既有转换与控制路径。没有重新构建桥镜像，也未运行真机运动测试。真机运动记录补齐、控制器实际限位和资源再分发许可仍需外部资料或现场确认。

## 9. 通用单关节接口调整（按用户追加要求）

`position_control/` 和 `position_servo/` 各自根目录下的 19 个 `manual_<joint>.py` 已全部移除，共删除 38 个包装文件。第 5 节、第 8 节中保留这些入口和左肩特殊默认值的决定不再适用；原记录仅保留用于追溯。

当前两个入口均用 `--joint-name` 选择 19 轴之一，`--help` 列出完整可选值：

```bash
python3 native_bridge/tests/position_control/movej_single_joint.py \
  --joint-name Neck_Y --target 0.05
python3 native_bridge/tests/position_servo/servoj_single_joint.py \
  --joint-name Wrist_Z_L --duration 1 --delta 0.01
```

MoveJ 所有关节均须显式给出绝对 `--target`，左肩也不再提供默认目标或专用输出字段。ServoJ 保留相对 `--delta`、运动时长和按关节命名的结果目录。两者仍默认预览，只有追加 `--execute` 才发送控制请求；其余反馈检查、限位、部位映射和收尾逻辑保留。

`position_servo/latency/manual_wrist_z_l_latency.py` 是独立的延迟诊断入口，仍复用通用 ServoJ 实现；本轮没有删除它或复制出新的关节控制实现。

文档中的当前示例已迁移到通用入口。本轮未运行真机控制。

## 10. 早期证据材料清理（按用户追加要求）

已删除整个 `native_bridge/evidence/` 目录：9 份早期转换、接口、模拟、只读反馈和无指令握手记录，以及第一轮整理时新增的证据索引，共 10 个文件。第 8 节记录的“保留历史证据”和“新增证据索引”已由本次决定替代。

桥 README 已移除旧索引链接及依赖这些历史材料的验收表述，历史整理计划标注了删除状态，`.dockerignore` 已清除失效目录规则。

当前测试脚本、合成输入及 `results/` 本机输出保留。模拟器容器中的 `/evidence` 仍是运行时结果挂载点，映射到当前 `native_bridge/results/`，不依赖被删除的宿主目录，因而无需更改模拟链路。

本轮仅删除历史材料并修改说明，未修改运行代码或执行真机操作；检查确认目录已不存在、文档相对链接有效且 `git diff --check` 通过。

## 11. 通用单臂笛卡尔接口调整（按用户追加要求）

已删除 `cartesian_position/`、`cartesian_servo/` 中左右臂的 4 个包装文件；两个目录分别仅保留 `movel_single_arm.py` 和 `servol_single_arm.py`。此前关于保留四个原入口的决定由本次要求替代。

两个脚本均可直接执行，通过必填的 `--side left|right` 选择手臂；MoveL 仍使用 `--delta` 或 `--absolute`，ServoL 仍使用 `--duration` 和 `--delta`。默认预览，只有 `--execute` 才发送控制；MoveL 的 `--check-only` 保留。请求中的目标、另一臂实时位姿、端点、掩码和输出字段的侧别语义保持一致，ServoL 默认输出仍分别位于 `results/Left_TCP/`、`results/Right_TCP/`。

当前模块 README 和桥 README 已同步迁移。没有运行真机控制。


## 12. 整理文档归档位置调整

按用户要求删除根目录旧 `CODE_ORGANIZATION_PLAN.md`；现行计划位于 [docs/CODE_ORGANIZATION_PLAN.md](CODE_ORGANIZATION_PLAN.md)，本报告也由根目录移入 `docs/`。根 README 已更新导航，Git 忽略规则仅为这两份正式文档开放，不引入本地过程材料。

## 13. 操作者依赖模拟测试清理

按用户要求删除操作者依赖模拟测试目录及相关文档入口、运行命令和历史测试数量说明。现有运动控制脚本、桥模拟回归、延迟分析工具及其他软件包原有测试保留。
