# WA1 MuJoCo 离线仿真

本包将同级实验目录 `WA1_mujoco/wa1_assets` 中的模型、生成器、网格和右臂内窥镜装配整理到 ROS 2 工作空间，并把官方上肢 FK/IK 库封装为本机调用接口。运行仿真和运动学不需要 Jetson、PICO、ROS Master、桥容器或机器人网络。ROS 2 在这里只用于定位和启动软件包，脚本不创建 ROS 节点。

## 模型与坐标

- `assets/wa1_assets/WA1_400L/scene.xml` 是仿真入口；`WA1_400L.xml` 是含惯量、碰撞网格和执行器的机器人模型。`BASE` 固定在世界坐标 `(0, 0, 0.2966)` m。
- 场景重力保持 `(0, 0, -9.81)` m/s²。机器人 33 个刚体（包括内窥镜）设为 `gravcomp=1`，按理想重力补偿处理；场景中额外加入的物体仍受重力。生成配置位于 `assets/wa1_assets/default_cfg.yaml` 的 `mujoco.robot_gravcomp`。
- 19 个位置执行器覆盖双臂 14 轴、颈部 2 轴、腰部 2 轴、升降 1 轴。另有两个轮电机。`home` 关键帧保存各关节初值与对应位置控制目标。
- 内窥镜刚性安装于 `TCP_R`，安装原点与 `TCP_R` 重合，当前绕其 `+Y` 轴旋转 90°；器械尖端 `endoscope_tcp` 在内窥镜局部 `-Z` 方向 0.4 m。器械的质量、惯量来自原始内窥镜 URDF。
- **离线 IK 的目标始终是双臂原有的 `TCP_L`、`TCP_R` 相对 `BASE` 的位姿。** `endoscope_tcp` 仅用于可视化和将来单独开发器械运动规划。

公共位姿格式是 `[x, y, z, qx, qy, qz, qw]`，位置单位 m。官方 C++ 库内部的六维位姿顺序为 `xyz + yaw,pitch,roll`（ZYX），封装层负责四元数转换。FK 输入以关节名组织，内部转换为官方顺序：左臂 7、右臂 7、`Neck_Z`、`Neck_Y`、`Lifting_Z`、`Waist_Z`、`Waist_Y`。

官方 IK 接口固定单臂第七轴 `Wrist_X`，可能返回零、一或多组候选解。`solve()` 返回全部候选和臂角 `phi_rad`；`nearest()` 按与参考关节角的距离选择一组。腰部或升降不为零时，封装层先用 MuJoCo 中 `BASE → WAIST_PITCH` 的变换把目标换算到官方 IK 的零躯干条件，再解算。官方 FK 使用当前 19 轴状态直接计算。

## 目录

| 路径 | 作用 |
|---|---|
| `assets/wa1_assets/` | 从独立实验目录复制的 URDF、MJCF、场景、33 个网格及生成脚本 |
| `vendor/model/` | 官方运动学使用的 URDF 与配置 |
| `vendor/include/`、`vendor/lib/` | 官方上肢库头文件及本机运行所需的共享库 |
| `src/official_kinematics.cpp` | C ABI：初始化、双臂 FK、固定第七轴的 IK |
| `python/wa1_mujoco/kinematics.py` | MuJoCo 状态与官方运动学间的关节、位姿和躯干变换 |
| `scripts/check_kinematics` | 离线交叉检查 FK、IK、内窥镜装配和 2 秒动力学运行 |
| `scripts/launch_sim` | 交互查看器、无界面仿真及双臂 IK 伺服示例 |

Git LFS 保存 STL 网格和随包提供的二进制库。克隆后先运行 `git lfs pull`，否则 MuJoCo 无法加载网格，运动学库也无法链接。包面向 Ubuntu 22.04 `x86_64` 和 ROS 2 Humble。

## 安装、编译与启动

在工作空间根目录执行。Python 虚拟环境使用系统 ROS 2 包，并安装 MuJoCo、NumPy、PyYAML 与 `typing_extensions`；已具备这些依赖的虚拟环境也可直接使用。

```bash
cd ~/WA1/WA1_ws
sudo apt install libeigen3-dev python3-venv
git lfs pull
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r src/wa1_mujoco/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select wa1_mujoco
source install/setup.bash
ros2 run wa1_mujoco check_kinematics
ros2 run wa1_mujoco launch_sim
```

`launch_sim` 打开 MuJoCo 查看器并加载 `home`；按 `Ctrl+C` 结束。仅在终端运行 2 秒仿真：

```bash
ros2 run wa1_mujoco launch_sim --headless --seconds 2
```

双臂 IK 示例先将仿真关节置为遥操作初始姿态，再将两个原始 TCP 的目标沿 `BASE +X` 平移 5 mm；求解后把关节目标写入 MuJoCo 位置执行器：

```bash
ros2 run wa1_mujoco launch_sim --ik-demo
ros2 run wa1_mujoco launch_sim --ik-demo --headless --seconds 2
```

这个示例的动力学跟踪误差与静态 IK 解算残差是两件事。启用理想重力补偿后，本机 2 秒运行的左右 TCP 动态位置误差约为 0.00246 mm、0.00243 mm；静态 IK 解算后放入 MuJoCo 的位姿残差约为 1 µm。现有 MuJoCo 执行器是资产中的位置伺服模型，未复刻机器人底层力矩控制器；`body gravcomp` 产生的补偿力默认计入被动力，暂不受执行器力限约束。

## 在程序中调用

`load_home()` 返回 MuJoCo 模型与数据。`pose_in_base()` 从模型读双臂 TCP；`OfflineKinematics.forward()` 调用本机官方 FK；`solve()` 以 TCP 相对 BASE 的位姿求全部官方 IK 候选。

```python
from wa1_mujoco.kinematics import (OfflineKinematics, joint_positions,
                                  load_home, pose_in_base)

model, data = load_home()
kin = OfflineKinematics(model)
joints = joint_positions(model, data)
tcp_left = pose_in_base(data, 'left')
fk_left = kin.forward(joints, 'left')
solutions = kin.solve(tcp_left, 'left', joints)
chosen = kin.nearest(solutions, joints)
```

源码位于包内 `python/`；`colcon build` 会将其安装为可直接导入的 Python 包。运行独立 Python 程序前，先在该终端激活虚拟环境并加载 ROS 2 与 `install/setup.bash`。

修改导入的 URDF、模型配置或生成脚本后，可在 `assets/wa1_assets/` 下运行 `python main.py WA1_400L --convert` 重新生成 MJCF，随后重新 `colcon build --packages-select wa1_mujoco`。`--convert` 复用现有 `robot_cfg.yaml`；`--override` 会用默认配置覆盖手工修改。官方 IK 使用 `vendor/model/WA1_400L.urdf`，更换运动学 URDF 时须同步核对官方库与 MuJoCo 的 FK 交叉误差。

## 本机验收

`check_kinematics` 已在无机器人连接的 PC 上通过：场景重力为 `-9.81 m/s²`，全部 33 个机器人刚体的 `gravcomp=1`；`home`、遥操作姿态与非零躯干姿态的双臂 FK 和 MuJoCo 相差不超过 `2.5e-7 m`、`4.5e-5°`；左右臂的固定第七轴 IK 在零、非零躯干条件下均有解，回代 MuJoCo 的位置误差约 `9.5e-7 m`、姿态误差约 `0.0002°`。内窥镜安装原点与 `TCP_R` 完全重合，尖端偏移 `0.4 m`；模型持续运行 2 秒状态有限。官方库求解时可能打印某个候选分支关节越界的信息；最终候选是否有效以返回列表和交叉检查为准。

修改 `convert_mjcf.py` 中内窥镜相对 `TCP_R` 的 `pos` 或 `quat` 后，需要在 `assets/wa1_assets/` 运行 `python main.py WA1_400L --convert`，再回到工作空间根目录运行 `colcon build --packages-select wa1_mujoco`。`ros2 run wa1_mujoco launch_sim` 加载的是安装目录中的场景，不会直接读取源码目录。可用 `ros2 run wa1_mujoco check_kinematics --expect-endoscope-quat 0.7071068 0 0.7071068 0` 核对安装后的 +90° Y 轴姿态；输出还会列出器械安装位姿和尖端在 `TCP_R` 下的位置。
