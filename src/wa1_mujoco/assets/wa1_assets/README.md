# wa1_assets

WA1 机器人模型资产仓库。模型文件由 Git LFS 管理。

## 拉取模型

首次使用前安装并初始化 Git LFS：

```bash
sudo apt update
sudo apt install git-lfs
git lfs install
```

克隆仓库并下载实际网格文件：

```bash
git clone https://github.com/kai698/wa1_assets.git
cd wa1_assets
git lfs pull
```

验证 LFS 文件：

```bash
git lfs ls-files
git check-attr filter -- WA1_400L/meshes/BASE.STL
```

`git lfs ls-files` 应列出 `WA1_400L/meshes/` 下的模型，属性检查应返回 `filter: lfs`。
如果模型文件仍以 `version https://git-lfs.github.com/spec/v1` 开头，说明 LFS 对象尚未下载成功。

## 生成配置和 MJCF

项目环境统一由 `uv` 管理，使用 Python 3.11、PyYAML 和 MuJoCo。首次创建并激活环境：

```bash
uv venv --python 3.11 .venv
source .venv/bin/activate
uv pip install PyYAML mujoco
```

激活后使用 `uv` 运行转换入口：

```bash
uv run --active main.py WA1_400L --override
uv run --active main.py WA1_400L --convert
```

第一条命令从 `WA1_400L.urdf` 生成 `WA1_400L/robot_cfg.yaml`；已有配置默认会被复用，只有传入 `--override` 才会重新生成。第二条命令生成 `WA1_400L/WA1_400L.xml` 和配套的 `scene.xml`，生成的 MJCF 可直接由 MuJoCo 加载。

## WA1_400L 的 MuJoCo 装配

当前转换脚本将 `BASE` 固定在 MuJoCo 世界坐标系中；其世界位置由 `robot_cfg.yaml` 的 `init_state.root_pos` 指定。模型不再包含底座自由关节。

`WA1_400L/meshes/endoscope.STL` 来自同级 `器械模型/endoscope/meshes/endoscope.STL`。转换时，器械的 `endoscope` 坐标系作为 `TCP_R` 的固定子坐标系；当前安装原点与 `TCP_R` 重合，朝向相对 `TCP_R` 绕 `+Y` 轴旋转 90°。`endoscope_tcp` site 位于器械局部坐标系的 `(0, 0, -0.4)` 米处。器械的质量与惯量采用原 `endoscope.urdf` 中的数据。

双臂、腰部、脖子和升降的 19 个关节使用 MuJoCo 位置伺服，`ctrl` 为关节目标位置（转动关节单位为弧度，升降单位为米）。两个驱动轮仍使用电机模式。伺服增益在 `default_cfg.yaml` 的 `mujoco.position_servo_gains` 中设置，输出力受 `robot_cfg.yaml` 中的 `Tm` 限制。导入的 CAD 碰撞网格在机器人内部有重叠，因此机器人网格间不生成接触；机器人与地面仍可接触。

场景重力保持 `(0, 0, -9.81)` m/s²；`mujoco.robot_gravcomp.WA1_400L: 1` 使 `BASE` 以下的机器人刚体（包括内窥镜）逐个补偿自身重量。场景中的其他物体仍受重力。

`home` 关键帧保存初始关节位置和对应的伺服目标：左臂 `[0, 0.4360, 0.5234, 0, 0, 0, 0]`，右臂 `[0, -0.4360, -0.5234, 0, 0, 0, 0]`，腰部、脖子和升降为零。臂部顺序为 `Shoulder_Y, Shoulder_X, Shoulder_Z, Elbow, Wrist_Z, Wrist_Y, Wrist_X`。

按 `wa1_mujoco` 软件包根目录 README 加载 Python 与 ROS 2 环境后，可在本目录重新生成并从 `home` 姿态打开带地面的场景：

```bash
python main.py WA1_400L --convert
python launch_wa1.py
```

直接使用 `python -m mujoco.viewer --mjcf=WA1_400L/scene.xml` 会以 MuJoCo 的零位启动；若使用此命令，可在查看器中加载 `home` 关键帧。
