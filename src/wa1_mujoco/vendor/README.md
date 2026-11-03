# 官方离线运动学组件

`include/pico_runtime/uplimb/UplimbController.h`、`lib/libuplimb_library.so.1.2.4.0`、`model/` 中的 URDF 与 YAML 均来自 WA1 小脑 PICO 的 `/opt/ros/noetic` 安装。`lib/` 中其余共享库是该上肢库在 PC Ubuntu 22.04 上运行所需的依赖；其中 `libtinyxml.so.2.6.2` 取自本机 Ubuntu 22.04，其余保留 PICO 上的版本。头显 PICO4 的 SDK 与这些文件无关。

运行时 Python 封装在临时目录生成一份配置，把原 YAML 中的 PICO 绝对路径换为已安装软件包的路径；原配置文件与官方 URDF 保持原样。库只调用 `init`、`getForwardKinematics` 和 `getInverseKinematics`。没有控制器、ROS Master 或机器人设备连接。

关键文件 SHA-256：

```text
7ff108a04f6f8fa0e5cc6dbf5b5eefb8fcf3e2e70aa1cd67a409310c950e2a75  lib/libuplimb_library.so.1.2.4.0
70da5d1451d3fe6ae04d16dec73ced19a70c6cadf079e6a31f45601b8f3c6118  model/WA1_400L.urdf
5c1b09e5b7449bc60ba55919a3dbd1f36cf3e1c154bd896419c8c6f2cb717423  model/robot_define_WA1.yaml
```
