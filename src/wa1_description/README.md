# WA1 机器人描述包

`WA1_400L.urdf`、`meshes/` 和 `launch/display.launch.py` 提供无需额外手部描述包的基础可视化。`urdf/assembly.xacro`、`launch/robot_description.launch.py` 与 `config/options.yaml` 可组合可选末端工具；启用这些装配入口前，须自行提供 `optional_description`、`f5d6_l_description`、`f5d6_p_description` 中实际用到的包。

这些可选手部包不属于桥接和双臂遥操作的编译依赖，因此不再作为本包的无条件依赖声明。原始资源的许可字段仍为供应商给出的 `TODO`，公开发布前需要确认授权。
