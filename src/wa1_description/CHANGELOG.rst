^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wa1_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-08-06)
------------------
* 重构包结构，将 WA1_400L 整理为小写 wa1_description，并迁移 meshes、urdf、launch、config 等模型资源。
* 增加 ROS1/ROS2 双版本构建支持，使用 package.xml 条件依赖和 CMakeLists.txt 分支区分 catkin 与 ament_cmake。
* 增加 ROS2 Python launch 文件，保留并更新 ROS1 display、gazebo、realdisplay launch 文件。
* 新增 realtime_joint_state_merger.py，并统一 upperlimb 和 hand 相关节点映射。
* 新增 bbox_collision.model.yaml，并将碰撞模型中的 URDF 路径整理为相对路径。
* 增加 Jenkins ROS_VERSION 镜像选择和仓库 makefile 构建入口，支持 build、deb、clean 目标。
* [历史CAHNGELOG] 整理模型包（2026-07-28）
* [历史CAHNGELOG] 以算法配置文件为标准核对限位（2026-07-28）
* [历史CAHNGELOG] 检验模型包是否正常加载、显示（2026-07-28）
* [历史CAHNGELOG] 加入版本号（版本分别代表着：机械变更、上肢运控变更、中间件变更）（2026-07-28）
* [历史CAHNGELOG] 一个包裹中如果有多个型号，则以量产版为标准更新版本号，如 WA2 的联产机型为 WA2_LS（2026-07-28）
