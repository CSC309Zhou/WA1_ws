# 历史基础镜像材料

本目录只用于保存开发期间的基础镜像尝试，不参与现在的增量构建。`Dockerfile.humble` 未在全新 PC 上验证；它引用迁移前的 `native_bridge/base_image/ros1/` 路径，并克隆未固定版本的上游源码，不能把它当成已冻结本机镜像的准确构建配方或正式发布入口。

正式增量 Dockerfile 是 [`../../../Dockerfile`](../../../Dockerfile)。其中仍需要的 ROS1 包构建元数据已移到 [`../../../ros1_interfaces/`](../../../ros1_interfaces/)。基础镜像的发布状态和依赖契约见 [`../../BASE_IMAGE.md`](../../BASE_IMAGE.md)。
