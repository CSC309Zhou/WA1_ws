cmake_minimum_required(VERSION 3.8)
project(upperlimb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joints.msg"
  "msg/DualPose.msg"
  "msg/SpeedJ.msg"
  "msg/SpeedL.msg"
  "msg/Pose.msg"
  "msg/TcpSpeed.msg"
  "msg/UplimbState.msg"
  "msg/ArmTypeEnum.msg"
  "srv/Servo.srv"
  "srv/MoveJ.srv"
  "srv/MoveL.srv"
  "srv/MoveJByPath.srv"
  "srv/MoveLByPath.srv"
  "srv/MoveJByPose.srv"
  "srv/IK.srv"
  "srv/FK.srv"
  "srv/IsSingular.srv"
  "srv/ArmType.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
install(FILES ros1_mapping_rules.yaml DESTINATION share/${PROJECT_NAME})
ament_package()
