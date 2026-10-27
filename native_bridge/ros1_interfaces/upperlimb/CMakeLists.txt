cmake_minimum_required(VERSION 3.0.2)
project(upperlimb)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Joints.msg
  DualPose.msg
  SpeedJ.msg
  SpeedL.msg
  Pose.msg
  TcpSpeed.msg
  UplimbState.msg
  ArmTypeEnum.msg
)

add_service_files(
  FILES
  Servo.srv
  MoveJ.srv
  MoveL.srv
  MoveJByPath.srv
  MoveLByPath.srv
  MoveJByPose.srv
  IK.srv
  FK.srv
  IsSingular.srv
  ArmType.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
