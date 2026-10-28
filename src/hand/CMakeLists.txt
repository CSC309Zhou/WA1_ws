cmake_minimum_required(VERSION 3.8)
project(hand)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PressureSensor.msg"
  "srv/HandJoint.srv"
  "srv/Gesture.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
install(FILES ros1_mapping_rules.yaml DESTINATION share/${PROJECT_NAME})
ament_package()
