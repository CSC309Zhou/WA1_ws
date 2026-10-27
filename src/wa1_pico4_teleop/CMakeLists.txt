cmake_minimum_required(VERSION 3.8)
project(wa1_pico4_teleop)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pico4_ros2 REQUIRED)
find_package(upperlimb REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

add_library(teleop_math src/teleop_math.cpp)
target_include_directories(teleop_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(wa1_teleop src/wa1_teleop.cpp)
ament_target_dependencies(wa1_teleop rclcpp rcl_interfaces pico4_ros2 upperlimb geometry_msgs
  sensor_msgs std_msgs std_srvs)
target_link_libraries(wa1_teleop teleop_math PkgConfig::JSONCPP)

install(TARGETS teleop_math wa1_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/monitor_status.py DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_monitor)
install(PROGRAMS scripts/preview_direction_check.py DESTINATION lib/${PROJECT_NAME}
  RENAME preview_direction_check)
install(DIRECTORY include/ DESTINATION include)
install(FILES launch/teleop.launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(FILES config/wa1_teleop.yaml DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_teleop_math test/test_teleop_math.cpp)
  if(TARGET test_teleop_math)
    target_link_libraries(test_teleop_math teleop_math)
  endif()
endif()
ament_package()
