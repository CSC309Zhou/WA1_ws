cmake_minimum_required(VERSION 3.8)
project(pico4_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

set(PICO4_SDK_DEB "${CMAKE_CURRENT_SOURCE_DIR}/vendor/XRoboToolkit-PC-Service_1.0.0.0_amd64.deb"
  CACHE FILEPATH "Path to the locally supplied XRoboToolkit PC Service deb")
set(VENDOR_ROOT "${CMAKE_CURRENT_BINARY_DIR}/vendor_root")
set(VENDOR_SDK "${VENDOR_ROOT}/opt/apps/roboticsservice/SDK")
if(NOT EXISTS "${PICO4_SDK_DEB}")
  message(FATAL_ERROR "Missing XRoboToolkit delivery: ${PICO4_SDK_DEB}; set PICO4_SDK_DEB to a local .deb")
endif()
execute_process(COMMAND dpkg-deb -x "${PICO4_SDK_DEB}" "${VENDOR_ROOT}"
  RESULT_VARIABLE VENDOR_EXTRACT_RESULT)
if(NOT VENDOR_EXTRACT_RESULT EQUAL 0 OR NOT EXISTS "${VENDOR_SDK}/include/PXREARobotSDK.h")
  message(FATAL_ERROR "Could not extract the local XRoboToolkit delivery")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XrPose.msg"
  "msg/XrController.msg"
  "msg/XrFrame.msg"
  DEPENDENCIES std_msgs)

add_library(xr_json_parser src/xr_json_parser.cpp)
target_include_directories(xr_json_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xr_json_parser PkgConfig::JSONCPP)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(xr_json_parser "${cpp_typesupport_target}")

add_executable(xr_receiver src/xr_receiver.cpp)
target_include_directories(xr_receiver PRIVATE "${VENDOR_SDK}/include")
ament_target_dependencies(xr_receiver rclcpp ament_index_cpp std_msgs)
target_link_libraries(xr_receiver xr_json_parser "${cpp_typesupport_target}" dl)

install(TARGETS xr_json_parser xr_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pose_viewer.py DESTINATION lib/${PROJECT_NAME} RENAME pose_viewer)
install(FILES scripts/viewer_math.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${VENDOR_ROOT}/opt/apps/roboticsservice/"
  DESTINATION share/${PROJECT_NAME}/vendor/roboticsservice
  USE_SOURCE_PERMISSIONS
  PATTERN "SDKDemo" EXCLUDE
  PATTERN "RobotDataRecorder" EXCLUDE
  PATTERN "RobotDemoQt" EXCLUDE
  PATTERN "ConsoleDemo" EXCLUDE)
install(DIRECTORY launch test_data web config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES vendor/original_Custom.msg vendor/original_Controller.msg vendor/original_Head.msg
  DESTINATION share/${PROJECT_NAME}/vendor/reference)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_xr_json_parser test/test_xr_json_parser.cpp)
  if(TARGET test_xr_json_parser)
    target_link_libraries(test_xr_json_parser xr_json_parser "${cpp_typesupport_target}")
  endif()
  ament_add_pytest_test(test_viewer_math test/test_viewer_math.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
