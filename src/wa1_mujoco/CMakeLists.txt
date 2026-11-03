cmake_minimum_required(VERSION 3.16)
project(wa1_mujoco LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
ament_python_install_package(wa1_mujoco PACKAGE_DIR python/wa1_mujoco)

add_library(wa1_official_kinematics SHARED src/official_kinematics.cpp)
target_compile_features(wa1_official_kinematics PRIVATE cxx_std_17)
target_include_directories(wa1_official_kinematics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(wa1_official_kinematics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib/libuplimb_library.so.1.2.4.0)
target_link_options(wa1_official_kinematics PRIVATE
  "LINKER:-rpath-link,${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib"
  "LINKER:--disable-new-dtags")
set_target_properties(wa1_official_kinematics PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib"
  INSTALL_RPATH "$ORIGIN")

install(TARGETS wa1_official_kinematics LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY vendor/lib/ DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY assets/ DESTINATION share/${PROJECT_NAME}/assets
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY vendor/model/ DESTINATION share/${PROJECT_NAME}/vendor/model)
install(PROGRAMS scripts/launch_sim scripts/check_kinematics
  DESTINATION lib/${PROJECT_NAME})
ament_package()
