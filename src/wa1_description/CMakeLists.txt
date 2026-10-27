cmake_minimum_required(VERSION 3.5)
project(wa1_description)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)

  install(DIRECTORY config launch meshes sensors urdf
    DESTINATION share/${PROJECT_NAME})

  ament_package()
else()
  find_package(catkin REQUIRED)
  catkin_package()

  foreach(dir config launch meshes sensors urdf)
    install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endforeach()

  catkin_install_python(PROGRAMS scripts/realtime_joint_state_merger.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
