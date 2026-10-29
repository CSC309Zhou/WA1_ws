cmake_minimum_required(VERSION 3.8)
project(wa1_description)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/live_joint_states.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description_resources
    test/test_description_resources.py
  )
endif()

ament_package()
