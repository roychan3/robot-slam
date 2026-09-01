cmake_minimum_required(VERSION 3.8)
project(robot_slam)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/loop_closure_test.py
  scripts/save_pointcloud.py
  scripts/smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
