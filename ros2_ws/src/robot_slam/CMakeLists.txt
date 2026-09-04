cmake_minimum_required(VERSION 3.8)
project(robot_slam)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(rbpf_slam_node src/rbpf_slam_node.cpp)
target_compile_features(rbpf_slam_node PUBLIC cxx_std_17)
ament_target_dependencies(
  rbpf_slam_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(ekf_slam_node src/ekf_slam_node.cpp)
target_compile_features(ekf_slam_node PUBLIC cxx_std_17)
ament_target_dependencies(
  ekf_slam_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

install(PROGRAMS
  scripts/accuracy_monitor.py
  scripts/loop_closure_test.py
  scripts/robot_control.py
  scripts/save_pointcloud.py
  scripts/smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ekf_slam_node rbpf_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
