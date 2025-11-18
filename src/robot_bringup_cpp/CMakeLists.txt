cmake_minimum_required(VERSION 3.8)
project(robot_bringup_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

# Add executable files
add_executable(serial_bridge_node src/serial_bridge_node.cpp)
ament_target_dependencies(serial_bridge_node
  rclcpp
  geometry_msgs
)

add_executable(odom_publisher_node_cpp src/odom_publisher_node.cpp)
ament_target_dependencies(odom_publisher_node_cpp
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(imu_publisher_node src/imu_publisher_node.cpp)
ament_target_dependencies(imu_publisher_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

# Install executable files
install(TARGETS
  serial_bridge_node
  odom_publisher_node_cpp
  imu_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python helper scripts (nodes)
install(PROGRAMS
  scripts/particle_cloud_to_pose_array.py
  scripts/patrol_manager.py
  scripts/waypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(robot_bringup_cpp_rqt
  PACKAGE_DIR python/robot_bringup_cpp_rqt
)

pluginlib_export_plugin_description_file(rqt_gui rqt/patrol_panel_plugin.xml)
# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install rqt plugin XML
install(FILES
  rqt/patrol_panel_plugin.xml
  DESTINATION share/${PROJECT_NAME}/rqt
)

# Install map files (包含所有地图目录)
install(DIRECTORY
  maps
  maps2
  map3
  DESTINATION share/${PROJECT_NAME}
)

# Install world files (optional, for simulation only)
# install(DIRECTORY
#   world
#   DESTINATION share/${PROJECT_NAME}
# )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PatrolStatus.msg"
  "srv/SetWaypoints.srv"
  DEPENDENCIES geometry_msgs
  SKIP_GROUP_MEMBERSHIP_CHECK
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
