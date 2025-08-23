cmake_minimum_required(VERSION 3.8)
project(control_command)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)  # 添加这个依赖
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# 修复文件名匹配问题
add_executable(robot_control_node
  src/robot_controller_node.cpp 
  src/insert_strategy.cpp
)

ament_target_dependencies(robot_control_node
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs 
  tf2_geometry_msgs
)

# 安装可执行文件
install(TARGETS
  robot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py" PATTERN "*.launch.py"
)

ament_package()
