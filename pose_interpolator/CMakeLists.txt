cmake_minimum_required(VERSION 3.8)
project(pose_interpolator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amr_ros_msg REQUIRED)

# 添加生成的头文件目录
include_directories(
  include
  #${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 创建可执行文件
add_executable(pose_interpolator_node
  src/pose_interpolator_node.cpp
)

# 链接依赖项
target_link_libraries(pose_interpolator_node
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  #${PROJECT_NAME}__rosidl_typesupport_cpp  # 使用正确的消息库链接
)

# 添加目标依赖
ament_target_dependencies(pose_interpolator_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
  amr_ros_msg
  
)

# 安装目标
install(TARGETS pose_interpolator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

ament_package()
