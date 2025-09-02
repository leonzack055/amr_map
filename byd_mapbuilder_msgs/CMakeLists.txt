cmake_minimum_required(VERSION 3.5)
project(byd_mapbuilder_msgs)

# Default to C++14.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# 声明消息文件
set(msg_files
  "msg/MapBuildStatus.msg"
  "msg/MapBuildRequest.msg"
  "msg/MapBuildProcess.msg"
)

set(srv_files
  "srv/MapBuild.srv"
)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()