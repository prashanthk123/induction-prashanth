cmake_minimum_required(VERSION 3.8)
project(coordinate_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CoordinateTraverse.action"
)
ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/coordinate_follower_server.py
  scripts/coordinate_follower_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
