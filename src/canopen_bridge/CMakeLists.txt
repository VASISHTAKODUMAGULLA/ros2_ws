cmake_minimum_required(VERSION 3.8)
project(canopen_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(canopen_bridge_node
  src/canopen_bridge_node.cpp
  src/canopen_interface.cpp)

# Headers live alongside sources in src/ (flat layout).
target_include_directories(canopen_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_features(canopen_bridge_node PUBLIC cxx_std_17)

ament_target_dependencies(canopen_bridge_node
  rclcpp
  std_msgs
  geometry_msgs)

install(TARGETS canopen_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
