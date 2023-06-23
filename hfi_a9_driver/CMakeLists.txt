cmake_minimum_required(VERSION 3.8)
project(hfi_a9_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(telerobot_interfaces REQUIRED)

include_directories("include/io")
include_directories("include/containers")

add_executable(${PROJECT_NAME} src/main.cpp include/io/SerialConnector.hpp src/io/SerialConnector.cpp include/containers/CircularBuffer.hpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs geometry_msgs telerobot_interfaces)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
