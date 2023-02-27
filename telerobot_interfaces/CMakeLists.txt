cmake_minimum_required(VERSION 3.8)
project(telerobot_interfaces)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Motor.msg" "msg/Battery.msg")

ament_package()
