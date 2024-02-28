cmake_minimum_required(VERSION 3.5)
project(opencv_cam_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)

add_executable(talker main.cpp)
ament_target_dependencies(talker rclcpp std_msgs opencv_calib3d opencv_core opencv_highgui opencv_imgproc)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
