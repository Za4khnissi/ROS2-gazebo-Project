cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(PROGRAMS
  random_movement.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
