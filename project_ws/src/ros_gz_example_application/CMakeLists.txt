cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  identify_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  sound.wav
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
