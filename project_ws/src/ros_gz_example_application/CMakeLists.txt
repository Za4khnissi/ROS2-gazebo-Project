cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

file(GLOB PYTHON_FILES *.py)

install(PROGRAMS
  ${PYTHON_FILES}
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

install(FILES
  sound.mp3
  sound.wav
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
