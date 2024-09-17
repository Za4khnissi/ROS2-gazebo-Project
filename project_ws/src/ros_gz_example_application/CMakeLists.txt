cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  identify_service.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

install(FILES
  sound.wav
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
