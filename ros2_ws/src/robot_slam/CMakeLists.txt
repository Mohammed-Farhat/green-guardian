cmake_minimum_required(VERSION 3.8)
project(robot_slam)

find_package(ament_cmake REQUIRED)

# Install the launch directory so ros2 launch can find it
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
