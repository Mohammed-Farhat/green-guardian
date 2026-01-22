cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Default to C++17 if needed by tools (not required here, but harmless)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# Install resource folders
install(
  DIRECTORY
    urdf
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
