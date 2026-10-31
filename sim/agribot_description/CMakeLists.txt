cmake_minimum_required(VERSION 3.8)
project(agribot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME})

# Let Gazebo resolve package://agribot_description/... mesh URIs.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
