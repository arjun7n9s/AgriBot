cmake_minimum_required(VERSION 3.8)
project(agribot_field)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models config
  DESTINATION share/${PROJECT_NAME})

# Put the models on Gazebo's resource path so model://tomato_ripe etc. resolve.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
