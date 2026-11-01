cmake_minimum_required(VERSION 3.8)
project(agribot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY urdf config worlds launch rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/sim_cut_model.py
  scripts/film_director.py
  scripts/capture_frames.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_agribot_gazebo test)
endif()

ament_package()
