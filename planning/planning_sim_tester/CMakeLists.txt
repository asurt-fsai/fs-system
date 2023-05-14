cmake_minimum_required(VERSION 3.0.2)
project(planning_sim_tester)

find_package(catkin REQUIRED)

catkin_package(
  LIBRARIES
  planning_sim_tester
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

# catkin_install_python(PROGRAMS nodes/lidar.py DESTINATION
#                       ${CATKIN_PACKAGE_BIN_DESTINATION})
