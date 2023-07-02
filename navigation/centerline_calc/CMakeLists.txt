cmake_minimum_required(VERSION 3.0.2)
project(centerline_calc)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs ros_numpy geometry_msgs nav_msgs visualization_msgs)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  centerline_calc
  CATKIN_DEPENDS
  rospy
  std_msgs
  ros_numpy
  geometry_msgs
  nav_msgs
  visualization_msgs
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
