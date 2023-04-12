cmake_minimum_required(VERSION 3.0.2)
project(lqr)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs ros_numpy geometry_msgs)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  lqr
  CATKIN_DEPENDS
  rospy
  std_msgs
  ros_numpy
  geometry_msgs
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/lqr_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
