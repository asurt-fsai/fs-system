cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             ros_numpy
             ackermann_msgs
             std_msgs
             asurt_msgs
             geometry_msgs
             nav_msgs)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  pure_pursuit
  CATKIN_DEPENDS
  rospy
  ros_numpy
  ackermann_msgs
  std_msgs
  asurt_msgs
  geometry_msgs
  nav_msgs
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/purepursuit_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
