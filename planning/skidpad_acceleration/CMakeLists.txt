cmake_minimum_required(VERSION 3.0.2)
project(skidpad_acceleration)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             std_msgs
             ros_numpy
             geometry_msgs
             nav_msgs)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  skidpad_acceleration
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  ros_numpy
  geometry_msgs
  nav_msgs
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/acceleration.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
