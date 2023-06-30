cmake_minimum_required(VERSION 3.0.2)
project(simple_pure_pursuit)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             ros_numpy
             ackermann_msgs
             std_msgs
             asurt_msgs
             geometry_msgs
             nav_msgs
             tf_helper)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES

  CATKIN_DEPENDS
  rospy
  ros_numpy
  ackermann_msgs
  std_msgs
  asurt_msgs
  geometry_msgs
  nav_msgs
  tf_helper
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/simple_pp_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
