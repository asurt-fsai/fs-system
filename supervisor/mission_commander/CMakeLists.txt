cmake_minimum_required(VERSION 3.0.2)
project(mission_commander)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             sensor_msgs
             std_msgs
             asurt_msgs
             ros_numpy
             tf_helper
             eufs_msgs)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  mission_commander
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  asurt_msgs
  tf_helper
  ros_numpy
  eufs_msgs
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})


install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS
nodes/mission_commander_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install launch files used for tests
install (DIRECTORY  testing/launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
