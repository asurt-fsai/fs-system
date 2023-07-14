cmake_minimum_required(VERSION 3.0.2)
project(supervisor)

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
  supervisor
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
nodes/supervisor_node.py
nodes/ros_can_simulator.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install launch files used for tests
install (DIRECTORY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
