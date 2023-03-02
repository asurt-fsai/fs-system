cmake_minimum_required(VERSION 3.0.2)
project(moreo)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             roscpp
             rospy
             std_msgs
             geometry_msgs
             asurt_msgs)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  mrpython_pcl
  CATKIN_DEPENDS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  asurt_msgs
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/moreo_system.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
