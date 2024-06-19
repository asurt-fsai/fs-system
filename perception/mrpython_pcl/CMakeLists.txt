cmake_minimum_required(VERSION 3.0.2)
project(mrpython_pcl)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             std_msgs
             asurt_msgs
             sensor_msgs
             ros_numpy
             tf_helper)

catkin_package(
  LIBRARIES
  mrpython_pcl
  CATKIN_DEPENDS
  rospy
  std_msgs
  asurt_msgs
  sensor_msgs
  ros_numpy
  tf_helper
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/lidar.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
