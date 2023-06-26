cmake_minimum_required(VERSION 3.0.2)
project(zed)

find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             message_generation
             roscpp
             rospy
             tf_helper
             sensor_msgs
             visualization_msgs
             stereo_msgs
             asurt_msgs
             std_msgs
             darknet_ros)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  zed
  roscpp
  rospy
  tf_helper
  sensor_msgs
  visualization_msgs
  stereo_msgs
  asurt_msgs
  std_msgs
  darknet_ros
  # DEPENDS system_lib
)

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/wrapper.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
