cmake_minimum_required(VERSION 3.0.2)
project(smornn)

find_package(catkin REQUIRED COMPONENTS rospy asurt_msgs tf_helper)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES smornn CATKIN_DEPENDS rospy asurt_msgs tf_helper
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/smornn_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
