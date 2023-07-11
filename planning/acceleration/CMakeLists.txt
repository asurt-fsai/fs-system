cmake_minimum_required(VERSION 3.0.2)
project(acceleration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
catkin_python_setup()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES acceleration
#  CATKIN_DEPENDS rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_install_python(PROGRAMS
  nodes/planning.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
