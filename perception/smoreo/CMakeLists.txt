cmake_minimum_required(VERSION 3.0.2)
project(smoreo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  asurt_msgs
  tf_helper
)
catkin_python_setup()

generate_dynamic_reconfigure_options(
  src/smoreo/cfg/smoreo.cfg

)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  smoreo
  CATKIN_DEPENDS
  roscpp
  rospy
  # DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src rostests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)



catkin_install_python(PROGRAMS nodes/smoreo_system.py nodes/tuner.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
