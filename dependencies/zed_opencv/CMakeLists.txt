cmake_minimum_required(VERSION 3.0.2)
project(zed_opencv)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slam_manager
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/zed_opencv_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
