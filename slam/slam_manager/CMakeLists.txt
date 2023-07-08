cmake_minimum_required(VERSION 3.0.2)
project(slam_manager)

find_package(catkin REQUIRED)

catkin_python_setup()

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

catkin_install_python(PROGRAMS nodes/switching_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
