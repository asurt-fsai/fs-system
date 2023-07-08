cmake_minimum_required(VERSION 3.0.2)
project(waypoint_cleaner)
find_package(catkin REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_apf
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
catkin_python_setup()

catkin_install_python(PROGRAMS nodes/simple_cleaner_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
