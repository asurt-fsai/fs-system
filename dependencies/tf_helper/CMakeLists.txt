cmake_minimum_required(VERSION 3.0.2)
project(tf_helper)

find_package(catkin REQUIRED)

catkin_python_setup()

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if your
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(
  # INCLUDE_DIRS include LIBRARIES tf_helper CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(# include ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
