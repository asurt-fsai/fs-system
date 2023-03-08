cmake_minimum_required(VERSION 3.0.2)
project(planning_trees)

find_package(catkin REQUIRED)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES planning_trees CATKIN_DEPENDS
  # other_catkin_pkg DEPENDS system_lib
)

# ##############################################################################
# Build ##
# ##############################################################################

# Directories to look for headers
include_directories(${catkin_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME} src/cone.cpp src/waypoint.cpp src/path.cpp src/path_queue.cpp
                            src/tree_search.cpp)

# Add cmake target dependencies of the library as an example, code may need to
# be generated before libraries either from message generation or dynamic
# reconfigure add_dependencies(${PROJECT_NAME}
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide add_executable(${PROJECT_NAME}_node
# src/planning_trees_node.cpp)

# Rename C++ executable without prefix The above recommended prefix causes long
# target names, the following renames the target back to the shorter version for
# ease of user use e.g. "rosrun someones_pkg node" instead of "rosrun
# someones_pkg someones_pkg_node" set_target_properties(${PROJECT_NAME}_node
# PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )

# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtests
catkin_add_gtest(${PROJECT_NAME}-cone_test test/cone_test.cpp)
target_link_libraries(${PROJECT_NAME}-cone_test ${catkin_LIBRARIES}
${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-waypoint_test test/waypoint_test.cpp)
target_link_libraries(${PROJECT_NAME}-waypoint_test ${catkin_LIBRARIES}
${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-path_test test/path_test.cpp)
target_link_libraries(${PROJECT_NAME}-path_test ${catkin_LIBRARIES}
${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-path_queue_test test/path_queue_test.cpp)
target_link_libraries(${PROJECT_NAME}-path_queue_test ${catkin_LIBRARIES}
                      ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-tree_search_test test/tree_search_test.cpp)
target_link_libraries(${PROJECT_NAME}-tree_search_test ${catkin_LIBRARIES}
                      ${PROJECT_NAME})
