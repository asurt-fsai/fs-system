cmake_minimum_required(VERSION 3.0.2)
project(supervisor)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             sensor_msgs
             std_msgs
             asurt_msgs
             ros_numpy
             tf_helper)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  supervisor
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  asurt_msgs
  tf_helper
  ros_numpy
  # DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_python_setup()

catkin_install_python(PROGRAMS
nodes/mesh_viz.py
nodes/supervisor_node.py
nodes/driver_node.py
nodes/ros_can_simulator.py
tests-node/placeholder_test_node.py
tests-node/visualizer_test_node.py
tests-node/mission_launcher_test_node.py
tests-unit/test_module.py
tests-unit/test_visualizer.py
tests-unit/test_mission_launcher.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install launch files used for tests
install (DIRECTORY tests-launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
