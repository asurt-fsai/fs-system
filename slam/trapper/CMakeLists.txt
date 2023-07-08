cmake_minimum_required(VERSION 3.5)
project(trapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  asurt_msgs
  tf_helper
  visualization_msgs
)

catkin_python_setup()
add_service_files(
  FILES
  ResetAttributes.srv
)

generate_messages(
  DEPENDENCIES
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
