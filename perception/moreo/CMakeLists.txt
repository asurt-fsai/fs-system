cmake_minimum_required(VERSION 3.0.2)
project(moreo)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             roscpp
             rospy
             std_msgs
             geometry_msgs
             darknet_ros_msgs
             asurt_msgs)

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


catkin_install_python(PROGRAMS nodes/moreo_system.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(# include
                    ${catkin_INCLUDE_DIRS})
