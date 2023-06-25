cmake_minimum_required(VERSION 3.0.2)
project(zed)


find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             message_generation
             roscpp
             rospy
             std_msgs
             geometry_msgs
             asurt_msgs
             tf_helper)
catkin_python_setup()


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/wrapper.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
