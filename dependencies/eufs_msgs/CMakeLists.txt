cmake_minimum_required(VERSION 3.5)
project(eufs_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy
                                        std_msgs geometry_msgs sensor_msgs)

################### Add the message you need here #############################
add_message_files(FILES CanState.msg WheelSpeeds.msg WheelSpeedsStamped.msg VehicleCommands.msg VehicleCommandsStamped.msg)
###############################################

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS message_generation roscpp rospy std_msgs)
include_directories(${catkin_INCLUDE_DIRS})
