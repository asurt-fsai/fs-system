cmake_minimum_required(VERSION 3.5)
project(eufs_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# List of message files
set(msg_files
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/CanState.msg"
  "msg/CarState.msg"
  "msg/ChassisCommand.msg"
  "msg/ChassisState.msg"
  "msg/ConeArray.msg"
  "msg/ConeArrayWithCovariance.msg"
  "msg/ConeWithCovariance.msg"
  "msg/Costmap.msg"
  "msg/EKFErr.msg"
  "msg/EKFState.msg"
  "msg/FullState.msg"
  "msg/IntegrationErr.msg"
  "msg/LapStats.msg"
  "msg/PathIntegralParams.msg"
  "msg/PathIntegralStats.msg"
  "msg/PathIntegralStatus.msg"
  "msg/PathIntegralTiming.msg"
  "msg/PointArray.msg"
  "msg/PointArrayStamped.msg"
  "msg/Runstop.msg"
  "msg/SLAMErr.msg"
  "msg/SLAMState.msg"
  "msg/SystemStatus.msg"
  "msg/TopicStatus.msg"
  "msg/Waypoint.msg"
  "msg/WaypointArrayStamped.msg"
  "msg/WheelOdometryErr.msg"
  "msg/WheelSpeeds.msg"
  "msg/WheelSpeedsStamped.msg"
)

# Fixes 'No rule to make target..'
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

set(action_files
  "action/CheckForObjects.action"
)

# Generate messages & services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
  ADD_LINTER_TESTS
)

ament_package()
