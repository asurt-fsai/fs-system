#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/nvidia/catkin_fs/devel/setup.bash
bash -c "rosrun rviz rviz -d /home/nvidia/catkin_fs/src/supervisor/rviz/system.rviz"

exec $SHELL
