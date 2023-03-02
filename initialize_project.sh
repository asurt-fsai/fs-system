#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PYTHON_SUFFIX=""
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
    PYTHON_SUFFIX=3
fi

ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rospy"
ADDITIONAL_PACKAGES="$ADDITIONAL_PACKAGES python3-pcl" #perception/mrpython-pcl
ADDITIONAL_PACKAGES="$ADDITIONAL_PACKAGES ros-$ROS_DISTRO-velodyne" #dependancies velodyne


if [ "$(lsb_release -sc)" = "focal" ]; then
    ADDITIONAL_PACKAGES="$ADDITIONAL_PACKAGES
                         python-is-python3"
fi


echo ADDITIONAL PACKAGES $ADDITIONAL_PACKAGES

sudo apt update
sudo apt-get install --no-install-recommends -y \
    python$PYTHON_SUFFIX-pip \
    python$PYTHON_SUFFIX-catkin-tools \
    python$PYTHON_SUFFIX-rosdep \
    wget \
    $ADDITIONAL_PACKAGES

pip$PYTHON_SUFFIX install --upgrade pip$PYTHON_SUFFIX
pip$PYTHON_SUFFIX install -r $SCRIPT_DIR/requirements.txt

git submodule update --init --depth 1

chmod +x $SCRIPT_DIR/.hooks/install_hooks.sh
$SCRIPT_DIR/.hooks/install_hooks.sh
