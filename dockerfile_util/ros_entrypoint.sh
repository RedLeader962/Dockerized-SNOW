#!/bin/bash

# from https://github.com/dusty-nv/jetson-containers/tree/master/packages
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

find /dev/input
chmod a+rw /dev/input/js0

exec "$@"