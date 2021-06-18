#!/bin/bash

# ref: https://github.com/dusty-nv/jetson-containers/tree/master/packages
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

#source "~/catkin_ws/devel/setup.bash"
#source "~/catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh"

#printenv | grep ROS
#printenv | grep AR_

find /dev/input
chmod a+rw /dev/input/js0

exec "$@"