#!/bin/bash

# ref: https://github.com/dusty-nv/jetson-containers/tree/master/packages
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

ros_devel_env_setup="~/catkin_ws/devel/setup.bash"
echo "sourcing   $ros_devel_env_setup"

autorally_env_setup="~/catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh"
echo "sourcing   $autorally_env_setup"

echo "AR_JOYSTICK $AR_JOYSTICK"

#printenv | grep ROS
#printenv | grep AR_

joystickZero="/dev/input/js0"

if [[ -d "$joystickZero" ]]; then
  chmod a+rw /dev/input/js0
else
  echo "SNOW-AutoRally: No input device js0 detected"
fi

exec "$@"