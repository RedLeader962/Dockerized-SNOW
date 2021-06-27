#!/bin/bash

# ref: https://github.com/dusty-nv/jetson-containers/tree/master/packages
set -e

ROS_ENV_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
echo "sourcing   $ROS_ENV_SETUP"
source "$ROS_ENV_SETUP"

ROS_DEVEL_ENV_SETUP="${DEV_WORKSPACE}/devel/setup.bash"
echo "sourcing   $ROS_DEVEL_ENV_SETUP"
source "$ROS_DEVEL_ENV_SETUP"

echo
echo "Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "It should include the directory you're in: /home/<youruser>/catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo
printenv | grep AR

JOYSTICK_ZERO="/dev/input/js0"
# The '-c' flag is a character device file test operator
if [[ -c "$JOYSTICK_ZERO" ]]; then
  chmod a+rw /dev/input/js0
else
  echo "SNOW-AutoRally: No input device js0 detected"
fi

#autorally_env_setup="${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvLocal.sh"
#echo "sourcing   $autorally_env_setup"
#. "$autorally_env_setup"

#echo "AR_MPPI_PARAMS_PATH $AR_MPPI_PARAMS_PATH"
#echo "AR_CONFIG_PATH $AR_CONFIG_PATH"
#echo "AR_JOYSTICK $AR_JOYSTICK"

exec "$@"
