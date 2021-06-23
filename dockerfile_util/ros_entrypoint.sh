#!/bin/bash -i --verbose

# ref: https://github.com/dusty-nv/jetson-containers/tree/master/packages
set -e

#ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
#echo "sourcing   $ros_env_setup"
#source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

#ros_devel_env_setup=~/catkin_ws/devel/setup.bash
#echo "sourcing   $ros_devel_env_setup"
#source "$ros_devel_env_setup"

echo "Make sure your workspace is properly overlayed by the setup script by making sure the ROS_PACKAGE_PATH environment variable includes the directory you're in."
echo "You should see: /home/<youruser>/catkin_ws/src:/opt/ros/melodic/share"
echo "ROS_PACKAGE_PATH $ROS_PACKAGE_PATH"

joystickZero="/dev/input/js0"
# The '-c' flag is a character device file test operator
if [[ -c "$joystickZero" ]]; then
  chmod a+rw /dev/input/js0
else
  echo "SNOW-AutoRally: No input device js0 detected"
fi

autorally_env_setup=~/catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh
echo "sourcing   $autorally_env_setup"
source "$autorally_env_setup"

echo "AR_MPPI_PARAMS_PATH $AR_MPPI_PARAMS_PATH"
echo "AR_CONFIG_PATH $AR_CONFIG_PATH"
echo "AR_JOYSTICK $AR_JOYSTICK"


exec "$@"