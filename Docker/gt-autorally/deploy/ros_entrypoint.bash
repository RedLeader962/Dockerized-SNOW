#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

ROS_ENV_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
echo "sourcing   ${ROS_ENV_SETUP}"
source "${ROS_ENV_SETUP}"

ROS_DEVEL_ENV_SETUP="${DS_DEV_WORKSPACE}/devel/setup.bash"
echo "sourcing   ${ROS_DEVEL_ENV_SETUP}"
source "${ROS_DEVEL_ENV_SETUP}"

echo
echo "  Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "  It should include the directory you're in: /home/<youruser>/ros_catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo

JOYSTICK_ZERO="/dev/input/js0"
if [[ -c "$JOYSTICK_ZERO" ]]; then  # Note: The '-c' flag is a character device file test operator
  chmod a+rw /dev/input/js0
else
  echo "${0}  >>> No input device js0 detected!" >&2
fi

exec "$@"
