#!/bin/bash

set -e # exit script if any statement returns a non-true return value

cd "${DEV_WORKSPACE}"

# Install AutoRally dependencies
apt-get update && rosdep install --from-path src --ignore-src --default-yes
#rm -rf /var/lib/apt/lists/*

# Build AutoRally
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make
source "${DEV_WORKSPACE}/devel/setup.bash"

# AutoRally environment setup
autorally_env_setup="${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvLocal.sh"
echo "source ${autorally_env_setup}" >> ~/.bashrc


JOYSTICK_ZERO="/dev/input/js0"
# The '-c' flag is a character device file test operator
if [[ -c "$JOYSTICK_ZERO" ]]; then
  chmod a+rw /dev/input/js0
else
  echo "> SNOW-AutoRally: No input device js0 detected"
fi

echo
echo "Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "It should include the directory you're in: /home/<youruser>/catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo

cd /
#exec bash