#!/usr/bin/env bash

set -e # exit script if any statement returns a non-true return value
set -v # verbose

## Clone AutoRally and dependencies
#DEV_BRANCH=SNOW-melodic-devel
#cd "${DEV_WORKSPACE}/src/"
#echo "Pull latest ${DEV_BRANCH} branch"
#git clone --branch ${DEV_BRANCH} https://github.com/RedLeader962/autorally.git
cd "${DEV_WORKSPACE}"
apt-get update && rosdep install --from-path src --ignore-src --default-yes
#rm -rf /var/lib/apt/lists/*

# Build AutoRally
source "/opt/ros/${ROS_DISTRO}/setup.bash"
#cd "${DEV_WORKSPACE}"
catkin_make
source "${DEV_WORKSPACE}/devel/setup.bash"

# AutoRally environment setup
#autorally_env_setup="${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvLocal.sh"
autorally_env_setup="${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvRemote.sh"
echo "sourcing   ${autorally_env_setup}"
. "${autorally_env_setup}"

echo "<<< RED" # todo:on task end >> delete this line ←
echo ". ${autorally_env_setup}" >> ~/.bashrc

## AutoRally environment setup
#autorally_env_setup="${DEV_WORKSPACE}/src/autorally_fork/autorally_util/setupEnvLocal.sh"
#echo "sourcing   $autorally_env_setup"
#source "$autorally_env_setup"
#
#echo ". ${DEV_WORKSPACE}/src/autorally_fork/autorally_util/setupEnvLocal.sh" >> ~/.bashrc

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
printenv | grep AR_
echo

cd /
