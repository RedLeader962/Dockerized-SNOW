#!/bin/bash

# ref: https://github.com/dusty-nv/jetson-containers/tree/master/packages
set -e

# === Fork AutoRally repo and install ==================================================================================
#
# Steps:
#   1. clone AutoRally repos
#   2. Install AutoRally ROS Dependencies
#   3. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore (apparently).
#       See pullrequest 243548 merge into `ros:master` on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

# Clone AutoRally and dependencies
DEV_BRANCH=SNOW-melodic-devel
cd "${DEV_WORKSPACE}/src/"
echo "Pull latest ${DEV_BRANCH} branch"
git clone --branch ${DEV_BRANCH} https://github.com/RedLeader962/autorally.git
cd ${DEV_WORKSPACE}
apt-get update && rosdep install --from-path src --ignore-src --default-yes
rm -rf /var/lib/apt/lists/*

# Build AutoRally
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${DEV_WORKSPACE}"
catkin_make
source "${DEV_WORKSPACE}/devel/setup.bash"

# ===Sourcing===========================================================================================================
#ROS_ENV_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
#echo "sourcing   $ROS_ENV_SETUP"
#source "$ROS_ENV_SETUP"
#
#ROS_DEVEL_ENV_SETUP="${DEV_WORKSPACE}/devel/setup.bash"
#echo "sourcing   $ROS_DEVEL_ENV_SETUP"
#source "$ROS_DEVEL_ENV_SETUP"

echo
echo "Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "It should include the directory you're in: /home/<youruser>/catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo

JOYSTICK_ZERO="/dev/input/js0"
# The '-c' flag is a character device file test operator
if [[ -c "$JOYSTICK_ZERO" ]]; then
  chmod a+rw /dev/input/js0
else
  echo "SNOW-AutoRally: No input device js0 detected"
fi

# Environment setup

autorally_env_setup=${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvLocal.sh
echo "sourcing   $autorally_env_setup"
source "$autorally_env_setup"

echo ". ${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvLocal.sh" >>~/.bashrc

#echo "AR_MPPI_PARAMS_PATH $AR_MPPI_PARAMS_PATH"
#echo "AR_CONFIG_PATH $AR_CONFIG_PATH"
#echo "AR_JOYSTICK $AR_JOYSTICK"
echo
printenv | grep AR_
echo

## ... Generate Documentation ...........................................................................................
## Run doxygen with the `-u` flag to remove obsolete configuration tag
## TODO: unmute
#cd ${DEV_WORKSPACE}/src/autorally/ \
#    && doxygen -u

cd ~/
