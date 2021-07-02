#!/bin/bash



# === Fork AutoRally repo and install ==================================================================================
#
# Steps:
#   1. clone AutoRally repos
#   2. Install AutoRally ROS Dependencies
#   3. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore (apparently).
#       See pullrequest 243548 merge into `ros:master` on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

DEV_BRANCH=SNOW-melodic-devel

# Clone AutoRally and dependencies
cd "${DEV_WORKSPACE}/src/"
echo "Pull latest ${DEV_BRANCH} branch"
git clone --branch ${DEV_BRANCH} https://github.com/RedLeader962/autorally.git



## ... Generate Documentation ...........................................................................................
## Run doxygen with the `-u` flag to remove obsolete configuration tag
## TODO: unmute
#cd ${DEV_WORKSPACE}/src/autorally/ \
#    && doxygen -u




sudo xhost +si:localuser:root


#test