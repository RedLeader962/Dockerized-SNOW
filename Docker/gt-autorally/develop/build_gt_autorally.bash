#!/bin/bash -i

## Note: Only use 'set -e' for debugging until 'autorally_util/setupEnvVariables.sh' is refactored
#set -e # exit script if any statement returns a non-true return value
#set -v

cd "${DS_DEV_WORKSPACE}"

# Install AutoRally dependencies
sudo apt-get update
rosdep install --from-path src --ignore-src -r --default-yes
#rm -rf /var/lib/apt/lists/*

# Build AutoRally
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make
source "${DS_DEV_WORKSPACE}/devel/setup.bash"

# AutoRally environment setup
#autorally_env_setup="${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/autorally_util/setupEnvLocal.sh"
autorally_env_setup="${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/autorally_util/setupEnv${DS_HOST_TYPE}.sh"
#source "${autorally_env_setup}"
echo "source ${autorally_env_setup}" >> ~/.bashrc

# (NICE TO HAVE) todo:implement >> Terminal splash screen

JOYSTICK_ZERO="/dev/input/js0"
# The '-c' flag is a character device file test operator
if [[ -c "$JOYSTICK_ZERO" ]]; then
  chmod a+rw /dev/input/js0
else
  echo "${0}  >>> No input device js0 detected!" >&2
fi

echo
echo "  Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "  It should include the directory you're in: /home/<youruser>/ros_catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo

cd /

#source ~/.bashrc
#echo
#printenv | grep AR_
#echo

#echo "  Finished building AutoRally, just run"
#echo "      source ~/.bashrc"
#echo "  then run"
#echo "      printenv | grep AR_"
#echo "  to check if AutoRally was properly sourced or open a new terminal"
#echo


echo "
  Done building SNOW_AutoRally.
  Finale step:
    1. source your SNOW_AutoRally environment
      # source ~/.bashrc
    2. check if SNOW_AutoRally was properly sourced
      # printenv | grep AR_
  or open a new terminal in the container
    $ bash /open_new_terminal.bash <container name>

"


## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
