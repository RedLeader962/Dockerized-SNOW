#!/bin/bash -i

## Note: Only use 'set -e' for debugging until 'autorally_util/setupEnvVariables.sh' is refactored
#set -e # exit script if any statement returns a non-true return value
#set -v

cd "${DEV_WORKSPACE}"

# Install AutoRally dependencies
sudo apt-get update
rosdep install --from-path src --ignore-src -r --default-yes
#rm -rf /var/lib/apt/lists/*

# Build AutoRally
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make
source "${DEV_WORKSPACE}/devel/setup.bash"

# AutoRally environment setup
autorally_env_setup="${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvLocal.sh"
#source "${autorally_env_setup}"
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

#source ~/.bashrc
#echo
#printenv | grep AR_
#echo

echo "Finished building AutoRally, just run"
echo "    source ~/.bashrc"
echo "then run"
echo "    printenv | grep AR_"
echo "to check if AutoRally was properly sourced"
echo "or a new terminal"

## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
