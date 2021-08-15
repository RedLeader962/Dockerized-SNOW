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
#autorally_env_setup="${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/autorally_util/setupEnvXavierWarthog.sh"
autorally_env_setup="${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/autorally_util/setupEnv${DS_HOST_TYPE}.sh"
#source "${autorally_env_setup}"
echo "source ${autorally_env_setup}" >> ~/.bashrc


echo -e "
\033[1;2m


               .|'''.|
               ||..  '
               ''|||.            \033[0m \033[1;37m
•••·· ·· · Dockerized-SNOW ··· ·••$(printf '·%.s' {1..$(( $(tput cols) - 41 ))}) ·· ·\033[0m\033[1;2m
                NorLab
              .     '||
              |'....|'
\n\033[0m \033[2;37m
       https://norlab.ulaval.ca
    https://redleader962.github.io


\033[0m
"

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

echo "
  Done building SNOW_AutoRally. Finale step:
    1. Source your SNOW_AutoRally environment
      # source ~/.bashrc
    2. Check if SNOW_AutoRally was properly sourced
      # printenv | grep AR_
    3. Collect the ROS and SNOW_AutoRally environement variable
      # bash /fetch_ros_env.bash
    3. Setup CLion:
      a. Go to Preference > Build,Execution,Deployment > CMake
      b. Set CMake > Build directory: <WORKSPACE_DIRECTORY>/build
          In our case: ros_catkin_ws/build
      c. Set CMake > CMake options: --trace -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
      d. Copy/pass the fetch_ros_env.bash output to CMake > Environment
          It's a workaround to the «start CLion from the same shell» procedure of the CLion doc

"

## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
