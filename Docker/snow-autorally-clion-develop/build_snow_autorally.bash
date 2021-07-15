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
autorally_env_setup="${DEV_WORKSPACE}/src/${SRC_CODE_REPOSITORY_NAME}/autorally_util/setupEnvLocal.sh"
#source "${autorally_env_setup}"
echo "source ${autorally_env_setup}" >> ~/.bashrc


echo -e "
\033[1;2m


                   .|'''.|  '|.   '|'  ..|''||   '|| '||'  '|'
                   ||..  '   |'|   |  .|'    ||   '|. '|.  .'
                    ''|||.   | '|. |  ||      ||   ||  ||  |
                  .     '||  |   |||  '|.     ||    ||| |||
                  |'....|'  .|.   '|   ''|...|'      |   |

                               (Dockerized-SNOW)

                https://github.com/RedLeader962/Dockerized-SNOW
                           https://norlab.ulaval.ca

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
echo "  It should include the directory you're in: /home/<youruser>/catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo

cd /

#source ~/.bashrc
#echo
#printenv | grep AR_
#echo

echo "
  Done building SNOW-AutoRally. Finale step:
    1. Source your SNOW-AutoRally environment
      # source ~/.bashrc
    2. Check if SNOW-AutoRally was properly sourced
      # printenv | grep AR_
    3. Collect the ROS and SNOW-AutoRally environement variable
      # bash /fetch_ros_env.bash
    3. Setup CLion:
      a. Go to Preference > Build,Execution,Deployment > CMake
      b. Set CMake > Build directory: <WORKSPACE_DIRECTORY>/build
          In our case: catkin_ws/build
      c. Copy/pass the fetch_ros_env.bash output to CMake > Environment

      Note:
      - DO NOT add -DCATKIN_DEVEL_PREFIX:PATH=<WORKSPACE_DIRECTORY>/devel to CMake > CMake options
          (Contrary to CLion doc instruction)
      - No need to  use the start CLion from the same shell procedure from CLion doc

"

## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
