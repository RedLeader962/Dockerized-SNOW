#!/bin/bash -i

#set -e # exit script if any statement returns a non-true return value
#set -v

cd "${DEV_WORKSPACE}"

# Install dependencies
sudo apt-get update
rosdep install --from-path src --ignore-src -r --default-yes
#rm -rf /var/lib/apt/lists/*

# Build AutoRally
source "/opt/ros/${ROS_DISTRO}/setup.bash"
catkin_make
source "${DEV_WORKSPACE}/devel/setup.bash"

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
## Environment setup
#norlab_mppi_env_setup="${DEV_WORKSPACE}/src/${SRC_CODE_REPOSITORY_NAME}/mppi_util/setupEnv${HOST_TYPE}.sh"
#echo "source ${norlab_mppi_env_setup}" >> ~/.bashrc


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

echo
echo "  Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "  It should include the directory you're in: /home/<youruser>/catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep ROS
echo

cd /

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
#echo "
#  Done building NorLab-MPPI.
#  Finale step:
#    1. source your NorLab-MPPI environment
#      # source ~/.bashrc
#    2. check if NorLab-MPPI was properly sourced
#      # printenv | grep AR_
#  or open a new terminal in the container
#    $ bash /open_new_terminal.bash <container name>
#"


## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
