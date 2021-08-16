#!/bin/bash -i

#set -e # exit script if any statement returns a non-true return value
#set -v

cd "${DS_DEV_WORKSPACE}"

# Install dependencies
sudo apt-get update
rosdep install --from-path src --ignore-src -r --default-yes
#rm -rf /var/lib/apt/lists/*

python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_ROS_ROOT} -DCMAKE_BUILD_TYPE=Release \
    &&  rm -rf /var/lib/apt/lists/*

source "${DS_ROS_ROOT}/setup.bash"


# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
## Environment setup
#norlab_mppi_env_setup="${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/mppi_util/setupEnv${DS_HOST_TYPE}.sh"
#echo "source ${norlab_mppi_env_setup}" >> ~/.bashrc

# (NICE TO HAVE) todo:implement >> Terminal splash screen

echo
echo "  Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "  It should include the directory you're in: /home/<youruser>/ros_catkin_ws/src:/opt/ros/melodic/share"
echo
printenv | grep -e ROS -e DS_
echo

cd /

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
#echo "
#  Done building norlab-mppi.
#  Finale step:
#    1. source your norlab-mppi environment
#      # source ~/.bashrc
#    2. check if norlab-mppi was properly sourced
#      # printenv | grep AR_
#  or open a new terminal in the container
#    $ bash /open_new_terminal.bash <container name>
#"


## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
