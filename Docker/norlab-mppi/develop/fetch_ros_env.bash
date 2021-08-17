#!/bin/bash

# Credit: Anas Abou Allaban
#   https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/

ros_env="AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH
LD_LIBRARY_PATH PATH ROS_DISTRO ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_VERSION
ROS_ETC_DIR
ROS_ROOT
ROS_MASTER_URI
ROS_PACKAGE_PATH
ROSLISP_PACKAGE_DIRECTORIES
DS_PYCHARM_DEV_SERVER_PORT
DS_PYCHARM_DEV_USER
DS_DEV_WORKSPACE
DS_TARGET_PROJECT_SRC_REPO
DS_HOST_TYPE
DS_PYTHON3_VERSION
DS_ROS_PKG
DS_ROS_PKG
DS_ROS_ROOT
DS_IMAGE_ARCHITECTURE
DS_IMG_LSB_RELEASE
DS_DEV_WORKSPACE
MASTER_HOSTNAME
HOSTNAME
ROSLAUNCH_SSH_UNKNOWN
MASTER_USER
"
env_string=""
for e in ${ros_env}; do
  env_string+="$e=${!e};"
done
echo "
  Copy the following in Clion > Preference > Build,Execution,Deployment > CMake > Environment

  $env_string

  Finaly reload the CMake project and start using CLion auto-completion, code hint
  and remote GDB debuging tools ... like there is no tomorow!

"
