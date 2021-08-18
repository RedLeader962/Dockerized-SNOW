#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo
echo "Starting container ssh server on port ${DS_PYCHARM_DEV_SERVER_PORT}"
# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_clion_snow_dev
#/usr/sbin/sshd -e -f /etc/ssh/sshd_config_pycharm_ds_dev_norlab_mppi

echo "
    To connect remotely to the container:
        Default user: ${DS_PYCHARM_DEV_USER}  pass: lasagne

        $ ssh -p ${DS_PYCHARM_DEV_SERVER_PORT} ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}')
        $ sftp -P ${DS_PYCHARM_DEV_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
        $ scp -P ${DS_PYCHARM_DEV_SERVER_PORT} source target
        $ scp -P ${DS_PYCHARM_DEV_SERVER_PORT} /path/to/foo ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}'):/dest/
"

echo "

    From inside the container, use the norlab-mppi build script to compile your development source code and source your ROS environment when needed:

      # bash /rebuild_norlab_mppi.bash

    Check if Python3 is working properly by running this command

      # python3 /ros_catkin_ws/src/${DS_TARGET_PROJECT_SRC_REPO}/src/container_related/try_pytorch.py

    Recall that your project source code is mapped in the container in dir /ros_catkin_ws/src/${DS_TARGET_PROJECT_SRC_REPO}/
    Note: Docker container user are by default sudo user. That's why it's # instead of $.
"

exec "$@"
