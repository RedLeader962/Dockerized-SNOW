#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo
echo "Starting container ssh server on port ${DS_PYCHARM_DEV_USER}"
# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
#/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_clion_snow_dev
/usr/sbin/sshd -e -f /etc/ssh/sshd_config_clion_snow_dev

echo "
    To connect remotely to the container:
        Default user: ${DS_PYCHARM_DEV_USER}  pass: lasagne

        $ ssh -p ${DS_PYCHARM_DEV_USER} ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}')
        $ sftp -P ${DS_PYCHARM_DEV_USER} openssh-$(hostname -I | awk '{print $1}')
        $ scp -P ${DS_PYCHARM_DEV_USER} source target
        $ scp -P ${DS_PYCHARM_DEV_USER} /path/to/foo ${DS_PYCHARM_DEV_USER}@$(hostname -I | awk '{print $1}'):/dest/
"

echo "

    From inside the container, use the norlab-mppi build script to compile your development source code and source your ROS environment:
       root@norlab-og:/# bash /rebuild_norlab_mppi.bash

    Note: Docker container user are by default sudo user. That's why it's # instead of $.
"

exec "$@"
