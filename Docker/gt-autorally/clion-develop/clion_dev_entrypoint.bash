#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo
echo "Starting container ssh server on port ${CLION_DEV_SERVER_PORT}"
# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
#/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_dockerized_snow_openssh_server
/usr/sbin/sshd -e -f /etc/ssh/sshd_config_dockerized_snow_openssh_server

echo "
    To connect remotely to the container:
        Default user: ${CLION_DEV_USER}  pass: lasagne

        $ ssh -p ${CLION_DEV_SERVER_PORT} ${CLION_DEV_USER}@$(hostname -I | awk '{print $1}')
        $ sftp -P ${CLION_DEV_SERVER_PORT} openssh-$(hostname -I | awk '{print $1}')
        $ scp -P ${CLION_DEV_SERVER_PORT} /path/to/foo ${CLION_DEV_USER}@$(hostname -I | awk '{print $1}'):/dest/
"
echo "

    Use the SNOW_AutoRally build script to compile your code
    and source your ROS environment:

        # bash /rebuild_gt_autorally.bash

"

#echo "    When it's done, fecth your environnement variable for CLion using
#
#        # bash /fetch_ros_env.bash
#
#"

exec "$@"
