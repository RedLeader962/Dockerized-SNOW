#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo
echo "Start ssh server"
# sshd flag
# -D : sshd will not detach and does not become a daemon. This allows easy monitoring of sshd.
# -e : sshd will send the output to the standard error instead of the system log.
# -f : config_file
#/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_clion_snow_dev
/usr/sbin/sshd -e -f /etc/ssh/sshd_config_clion_snow_dev
echo
echo "

    Use the SNOW-AutoRally build script to compile your code and source your ROS environment:
       bash /build_snow_autorally.bash

"

exec "$@"
