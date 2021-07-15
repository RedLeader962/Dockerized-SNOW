#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

# Start ssh server
/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_clion_snow_dev

echo "

    Use the SNOW-AutoRally build script to compile your code and source your ROS environment:
       bash /build_snow_autorally.bash

"

exec "$@"
