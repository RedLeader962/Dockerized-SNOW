#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo "

    From inside the container, use the norlab-mppi build script to compile your development source code and source your ROS environment:
       root@norlab-og:/# bash /rebuild_norlab_mppi.bash

    Note: Docker container user are by default sudo user. That's why it's `#` instead of `$`.
"

exec "$@"
