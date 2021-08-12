#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo "

    Use the NorLab-MPPI build script to compile your code and source your ROS environment:
       $ bash /build_norlab_mppi.bash

"

exec "$@"
