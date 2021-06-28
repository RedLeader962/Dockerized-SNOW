#!/usr/bin/env bash

set -e  # exit script if any statement returns a non-true return value


if [ $# -ne 1 ]; then
  echo "SNOW-AutoRally | missing argument: $0 <dir with workspace>"
  exit 1
fi

sudo xhost +si:localuser:root

DOCKER_OPTS=

# mount the subt_solution source code as a volume into the workspace 'solution_ws'
#CONTAINER_WS_PATH_="${DEV_WORKSPACE}/src/"
CONTAINER_WS_PATH_="/catkin_ws/src/"   # (Priority) todo:refactor >> this line ← make it global
WS_DIR=$1
#WS_DIRNAME=$(basename $WS_DIR)
WS_DIRNAME=autorally
echo "Workspace: $WS_DIR -> $CONTAINER_WS_PATH_$WS_DIRNAME"
DOCKER_OPTS="$DOCKER_OPTS --volume $WS_DIR:$CONTAINER_WS_PATH_$WS_DIRNAME"

sudo docker run \
  --runtime nvidia \
  --interactive \
  --tty \
  --rm \
  --device=/dev/input/js0 \
  --network host \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  $DOCKER_OPTS \
  snow-auto-rally-dev
