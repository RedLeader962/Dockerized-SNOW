#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

#
#if [ $# -ne 1 ]; then
#  echo "SNOW-AutoRally | missing argument: $0 <dir with workspace>"
#  exit 1
#fi

echo -n "Name that new container, the crazier the better: "
read CONTAINER_NAME
export LATEST_CONTAINER_USED="${CONTAINER_NAME}"

echo "Enter the host source code directory to mount inside the container "
echo -n "(must be an absolute path eg. /home/snowxavier/Repositories/SNOW-AutoRally): "
read HOST_SOURCE_CODE_PATH

echo "starting ${CONTAINER_NAME}"

CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
#WS_DIR=$1
WS_DIR=$HOST_SOURCE_CODE_PATH
WS_DIRNAME=$(basename $WS_DIR)
#WS_DIRNAME=SNOW-AutoRally
echo "Source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"

## # todo:assessment (ref task NLSAR-159 Fix the execute permission of source code mounted volume)
#sudo chmod --recursive +x "${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"

sudo xhost +si:localuser:root

sudo docker run \
  --name "${CONTAINER_NAME}" \
  --runtime nvidia \
  --interactive \
  --tty \
  --device=/dev/input/js0 \
  --network host \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  --security-opt seccomp=unconfined \
  --volume "${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}" \
  snow-autorally-develop
