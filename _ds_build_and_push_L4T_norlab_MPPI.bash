#!/bin/bash

#CONTAINER_NAMES="IamSnow-NX"
#
## Stop container if he is stopped
#if [ -z `docker ps -qf "name=^/${CONTAINER_NAMES}$"` ]; then
#    echo "Stoping container $(docker stop ${CONTAINER_NAMES})"
#fi

if [ "$(uname -m)" == "aarch64" ]; then

  echo "Building norlab-mppi-dependencies:arm64-l4t-r32.6.1"
  sudo docker build -t norlabsnow/norlab-mppi-dependencies:arm64-l4t-r32.6.1 -f ./Docker/norlab-mppi/dependencies/Dockerfile  --build-arg BASE_IMAGE=norlabsnow/norlab-mppi-ros-melodic-python3:arm64-l4t-r32.6.1  ./Docker

  echo "Pushing to dockerhub"
  sudo docker push norlabsnow/norlab-mppi-dependencies:arm64-l4t-r32.6.1
  echo "norlab-mppi-dependencies:arm64-l4t-r32.6.1 builded and pushed to dockerhub"

fi
