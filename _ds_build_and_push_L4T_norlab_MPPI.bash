#!/bin/bash


# Load environment variable from file
set -o allexport; source ds.env; set +o allexport

bash ./visual/terminal_splash.bash

#CONTAINER_NAMES="IamSnow-NX"
#
## Stop container if he is stopped
#if [ -z `docker ps -qf "name=^/${CONTAINER_NAMES}$"` ]; then
#    echo "Stoping container $(docker stop ${CONTAINER_NAMES})"
#fi


if [[ $(uname -m) == "aarch64" ]]; then

  cd /home/snow/Repositories/Dockerized-SNOW


#  sudo docker build -t norlabsnow/norlab-mppi-dependencies:arm64-l4t-r32.6.1 -f ./Docker/norlab-mppi/dependencies/Dockerfile  --build-arg BASE_IMAGE=norlabsnow/norlab-mppi-ros-melodic-python3:arm64-l4t-r32.6.1  ./Docker \
#    && echo "${DS_MSG_BASE} Pushing to dockerhub" \
#    && sudo docker push norlabsnow/norlab-mppi-dependencies:arm64-l4t-r32.6.1 \
#    && echo "${DS_MSG_DONE} norlab-mppi-dependencies:arm64-l4t-r32.6.1 builded and pushed to dockerhub"

  echo "${DS_MSG_BASE} Building norlab-mppi-dependencies:arm64-l4t-r32.6.1"
  bash ds_build_dependencies.bash \
    && echo "${DS_MSG_BASE} Pushing to dockerhub" \
    && sudo docker push norlabsnow/norlab-mppi-dependencies:arm64-l4t-r32.6.1 \
    && echo "${DS_MSG_DONE} norlab-mppi-dependencies:arm64-l4t-r32.6.1 builded and pushed to dockerhub"

  echo "${DS_MSG_BASE} Building norlab-mppi-develop:arm64-l4t-r32.6.1-XavierSA"
  bash ds_build_develop.bash  \
    && echo "${DS_MSG_BASE} Pushing to dockerhub" \
    && sudo docker push norlabsnow/norlab-mppi-develop:arm64-l4t-r32.6.1-XavierSA \
    && echo "${DS_MSG_DONE} norlabsnow/norlab-mppi-develop:arm64-l4t-r32.6.1-XavierSA builded and pushed to dockerhub"


fi

