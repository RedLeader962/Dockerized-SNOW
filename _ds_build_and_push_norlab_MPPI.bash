#!/bin/bash


# Load environment variable from file
set -o allexport; source ds.env; set +o allexport

#bash ./visual/terminal_splash.bash

if [[ $(uname -m) == "aarch64" ]]; then
  AARCH=""
  DEPEND_IMG_TAG="arm64-l4t-r32.6.1"
  DEV_IMG_TAG="arm64-l4t-r32.6.1-XavierSA"
  CONTAINER_NAMES="IamSnow-NX"
elif [[ $(uname -m) == "x86_64" ]]; then
  AARCH="--x86"
  DEPEND_IMG_TAG="x86-ubuntu18.04"
  DEV_IMG_TAG="x86-ubuntu18.04"
  CONTAINER_NAMES="IamSnow"
fi


cd ${HOME}/Repositories/Dockerized-SNOW
sudo git pull
#
#echo -e "${DS_MSG_BASE} Building norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
#bash ds_build_melodic_python3.bash ${AARCH} \
#  && echo -e "${DS_MSG_BASE} Pushing to dockerhub" \
#  && sudo docker push norlabsnow/norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG} \
#  && echo -e "${DS_MSG_DONE} norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG} builded and pushed to dockerhub"
#
#
#echo -e "${DS_MSG_BASE} Building norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
#bash ds_build_dependencies.bash ${AARCH} \
#  && echo -e "${DS_MSG_BASE} Pushing to dockerhub" \
#  && sudo docker push norlabsnow/norlab-mppi-dependencies:${DEPEND_IMG_TAG} \
#  && echo -e "${DS_MSG_DONE} norlab-mppi-dependencies:${DEPEND_IMG_TAG} builded and pushed to dockerhub"
#
#echo -e "${DS_MSG_BASE} Building norlab-mppi-develop:${DEV_IMG_TAG}"
#bash ds_build_develop.bash ${AARCH} \
#  && echo -e "${DS_MSG_BASE} Pushing to dockerhub" \
#  && sudo docker push norlabsnow/norlab-mppi-develop:${DEV_IMG_TAG} \
#  && echo -e "${DS_MSG_DONE} norlabsnow/norlab-mppi-develop:${DEV_IMG_TAG} builded and pushed to dockerhub"

# Stop container if he is started
#if [ ! -z `docker ps -qf "name=^/${CONTAINER_NAMES}$"` ]; then
#if [ `docker ps --format "{{.Names}}"` == ${CONTAINER_NAMES} ]; then
if [ `docker ps --quiet --all --format "{{.Names}}" | grep Iam` == ${CONTAINER_NAMES} ]; then
    echo -e "${DS_MSG_DONE} I'M IN <<<<<<<<<<<<<<<<<<<<<<<"
    echo "Stoping container $(docker stop ${CONTAINER_NAMES})"
    echo "Removing container $(docker rm ${CONTAINER_NAMES})"
    bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG}
elif [ ! -z `docker container list -qf "name=^/${CONTAINER_NAMES}$"` ]; then
    echo "Removing container $(docker rm ${CONTAINER_NAMES})"
    bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG}
else
    bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG}
fi

if [ -z `docker ps -qf "name=^/${CONTAINER_NAMES}$"` ]; then
    echo -e "${DS_MSG_DONE} ${CONTAINER_NAMES} is up and running"
fi

