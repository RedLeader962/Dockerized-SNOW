#!/bin/bash

if [[ -z ${DS_PATH} ]]; then
  DS_PATH=$(sudo find / -name "Dockerized-SNOW" -type d 2>/dev/null)
fi
cd ${DS_PATH}
sudo git pull

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

NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD_AND_PUSH=false
NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD_AND_PUSH=false
NORLAB_MPPI_DEPENDENCIES_BUILD_AND_PUSH=false
NORLAB_MPPI_DEVELOP_BUILD_AND_PUSH=false
NORLAB_MPPI_DEVELOP_INSTANTIATED=false


# ...Build & push.......................................................................................................
echo -e "${DS_MSG_BASE} Building norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
bash ds_build_melodic_python3.bash ${AARCH} \
  && echo -e "${DS_MSG_BASE} Pushing to dockerhub" \
  && sudo docker push norlabsnow/norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG} \
  && echo -e "${DS_MSG_DONE} norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG} built and pushed to dockerhub" \
  && NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD_AND_PUSH=true \
  && echo

# Fail fast
if [ $NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD_AND_PUSH == true ]; then
    echo -e "${DS_MSG_DONE} build&push norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
else
    echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
    exit 1
fi


echo -e "${DS_MSG_BASE} Building norlab-mppi-dependencies-wo-services and norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
bash ds_build_dependencies.bash ${AARCH} \
  && echo -e "${DS_MSG_BASE} Pushing to dockerhub" \
  && sudo docker push norlabsnow/norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG} \
  && echo -e "${DS_MSG_DONE} norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG} built and pushed to dockerhub" \
  && NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD_AND_PUSH=true \
  && echo

# Fail fast
if [ $NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD_AND_PUSH == true ]; then
    echo -e "${DS_MSG_DONE} build&push norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
else
    echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
    exit 1
fi

sudo docker push norlabsnow/norlab-mppi-dependencies:${DEPEND_IMG_TAG} \
  && echo -e "${DS_MSG_DONE} norlab-mppi-dependencies:${DEPEND_IMG_TAG} built and pushed to dockerhub" \
  && NORLAB_MPPI_DEPENDENCIES_BUILD_AND_PUSH=true \
  && echo

# Fail fast
if [ $NORLAB_MPPI_DEPENDENCIES_BUILD_AND_PUSH == true ]; then
    echo -e "${DS_MSG_DONE} build&push norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
else
    echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
    exit 1
fi


echo -e "${DS_MSG_BASE} Building norlab-mppi-develop:${DEV_IMG_TAG}"
bash ds_build_develop.bash ${AARCH} \
  && echo -e "${DS_MSG_BASE} Pushing to dockerhub" \
  && sudo docker push norlabsnow/norlab-mppi-develop:${DEV_IMG_TAG} \
  && echo -e "${DS_MSG_DONE} norlabsnow/norlab-mppi-develop:${DEV_IMG_TAG} built and pushed to dockerhub" \
  && NORLAB_MPPI_DEVELOP_BUILD_AND_PUSH=true \
  && echo


# ...Build & push pass/fail status......................................................................................
echo "
............................................................................
Summary
"
if [ $NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD_AND_PUSH == true ]; then
    echo -e "${DS_MSG_DONE} build&push norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
else
    echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
    exit 1
fi

if [ $NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD_AND_PUSH == true ]; then
    echo -e "${DS_MSG_DONE} build&push norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
else
    echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
    exit 1
fi
if [ $NORLAB_MPPI_DEPENDENCIES_BUILD_AND_PUSH == true ]; then
    echo -e "${DS_MSG_DONE} build&push norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
else
    echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
    exit 1
fi

if [ $NORLAB_MPPI_DEVELOP_BUILD_AND_PUSH == true ]; then
    echo -e "${DS_MSG_DONE} build&push norlab-mppi-develop:${DEV_IMG_TAG}"
else
    echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-develop:${DEV_IMG_TAG}"
    exit 1
fi
echo "............................................................................
"

# ...Instantiate container..............................................................................................
# Fetch all container name, strip those unrelated one and test for exact name
if [ `docker ps --quiet --all --format "{{.Names}}" | grep ${CONTAINER_NAMES}` == ${CONTAINER_NAMES} ]; then
    # Stop and remove container if he is started
    echo -e "${DS_MSG_BASE} Stopping container $(docker stop ${CONTAINER_NAMES})" \
      && echo -e "${DS_MSG_BASE} Removing container $(docker rm ${CONTAINER_NAMES})" \
      && echo -e "${DS_MSG_BASE} Starting a new $(docker rm ${CONTAINER_NAMES})" instance\
      && bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG} \
      && NORLAB_MPPI_DEVELOP_INSTANTIATED=true
elif [ `docker container ls --quiet --all --format "{{.Names}}" | grep ${CONTAINER_NAMES}` == ${CONTAINER_NAMES} ]; then
    # Remove container if he is started
    echo -e "${DS_MSG_BASE} Removing container $(docker rm ${CONTAINER_NAMES})" \
      && echo -e "${DS_MSG_BASE} Starting a new $(docker rm ${CONTAINER_NAMES})" instance\
      && bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG} \
      && NORLAB_MPPI_DEVELOP_INSTANTIATED=true
else
    bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG} \
      && NORLAB_MPPI_DEVELOP_INSTANTIATED=true
fi

## ...Instantiate pass/fail status......................................................................................
#if [ $NORLAB_MPPI_DEVELOP_INSTANTIATED == true ]; then
#    echo -e "${DS_MSG_DONE} ${CONTAINER_NAMES} is instantiated and running"
#else
#    echo -e "${DS_MSG_ERROR} ${CONTAINER_NAMES} was not instantiated and is not running"
#fi
#
