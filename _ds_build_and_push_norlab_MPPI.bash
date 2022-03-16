#!/bin/bash

if [[ -z ${DS_PATH} ]]; then
  DS_PATH=$(sudo find / -name "Dockerized-SNOW" -type d 2>/dev/null)
fi
cd ${DS_PATH}
sudo git pull

# Load environment variable from file
set -o allexport
source ds.env
set +o allexport

#bash ./visual/terminal_splash.bash

PUSH=true

function print_help_in_terminal() {

  echo -e "\$ ${0} [<optional argument>]

\033[1m<optional argument>:\033[0m
  -h, --help                      Get help
  --nopush                     build container and instanciate only. Dont push to docker hub

"
}

for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --nopush)
    PUSH=false
    shift # Remove --buildOnly from processing
    ;;
  --)
    shift
    break
    ;;
  *)
    break
    ;;
  esac

  shift
done

if [[ $(uname -m) == "aarch64" ]]; then
  ARCHITECTURE_FLAG=""
  DEPEND_IMG_TAG="arm64-l4t-r32.6.1"
  DEV_IMG_TAG="arm64-l4t-r32.6.1-XavierSA"
  CONTAINER_NAMES="IamSnow-NX"
elif [[ $(uname -m) == "x86_64" ]]; then
  ARCHITECTURE_FLAG="--x86"
  DEPEND_IMG_TAG="x86-ubuntu18.04"
  DEV_IMG_TAG="x86-ubuntu18.04"
  CONTAINER_NAMES="IamSnow"
elif [[ $(uname -s) == "Darwin" ]] && [[ $(uname -m) == "arm64" ]]; then
  ARCHITECTURE_FLAG="--M1"
  DEPEND_IMG_TAG="arm64-Darwin-ubuntu18.04"
  DEV_IMG_TAG="arm64-Darwin-ubuntu18.04"
  CONTAINER_NAMES="IamSnow-M1"
  CONTAINER_FLAG="--osx --src=${HOME}/PycharmProjects/NorLab_MPPI"
else
  echo -e "${DS_MSG_ERROR} This script does not yet support the $(uname -s) $(uname -m) architecture"
  exit 1
fi

NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD=false
NORLAB_MPPI_ROS_MELODIC_PYTHON_3_PUSH=false
NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD=false
NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_PUSH=false
NORLAB_MPPI_DEPENDENCIES_BUILD=false
NORLAB_MPPI_DEPENDENCIES_PUSH=false
NORLAB_MPPI_DEVELOP_BUILD=false
NORLAB_MPPI_DEVELOP_PUSH=false
NORLAB_MPPI_DEVELOP_TEAMCITY_BUILD=false
NORLAB_MPPI_DEVELOP_TEAMCITY_PUSH=false
NORLAB_MPPI_DEVELOP_INSTANTIATED=false

sudo docker login

# ...Build & push.......................................................................................................

function echo_fail_fast_msg() {
  # Fail fast
  if [ ${1} == true ]; then
    echo -e "${DS_MSG_DONE} ${2}"
  else
    echo -e "${DS_MSG_ERROR} failed to ${2}"
    exit 1
  fi
}

echo -e "${DS_MSG_BASE} Building norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
bash ds_build_melodic_python3.bash ${ARCHITECTURE_FLAG} \
  && NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD=true

echo_fail_fast_msg ${NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD} "build norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"

if [ ${PUSH} == true ]; then
  bash echo -e "${DS_MSG_BASE} Pushing norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG} to dockerhub" &&
    sudo docker push norlabsnow/norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG} &&
    NORLAB_MPPI_ROS_MELODIC_PYTHON_3_PUSH=true

  echo_fail_fast_msg $NORLAB_MPPI_ROS_MELODIC_PYTHON_3_PUSH "push norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
fi

echo -e "${DS_MSG_BASE} Building norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
bash ds_build_dependencies.bash ${ARCHITECTURE_FLAG} --noservices \
  && NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD=true

echo_fail_fast_msg ${NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD} "build norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"

if [ ${PUSH} == true ]; then
  bash echo -e "${DS_MSG_BASE} Pushing norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG} to dockerhub" &&
    sudo docker push norlabsnow/norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG} &&
    NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_PUSH=true

  echo_fail_fast_msg ${NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_PUSH} "push norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
fi

echo -e "${DS_MSG_BASE} Building norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
bash ds_build_dependencies.bash ${ARCHITECTURE_FLAG} \
  && NORLAB_MPPI_DEPENDENCIES_BUILD=true

echo_fail_fast_msg ${NORLAB_MPPI_DEPENDENCIES_BUILD} "build norlab-mppi-dependencies:${DEPEND_IMG_TAG}"

if [ ${PUSH} == true ]; then
  bash echo -e "${DS_MSG_BASE} Pushing norlab-mppi-dependencies:${DEPEND_IMG_TAG} to dockerhub" &&
    sudo docker push norlabsnow/norlab-mppi-dependencies:${DEPEND_IMG_TAG} &&
    NORLAB_MPPI_DEPENDENCIES_PUSH=true

    echo_fail_fast_msg ${NORLAB_MPPI_DEPENDENCIES_PUSH} "push norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
fi

echo -e "${DS_MSG_BASE} Building norlab-mppi-develop:${DEV_IMG_TAG}"
bash ds_build_develop.bash ${ARCHITECTURE_FLAG} \
  && NORLAB_MPPI_DEVELOP_BUILD=true \
  && NORLAB_MPPI_DEVELOP_TEAMCITY_BUILD=true

echo_fail_fast_msg ${NORLAB_MPPI_DEVELOP_BUILD} "build norlab-mppi-develop::${DEPEND_IMG_TAG}"
echo_fail_fast_msg ${NORLAB_MPPI_DEVELOP_TEAMCITY_BUILD} "build norlab-mppi-develop-teamcity::${DEPEND_IMG_TAG}"

if [ ${PUSH} == true ]; then
  bash echo -e "${DS_MSG_BASE} Pushing norlab-mppi-develop:${DEV_IMG_TAG} to dockerhub" &&
    sudo docker push norlabsnow/norlab-mppi-develop:${DEV_IMG_TAG} &&
    NORLAB_MPPI_DEVELOP_PUSH=true &&
    sudo docker push norlabsnow/norlab-mppi-develop-teamcity:${DEV_IMG_TAG} &&
    NORLAB_MPPI_DEVELOP_TEAMCITY_PUSH=true

    echo_fail_fast_msg ${NORLAB_MPPI_DEVELOP_PUSH} "push norlab-mppi-develop::${DEPEND_IMG_TAG}"
    echo_fail_fast_msg ${NORLAB_MPPI_DEVELOP_TEAMCITY_PUSH} "push norlab-mppi-develop-teamcity::${DEPEND_IMG_TAG}"
fi

# ...Build & push pass/fail status......................................................................................
echo "
............................................................................
Summary
"
if [ $NORLAB_MPPI_ROS_MELODIC_PYTHON_3_BUILD == true ] || [ $NORLAB_MPPI_ROS_MELODIC_PYTHON_3_PUSH == true ]; then
  echo -e "${DS_MSG_DONE} build&push norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
else
  echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-ros-melodic-python3:${DEPEND_IMG_TAG}"
  exit 1
fi

if [ $NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_BUILD == true ] || [ $NORLAB_MPPI_DEPENDENCIES_WO_SERVICES_PUSH == true ]; then
  echo -e "${DS_MSG_DONE} build&push norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
else
  echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-dependencies-wo-services:${DEPEND_IMG_TAG}"
  exit 1
fi
if [ $NORLAB_MPPI_DEPENDENCIES_BUILD == true ] || [ $NORLAB_MPPI_DEPENDENCIES_PUSH == true ]; then
  echo -e "${DS_MSG_DONE} build&push norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
else
  echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-dependencies:${DEPEND_IMG_TAG}"
  exit 1
fi

if [ $NORLAB_MPPI_DEVELOP_BUILD == true ] || [ $NORLAB_MPPI_DEVELOP_PUSH == true ]; then
  echo -e "${DS_MSG_DONE} build&push norlab-mppi-develop:${DEV_IMG_TAG}"
else
  echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-develop:${DEV_IMG_TAG}"
  exit 1
fi
if [ $NORLAB_MPPI_DEVELOP_TEAMCITY_BUILD == true ] || [ $NORLAB_MPPI_DEVELOP_TEAMCITY_PUSH == true ]; then
  echo -e "${DS_MSG_DONE} build&push norlab-mppi-develop-teamcity:${DEV_IMG_TAG}"
else
  echo -e "${DS_MSG_ERROR} failed to build or push norlab-mppi-develop-teamcity:${DEV_IMG_TAG}"
  exit 1
fi
echo "............................................................................
"

# ...Instantiate container..............................................................................................
# Fetch all container name, strip those unrelated one and test for exact name
if [ $(docker ps --quiet --all --format "{{.Names}}" | grep ${CONTAINER_NAMES}) == ${CONTAINER_NAMES} ]; then
  # Stop the container if he is started and remove it
  echo -e "${DS_MSG_BASE} Stopping container $(docker stop ${CONTAINER_NAMES})" &&
    echo -e "${DS_MSG_BASE} Removing container $(docker rm ${CONTAINER_NAMES})" &&
    echo -e "${DS_MSG_BASE} Starting a new instance" &&
    bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG} ${CONTAINER_FLAG} &&
    NORLAB_MPPI_DEVELOP_INSTANTIATED=true
elif [ $(docker container ls --quiet --all --format "{{.Names}}" | grep ${CONTAINER_NAMES}) == ${CONTAINER_NAMES} ]; then
  # Remove container if he is instanciated
  echo -e "${DS_MSG_BASE} Removing container $(docker rm ${CONTAINER_NAMES})" &&
    echo -e "${DS_MSG_BASE} Starting a new instance" &&
    bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG} ${CONTAINER_FLAG} &&
    NORLAB_MPPI_DEVELOP_INSTANTIATED=true
else
  bash ds_instantiate_develop.bash --name=${CONTAINER_NAMES} --runTag=${DEV_IMG_TAG} ${CONTAINER_FLAG} &&
    NORLAB_MPPI_DEVELOP_INSTANTIATED=true
fi

## ...Instantiate pass/fail status......................................................................................
#if [ $NORLAB_MPPI_DEVELOP_INSTANTIATED == true ]; then
#    echo -e "${DS_MSG_DONE} ${CONTAINER_NAMES} is instantiated and running"
#else
#    echo -e "${DS_MSG_ERROR} ${CONTAINER_NAMES} was not instantiated and is not running"
#fi
#
