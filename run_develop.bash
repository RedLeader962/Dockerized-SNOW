#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo -e "
\033[1;2m


        .|'''.|
        ||..  '
         ''|||.
       .     '||
       |'....|'

   (Dockerized-SNOW)
https://norlab.ulaval.ca

\033[0m
"

function print_help_in_terminal() {

  echo -e "
  ${0} [<optional argument>]

    <optional argument>:
      -h, --help                      Get help
      --runTag=<thatTag>     Overwrite image tag eg.: arm64-l4t-r32.6.1-XavierSA-test, x86-ubuntu20.04-gazebo-dart
      --name=<myCoolContainer>        Name that new container, the crazier the better
      --src=<myCoolSrcCode>           Host source code directory to mount inside the container.
                                      Must be an absolute path eg.: /home/snowxavier/Repositories/SNOW_AutoRally
      --data==<myCrazyDataDir>        Host data directory to mount inside the container.
                                      Must be an absolute path eg.: /home/snowxavier/Repositories/wt_data

      --GT-AR                         Project version: Georgia Tech AutoRally refactoring
      --clion                         Build the version to use with CLion IDE (use with the --GT-AR flag)

      --name=xc                       Shortcut: ---name=xavier_red_clion
      --data=jetson                   Shortcut: --volume \"\$HOME/Repositories/wt_data:/mnt/wt_data:ro\"

    Note: you can pass any docker run flag as additional argument eg:
      --rm
      --volume=\"/my/host/path/data:/my/container/path/data\"
      -e DS_HOST_TYPE=XavierWarthog

      ref: https://docs.docker.com/engine/reference/commandline/run/

    Recommandation:
      $ cd ~/my/source/code/dir/
      $ sudo git clone https://github.com/RedLeader962/SNOW_AutoRally.git
      $ cd ~/my/source/code/dir/Dockerized-SNOW
      $ bash run_snow_develop.bash --name=MyCrazyContainer --src=/absolute/path/to/source/code/dir/SNOW_AutoRally
  "
}



USER_ARG=""
HOST_SOURCE_CODE_FLAG=""
HOST_DATA_DIR_FLAG=""
DS_IMAGE_TAG="arm64-l4t-r32.6.1-XavierSA"
IDE="develop"
DS_SUB_PROJECT="norlab-mppi"
DS_SUB_PROJECT_GIT="NorLab_MPPI"


# todo:on task end >> delete next bloc ↓↓
#echo "
#${0}: all arg >> ${@}
#"

for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --GT-AR)
    DS_SUB_PROJECT="gt-autorally"
    DS_SUB_PROJECT_GIT="SNOW_AutoRally"
    shift # Remove --GT-AR from processing
    ;;
  --clion)
    IDE="clion-develop"
    shift # Remove --clion from processing
    ;;
  --name)
    echo "${0} >> pass argument with the equal sign: --name=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --src)
    echo "${0} >> pass argument with the equal sign: --src=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --data)
    echo "${0} >> pass argument with the equal sign: --data=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --name=xc)
    CONTAINER_NAME="xavier_red_clion" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    echo
    echo "new container name: ${CONTAINER_NAME}"
    echo
    ;;
  --name=?*)
    CONTAINER_NAME="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    echo
    echo "new container name: ${CONTAINER_NAME}"
    echo
    ;;
  --runTag)
    echo "${0} >> pass argument with the equal sign: --runTag=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --runTag=?*)
    DS_IMAGE_TAG="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    ;;
#  --src=gtar)
##    WS_DIR="/home/snowxavier/Repositories/SNOW_AutoRally"
#    WS_DIR="${HOME}/Repositories/SNOW_AutoRally"
#    CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/ros_catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
#    WS_DIRNAME=$(basename $WS_DIR)
#    HOST_SOURCE_CODE_FLAG=" --volume ${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
#    echo "Source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
#    ;;
#  --src=nlmppi)
##    WS_DIR="/home/snowxavier/Repositories/NorLab_MPPI"
#    WS_DIR="${HOME}/Repositories/NorLab_MPPI"
#    CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/ros_catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
#    WS_DIRNAME=$(basename $WS_DIR)
#    HOST_SOURCE_CODE_FLAG=" --volume ${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
#    echo "Source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
#    ;;
  --src=?*)
    WS_DIR="${arg#*=}"                                  # Remove every character up to the '=' and assign the remainder
    CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/ros_catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_SOURCE_CODE_FLAG="${HOST_SOURCE_CODE_FLAG} --volume ${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
    echo "Source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
    ;;
  --data=jetson)
    WS_DIR="${HOME}/Repositories/wt_data"
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_DATA_DIR_FLAG="${HOST_DATA_DIR_FLAG} --volume ${WS_DIR}:/mnt/${WS_DIRNAME}:ro"
    echo "Data directory mapping from host to container: ${WS_DIR} >>> /mnt/${WS_DIRNAME}"
  --data=?*)
    WS_DIR="${arg#*=}"                                  # Remove every character up to the '=' and assign the remainder
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_DATA_DIR_FLAG="${HOST_DATA_DIR_FLAG} --volume ${WS_DIR}:/mnt/${WS_DIRNAME}:ro"
    echo "Data directory mapping from host to container: ${WS_DIR} >>> /mnt/${WS_DIRNAME}"
  --)
    shift
    break
    ;;
  -?* | --?*)
    #    echo $0: $1: unrecognized option >&2 # Note: '>&2' = print to stderr
    USER_ARG="${USER_ARG} ${arg}"
    shift # Remove generic argument from processing
    ;;
  *)
    break
    ;;
  esac

  shift
done


if [[ -z $HOST_SOURCE_CODE_FLAG ]]; then
  WS_DIR="${HOME}/Repositories/${DS_SUB_PROJECT_GIT}"
  CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/ros_catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
  WS_DIRNAME=$(basename $WS_DIR)
  HOST_SOURCE_CODE_FLAG=" --volume ${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
  echo "Use default source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
fi

SRC_CODE_REPOSITORY_NAME

echo "
Run container ${DS_SUB_PROJECT}/${IDE} with tag: ${DS_IMAGE_TAG}
"


## todo:on task end >> mute next bloc ↓↓
echo "
${0}:
  DS_SUB_PROJECT >> ${DS_SUB_PROJECT}
  DS_IMAGE_TAG >> ${DS_IMAGE_TAG}
  HOST_SOURCE_CODE_FLAG >> ${HOST_SOURCE_CODE_FLAG}
  HOST_DATA_DIR_FLAG >> ${HOST_DATA_DIR_FLAG}
  USER_ARG >> ${USER_ARG}
"

## todo:assessment (ref task NLSAR-159 Fix the execute permission of source code mounted volume)
#sudo chmod --recursive +x "${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"

export DISPLAY=:0
#echo "export DISPLAY=:0" >> ~/.bashrc

# Note on xhost usage:
#           $ xhost [[+-][family:]name]
#
#   familly:
#     - local:      contains only one name, the empty string
#     - inet:       Internet host (IPv4)
#     - inet6:      Internet host (IPv6)
#     - si:         Server Interpreted : si:<type>:<value>

#sudo xhost +si:localuser:root
sudo xhost + # (Priority) todo:fixme!! (ref task NLSAR-189)



sudo docker run \
  --runtime nvidia \
  --interactive \
  --tty \
  --network host \
  --device=/dev/input/js0 \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  ${HOST_DATA_DIR_FLAG} \
  ${HOST_SOURCE_CODE_FLAG} \
  --security-opt seccomp=unconfined \
  --security-opt apparmor=unconfined \
  --cap-add sys_ptrace \
  ${USER_ARG} \
  norlabsnow/${DS_SUB_PROJECT}/${IDE}:${DS_IMAGE_TAG}

# -p10.0.1.103:2222:22 \
# Change -p10.0.1.7:<host port>:<container port> to your host ip adress
