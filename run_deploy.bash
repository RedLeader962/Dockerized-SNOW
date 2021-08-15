#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

bash ./visual/terminal_splash.bash

function print_help_in_terminal() {

  echo -e "\$ ${0}  [<optional argument>]


\033[1mDefault setting:\033[0m
- Project: ${DS_SUB_PROJECT}
- Image tag: --runTag=arm64-l4t-r32.6.1-XavierSA
- Host data directory:

\033[1m<optional argument>:\033[0m
  -h, --help                      Get help
  --runTag=<thatTag>     Overwrite image tag eg.: arm64-l4t-r32.6.1-XavierSA-test, x86-ubuntu20.04-gazebo-dart
  --name=<myCoolContainer>        Name that new container, the crazier the better
  --data=<myCrazyDataDir>        Host data directory to mount inside the container.
                                  Must be an absolute path eg.: /home/snowxavier/Repositories/wt_data
  --data=jetson                   Shortcut: --volume \"\$HOME/Repositories/wt_data:/mnt/wt_data:ro\"
  --GT-AR                         Project version: Georgia Tech AutoRally refactoring

\033[1mNote:\033[0m You can pass any docker run flag as additional argument eg:
  --rm
  --volume=\"/my/host/path/data:/my/container/path/data\"

\033[2mRef.: https://docs.docker.com/engine/reference/commandline/run/
\033[0m"
}

USER_ARG=""
HOST_DATA_DIR_FLAG=""
DS_IMAGE_TAG="arm64-l4t-r32.6.1-XavierSA"
IDE="develop"
DS_SUB_PROJECT="norlab-mppi"
DS_TARGET_PROJECT_SRC_REPO="NorLab_MPPI"


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
    DS_TARGET_PROJECT_SRC_REPO="SNOW_AutoRally"
    shift # Remove --GT-AR from processing
    ;;
  --name)
    echo "${0} >> pass argument with the equal sign: --name=${2}" >&2 # Note: '>&2' = print to stderr
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
  --data=jetson)
    WS_DIR="${HOME}/Repositories/wt_data"
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_DATA_DIR_FLAG="${HOST_DATA_DIR_FLAG} --volume ${WS_DIR}:/mnt/${WS_DIRNAME}:ro"
    echo "Data directory mapping from host to container: ${WS_DIR} >>> /mnt/${WS_DIRNAME}"
    ;;
  --data=?*)
    WS_DIR="${arg#*=}"                                  # Remove every character up to the '=' and assign the remainder
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_DATA_DIR_FLAG="${HOST_DATA_DIR_FLAG} --volume ${WS_DIR}:/mnt/${WS_DIRNAME}:ro"
    echo "Data directory mapping from host to container: ${WS_DIR} >>> /mnt/${WS_DIRNAME}"
    ;;
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

echo "
Run container ${DS_SUB_PROJECT}-${IDE} with tag: ${DS_IMAGE_TAG}
"

## todo:on task end >> mute next bloc ↓↓
echo "
${0}:
  DS_SUB_PROJECT >> ${DS_SUB_PROJECT}
  DS_IMAGE_TAG >> ${DS_IMAGE_TAG}
  HOST_DATA_DIR_FLAG >> ${HOST_DATA_DIR_FLAG}
  USER_ARG >> ${USER_ARG}
"
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
  --device=/dev/input/js0 \
  --network host \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  ${HOST_DATA_DIR_FLAG} \
  ${USER_ARG} \
  norlabsnow/${DS_SUB_PROJECT}-deploy:${DS_IMAGE_TAG}


