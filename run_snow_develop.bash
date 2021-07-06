#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo -e "
\033[1;90m


                                      '||''|.
                                       ||   ||
                                       ||    ||
                                       ||    ||
                                      .||...|'

                                  (Dockerized-SNOW)

                              https://norlab.ulaval.ca

\033[0m
"

function print_help_in_terminal() {

  echo -e "
  ${0} [<optional argument>]

    optional argument:
      -h, --help                    Get help
      --x86                         Get the image version compiled for x86 workstation
      --name=<myCoolContainer>      Name that new container, the crazier the better
      --src=<myCoolSrcCode>         Host source code directory to mount inside the container.
                                    Must be an absolute path eg.:
                                        /home/snowxavier/Repositories/SNOW-AutoRally

  "
}

USER_ARG=""
HOST_SOURCE_CODE_PATH=""
IMAGE_TAG="arm64-l4t"

# todo:on task end >> next bloc ↓↓
echo "
${0} | all arg >> ${@}
"

for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --x86)
    IMAGE_TAG="x86"
    shift # Remove --x86 from processing
    ;;
  --name)
    echo $0: use --name=${2} instead of --name ${2} >&2  # Note: '>&2' = print to stderr
    exit
    ;;
  --src)
    echo $0: use --src=<myCoolSrcCode> >&2  # Note: '>&2' = print to stderr
    exit
    ;;
  --name=?*)
    CONTAINER_NAME="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    ;;
  --src=?*)
    WS_DIR="${arg#*=}"                                    # Remove every character up to the '=' and assign the remainder
    CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_SOURCE_CODE_PATH=" --volume ${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
    echo "Source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
    ;;
  --)
    shift
    break
    ;;
  -?*)
    echo $0: $1: unrecognized option >&2  # Note: '>&2' = print to stderr
    ;;
  *)
    break
    ;;
  esac

  shift
done

# todo:on task end >> next bloc ↓↓
echo "
${0} |
  USER_ARG >> ${USER_ARG}
  HOST_SOURCE_CODE_PATH >> ${HOST_SOURCE_CODE_PATH}
"

## todo:assessment (ref task NLSAR-159 Fix the execute permission of source code mounted volume)
#sudo chmod --recursive +x "${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"

sudo xhost +si:localuser:root

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
  ${HOST_SOURCE_CODE_PATH} \
  --security-opt seccomp=unconfined \
  ${USER_ARG} \
  norlabsnow/snow-autorally-develop:${IMAGE_TAG}
