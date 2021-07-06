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
  ${0}  [<optional argument>]

    optional argument:
      -h, --help                Get help
      --x86                     Get the image version compiled for x86 workstation
      --name=<myCoolContainer>  Name that new container, the crazier the better

    Note: you can pass any docker run flag as additional argument eg:
      --rm
      --volume=/my/host/path/data:/my/container/path/data

      ref: https://docs.docker.com/engine/reference/commandline/run/
  "
}

USER_ARG=""
IMAGE_TAG="arm64-l4t"

# todo:on task end >> next bloc ↓↓
echo "
${0}: all arg >> ${@}
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
    echo "${0} >> pass argument with the equal sign: --name=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --name=?*)
    CONTAINER_NAME="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    ;;
  --)
    shift
    break
    ;;
  -?*|--?*)
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

# todo:on task end >> next bloc ↓↓
echo "
${0}:
  USER_ARG >> ${USER_ARG}
  IMAGE_TAG >> ${IMAGE_TAG}
"

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
  ${USER_ARG} \
  norlabsnow/snow-autorally-deploy:${IMAGE_TAG}
