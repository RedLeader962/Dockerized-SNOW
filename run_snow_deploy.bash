#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

#if [ $# -ne 1 ]; then
#  echo "SNOW-AutoRally | missing argument: $0 <name that container>"
#  exit 1
#fi




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

function print_help_in_terminal () {

  echo -e "
  run_snow_deploy.bash [<optional argument>]

    optional argument:
      -h, --help                Get help
      --x86                     Get the image version compiled for x86 workstation
      --name <myCoolContainer>  Name that new container, the crazier the better

  "
}

USER_ARG=""
IMAGE_TAG="arm64-l4t"

for arg in "$@"; do
  case $arg in
  -h|--help)
    print_help_in_terminal
    exit
    ;;
  --x86)
    IMAGE_TAG="x86"
    shift # Remove --x86 from processing
    ;;
  --name=*)
    CONTAINER_NAME="${arg#*=}"  # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    shift # Remove --name= from processing
    ;;
  *)
    USER_ARG="${USER_ARG} $1"
    shift # Remove generic argument from processing
    ;;
  esac
done

#echo -n "which image tag? arm64-l4t or x86"
#read IMAGE_TAG
#
#echo -n "Name that new container, the crazier the better: "
#read CONTAINER_NAME
#echo "starting ${CONTAINER_NAME}"
#
#export LATEST_CONTAINER_USED="${CONTAINER_NAME}"

sudo xhost +si:localuser:root

sudo docker run \
  --runtime nvidia \
  --interactive \
  --tty \
  --rm \
  --device=/dev/input/js0 \
  --network host \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  ${USER_ARG} \
  norlabsnow/snow-autorally-deploy:${IMAGE_TAG}