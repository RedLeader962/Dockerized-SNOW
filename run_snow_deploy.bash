#!/bin/bash


#set -e  # exit script if any statement returns a non-true return value


#if [ $# -ne 1 ]; then
#  echo "SNOW-AutoRally | missing argument: $0 <name that container>"
#  exit 1
#fi


#for arg in "$@"; do
#  case $arg in
#  --name=*)
#    CONTAINER_NAME="${arg#*=}"  # Remove every character up to the '=' and assign the remainder
#    shift # Remove --name= from processing
#    ;;
#  *)
#    OTHER_ARGUMENTS+=("$1")
#    shift # Remove generic argument from processing
#    ;;
#  esac
#done

echo -n "Name that new container, the crazier the better: "
read CONTAINER_NAME
echo "starting ${CONTAINER_NAME}"

export LATEST_CONTAINER_USED="${CONTAINER_NAME}"

sudo xhost +si:localuser:root

sudo docker run \
  --name "${CONTAINER_NAME}" \
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
  norlabsnow/snow-autorally-deploy
