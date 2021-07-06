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

    Default compilation: arm64 with Linux for Tegra (L4T) os

    optional argument:
      -h, --help                Get help
      --x86                     Get the image version compiled for x86 workstation

    Note: you can pass any docker build flag as additional argument eg:
      --build-arg ROS_PKG=desktop-full
      --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.5.0

      ref:
       - https://docs.docker.com/engine/reference/commandline/build/
  "
}

# Note:
#   NVIDIA base image with CUDA and OpenGL support
#   - https://hub.docker.com/r/nvidia/cudagl/
#   - https://github.com/NVIDIA/nvidia-docker/wiki/CUDA
#   - https://ngc.nvidia.com/catalog/containers/nvidia:cudagl

USER_ARG=""
IMAGE_TAG="arm64-l4t"

# todo:on task end >> delete next bloc ↓↓
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
    BASE_IMG_ARG=" --build-arg BASE_IMAGE=nvcr.io/nvidia/11.3.1-devel-ubuntu18.04"
    shift # Remove --x86 from processing
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

# todo:on task end >> delete next bloc ↓↓
echo "
${0}:
  USER_ARG >> ${USER_ARG}
  IMAGE_TAG >> ${IMAGE_TAG}
"

sudo docker build \
  -t norlabsnow/ros-melodic-snow-autorally-dependencies:${IMAGE_TAG} \
  -f ./Docker/ros-melodic-snow-autorally-dependencies/Dockerfile \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker