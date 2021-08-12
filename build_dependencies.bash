#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

echo -e "
\033[1;2m


                   .|'''.|  '|.   '|'  ..|''||   '|| '||'  '|'
                   ||..  '   |'|   |  .|'    ||   '|. '|.  .'
                    ''|||.   | '|. |  ||      ||   ||  ||  |
                  .     '||  |   |||  '|.     ||    ||| |||
                  |'....|'  .|.   '|   ''|...|'      |   |

                               (Dockerized-SNOW)

                https://github.com/RedLeader962/Dockerized-SNOW
                           https://norlab.ulaval.ca

\033[0m
"


function print_help_in_terminal() {

  echo -e "
  ${0}  [<optional argument>]

    Default compilation: arm64 with Linux for Tegra (L4T) os

    <optional argument>:
      -h, --help                Get help
      --x86                     Get the image version compiled for x86 workstation
      --GT-AR                   Build version: GeogiaTech AutoRally refactoring project

    Note: you can pass any docker build flag as additional argument eg:
      --build-arg=\"ROS_PKG=desktop-full\"
      --build-arg=\"BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.5.0\"

    Ref. docker build command:
      - https://docs.docker.com/engine/reference/commandline/build/
    Ref. nvidia-docker base image for jetson:
      - https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
      - https://developer.nvidia.com/embedded/jetson-cloud-native
    Ref. nvidia-docker base image with CUDA and OpenGL support:
      - https://hub.docker.com/r/nvidia/cudagl/
      - https://github.com/NVIDIA/nvidia-docker/wiki/CUDA
      - https://ngc.nvidia.com/catalog/containers/nvidia:cudagl
  "
}

# Note:
#   nvidia-docker base image for jetson
#     - https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#     - https://developer.nvidia.com/embedded/jetson-cloud-native
#   nvidia-docker base image with CUDA and OpenGL support
#     - https://hub.docker.com/r/nvidia/cudagl/
#     - https://github.com/NVIDIA/nvidia-docker/wiki/CUDA
#     - https://ngc.nvidia.com/catalog/containers/nvidia:cudagl

USER_ARG=""
IMAGE_TAG="arm64-l4t"
BASE_IMG_ARG=""
DS_PROJECT_REPO="NorLab-MPPI"

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}: all arg >> ${@}
#"


for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --x86)
    IMAGE_TAG="x86"
    BASE_IMG_ARG=" --build-arg BASE_IMAGE=nvcr.io/nvidia/cudagl:11.3.1-devel-ubuntu18.04"
    shift # Remove --x86 from processing
    ;;
  --GT-AR)
    DS_PROJECT_REPO="GT-autorally"
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

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}:
#  USER_ARG >> ${USER_ARG}
#  IMAGE_TAG >> ${IMAGE_TAG}
#  BASE_IMG_ARG >> ${BASE_IMG_ARG}
#"

sudo docker build \
  -t norlabsnow/${DS_PROJECT_REPO}-dependencies:${IMAGE_TAG} \
  -f ./Docker/${DS_PROJECT_REPO}/dependencies/Dockerfile \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker