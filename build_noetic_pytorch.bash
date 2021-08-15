#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

bash ./visual/terminal_splash.bash


function print_help_in_terminal() {

  echo -e "\$ ${0}  [<optional argument>]

Default compilation:
- Project: norlab-mppi
- Architecture & OS: arm64 with Linux for Tegra (L4T) os version 32.6.1 (tag: arm64-l4t-r32.6.1)

<optional argument>:
  -h, --help                Get help
  --x86                     Build the image version compiled for x86 workstation instead of arm64-l4t
  --l4t-version=<version>   Build arm64-l4t using an other release version (default: r32.6.1)
  --appendToTag=<detail>    Add suplemental details to the builded image tag eg.: --appendToTag=test

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
DS_IMAGE_TAG="arm64-l4t"
BASE_IMG_VERSION=""
BASE_IMG_ARG=""
DS_SUB_PROJECT="norlab-mppi"
ADD_TO_TAG=""

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
    DS_IMAGE_TAG="x86"
    shift # Remove --x86 from processing
    ;;
  --l4t-version)
    echo "${0} >> pass argument with the equal sign: --l4t-version=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --l4t-version=?*)
    BASE_IMG_VERSION="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo "Base image tag: ${BASE_IMG_VERSION}"
    ;;
  --appendToTag)
    echo "${0} >> pass argument with the equal sign: --appendToTag=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --appendToTag=?*)
    ADD_TO_TAG="${ADD_TO_TAG}${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo
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

# ---Compose image------------------------------------------------------------------------------------------------------
if [[ "$DS_IMAGE_TAG" == "arm64-l4t" ]] && [[ "$DS_SUB_PROJECT" == "norlab-mppi" ]]; then
  if [[ "$BASE_IMG_VERSION" == "" ]]; then
    BASE_IMG_VERSION="r32.6.1"
  fi
  BASE_IMG_ARG=" --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-base:${BASE_IMG_VERSION}"
elif [[ "$DS_IMAGE_TAG" == "x86" ]] && [[ "$DS_SUB_PROJECT" == "norlab-mppi" ]]; then
  BASE_IMG_VERSION="ubuntu20.04"
  BASE_IMG_ARG=" --build-arg BASE_IMAGE=nvcr.io/nvidia/cudagl:11.4.0-devel-${BASE_IMG_VERSION}"
else
  echo  "$DS_SUB_PROJECT is not currently supported"
  exit
fi

# ---Construct image tag------------------------------------------------------------------------------------------------
DS_IMAGE_TAG="${DS_IMAGE_TAG}-${BASE_IMG_VERSION}"

if [[ "$ADD_TO_TAG" != "" ]]; then
  DS_IMAGE_TAG="${DS_IMAGE_TAG}-${ADD_TO_TAG}"
fi

# todo:on task end >> delete next bloc ↓↓
echo "
${0}:
  USER_ARG >> ${USER_ARG}
  DS_IMAGE_TAG >> ${DS_IMAGE_TAG}
  BASE_IMG_ARG >> ${BASE_IMG_ARG}
  BASE_IMG_VERSION >> ${BASE_IMG_VERSION}
  DS_SUB_PROJECT >> ${DS_SUB_PROJECT}
"

# ---Build docker image-------------------------------------------------------------------------------------------------

sudo docker build \
  -t norlabsnow/${DS_SUB_PROJECT}-noetic-pytorch:${DS_IMAGE_TAG} \
  -f ./Docker/${DS_SUB_PROJECT}/noetic-pytorch/Dockerfile \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker/${DS_SUB_PROJECT}/noetic-pytorch
