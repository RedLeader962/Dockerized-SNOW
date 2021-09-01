#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

# Load environment variable from file
set -o allexport; source .env; set +o allexport

bash ./visual/terminal_splash.bash


function print_help_in_terminal() {

  echo -e "\$ ${0}  [<optional argument>]

\033[1mDefault compilation:\033[0m
- Project: ${DS_SUB_PROJECT}
- Architecture & OS: arm64 with Linux for Tegra (L4T) os version 32.6.1 (tag: arm64-l4t-r32.6.1)

\033[1m<optional argument>:\033[0m
  -h, --help                Get help
  --x86                     Build the image version compiled for x86 workstation instead of arm64-l4t
  --l4t-version=<version>   Build arm64-l4t using an other release version (default: r32.6.1)
  --appendToTag=<detail>    Add suplemental details to the builded image tag eg.: --appendToTag=test
  --dryrun                  Print the docker run command but dont execute it

\033[1mNote:\033[0m You can pass any docker build flag as additional argument eg:
  --build-arg=\"DS_ROS_PKG=desktop-full\"
  --build-arg=\"BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.5.0\"

\033[2mRef. docker build command:
  - https://docs.docker.com/engine/reference/commandline/build/
Ref. nvidia-docker base image for jetson:
  - https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
  - https://developer.nvidia.com/embedded/jetson-cloud-native
Ref. nvidia-docker base image with CUDA and OpenGL support:
  - https://hub.docker.com/r/nvidia/cudagl/
  - https://github.com/NVIDIA/nvidia-docker/wiki/CUDA
  - https://ngc.nvidia.com/catalog/containers/nvidia:cudagl
\033[0m"
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
DRY_RUN=false

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
    DS_IMAGE_TAG="x86"
    shift # Remove --x86 from processing
    ;;
  --dryrun)
    DRY_RUN=true
    shift # Remove --dryrun from processing
    ;;
  --l4t-version)
    echo -e "${DS_MSG_ERROR} ${0} >> pass argument with the equal sign: --l4t-version=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --l4t-version=?*)
    BASE_IMG_VERSION="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo "Base image tag: ${BASE_IMG_VERSION}"
    ;;
  --appendToTag)
    echo -e "${DS_MSG_ERROR} ${0} >> pass argument with the equal sign: --appendToTag=${2}" >&2 # Note: '>&2' = print to stderr
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
  #  BASE_IMG_VERSION="ubuntu20.04"
  BASE_IMG_VERSION="ubuntu18.04"
  BASE_IMG_ARG=" --build-arg BASE_IMAGE=nvcr.io/nvidia/cudagl:11.4.0-devel-${BASE_IMG_VERSION}"
else
  echo -e "${DS_MSG_ERROR} $DS_SUB_PROJECT is not currently supported"
  exit
fi

DS_IMAGE_ARCHITECTURE="${DS_IMAGE_TAG}"
USER_ARG="${USER_ARG} --build-arg DS_IMAGE_ARCHITECTURE=${DS_IMAGE_ARCHITECTURE}"


# ---Construct image tag------------------------------------------------------------------------------------------------
DS_IMAGE_TAG="${DS_IMAGE_TAG}-${BASE_IMG_VERSION}"

if [[ "$ADD_TO_TAG" != "" ]]; then
  DS_IMAGE_TAG="${DS_IMAGE_TAG}-${ADD_TO_TAG}"
fi

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}:
#  USER_ARG >> ${USER_ARG}
#  DS_IMAGE_TAG >> ${DS_IMAGE_TAG}
#  BASE_IMG_ARG >> ${BASE_IMG_ARG}
#  BASE_IMG_VERSION >> ${BASE_IMG_VERSION}
#  DS_SUB_PROJECT >> ${DS_SUB_PROJECT}
#"


if [ $DRY_RUN == true ]; then
  echo -e "${DS_MSG_EMPH_FORMAT}${0} dry run${DS_MSG_END_FORMAT}:
  sudo docker build -t norlabsnow/${DS_SUB_PROJECT}-ros-melodic-python3:${DS_IMAGE_TAG} -f ./Docker/${DS_SUB_PROJECT}/ros-melodic-python3/Dockerfile ${BASE_IMG_ARG} ${USER_ARG} ./Docker/${DS_SUB_PROJECT}/ros-melodic-python3
  "
  exit
fi

# ---Build docker image-------------------------------------------------------------------------------------------------

sudo docker build \
  -t norlabsnow/${DS_SUB_PROJECT}-ros-melodic-python3:${DS_IMAGE_TAG} \
  -f ./Docker/${DS_SUB_PROJECT}/ros-melodic-python3/Dockerfile \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker/${DS_SUB_PROJECT}/ros-melodic-python3