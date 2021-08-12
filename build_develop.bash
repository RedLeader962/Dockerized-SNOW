#!/bin/bash

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

    Default compilation:
    - norlab-mppi project
    - arm64 with Linux for Tegra (L4T) os version 32.6.1 (tag: arm64-l4t-r32.6.1)

    <optional argument>:
      -h, --help                Get help
      --x86                     Build the image version compiled for x86 workstation instead of arm64-l4t
      --l4t-version=<version>   Build arm64-l4t using an other release version (default: r32.6.1)
      --GT-AR                   Project version: Georgia Tech AutoRally refactoring
      --host-type=<type>        Specified the container host type: (default) XavierStandAlone, XavierWarthog, local
      --clion                   Build the version to use with CLion IDE (use with the --GT-AR flag)
      --appendToTag=<detail>    Add supplemental details to the built image tag eg.: --appendToTag=test

    Note: you can pass any docker build flag as additional argument eg:
      --build-arg=\"SRC_CODE_REPOSITORY_NAME=SNOW_AutoRally\"
      --build-arg BASE_IMG_TAG=\"arm64-l4t-r32.6.1\"

        Available base img tag: arm64-l4t-r32.5.0, arm64-l4t-r32.6.1, x86-ubuntu18.04, x86-ubuntu20.04

    Ref. docker build command:
      - https://docs.docker.com/engine/reference/commandline/build/
  "
}

USER_ARG=""
DS_IMAGE_TAG="arm64-l4t"
BASE_IMG_VERSION=""
BASE_IMG_ARG=""
DS_SUB_PROJECT="norlab-mppi"
ADD_TO_TAG=""
IDE="develop"
HOST_TYPE=""

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
  --GT-AR)
    DS_SUB_PROJECT="gt-autorally"
    shift # Remove --GT-AR from processing
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
    ADD_TO_TAG="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo
    ;;
  --host-type)
    HOST_TYPE="XavierWarthog"
    USER_ARG="${USER_ARG} --build-arg HOST_TYPE=XavierWarthog"
    shift # Remove --host-type from processing
    ;;
  --clion)
    IDE="clion-develop"
    shift # Remove --clion from processing
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


if [[ "$DS_SUB_PROJECT" == "norlab-mppi" ]]; then

  if [[ "$DS_IMAGE_TAG" == "arm64-l4t" ]]; then
    if [[ "$BASE_IMG_VERSION" == "" ]]; then
      BASE_IMG_VERSION="r32.6.1"
    fi
  elif [[ "$DS_IMAGE_TAG" == "x86" ]]; then
    BASE_IMG_VERSION="ubuntu20.04"
  fi

  if [[ "$IDE" == "clion-develop" ]]; then
    echo "Warning: clion-develop image is currently only implemented for the GT-autorally refactoring project"
    IDE="develop"
  fi

elif [[ "$DS_SUB_PROJECT" == "gt-autorally" ]]; then

  if [[ "$DS_IMAGE_TAG" == "arm64-l4t" ]]; then
    if [[ "$BASE_IMG_VERSION" == "" ]]; then
      BASE_IMG_VERSION="r32.5.0"
    fi
  elif [[ "$DS_IMAGE_TAG" == "x86" ]]; then
    BASE_IMG_VERSION="ubuntu18.04"
  fi

else
  echo  "$DS_SUB_PROJECT is not currently supported"
  exit
fi

DS_IMAGE_TAG="${DS_IMAGE_TAG}-${BASE_IMG_VERSION}"
BASE_IMG_ARG=" --build-arg BASE_IMG_TAG=${DS_IMAGE_TAG}"

if [[ "$ADD_TO_TAG" != "" ]]; then
  DS_IMAGE_TAG="${DS_IMAGE_TAG}-${ADD_TO_TAG}"
fi

# todo:on task end >> delete next bloc ↓↓
echo "
${0}:
  USER_ARG >> ${USER_ARG}
  DS_IMAGE_TAG >> ${DS_IMAGE_TAG}
  BASE_IMG_ARG >> ${BASE_IMG_ARG}
  DS_SUB_PROJECT >> ${DS_SUB_PROJECT}
  BASE_IMG_VERSION >> ${BASE_IMG_VERSION}
  IDE >> ${IDE}
  ADD_TO_TAG >> ${ADD_TO_TAG}
"

# (CRITICAL) todo:on task end >> unmute next bloc ↓↓
#sudo docker build \
#  -t norlabsnow/${DS_SUB_PROJECT}-${IDE}:${DS_IMAGE_TAG} \
#  -f ./Docker/${DS_SUB_PROJECT}/${IDE}/Dockerfile \
#  ${BASE_IMG_ARG} \
#  ${USER_ARG} \
#  ./Docker/${DS_SUB_PROJECT}-${IDE}
