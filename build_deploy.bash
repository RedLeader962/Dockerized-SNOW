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

    <optional argument>:
      -h, --help                Get help
      --baseImgTag=<theMarvelousTag>  The base image tag to use eg.: arm64-l4t-r32.5.0, x86-ubuntu20.04
      --GT-AR                         Build version: Georgia Tech AutoRally refactoring project (default: NorLab-MPPI)
      --XavierWarthog           Use it for container deployed on the Warthog

    Default compilation: arm64 with Linux for Tegra (L4T) os

    Note: you can pass any docker build flag as additional argument eg:
      --build-arg=\"SRC_CODE_DOMAIN_NAME=RedLeader962\"
      --build-arg=\"SRC_CODE_REPOSITORY_NAME=SNOW_AutoRally\"
      --build-arg=\"DEV_BRANCH=SNOW-melodic-devel\"

    Ref. docker build command:
      - https://docs.docker.com/engine/reference/commandline/build/
  "
}

#      --x86                     Get the image version compiled for x86 workstation

USER_ARG=""
IMAGE_TAG="arm64-l4t-r32.6.1"
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

#  --x86)
#    IMAGE_TAG="x86"
#    BASE_IMG_ARG=" --build-arg BASE_IMG_TAG=x86"
#    shift # Remove --x86 from processing
#    ;;
  --XavierWarthog)
    IMAGE_TAG="${IMAGE_TAG}-XavierWarthog"
    USER_ARG="${USER_ARG} --build-arg HOST_TYPE=XavierWarthog"
    shift # Remove --XavierWarthog from processing
    ;;
  --GT-AR)
    DS_PROJECT_REPO="GT-autorally"
    shift # Remove --GT-AR from processing
    ;;
  --baseImgTag)
    echo "${0} >> pass argument with the equal sign: --baseImgTag=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --baseImgTag=?*)
    IMAGE_TAG="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo "Base image tag: ${IMAGE_TAG}"
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

# todo:on task end >> delete next bloc ↓↓
echo "
${0}:
  USER_ARG >> ${USER_ARG}
  IMAGE_TAG >> ${IMAGE_TAG}
  BASE_IMG_ARG >> ${BASE_IMG_ARG}
  DS_PROJECT_REPO >> ${DS_PROJECT_REPO}
"

sudo docker build \
  -t norlabsnow/${DS_PROJECT_REPO}/deploy:${IMAGE_TAG} \
  -f ./Docker/${DS_PROJECT_REPO}/deploy/Dockerfile \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker/${DS_PROJECT_REPO}/deploy
