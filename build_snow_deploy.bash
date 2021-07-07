#!/bin/bash

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
      --build-arg=\"SRC_CODE_DOMAIN_NAME=RedLeader962\"
      --build-arg=\"SRC_CODE_REPOSITORY_NAME=SNOW-AutoRally\"
      --build-arg=\"DEV_BRANCH=SNOW-melodic-devel\"

    Ref. docker build command:
      - https://docs.docker.com/engine/reference/commandline/build/
  "
}


USER_ARG=""
IMAGE_TAG="arm64-l4t"
BASE_IMG_ARG=""

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
    BASE_IMG_ARG=" --build-arg BASE_IMG_TAG=x86"
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
  BASE_IMG_ARG >> ${BASE_IMG_ARG}
"

sudo docker build \
  -t norlabsnow/snow-autorally-deploy:${IMAGE_TAG} \
  -f ./Docker/snow-autorally-deploy/Dockerfile \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker/snow-autorally-deploy