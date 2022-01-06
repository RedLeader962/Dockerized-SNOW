#!/bin/bash

# Load environment variable from file
set -o allexport; source ds.env; set +o allexport

bash ./visual/terminal_splash.bash

function print_help_in_terminal() {

  echo -e "\$ ${0}  [<optional argument>]

\033[1mDefault compilation:\033[0m
- Project: ${DS_SUB_PROJECT}
- Architecture & OS: arm64 with Linux for Tegra (L4T) os version 32.6.1 (tag: arm64-l4t-r32.6.1)
- Host type: XavierStandAlone compute box

\033[1m<optional argument>:\033[0m
  -h, --help                Get help
  --x86                     Build the image version compiled for x86 workstation instead of arm64-l4t
  --l4t-version=<version>   Build arm64-l4t using an other release version (default: r32.6.1)
  --host-type=<type>        Specified the container host type: XavierStandAlone, XavierWarthog, local
  --dryrun                  Print the docker run command but dont execute it
  --GT-AR                   Project version: Georgia Tech AutoRally refactoring
  --clion                   Build the version to use with CLion IDE (use with the --GT-AR flag)
  --appendToTag=<detail>    Add supplemental details to the built image tag eg.: --appendToTag=test

\033[1mNote:\033[0m You can pass any docker build flag as additional argument eg:
  --build-arg=\"DS_TARGET_PROJECT_SRC_REPO=SNOW_AutoRally\"
  --build-arg=\"BASE_IMG_TAG=arm64-l4t-r32.6.1\"

    Available base img tag: arm64-l4t-r32.5.0, arm64-l4t-r32.6.1, x86-ubuntu18.04, x86-ubuntu20.04

\033[2mRef. docker build command:
  - https://docs.docker.com/engine/reference/commandline/build/
\033[0m"
}

USER_ARG=""
DS_IMAGE_TAG="arm64-l4t"
BASE_IMG_VERSION=""
BASE_IMG_ARG=""
DS_SUB_PROJECT="norlab-mppi"
ADD_TO_TAG=""
IDE="develop"
DS_HOST_TYPE=""
DRY_RUN=false

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
    ADD_TO_TAG="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo
    ;;
  --host-type)
    echo -e "${DS_MSG_ERROR} ${0} >> pass argument with the equal sign: --host-type=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --host-type=?*)
    DS_HOST_TYPE="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo
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

# ---Compose image------------------------------------------------------------------------------------------------------
if [[ "$DS_SUB_PROJECT" == "norlab-mppi" ]]; then
  if [[ "$DS_IMAGE_TAG" == "arm64-l4t" ]]; then
    if [[ "$BASE_IMG_VERSION" == "" ]]; then
      BASE_IMG_VERSION="r32.6.1"
    fi
  elif [[ "$DS_IMAGE_TAG" == "x86" ]]; then
    #  BASE_IMG_VERSION="ubuntu20.04"
    BASE_IMG_VERSION="ubuntu18.04"
  fi

  if [[ "$IDE" == "clion-develop" ]]; then
    echo -e "${DS_MSG_WARNING} Containers for clion development are currently only implemented for the GT-autorally refactoring project."
    echo "         Build ${DS_SUB_PROJECT}-develop image instead"
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
  echo  -e "${DS_MSG_ERROR} $DS_SUB_PROJECT is not currently supported"
  exit
fi

# ---Check legal combinaison: DS_HOST_TYPE vs DS_IMAGE_TAG--------------------------------------------------------------

# Case flag --host-type=* empty
if [[ "$DS_HOST_TYPE" == "" ]]; then
  if [[ "$DS_IMAGE_TAG" == "arm64-l4t" ]]; then
    DS_HOST_TYPE="XavierStandAlone"
  elif [[ "$DS_IMAGE_TAG" == "x86" ]]; then
    DS_HOST_TYPE="local"
  fi
fi

if [[ "$DS_HOST_TYPE" == "local" ]]; then
  if [[ "$DS_IMAGE_TAG" == "x86" ]]; then
    USER_ARG="${USER_ARG} --build-arg DS_HOST_TYPE=${DS_HOST_TYPE}"
    echo "Host type: ${DS_HOST_TYPE}"
  else
    echo -e "${DS_MSG_ERROR} Host type ${DS_HOST_TYPE} is for x86 build only"
    echo
    exit
  fi
elif [[ "$DS_HOST_TYPE" == "XavierStandAlone" || "$DS_HOST_TYPE" == "XavierWarthog" ]]; then
  if [[ "$DS_IMAGE_TAG" == "arm64-l4t" ]]; then
    USER_ARG="${USER_ARG} --build-arg DS_HOST_TYPE=${DS_HOST_TYPE}"
    echo "Host type: ${DS_HOST_TYPE}"
  else
    echo -e "${DS_MSG_ERROR} Host type ${DS_HOST_TYPE} is for arm64-l4t build only"
    echo
    exit
  fi
else
  echo -e "${DS_MSG_ERROR} Host type ${DS_HOST_TYPE} is not currently supported. Choose between: (default) XavierStandAlone, XavierWarthog or local"
  echo
  exit
fi

# ---Construct image tag------------------------------------------------------------------------------------------------
DS_IMAGE_TAG="${DS_IMAGE_TAG}-${BASE_IMG_VERSION}"
BASE_IMG_ARG=" --build-arg BASE_IMG_TAG=${DS_IMAGE_TAG}"

if [[ "$DS_HOST_TYPE" == "local" ]]; then
  :
elif [[ "$DS_HOST_TYPE" == "XavierStandAlone" ]]; then
  DS_IMAGE_TAG="${DS_IMAGE_TAG}-XavierSA"
elif [[ "$DS_HOST_TYPE" == "XavierWarthog" ]]; then
  DS_IMAGE_TAG="${DS_IMAGE_TAG}-XavierWT"
fi

if [[ "$ADD_TO_TAG" != "" ]]; then
  DS_IMAGE_TAG="${DS_IMAGE_TAG}-${ADD_TO_TAG}"
fi


if [ $DRY_RUN == true ]; then
  echo -e "
  ${DS_MSG_EMPH_FORMAT}${0} dry run${DS_MSG_END_FORMAT}:
  sudo docker build -t norlabsnow/${DS_SUB_PROJECT}-${IDE}:${DS_IMAGE_TAG} -f ./Docker/${DS_SUB_PROJECT}/${IDE}/Dockerfile ${BASE_IMG_ARG} ${USER_ARG} ./Docker/${DS_SUB_PROJECT}/${IDE}

  ${DS_MSG_EMPH_FORMAT}${0} TeamCity docker image counterpart dry run${DS_MSG_END_FORMAT}:
  sudo docker build -t norlabsnow/${DS_SUB_PROJECT}-${IDE}-teamcity:${DS_IMAGE_TAG} -f ./Docker/${DS_SUB_PROJECT}/teamcity/Dockerfile --build-arg BASE_IMG="${DS_SUB_PROJECT}-${IDE}" ${BASE_IMG_ARG} ${USER_ARG} ./Docker/${DS_SUB_PROJECT}/${IDE}
  "
  exit
fi

# ---Build docker image-------------------------------------------------------------------------------------------------
sudo docker build \
  -t norlabsnow/${DS_SUB_PROJECT}-${IDE}:${DS_IMAGE_TAG} \
  -f ./Docker/${DS_SUB_PROJECT}/${IDE}/Dockerfile \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker/${DS_SUB_PROJECT}/${IDE}

# ---Build TeamCity docker image counterpart----------------------------------------------------------------------------
sudo docker build \
  -t norlabsnow/${DS_SUB_PROJECT}-${IDE}-teamcity:${DS_IMAGE_TAG} \
  -f ./Docker/${DS_SUB_PROJECT}/teamcity/Dockerfile \
  --build-arg BASE_IMG="${DS_SUB_PROJECT}-dependencies-wo-services" \
  ${BASE_IMG_ARG} \
  ${USER_ARG} \
  ./Docker
