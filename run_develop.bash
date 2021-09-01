#!/bin/bash

#set -e  # exit script if any statement returns a non-true return value

# Load environment variable from file
set -o allexport; source .env; set +o allexport

bash ./visual/terminal_splash.bash

function print_help_in_terminal() {

  echo -e "\$ ${0} --name=<myCoolContainer> [<optional argument>]

\033[1mDefault setting:\033[0m
- Project: ${DS_SUB_PROJECT}
- Image tag: --runTag=${DS_IMAGE_TAG}
- Host source code directory: --src=${HOME}/Repositories/NorLab_MPPI
- Host data directory: none

\033[1mRequired argument:\033[0m
  --name=<myCoolContainer>        Name that new container, the crazier the better

\033[1m<optional argument>:\033[0m
  -h, --help                      Get help
  --runTag=<thatTag>              Overwrite default image tag eg.: x86-ubuntu18.04-gazebo-dart
  --src=<myCoolSrcCode>           Host source code directory to mount inside the container.
                                  Must be an absolute path eg.: /home/snowxavier/Repositories/${DS_TARGET_PROJECT_SRC_REPO}
  --data=<myCrazyDataDir>         Host data directory to mount inside the container.
                                  Must be an absolute path eg.: /home/snowxavier/Repositories/wt_data
  --dryrun                        Print the docker run command but dont execute it
  --osx                           Mac osX configuration: switch network flag to bridge and explicitly publish container port

  --GT-AR                         Project version: Georgia Tech AutoRally refactoring
  --clion                         Build the version to use with CLion IDE (use with the --GT-AR flag)

  --data=jetson                   Shortcut: --volume=\"\$HOME/Repositories/wt_data:/mnt/wt_data:ro\"

\033[1mNote:\033[0m You can pass any docker run flag as additional argument eg:
  --rm
  --volume=\"/my/host/path/data:/my/container/path/data\"
  -e DS_HOST_TYPE=XavierWarthog

\033[2mRef: https://docs.docker.com/engine/reference/commandline/run/
\033[0m
\033[1mRecommandation on x86 host:\033[0m
  $ mkdir -p ~/Repositories && cd ~/Repositories
  $ sudo git clone https://github.com/norlab-ulaval/NorLab_MPPI.git
  $ sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git
  $ cd ~/Repositories/Dockerized-SNOW

  $ sudo docker pull norlabsnow/norlab-mppi-develop:x86-ubuntu18.04
  $ bash run_develop.bash --runTag=x86-ubuntu18.04 --name=IamSnow --src=\"\$HOME/Repositories/${DS_TARGET_PROJECT_SRC_REPO}\"

"
}



USER_ARG=""
HOST_SOURCE_CODE_FLAG=""
HOST_DATA_DIR_FLAG=""
HOST_SRC_PATH=""
HOST_DATA_PATH=""
CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/ros_catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
DS_IMAGE_TAG="arm64-l4t-r32.6.1-XavierSA"
IDE="develop"
DS_SUB_PROJECT="norlab-mppi"
DS_TARGET_PROJECT_SRC_REPO="NorLab_MPPI"
# alt repo: SNOW_AutoRally
DRY_RUN=false
OSX=false
NAMED=false


# todo:on task end >> delete next bloc ↓↓
#echo "
#${0}: all arg >> ${@}
#"

for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --GT-AR)
    DS_SUB_PROJECT="gt-autorally"
    DS_TARGET_PROJECT_SRC_REPO="SNOW_AutoRally"
    shift # Remove --GT-AR from processing
    ;;
  --clion)
    IDE="clion-develop"
    shift # Remove --clion from processing
    ;;
  --dryrun)
    DRY_RUN=true
    shift # Remove --dryrun from processing
    ;;
  --osx)
    OSX=true
    shift # Remove --osx from processing
    ;;
  --name)
    echo -e "${DS_MSG_ERROR} ${0} >> pass argument with the equal sign: --name=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --src)
    echo -e "${DS_MSG_ERROR} ${0} >> pass argument with the equal sign: --src=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --data)
    echo -e "${DS_MSG_ERROR} ${0} >> pass argument with the equal sign: --data=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --name=xc)
    CONTAINER_NAME="xavier_red_clion" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    NAMED=true
    echo
    ;;
  --name=?*)
    CONTAINER_NAME="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    NAMED=true
    echo
    ;;
  --runTag)
    echo -e "${DS_MSG_ERROR} ${0} >> pass argument with the equal sign: --runTag=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --runTag=?*)
    DS_IMAGE_TAG="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    ;;
  --src=?*)
    HOST_SRC_PATH="${arg#*=}"                                  # Remove every character up to the '=' and assign the remainder
    if [[ -d ${HOST_SRC_PATH} ]]; then
      SRC_BASENAME=$(basename ${HOST_SRC_PATH})
      HOST_SOURCE_CODE_FLAG="${HOST_SOURCE_CODE_FLAG} --volume ${HOST_SRC_PATH}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${SRC_BASENAME}"
      echo "Source code mapping from host to container: ${HOST_SRC_PATH} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${SRC_BASENAME}"
    else
      echo -e "${DS_MSG_ERROR} Be advise, the ${DS_MSG_ERROR_FORMAT}${DS_TARGET_PROJECT_SRC_REPO} source code is unreachable with given path ${HOST_SRC_PATH}${DS_MSG_END_FORMAT}. Make sure you have cloned the ${DS_TARGET_PROJECT_SRC_REPO}.git repository prior to running ${0} then provide it's absolute path to ${0} using ${DS_MSG_EMPH_FORMAT}--src=/absolute/path/to/source/code/dir/${DS_TARGET_PROJECT_SRC_REPO}${DS_MSG_END_FORMAT}"
      echo
      exit
    fi
    ;;
  --data=jetson)
    HOST_DATA_PATH="${HOME}/Repositories/wt_data"
    DATA_BASENAME=$(basename $HOST_DATA_PATH)
    HOST_DATA_DIR_FLAG="${HOST_DATA_DIR_FLAG} --volume ${HOST_DATA_PATH}:/mnt/${DATA_BASENAME}:ro"
    echo "Data directory mapping from host to container: ${HOST_DATA_PATH} >>> /mnt/${DATA_BASENAME}"
    ;;
  --data=?*)
    HOST_DATA_PATH="${arg#*=}"                                  # Remove every character up to the '=' and assign the remainder
    DATA_BASENAME=$(basename $HOST_DATA_PATH)
    HOST_DATA_DIR_FLAG="${HOST_DATA_DIR_FLAG} --volume ${HOST_DATA_PATH}:/mnt/${DATA_BASENAME}:ro"
    echo "Data directory mapping from host to container: ${HOST_DATA_PATH} >>> /mnt/${DATA_BASENAME}"
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


#printenv | grep -e DS_MSG_ERROR -e DS_
if [[ "${NAMED}" == "false" ]]; then
    echo -e "${DS_MSG_ERROR} Please name your container with a meaningful name using the ${DS_MSG_EMPH_FORMAT}--name=<myCoolContainer>${DS_MSG_END_FORMAT} flag. The crazier the better!"
    echo
    exit
fi

# Set default source code location if user did not use the --src=<myCoolSrcCode> flag.
if [[ -z $HOST_SOURCE_CODE_FLAG ]]; then
  DEFAULT_HOST_SRC_PATH="${HOME}/Repositories/${DS_TARGET_PROJECT_SRC_REPO}"

  if [[ -d ${DEFAULT_HOST_SRC_PATH} ]]; then
    SRC_BASENAME=$(basename ${DEFAULT_HOST_SRC_PATH})
    HOST_SOURCE_CODE_FLAG=" --volume ${DEFAULT_HOST_SRC_PATH}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${SRC_BASENAME}"
    echo -e "Using ${DS_MSG_EMPH_FORMAT}default source code mapping${DS_MSG_END_FORMAT} from host to container: ${DS_MSG_EMPH_FORMAT}${DEFAULT_HOST_SRC_PATH} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${SRC_BASENAME}${DS_MSG_END_FORMAT}"
  else
    echo -e "${DS_MSG_ERROR} Be advise, the ${DS_MSG_ERROR_FORMAT}${DS_TARGET_PROJECT_SRC_REPO} source code is unreachable with given path ${DEFAULT_HOST_SRC_PATH}${DS_MSG_END_FORMAT}. Make sure you have cloned the ${DS_TARGET_PROJECT_SRC_REPO}.git repository prior to running ${0} then provide it's absolute path to ${0} using ${DS_MSG_EMPH_FORMAT}--src=/absolute/path/to/source/code/dir/${DS_TARGET_PROJECT_SRC_REPO}${DS_MSG_END_FORMAT}"
    echo
    exit
  fi
fi

# Pass the target project repository basename to the docker container envenrionment variable
USER_ARG="${USER_ARG} -e DS_TARGET_PROJECT_SRC_REPO=${DS_TARGET_PROJECT_SRC_REPO}"

echo -e "
Create container instance from docker image ${DS_SUB_PROJECT}-${IDE} with tag ${DS_IMAGE_TAG}
New container name: ${DS_MSG_EMPH_FORMAT}${CONTAINER_NAME}${DS_MSG_END_FORMAT}
"


### todo:on task end >> mute next bloc ↓↓
#echo "
#${0}:
#  DS_SUB_PROJECT >> ${DS_SUB_PROJECT}
#  DS_IMAGE_TAG >> ${DS_IMAGE_TAG}
#  HOST_SOURCE_CODE_FLAG >> ${HOST_SOURCE_CODE_FLAG}
#  HOST_DATA_DIR_FLAG >> ${HOST_DATA_DIR_FLAG}
#  USER_ARG >> ${USER_ARG}
#"



## todo:assessment (ref task NLSAR-159 Fix the execute permission of source code mounted volume)
#sudo chmod --recursive +x "${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${SRC_BASENAME}"

# todo:investigate?? >> The next line cause error on other os
#DS_HOST_IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')

#export DISPLAY=:0
export DISPLAY=:0
#echo "export DISPLAY=:0" >> ~/.bashrc

# Note on xhost usage:
#           $ xhost [[+-][family:]name]
#
#   familly:
#     - local:      contains only one name, the empty string
#     - inet:       Internet host (IPv4)
#     - inet6:      Internet host (IPv6)
#     - si:         Server Interpreted : si:<type>:<value>

#sudo xhost +si:localuser:root
sudo xhost +  # (Priority) todo:fixme!! (ref task NLSAR-189)


RUNTIME_FLAG="--runtime nvidia "
NETWORK_FLAG="--network host "
if [ $OSX == true ]; then
  # 80 --> default ros master port
  # 11311 --> Warthog ros master
  # 22 --> ssh
  # 7777 --> gdbserver
  # 5900 --> vnc
  # 0-1023 --> well-know ports
  # ⚠️ | Be advise, you cannot ping from a container outside: "Docker Desktop for Mac can’t route traffic to containers."
  #     https://docs.docker.com/docker-for-mac/networking/
  RUNTIME_FLAG=""
  NETWORK_FLAG="--network=bridge \
  --publish=11311:11311 --publish=2222:22 --publish=5900:5900  --publish=80:80 \
  "
#  --publish=7777:7777
#  USER_ARG="${USER_ARG} --env='DISPLAY=${DS_HOST_IP}:0' "
fi

if [ $DRY_RUN == true ]; then
  echo -e "${DS_MSG_EMPH_FORMAT}${0} dry run${DS_MSG_END_FORMAT}:
  sudo docker run ${RUNTIME_FLAG} --interactive --tty ${NETWORK_FLAG} --device=/dev/input/js0 --env DISPLAY=$DISPLAY --privileged --volume "/tmp/.X11-unix/:/tmp/.X11-unix" --volume \"/etc/localtime:/etc/localtime:ro\" ${HOST_DATA_DIR_FLAG} ${HOST_SOURCE_CODE_FLAG} --security-opt seccomp=unconfined --security-opt apparmor=unconfined --cap-add sys_ptrace --hostname "${CONTAINER_NAME}" ${USER_ARG} norlabsnow/${DS_SUB_PROJECT}-${IDE}:${DS_IMAGE_TAG}
  "
  exit
fi



sudo docker run \
  ${RUNTIME_FLAG} \
  --interactive \
  --tty \
  ${NETWORK_FLAG} \
  --device=/dev/input/js0 \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --env="QT_X11_NO_MITSHM=1" \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  ${HOST_DATA_DIR_FLAG} \
  ${HOST_SOURCE_CODE_FLAG} \
  --security-opt seccomp=unconfined \
  --security-opt apparmor=unconfined \
  --cap-add sys_ptrace \
  --hostname ${CONTAINER_NAME} \
  ${USER_ARG} \
  norlabsnow/${DS_SUB_PROJECT}-${IDE}:${DS_IMAGE_TAG}

# -p10.0.1.103:2222:22 \
# Change -p10.0.1.7:<host port>:<container port> to your host ip adress
