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
  ${0} [<optional argument>]

    <optional argument>:
      -h, --help                    Get help
      --x86                         Get the image version compiled for x86 workstation
      --name=<myCoolContainer>      Name that new container, the crazier the better
      --src=<myCoolSrcCode>         Host source code directory to mount inside the container.
                                    Must be an absolute path eg.:
                                        /home/snowxavier/Repositories/SNOW-AutoRally
      --src=xsa                     Shortcut: ---src=/home/snowxavier/Repositories/SNOW-AutoRally
      --clion                       Run the version to use with CLion IDE

    Note: you can pass any docker run flag as additional argument eg:
      --rm
      --volume=\"/my/host/path/data:/my/container/path/data\"

      ref: https://docs.docker.com/engine/reference/commandline/run/

    Recommandation:
      $ cd ~/my/source/code/dir/
      $ sudo git clone https://github.com/RedLeader962/SNOW-AutoRally.git
      $ cd ~/my/source/code/dir/Dockerized-SNOW
      $ bash run_snow_develop.bash --name=MyCrazyContainer --src=/absolute/path/to/source/code/dir/SNOW-AutoRally
  "
}

USER_ARG=""
HOST_SOURCE_CODE_PATH=""
IMAGE_TAG="arm64-l4t"
IDE="develop"

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
  --x86)
    IMAGE_TAG="x86"
    shift # Remove --x86 from processing
    ;;
  --clion)
    IDE="clion-develop"
    shift # Remove --clion from processing
    ;;
  --name)
    echo "${0} >> pass argument with the equal sign: --name=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --src)
    echo "${0} >> pass argument with the equal sign: --src=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --name=?*)
    CONTAINER_NAME="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    ;;
  --src=xsa)
    WS_DIR="/home/snowxavier/Repositories/SNOW-AutoRally"
    CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_SOURCE_CODE_PATH=" --volume ${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
    echo "Source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
    ;;
  --src=?*)
    WS_DIR="${arg#*=}"                                    # Remove every character up to the '=' and assign the remainder
    CONTAINER_SIDE_HOST_SRC_CODE_VOLUME="/catkin_ws/src/" # (Priority) todo:refactor >> this line ← make it global
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_SOURCE_CODE_PATH=" --volume ${WS_DIR}:${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
    echo "Source code mapping from host to container: ${WS_DIR} >>> ${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"
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

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}:
#  USER_ARG >> ${USER_ARG}
#  HOST_SOURCE_CODE_PATH >> ${HOST_SOURCE_CODE_PATH}
#  IMAGE_TAG >> ${IMAGE_TAG}
#"

## todo:assessment (ref task NLSAR-159 Fix the execute permission of source code mounted volume)
#sudo chmod --recursive +x "${CONTAINER_SIDE_HOST_SRC_CODE_VOLUME}${WS_DIRNAME}"

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
sudo xhost +si:localuser:root

sudo docker run \
  --runtime nvidia \
  --interactive \
  --tty \
  --network host \
  --device=/dev/input/js0 \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  ${HOST_SOURCE_CODE_PATH} \
  --security-opt seccomp=unconfined \
  --security-opt apparmor=unconfined \
  --cap-add sys_ptrace \
  ${USER_ARG} \
  norlabsnow/snow-autorally-${IDE}:${IMAGE_TAG}

#  --hostname snowxavier-dev \
# -td
# -p10.0.1.103:2222:22 \
# Change -p10.0.1.7:<host port>:<container port> to your host ip adress

