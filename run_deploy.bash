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

    <optional argument>:
      -h, --help                Get help
      --baseImgTagOW=<thatTag>        Overwrite base image tag  eg.: arm64-l4t-r32.5.0, x86-ubuntu20.04
      --GT-AR                         Build version: Georgia Tech AutoRally refactoring project (default: norlab-mppi)
      --host-type           Use it for container deployed on the Warthog
      --name=<myCoolContainer>  Name that new container, the crazier the better
      --data==<myCrazyDataDir>        Host data directory to mount inside the container.
                                      Must be an absolute path eg.: /home/snowxavier/Repositories/wt_data
      --data=jetson                   Shortcut: --volume \"\$HOME/Repositories/wt_data:/mnt/wt_data:ro\"

    Note: you can pass any docker run flag as additional argument eg:
      --rm
      --volume=\"/my/host/path/data:/my/container/path/data\"

    Ref.: https://docs.docker.com/engine/reference/commandline/run/
  "
}

# --x86                     Get the image version compiled for x86 workstation

USER_ARG=""
DS_IMAGE_TAG="arm64-l4t-r32.6.1"
DS_SUB_PROJECT="norlab-mppi"

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
#    DS_IMAGE_TAG="x86"
#    shift # Remove --x86 from processing
#    ;;
  --host-type)
    USER_ARG="${USER_ARG} -e HOST_TYPE=XavierWarthog"
    shift # Remove --host-type from processing
    ;;
  --GT-AR)
    DS_SUB_PROJECT="gt-autorally"
    shift # Remove --GT-AR from processing
    ;;
  --name)
    echo "${0} >> pass argument with the equal sign: --name=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --data)
    echo "${0} >> pass argument with the equal sign: --data=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --name=?*)
    CONTAINER_NAME="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    USER_ARG="${USER_ARG} --name ${CONTAINER_NAME}"
    echo
    echo "new container name: ${CONTAINER_NAME}"
    echo
    ;;
  --baseImgTagOW)
    echo "${0} >> pass argument with the equal sign: --baseImgTagOW=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --baseImgTagOW=?*)
    DS_IMAGE_TAG="${arg#*=}" # Remove every character up to the '=' and assign the remainder
    echo "Base image tag: ${DS_IMAGE_TAG}"
    ;;
  --data=jetson)
    WS_DIR="${HOME}/Repositories/wt_data"
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_DATA_DIR_FLAG=" --volume ${WS_DIR}:/mnt/${WS_DIRNAME}:ro"
    echo "Data directory mapping from host to container: ${WS_DIR} >>> /mnt/${WS_DIRNAME}"
  --data=?*)
    WS_DIR="${arg#*=}"                                  # Remove every character up to the '=' and assign the remainder
    WS_DIRNAME=$(basename $WS_DIR)
    HOST_DATA_DIR_FLAG=" --volume ${WS_DIR}:/mnt/${WS_DIRNAME}:ro"
    echo "Data directory mapping from host to container: ${WS_DIR} >>> /mnt/${WS_DIRNAME}"
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
  DS_IMAGE_TAG >> ${DS_IMAGE_TAG}
  DS_SUB_PROJECT >> ${DS_SUB_PROJECT}
"

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
sudo xhost + # (Priority) todo:fixme!! (ref task NLSAR-189)

sudo docker run \
  --runtime nvidia \
  --interactive \
  --tty \
  --device=/dev/input/js0 \
  --network host \
  --env DISPLAY=$DISPLAY \
  --privileged \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  ${HOST_DATA_DIR_FLAG} \
  ${USER_ARG} \
  norlabsnow/${DS_SUB_PROJECT}/deploy:${DS_IMAGE_TAG}


