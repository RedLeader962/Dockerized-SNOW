#!/bin/bash

# Load environment variable from file
set -o allexport; source .env; set +o allexport


bash ./visual/terminal_splash.bash

function print_help_in_terminal() {

  echo -e "\$ ${0} [<optional argument>] <CONTAINER_NAMES>

Open a new interactive terminal with pseudo-TTY

\033[1m<optional argument>:\033[0m
  -h, --help                Get help

\033[1mNote:\033[0m You can pass any docker build flag in <optional argument> eg.:
  --env=\"VAR=1\"        (to set environment variables)

\033[2mRef. docker exec command:
  - https://docs.docker.com/engine/reference/commandline/exec/
\033[0m"
}

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}: all arg >> ${@}
#"

if [ $# -ne 1 ]; then
  echo -e "${DS_MSG_ERROR} missing argument: $0 <CONTAINER_NAMES>
  If your not sure, run in terminal
        $ docker ps -a
  and check the STATUS column to see running container"
  exit 1
fi

CONTAINER_NAMES=""
USER_ARG=""

for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
  --)
    shift
    ;;
  -?* | --?*)
    #    echo $0: $1: unrecognized option >&2 # Note: '>&2' = print to stderr
    USER_ARG="${USER_ARG} ${arg}"
    shift # Remove generic argument from processing
    ;;
  *)
    CONTAINER_NAMES="${arg}"
    break
    ;;
  esac

  shift
done

## todo:on task end >> delete next bloc ↓↓
#echo "
#${0}:
#  USER_ARG >> ${USER_ARG}
#  CONTAINER_NAMES >> ${CONTAINER_NAMES}
#"

#sudo docker exec -it ${USER_ARG} ${CONTAINER_NAMES} bash

REDTAIL_ID=`docker ps -aqf "name=^/${CONTAINER_NAMES}$"`
if [ -z `docker ps -qf "name=^/${CONTAINER_NAMES}$"` ]; then
    xhost +local:${REDTAIL_ID}
    echo "Starting and attaching to ${CONTAINER_NAMES} container..."
    docker start ${REDTAIL_ID}
#    docker attach ${REDTAIL_ID}
#else
#    echo "Found running ${CONTAINER_NAMES} container, attaching bash..."
#    docker exec -it ${REDTAIL_ID} bash
fi

sudo docker exec -it ${USER_ARG} ${CONTAINER_NAMES} bash
