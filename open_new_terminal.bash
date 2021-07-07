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
    ${0} [<optional argument>] <CONTAINER_NAMES> [<COMMAND>]

    <optional argument>:
      -h, --help                Get help

    Open a new interactive terminal with pseudo-TTY

    Note: you can pass any docker build flag as additional argument eg:
      --env=\"VAR=1\"        (to set environment variables)

    Ref. docker exec command:
      - https://docs.docker.com/engine/reference/commandline/exec/
  "
}

# todo:on task end >> delete next bloc ↓↓
echo "
${0}: all arg >> ${@}
"

if [ $# -ne 1 ]; then
  echo "  >> missing argument: $0 <CONTAINER_NAMES>"
  echo "  If your not sure, run in terminal"
  echo "         docker ps -a"
  echo "  and check the STATUS column to see running container"
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

# todo:on task end >> delete next bloc ↓↓
echo "
${0}:
  USER_ARG >> ${USER_ARG}
  CONTAINER_NAMES >> ${CONTAINER_NAMES}
"

sudo docker exec \
  -it \
  ${USER_ARG} \
  ${CONTAINER_NAMES} \
  bash
