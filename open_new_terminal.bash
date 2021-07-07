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
    ${0} [<optional argument>] <CONTAINER_NAMES>

    <optional argument>:
      -h, --help                Get help
      --cmd=\"<theCommand>\"    Default: bash
                                eg.: --cmd=\"touch /tmp/myNewFile.md \"

    Note: you can pass any docker build flag as additional argument eg:
      -it               (interactive and allocate tty)
      --detach          (to run the command in backgroud)
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
  echo "  @{0} >> missing argument: $0 <CONTAINER_NAMES>"
  echo "  If your not sure, run in terminal"
  echo "         docker ps -a"
  echo "  and check the STATUS column to see running container"
  exit 1
fi

CONTAINER_NAMES=""
COMMAND="bash"
USER_ARG=""

for arg in "$@"; do
  case $arg in
  -h | --help)
    print_help_in_terminal
    exit
    ;;
#  *)
#    CONTAINER_NAMES="${arg}"
#    ;;
  --cmd)
    echo "${0} >> pass argument with the equal sign: --cmd=${2}" >&2 # Note: '>&2' = print to stderr
    echo
    exit
    ;;
  --cmd=?*)
    COMMAND="${arg#*=}" # Remove every character up to the '=' and assign the remainder
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
  COMMAND >> ${COMMAND}
"

sudo docker exec \
  ${USER_ARG} \
  ${CONTAINER_NAMES} \
  ${COMMAND}
