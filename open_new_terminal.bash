#!/bin/bash

if [ $# -ne 1 ]; then
  echo "SNOW-AutoRally | missing argument: $0 <container name with tag to execute>"
  echo "If your not sure, run in terminal"
  echo "       docker ps -a"
  echo "and check the STATUS column to see running container"
  exit 1
fi

sudo docker exec -it $1 bash