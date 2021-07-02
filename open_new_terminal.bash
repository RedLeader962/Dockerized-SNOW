#!/bin/bash

if [ $# -ne 1 ]; then
  echo "SNOW-AutoRally | missing argument: $0 <container name to execute>"
  exit 1
fi

sudo docker exec -it $1 bash