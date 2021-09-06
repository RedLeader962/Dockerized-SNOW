#!/bin/bash

CONTAINER_NAMES="IamSnow-NX"

# Stop container if he is stopped
if [ -z `docker ps -qf "name=^/${CONTAINER_NAMES}$"` ]; then
    echo "Stoping container $(docker stop ${CONTAINER_NAMES})"
fi

