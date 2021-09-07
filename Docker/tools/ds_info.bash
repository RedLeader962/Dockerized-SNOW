#!/bin/bash

PCK_VERSION=$(pip3 list --format freeze)
SP="  "
echo -e "
${DS_CONTAINER_NAME container info}\033[1;37m
${SP}ROS distro:         ${ROS_DISTRO}
${SP}python3 version:    ${DS_PYTHON3_VERSION}\033[0m
${SP}PyTorch version:    $(echo "${PCK_VERSION}" | grep torch | sed 's/torch==//g')
${SP}PyCuda version:     $(echo "${PCK_VERSION}" | grep pycuda | sed 's/pycuda==//g')
${SP}Numpy version:      $(echo "${PCK_VERSION}" | grep numpy | sed 's/numpy==//g')
${SP}ROS package:        ${DS_ROS_PKG}
${SP}ROS python version: ${ROS_PYTHON_VERSION}
${SP}ROS master uri:     ${ROS_MASTER_URI}
${SP}DS image aarch:     ${DS_IMAGE_ARCHITECTURE}
${SP}DS src code path:   ${DS_DEV_WORKSPACE}/${DS_TARGET_PROJECT_SRC_REPO}
"
