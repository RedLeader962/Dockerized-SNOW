#!/bin/bash

PCK_VERSION=$(pip3 list --format freeze)

echo -e "
\033[1;37mROS distro:         ${ROS_DISTRO}
python3 version:    ${DS_PYTHON3_VERSION}\033[0m
PyTorch version:    $(echo "${PCK_VERSION}" | grep torch | sed 's/torch==//g')
PyCuda version:     $(echo "${PCK_VERSION}" | grep pycuda | sed 's/pycuda==//g')
Numpy version:      $(echo "${PCK_VERSION}" | grep numpy | sed 's/numpy==//g')

ROS package:        ${DS_ROS_PKG}
ROS python version: ${ROS_PYTHON_VERSION}
ROS master uri:     ${ROS_MASTER_URI}
DS image aarch:     ${DS_IMAGE_ARCHITECTURE}
DS src code path:   ${DS_DEV_WORKSPACE}${DS_TARGET_PROJECT_SRC_REPO}
"
