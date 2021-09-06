#!/bin/bash

echo "
ROS distro:         ${ROS_DISTRO}
python3 version:    ${DS_PYTHON3_VERSION}
PyTorch version:    $(pip3 show torch | grep Version | sed 's/Version: //g')
PyCuda version:     $(pip3 show pycuda | grep Version | sed 's/Version: //g')
Numpy version:      $(pip3 show numpy | grep Version | sed 's/Version: //g')

ROS package:        ${DS_ROS_PKG}
ROS python version: ${ROS_PYTHON_VERSION}
ROS master uri:     ${ROS_MASTER_URI}
DS image aarch:     ${DS_IMAGE_ARCHITECTURE}
DS src code path:   ${DS_DEV_WORKSPACE}${DS_TARGET_PROJECT_SRC_REPO}
"
