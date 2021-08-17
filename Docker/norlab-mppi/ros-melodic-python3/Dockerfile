
# /// norlab-mppi/dependencies /////////////////////////////////////////////////////////////////////////////////////////
# This container is the base image for all other norlab-mppi images: dev, deploy, ...
#
# References:
# - NVIDIA Container Runtime on Jetson: https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson
# - dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic
# - ROS noetic installation from source: http://wiki.ros.org/noetic/Installation/Source


# Base image: nvidia linux 4 tegra (L4T) nvidia docker container official image
#   l4t-base: https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#   l4t-ros-noetic-pytorch: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
FROM ${BASE_IMAGE} AS ros-base-image
MAINTAINER luc.coupal.1@ulaval.ca

#ARG DS_ROS_PKG=desktop_full
ARG DS_ROS_PKG=ros_base
ENV DS_ROS_PKG=${DS_ROS_PKG}
ENV ROS_DISTRO=melodic
ENV DS_ROS_ROOT=/opt/ros/${ROS_DISTRO}
#ENV ROS_PYTHON_VERSION=3

ARG DS_IMAGE_ARCHITECTURE=arm64-l4t
ENV DS_IMAGE_ARCHITECTURE=${DS_IMAGE_ARCHITECTURE}

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive
ENV DS_IMG_LSB_RELEASE=bionic
ENV DS_DEV_WORKSPACE=$HOME/ros_catkin_ws

# Python 3.6 is a requirement for PyTorch compiled for JetPack (L4T)
#   https://elinux.org/Jetson_Zoo#PyTorch_.28Caffe2.29
ENV DS_PYTHON3_VERSION=python3.6

WORKDIR /workspace


# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        cmake \
        build-essential \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        ca-certificates \
        git \
        libusb-dev \
        usbutils \
        vim \
        tree \
        apt-utils \
    && rm -rf /var/lib/apt/lists/*



## install C++ development utilities
## Note: libboost-all-dev and  libtbb-dev are GTSAM dependencies
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        openssh-server \
#        doxygen \
#        texinfo \
#        libboost-all-dev\
#        libtbb-dev \
#    && rm -rf /var/lib/apt/lists/*


# === Install ROS ======================================================================================================
# ... register the ROS package source ..................................................................................
# setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


## Ref: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ... Install ROS, create and build catkin workspace ...................................................................

# Credit for the next two RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        ros-${ROS_DISTRO}-`echo "${DS_ROS_PKG}" | tr '_' '-'` \
        python-rosdep \
        python-rosinstall-generator \
        python-vcstool \
        python-wstool \
        python-rosinstall \
        build-essential \
    && rosdep init \
    && rosdep update \
    && rosdep fix-permissions \
    && rm -rf /var/lib/apt/lists/*

#        python3-rosinstall \
#          ros-melodic-joy \
#          python3-wstool \

#
#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash" \
#    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN mkdir -p "${DS_DEV_WORKSPACE}/src"
WORKDIR "${DS_DEV_WORKSPACE}/src"
RUN git clone https://github.com/norlab-ulaval/icp_odom_with_twist.git

WORKDIR "${DS_DEV_WORKSPACE}"
RUN apt-get update \
    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} -y

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make \
    && source ${DS_DEV_WORKSPACE}/devel/setup.bash" \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${DS_DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc


#WORKDIR "${DS_DEV_WORKSPACE}"
## 1. rosinstall_generator + vcs import steps: download the source code for ROS packages into the ./src directory
## 2. rosdep install step: looks at all the packages in the src directory and tries to find and install their dependencies on your platform
## 3. catkin_make_isolated step: build everything in the catkin workspace DS_DEV_WORKSPACE
#RUN rosinstall_generator ${DS_ROS_PKG} --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${DS_ROS_PKG}.rosinstall && \
#    vcs import ./src < ${ROS_DISTRO}-${DS_ROS_PKG}.rosinstall
#
##RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash" \
#RUN apt-get update \
#    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} -y \
#    && python ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_DEV_WORKSPACE}/install_isolated -DCMAKE_BUILD_TYPE=Release \
#    && rm -rf /var/lib/apt/lists/*
#
## Note on catkin_make_isolated:
## - Flag `--install-space <path/to/somewhere/else>` overwrite the default `~/ros_catkin_ws/install_isolated` install dir.
## - It treats each package as a separate cmake project, and builds and installs each separately in dependency order.
##   This is why it's able to build plain cmake packages.
##   ref: https://answers.ros.org/question/320613/catkin_make-vs-catkin_make_isolated-which-is-preferred/
#
#RUN /bin/bash -c "source ${DS_DEV_WORKSPACE}/install_isolated/setup.bash" \
#    && echo "source ${DS_DEV_WORKSPACE}/install_isolated/setup.bash" >> ~/.bashrc


RUN apt-get update \
    && apt-get install -y rviz


WORKDIR "${DS_DEV_WORKSPACE}"


# ===Install Gazebo=====================================================================================================

#WORKDIR "${DS_DEV_WORKSPACE}"
#
## Note:
##   - AutoRally require Gazebo version 9.XX
##   - Latest Gazebo 9.XX require an upgrade of `ignition-math`
#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
## Setup keys
#RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
#
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends gazebo9 \
#    && rm -rf /var/lib/apt/lists/*
#
#RUN apt-get update \
#    && apt-get upgrade --assume-yes --no-install-recommends libignition-math2 \
#    && rm -rf /var/lib/apt/lists/*
#
### TODO: To install DART to use with ROS, ROS must be install from source
##RUN apt-get update \
##    && apt-get install --assume-yes --no-install-recommends libdart-core5-dev \
##    && rm -rf /var/lib/apt/lists/*


# ===Build ROS step DONE================================================================================================
FROM ros-base-image AS ros-python3-base-image

# Inspired from the ROS In Docker Container post by THE INSANE TECHIE May 7, 2020
#  https://dhanoopbhaskar.com/blog/2020-05-07-ros-in-docker-container/

RUN apt-get update && apt-get upgrade -y \
     && apt-get install -y software-properties-common \
     && add-apt-repository ppa:deadsnakes/ppa \
     && apt-get install -y ${DS_PYTHON3_VERSION}

# Deadsnakes python 3.9 on Ubuntu 18.04 LTS https://gist.github.com/plembo/6bc141a150cff0369574ce0b0a92f5e7
RUN apt-get update && apt-get install -y \
    python-pip \
    python3-pip \
    python3-all-dev

RUN apt-get install -y \
    python3-rospkg*


#ARG ROS_DISTRO_ROS_PCK="ros-${ROS_DISTRO}-`echo ${DS_ROS_PKG} | tr '_' '-'`"
ARG ROS_DISTRO_ROS_PCK=ros-${ROS_DISTRO}-ros-base
RUN echo "ROS_DISTRO_ROS_PCK=${ROS_DISTRO_ROS_PCK}"

RUN apt-get update --fix-missing
RUN dpkg --configure -a
RUN apt-get install -f
RUN apt-get install -y \
    ${ROS_DISTRO_ROS_PCK} --fix-missing
RUN apt-get install -y python-catkin-tools

RUN apt-get install --no-install-recommends --no-install-suggests -y \
    build-essential \
    cmake \
    ${DS_PYTHON3_VERSION}-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade pip
RUN python2 -m pip install --upgrade pip
RUN python3 -m pip install --upgrade pip
RUN pip3 install --upgrade pip
RUN pip3 install --upgrade setuptools
RUN ${DS_PYTHON3_VERSION} -m pip install --upgrade pip
RUN ${DS_PYTHON3_VERSION} -m pip install --upgrade setuptools


## ===INstall Python 3 step DONE========================================================================================
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
WORKDIR "${DS_DEV_WORKSPACE}"
CMD [ "bash" ]


# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///



