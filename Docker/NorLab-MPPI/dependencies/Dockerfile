
# /// NorLab-MPPI/dependencies /////////////////////////////////////////////////////////////////////////////////////////
# This container is the base image for all other imageS: dev, deploy, ...
#
# Usefull ref:
# - NVIDIA Container Runtime on Jetson: https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson
# - dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic


# Base image: nvidia linux 4 tegra (l 4 t) nvidia docker container official image
#   https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
FROM ${BASE_IMAGE}

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG ROS_PKG=desktop-full
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# Setup timezone
#   Source: OSRF docker_images
#       https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

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
        apt-utils \
        tree \
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


# === Install ROS + GAZEBO =============================================================================================
# ... register the ROS package source ..................................................................................
# setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup your keys (from https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.melodic)
# $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# or this alternative from wiki.ROS.org
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# install ROS packages
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-vcstools \
    && rosdep init \
    && rosdep update \
    && rosdep fix-permissions \
    && rm -rf /var/lib/apt/lists/*




# ... Install ROS, create and build catkin workspace ..............................................................................

# ❯❯❯ standard ROS install method ❯❯❯ . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..

## Install ROS
#RUN apt-get update \
#    && cd ${ROS_ROOT} \
#    && apt-get install --assume-yes --no-install-recommends \
#      ros-${ROS_DISTRO}-`echo "${ROS_PKG}" | tr '_' '-'` \
#    && rm -rf /var/lib/apt/lists/*

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .❮❮❮ standard ROS install method ❮❮❮


ENV DEV_WORKSPACE=$HOME/catkin_ws

RUN mkdir -p ${DEV_WORKSPACE}/src
WORKDIR "${DEV_WORKSPACE}"


# This RUN step is from dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic
RUN rosinstall_generator ${ROS_PKG} vision_msgs --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall \
    && vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src \
    && apt-get update \
    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} --skip-keys python3-pykdl -y \
#    && python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /var/lib/apt/lists/*
# Mote: `catkin_make_isolated` treats each package as a separate cmake project, and builds and installs each separately, \
#       in dependency order. This is why it's able to build plain cmake packages.
# ref: https://answers.ros.org/question/320613/catkin_make-vs-catkin_make_isolated-which-is-preferred/


RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make \
    && source ${DEV_WORKSPACE}/devel/setup.bash" \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc
# Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment
# variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/catkin_ws/src:/opt/ros/melodic/share

## ... Install Gazebo ...................................................................................................
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


# ... Clone required repo ..............................................................................................
WORKDIR "${DEV_WORKSPACE}/src"
RUN git clone https://github.com/norlab-ulaval/icp_odom_with_twist.git

WORKDIR "${DEV_WORKSPACE}"

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////////// NorLab-MPPI/dependencies ///



