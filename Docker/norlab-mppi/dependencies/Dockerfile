
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

#ARG BASE_IMAGE=nvidiajetson/l4t-ros-noetic-pytorch:r32.5
#ARG BASE_IMAGE=norlabsnow/norlab-mppi-noetic-pytorch:x86-ubuntu20.04
ARG BASE_IMAGE=norlabsnow/norlab-mppi-noetic-pytorch:arm64-l4t-r32.6.1
FROM ${BASE_IMAGE}

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO=noetic
ENV DS_ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

WORKDIR /workspace

## (NICE TO HAVE) todo:investigate?? ( NLSAR-226 ) >> ln: failed to create symbolic link '/etc/localtime': File exists
## Setup timezone
##   Source: OSRF docker_images
##       https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
#RUN echo 'Etc/UTC' > /etc/timezone && \
#    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#    apt-get update && \
#    apt-get install -q -y --no-install-recommends tzdata && \
#    rm -rf /var/lib/apt/lists/*

# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        ca-certificates \
        git \
        libusb-dev \
        usbutils \
        vim \
        apt-utils \
        tree \
        python3-pip \
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


# install additional ROS packages
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
      python3-rosinstall \
      python3-wstool \
      build-essential && \
    rm -rf /var/lib/apt/lists/*


# Hack to install matplotlib on arm64
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
      python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# install aditional python package
RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir --verbose setuptools
RUN pip3 install --no-cache-dir --verbose wheel
RUN pip3 install --no-cache-dir --verbose pytest
RUN pip3 install --no-cache-dir --verbose jetson-stats

RUN pip3 install --no-cache-dir --verbose scipy
RUN pip3 install --no-cache-dir --verbose scikit-learn
RUN pip3 install --no-cache-dir --verbose pandas
RUN pip3 install --no-cache-dir --verbose pycuda
#RUN pip3 install --no-cache-dir --verbose numba
#RUN pip3 install --no-cache-dir --ignore-installed pybind11


ENV DS_DEV_WORKSPACE=$HOME/ros_catkin_ws

# ... Clone required repo ..............................................................................................
WORKDIR "${DS_DEV_WORKSPACE}/src"
RUN git clone https://github.com/norlab-ulaval/icp_odom_with_twist.git

# ---Build the ROS source-----------------------------------------------------------------------------------------------
# Procedure ref: ROS noetic installation from source http://wiki.ros.org/noetic/Installation/Source

# ... register the ROS package source ..................................................................................
# (CRITICAL) todo:assess >> next bloc ↓↓
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

### (CRITICAL) todo:assess >> next bloc ↓↓
## setup sources.list
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
## Ref: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

## install additional ROS packages
#RUN apt-get update \
#    && apt-get install --assume-yes \
#      geometry2 \
#        ros-noetic-tf2 \
#    && rosdep update \
#    && rosdep fix-permissions \
#    && rm -rf /var/lib/apt/lists/*

#    && apt-get install --assume-yes --no-install-recommends \
#      ros-noetic-tf2-tools \

# (Priority) todo:fixme!! (ref task NLSAR-232 🩹→ (arm64-l4t version) E: Unable to locate package ros-noetic-tf2-tools)
WORKDIR "${DS_DEV_WORKSPACE}"

##RUN rosinstall_generator tf2 --rosdistro ${ROS_DISTRO} --deps | vcs import ./src \
#RUN rosinstall_generator ${ROS_PKG} --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall \
#    && vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src \
#    && apt-get update \
#    && rosdep fix-permissions \
#    && rosdep update  \
#    && rosdep --reinstall install geometry2 --rosdistro ${ROS_DISTRO} -y \
#    && python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_ROS_ROOT} -DCMAKE_BUILD_TYPE=Release \
#    && rm -rf /var/lib/apt/lists/*

#ros-noetic-tf2
RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash" \
    && rosinstall_generator tf2 --rosdistro ${ROS_DISTRO} --deps --exclude RPP --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall \
    && vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src \
    && apt-get update \
    && rosdep fix-permissions \
    && rosdep update  \
    && rosdep --reinstall install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} -y \
    && python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_ROS_ROOT} -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /var/lib/apt/lists/*

#WORKDIR "${DS_DEV_WORKSPACE}"
#RUN apt-get update \
#    && rosdep fix-permissions \
#    && rosdep update \
#    && rosdep fix-permissions
#
#RUN python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_ROS_ROOT} -DCMAKE_BUILD_TYPE=Release \
#    &&  rm -rf /var/lib/apt/lists/*


# Note on catkin_make_isolated:
# - Flag `--install-space <path/to/somewhere/else>` overwrite the default `~/ros_catkin_ws/install_isolated` install dir.
# - It treats each package as a separate cmake project, and builds and installs each separately in dependency order.
#   This is why it's able to build plain cmake packages.
#   ref: https://answers.ros.org/question/320613/catkin_make-vs-catkin_make_isolated-which-is-preferred/

WORKDIR "${DS_DEV_WORKSPACE}"

RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash" \
    && echo "source ${DS_ROS_ROOT}/setup.bash" >> ~/.bashrc



WORKDIR "${DS_DEV_WORKSPACE}"

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
# Check the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share

CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///



