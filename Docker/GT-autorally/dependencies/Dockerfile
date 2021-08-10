
# /// GT-autorally/dependencies //////////////////////////////////////////////////////////////////////////
# This container is the base image for all other imageS: dev, deploy, ...
#
# Usefull ref:
# - NVIDIA Container Runtime on Jetson: https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson
# - dusty-nv/jetson-containers: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.melodic


# Base image: nvidia linux 4 tegra (l 4 t) nvidia docker container official image
#   https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.5.0
FROM ${BASE_IMAGE}

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG ROS_PKG=desktop-full
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

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
# Note: libboost-all-dev and  libtbb-dev are GTSAM dependencies
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        git \
        cmake \
        build-essential \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        ca-certificates \
        doxygen \
        openssh-server \
        apt-utils \
        libusb-dev \
        usbutils \
        texinfo \
        libboost-all-dev\
        libtbb-dev \
        vim \
    && rm -rf /var/lib/apt/lists/*


# === Install AutoRally MPPI Dependencies ==============================================================================

# ... Install CNPY .....................................................................................................
# Note: cmake will install in the default directory `/usr/local`
RUN cd /opt \
    && git clone https://github.com/rogersce/cnpy.git \
    && mkdir $HOME/build \
    && cd $HOME/build \
    && cmake /opt/cnpy \
    && make \
    && make install


# ... Install GTSAM ....................................................................................................
# Performance:
#   Source: https://github.com/borglab/gtsam/blob/develop/INSTALL.md
#   - CMAKE_BUILD_TYPE: Debug (default), Release (Optimizations turned on, no debug symbols, run 10X faster)
#       $ cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF -DCMAKE_BUILD_TYPE=Release ..
#   - Enable TBB (it can speed up optimization by 30-50%)
#   - Add -march=native to GTSAM_CMAKE_CXX_FLAGS. A performance gain of 25-30 % can be expected on modern processors
# To runs unit tests (optional step ... it's very long)
#   $ make check
RUN cd /opt \
    && git clone https://github.com/borglab/gtsam.git \
    && cd /opt/gtsam \
    && mkdir build \
    && cd build \
#    && cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF .. \
    && cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF -DCMAKE_BUILD_TYPE=Release .. \
    && make install \
    && ldconfig
#    && sudo ldconfig


# ... Install eigen ....................................................................................................
# Upgrade Eigen to version >=3.3.5.  Check package version: $ pkg-config --modversion eigen3
RUN cd /opt \
    && git clone https://gitlab.com/libeigen/eigen.git \
    && cd /opt/eigen \
    && mkdir build \
    && cd build \
    && cmake /opt/eigen \
    && make install


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
        ros-melodic-`echo "${ROS_PKG}" | tr '_' '-'` \
#        ros-melodic-joy \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        python-vcstools \
    && rm -rf /var/lib/apt/lists/*


# update and initialize rosdep
RUN apt-get update \
    && cd ${ROS_ROOT} \
    && rosdep init \
    && rosdep update \
    && rosdep fix-permissions \
    && rm -rf /var/lib/apt/lists/*


# ... Install Gazebo ...................................................................................................
# Note:
#   - AutoRally require Gazebo version 9.XX
#   - Latest Gazebo 9.XX require an upgrade of `ignition-math`
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# Setup keys
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends gazebo9 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get upgrade --assume-yes --no-install-recommends libignition-math2 \
    && rm -rf /var/lib/apt/lists/*

## TODO: To install DART to use with ROS, ROS must be install from source
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends libdart-core5-dev \
#    && rm -rf /var/lib/apt/lists/*


# ... Create and build a catkin workspace ..............................................................................
ENV DEV_WORKSPACE=$HOME/catkin_ws

#WORKDIR "${DEV_WORKSPACE}/src"
RUN mkdir -p ${DEV_WORKSPACE}/src
WORKDIR "${DEV_WORKSPACE}"

#    && mkdir -p ${DEV_WORKSPACE}/src \
#    && cd ${DEV_WORKSPACE}/ \
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make \
    && source ${DEV_WORKSPACE}/devel/setup.bash" \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc
# Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment
# variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/catkin_ws/src:/opt/ros/melodic/share

# Clone AutoRally required driver
WORKDIR "${DEV_WORKSPACE}/src"
RUN git clone https://github.com/AutoRally/imu_3dm_gx4.git \
    && git clone https://github.com/ros-drivers/pointgrey_camera_driver.git \
    && git clone https://github.com/norlab-ulaval/icp_odom_with_twist.git


WORKDIR "${DEV_WORKSPACE}"

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
CMD [ "bash" ]

# ////////////////////////////////////////////////////////////////////////// GT-autorally/dependencies ///



