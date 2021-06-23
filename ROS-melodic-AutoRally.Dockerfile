# Adaptation of `Dockerfile.ros.melodic` by dusty-nv
#   at https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.melodic
#

# /// NVIDIA-docker SNOW-AutoRally /////////////////////////////////////////////////////////////////////////////////////

# This dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.5.0
FROM ${BASE_IMAGE}

#ARG ROS_PKG=ros_base
ARG ROS_PKG=desktop-full
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# install development utilities
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
        libusb-dev \
        texinfo \
        libboost-all-dev\
        apt-utils \
        usbutils \
    && rm -rf /var/lib/apt/lists/*


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

#RUN apt-add-repository ppa:dartsim/ppa \
#    && apt-get update \
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends libdart6-all-dev \
    && rm -rf /var/lib/apt/lists/*

# update and initialize rosdep
RUN apt-get update \
    && cd ${ROS_ROOT} \
    && rosdep init \
    && rosdep update \
    && rosdep fix-permissions \
    && rm -rf /var/lib/apt/lists/*


# ... Install MPPI Dependencies ........................................................................................
# Note: cmake will install in the default directory `/usr/local`
RUN cd /opt \
    && git clone https://github.com/rogersce/cnpy.git \
    && mkdir $HOME/build \
    && cd $HOME/build \
    && cmake /opt/cnpy \
    && make \
    && make install


# ... Install GTSAM ....................................................................................................
# CMAKE_BUILD_TYPE: https://github.com/borglab/gtsam/blob/develop/INSTALL.md
# Debug: default
# Release: Optimizations turned on, no debug symbols
#   $ cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF -DCMAKE_BUILD_TYPE=Release ..
# runs unit tests (optional step ... it's very long)
#   $ make check
RUN cd /opt \
    && git clone https://github.com/borglab/gtsam.git \
    && cd /opt/gtsam \
    && mkdir build \
    && cd build \
#    && cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF .. \
    && cmake -DGTSAM_INSTALL_GEOGRAPHICLIB=ON -DGTSAM_WITH_EIGEN_MKL=OFF -DCMAKE_BUILD_TYPE=Release .. \
    && make install \
    && sudo ldconfig

# ... Fix missing import ...............................................................................................
# TODO:refactor out to the first `apt-get`
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        usbutils \
#    && rm -rf /var/lib/apt/lists/*

# ... Create and build a catkin workspace ..............................................................................
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && mkdir -p ~/catkin_ws/src \
    && cd ~/catkin_ws/ \
    && catkin_make \
    && source ~/catkin_ws/devel/setup.bash" \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
# Make sure your workspace is properly overlayed by the setup script by making sure the ROS_PACKAGE_PATH environment
# variable includes the directory you're in.
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/catkin_ws/src:/opt/ros/melodic/share


# ... Fork AutoRally repo and install ..................................................................................
# Steps:
#   1. Upgrade Eigen to version >=3.3.5.  Check package version: $ pkg-config --modversion eigen3
#   2. clone AutoRally repos
#   3. Install AutoRally ROS Dependencies
#   4. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore. Check pullrequest 243548 merge into `ros:master`
#       on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

# Install eigen
RUN cd /opt \
    && git clone https://gitlab.com/libeigen/eigen.git \
    && cd /opt/eigen \
    && mkdir build \
    && cd build \
    && cmake /opt/eigen \
    && make install

RUN echo "Pull latest SNOW-melodic-devel branch"

# Clone AutoRally and dependencies
RUN cd ~/catkin_ws/src \
    && git clone --branch SNOW-melodic-devel https://github.com/RedLeader962/autorally.git  \
    && git clone https://github.com/AutoRally/imu_3dm_gx4.git \
    && git clone https://github.com/ros-drivers/pointgrey_camera_driver.git \
    && cd ~/catkin_ws \
    && apt-get update \
    && rosdep install --from-path src --ignore-src --default-yes \
    && rm -rf /var/lib/apt/lists/*

# Build AutoRally
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
        && cd ~/catkin_ws/ \
        && catkin_make \
        && source ~/catkin_ws/devel/setup.bash"


## /=== IN PROGRESS =====================================================================================================
#
## ... Environment setup ................................................................................................
## Due to the additional requirement of ROS's distributed launch system, you must run
RUN echo "source ~/catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh" >> ~/.bashrc

## TODO: To remove but KEEP COMMENT!
#RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc \
#    && echo 'source ~/catkin_ws/devel/setup.sh' >> ~/.bashrc \
#    && echo 'source ~/catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh' >> ~/.bashrc

## before using any AutoRally components. See https://github.com/AutoRally/autorally/wiki for more information
## about how to set this system up for distributed launches on your vehicle platform.
#
## TODO: To remove
#RUN /bin/bash -c 'source ~/.bashrc'

# ... Generate Documentation ...........................................................................................
# Run doxygen with the `-u` flag to remove obsolete configuration tag
# TODO: unmute
#RUN cd ~/catkin_ws/src/autorally/ \
#    && doxygen -u

## ===================================================================================================== IN PROGRESS ===/



# ... Finish container setup ...........................................................................................
COPY ./dockerfile_util/ros_entrypoint.sh /ros_entrypoint.sh
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.sh"
#RUN /bin/bash -c "chmod a+rw /dev/input/js0"  # Quick fix: pushed to ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS

# ///////////////////////////////////////////////////////////////////////////////////// NVIDIA-docker SNOW-AutoRally ///



