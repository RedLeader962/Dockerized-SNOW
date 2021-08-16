
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
FROM ${BASE_IMAGE} AS noetic-base-image

#ARG ROS_PKG=desktop_full
ARG ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV DS_ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ARG DS_IMAGE_ARCHITECTURE=arm64-l4t
ENV DS_IMAGE_ARCHITECTURE=${DS_IMAGE_ARCHITECTURE}

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive

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


# === Install ROS ======================================================================================================
# ... register the ROS package source ..................................................................................
# setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Ref: https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ... Install ROS, create and build catkin workspace ...................................................................
ENV DS_DEV_WORKSPACE=$HOME/ros_catkin_ws

# Credit for the next two RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          libpython3-dev \
          python3-rosdep \
          python3-rosinstall-generator \
          python3-vcstool \
          build-essential && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir ${DS_DEV_WORKSPACE} && \
    cd ${DS_DEV_WORKSPACE} && \
    rosinstall_generator ${ROS_PKG} --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall && \
    mkdir src && \
    vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_ROS_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

#   rosinstall_generator ${ROS_PKG} vision_msgs --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall && \

# Note on catkin_make_isolated:
# - Flag `--install-space <path/to/somewhere/else>` overwrite the default `~/ros_catkin_ws/install_isolated` install dir.
# - It treats each package as a separate cmake project, and builds and installs each separately in dependency order.
#   This is why it's able to build plain cmake packages.
#   ref: https://answers.ros.org/question/320613/catkin_make-vs-catkin_make_isolated-which-is-preferred/

WORKDIR "${DS_DEV_WORKSPACE}"

RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash" \
    && echo "source ${DS_ROS_ROOT}/setup.bash" >> ~/.bashrc

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
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
WORKDIR "${DS_DEV_WORKSPACE}"
CMD [ "bash" ]


# ===Install rviz/OpenCV/PyTorch/PyCUDA=================================================================================
# Credit for the next RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

RUN apt-get update && \
    apt-get install -y vim && \
    apt-get install -y rviz


# (NICE TO HAVE) todo:fixme!! (ref task NLSAR-225)
##
## install OpenCV (with GStreamer support)
##
##COPY jetson-ota-public.asc /etc/apt/trusted.gpg.d/jetson-ota-public.asc
#RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
## Source: https://github.com/dusty-nv/jetson-containers/issues/5#issuecomment-673458718
#
#RUN echo "deb https://repo.download.nvidia.com/jetson/common r32.4 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
#    apt-get update && \
#    apt-get install -y --no-install-recommends \
#            libopencv-python \
#    && rm /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
#    && rm -rf /var/lib/apt/lists/*






# ===Install numpy + PyTorch prerequisite===============================================================================
FROM noetic-base-image AS noetic-numpy-base-image

#
# install PyTorch prerequisites (many of these are for numpy)
#
ENV PATH="/usr/local/cuda-10.2/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-10.2/lib64:/usr/local/cuda-10.2/extras/CUPTI/lib64:${LD_LIBRARY_PATH}"


RUN apt-get update \
    && apt-get install -y software-properties-common \
    && ldconfig \
    && apt-get install -y --no-install-recommends \
            python3-pip \
            python3-dev \
            libopenblas-base \
            libopenmpi-dev \
            openmpi-bin \
            openmpi-common \
            gfortran \
    && rm -rf /var/lib/apt/lists/*

#            libopenmpi2 \

RUN pip3 install setuptools Cython wheel
RUN pip3 install numpy --verbose



# ===PyTorch============================================================================================================

FROM noetic-numpy-base-image AS noetic-pytorch-base-image

# PyTorch (for JetPack)
#  https://elinux.org/Jetson_Zoo#PyTorch_.28Caffe2.29
#
#  PyTorch v1.2.0 https://nvidia.box.com/shared/static/lufbgr3xu2uha40cs9ryq1zn4kxsnogl.whl (torch-1.2.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.3.0 https://nvidia.box.com/shared/static/017sci9z4a0xhtwrb4ps52frdfti9iw0.whl (torch-1.3.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.4.0 https://nvidia.box.com/shared/static/c3d7vm4gcs9m728j6o5vjay2jdedqb55.whl (torch-1.4.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.5.0 https://nvidia.box.com/shared/static/3ibazbiwtkl181n95n9em3wtrca7tdzp.whl (torch-1.5.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.7.0 https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl (torch-1.7.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.9.0 https://nvidia.box.com/shared/static/h1z9sw4bb1ybi0rm3tu8qdj8hs05ljbm.whl (torch-1.9.0-cp36-cp36m-linux_aarch64.whl)
ARG PYTORCH_URL=https://nvidia.box.com/shared/static/h1z9sw4bb1ybi0rm3tu8qdj8hs05ljbm.whl
ARG PYTORCH_WHL=torch-1.9.0-cp36-cp36m-linux_aarch64.whl

# Pytorch for x86
# https://pytorch.org/get-started/locally/

# (CRITICAL) todo:fixme!! (ref task NLSAR-230)
# Conditional build stage base on architecture version (arm64-l4t and x86)
RUN if [[ "${DS_IMAGE_ARCHITECTURE}" == "arm64-l4t" ]]; then \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
        pip3 install ${PYTORCH_WHL} --verbose && \
        rm ${PYTORCH_WHL}; \
  elif [[ "${DS_IMAGE_ARCHITECTURE}" == "x86" ]]; then \
    pip3 install torch==1.9.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html; \
  else \
    echo "Architecture ${DS_IMAGE_ARCHITECTURE} is not curently suported"; \
    exit; \
  fi

# Install PyTorch for x86 with torchvision
#    pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html \


# Choose final stage based on input argument passed to `docker build --build-arg` arg
FROM noetic-pytorch-base-image AS final

## ------------------------
## torchvision 0.4
## ------------------------
#
#ARG TORCHVISION_VERSION=v0.7.0
##ARG PILLOW_VERSION="pillow<7"
#ARG TORCH_CUDA_ARCH_LIST="7.2"
#
#RUN printenv && echo "torchvision version = $TORCHVISION_VERSION" && echo "pillow version = $PILLOW_VERSION" && echo "TORCH_CUDA_ARCH_LIST = $TORCH_CUDA_ARCH_LIST"
#
#RUN apt-get update && \
#    apt-get install -y --no-install-recommends \
#          git \
#          build-essential \
#          libjpeg-dev \
#          zlib1g-dev \
#    && rm -rf /var/lib/apt/lists/*
#
#RUN git clone -b ${TORCHVISION_VERSION} https://github.com/pytorch/vision torchvision && \
#    cd torchvision && \
#    python3 setup.py install && \
#    cd ../ && \
#    rm -rf torchvision



# ===Install PyCUDA=====================================================================================================
#   - pycuda 2021.1 documentation: https://documen.tician.de/pycuda/index.html
# (CRITICAL) todo:fixme!! (ref task NLSAR-225)

##
## PyCUDA
##
#ENV PATH="/usr/local/cuda/bin:${PATH}"
#ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
#RUN echo "$PATH" && echo "$LD_LIBRARY_PATH"
#
#RUN pip3 install --no-cache-dir --verbose pycuda six

## Ref pycuda doc install instruction: https://wiki.tiker.net/PyCuda/Installation/Linux/#installing-pycuda-on-linux
#RUN tar xfz pycuda-VERSION.tar.gz \
#    && cd pycuda-VERSION \
#    && python3 configure.py --cuda-root=/usr/local/cuda/bin \
#    && su -c "make install"
#
## Test PyCUDA
#RUN echo " \
#    Test PyCUDA \
#    " \
#    && cd pycuda-VERSION/test \
#    && python test_driver.py

#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        build-essential \
#        python3-dev \
#        python3-setuptools \
#        libboost-python-dev \
#        libboost-thread-dev \
#    && rm -rf /var/lib/apt/lists/*
#
#RUN pip3 install pycuda --verbose


# -------------------
# torch2trt installations
#
#   torch2trt is a PyTorch to TensorRT converter which utilizes the TensorRT Python API
#   https://github.com/NVIDIA-AI-IOT/torch2trt
# -------------------
# (CRITICAL) todo:fixme!! (ref task NLSAR-225)
#RUN git clone https://github.com/NVIDIA-AI-IOT/torch2trt && \
#    cd torch2trt && \
#    python3 setup.py install --plugins


# ===Install rviz/OpenCV/PyTorch/PyCUDA step DONE=======================================================================
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
WORKDIR "${DS_DEV_WORKSPACE}"
CMD [ "bash" ]


# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///



