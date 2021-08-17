
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
#ARG BASE_IMAGE=norlabsnow/norlab-mppi-ros-melodic-python3:x86-ubuntu20.04
ARG BASE_IMAGE=norlabsnow/norlab-mppi-ros-melodic-python3:arm64-l4t-r32.6.1
FROM ${BASE_IMAGE} AS noetic-base-imag

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

#ENV ROS_DISTRO=noetic
ENV DS_ROS_ROOT=/opt/ros/${ROS_DISTRO}
#ENV ROS_PYTHON_VERSION=3

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


# ===Install OpenCV/PyTorch/PyCUDA======================================================================================
# Credit for the next RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

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


# ===PyTorch============================================================================================================
FROM noetic-base-imag AS noetic-pytorch-base-image

#
# install PyTorch prerequisites (many of these are for numpy)
#
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"


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

RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose setuptools
RUN pip3 install --no-cache-dir --verbose wheel
RUN pip3 install --no-cache-dir --verbose Cython
#RUN pip3 install --no-cache-dir --verbose numpy




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

RUN /bin/bash -c "if [[ ${DS_IMAGE_ARCHITECTURE} == 'arm64-l4t' ]]; then \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
        pip3 install --no-cache-dir --verbose ${PYTORCH_WHL} --verbose && \
        rm ${PYTORCH_WHL}; \
  elif [[ ${DS_IMAGE_ARCHITECTURE} == 'x86' ]]; then \
    pip3 install --no-cache-dir --verbose torch==1.9.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html; \
  else \
    echo 'Architecture ${DS_IMAGE_ARCHITECTURE} is not curently suported'; \
    exit; \
  fi"

# Install PyTorch for x86 with torchvision
#    pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html \




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



# ===Install noetic scientific stack step ==============================================================================
FROM noetic-pytorch-base-image AS noetic-scientific-stack-base-image

# install aditional python package
RUN pip3 install --upgrade pip
#RUN pip3 install --no-cache-dir --verbose setuptools
#RUN pip3 install --no-cache-dir --verbose wheel

RUN pip3 install --no-cache-dir --verbose numpy
RUN pip3 install --no-cache-dir --verbose scipy
RUN pip3 install --no-cache-dir --verbose scikit-learn
RUN pip3 install --no-cache-dir --verbose pandas

# Hack to install matplotlib on arm64
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
      python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*



# ===Install performance optimization package===========================================================================
FROM noetic-scientific-stack-base-image AS noetic-performance-pkg-base-image

RUN pip3 install --no-cache-dir --verbose pycuda
#RUN pip3 install --no-cache-dir --verbose numba
#RUN pip3 install --no-cache-dir --ignore-installed pybind11


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

# ===Final build step — install utilities===============================================================================
FROM noetic-performance-pkg-base-image AS final

RUN pip3 install --no-cache-dir --verbose pytest
RUN pip3 install --no-cache-dir --verbose jetson-stats

# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        usbutils \
        openssh-server \
    && rm -rf /var/lib/apt/lists/*

## install C++ development utilities
## Note: libboost-all-dev and  libtbb-dev are GTSAM dependencies
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        doxygen \
#        texinfo \
#        libboost-all-dev\
#        libtbb-dev \
#    && rm -rf /var/lib/apt/lists/*





#ENV DS_DEV_WORKSPACE=$HOME/ros_catkin_ws
#
## ... Clone required repo ..............................................................................................
#WORKDIR ${DS_DEV_WORKSPACE}/src
#RUN git clone https://github.com/norlab-ulaval/icp_odom_with_twist.git \
#    && cd ${DS_DEV_WORKSPACE} \
#    && apt-get update \
#    && rosdep install --from-path src --ignore-src --default-yes \
#    && rm -rf /var/lib/apt/lists/*
#
#
#WORKDIR ${DS_DEV_WORKSPACE}
#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
#        && catkin_make \
#        && source ${DS_DEV_WORKSPACE}/devel/setup.bash"
#
#
## ... Clone required repo ..............................................................................................
#WORKDIR ${DS_DEV_WORKSPACE}/src
#RUN /bin/bash -c "source ${DS_DEV_WORKSPACE}/install_isolated/setup.bash" \
#    && git clone https://github.com/norlab-ulaval/icp_odom_with_twist.git
#
##    && cd ${DS_DEV_WORKSPACE} \
##    && apt-get update \
##    && rosdep install --from-path ./src --ignore-src --default-yes  --rosdistro=${ROS_DISTRO} \
##    && rm -rf /var/lib/apt/lists/*
#
#WORKDIR ${DS_DEV_WORKSPACE}
#RUN /bin/bash -c "source ${DS_DEV_WORKSPACE}/install_isolated/setup.bash" \
#    && python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_DEV_WORKSPACE}/install_isolated -DCMAKE_BUILD_TYPE=Release \
#    && rm -rf /var/lib/apt/lists/*


WORKDIR ${DS_DEV_WORKSPACE}
RUN /bin/bash -c "source ${DS_DEV_WORKSPACE}/install_isolated/setup.bash \
    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_"



# ---Build the ROS source-----------------------------------------------------------------------------------------------
# Procedure ref: ROS noetic installation from source http://wiki.ros.org/noetic/Installation/Source

# ... register the ROS package source ..................................................................................
# (CRITICAL) todo:assess >> next bloc ↓↓
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

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

#RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash \
#    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_"

##ros-noetic-tf2
#RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash" \
#    && rosinstall_generator tf2 --rosdistro ${ROS_DISTRO} --deps --exclude RPP --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall \
#    && vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src \
#    && apt-get update \
#    && rosdep fix-permissions \
#    && rosdep update  \
#    && rosdep --reinstall install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} -y \
#    && python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_ROS_ROOT} -DCMAKE_BUILD_TYPE=Release \
#    && rm -rf /var/lib/apt/lists/*

#WORKDIR "${DS_DEV_WORKSPACE}"
#RUN apt-get update \
#    && rosdep install --from-path src --ignore-src --default-yes \
#    && rosdep fix-permissions \
#    && rosdep update \
#    && rosdep fix-permissions
#
#RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash" \
#    && python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${DS_ROS_ROOT} -DCMAKE_BUILD_TYPE=Release \
#    &&  rm -rf /var/lib/apt/lists/*


# Note on catkin_make_isolated:
# - Flag `--install-space <path/to/somewhere/else>` overwrite the default `~/ros_catkin_ws/install_isolated` install dir.
# - It treats each package as a separate cmake project, and builds and installs each separately in dependency order.
#   This is why it's able to build plain cmake packages.
#   ref: https://answers.ros.org/question/320613/catkin_make-vs-catkin_make_isolated-which-is-preferred/



WORKDIR "${DS_DEV_WORKSPACE}"

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
# Check the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share

CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///



