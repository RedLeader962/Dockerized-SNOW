
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
#ARG BASE_IMAGE=norlabsnow/norlab-mppi-ros-noetic:x86-ubuntu18.04
ARG BASE_IMAGE=norlabsnow/norlab-mppi-ros-melodic-python3:arm64-l4t-r32.6.1
FROM ${BASE_IMAGE} AS ros-python3-base-image

ENV LANG "C.UTF-8"
ENV LC_ALL "C.UTF-8"

#ENV DS_ROS_ROOT=/opt/ros/${ROS_DISTRO}
#ENV ROS_PYTHON_VERSION=3
#RUN echo "export ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION}" >> ~/.bashrc

WORKDIR "/workspace"

## (NICE TO HAVE) todo:investigate?? ( NLSAR-226 ) >> ln: failed to create symbolic link '/etc/localtime': File exists
## Setup timezone
##   Source: OSRF docker_images
##       https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
#RUN echo 'Etc/UTC' > /etc/timezone && \
#    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#    apt-get update && \
#    apt-get install -q -y --no-install-recommends tzdata && \
#    rm -rf /var/lib/apt/lists/*


# ===OpenCV install ====================================================================================================
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
FROM ros-python3-base-image AS ros-pytorch-base-image

ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

RUN apt-get update \
    && apt-get install -y software-properties-common \
    && ldconfig \
    && apt-get install -y --no-install-recommends \
            libopenblas-base \
            libopenmpi-dev \
            python3-pip \
            python3-dev \
            openmpi-bin \
            openmpi-common \
            gfortran \
    && rm -rf /var/lib/apt/lists/*

# Note: libopenblas-base, libopenmpi-dev, python3-pip, Cython, numpy are all requirement for PyTorch
#       build for JetPack (L4T). https://elinux.org/Jetson_Zoo#PyTorch_.28Caffe2.29

RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose setuptools
RUN pip3 install --no-cache-dir --verbose wheel
RUN pip3 install --no-cache-dir --verbose Cython
RUN pip3 install --no-cache-dir --verbose numpy


# PyTorch (for JetPack)
#  https://elinux.org/Jetson_Zoo#PyTorch_.28Caffe2.29
#
#  PyTorch v1.2.0 https://nvidia.box.com/shared/static/lufbgr3xu2uha40cs9ryq1zn4kxsnogl.whl (torch-1.2.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.3.0 https://nvidia.box.com/shared/static/017sci9z4a0xhtwrb4ps52frdfti9iw0.whl (torch-1.3.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.4.0 https://nvidia.box.com/shared/static/c3d7vm4gcs9m728j6o5vjay2jdedqb55.whl (torch-1.4.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.5.0 https://nvidia.box.com/shared/static/3ibazbiwtkl181n95n9em3wtrca7tdzp.whl (torch-1.5.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.7.0 https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl (torch-1.7.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.9.0 https://nvidia.box.com/shared/static/h1z9sw4bb1ybi0rm3tu8qdj8hs05ljbm.whl (torch-1.9.0-cp36-cp36m-linux_aarch64.whl)
ARG PYTORCH_L4T_WHL_URL=https://nvidia.box.com/shared/static/h1z9sw4bb1ybi0rm3tu8qdj8hs05ljbm.whl
ARG PYTORCH_L4T_WHL=torch-1.9.0-cp36-cp36m-linux_aarch64.whl

# PyTorch (official)
ARG PYTORCH_OFFICIAL_WHL_URL=https://download.pytorch.org/whl/torch_stable.html

# Pytorch for M1 chips
# PyTorch v1.9.0 https://download.pytorch.org/whl/cpu/torch-1.9.0-cp39-none-macosx_11_0_arm64.whl cpu/torch-1.9.0-cp39-none-macosx_11_0_arm64.whl
# PyTorch v1.9.1 https://download.pytorch.org/whl/cpu/torch-1.9.1-cp39-none-macosx_11_0_arm64.whl cpu/torch-1.9.1-cp39-none-macosx_11_0_arm64.whl
ARG PYTORCH_URL_M1=https://download.pytorch.org/whl/cpu/torch-1.9.0-cp39-none-macosx_11_0_arm64.whl
ARG PYTORCH_WHL_M1=torch-1.9.0-cp39-none-macosx_11_0_arm64.whl

# Pytorch for x86
# https://pytorch.org/get-started/locally/
# cu111 ==> cuda version
ARG PYTORCH_VERSION_X86="1.9.0+cu111"

# (CRITICAL) todo:fixme!! (ref task NLSAR-230)
# Conditional build stage base on architecture version (arm64-l4t and x86)

# Note: wget -O save the file under a diferent name
RUN /bin/bash -c "if [[ ${DS_IMAGE_ARCHITECTURE} == 'arm64-l4t' ]]; then \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_L4T_WHL_URL} -O ${PYTORCH_L4T_WHL} && \
        pip3 install --no-cache-dir --verbose ${PYTORCH_L4T_WHL} --verbose && \
        rm ${PYTORCH_L4T_WHL}; \
    elif [[ ${DS_IMAGE_ARCHITECTURE} == 'arm64-Darwin' ]]; then \
      pip3 install --no-cache-dir --verbose torch==1.9.0 -f ${PYTORCH_OFFICIAL_WHL_URL}; \
    elif [[ ${DS_IMAGE_ARCHITECTURE} == 'x86' ]]; then \
      pip3 install --no-cache-dir --verbose torch==${PYTORCH_VERSION_X86} -f ${PYTORCH_OFFICIAL_WHL_URL}; \
    else \
      echo 'Architecture ${DS_IMAGE_ARCHITECTURE} is not curently suported'; \
      exit 1; \
    fi"

## Install PyTorch for x86 with torchvision
#    pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html \


# . . torchvision 0.4 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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



# ===Install python scientific stack ===================================================================================
FROM ros-pytorch-base-image AS ros-scientific-stack-base-image

# install aditional python package
RUN pip3 install --upgrade pip
#RUN pip3 install --no-cache-dir --verbose setuptools
#RUN pip3 install --no-cache-dir --verbose wheel

RUN pip3 install --no-cache-dir --verbose scipy
RUN pip3 install --no-cache-dir --verbose scikit-learn
RUN pip3 install --no-cache-dir --verbose pandas

# ...Configure Matplotlib to use a non-interactive backends for TeamCity run............................................
# Procedure to fix NLSAR-343 ﹅→ `ModuleNotFoundError` when using pyplot & `couldn't connect to display`
# Solution: install python3-tk and configure matplotlib to use a non-interactive backend in TeamCity
# Ref:
#   - https://matplotlib.org/2.0.2/faq/usage_faq.html#what-is-a-backend
#   - https://matplotlib.org/stable/tutorials/introductory/customizing.html?highlight=matplotlibrc#the-matplotlibrc-file
#
## Option 1: Set the MPLBACKEND environment variable at container level in `matplotlibrc`
#RUN echo "backend : Agg" >> /etc/matplotlibrc
#
## Option 2: Set the MPLBACKEND environment variable manualy in the TeamCity build step
#export MPLBACKEND="Agg"
#python3 -m pytest tests/

# Hack to install matplotlib on arm64 and it's required module tkinter
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
      python3-matplotlib \
      python3-tk \
    && rm -rf /var/lib/apt/lists/*



# ===Install performance optimization package===========================================================================
FROM ros-scientific-stack-base-image AS ros-performance-pkg-base-image

# . . Numba install. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://numba.pydata.org

# LLVM v3.9 <== (Python3.6 + JetPack 4.3/4.4)
# Numba v0.31 <== (Python3.6 + JetPack 4.3/4.4)
#ENV LVL_VERSION=3.9
#ENV LVLLITE_VERSION=0.16.0
ENV LVL_VERSION=4.0
ENV LVLLITE_VERSION=0.20.0

# Inspired by https://gitee.com/Cheng_Loon/jetson-packages-family-good
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        llvm-${LVL_VERSION} \
        llvm-${LVL_VERSION}-dev \
    && export LLVM_CONFIG=/usr/lib/llvm-${LVL_VERSION}/bin/llvm-config \
    && cd ${HOME} \
    && wget https://github.com/numba/llvmlite/archive/v${LVLLITE_VERSION}.zip \
    && unzip v${LVLLITE_VERSION}.zip \
    && cd llvmlite-${LVLLITE_VERSION} \
    && chmod 777 -R /usr/local/lib/python3.6/dist-packages/ \
    && python3 setup.py install

RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
    python3-numba

#RUN pip3 install --no-cache-dir --verbose llvmlite
#RUN pip3 install --no-cache-dir --verbose numba
#RUN pip3 install --no-cache-dir --ignore-installed pybind11

# . . PyCuda install. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
## (Priority) dropping pycuda as it's development seems to lag

## pycuda 2021.1 documentation: https://documen.tician.de/pycuda/index.html
#RUN pip3 install --no-cache-dir --verbose pycuda

###
### pycuda custom install
## (StandBy) todo:fixme!! (ref task NLSAR-225)
###
##ENV PATH="/usr/local/cuda/bin:${PATH}"
##ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
##RUN echo "$PATH" && echo "$LD_LIBRARY_PATH"
##
##RUN pip3 install --no-cache-dir --verbose pycuda six
#
### Ref pycuda doc install instruction: https://wiki.tiker.net/PyCuda/Installation/Linux/#installing-pycuda-on-linux
##RUN tar xfz pycuda-VERSION.tar.gz \
##    && cd pycuda-VERSION \
##    && python3 configure.py --cuda-root=/usr/local/cuda/bin \
##    && su -c "make install"
##
### Test PyCUDA
##RUN echo " \
##    Test PyCUDA \
##    " \
##    && cd pycuda-VERSION/test \
##    && python test_driver.py
#
##RUN apt-get update \
##    && apt-get install --assume-yes --no-install-recommends \
##        build-essential \
##        python3-dev \
##        python3-setuptools \
##        libboost-python-dev \
##        libboost-thread-dev \
##    && rm -rf /var/lib/apt/lists/*


# . . torch2trt install. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#   torch2trt is a PyTorch to TensorRT converter which utilizes the TensorRT Python API
#   https://github.com/NVIDIA-AI-IOT/torch2trt

# (CRITICAL) todo:fixme!! (ref task NLSAR-225)
#RUN git clone https://github.com/NVIDIA-AI-IOT/torch2trt && \
#    cd torch2trt && \
#    python3 setup.py install --plugins

# ===install utilities==================================================================================================
FROM ros-performance-pkg-base-image AS python-dev-tools

RUN pip3 install --no-cache-dir --verbose pyyaml
RUN pip3 install --no-cache-dir --verbose termcolor
RUN pip3 install --no-cache-dir --verbose decorator
RUN pip3 install --no-cache-dir --verbose -U mock
RUN pip3 install --no-cache-dir --verbose pytest
RUN pip3 install --no-cache-dir --verbose pytest-benchmark
RUN pip3 install --no-cache-dir --verbose pytest-mock
RUN pip3 install --no-cache-dir --verbose python-dotenv


# ...Install simulator..................................................................................................
# Dependencies
#   box2d-py dependency: swig
#   Gym headless server rendering require a virtual X server like xvfb
#       pyvirtualdisplay is a python wraper for xvfb
#       gym-notebook-wrapper is wrapper for running gym and recording movie on Jupyter notebook
#           ↳ |  require xvfb and python-opengl
#   gym VideoRecorder dependency: ffmpeg and imageio-ffmpeg
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        swig \
        xvfb \
        python3-opengl \
        ffmpeg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --verbose pyvirtualdisplay
RUN pip3 install --no-cache-dir --verbose imageio-ffmpeg
# Install gym
RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose gym

# Gym depencies
RUN pip3 install --no-cache-dir --verbose pyglet
RUN pip3 install --no-cache-dir --verbose box2d-py

# Jupyter + gym related
RUN pip3 install --no-cache-dir --verbose jupyterlab
RUN pip3 install --no-cache-dir --verbose notebook
RUN pip3 install --no-cache-dir --verbose gym-notebook-wrapper


# ...Sequential decision making related tools...........................................................................
# Model predictive control python toolbox https://www.do-mpc.com/en/latest/index.html
RUN pip3 install --no-cache-dir --verbose do-mpc



# ===Final build step===================================================================================================
FROM python-dev-tools AS final

# ...install development utilities......................................................................................
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        usbutils \
        rsync \
        tar \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

## ...install C++ development utilities.................................................................................
## Note: libboost-all-dev and  libtbb-dev are GTSAM dependencies
#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        doxygen \
#        texinfo \
#        libboost-all-dev\
#        libtbb-dev \
#        gcc \
#        g++ \
#        gdb \
#        clang \
#        cmake \
#        rsync \
#        tar \
#        gdbserver \
#    && rm -rf /var/lib/apt/lists/*


# ❯❯❯ Temporary build step ❯❯❯..........................................................................................
# ★ | Temporary work around for the numpy 1.19.5 `Illegal instruction (core dumped)` error when executed on aarch64
#   - https://github.com/numpy/numpy/issues/18131
#   - see Task NLSAR-237 🩹→ Illegal instruction (core dumped) on the arm64-l4t image when running pytorch

RUN pip3 install --no-cache-dir --verbose numpy==1.19.4
# ..........................................................................................❮❮❮ Temporary build step ❮❮❮


CMD [ "bash Xvfb :0 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &> xvfb.log &" ]
# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///



