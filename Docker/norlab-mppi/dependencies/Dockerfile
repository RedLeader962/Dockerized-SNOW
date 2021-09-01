
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
FROM ${BASE_IMAGE} AS ros-base-imag

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

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
FROM ros-base-imag AS ros-pytorch-base-image

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

## Install PyTorch for x86 with torchvision
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
FROM ros-pytorch-base-image AS ros-scientific-stack-base-image

# install aditional python package
RUN pip3 install --upgrade pip
#RUN pip3 install --no-cache-dir --verbose setuptools
#RUN pip3 install --no-cache-dir --verbose wheel

RUN pip3 install --no-cache-dir --verbose scipy
RUN pip3 install --no-cache-dir --verbose scikit-learn
RUN pip3 install --no-cache-dir --verbose pandas

# Hack to install matplotlib on arm64
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
      python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*



# ===Install performance optimization package===========================================================================
FROM ros-scientific-stack-base-image AS ros-performance-pkg-base-image

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
FROM ros-performance-pkg-base-image AS final

RUN pip3 install --no-cache-dir --verbose pyyaml
RUN pip3 install --no-cache-dir --verbose jupyter
RUN pip3 install --no-cache-dir --verbose jetson-stats
RUN pip3 install --no-cache-dir --verbose termcolor
RUN pip3 install --no-cache-dir --verbose decorator
RUN pip3 install --no-cache-dir --verbose -U mock
RUN pip3 install --no-cache-dir --verbose pytest
RUN pip3 install --no-cache-dir --verbose pytest-benchmark
RUN pip3 install --no-cache-dir --verbose pytest-mock

# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        usbutils \
        openssh-server \
        rsync \
        tar \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

## install C++ development utilities
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


# ...Configure prompt...................................................................................................
# https://powerline.readthedocs.io/en/latest/index.html
RUN sudo pip3 install powerline-status
RUN apt-get update \
    && apt-get install -y --no-install-recommends fonts-powerline \
    && rm -rf /var/lib/apt/lists/*

RUN ( \
    echo "if [ -f `which powerline-daemon` ]; then"; \
    echo "  powerline-daemon -q"; \
    echo "  POWERLINE_BASH_CONTINUATION=1"; \
    echo "  POWERLINE_BASH_SELECT=1"; \
    echo "  . $(pip show powerline-status | grep Location: | sed 's/Location: //g')/powerline/bindings/bash/powerline.sh"; \
    echo "fi"; \
  ) >> ~/.bashrc

#ARG POWERLINE_DIR="$(pip3 show powerline-status | grep Location: | sed 's/Location: //g')/powerline"
ENV POWERLINE_DIR=/usr/local/lib/${DS_PYTHON3_VERSION}/dist-packages/powerline
WORKDIR $POWERLINE_DIR
#RUN echo $POWERLINE_DIR

# (Priority) todo:on task end >> delete next line
RUN pwd

COPY ./prompt/config_files/config.json config_files/
COPY ./prompt/config_files/themes/shell/dockerized_snow.json config_files/themes/shell/dockerized_snow.json
COPY ./prompt/config_files/colorschemes/shell/dockerized_snow.json config_files/colorschemes/shell/dockerized_snow.json

# (Priority) todo:on task end >> delete next line
RUN /bin/bash -c "tree -L 4"


WORKDIR ${DS_DEV_WORKSPACE}
# ...Setup ssh server...................................................................................................
# ssh port, remaped from default 22 to 2222
ARG DS_PYCHARM_DEV_SERVER_PORT=2222
ENV DS_PYCHARM_DEV_SERVER_PORT=${DS_PYCHARM_DEV_SERVER_PORT}
EXPOSE ${DS_PYCHARM_DEV_SERVER_PORT}

# Inspired from https://austinmorlan.com/posts/docker_clion_development/
RUN ( \
    echo "LogLevel DEBUG2"; \
    echo "PermitRootLogin yes"; \
    echo "PasswordAuthentication yes"; \
    echo "Port ${DS_PYCHARM_DEV_SERVER_PORT}"; \
    echo "Subsystem sftp /usr/lib/openssh/sftp-server"; \
  ) > /etc/ssh/sshd_config_dockerized_snow_openssh_server \
  && mkdir /run/sshd

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

## gdbserver port
#EXPOSE 7777

# ...Add new user.......................................................................................................
ARG NEW_USER=pycharm-debugger
ENV DS_PYCHARM_DEV_USER=${NEW_USER}
ARG PASSWORD=lasagne
RUN useradd -m ${NEW_USER} \
  && yes ${PASSWORD} | passwd ${NEW_USER}

# ...root config........................................................................................................
# user:newpassword
RUN echo "root:${PASSWORD}" | chpasswd

# (Optional) Change default shell for new user
#RUN usermod -s /bin/bash ${NEW_USER}


WORKDIR ${DS_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source ${DS_DEV_WORKSPACE}/devel/setup.bash \
    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_"


# Temporary work around for the numpy 1.19.5 `Illegal instruction (core dumped)` error when executed on arm64
#   - https://github.com/numpy/numpy/issues/18131
#   - see Task NLSAR-237 🩹→ Illegal instruction (core dumped) on the arm64-l4t image when running pytorch
RUN pip3 install --no-cache-dir --verbose numpy==1.19.4


# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
# Check the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share

CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///



