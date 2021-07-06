#!/usr/bin/env bash

# NVIDIA base image with CUDA and OpenGL support
# - https://hub.docker.com/r/nvidia/cudagl/
# - https://github.com/NVIDIA/nvidia-docker/wiki/CUDA
# - https://ngc.nvidia.com/catalog/containers/nvidia:cudagl

sudo docker build \
  -t norlabsnow/ros-melodic-snow-autorally-dependencies:x86 \
  -f ./Docker/ros-melodic-snow-autorally-dependencies/Dockerfile \
  --build-arg BASE_IMAGE=nvcr.io/nvidia/11.3.1-devel-ubuntu18.04 \
  ./Docker