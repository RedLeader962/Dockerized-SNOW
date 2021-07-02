#!/usr/bin/env bash

sudo docker build \
  -t ros-melodic-snow-autorally-dependencies \
  -f ./Docker/ros-melodic-snow-autorally-dependencies/Dockerfile \
  ./Docker