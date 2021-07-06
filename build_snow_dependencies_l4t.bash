#!/usr/bin/env bash

sudo docker build \
  -t norlabsnow/ros-melodic-snow-autorally-dependencies:arm64-l4t \
  -f ./Docker/ros-melodic-snow-autorally-dependencies/Dockerfile \
  ./Docker