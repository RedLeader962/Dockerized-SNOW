#!/bin/bash

sudo docker build \
  -t norlabsnow/snow-autorally-develop:x86 \
  -f ./Docker/snow-autorally-develop/Dockerfile \
  --build-arg BASE_IMG_TAG=x86 \
  ./Docker/snow-autorally-develop