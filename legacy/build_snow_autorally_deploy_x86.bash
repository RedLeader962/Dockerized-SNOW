#!/bin/bash

sudo docker build \
  -t norlabsnow/snow-autorally-deploy:x86 \
  -f ./Docker/snow-autorally-deploy/Dockerfile \
  --build-arg BASE_IMG_TAG=x86 \
  ./Docker/snow-autorally-deploy