#!/bin/bash

sudo docker build \
  -t norlabsnow/snow-autorally-develop:x86 \
  -f ./Docker/snow-autorally-develop/Dockerfile \
  --build-arg CODE_VERSION=x86 \
  ./Docker/snow-autorally-develop