#!/bin/bash

sudo docker build \
  -t norlabsnow/snow-autorally-deploy:arm64-l4t \
  -f ./Docker/snow-autorally-deploy/Dockerfile \
  ./Docker/snow-autorally-deploy