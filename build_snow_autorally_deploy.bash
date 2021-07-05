#!/bin/bash

sudo docker build \
  -t norlabsnow/snow-autorally-deploy \
  -f ./Docker/snow-autorally-deploy/Dockerfile \
  ./Docker/snow-autorally-deploy