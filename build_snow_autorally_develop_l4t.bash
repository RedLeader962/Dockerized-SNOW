#!/bin/bash

sudo docker build \
  -t norlabsnow/snow-autorally-develop:arm64-l4t \
  -f ./Docker/snow-autorally-develop/Dockerfile \
  ./Docker/snow-autorally-develop