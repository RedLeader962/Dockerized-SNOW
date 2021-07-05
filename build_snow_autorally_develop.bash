#!/bin/bash

sudo docker build \
  -t norlabsnow/snow-autorally-develop \
  -f ./Docker/snow-autorally-develop/Dockerfile \
  ./Docker/snow-autorally-develop