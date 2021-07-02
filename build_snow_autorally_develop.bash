#!/bin/bash

sudo docker build \
  -t snow-autorally-develop \
  -f .Docker/snow-autorally-develop/Dockerfile \
  ./Docker