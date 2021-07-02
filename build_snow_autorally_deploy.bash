#!/bin/bash

sudo docker build \
  -t snow-autorally-deploy \
  -f ./Docker/snow-autorally-deploy/Dockerfile \
  ./Docker