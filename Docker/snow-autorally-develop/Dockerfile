
# /// snow-autorally-develop ///////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t
FROM norlabsnow/ros-melodic-snow-autorally-dependencies:${BASE_IMG_TAG}

ARG SRC_CODE_REPOSITORY_NAME=SNOW-AutoRally
# Note: `SRC_CODE_REPOSITORY_NAME` is required by the build_snow_autorally.bash script
ENV SRC_CODE_REPOSITORY_NAME=${SRC_CODE_REPOSITORY_NAME}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY build_snow_autorally.bash /
COPY ros_entrypoint.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash" \
    && /bin/bash -c "chmod +x /build_snow_autorally.bash"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]

# /////////////////////////////////////////////////////////////////////////////////////////// snow-autorally-develop ///



