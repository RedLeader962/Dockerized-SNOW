
# /// NorLab-MPPI/develop //////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
FROM norlabsnow/NorLab-MPPI-dependencies:${BASE_IMG_TAG}

ARG SRC_CODE_REPOSITORY_NAME=NorLab_MPPI
# Note: `SRC_CODE_REPOSITORY_NAME` is required by the build_norlab_mppi.bash script
ENV SRC_CODE_REPOSITORY_NAME=${SRC_CODE_REPOSITORY_NAME}

# Default: Local , Option: XavierWarthog
ARG HOST_TYPE=Local
ENV HOST_TYPE=${HOST_TYPE}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY build_norlab_mppi.bash /
COPY ros_entrypoint.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash" \
    && /bin/bash -c "chmod +x /build_norlab_mppi.bash"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]

# ////////////////////////////////////////////////////////////////////////////////////////////// NorLab-MPPI/develop ///



