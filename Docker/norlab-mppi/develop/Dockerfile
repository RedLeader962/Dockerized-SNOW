
# /// norlab-mppi/develop //////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
FROM norlabsnow/norlab-mppi-dependencies:${BASE_IMG_TAG}

ARG SRC_CODE_REPOSITORY_NAME=NorLab_MPPI
# Note: `SRC_CODE_REPOSITORY_NAME` is required by the build_norlab_mppi.bash script
ENV SRC_CODE_REPOSITORY_NAME=${SRC_CODE_REPOSITORY_NAME}

# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY build_norlab_mppi.bash /
COPY ros_entrypoint.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash" \
    && /bin/bash -c "chmod +x /build_norlab_mppi.bash"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]

# ////////////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/develop ///



