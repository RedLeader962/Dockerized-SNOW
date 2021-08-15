
# /// norlab-mppi/develop //////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
FROM norlabsnow/norlab-mppi-dependencies:${BASE_IMG_TAG}

ARG DS_TARGET_PROJECT_SRC_REPO=NorLab_MPPI
# Note: `DS_TARGET_PROJECT_SRC_REPO` is required by the rebuild_norlab_mppi.bash script
ENV DS_TARGET_PROJECT_SRC_REPO=${DS_TARGET_PROJECT_SRC_REPO}

# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY rebuild_norlab_mppi.bash /
COPY ros_entrypoint.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash" \
    && /bin/bash -c "chmod +x /rebuild_norlab_mppi.bash"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]

# ////////////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/develop ///



