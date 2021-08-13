
# /// gt-autorally/develop ///////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r32.5.0
FROM norlabsnow/gt-autorally-dependencies:${BASE_IMG_TAG}

ARG SRC_CODE_REPOSITORY_NAME=SNOW_AutoRally
# Note: `SRC_CODE_REPOSITORY_NAME` is required by the rebuild_gt_autorally.bash script
ENV SRC_CODE_REPOSITORY_NAME=${SRC_CODE_REPOSITORY_NAME}

# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY rebuild_gt_autorally.bash /
COPY ros_entrypoint.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash" \
    && /bin/bash -c "chmod +x /rebuild_gt_autorally.bash"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]

# /////////////////////////////////////////////////////////////////////////////////////////// gt-autorally/develop ///



