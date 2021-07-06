
# /// snow-autorally-develop ///////////////////////////////////////////////////////////////////////////////////////////

ARG  CODE_VERSION=arm64-l4t
FROM norlabsnow/ros-melodic-snow-autorally-dependencies:${CODE_VERSION}

ARG src_code_repository=SNOW-AutoRally
ENV SRC_CODE_REPOSITORY=${src_code_repository}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY build_snow_autorally.bash /
RUN /bin/bash -c "chmod +x /build_snow_autorally.bash"

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]
# /////////////////////////////////////////////////////////////////////////////////////////// snow-autorally-develop ///



