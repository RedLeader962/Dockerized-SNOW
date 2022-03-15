
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
COPY fetch_ros_env.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash" \
    && /bin/bash -c "chmod +x /rebuild_norlab_mppi.bash" \
    && /bin/bash -c "chmod +x /fetch_ros_env.bash"


RUN ( \
  echo ""; \
  echo "# Dockerized-SNOW aliases (from develop img)"; \
  echo "alias ds_rebuild_norlab_mppi='bash /rebuild_norlab_mppi.bash'"; \
  echo "alias ds_fetch_ros_env='bash /fetch_ros_env.bash'"; \
  echo "alias ds_python3_check='python3 /ros_catkin_ws/src/${DS_TARGET_PROJECT_SRC_REPO}/src/container_related/try_pytorch.py'"; \
  echo ""; \
) >> ~/.bashrc

WORKDIR /ros_catkin_ws/src/${DS_TARGET_PROJECT_SRC_REPO}
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash Xvfb :0 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &> xvfb.log &" ]

# ////////////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/develop ///



