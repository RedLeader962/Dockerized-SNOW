
# /// norlab-mppi/deploy ///////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
FROM norlabsnow/norlab-mppi-dependencies:${BASE_IMG_TAG}


# === Clone the NorLab_MPPI repo and install ===========================================================================
#
# Steps:
#   1. clone NorLab_MPPI repos
#   2. Install Dependencies
#   3. Compilation & Running

ARG DS_TARGET_PROJECT_SRC_REPO=NorLab_MPPI
ENV DS_TARGET_PROJECT_SRC_REPO=${DS_TARGET_PROJECT_SRC_REPO}
ARG DS_TARGET_PROJECT_SRC_DOMAIN=RedLeader962
ENV DS_TARGET_PROJECT_SRC_DOMAIN=${DS_TARGET_PROJECT_SRC_DOMAIN}

ARG DEV_BRANCH=master

# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

# (Priority) todo:implement >> everything ↓↓
RUN echo "${0}:not implemented error" && exit

WORKDIR ${DS_DEV_WORKSPACE}/src
RUN echo "Pull latest ${DEV_BRANCH} branch" \
    && git clone --branch ${DEV_BRANCH} https://github.com/${DS_TARGET_PROJECT_SRC_DOMAIN}/${DS_TARGET_PROJECT_SRC_REPO}.git  \
    && cd ${DS_DEV_WORKSPACE} \
    && apt-get update \
    && rosdep install --from-path src --ignore-src --default-yes \
    && rm -rf /var/lib/apt/lists/*


WORKDIR ${DS_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
        && catkin_make"

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
## Environment setup
#RUN echo ". ${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/mppi_util/setupEnv${DS_HOST_TYPE}.sh" >> ~/.bashrc


# ... Finish container setup ...........................................................................................
WORKDIR /
COPY ros_entrypoint.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.bash"
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD [ "bash" ]

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS

# /////////////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/deploy ///



