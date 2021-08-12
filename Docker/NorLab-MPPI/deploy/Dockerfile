
# /// NorLab-MPPI/deploy ///////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r32.6.1
FROM norlabsnow/NorLab-MPPI-dependencies:${BASE_IMG_TAG}


# === Clone the NorLab_MPPI repo and install ===========================================================================
#
# Steps:
#   1. clone NorLab_MPPI repos
#   2. Install Dependencies
#   3. Compilation & Running

ARG SRC_CODE_REPOSITORY_NAME=NorLab_MPPI
ENV SRC_CODE_REPOSITORY_NAME=${SRC_CODE_REPOSITORY_NAME}
ARG SRC_CODE_DOMAIN_NAME=RedLeader962
ENV SRC_CODE_DOMAIN_NAME=${SRC_CODE_DOMAIN_NAME}

ARG DEV_BRANCH=master

# Default: Local , Option: XavierWarthog
ARG HOST_TYPE=Local
ENV HOST_TYPE=${HOST_TYPE}

WORKDIR ${DEV_WORKSPACE}/src
RUN echo "Pull latest ${DEV_BRANCH} branch" \
    && git clone --branch ${DEV_BRANCH} https://github.com/${SRC_CODE_DOMAIN_NAME}/${SRC_CODE_REPOSITORY_NAME}.git  \
    && cd ${DEV_WORKSPACE} \
    && apt-get update \
    && rosdep install --from-path src --ignore-src --default-yes \
    && rm -rf /var/lib/apt/lists/*


WORKDIR ${DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
        && catkin_make"

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
## Environment setup
#RUN echo ". ${DEV_WORKSPACE}/src/${SRC_CODE_REPOSITORY_NAME}/mppi_util/setupEnv${HOST_TYPE}.sh" >> ~/.bashrc


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

# /////////////////////////////////////////////////////////////////////////////////////////////// NorLab-MPPI/deploy ///



