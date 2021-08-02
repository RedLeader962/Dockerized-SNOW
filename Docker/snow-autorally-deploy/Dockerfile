
# /// snow-autorally-deploy ////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t
FROM norlabsnow/ros-melodic-snow-autorally-dependencies:${BASE_IMG_TAG}


# === Clone the SNOW-AutoRally fork of AutoRally repo and install ======================================================
#
# Steps:
#   1. clone AutoRally repos
#   2. Install AutoRally ROS Dependencies
#   3. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore (apparently).
#       See pullrequest 243548 merge into `ros:master` on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

ARG SRC_CODE_REPOSITORY_NAME=SNOW_AutoRally
ENV SRC_CODE_REPOSITORY_NAME=${SRC_CODE_REPOSITORY_NAME}
ARG SRC_CODE_DOMAIN_NAME=RedLeader962
ENV SRC_CODE_DOMAIN_NAME=${SRC_CODE_DOMAIN_NAME}
ARG DEV_BRANCH=SNOW-melodic-devel

ARG HOST_TYPE=Local # or XavierWarthog
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

# Environment setup
#RUN echo ". ${DEV_WORKSPACE}/src/${SRC_CODE_REPOSITORY_NAME}/autorally_util/setupEnvLocal.sh" >> ~/.bashrc
RUN echo ". ${DEV_WORKSPACE}/src/${SRC_CODE_REPOSITORY_NAME}/autorally_util/setupEnv${HOST_TYPE}.sh" >> ~/.bashrc


## ... Generate Documentation ..........................................................................................
## Run doxygen with the `-u` flag to remove obsolete configuration tag
## TODO: unmute
#RUN cd ${DEV_WORKSPACE}/src/${SRC_CODE_REPOSITORY_NAME}/ \
#    && doxygen -u


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

# //////////////////////////////////////////////////////////////////////////////////////////// snow-autorally-deploy ///



