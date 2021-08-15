
# /// gt-autorally/deploy ////////////////////////////////////////////////////////////////////////////////////////////

ARG  BASE_IMG_TAG=arm64-l4t-r32.5.0
FROM norlabsnow/gt-autorally-dependencies:${BASE_IMG_TAG}


# === Clone the SNOW-AutoRally fork of AutoRally repo and install ======================================================
#
# Steps:
#   1. clone AutoRally repos
#   2. Install AutoRally ROS Dependencies
#   3. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore (apparently).
#       See pullrequest 243548 merge into `ros:master` on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

ARG DS_TARGET_PROJECT_SRC_REPO=SNOW_AutoRally
ENV DS_TARGET_PROJECT_SRC_REPO=${DS_TARGET_PROJECT_SRC_REPO}
ARG DS_TARGET_PROJECT_SRC_DOMAIN=RedLeader962
ENV DS_TARGET_PROJECT_SRC_DOMAIN=${DS_TARGET_PROJECT_SRC_DOMAIN}

# Clone branch at tag AutoRally-Clean (it's after correction but prior to refactoring)
ARG DEV_BRANCH=AutoRally-Clean
#ARG DEV_BRANCH=SNOW-melodic-devel

# Default: Local , Option: XavierWarthog
ARG DS_HOST_TYPE=Local
ENV DS_HOST_TYPE=${DS_HOST_TYPE}

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

# Environment setup
#RUN echo ". ${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/autorally_util/setupEnvLocal.sh" >> ~/.bashrc
RUN echo ". ${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/autorally_util/setupEnv${DS_HOST_TYPE}.sh" >> ~/.bashrc


## ... Generate Documentation ..........................................................................................
## Run doxygen with the `-u` flag to remove obsolete configuration tag
## TODO: unmute
#RUN cd ${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/ \
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

# //////////////////////////////////////////////////////////////////////////////////////////// gt-autorally/deploy ///



