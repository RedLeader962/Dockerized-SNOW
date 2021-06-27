
# /// SNOW-AutoRally-deploy ////////////////////////////////////////////////////////////////////////////////////////////

ARG  CODE_VERSION=latest
FROM ros-melodic-auto-rally-dependencies:${CODE_VERSION}

WORKDIR ${DEV_WORKSPACE}

# === Fork AutoRally repo and install ==================================================================================
#
# Steps:
#   1. clone AutoRally repos
#   2. Install AutoRally ROS Dependencies
#   3. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore (apparently).
#       See pullrequest 243548 merge into `ros:master` on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

# Clone AutoRally and dependencies
ARG DEV_BRANCH=SNOW-melodic-devel
RUN cd ${DEV_WORKSPACE}/src \
    && echo "Pull latest ${DEV_BRANCH} branch" \
    && git clone --branch ${DEV_BRANCH} https://github.com/RedLeader962/autorally.git  \
#    && git clone https://github.com/AutoRally/imu_3dm_gx4.git \
#    && git clone https://github.com/ros-drivers/pointgrey_camera_driver.git \
    && cd ${DEV_WORKSPACE} \
    && apt-get update \
    && rosdep install --from-path src --ignore-src --default-yes \
    && rm -rf /var/lib/apt/lists/*

# Build AutoRally
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
        && cd ${DEV_WORKSPACE}/ \
        && catkin_make"
#        && catkin_make \
#        && source ${DEV_WORKSPACE}/devel/setup.bash"

# Environment setup
RUN echo ". ${DEV_WORKSPACE}/src/autorally/autorally_util/setupEnvLocal.sh" >> ~/.bashrc


## ... Generate Documentation ...........................................................................................
## Run doxygen with the `-u` flag to remove obsolete configuration tag
## TODO: unmute
#RUN cd ${DEV_WORKSPACE}/src/autorally/ \
#    && doxygen -u


# ... Finish container setup ...........................................................................................
COPY /ros_entrypoint.sh /ros_entrypoint.sh
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.sh"
#RUN /bin/bash -c "chmod a+rw /dev/input/js0"  # Quick fix: pushed to ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS

# //////////////////////////////////////////////////////////////////////////////////////////// SNOW-AutoRally-deploy ///



