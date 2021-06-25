# Adaptation of `Dockerfile.ros.melodic` by dusty-nv
#   at https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.melodic
#

# /// NVIDIA-docker SNOW-AutoRally /////////////////////////////////////////////////////////////////////////////////////

FROM ROS-melodic-AutoRally-dependencies:latest

#ARG ROS_PKG=ros_base
ARG ROS_PKG=desktop-full
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# === Fork AutoRally repo and install ==================================================================================
#
# Steps:
#   1. Upgrade Eigen to version >=3.3.5.  Check package version: $ pkg-config --modversion eigen3
#   2. clone AutoRally repos
#   3. Install AutoRally ROS Dependencies
#   4. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore (apparently).
#       See pullrequest 243548 merge into `ros:master` on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

# Clone AutoRally and dependencies
RUN cd ~/catkin_ws/src \
    && echo "Pull latest SNOW-melodic-devel branch" \
    && git clone --branch SNOW-melodic-devel https://github.com/RedLeader962/autorally.git  \
    && git clone https://github.com/AutoRally/imu_3dm_gx4.git \
    && git clone https://github.com/ros-drivers/pointgrey_camera_driver.git \
    && cd ~/catkin_ws \
    && apt-get update \
    && rosdep install --from-path src --ignore-src --default-yes \
    && rm -rf /var/lib/apt/lists/*

# Build AutoRally
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
        && cd ~/catkin_ws/ \
        && catkin_make"
#        && catkin_make \
#        && source ~/catkin_ws/devel/setup.bash"

# Environment setup
RUN echo ". ~/catkin_ws/src/autorally/autorally_util/setupEnvLocal.sh" >> ~/.bashrc


## ... Generate Documentation ...........................................................................................
## Run doxygen with the `-u` flag to remove obsolete configuration tag
## TODO: unmute
#RUN cd ~/catkin_ws/src/autorally/ \
#    && doxygen -u


# ... Finish container setup ...........................................................................................
COPY ../SNOW-AutoRally-dev/ros_entrypoint.sh /ros_entrypoint.sh
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.sh"
#RUN /bin/bash -c "chmod a+rw /dev/input/js0"  # Quick fix: pushed to ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS

# ///////////////////////////////////////////////////////////////////////////////////// NVIDIA-docker SNOW-AutoRally ///



