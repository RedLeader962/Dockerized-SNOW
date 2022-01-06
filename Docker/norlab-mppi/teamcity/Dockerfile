
# /// norlab-mppi/teamcity /////////////////////////////////////////////////////////////////////////////////////////////

ARG BASE_IMG_TAG=arm64-l4t-r32.6.1
#ARG BASE_IMG=norlab-mppi-develop
## (Priority) todo <-- we are here
ARG BASE_IMG=norlab-mppi-dependencies-wo-services
FROM norlabsnow/${BASE_IMG}:${BASE_IMG_TAG} as teamcity


ARG DS_TARGET_PROJECT_SRC_REPO=NorLab_MPPI
ENV DS_TARGET_PROJECT_SRC_REPO=${DS_TARGET_PROJECT_SRC_REPO}


WORKDIR /

# ===install teamcity & CI/CD utilities=================================================================================

RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose --upgrade pytest
RUN pip3 install --no-cache-dir --verbose teamcity-messages

RUN ( \
  echo "Configure Matplotlib to use a non-interactive backends for TeamCity run"; \
  export MPLBACKEND="Agg"; \
  echo "MPLBACKEND: $MPLBACKEND" \
)


# ===Final build step===================================================================================================
WORKDIR /
#RUN echo $PWD
COPY ./tools/ds_info.bash /
#RUN cd / && tree -L 1


# ...source & check ROS environement variable...........................................................................
# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
# Check the ROS_PACKAGE_PATH environment variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share
WORKDIR ${DS_DEV_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source ${DS_DEV_WORKSPACE}/devel/setup.bash \
    && printenv | grep -e AR_ -e ROS -e MASTER -e HOSTNAME -e DS_"



# ... Finish container setup ...........................................................................................
WORKDIR /ros_catkin_ws/src/${DS_TARGET_PROJECT_SRC_REPO}
CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/teamcity ///



