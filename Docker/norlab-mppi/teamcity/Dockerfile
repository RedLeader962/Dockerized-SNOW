
# /// norlab-mppi/teamcity /////////////////////////////////////////////////////////////////////////////////////////////

ARG BASE_IMG_TAG=arm64-l4t-r32.6.1
ARG BASE_IMG=norlab-mppi-develop
FROM norlabsnow/${BASE_IMG}:${BASE_IMG_TAG} as teamcity

WORKDIR /

# ===install teamcity & CI/CD utilities=================================================================================

RUN pip3 install --no-cache-dir --verbose --upgrade pip
RUN pip3 install --no-cache-dir --verbose --upgrade pytest
RUN pip3 install --no-cache-dir --verbose teamcity-messages


# ... Finish container setup ...........................................................................................

WORKDIR /ros_catkin_ws/src/${DS_TARGET_PROJECT_SRC_REPO}
#CMD [ "bash" ]
CMD [ "pwd" ]

# ///////////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/teamcity ///



