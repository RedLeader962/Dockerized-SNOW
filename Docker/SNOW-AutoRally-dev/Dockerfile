
# /// SNOW-AutoRally-dev ///////////////////////////////////////////////////////////////////////////////////////////////

ARG  CODE_VERSION=latest
FROM ros-melodic-auto-rally-dependencies:${CODE_VERSION}

# === Clone the SNOW-AutoRally fork of AutoRally repo and install ======================================================
#
# Steps:
#   1. clone AutoRally repos
#   2. Install AutoRally ROS Dependencies
#   3. Compilation & Running
# Note:
#   - No need to build Pointgrey Camera driver from source anymore (apparently).
#       See pullrequest 243548 merge into `ros:master` on 3 Apr 2020: https://github.com/ros/rosdistro/pull/24348

ARG DEV_BRANCH=SNOW-melodic-devel

#WORKDIR ${DEV_WORKSPACE}/src








# ... Finish container setup ...........................................................................................
WORKDIR /
COPY build_autorally.sh /

#ENTRYPOINT [ "/build_autorally.sh" ]
CMD [ "sh" ]


# /////////////////////////////////////////////////////////////////////////////////////////////// SNOW-AutoRally-dev ///



