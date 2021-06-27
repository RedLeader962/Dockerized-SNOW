
# /// SNOW-AutoRally-dev ///////////////////////////////////////////////////////////////////////////////////////////////

ARG  CODE_VERSION=latest
FROM ROS-melodic-AutoRally-dependencies:${CODE_VERSION}

WORKDIR ${DEV_WORKSPACE}



# ... Finish container setup ...........................................................................................
COPY /ros_entrypoint.sh /ros_entrypoint.sh
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /ros_entrypoint.sh"
#RUN /bin/bash -c "chmod a+rw /dev/input/js0"  # Quick fix: pushed to setup_autorally.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]
WORKDIR /

# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS

# /////////////////////////////////////////////////////////////////////////////////////////////// SNOW-AutoRally-dev ///



