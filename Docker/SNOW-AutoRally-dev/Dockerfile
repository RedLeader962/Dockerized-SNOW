
# /// SNOW-AutoRally-dev ///////////////////////////////////////////////////////////////////////////////////////////////

ARG  CODE_VERSION=latest
FROM ros-melodic-auto-rally-dependencies:${CODE_VERSION}


# ... Finish container setup ...........................................................................................
WORKDIR /
COPY build_autorally.bash /
RUN /bin/bash -c "chmod +x /build_autorally.bash"

#CMD [ "bash" ]
# /////////////////////////////////////////////////////////////////////////////////////////////// SNOW-AutoRally-dev ///



