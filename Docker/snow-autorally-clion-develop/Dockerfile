# Ref:
#   - https://github.com/JetBrains/clion-remote/blob/master/Dockerfile.remote-cpp-env
#   - https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/

# ❯❯❯ Original comment ❯❯❯ . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# CLion remote docker environment (How to build docker container, run and stop it)
#
# Build and run:
#   docker build -t clion/remote-cpp-env:0.5 -f Dockerfile.remote-cpp-env .
#   docker run -d --cap-add sys_ptrace -p127.0.0.1:2222:22 --name clion_remote_env clion/remote-cpp-env:0.5
#   ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[localhost]:2222"
#
# stop:
#   docker stop clion_remote_env
#
# ssh credentials (test user):
#   user@password
#  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .❮❮❮ Original comment ❮❮❮

#FROM ubuntu:20.04
#
#RUN DEBIAN_FRONTEND="noninteractive" apt-get update && apt-get -y install tzdata

# /// snow-autorally-clion-develop /////////////////////////////////////////////////////////////////////////////////////
#
# Usage:
#
#   $ docker run -td \
#                --name clion-remote-dev \
#                --network host \
#                --cap-add sys_ptrace \
#                --security-opt seccomp=unconfined \
#                --security-opt apparmor=unconfined \
#                norlabsnow:snow-autorally-clion-develop:arm64-l4t
#
#   Note:
#       The -d runs the container as a daemon, so control returns back to you.
#         --cap-add sys_ptrace
#       adds the ptrace capability, which is necessary for debugging.
#       Ref: https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/
#
#       Use the following docker run flags for gdb debbuging
#          --security-opt seccomp=unconfined --security-opt apparmor=unconfined
#       Ref: https://austinmorlan.com/posts/docker_clion_development/
#
# Connecting to the container:
#
#   ssh in the container using:
#      $ ssh <user>@<remote host ip> -p2222
#
#   <remote host ip> = The container use the same ip adresse as the host network,
#      but the container ssh server port is maped to 2222
#
#   If you previously ssh in a similar host, execute:
#       $ ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[localhost]:2222"
#
#   How to use the container mapped ssh port
#
#       $ ssh -p {port} user@<remote host ip>>
#       $ sftp -P {port} openssh-<remote host ip>>
#       $ scp -P {port} source target
#       $ scp -P {port} /path/to/foo user@<remote host ip>>:/dest/
#
# Fetching the ROS environent variable
#   1. source/build your ROS distro or run:
#           $ bash build_snow_autorally.bash
#   2. Collect the environment variable with this script:
#       ros_env="AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH LD_LIBRARY_PATH PATH ROS_DISTRO ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_VERSION"
#       env_string=""
#       for e in ${ros_env}; do
#           env_string+="$e=${!e};"
#       done
#       echo "$env_string"
#
# Credit: https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/

ARG  BASE_IMG_TAG=arm64-l4t
FROM norlabsnow/snow-autorally-develop:${BASE_IMG_TAG}

# ...install remote dev/debug utilities.................................................................................
RUN apt-get update \
    && apt-get upgrade --assume-yes \
    && apt-get install --assume-yes --no-install-recommends \
        openssh-server \
#        ssh \
#        build-essential \
        gcc \
        g++ \
        gdb \
        clang \
#        cmake \
        rsync \
        tar \
#        python \
        gdbserver \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ...Setup ssh server...................................................................................................
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Port 2222'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_clion_snow_dev \
  && mkdir /run/sshd

# ssh port, remaped from default 22 to 2222
ARG CLION_DEV_SERVER_PORT=2222
ENV CLION_DEV_SERVER_PORT=${CLION_DEV_SERVER_PORT}
EXPOSE ${CLION_DEV_SERVER_PORT}
# gdbserver port
EXPOSE 7777

# ...Add new user.......................................................................................................
ARG NEW_USER=snow-clion
ENV CLION_DEV_USER=${NEW_USER}
ARG PASSWORD=lasagne

RUN useradd -m ${NEW_USER} \
  && yes ${PASSWORD} | passwd ${NEW_USER}

# (Optional) Change default shell for new user
RUN usermod -s /bin/bash ${NEW_USER}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY clion_dev_entrypoint.bash /
COPY fetch_ros_env.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /clion_dev_entrypoint.bash"

# The ssh server is started by the entrypoint
ENTRYPOINT [ "/clion_dev_entrypoint.bash" ]
#CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]
CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////// snow-autorally-clion-develop ///


