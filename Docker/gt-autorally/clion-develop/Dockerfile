
# /// gt-autorally/clion-develop /////////////////////////////////////////////////////////////////////////////////////
# Purpose:
#   - build source file remotly
#   - use remote GDB debugging tools
#
# Usage:
#
#   $ docker run -td \
#                --name clion-remote-dev \
#                --network host \
#                --cap-add sys_ptrace \
#                --security-opt seccomp=unconfined \
#                --security-opt apparmor=unconfined \
#                norlabsnow:gt-autorally/clion-develop:arm64-l4t
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
#   Running sshd inside a container is not recommended normaly but it's a requirement in our case
#       https://docs.docker.com/samples/running_ssh_service/
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
#
# Ref:
#   - https://github.com/JetBrains/clion-remote/blob/master/Dockerfile.remote-cpp-env
#   - https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/

ARG  BASE_IMG_TAG=arm64-l4t-r32.5.0
FROM norlabsnow/gt-autorally-develop:${BASE_IMG_TAG}

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
        tree \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ...development tools & libraries......................................................................................
RUN apt-get update \
    && apt-get upgrade --assume-yes \
    && apt-get install --assume-yes --no-install-recommends \
        python-pip \
        python-wheel \
        python-virtualenv \
        python-pytest \
        python-setuptools \
        python-termcolor \
        python-frozendict \
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
ARG NEW_USER=clion-debugger
ENV CLION_DEV_USER=${NEW_USER}
ARG PASSWORD=lasagne

RUN echo 'root:SNOW' | chpasswd
RUN useradd -m ${NEW_USER} \
  && yes ${PASSWORD} | passwd ${NEW_USER}

# (Optional) Change default shell for new user
#RUN usermod -s /bin/bash ${NEW_USER}

# ... Finish container setup ...........................................................................................
WORKDIR /
COPY clion_dev_entrypoint.bash /
COPY rebuild_gt_autorally.bash /
COPY fetch_ros_env.bash /
# set read/write permission to entrypoint file and joystick dir js0
RUN /bin/bash -c "chmod +x /clion_dev_entrypoint.bash" \
    && /bin/bash -c "chmod +x /rebuild_gt_autorally.bash" \
    && /bin/bash -c "chmod +x /fetch_ros_env.bash"

# The ssh server is started by the entrypoint
ENTRYPOINT [ "/clion_dev_entrypoint.bash" ]
#CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]
CMD [ "bash" ]

# ///////////////////////////////////////////////////////////////////////////////////// gt-autorally/clion-develop ///


