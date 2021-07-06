[(return to main menu)](https://github.com/RedLeader962/Dockerized-SNOW)
# How-to _nvidia-docker_ manualy (a quick start)

## Usage:

### Instantiate a new container manualy

Note on configuration:

- the container run Gazebo GUI on the host computer via X11;
- and map the default joystick usb input from the host (eg. a Jetson Xavier) to the container.

```shell
export DISPLAY=:0
sudo xhost +si:localuser:root
sudo docker run 
    --device=/dev/input/js0 
    --runtime nvidia --gpus all  
    --network host 
    --name <theContainerCoolName> 
    --interactive 
    --tty 
    --env DISPLAY=$DISPLAY 
    --volume /tmp/.X11-unix/:/tmp/.X11-unix 
    snow-autorally-l4t-ros-melodic-full:<theLatestVersionTag>
```

**Flags explanation:**

- `--name` a meaningful container name

**Flags explanation:**

- `--runtime nvidia` set the container to use the NVIDIA container runtime
- `--volume` or `-v` set a mounting directory. We use this to mount the host X11 display in the container filesystem.
  Rendered output videos from the container can then be displayed on the host.
- `--device`or `-d` set full access from a container to a host attached device (eg. joystick, camera)

**Others usefull flags:**

- `--hostname` or `-H` specifies remote host name: eg. if you want to execute the run command on your Xavier
- `--publish` or `-p` publish a container’s port(s) to the host, necessary when you need a port to communicate with a
  program in your container.

### Stop and start a container

```shell
sudo docker stop <myCoolContainerName>

sudo docker start -i <myCoolContainerName>
```

### Opening new terminal access in a running container

```shell
sudo docker exec -it <myCoolContainerName> bash

# ex:
sudo docker exec -it amazing_vaughan bash
```
