# /// _SNOW-AutoRally nvidia-docker_ ////////////////////////////
- Latest: `snow-autorally-l4t-ros-melodic-full`
- Base image: `nvcr.io/nvidia/l4t-base:r32.5.0` 

## Setup alternative:
- build the docker image from a Jetson device;
- build the docker image from an x86_64 host using qemu (TODO);
- or pull the pre-builded docker image from DockerHub (TODO).

### Build the docker image from a Jetson device
1. Add "default-runtime": "nvidia" to your Jetson `/etc/docker/daemon.json` configuration file
    ```bash
    {
        "runtimes": {
            "nvidia": {
                "path": "nvidia-container-runtime",
                "runtimeArgs": []
            }
        },
        "default-runtime": "nvidia"
    }
    ```
2. Restart the Docker service or reboot your system
3. Clone the repo in the jetson 
   ```bash
   cd /opt
   git clone https://github.com/RedLeader962/SNOW_AutoRally.git
   cd /opt/SNOW_AutoRally 
   ```
4. Build the nvidia-docker image
    ```bash
    sudo docker build -t snow-autorally-l4t-ros-melodic-full:<theLatestVersionTag> -f ROS-melodic-AutoRally.Dockerfile .
    ```
   Note: 
   - The `.` is the context for the building step (It's the current directory);
   - Set the `<theLatestVersionTag>` following the pattern `rX.Y`
   

### Build the docker image from an x86_64 host using qemu 
(TODO)
### Pull the pre-build container from DockerHub 
(TODO)


## Usage:
### Instantiate a new container
Note on configuration:
- run Gazebo GUI on the host computer via X11
- map the default joystick usb input from the host (eg. a Jetson Xavier) to the container    
```bash
export DISPLAY=:0
sudo xhost +si:localuser:root
sudo docker run \
    --device=/dev/input/js0 \
    --runtime nvidia --gpus all \ 
    --network host \
    --name <theContainerCoolName> \
    --interactive \
    --tty \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix/:/tmp/.X11-unix \
    snow-autorally-l4t-ros-melodic-full:<theLatestVersionTag>
```

**Flags explanation:**
- `--name` a meaningful container name

**Flags explanation:**
- `--runtime nvidia` set the container to use the NVIDIA container runtime  
- `--volume` or `-v` set a mounting directory.
  We use this to mount the host X11 display in the container filesystem. Rendered output videos from the container can then be displayed on the host.
- `--device`or `-d` set full access from a container to a host attached device (eg. joystick, camera)  

**Others usefull flags:**
- `--hostname` or `-H` specifies remote host name: eg. if you want to execute the run command on your Xavier
- `--publish` or `-p` publish a container’s port(s) to the host, necessary when you need a port to communicate with a program in your container.

### Stop and start a container
```bash
sudo docker stop <myCoolContainerName>

sudo docker start -i <myCoolContainerName>
```

###  Opening new terminal access in a running container
```bash
sudo docker exec -it <myCoolContainerName> bash

# ex:
sudo docker exec -it amazing_vaughan bash
```

---
# Quick ref
```bash
printenv | grep ROS 

```
 
---
# Test AutoRally Configuration in Gazebo 
## 1. Start AutoRally simulator
```bash
roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch
```

## 2. Configure joystick with ROS
[joy/Tutorials/ConfiguringALinuxJoystick - ROS Wiki](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
 
1. Check if os recognize the joystick 
    ```bash
    # check listing of input device
    ls /dev/input/
    
    # check if the joystick is working (could be an other device eg. `js3`) 
    sudo jstest /dev/input/js0
    # move joystick and check data
    ```
2. Fix permission 
    ```bash
    sudo chmod a+rw /dev/input/js0
    ```

3. Start AutoRally 
4. set `rosparam` joy_node/dev 
    ```bash
    rosparam set joy_node/dev "/dev/input/js0"
    rosparam get joy_node/dev
    ```
5. restart `joy_node` and check topic data
   ```bash
    rosrun joy joy_node
    
    # in a new terminal
    rostopic echo joy
    # move joystick and check data
    ```

## 3. Verify if runstop motion is enabled 
```bash
rostopic echo /chassisState
# check for the runstopMotionEnabled field 
```
 
---
# Autonomous Driving in Simulation
Reset robot position the robot in the same spot as when the simulation starts and check if `runstopMotionEnabled = true`.

1. Start waypoint follower:
    ```bash
    roslaunch autorally_control waypointFollower.launch
    ```

2. Start constant speed controller and tell it what speed to drive:
    ```bash
    roslaunch autorally_control constantSpeedController.launch
    rosrun rqt_publisher rqt_publisher
    ```

> Configure a publisher on topic `constantSpeedController/speedCommand` of type `std_msgs/Float64` at rate 10 with value of 3 (you can adjust he value once everything is running). The value is the target velocity in m/s, and **as soon as you do this the platform should move if motion is enabled**.
>
> If the robot turns and hits the barrier it's probably because the state estimator has not converged, so its orientation estimate is incorrect. Just select the track barriers and move them up to allow the robot to continue driving, and the estimator should converge and the vehicle will return to within the barriers.
