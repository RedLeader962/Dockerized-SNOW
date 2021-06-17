# SNOW-AutoRally nvidia-docker
## Build:
1. Add "default-runtime": "nvidia" to your `/etc/docker/daemon.json` configuration file
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
2. Restart the Docker service or reboot your system .
3. Execute build command
    ```bash
    sudo docker build -t snow-autorally-l4t-ros-melodic-full:r1.0 -f ROS-melodic-AutoRally.Dockerfile .
    ```

## Usage:
```bash
export DISPLAY=:0
sudo xhost +si:localuser:root
sudo docker run --device=/dev/input/js0 --runtime nvidia --gpus all --network host --name joystick -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix snow-autorally-l4t-ros-melodic-full:r1.3
```

**Flags Options Explained:**
- `--runtime` nvidia refers to using the NVIDIA container runtime while running the l4t-base container
- `-it` refers to running in interactive mode
- `-v` refers to the mounting directory, and used to mount host’s X11 display in the container filesystem to render output videos
- `--name` refers to the specification of the container name
- `-d`or `--device` refers to mapping an attached device such as camera to the container with full access
- `-H` or `--hostname` specifies remote host name: eg. if you want to execute the run command on your Xavier
- `-p` or `--publish` publish a container’s port(s) to the host, necessary when you need a port to communicate with a program in your container.

# Opening new terminal access in the container
```bash
sudo docker exec -it myCoolContainerName bash

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
