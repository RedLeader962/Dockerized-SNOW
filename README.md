# /// _Dockerized SNOW_ /////////////////////////////////////////////////

Containerized workflow for the _SNOW-AutoRally_ project leveraging _nvidia-docker_ technology

- Base image: `nvcr.io/nvidia/l4t-base:r32.5.0`
- Latest images: 
  - `ros-melodic-auto-rally-dependencies`
  - `snow-auto-rally-deploy`
  - `snow-auto-rally-dev`

<br>
<p>
<a href="https://viewer.diagrams.net/?target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=dockerized_snow_plan.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FRedLeader962%2FSNOW_AutoRally%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio">
<img src="drawio/dockerized_snow_plan.svg">
</a>
</p>
<p>
( <a href="https://app.diagrams.net/?mode=github#HRedLeader962%2FSNOW_AutoRally%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio" target="_blank" rel="noopener noreferrer">Edit
</a>
diagram)
</p>
<br>

---
#### Remote development quick ref

```shell
ssh snowxavier@10.0.1.103
sudo docker exec -it thirsty_dirac bash
```

## Setup alternative:

- build the docker image from a Jetson device;
- build the docker image from an x86_64 host using qemu (TODO);
- or pull the pre-builded docker image from DockerHub (TODO).

### Build the docker image from a Jetson device

1. Add "default-runtime": "nvidia" to your Jetson `/etc/docker/daemon.json` configuration file
    ```shell
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
   ```shell
   cd /opt
   git clone https://github.com/RedLeader962/SNOW_AutoRally.git
   cd /opt/SNOW_AutoRally 
   ```
4. Build the nvidia-docker image
    ```shell
    sudo docker build -t snow-autorally-l4t-ros-melodic-full:<theLatestVersionTag> -f Dockerfile .
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

---

# Quick ref

```shell
printenv | grep ROS 
```

 
---

# Test AutoRally Configuration in Gazebo

## 1. Start AutoRally simulator

```shell
roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch
```

## 2. Configure joystick with ROS

[joy/Tutorials/ConfiguringALinuxJoystick - ROS Wiki](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

1. Check if os recognize the joystick
    ```shell
    # check listing of input device
    ls /dev/input/
    
    # Install jstest-gtk 
    # check if the joystick is working (could be an other device eg. `js3`) 
    sudo jstest /dev/input/js0
    # move joystick and check data
    ```
2. Fix permission
   ```shell
   # check permission 
   ls -l /dev/input/js0
   
   # fix
   sudo chmod a+rw /dev/input/js0
   ```

3. Start AutoRally
4. set `rosparam` joy_node/dev
    ```shell
    rosparam set joy_node/dev "/dev/input/js0"
    rosparam get joy_node/dev
    ```
5. restart `joy_node` and check topic data
   ```shell
    rosrun joy joy_node
    
    # in a new terminal
    rostopic echo joy
    # move joystick and check data
    ```

## 3. Verify if runstop motion is enabled

```shell
rostopic echo /chassisState
# check for the runstopMotionEnabled field 
```

---

# Quick hack for AutoRally unrecognized device problem

Pipeline: device ››› Xavier L4T ››› nvidia-container ››› `joystickController` ›››  `StateEstimator`
››› `path_integral_nn`
Requirement for MPPI: source of `runstop` information via device

### Assess recognize device attributes using from udev device manager

```shell
udevadm info -a -p $(udevadm info -q path -n /dev/input/js0)

# AutoRally implementation in setupEnvVariable.sh
# ID_INPUT_JOYSTICK is to restrictive!
udevadm info -n /dev/input/js0 -q property --export | grep ID_INPUT_JOYSTICK
```  

💎 | Quick hack

1. Start the `snow-autorally-l4t-ros-melodic-full` container,
2. if ```[WARNING] No joystick detected.``` is printed in the terminal,
3. overwrite the `AR_JOYSTICK` environment variable generated by  ```autorally/autorally_util/setupEnvVariables.sh```

```shell
sudo docker start -i <myCoolContainerName>
#or
sudo docker exec -it amazing_vaughan bash

# > [WARNING] No joystick detected.

export AR_JOYSTICK=/dev/input/js0 

printenv | grep AR_
```

 
---

# Autonomous Driving in Simulation

1. Launch an AutoRally simulation in gazebo
   ```shell
   roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch
   ```
   or reset the robot position in Gazebo at the same spot as when the simulation starts
   ```shell
   rosservice call /gazebo/reset_simulation
   ```
2. Open a new terminal, subscribe to this topic
   ```shell
   rostopic echo /chassisState
   ```
   and set `runstopMotionEnabled = true` using the joystick
3. Start waypoint follower:
    ```shell
    roslaunch autorally_control waypointFollower.launch
    ```
4. Start constant speed controller and tell it what speed to drive:
    ```shell
    roslaunch autorally_control constantSpeedController.launch
    rosrun rqt_publisher rqt_publisher
    ```

> Configure a publisher on topic `constantSpeedController/speedCommand` of type `std_msgs/Float64` at rate 10 with value of 3 (you can adjust he value once everything is running). The value is the target velocity in m/s, and **as soon as you do this the platform should move if motion is enabled**.

```shell
rostopic pub -r 10 /constantSpeedController/speedCommand std_msgs/Float64 '{data: 1.5}'
```

> If the robot turns and hits the barrier it's probably because the state estimator has not converged, so its orientation estimate is incorrect. Just select the track barriers and move them up to allow the robot to continue driving, and the estimator should converge and the vehicle will return to within the barriers.


---

# Autonomous Driving in Simulation using MPPI

1. Launch an AutoRally simulation in gazebo
   ```shell
   roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch
   ```

   Reset the robot position in Gazebo at the same spot as when the simulation starts (if needed)
   ```shell
   rosservice call /gazebo/reset_world
   ```
2. Open a new terminal, subscribe to those topic
   ```shell
   rostopic echo /chassisState
   ```
   and set `runstopMotionEnabled = true` using the joystick **BUT DONT MOVE THE ROBOT!**
3. launch the state estimator with those arguments specific to the simulator case
   ```shell
   # check available parameter
   roslaunch --ros-args autorally_core stateEstimator.launch 
     
   # start the node with the argument for running in simulation 
   roslaunch autorally_core stateEstimator.launch InvertY:=false InvertZ:=false FixedInitialPose:=true sim:=true
   ```
   Note: the AutoRally doc as an obsolete procedure for lauching
   the [State Estimator](https://github.com/AutoRally/autorally/wiki/State-Estimator). It tell to overwrite the setting
   on the parameter server using `rosparam set /gps_imu/FixedInitialPose true` but the `stateEstimator.launch` file as a
   param `sim` with default argument `false` that overwrite this setting. Pass argument `sim:=true` to the launcher
   instead.
4. move the vehicle around manually until the state estimator has converged
5. Open a new terminal and start MPPI
   ```shell
   roslaunch autorally_control path_integral_nn.launch
   ```
6. To change configuration as you run
   (
   see [MPPI · AutoRally/autorally Wiki](https://github.com/AutoRally/autorally/wiki/Model-Predictive-Path-Integral-Controller-(MPPI)#dynamic-reconfigure-variables)
   for more `dynamic_reconfigure` parameter details)
   > - `max_throttle`: Maximum applied throttle, Def:0.65, min: 0.0, max: 1.0
   > - `desired_speed`: Speed Target for the MPPI controller, Def:6.0, min: 0.0, max:  25.0
   > - `speed_coefficient`: Weight for acheiving target velocity, Def:4.25, min: 0.0, max:  20.0
   > - `track_coefficient`: Weight for staying on the track, Def:200.0, min: 0, max: 500.0
   > - `max_slip_angle`: maximum allowable slip angle before killing trajectory, Def:1.25, min: 0.0, max: 3.14
   > - `slip_penalty`: Penalty for violating slip angle threshold, Def:10.0, min: 0, max: 1000.0
   > - `crash_coefficient`: Penalty for crashing, Def:10000, min: 0, max: 20000
   > - `track_slop`: Value for clipping track cost to zero, Def:0, min: 0, max: .75
   > - `steering_coeff`: Steering Cost Coefficient, Def:0.0, min: 0, max: 1.0
   > - `throttle_coeff`: Throttle Cost Coefficient, Def:0.0, min: 0, max: 1.0
   ```shell
   rosrun dynamic_reconfigure dynparam set /mppi_controller desired_speed 7.5
   rosrun dynamic_reconfigure dynparam set /mppi_controller speed_coefficient 25.0  
   rosrun dynamic_reconfigure dynparam set /mppi_controller track_coefficient 200.0  
   rosrun dynamic_reconfigure dynparam set /mppi_controller max_throttle 1.0 
   rosrun dynamic_reconfigure dynparam set /mppi_controller max_slip_angle 3.14  
   rosrun dynamic_reconfigure dynparam set /mppi_controller slip_penalty 100.0  
   ```
7. Open a new terminal, subscribe to this topic and make sure the controller is run **extremely close** to 40 hz
   ```shell
   rostopic hz /mppi_controller/chassisCommand
   ```

---

# Simulator services

These services allow the user to pause and unpause physics in simulation:

Ref. [Gazebo : Tutorial : ROS communication](http://gazebosim.org/tutorials/?tut=ros_comm#Services:Simulationcontrol)

```shell
# Resets the entire simulation including the time
rosservice call /gazebo/reset_simulation

# Resets the model's poses
rosservice call /gazebo/reset_world

# Pause physics updates.
rosservice call /gazebo/pause_physics

# Resume physics updates.
rosservice call /gazebo/unpause_physics
```
