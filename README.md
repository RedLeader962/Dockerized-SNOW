# _/// Dockerized SNOW //////////////////////////////////////////_

Containerized workflow for the NorLab _SNOW-AutoRally_ project leveraging [_nvidia-docker_](https://github.com/NVIDIA/nvidia-docker) technology

<p> <img src="./images/norlab_logo_acronym_dark.png" width="200"> </p>

<br>
<p>
<a href="https://viewer.diagrams.net/?target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=dockerized_snow_plan.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio">
<img src="drawio/dockerized_snow_plan.svg">
</a>
</p>
<p>
( 
<a href="https://viewer.diagrams.net/?target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=dockerized_snow_plan.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio">
fullscreen
</a>
  |  <a href="https://app.diagrams.net/?mode=github#HRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio" target="_blank" rel="noopener noreferrer">edit
</a>
diagram )
</p>
<br>

---
#### Project link: 
- [Youtrack](https://redleader.myjetbrains.com/youtrack/dashboard?id=bce3112d-bda1-425c-8628-802a047be4d3) dashboard
- [SNOW-AutoRally](https://github.com/RedLeader962/autorally)


#### Images:
- Base image: `nvcr.io/nvidia/l4t-base:r32.5.0`
- Latest images: 
   - `ros-melodic-snow-autorally-dependencies`
   - `snow-autorally-deploy`
   - `snow-autorally-develop`
   
#### _nvidia-docker_ references:
- [nvidia-docker: Build and run Docker containers leveraging NVIDIA GPUs](https://github.com/NVIDIA/nvidia-docker) 
  - [NVIDIA Container Runtime on _Jetson_](https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson)
  - [Driver containers](https://github.com/NVIDIA/nvidia-docker/wiki/Driver-containers)
- [NVIDIA Cloud Native Technologies](https://docs.nvidia.com/datacenter/cloud-native/#)

---
# Usage

##### Workflow option:

- [build the _docker_ image from a _Jetson_ device](#build-the-docker-image-from-a-jetson-device);
- [build the _docker_ image from an x86 host using _qemu_](README_cross_compiler.md);
- or pull the pre-builded _docker_ image from _DockerHub_ (TODO).

##### How-to
- [How-to _nvidia-docker_ manualy (a quick start)](README_docker_manualy_quickstart.md)
- [Test AutoRally Configuration (**Revised instruction**)](https://github.com/RedLeader962/SNOW-AutoRally#test-autorally-configuration-in-gazebo-revised-instruction)

## Build the _docker_ image from a _Jetson_ device

Requirement: the latest _Jetpack_ must be installed on the _Jetson_ 

#### 1. Add "default-runtime": "nvidia" to your _Jetson_ `/etc/docker/daemon.json` configuration file
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
#### 2. Restart the _docker_ service or reboot your system
#### 3. Clone the repo in the _Jetson_
```shell
sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git && cd Dockerized-SNOW
```

#### 7. Build the _nvidia-docker_ image using the following script
```shell
./build_snow_dependencies.bash
./build_snow_autorally_develop.bash
./build_snow_autorally_deploy.bash
```
or manualy
```shell
cd Docker/ros-melodic-snow-autorally-dependencies
sudo docker build --tag ros-melodic-snow-autorally-dependencies:<theLatestVersionTag> -f Dockerfile .
```
Note:
- The `.` is the context for the building step (It's the current directory);
- Set the `<theLatestVersionTag>` following the pattern `rX.Y` or use `latest`

#### 8. Run the container using the following script
```shell
./run_snow_develop.bash
./run_snow_deploy.bash

# To open additional terminal in a running container 
./open_new_terminal.bash
```


---



 

