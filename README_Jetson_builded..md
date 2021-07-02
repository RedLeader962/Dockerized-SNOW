[(return to main menu)](https://github.com/RedLeader962/Dockerized-SNOW)
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

#### 4. Build the _nvidia-docker_ image using the following script
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

#### 5. Run the container using the following script
```shell
./run_snow_develop.bash
./run_snow_deploy.bash

# To open additional terminal in a running container 
./open_new_terminal.bash
```


---