[(return to main menu)](https://github.com/RedLeader962/Dockerized-SNOW)
## Using the _nvidia-docker_ image on a _Jetson_ device (`arm64-l4t`)

Requirement: the latest _Jetpack_ must be installed on the _Jetson_ 

### 1. Add "default-runtime": "nvidia" to your _Jetson_ `/etc/docker/daemon.json` configuration file
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
### 2. Restart the _docker_ service or reboot your system
### 3. Clone the repo in the _Jetson_
```shell
sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git && cd Dockerized-SNOW
```

### 4. Build the _nvidia-docker_ image using the following script
Use the `--help` flag for instruction

```shell
bash ./ds_build_dependencies.bash
bash ./ds_build_develop.bash
bash ./ds_build_deploy.bash
```


### 5. Run the container using the following script
```shell
bash ./ds_instantiate_develop.bash
bash ./ds_instantiate_deploy.bash

# To open additional terminal in a running container 
bash ./ds_attach.bash <container name to execute>
```



---