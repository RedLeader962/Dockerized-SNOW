[(return to main menu)](https://github.com/RedLeader962/Dockerized-SNOW)
# Setup for building Jetson Containers on an x86 host using _qemu_

###### References
- [Enabling _Jetson_ Containers on an x86 workstation (using _qemu_) | NVIDIA/nvidia-docker Wiki](https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson#enabling-jetson-containers-on-an-x86-workstation-using-qemu)
- [Running and Building ARM Docker Containers on x86 | Stereolabs](https://www.stereolabs.com/docs/docker/building-arm-container-on-x86/)

#### 1. Install Docker

```shell
# uninstall old docker version
sudo apt-get remove docker docker-engine docker.io containerd runc

sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
    
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

echo \
  "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update && sudo apt-get install docker-ce docker-ce-cli containerd.io

# Verify install
sudo docker run hello-world
# It should start a docker container, print a few msg like "Hello from Docker!" and exit
```

#### 2. _NVIDIA CONTAINER TOOLKIT_
##### 2.1. Check system requirement
Check the following [pre-Requisites](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#pre-requisites)
and make sure your system comply with those:
- [ ] NVIDIA Drivers are installed
- [ ] and platform requirement

###### Useful command to check nvidia driver
```shell
lspci | grep -i nvidia
uname -m && cat /etc/*release
ls -l /usr/local | grep cuda
nvidia-smi
```


##### 2.2. Install and setup _nvidia-docker_ on Ubuntu
Follow these step:
```shell
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   
sudo apt-get update
sudo apt-get install -y nvidia-docker2

sudo systemctl restart docker
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 460.80       Driver Version: 460.80       CUDA Version: 11.2     |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |                               |                      |               MIG M. |
# |===============================+======================+======================|
# |   0  GeForce GTX 108...  Off  | 00000000:2B:00.0  On |                  N/A |
# |  0%   22C    P8    17W / 280W |    323MiB / 11170MiB |      0%      Default |
# |                               |                      |                  N/A |
# +-------------------------------+----------------------+----------------------+
# 
# +-----------------------------------------------------------------------------+
# | Processes:                                                                  |
# |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
# |        ID   ID                                                   Usage      |
# |=============================================================================|
# +-----------------------------------------------------------------------------+
```

###### References: 
- [Setting up NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit)
- https://developer.nvidia.com/cuda-downloads
- [Installation (Native GPU Support) · NVIDIA/_nvidia-docker_ Wiki](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support))

#### 4. Setup _qemu_ virtualization on x86 workstation
###### References:
- [Enabling Jetson Containers on an x86 workstation (using _qemu_) | NVIDIA/nvidia-docker Wiki](https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson#enabling-jetson-containers-on-an-x86-workstation-using-qemu)
- [multiarch/qemu-user-static](https://github.com/multiarch/qemu-user-static)

##### 4.1 Install _qemu_
```shell
uname -m
# x86_64

sudo apt-get update && sudo apt-get install -y --no-install-recommends qemu qemu-user-static binfmt-support

# Check if the entries look good.
sudo cat /proc/sys/fs/binfmt_misc/status
# >>> enabled

# See if /usr/bin/qemu-aarch64-static exists as one of the interpreters.
cat /proc/sys/fs/binfmt_misc/qemu-aarch64
# enabled
# interpreter /usr/bin/qemu-aarch64-static
# flags: OCF
# offset 0
# magic 7f454c460201010000000000000000000200b700
# mask ffffffffffffff00fffffffffffffffffeffffff
``` 

##### 4.2 Fix the `F` flag
> If the flags does not include ‘F’ then the kernel is loading the interpreter lazily. The easiest fix is to have the binfmt-support package version >= 2.1.7, which automatically includes the --fix-binary (F) option. 
>
> With this in place, simply run the following command to update your flags:

```shell
sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes -c yes
```
Source: [Running or building a container on x86 (using qemu+binfmt_misc) is failing · NVIDIA/nvidia-docker Wiki](https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson#enabling-jetson-containers-on-an-x86-workstation-using-qemu)

##### 4.3 Test the _arm64_ emulation environment
```shell
sudo docker run --rm -t arm64v8/ubuntu uname -m
# aarch64
```
Ref: [multiarch/qemu-user-static](https://github.com/multiarch/qemu-user-static)

**Congrats!!! arm64 virtualization is working on your x86 machine**

#### 5. (Optional Step) Building _Jetson nvidia-docker_ containers on your _x86_ workstation 

Assuming the cuda samples are installed on the x86 workstation at `/usr/local/cuda/samples`.

```shell
sudo docker run -it --name x86host -v /usr/local/cuda:/usr/local/cuda nvcr.io/nvidia/l4t-base:r32.5.0

# The container is now running
root@x86host:/# apt-get update && apt-get install -y --no-install-recommends make g++
root@x86host:/# cp -r /usr/local/cuda/samples /tmp
root@x86host:/# cd /tmp/samples/5_Simulations/nbody
root@x86host:/# make

# Exit the container when your done
root@x86host:/# exit
```
 

#### 6. Clone the repo on your x86 workstation
```shell
sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git && cd Dockerized-SNOW
```
Note: To change directory ownership `sudo chown -R $USER ~/<myPathTo>/Dockerized-SNOW`

#### 7. Build the _nvidia-docker_ image using the following script

```shell
bash ./build_snow_dependencies.bash
bash ./build_snow_autorally_develop.bash
bash ./build_snow_autorally_deploy.bash
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
bash ./run_snow_develop.bash
bash ./run_snow_deploy.bash

# To open additional terminal in a running container 
bash ./open_new_terminal.bash <container name to execute>
```

---