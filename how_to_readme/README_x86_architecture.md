[(return to main menu)](https://github.com/RedLeader962/Dockerized-SNOW)
# Using the _nvidia-docker_ image on a `x86` host

### 1. Install Docker

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

### 2. Install _NVIDIA CONTAINER TOOLKIT_
#### 2.1. Check system requirement
Check the following [pre-Requisites](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#pre-requisites)
and make sure your system comply with those:
- [ ] NVIDIA Drivers are installed
- [ ] and platform requirement

##### Useful command to check nvidia driver
```shell
lspci | grep -i nvidia
uname -m && cat /etc/*release
ls -l /usr/local | grep cuda
nvidia-smi
```


#### 2.2. Install and setup _nvidia-docker_ on Ubuntu
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

##### References: 
- [Setting up NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit)
- https://developer.nvidia.com/cuda-downloads
- [Installation (Native GPU Support) · NVIDIA/_nvidia-docker_ Wiki](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support))


### 3. Clone the repo on your x86 workstation
```shell
sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git && cd Dockerized-SNOW
```
Note: To change directory ownership `sudo chown -R $USER ~/<myPathTo>/Dockerized-SNOW`

### 4. Build the _nvidia-docker_ image using the following script
Use the `--help` flag for instruction
```shell
bash ./build_snow_dependencies.bash --x86
bash ./build_snow_develop.bash --x86
bash ./build_snow_deploy.bash --x86
```

### 5. Run the container using the following script
```shell
bash ./run_snow_develop.bash --x86
bash ./run_snow_deploy.bash --x86

# To open additional terminal in a running container 
bash ./open_new_terminal.bash <container name to execute>
```

---