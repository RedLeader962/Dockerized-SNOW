
<div align="center">
<a href="https://norlab.ulaval.ca">
<img src="visual/norlab_logo_acronym_dark.png" width="200">
</a>
<br>
<br>
<br>
</div>

# _Dockerized-SNOW_


<div align="center">
<p>
<sup>
<b>Project related link: </b> &nbsp; 
<a href="https://redleader.myjetbrains.com/youtrack/dashboard?id=bce3112d-bda1-425c-8628-802a047be4d3">NLSAR</a>
(Youtrack) &nbsp; • &nbsp;  
<a href="https://hub.docker.com/u/norlabsnow">norlabsnow</a>
(Docker Hub) &nbsp; • &nbsp; 
<a href="https://github.com/norlab-ulaval/NorLab_MPPI">NorLab_MPPI</a>
(GitHub) &nbsp; • &nbsp; 
<a href="https://github.com/RedLeader962/SNOW_AutoRally">SNOW_AutoRally</a>
(GitHub)
&nbsp;
</sup>
</p>

[comment]: <> (<br>)
</div>

Containerized development workflow for the _NorLab_MPPI_ and _SNOW_AutoRally_ projects leveraging [_nvidia-docker_](https://github.com/NVIDIA/nvidia-docker) technology. 
Believe it or not, it's configured for developing with **ROS melodic** in **python 3.6**.

**Key benefit:** custom dependency management, development environment consistency, easy deployment to robots compute box and results reproducibility.    
 

Author: [Luc Coupal](https://redleader962.github.io) 

![](visual/splash_and_promt_screenshot_6.png)

<!-- 
```shell

                                          .|'''.|                                               
                                          ||..  '                                               
····································· Dockerized-SNOW ••········································
                                           ''|||.                                               
                                         .     '||                                              
                                         |'....|'                                               

```
-->

<!-- 
<br>
<div align="center">
<p>
<a href="https://viewer.diagrams.net/?target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=dockerized_snow_plan.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio">
<img src="drawio/dockerized_snow_plan.svg">
</a>
</p>
<sub>
(&nbsp; View diagram  
<a href="https://viewer.diagrams.net/?target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=dockerized_snow_plan.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio">
fullscreen
</a>
&nbsp; • &nbsp;
<a href="https://app.diagrams.net/?mode=github#HRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_plan.drawio" target="_blank" rel="noopener noreferrer">edit
</a>
diagram &nbsp;)
</sub>
</div>
<br>
-->

<br>

## Container prompt legend

<br>

<div align="center">
<p>
<img src="drawio/prompt_legend.svg">
</p>
</div>


[comment]: <> (![]&#40;drawio/prompt_legend.png&#41;)

<br>

## Usage

<details open>
  <summary style="font-size: large;">Quick start for the NorLab_MPPI project on <em>x86</em> workstation</summary>
  <br>

  - **Requirement:** _docker_ and _nvidia container toolkit_ must be installed ([follow install step 1 and 2](how_to_readme/README_x86_architecture.md))
  - **Tips:** You can use the `--help` flag for usage instruction on most  `ds_*` command
  
  ```shell
  # Create a directory for your development source code if you dont already have one
  mkdir -p ~/Repositories && cd ~/Repositories
  
  # Clone both repositories 
  sudo git clone https://github.com/norlab-ulaval/NorLab_MPPI.git
  sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git
  
  # Install aliases and check Nvidia NVCC  
  cd ~/Repositories/Dockerized-SNOW
  source ds_setup.bash
  
  # Pull the norlab-mppi-develop image from norlabsnow Dockerhub with the x86-ubuntu18.04 tag
  sudo docker pull norlabsnow/norlab-mppi-develop:x86-ubuntu18.04 
  
  # Create a new docker image instance for development on your machine and start working on the 
  # NorLab_MPPI project using ROS melodic, Python 3 and Pytorch right away.
  ds_instantiate_develop --runTag=x86-ubuntu18.04 --name=MyCoolName --src="$HOME/Repositories/NorLab_MPPI"

  ```

  To open an terminal inside _MyCoolNmae_, use the following convenient script 
  ```shell 
  ds_attach MyCoolNmae
  ```
  or use `sudo docker exec -it MyCoolName bash`

  <br>
</details>


<details>
  <summary style="font-size: large;">Quick start for the NorLab_MPPI project on <em>Apple M1 (arm64)</em> workstation</summary>
  <br>

  - It's the same image as the `arm64-l4t` but with PyTorch and Numba compiled specifically for `arm64-Darwin`
  - **Tips:** You can use the `--help` flag for usage instruction on most  `ds_*` command
  - Be advise, cuda is not supported on Apple computer so PyTorch and Numba will work on cpu 
  
  ```shell
  # Create a directory for your development source code if you dont already have one
  mkdir -p ~/Repositories && cd ~/Repositories
  
  # Clone both repositories 
  sudo git clone https://github.com/norlab-ulaval/NorLab_MPPI.git
  sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git
  
  # Install aliases
  cd ~/Repositories/Dockerized-SNOW
  source ds_setup.bash
  
  # Pull the norlab-mppi-develop image from norlabsnow Dockerhub with the arm64-Darwin-ubuntu18.04 tag
  sudo docker pull norlabsnow/norlab-mppi-develop:arm64-Darwin-ubuntu18.04
  
  # Create a new docker image instance for development on your machine and start working on the 
  # NorLab_MPPI project using ROS melodic, Python 3 and Pytorch right away.
  ds_instantiate_develop --runTag=arm64-Darwin-ubuntu18.04 --osx --name=MyCoolName --src="$HOME/Repositories/NorLab_MPPI"

  ```

  To open an terminal inside _MyCoolName_, use the following convenient script 
  ```shell 
  ds_attach MyCoolName
  ```
  or use `sudo docker exec -it MyCoolName bash`

  <br>
</details>

<details>
  <summary style="font-size: large;">Quick start for the NorLab_MPPI project on <em>Apple (x86)</em> workstation</summary>
  <br>

  - **Tips:** You can use the `--help` flag for usage instruction on most  `ds_*` command
  - Be advise, cuda is not supported on Apple computer so PyTorch and Numba will work on cpu only
  
  Use a x86 docker image and let docker use roseta to emulate the x86 architecture
  ```shell
  # Install roseta
  softwareupdate --install-rosetta
  ```

  ```shell
  # Create a directory for your development source code if you dont already have one
  mkdir -p ~/Repositories && cd ~/Repositories
  
  # Clone both repositories 
  sudo git clone https://github.com/norlab-ulaval/NorLab_MPPI.git
  sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git
  
  # Install aliases 
  cd ~/Repositories/Dockerized-SNOW
  source ds_setup.bash
  
  # Pull the norlab-mppi-develop image from norlabsnow Dockerhub with the arm64-Darwin-ubuntu18.04 tag
  sudo docker pull norlabsnow/norlab-mppi-develop:x86-ubuntu18.04
  
  # Create a new docker image instance for development on your machine and start working on the 
  # NorLab_MPPI project using ROS melodic, Python 3 and Pytorch right away.
  ds_instantiate_develop --platform='linux/amd64' --runTag=x86-ubuntu18.04 --osx --name=MyCoolName --src="$HOME/Repositories/NorLab_MPPI"
  # Runnning `uname -m` inside the container will confirm the type of architecture

  ```

  To open an terminal inside _MyCoolname_, use the following convenient script 
  ```shell 
  ds_attach MyCoolname
  ```
  or use `sudo docker exec -it MyCoolName bash`

  <br>
</details>


<details>
  <summary style="font-size: large;">Quick start for the SNOW_AutoRally project on <em>x86</em> workstation</summary>
  <br>

  - **Requirement:** _docker_ and _nvidia container toolkit_ must be installed ([follow install step 1 and 2](how_to_readme/README_x86_architecture.md))
  - **Tips:** You can use the `--help` flag for usage instruction on any dockerized-snow bash script
  
  ```shell
  # Create a directory for your development source code if you dont already have one
  mkdir -p ~/Repositories && cd ~/Repositories
  
  # Clone both repositories 
  sudo git clone https://github.com/RedLeader962/SNOW_AutoRally.git
  sudo git clone https://github.com/RedLeader962/Dockerized-SNOW.git
  cd ~/Repositories/Dockerized-SNOW
  
  # Pull the norlab-mppi-develop image from norlabsnow Dockerhub with the x86-ubuntu18.04 tag
  bash ds_build_dependencies.bash --x86 --GT-AR
  bash ds_build_develop.bash --x86 --GT-AR
  
  
  # Create a new docker image instance for development on your machine and start working on the 
  # NorLab_MPPI project using ROS melodic, Python 3 and Pytorch right away.
  bash ds_instantiate_develop.bash --runTag=x86-ubuntu18.04 --name=THEgtar --src="$HOME/Repositories/SNOW_AutoRally"
  
  
  
                                            .|'''.|                                               
                                            ||..  '                                               
  ····································· Dockerized-SNOW ••········································
                                             ''|||.                                               
                                           .     '||                                              
                                           |'....|'                                               
  
                                   https://norlab.ulaval.ca                                       
                                https://redleader962.github.io                                    
  
  root@norlab-og:/#
  ```
  Then follow the step at [SNOW_AutoRally: Autonomous Driving in Simulation using MPPI](https://github.com/RedLeader962/SNOW_AutoRally#autonomous-driving-in-simulation-using-mppi)
  ```docker
  roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch
  
  ...
  ```

  <br>
</details> 

<br>


## High level architecture  

<div align="center">
<p>
<a href="https://viewer.diagrams.net/?target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=dockerized_snow_norlab_mppi_plan2.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_norlab_mppi_plan2.drawio">
<img src="drawio/dockerized_snow_norlab_mppi_plan2.svg">
</a>
</p>
<sub>
(&nbsp; View diagram  
<a href="https://viewer.diagrams.net/?target=blank&highlight=0000ff&edit=_blank&layers=1&nav=1&title=dockerized_snow_norlab_mppi_plan2.drawio#Uhttps%3A%2F%2Fraw.githubusercontent.com%2FRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_norlab_mppi_plan2.drawio">
fullscreen
</a>
&nbsp; • &nbsp;
<a href="https://app.diagrams.net/?mode=github#HRedLeader962%2FDockerized-SNOW%2Fmaster%2Fdrawio%2Fdockerized_snow_norlab_mppi_plan2.drawio" target="_blank" rel="noopener noreferrer">edit
</a>
diagram &nbsp;)
</sub>
</div>
<br>


---
### Instruction:
- [**★ | Setup PyCharm for local development using a Docker Python interpreter**](how_to_readme/README_setup_pycharm_python_docker_interpreter.md)
- [**Using the _nvidia-docker_ image on a _Jetson_ device (`arm64-l4t`)**](how_to_readme/README_Jetson_builded.md)
- [**Using the _nvidia-docker_ image on a `x86` host**](how_to_readme/README_x86_architecture.md)
- [**Building the `arm64-l4t` nvidia-docker image on a `x86` host using _qemu_ virtualization**](how_to_readme/README_cross_compiler.md)

### How-to
- [**How-to _nvidia-docker_ manualy (a quick start)**](how_to_readme/README_docker_manualy_quickstart.md)
- [**Test AutoRally Configuration (**Revised instruction**)**](https://github.com/RedLeader962/SNOW_AutoRally#test-autorally-configuration-in-gazebo-revised-instruction)
- [How-to push image localy builded image to docker hub from command line](how_to_readme/README_push_to_dockerhub.md)


### Images:
To pull the latest image from _docker hub_, execute the following in terminal: 
```shell
sudo docker pull <container name>:<tag>
```
with `<container name>`= _theImageName_ and `<tag>`= _theHostArchitecture_  

<!-- 
Latest images for _Jetson_: 
  - `norlabsnow/gt-autorally/deploy:arm64-l4t`
  - `norlabsnow/gt-autorally/develop:arm64-l4t`
  - `norlabsnow/gt-autorally/dependencies:arm64-l4t`

Latest images for _x86_64_ workstion: 
  - `norlabsnow/gt-autorally/deploy:x86`
  - `norlabsnow/gt-autorally/develop:x86`
  - `norlabsnow/gt-autorally/dependencies:x86`

Base image: 
  - `nvcr.io/nvidia/l4t-base:r32.5.0`
  - `nvcr.io/nvidia/cudagl:11.3.1-devel-ubuntu18.04`
-->

---
### References:

<details>
<summary>nvidia-docker Documentation:</summary>

- [nvidia-docker: Build and run Docker containers leveraging NVIDIA GPUs](https://github.com/NVIDIA/nvidia-docker) 
  - [NVIDIA Container Runtime on _Jetson_](https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson)
  - [Driver containers](https://github.com/NVIDIA/nvidia-docker/wiki/Driver-containers)
- [NVIDIA Cloud Native Technologies](https://docs.nvidia.com/datacenter/cloud-native/#)
- Base image for _jetson_:
  - https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
  - https://developer.nvidia.com/embedded/jetson-cloud-native
- Base image with _CUDA_ and _OpenGL_ support:
  - https://hub.docker.com/r/nvidia/cudagl/
  - https://github.com/NVIDIA/nvidia-docker/wiki/CUDA
  - https://ngc.nvidia.com/catalog/containers/nvidia:cudagl

</details>

<details>
<summary>Docker Documentation:</summary>

- [Use the Docker command line | Docker Documentation](https://docs.docker.com/engine/reference/commandline/cli/)
- [Dockerfile reference | Docker Documentation](https://docs.docker.com/engine/reference/builder/)

</details>

---




 

