<div align="center">
<a href="https://norlab.ulaval.ca">
<img src="./images/norlab_logo_acronym_dark.png" width="200">
</a>
<br>
</div>

# _/// Dockerized SNOW //////////////////////////////////////////_
Containerized workflow for the NorLab _SNOW-AutoRally_ project leveraging [_nvidia-docker_](https://github.com/NVIDIA/nvidia-docker) technology

<div align="left">
<p>
<sup>
<b>Project related link: </b> &nbsp; 
<a href="https://redleader.myjetbrains.com/youtrack/dashboard?id=bce3112d-bda1-425c-8628-802a047be4d3">NLSAR - NorLab SNOW AutoRally</a>
(Youtrack) &nbsp; • &nbsp;  
<a href="https://hub.docker.com/u/norlabsnow">norlabsnow</a>
(Docker Hub) &nbsp; • &nbsp; 
<a href="https://github.com/RedLeader962/autorally">SNOW-AutoRally</a>
(GitHub)
&nbsp;
</sup>
</p>
</div>
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

---
## Usage

#### Workflow option:

- Instruction for [**building the _docker_ image from a _Jetson_ device**](how_to_readme/README_Jetson_builded.md)
- Instruction for [**building the _docker_ image from an x86 host using _qemu_**](how_to_readme/README_cross_compiler.md)
- Instruction for pulling the pre-builded _docker_ image from _DockerHub_ (TODO)

#### How-to
- [**How-to _nvidia-docker_ manualy (a quick start)**](how_to_readme/README_docker_manualy_quickstart.md)
- [**Test AutoRally Configuration (**Revised instruction**)**](https://github.com/RedLeader962/SNOW-AutoRally#test-autorally-configuration-in-gazebo-revised-instruction)
- [How-to push image localy builded image to docker hub from command line](how_to_readme/README_push_to_dockerhub.md)


#### Images:
- Base image: `nvcr.io/nvidia/l4t-base:r32.5.0`
- Latest images on _docker hub_: 
   - `docker pull norlabsnow/snow-autorally-deploy`
   - `docker pull norlabsnow/snow-autorally-develop`
   - `docker pull norlabsnow/ros-melodic-snow-autorally-dependencies`
  
---

<details>
<summary>NVIDIA-docker references:</summary>

- [nvidia-docker: Build and run Docker containers leveraging NVIDIA GPUs](https://github.com/NVIDIA/nvidia-docker) 
  - [NVIDIA Container Runtime on _Jetson_](https://github.com/NVIDIA/nvidia-docker/wiki/NVIDIA-Container-Runtime-on-Jetson)
  - [Driver containers](https://github.com/NVIDIA/nvidia-docker/wiki/Driver-containers)
- [NVIDIA Cloud Native Technologies](https://docs.nvidia.com/datacenter/cloud-native/#)

</details>

---




 

