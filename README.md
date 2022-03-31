# Cocobots

This repository was developed on an Ubuntu 20.04, with Nvidia GPU (CUDA 11.6), Docker (4.5). It is supposed to work for Ubuntu 20.04 or Windows WSL, but it hasn't been tested on MacOS.

It contains the ROS2 Foxy with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a GPU accelerated Ubuntu 20.04 Docker environment. 

Webots works with Linux (it  doesn't run on Ubuntu versions earlier than 18.04), Windows (8 and 10) and Mac (10.15 "Catalina" and 10.14 "Mojave"). It needs an NVIDIA or AMD OpenGL.

The repository was based on:
[RoboticaUtnFrba Dockerfile](https://github.com/RoboticaUtnFrba/create2_docker) and
[NIURoverTeam Dockerfile](https://github.com/NIURoverTeam/Dockerfiles/tree/master/webots_ros2_foxy)

## Prerequisites

* An Nvidia GPU (?)
* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* for [docker-nvidia2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) either follow the official guidelines from the website, or:
```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```
* (If installed on Windows WSL, then you need an Xserver as well, eg VcXsrvs)

## Installation

1. Create a workspace folder (cocobots_ws) in the home directory
2. Git clone [https://github.com/alexandrosnic/cocobots](https://github.com/alexandrosnic/cocobots) in the root of your workspaces folder
3. Build the docker:
```
docker-compose up --build webots_simulation
```
4. Run the docker
```
docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cocobots_ws_webots_simulation
```
5. Et voila, webots and ros2 installed