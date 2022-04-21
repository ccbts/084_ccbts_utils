# Cocobots Repository

This repository was developed on an Ubuntu 20.04, with Nvidia GPU (CUDA 11.6), Docker (4.5). It is supposed to work for Ubuntu 20.04 or Windows WSL, but it hasn't been tested on MacOS.

It contains two options:
1. the ROS2 Foxy with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a GPU accelerated Ubuntu 20.04 Docker environment. 
2. Only the ROS2 Foxy with all the required dependencies for this project

Webots works with Linux (it  doesn't run on Ubuntu versions earlier than 18.04), Windows (8 and 10) and Mac (10.15 "Catalina" and 10.14 "Mojave"). It needs an NVIDIA or AMD OpenGL.

The repository was based on:
[RoboticaUtnFrba Dockerfile](https://github.com/RoboticaUtnFrba/create2_docker) and
[NIURoverTeam Dockerfile](https://github.com/NIURoverTeam/Dockerfiles/tree/master/webots_ros2_foxy)

Follow either of the following instructions:

## a. Only ROS2 docker

### Prerequisites

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [rocker](https://github.com/osrf/rocker)
* (If installed on Windows WSL, then you will probably need an Xserver as well, eg [VcXsrvs](https://sourceforge.net/projects/vcxsrv/))

Installation instructions below

### Installation for ROS2

1. Create a folder in the home directory and name it "cocobots_ws". This will be your workspace directory
2. Open a terminal and enter the cocobots_ws directory.
3. Git clone the [cocobots repository](https://github.com/alexandrosnic/cocobots_docker) in the root of your workspace folder:
```
git clone https://github.com/alexandrosnic/cocobots_docker
```
4. Install the dependencies
```
sudo ./docker/ros2/setup_project.sh
```
5. Build the docker (May need "sudo"):
```
docker-compose -f docker/ros2/docker-compose.yaml build
```
6. Run the docker (May need "sudo"):
```
rocker --devices /dev/dri/card --x11 ros2_ros2:latest
```
7. Make sure that ROS2 is installed and run properly, by typing any ROS2 command like:
```
ros2 --help
```

## b. ROS2 + Webots on NVidia docker

### Prerequisites

* An Nvidia GPU
* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [docker-nvidia2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) 
* (If installed on Windows WSL, then you need an Xserver as well, eg VcXsrvs)

Installation instructions below

### Installation for ROS2 - Webots on Nvidia

1. Create a folder in the home directory and name it "cocobots_ws". This will be your workspace directory
2. Open a terminal and enter the cocobots_ws directory.
3. Git clone the [cocobots repository](https://github.com/alexandrosnic/cocobots_docker) in the root of your workspace folder:
```
git clone https://github.com/alexandrosnic/cocobots_docker
```
4. Install the dependencies
```
sudo ./docker/ros2/setup_project.sh
```
5. Build the docker:
```
docker-compose -f docker/webots_ros2/docker-compose.yaml build
```
6. Run the docker (May need "sudo"):
```
docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cocobots_ws_webots_simulation
```
7. Make sure that ROS2 is installed and run properly, by typing any ROS2 command like:
```
ros2 --help
```