# Cocobots Repository

This repository was developed on an Ubuntu 20.04, with Nvidia GPU (CUDA 11.6). It is supposed to work for Ubuntu 20.04 or Windows WSL, but it hasn't been tested on MacOS. 
To install WSL2 in Windows, follow the above installation instructions:

## Windows WSL2 Installation

If you work on Windows, follow these guidelines to install WSL2:

<details>
  <summary>Click to expand!</summary>
  
## Install WSL2
1. Open cmd with administrator rights
2. Install wsl and then restart:
```
wsl --install
```
3. Follow the instructions to install Ubuntu 20.04

## Install Docker Desktop for Windows
1. [Docker Desktop](https://docs.docker.com/desktop/windows/install/)

## [Optionally] Configure WSL2 through Visual Studio Code
1. Download and install [Visual Studio Code (VSC)](https://code.visualstudio.com/download)
2. Open VSC and download the "Remote - WSL" and "Remote - Containers" extensions from the Extensions tab on the left hand side.
3. From the green icon on the left-down corner, choose "New WSL Window"
4. You can now interact with the Ubuntu documents and terminal

</details>


This repository contains three options. Follow either of the following instructions:


## 1. ROS2 Foxy

A ROS2 Foxy (docker) setup with all the required dependencies for this project, in a non-GPU accelerated Ubuntu 20.04. 

<details>
  <summary>Click to expand!</summary>
  
## I. Only ROS2 docker

### Prerequisites

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [rocker](https://github.com/osrf/rocker)
* (If installed on Windows WSL, then you will probably need an Xserver as well, eg [VcXsrvs](https://sourceforge.net/projects/vcxsrv/))

Installation instructions below

### Installation for ROS2

1. Choose the folder in which to save the repository (eg in a "cocobots" folder
2. Open a terminal and enter that directory (cocobots).
3. Git clone the [cocobots repository](https://github.com/alexandrosnic/cocobots_docker):
```
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/084_cocobots_docker.git
```
It will prompt you to enter the bitbucket password
4. Install the dependencies
```
sudo ./084_cocobots_docker/ros2/setup_project.sh
```
5. Build the docker (May need "sudo"):
```
docker-compose -f 084_cocobots_docker/ros2/docker-compose.yaml build
```
6. Run the docker (May need "sudo"):
```
rocker --devices /dev/dri/card --x11 ros2_custom
```
7. Make sure that ROS2 is installed and run properly, by typing any ROS2 command like:
```
ros2 --help
```
To run webots:
```
webots --no-sandbox
```

</details>

## 2. ROS2 Foxy - Webots (With Nvidia)

A ROS2 Foxy (docker) setup with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a GPU accelerated Ubuntu 20.04 environment. 

<details>
  <summary>Click to expand!</summary>
  
## II. ROS2 + Webots on NVidia docker

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
3. Git clone the [cocobots repository](https://github.com/alexandrosnic/cocobots_docker) in the root of your workspace folder (cocobots_ws):
```
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/084_cocobots_docker.git
```
4. Install the dependencies
```
sudo ./084_cocobots_docker/webots_ros2/setup_project.sh
```
5. Build the docker:
```
docker-compose -f 084_cocobots_docker/webots_ros2/docker-compose.yaml build
```
6. Run the docker (May need "sudo"):
```
docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ros2_webots
```
7. By now, you already set up and can interact with ROS2 and Webots. If you also want to clone the cocobots repositories, then follow the rest instructions:
```
cd cocobots_ws/src/
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/085_cocobots_environment.git
```

</details>

## 3. ROS2 Foxy - Webots (Without Nvidia)

A ROS2 Foxy (docker) setup with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a non-GPU Ubuntu 20.04  environment. 

<details>
  <summary>Click to expand!</summary>
  
## III. ROS2 + Webots without NVidia docker

### Prerequisites

* [docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [rocker](https://github.com/osrf/rocker)
* (If installed on Windows WSL, then you will probably need an Xserver as well, eg [VcXsrvs](https://sourceforge.net/projects/vcxsrv/))

Installation instructions below

### Installation for ROS2

1. Create a folder in the home directory and name it "cocobots_ws". This will be your workspace directory
2. Open a terminal and enter the cocobots_ws directory.
3. Git clone the [cocobots repository](https://github.com/alexandrosnic/cocobots_docker) in the root of your workspace folder (cocobots_ws):
```
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/084_cocobots_docker.git
```
4. Install the dependencies
```
sudo ./084_cocobots_docker/ros2/setup_project.sh
```
5. Pull the docker:
```
pull docker alexnic/ros2_webots_nonvidia:version1
```
6. Run the docker (May need "sudo"):
```
rocker --devices /dev/dri/card --x11 ros2_webots_nonvidia:version1
```
7. Make sure that ROS2 is installed and run properly, by typing any ROS2 command like:
```
ros2 --help
```
8. To run webots:
```
webots --no-sandbox
```

</details>

Webots works with Linux (it  doesn't run on Ubuntu versions earlier than 18.04), Windows 8 and 10 (and 11?) and Mac (10.15 "Catalina" and 10.14 "Mojave"). It needs an NVIDIA or AMD OpenGL.

The repository was based on:
[RoboticaUtnFrba Dockerfile](https://github.com/RoboticaUtnFrba/create2_docker) and
[NIURoverTeam Dockerfile](https://github.com/NIURoverTeam/Dockerfiles/tree/master/webots_ros2_foxy)








<<<<<<< HEAD

<!-- TODO

Run Dockerfile2 -->
=======
>>>>>>> d4e767def14af21e368e392bc04eb7c89e1a92e3
