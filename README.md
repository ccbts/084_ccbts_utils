# Cocobots Repository

This repository consists of dockerfiles to installation instructions for ROS2 and Webots. 

## System requirements

It is highly recommended to run Webots on a supported [GPU](https://www.cyberbotics.com/doc/guide/system-requirements), however, it can also work without a graphics card.
For this, make sure you have, or update to the latest GPU drives from [here](https://www.nvidia.com/download/index.aspx).

The following operating systems are supported:
* Linux (preferred)
* Windows through Windows Subsystem for Linux (WSL). To install WSL, follow these [guidelines](https://docs.microsoft.com/en-us/windows/wsl/install). A current limitation is that [nvidia-docker2](https://docs.nvidia.com/cuda/wsl-user-guide/index.html#known-limitations-for-linux-cuda-apps) for WSL is still under development and for that reason ROS2 and Webots can only be installed through docker without GPU support. For a GPU system a bash script will be provided instead of docker. 
* Mac (currently doesn't work properly)

<!-- ## Dependencies
* For WSL, install [Docker](https://docs.docker.com/get-docker/) 

## [Optionally] Configure WSL2 through Visual Studio Code
1. Download and install [Visual Studio Code (VSC)](https://code.visualstudio.com/download)
2. Open VSC and download the "Remote - WSL" and "Remote - Containers" extensions from the Extensions tab on the left hand side.
3. From the green icon on the left-down corner, choose "New WSL Window"
4. You can now interact with the Ubuntu documents and terminal -->

<!-- </details> -->

## Installation guidelines

This repository contains three options. 
* **I. ROS2 Foxy - Webots on Ubuntu (with or without GPU)**
* **II. ROS2 Foxy - Webots on WSL (with or without GPU)**
* **III. ROS2 Foxy - Webots on Mac**

Follow the one suitable for your System:


<!-- </details> -->

## I. ROS2 Foxy - Webots on Ubuntu (with or without GPU)

A ROS2 Foxy (docker) setup with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a Ubuntu 20.04 environment. The setup may work with or without GPU (follow the corresponding guidelines). 

<!-- <details>
  <summary>Click to expand!</summary>
  
## II. ROS2 + Webots on NVidia docker -->

### Prerequisites

These tools will be installed in step 3. Ignore it if they are already installed 

* [docker](https://docs.docker.com/engine/install/ubuntu/) (and make sure is running)
* [docker-compose](https://docs.docker.com/compose/install/)
* [docker-nvidia2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

### Installation

1. Open a terminal, navigate in the 'home' directory, and create a folder "cocobots_ws". This will be your workspace directory
```
mkdir -p cocobots_ws
chown -R $USER:$USER home/$USER/cocobots_ws
cd cocobots_ws
```
2. Git clone the [cocobots repository](https://bitbucket.org/dsgbielefeld/084_cocobots_docker/src/master/) in the root of your workspace folder (cocobots_ws):
```
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/084_cocobots_docker.git
```
3. Install the dependencies
* If there is GPU on the system:
  ```
  sudo ./084_cocobots_docker/webots_ros2/setup_project_ubuntu_gpu.sh
  ```
* Otherwise:
  ```
  sudo ./084_cocobots_docker/webots_ros2/setup_project_ubuntu_nogpu.sh
  ```
4. Build the docker:
  ```
  docker-compose -f 084_cocobots_docker/webots_ros2/compose-ubuntu.yaml build
  ```
5. Run the docker (May need "sudo"):
* If there is GPU on the system:
  ```
  docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ros2_webots
  ```
* Otherwise:
  ```
  rocker --devices /dev/dri/card --x11 ros2_webots
  ```
6. By now, you already set up and can interact with ROS2 and Webots in docker. If you also want to clone the cocobots repositories, then follow the rest instructions:
```
cd cocobots_ws/src/
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/085_cocobots_environment.git
cd ..
colcon build
source install/local_setup.bash
```
7. To launch the Cocobots world:
```
ros2 launch cocobots_simu cocobots_launch.py
```



## II. ROS2 Foxy - Webots on WSL (with or without GPU)

A ROS2 Foxy setup with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a WSL environment. 
The setup may work with or without GPU (follow the corresponding guidelines). 

<!-- <details>
  <summary>Click to expand!</summary>
  
## II. ROS2 + Webots on NVidia docker -->

### Prerequisites

<!-- * [docker](https://docs.docker.com/engine/install/ubuntu/) (and make sure is running)
* [docker-compose](https://docs.docker.com/compose/install/)
* For WSL: [docker-nvidia2](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) -->
The nvidia-docker2 for WSL currently does not support OpenGL applications like Webots, thus, it cannot be installed through a docker. For that reason, that bash script of step 3 will be used instead, or ignore that step if already these tools are installed:

* [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* [Webots](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt)
* Xserver running, eg [VcXsrvs](https://sourceforge.net/projects/vcxsrv/). To configure it, follow this [tutorial](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242)


### Installation

1. Open a terminal, navigate in the 'home' directory, and create a folder "cocobots_ws". This will be your workspace directory

```
mkdir -p cocobots_ws/src
chown -R $USER:$USER home/$USER/cocobots_ws
cd cocobots_ws
```
2. Git clone the [cocobots repository](https://bitbucket.org/dsgbielefeld/084_cocobots_docker/src/master/) in the root of your workspace folder (cocobots_ws):
```
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/084_cocobots_docker.git
```
3. Install the dependencies
```
sudo ./084_cocobots_docker/webots_ros2/setup_project_wsl.sh
```
4. Create the workspace
```
colcon build
```
5. Source the workspace (and do it every time you open a new terminal):
```
source install/setup.bash
```
6. By now, you already set up and can interact with ROS2 and Webots. If you also want to clone the cocobots repositories, then follow the rest instructions:
```
cd src/
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/085_cocobots_environment.git
cd ..
colcon build
source install/local_setup.bash
```
7. Configure the Xserver following this [tutorial](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242) (First option: VcXsrv Windows X Server). To avoid having to export the DISPLAY every time that WSL is launched, you can include the command at the end of the /etc/bash.bashrc file:
```
echo export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0" >> /home/$USER/.bashrc
```
8. To launch the Cocobots world:
```
ros2 launch cocobots_simu cocobots_launch.py
```


<!-- </details> -->

## III. ROS2 Foxy - Webots on Mac

A ROS2 Foxy (docker) setup with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a non-GPU Mac environment. 

<!-- <details>
  <summary>Click to expand!</summary>
  
## III. ROS2 + Webots without NVidia docker -->

### Prerequisites

* [docker](https://docs.docker.com/engine/install/ubuntu/) (and make sure is running)
* [docker-compose](https://docs.docker.com/compose/install/)
* [rocker](https://github.com/osrf/rocker)
* Xserver, eg [XQuartz](https://www.xquartz.org/). To install and configure it, follow this [tutorial](https://affolter.net/running-a-docker-container-with-gui-on-mac-os/)

### Installation

1. Open a terminal, navigate in the 'home' directory, and create a folder "cocobots_ws". This will be your workspace directory

```
mkdir -p cocobots_ws
chown -R $USER:$USER home/$USER/cocobots_ws
cd cocobots_ws
```
2. Git clone the [cocobots repository](https://bitbucket.org/dsgbielefeld/084_cocobots_docker/src/master/) in the root of your workspace folder (cocobots_ws):
```
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/084_cocobots_docker.git
```
3. Install the dependencies
```
sudo ./084_cocobots_docker/webots_ros2/setup_project_mac.sh
```
4. Build the docker:
  ```
  docker-compose -f 084_cocobots_docker/webots_ros2/compose-mac.yaml build
  ```
5. Run the docker (May need "sudo"):
  ```
  rocker --devices /dev/dri/card --x11 ros2_webots
  ```
6. By now, you already set up and can interact with ROS2 and Webots in docker. If you also want to clone the cocobots repositories, then follow the rest instructions:
```
cd cocobots_ws/src/
git clone https://alexandrosnic@bitbucket.org/dsgbielefeld/085_cocobots_environment.git
cd ..
colcon build
source install/local_setup.bash
```
7. To launch the Cocobots world:
```
ros2 launch cocobots_simu cocobots_launch.py
```

<!-- </details> -->






<!-- 
TODO
1. Write what it is to be done in Windows, what in WSL
2. Clarify what is for Windows (WSL), what for ubuntu 
3. For presentation, add the structure, workspace, package, favourite commands
Nodes: A node is an executable that uses ROS to communicate with other nodes.
Messages: ROS data type used when subscribing or publishing to a topic.
Topics: Nodes can publish messages to a topic, as well as subscribe to a topic to receive messages.
Packages: A way to organize software in ROS. A package can contain nodes, message definitions, libraries, datasets etc.

4. Check it out https://www.logic2020.com/insight/tactical/wsl-docker-gpu-enabled-nvidia
Try Run the nvidia/cudagl and then install webots inside locally
Or use the previous docker and change the paths?
-->