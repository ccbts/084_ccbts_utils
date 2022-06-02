# Cocobots Utilities Repository

This repository consists of dockerfiles and installation guidelines for ROS2 and Webots. 

## System requirements

It is highly recommended to run Webots on a supported [GPU](https://www.cyberbotics.com/doc/guide/system-requirements), however, it can also work without a graphics card.
For this, make sure you have, or update to the latest GPU drives from [here](https://www.nvidia.com/download/index.aspx), and that your system has at least **OpenGL 3.3**.

The following operating systems are supported:
* Linux (preferred) 
* Windows through Windows Subsystem for Linux (WSL). To install WSL, follow these [guidelines](https://docs.microsoft.com/en-us/windows/wsl/install). A current limitation is that [nvidia-docker2](https://docs.nvidia.com/cuda/wsl-user-guide/index.html#known-limitations-for-linux-cuda-apps) for WSL is still under development and for that reason ROS2 and Webots can only be installed through docker without GPU support. For a GPU system a bash script will be provided instead of docker. 
* Mac (it doesn't work on some Macs due to limited support of OpenGL)

<!-- ## Dependencies
* For WSL, install [Docker](https://docs.docker.com/get-docker/) 

## [Optionally] Configure WSL2 through Visual Studio Code
1. Download and install [Visual Studio Code (VSC)](https://code.visualstudio.com/download)
2. Open VSC and download the "Remote - WSL" and "Remote - Containers" extensions from the Extensions tab on the left hand side.
3. From the green icon on the left-down corner, choose "New WSL Window"
4. You can now interact with the Ubuntu documents and terminal -->

<!-- </details> -->

## Installation guidelines

This repository contains two options. 
* **I. ROS2 Foxy - Webots on Ubuntu or Mac (with or without GPU)**
* **II. ROS2 Foxy - Webots on WSL (with or without GPU)**

Follow the one suitable for your System:


<!-- </details> -->

## I. ROS2 Foxy - Webots on Ubuntu or Mac (with or without GPU)

A ROS2 Foxy (docker) setup with all the required dependencies for this project (and it will be updated on the go), coupled with Webots simulator R2022a in a Ubuntu 20.04 environment. The setup may work with or without GPU (follow the corresponding guidelines). 

<!-- <details>
  <summary>Click to expand!</summary>
  
## II. ROS2 + Webots on NVidia docker -->

### Prerequisites

These tools will be installed in step 3. Ignore it if they are already installed 

* [docker](https://docs.docker.com/engine/install/ubuntu/) (and make sure is running)
* [docker-compose](https://docs.docker.com/compose/install/)
* [docker-nvidia2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) if on a **GPU-accelerated system**
* Xserver
  * For Mac: eg [XQuartz](https://www.xquartz.org/). To install and configure it, follow this [tutorial](https://affolter.net/running-a-docker-container-with-gui-on-mac-os/)
  * For Ubuntu:
  ```
  xhost +local:*
  Or xhost +local:docker (for docker)
  ```


### Installation

1. Open a terminal, navigate in the 'home' directory, and create a folder to store the dockerfile, eg "cocobots". 
```
mkdir -p cocobots
chown -R $USER:$USER /home/$USER/cocobots
cd cocobots
```
2. Git clone the [cocobots repository](https://github.com/ccbts/084_ccbts_utils) in the root of your workspace folder (cocobots):
```
git clone https://github.com/ccbts/084_ccbts_utils.git
```
3. Install the dependencies
* For Ubuntu with GPU:
  ```
  chmod +x ./084_ccbts_utils/webots_ros2/setup_project_ubuntu_gpu.sh;
  sudo bash ./084_ccbts_utils/webots_ros2/setup_project_ubuntu_gpu.sh
  ```
* For Ubuntu without GPU:
  ```
  chmod +x ./084_ccbts_utils/webots_ros2/setup_project_ubuntu_nogpu.sh
  sudo bash ./084_ccbts_utils/webots_ros2/setup_project_ubuntu_nogpu.sh
  ```
* For Mac:
  ```
  chmod +x ./084_ccbts_utils/webots_ros2/setup_project_mac.sh;
  /bin/bash ./084_ccbts_utils/webots_ros2/setup_project_mac.sh
  ```
4. Add the user to the sudo group:
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
5. Build the docker (May need "sudo"):
* With GPU:
  ```
  docker-compose -f 084_ccbts_utils/webots_ros2/compose-ubuntu.yaml build
  ```
* Without GPU:
  ```
  docker-compose -f 084_ccbts_utils/webots_ros2/compose-ubuntu-nogpu.yaml build
  ```
6. Run the docker (May need "sudo"). Remember you may need to run **xhost +local:docker** every time you run this command:
* If there is GPU on the system:
  ```
  docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ros2_webots
  ```
* Otherwise:
  ```
  docker run --rm -it --user=root -e DISPLAY -e TERM   -e QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix   -v /etc/localtime:/etc/localtime:ro  ros2_webots
  ```
7. By now, you already set up and can interact with ROS2 and Webots in docker. If you also want to clone the cocobots repositories, then follow the rest of the instructions inside the container. Remember to source everytime you open a new terminal:
```
cd cocobots_ws/src/;
git clone git@github.com:ccbts/085_ccbts_env.git;
cd ..;
colcon build;
source install/local_setup.bash;
export PYTHONPATH=${PYTHONPATH}:/home/${USER}/cocobots_ws/install/ccbts_environment/lib/python3.8/site-packages
```
8. To launch the simulation, run this command, otherwise, to control the UR from ROS2, move to step 8 to install the drivers:
```
ros2 launch ccbts_environment cocobots_launch.py
```
9. To manipulate the Universal Robot arm, install the UR driver. In the cocobots_ws directory run:
```
git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
10. Install the moveit package and dependencies. In the cocobots_ws directory run:
```
sudo apt-get install ros-foxy-moveit ros-foxy-warehouse-ros-mongo
``` 
<!-- 10. Create the connection between the PC and the robot. Connect the PC with the ethernet cable of the UR. Then open Network Settings and create a new Wired (Ethernet) connection with these settings. You may want to name this new connection UR or something similar:
```
IPv4
Manual
Address: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1
``` -->
11. To connect to the UR robot, you have to be connected to the same network that the UR is connected to, and identify its IP address (alternatively, you can also establish an ethernet connection). The run the launch file that starts the robot driver and the controllers:
```
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.4 launch_rviz:=true reverse_ip:=[your ip address. Find it with 'ifconfig'] limited:=true
```
12. Send some goal to the Joint Trajectory Controller by using a demo node from ros2_control_demos package by starting the following command in another terminal:
```
ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
```
13. To test the driver with the example MoveIt-setup, first start the controllers with the command at [11] then start MoveIt.
```
# Open a new terminal and see running containers
docker ps
# Identify the container ID of ros2_webots, and connect to it
docker exec -it [container id] bash
ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.0.4 launch_rviz:=true reverse_ip:=[your ip address. Find it with 'ifconfig']
```
14. To save the current state of the docker and be able to use it again
```
# Open a new terminal and see running containers
docker ps
# Identify the container ID of ros2_webots, and save its state
docker commit [container id] [container new name, eg ros2_webots_ur]
```

### Tips
* To be able to edit code from within the container, run the container from a terminal:
```
docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ros2_webots
```
Open VSCode, download the 'Remote - Containers' extension, click on the green bottom-left button, select 'Attach to running container' and choose the running container



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
The nvidia-docker2 for WSL currently does not support OpenGL applications like Webots, thus, it cannot be installed through a docker. For that reason, a bash script (step 3) will be used instead, or ignore that step if already these tools are installed:

* [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* [Webots](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-windows). Webots cannot run as GPU accelerated in WSL, so it is recommended to install Webots natively in Windows, following the official installation instructions, and then create a symlink (shortcut) to the executable file, as is described in step 5.
* Xserver running, eg [VcXsrvs](https://sourceforge.net/projects/vcxsrv/). To configure it, follow this [tutorial](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242). 
* WSL2 with Ubuntu 20.04 installed


### Installation

1. After you install Webots from the [official site](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-windows) open a WSL terminal and create a symlink (shortcut) of the Webots executable in a WSL directory, here will be /usr/local/bin/webots:
```
sudo ln -s /mnt/c/Program\ Files/Webots/msys64/mingw64/bin/webotsw.exe /usr/local/bin/webots
```
If Webots is installed in another directory other than Program Files, then change accordingly the above command. Also, you can create the symlink in another directory, but then make sure to include that directory in the PATH variable.
2. Navigate in the 'home' directory, and create a folder "cocobots_ws". This will be your workspace directory

```
mkdir -p cocobots_ws/src
chown -R $USER:$USER /home/$USER/cocobots_ws
cd cocobots_ws 
```
3. Git clone the [cocobots repository](https://github.com/ccbts/084_ccbts_utils) in the root of your workspace folder (cocobots_ws):
```
git clone https://github.com/ccbts/084_ccbts_utils.git
```
4. Install the dependencies
```
chmod +x ./084_ccbts_utils/webots_ros2/setup_project_wsl.sh;
sudo bash ./084_ccbts_utils/webots_ros2/setup_project_wsl.sh
```
5. Create the workspace in the cocobots_ws directory
```
colcon build
```
6. Source the workspace (and do it every time you open a new terminal):
```
source install/setup.bash
```
7. By now, you already set up and can interact with ROS2 and Webots. If you also want to clone the cocobots repositories, then follow the rest of the instructions. Remember to source everytime you open a new terminal:
```
cd src/;
git clone git@github.com:ccbts/085_ccbts_env.git;
cd ..;
colcon build;
source install/local_setup.bash;
export PYTHONPATH=${PYTHONPATH}:/home/${USER}/cocobots_ws/install/ccbts_environment/lib/python3.8/site-packages
```
8. Configure the Xserver. For VcXsrv (XLaunch), the configuration is Multiple Windows > Start no client > Check everything apart from "Native opengl" > Finish. To avoid having to export the DISPLAY every time that WSL is launched, you can include the command at the end of the /etc/bash.bashrc file:
```
echo "export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0" >> /home/$USER/.bashrc
```
If webots still don't open even after the installation, then run 
```
echo "export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0.0" >> /home/$USER/.bashrc
```
9. To launch the simulation, run this command, otherwise, to control the UR from ROS2, move to step 10 to install the drivers:
```
ros2 launch ccbts_environment cocobots_launch.py
```
10. To manipulate the Universal Robot arm, install the UR driver. In the cocobots_ws directory run:
```
git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
11. Install the moveit package and dependencies. In the cocobots_ws directory run:
```
sudo apt-get install ros-foxy-moveit ros-foxy-warehouse-ros-mongo
``` 
<!-- 12. Create the connection between the PC and the robot. Connect the PC with the ethernet cable of the UR. Then open Network Settings and create a new Wired (Ethernet) connection with these settings. You may want to name this new connection UR or something similar:
```
IPv4
Manual
Address: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1
``` -->
12. To connect to the UR robot, you have to be connected to the same network that the UR is connected to, and identify its IP address (alternatively, you can also establish an ethernet connection). The run the launch file that starts the robot driver and the controllers:
```
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.4 launch_rviz:=true reverse_ip:=[your ip address. Find it with 'ipconfig'] limited:=true
```
13. Send some goal to the Joint Trajectory Controller by using a demo node from ros2_control_demos package by starting the following command in another terminal:
```
ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
```
14. To test the driver with the example MoveIt-setup, first start the controllers with the command at [11] then start MoveIt.
```
# Open a new terminal and see running containers
docker ps
# Identify the container ID of ros2_webots, and connect to it
docker exec -it [container id] bash
ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.0.4 launch_rviz:=true reverse_ip:=[your ip address. Find it with 'ipconfig']
```
15. To save the current state of the docker and be able to use it again
```
# Open a new terminal and see running containers
docker ps
# Identify the container ID of ros2_webots, and save its state
docker commit [container id] [container new name, eg ros2_webots_ur]
```

## Webots Tips
* You have to modify the ur3e_environment/ur3e_driver.py to control the UR3e as you want to. An example can be found [here](https://github.com/cyberbotics/webots_ros2/wiki/Example-Universal-Robots) with this [controller](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_universal_robot/webots_ros2_universal_robot/ur5e_controller.py)


## Troubleshooting
1. Error: package not found
  * Make sure to run 'source install/setup.bash'
2. Webots is not opening
  * Make sure DISPLAY had been set correctly by "echo $DISPLAY", and that you have an X server running
  * If it still doesn't work, run:
  ```
  export LIBGL_ALWAYS_INDIRECT=0
  ```
  * Segmentation fault, run this (which will force Webots to run on CPU):
  ```
  export LIBGL_ALWAYS_SOFTWARE=0
  ```
3. Could not load the Qt platform plugin “xcb” … even though it was found
  * Xserver error. Make sure you ran all the steps of the tutorial ([Windows](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242), [Mac](https://affolter.net/running-a-docker-container-with-gui-on-mac-os/)) for setting the Xserver, and that the DISPLAY variable has been set correctly
  * If running on MAC, try this before going through the tutorial
  ```
  socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
  ```
  * If running on WSL, try exporting this display:
  ```
  export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0.0
  ```
  or this:
  ```
  export DISPLAY=$(route.exe print | grep 0.0.0.0 | head -1 | awk '{print $4}'):0.0
  ```
  * If running on Ubuntu:
  ```
  xhost +local:*
  ```
4. If the GUI tools don’t work (like rviz):
* Try using the environmental variable LIBGL_ALWAYS_INDIRECT:
```
export LIBGL_ALWAYS_INDIRECT=0
```
* ImportError: libQt5Core.so.5: cannot open shared object file: No such file or directory:
```
sudo strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
```



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

sudo apt install ros-foxy-moveit
in cocobots_ws:

git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/ccbts_ur_driver
vcs import src --skip-existing --input src/ccbts_ur_driver/Universal_Robots_ROS2_Driver.repos
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

vcs import src --skip-existing --input src/ccbts_ur_driver/MoveIt_Support.repos
vcs import src --skip-existing --input src/moveit2/moveit2.repos
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

-->