#!/bin/bash
#
# Usage: ./setup_project.sh (it requires sudo!)

# --------------------------- #
## UR Driver
# --------------------------- #

# Install ros2 control, moveit and other dependencies
sudo apt-get update && apt-get install -y \
		ros-foxy-ros2-control \
        ros-foxy-ros2-controllers \
        ros-foxy-moveit \
        ros-foxy-warehouse-ros-mongo \
        ros-foxy-tf-transformations \
        ros-foxy-srdfdom 

sudo pip3 install transforms3d

export UR_DRIVER_WS=~/ur_driver_ws
mkdir -p $UR_DRIVER_WS/src
cd $UR_DRIVER_WS
git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "UR ROS2 driver installed successfully"

# --------------------------- #
## DepthAI (Luxonis OAK D Lite)
# --------------------------- #

# Dependencies
sudp apt-get update && apt-get -y upgrade
sudo apt-get install -y \
        libopencv-dev \
        udev 

export DAI_WS=~/dai_ws
mkdir -p $DAI_WS/src
cd $DAI_WS
# RUN git clone https://github.com/luxonis/depthai.git
# WORKDIR $DAI_WS/depthai
# COPY requirements.txt .
# RUN pip install -r requirements.txt
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules
wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
wget https://raw.githubusercontent.com/luxonis/depthai-ros/main/underlay.repos
vcs import src < underlay.repos
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/foxy/setup.bash
colcon build 
source install/setup.bash
echo "depthAI driver installed successfully"

# --------------------------- #
## Cocobots workspace
# --------------------------- #

# Install the environment
export CCBTS_WS=~/cocobots_ws
cd $CCBTS_WS
git clone git@github.com:ccbts/085_ccbts_env.git src/ccbts_webots
echo "Cocobots Webots environment installed successfully"

git clone git@github.com:ccbts/092_ccbts_bringup.git src/ccbts_bringup
echo "Cocobots UR launch files installed successfully"

git clone git@github.com:alexandrosnic/vgc10_ros2_drivers.git src/vgc10_driver
echo "VGC10 drivers installed successfully"

git clone https://github.com/DeepX-inc/executive_smach.git src/executive_smach
echo "ROS2 state machines (smach) installed successfully"

git clone git@github.com:ccbts/093_ccbts_api.git src/ccbts_ros2_api
echo "Cocobots ROS2 API installed successfully"

colcon build
source install/setup.bash
export PYTHONPATH=${PYTHONPATH}:/home/${USER}/cocobots_ws/install/ccbts_environment/lib/python3.8/site-packages

# Source ros2 environment (update bashrc)
echo "source /home/root/dai_ws/install/setup.bash" >> /home/$USER/.bashrc
echo "source /home/root/ur_driver_ws/install/setup.bash" >> /home/$USER/.bashrc
echo "source /home/root/cocobots_ws/install/setup.bash" >> /home/$USER/.bashrc

echo "Dependencies installed"
