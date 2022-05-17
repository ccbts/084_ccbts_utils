#!/bin/bash
#
# Usage: ./setup_project.sh (it requires sudo!)

# Install some dependencies
sudo apt-get update
sudo apt-get install -qq curl dirmngr git software-properties-common sudo wget
# sudo apt-get install python3-rocker

# Install other dependencies too
# Install docker
# curl https://get.docker.com | sh \
#   && sudo systemctl --now enable docker

# ## docker-compose
# sudo curl -L "https://github.com/docker/compose/releases/download/1.28.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
# sudo chmod +x /usr/local/bin/docker-compose

# # Install nvidia-container-toolkit
# distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
#       && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#       && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
#             sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#             sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# sudo apt-get update
# sudo apt-get install -y nvidia-docker2
# sudo systemctl restart docker

###########################
######## ROS2 Foxy ########
###########################

# # Set locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# # Setup Sources
sudo apt update 
sudo apt install gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# # Install ROS2 Foxy
sudo apt-get update 
sudo apt-get install -y ros-foxy-desktop ~nros-foxy-rqt*
source /opt/ros/foxy/setup.bash
echo ". /opt/ros/foxy/setup.bash" >> /home/$USER/.bashrc

# install bootstrap tools
sudo apt-get update 
sudo apt-get install -y 
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ROS2 env variables
export ROS_DISTRO=foxy
export AMENT_PREFIX_PATH=/opt/ros/foxy
export COLCON_PREFIX_PATH=/opt/ros/foxy
# export LD_LIBRARY_PATH=/opt/ros/foxy/lib
export PATH=/opt/ros/foxy/bin:$PATH
export PYTHONPATH=${PYTHONPATH}:/opt/ros/foxy/lib/python3.8/site-packages
export ROS_PYTHON_VERSION=3
export ROS_VERSION=2
export LD_LIBRARY_PATH=/opt/ros/foxy/lib:${LD_LIBRARY_PATH}
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Dependencies for glvnd and X11.
sudo apt-get update 
sudo apt-get install -y \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
  libxtst6

# install rocker
sudo apt-get update 
sudo apt-get install -y \
    python3-rocker

###########################
########   Webots  ########
###########################

wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
sudo apt-get update
sudo apt-get install webots

# Install ROS related webots dependencies
sudo apt-get update 
sudo apt-get install -y \
    ros-foxy-webots-ros2 \
    ros-foxy-webots-ros2-driver \

# rosdep
sudo apt-get update
rosdep init 
rosdep update

# Webots env variables
export WEBOTS_HOME=/usr/local/webots
export PATH=/usr/local/webots:${PATH}
export PYTHONPATH=${PYTHONPATH}:${WEBOTS_HOME}/lib/controller/python38
export LD_LIBRARY_PATH=/opt/ros/foxy/lib:/usr/local/webots/lib/controller:${LD_LIBRARY_PATH}
export QTWEBENGINE_DISABLE_SANDBOX=1
export QT_X11_NO_MITSHM=1

echo "Dependencies were installed"
