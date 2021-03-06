#!/bin/bash
#
# Usage: ./setup_project.sh (it requires sudo!)

# Install some dependencies
brew update;
brew install -qq curl dirmngr git software-properties-common sudo;
# brew install python3-rocker

# Install other dependencies too
# Install docker
brew cask install docker

## docker-compose
curl -L "https://github.com/docker/compose/releases/download/1.28.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

# Install nvidia-container-toolkit
# distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
#       && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#       && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
#             sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#             tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# brew update
# brew install -y nvidia-docker2
# systemctl restart docker

echo "Dependencies were installed"
