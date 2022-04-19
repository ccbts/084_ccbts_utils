#!/bin/bash
set -e 
# setup environment
source $HOME/.bashrc 
# start in home directory 
cd  
exec bash -i -c $@

# Adding all the necessary ros sourcing
echo "" >> ~/.bashrc
echo "## ROS" >> ~/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /home/$USER/cocobots_ws/install/setup.bash" >> ~/.bashrc


# ----------------------- Next entrypoint ---------------
# Uncomment it when more packages will be added in main directory
# If uncomment it, then delete this part from Dockerfile, and run entrypoint from docker-compose, not dockerfile:
# ```
# WORKDIR ${HOME}/cocobots_ws
# RUN /bin/bash -c "source source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"

# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]
# ```

# #!/bin/bash
# # 
# # Script to update-build-run workspace for docker-compose services
# # 
# # Author: Emiliano J. Borghi Orue

# # Check arguments
# if [ "$#" -ne 2 ]; then
#     echo "Illegal number of parameters. Two are expected: ROS package and launchfile."
#     echo "Eg: turtlesim turtlesim multisim.launch.py"
#     exit -1
# fi

# # Arguments
# PACKAGE_NAME=$1
# LAUNCHFILE_NAME=$2

# # Source ROS 2
# . /opt/ros/foxy/setup.bash && \
# # Retrieve new lists of packages
# sudo apt-get update && \
# # Install workspace dependencies (not needed)
# rosdep install --from-path /cocobots_ws/src -yi --rosdistro=foxy && \
# # Build workspace
# colcon build --symlink-install && \
# # Source workspace
# . /cocobots_ws/install/setup.bash && \
# # Execute launchfile (See Emiliano's docker-compose)
# ros2 launch $PACKAGE_NAME $LAUNCHFILE_NAME