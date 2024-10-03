#!/bin/bash

set -e

source /opt/ros/$ROS_DISTRO/setup.bash

# Enable permission for webcam
sudo usermod -aG video $USER

# Enable permission for display
echo $'\nexport XAUTHORITY=$(ls /run/user/$UID/.mutter-*)' >> ~/.bashrc

# Check if the ROS workspace exists
if [ ! -d $ROS_WS ]
then
  printf "Setup failed: ROS workspace directory not found\n"
  exit 1
fi

# Install matplot++ dependencies
sudo apt update && sudo apt install -y \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  gnuplot \
  zlib1g-dev \
  libblas-dev \
  liblapack-dev

# Install matplot++
if [ ! -d matplotplusplus ]
then
  git clone https://github.com/alandefreitas/matplotplusplus.git
fi

cd matplotplusplus

if [ -d build ]
then
  rm -rf build
fi

# Build the library and install it
cmake --preset=system
cmake --build --preset=system
sudo cmake --install build/system

# Move to workspace root directory and clean up matplot++ source files
cd $ROS_WS && rm -rf matplotplusplus
  ros-humble-ur \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-usb-cam \
  ros-humble-aruco-opencv

# git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git src/Universal_Robots_ROS2_Gazebo_Simulation

# Install package dependencies
rosdep update && rosdep install --from-paths src -y --ignore-src

# Build the workspace
colcon build --symlink-install
