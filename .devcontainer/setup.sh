#!/bin/bash

set -e

# Enable permission for webcam
sudo usermod -aG video 467-terp

# Setup ROS Packages
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
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
