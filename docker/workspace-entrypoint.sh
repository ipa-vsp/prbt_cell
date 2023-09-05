#!/bin/bash
# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

sudo apt-get update
rosdep update
export ROS_TRACE_DIR=~/.ros/tracing

sudo groupadd -r admin
sudo usermod -aG admin ${USERNAME}

$@
