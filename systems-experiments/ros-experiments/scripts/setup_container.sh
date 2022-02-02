#!/bin/bash
set -x

sudo apt-get update
sudo apt-get install -y python3-pip
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make
source devel/setup.bash
pip3 install pandas
