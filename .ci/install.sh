#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install"

# get the current commit SHA
SHA=`git rev-parse HEAD`

# get the current package name
PACKAGE_NAME=${PWD##*/}

sudo apt-get -y install git

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core

echo "running the main install.sh"
./installation/install.sh

gitman update

# checkout the SHA
cd ~/uav_core/.gitman/$PACKAGE_NAME
git checkout "$SHA"

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_core
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "install ended"
