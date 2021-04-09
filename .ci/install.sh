#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

# get the path to this script
MY_PATH=`pwd`

echo "Starting install"

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

# link the up-to-date version of this package
rm -rf ~/uav_core/.gitman/$PACKAGE_NAME
ln -s "$MY_PATH" ~/uav_core/.gitman/$PACKAGE_NAME

mkdir -p ~/mrs_workspace/src
cd ~/mrs_workspace/src
ln -s ~/uav_core
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/mrs_workspace

echo "install ended"
