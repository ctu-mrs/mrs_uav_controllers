#!/bin/bash
# author: Robert Penicka
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install preparation" 

openssl aes-256-cbc -K $encrypted_f0fd3ee254e8_key -iv $encrypted_f0fd3ee254e8_iv -in ./.ci/deploy_key_github.enc -out ./.ci/deploy_key_github -d
eval "$(ssh-agent -s)"
chmod 600 ./.ci/deploy_key_github
ssh-add ./.ci/deploy_key_github

sudo apt-get update -qq
sudo apt-mark hold openssh-server

# the "gce-compute-image-packages" package often freezes the installation at some point
# the installation freezes when it tries to manage some systemd services
# this attempts to install the package and stop the problematic service during the process
((sleep 90 && (sudo systemctl stop google-instance-setup.service && echo "gce service stoped" || echo "gce service not stoped")) & (sudo timeout 120s apt-get -y install gce-compute-image-packages)) || echo "\e[1;31mInstallation of gce-compute-image-packages failed\e[0m"

sudo apt -y upgrade --fix-missing

sudo apt-get install dpkg git python-setuptools python3-setuptools python3-pip

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core

echo "running the main install.sh"
./installation/install.sh

gitman update

# get the current commit SHA
cd "$TRAVIS_BUILD_DIR"
SHA=`git rev-parse HEAD`

# get the current package name
PACKAGE_NAME=${PWD##*/}

# checkout the SHA
cd ~/uav_core/.gitman/$PACKAGE_NAME
git checkout "$SHA"

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_core
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "install part ended"
