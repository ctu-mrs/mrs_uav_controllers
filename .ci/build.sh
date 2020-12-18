#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting build"
cd ~/catkin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin build --limit-status-rate 0.2 --summarize
echo "Ended build"
