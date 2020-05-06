#!/bin/bash
# author: Robert Penicka
set -e

echo "Starting test build" 
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin build
echo "Ended test build"
